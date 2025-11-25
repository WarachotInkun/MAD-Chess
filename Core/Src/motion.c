#include "motion.h"
#include "config.h"
#include "stepper.h"
#include "stm32f7xx_hal.h"
#include "gpio.h"
#include "homing.h"
#include <string.h>

int16_t cur_x = 0;
int16_t cur_y = 0;

extern TIM_HandleTypeDef htim3; /* A */
extern TIM_HandleTypeDef htim4; /* B */
extern volatile uint8_t stepDone1, stepDone2;

#ifndef MOTION_DEFAULT_TIMEOUT_MS
#define MOTION_DEFAULT_TIMEOUT_MS 60000u
#endif
#ifndef MAG_GPIO_Port
#define MAG_GPIO_Port GPIOG
#endif
#ifndef MAG_Pin
#define MAG_Pin       GPIO_PIN_0
#endif

static inline uint32_t uabs32(int32_t v){ return (v < 0) ? (uint32_t)(-v) : (uint32_t)v; }
static inline void magnet_on(void)  { HAL_GPIO_WritePin(MAG_GPIO_Port, MAG_Pin, GPIO_PIN_SET);  HAL_Delay(5); }
static inline void magnet_off(void) { HAL_GPIO_WritePin(MAG_GPIO_Port, MAG_Pin, GPIO_PIN_RESET); HAL_Delay(5); }

static inline int move_rel_steps(int32_t dx_steps, int32_t dy_steps, uint32_t freq){
    if (INVERT_X) dx_steps = -dx_steps;
    if (INVERT_Y) dy_steps = -dy_steps;
    return corexy_move_delta_steps(dx_steps, dy_steps, freq, MOTION_DEFAULT_TIMEOUT_MS);
}

int corexy_move_delta_steps(int32_t dx_steps,
                            int32_t dy_steps,
                            uint32_t freq,
                            uint32_t timeout_ms)
{
    int32_t A = dx_steps + dy_steps; /* TIM3 */
    int32_t B = dx_steps - dy_steps; /* TIM4 */
    uint32_t a = uabs32(A), b = uabs32(B);

    stepDone1 = (a == 0);
    stepDone2 = (b == 0);

    if (a) Stepper1_Start(a, freq, (A >= 0));
    if (b) Stepper2_Start(b, freq, (B >= 0));

    uint32_t t0 = HAL_GetTick();
    if (timeout_ms == 0) timeout_ms = MOTION_DEFAULT_TIMEOUT_MS;

    while (!(stepDone1 && stepDone2)) {
        if ((HAL_GetTick() - t0) > timeout_ms) {
            Stepper_StopAll();
            return -1;
        }
    }
    /* STABLE finish: force STEP pins to GPIO LOW + hold torque */
    Stepper_SoftHoldBoth(0);
    return 0;
}

/* lock axis using SoftHold(0) (drive STEP LOW), run the other with Start_no_dis, finish with SoftHold(0) */
static int diag_single_locked(int8_t xsign, int8_t ysign, uint32_t s, uint32_t freq)
{
    if (s == 0) return 0;

    if (xsign == ysign) {
        /* x=y → A-only, lock B */
        Stepper2_SoftHold(0);                    /* lock B with STEP=LOW */
        stepDone1 = 0; stepDone2 = 1;
        Stepper1_ResumePWM(freq);
        Stepper1_Start_no_dis(2u*s, freq, (xsign > 0));

        uint32_t t0 = HAL_GetTick();
        while (!stepDone1) {
            if ((HAL_GetTick() - t0) > MOTION_DEFAULT_TIMEOUT_MS) { Stepper_StopAll(); return -1; }
        }
        Stepper1_SoftHold(0);                    /* finish A with STEP=LOW */
    } else {
        /* x=-y → B-only, lock A */
        Stepper1_SoftHold(0);                    /* lock A with STEP=LOW */
        stepDone1 = 1; stepDone2 = 0;
        Stepper2_ResumePWM(freq);
        Stepper2_Start_no_dis(2u*s, freq, (xsign > 0));

        uint32_t t0 = HAL_GetTick();
        while (!stepDone2) {
            if ((HAL_GetTick() - t0) > MOTION_DEFAULT_TIMEOUT_MS) { Stepper_StopAll(); return -1; }
        }
        Stepper2_SoftHold(0);                    /* finish B with STEP=LOW */
    }
    return 0;
}

int corexy_knight_to_cell(uint8_t target_x, uint8_t target_y, uint32_t freq)
{
    if (target_x >= GRID_SIZE_X || target_y >= GRID_SIZE_Y) return MOTION_ERR_OOB;
    if (target_x == (uint8_t)cur_x && target_y == (uint8_t)cur_y) return MOTION_ERR_SAME_POS;
    if (freq == 0) freq = DEFAULT_FREQ;

    int32_t cx = (int32_t)target_x - cur_x;
    int32_t cy = (int32_t)target_y - cur_y;
    int32_t acx = (cx<0)?-cx:cx, acy = (cy<0)?-cy:cy;
    if (!((acx==1 && acy==2) || (acx==2 && acy==1))) return MOTION_ERR_PATTERN;

    int32_t DX = cx * (int32_t)STEPS_PER_CELL_X;
    int32_t DY = cy * (int32_t)STEPS_PER_CELL_Y;
    if (INVERT_X) DX = -DX;
    if (INVERT_Y) DY = -DY;

    int8_t xsign = (DX >= 0)? +1 : -1;
    int8_t ysign = (DY >= 0)? +1 : -1;

    uint32_t s = (acx==1 && acy==2) ? (uint32_t)(STEPS_PER_CELL_X/2u)
                                    : (uint32_t)(STEPS_PER_CELL_Y/2u);
    if (s==0) s=1;

    if (diag_single_locked(xsign, ysign, s, freq) != 0) return MOTION_ERR;

    if (acx==1 && acy==2) {
        int32_t dy_mid = DY - (int32_t)(2u*s)*(int32_t)ysign;
        if (dy_mid && corexy_move_delta_steps(0, dy_mid, freq, MOTION_DEFAULT_TIMEOUT_MS)!=0) return MOTION_ERR;
    } else {
        int32_t dx_mid = DX - (int32_t)(2u*s)*(int32_t)xsign;
        if (dx_mid && corexy_move_delta_steps(dx_mid, 0, freq, MOTION_DEFAULT_TIMEOUT_MS)!=0) return MOTION_ERR;
    }

    if (diag_single_locked(xsign, ysign, s, freq) != 0) return MOTION_ERR;

    cur_x = target_x; cur_y = target_y;
    return MOTION_OK;
}

int corexy_move_to_cell(uint8_t x, uint8_t y, uint32_t freq)
{
    if (x >= GRID_SIZE_X || y >= GRID_SIZE_Y) return MOTION_ERR_OOB;
    if (x == (uint8_t)cur_x && y == (uint8_t)cur_y) return MOTION_ERR_SAME_POS;
    if (freq == 0) freq = DEFAULT_FREQ;

    int32_t cx = (int32_t)x - cur_x;
    int32_t cy = (int32_t)y - cur_y;

    int32_t acx = (cx<0)?-cx:cx, acy = (cy<0)?-cy:cy;
    if ((acx==1 && acy==2) || (acx==2 && acy==1)) {
        return corexy_knight_to_cell(x, y, freq);
    }

    int32_t dx = cx * (int32_t)STEPS_PER_CELL_X;
    int32_t dy = cy * (int32_t)STEPS_PER_CELL_Y;
    if (INVERT_X) dx = -dx;
    if (INVERT_Y) dy = -dy;

    if (dx==0 && dy==0) return MOTION_OK;

    if (cx!=0 && cy!=0 && (cx==cy || cx==-cy)) {
        uint32_t s = (uabs32(dx) < uabs32(dy)) ? uabs32(dx) : uabs32(dy);
        if (s > 0) {
            int8_t xsign = (dx>=0)? +1 : -1;
            int8_t ysign = (dy>=0)? +1 : -1;

            if (diag_single_locked(xsign, ysign, s, freq) != 0) return MOTION_ERR;

            int32_t sdx = (dx>=0)? +1 : -1;
            if (cx == cy) {
                dx -= (int32_t)s * sdx;
                dy -= (int32_t)s * ((dy>=0)? +1 : -1);
            } else {
                dx -= (int32_t)s * sdx;
                dy += (int32_t)s * sdx;
            }

            if (dx==0 && dy==0) {
                cur_x = x; cur_y = y;
                return MOTION_OK;
            }
        }
    }

    if (dx && corexy_move_delta_steps(dx, 0, freq, MOTION_DEFAULT_TIMEOUT_MS)!=0) return MOTION_ERR;
    if (dy && corexy_move_delta_steps(0, dy, freq, MOTION_DEFAULT_TIMEOUT_MS)!=0) return MOTION_ERR;

    cur_x = x; cur_y = y;
    return MOTION_OK;
}


int motion_castle_left(void)
{
    const uint32_t f  = DEFAULT_FREQ;
    const uint32_t hx = (uint32_t)STEPS_PER_CELL_X;
    const uint32_t hy = (uint32_t)STEPS_PER_CELL_Y;
    const uint32_t hx2 = hx/2u;

    if (corexy_move_to_cell(7,3,f)!=0) return -1;
    HAL_Delay(100);
    magnet_on();
    HAL_Delay(100);
    if (corexy_move_to_cell(7,1,f)!=0){ magnet_off(); return -1; }
    HAL_Delay(100);
    magnet_off();
    HAL_Delay(100);
    if (corexy_move_to_cell(7,0,f)!=0) return -1;
    HAL_Delay(100);
    magnet_on();
    HAL_Delay(100);
    if (move_rel_steps(-(int32_t)hx2, 0, f)!=0){ magnet_off(); return -1; }
    HAL_Delay(100);
    if (move_rel_steps(0, (int32_t)(2u*hy), f)!=0){ magnet_off(); return -1; }
    HAL_Delay(100);
    if (move_rel_steps( (int32_t)hx2, 0, f)!=0){ magnet_off(); return -1; }
    HAL_Delay(100);
    magnet_off();
    HAL_Delay(100);
    cur_x = 7; cur_y = 2;
    return 0;
}

int motion_castle_right(void)
{
    const uint32_t f  = DEFAULT_FREQ;
    const uint32_t hx = (uint32_t)STEPS_PER_CELL_X;
    const uint32_t hy = (uint32_t)STEPS_PER_CELL_Y;
    const uint32_t hx2 = hx/2u;

    if (corexy_move_to_cell(7,3,f)!=0) return -1;
    HAL_Delay(100);
    magnet_on();
    HAL_Delay(100);
    if (corexy_move_to_cell(7,5,f)!=0){ magnet_off(); return -1; }
    HAL_Delay(100);
    magnet_off();
    HAL_Delay(100);
    if (corexy_move_to_cell(7,7,f)!=0) return -1;
    HAL_Delay(100);
    magnet_on();
    HAL_Delay(100);
    if (move_rel_steps(-(int32_t)hx2, 0, f)!=0){ magnet_off(); return -1; }
    HAL_Delay(100);
    if (move_rel_steps(0, -(int32_t)(3u*hy), f)!=0){ magnet_off(); return -1; }
    HAL_Delay(100);
    if (move_rel_steps( (int32_t)hx2, 0, f)!=0){ magnet_off(); return -1; }
    HAL_Delay(100);
    magnet_off();
    HAL_Delay(100);
    cur_x = 7; cur_y = 4;
    return 0;
}

int motion_capture_to_row8(uint8_t x, uint8_t y)
{
	uartPrint_raw("\r\n1");
    if (x >= GRID_SIZE_X || y >= GRID_SIZE_Y){uartPrint_raw("\r\n2");return -1;}

    const uint32_t f  = DEFAULT_FREQ;
    const uint32_t hx = (uint32_t)STEPS_PER_CELL_X;
    const uint32_t hy = (uint32_t)STEPS_PER_CELL_Y;
    const uint32_t hx2 = hx / 2u;

    if (corexy_move_to_cell(x, y, f) != 0) {uartPrint_raw("\r\n3");}

    magnet_on();
    uartPrint_raw("\r\n4");
    if (x >= 4) {
        if (move_rel_steps(-(int32_t)hx2, 0, f) != 0) { uartPrint_raw("\r\n5");magnet_off(); return -1; }
    } else {
        if (move_rel_steps( (int32_t)hx2, 0, f) != 0) { uartPrint_raw("\r\n6");magnet_off(); return -1; }
    }

    int32_t dy_cells = (int32_t)7 - (int32_t)y;
    if (dy_cells != 0) {
    	uartPrint_raw("\r\n7");
        if (move_rel_steps(0, dy_cells * (int32_t)hy + hx2, f) != 0) { uartPrint_raw("\r\n8");magnet_off(); return -1; }
    }
    uartPrint_raw("\r\n9");
    magnet_off();
    home_reset_sequence();
    return 0;
}

int corexy_move_delta_steps_no_dis(int32_t dx_steps,
                                   int32_t dy_steps,
                                   uint32_t freq,
                                   uint32_t timeout_ms)
{
    int32_t A = dx_steps + dy_steps; /* TIM3 */
    int32_t B = dx_steps - dy_steps; /* TIM4 */
    uint32_t a = (A >= 0) ? (uint32_t)A : (uint32_t)(-A);
    uint32_t b = (B >= 0) ? (uint32_t)B : (uint32_t)(-B);

    stepDone1 = (a == 0);
    stepDone2 = (b == 0);

    if (a) { Stepper1_ResumePWM(freq); Stepper1_Start_no_dis(a, freq, (A >= 0)); }
    if (b) { Stepper2_ResumePWM(freq); Stepper2_Start_no_dis(b, freq, (B >= 0)); }

    uint32_t t0 = HAL_GetTick();
    if (timeout_ms == 0) timeout_ms = MOTION_DEFAULT_TIMEOUT_MS;

    while (!(stepDone1 && stepDone2)) {
        if ((HAL_GetTick() - t0) > timeout_ms) {
            Stepper_StopAll();
            return -1;
        }
    }
    /* STABLE finish */
    Stepper_SoftHoldBoth(0);
    return 0;
}
