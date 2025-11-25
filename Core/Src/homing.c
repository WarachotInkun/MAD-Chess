/*
 * homing.c  — Real homing with EXTI + debounce + jog/backoff
 */

#include "homing.h"
#include "motion.h"
#include "stepper.h"
#include "config.h"
#include "gpio.h"
#include "cli.h"               // สำหรับ io_puts
#include "stm32f7xx_hal.h"

/* -------- ค่าปลอดภัยเผื่อยังไม่ประกาศใน config.h -------- */
#ifndef HOMING_DEBOUNCE_MS
#define HOMING_DEBOUNCE_MS   10u
#endif
#ifndef HOMING_FREQ
#define HOMING_FREQ          800u
#endif
#ifndef HOMING_JOG_STEPS
#define HOMING_JOG_STEPS     200u
#endif
#ifndef HOMING_BACKOFF_X_STEPS
#define HOMING_BACKOFF_X_STEPS  400u
#endif
#ifndef HOMING_BACKOFF_Y_STEPS
#define HOMING_BACKOFF_Y_STEPS  400u
#endif
#ifndef HOMING_BACKOFF_FREQ
#define HOMING_BACKOFF_FREQ  400u
#endif
#ifndef HOMING_NUDGE_XP_STEPS
#define HOMING_NUDGE_XP_STEPS   200u
#endif
/* X_MIN_PIN / Y_MIN_PIN ต้องเป็น GPIO_PIN_x ที่แมปกับ EXTI */
#ifndef X_MIN_PIN
#warning "X_MIN_PIN not defined in config.h – defaulting to GPIO_PIN_1"
#define X_MIN_PIN GPIO_PIN_1
#endif
#ifndef Y_MIN_PIN
#warning "Y_MIN_PIN not defined in config.h – defaulting to GPIO_PIN_2"
#define Y_MIN_PIN GPIO_PIN_2
#endif
/* ----------------------------------------------------------- */

volatile uint8_t hit_x_min = 0, hit_y_min = 0;
volatile uint8_t homing_armed = 0;
static uint32_t last_irq_x = 0, last_irq_y = 0;

void homing_on_exti(uint16_t pin)
{
    if (!homing_armed) return;

    uint32_t now = HAL_GetTick();

    if (pin == X_MIN_PIN) {
        if (now - last_irq_x >= HOMING_DEBOUNCE_MS) {
            hit_x_min = 1;
            last_irq_x = now;
        }
        io_puts("X\r\n");
    } else if (pin == Y_MIN_PIN) {
        if (now - last_irq_y >= HOMING_DEBOUNCE_MS) {
            hit_y_min = 1;
            last_irq_y = now;
        }
        io_puts("Y\r\n");
    }
}

static inline void homing_clear_pending(void)
{
    __HAL_GPIO_EXTI_CLEAR_IT(X_MIN_PIN);
    __HAL_GPIO_EXTI_CLEAR_IT(Y_MIN_PIN);
}

void homing_arm_exti(void)
{
    hit_x_min = hit_y_min = 0;
    homing_armed = 1;
    homing_clear_pending();
    /* ถ้าคุณ disable NVIC ไว้ ให้ enable ที่นี่:
       HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
       HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
    */
}

void homing_disarm_exti(void)
{
    homing_armed = 0;
    hit_x_min = hit_y_min = 0;
    homing_clear_pending();
    /* ถ้า enable NVIC เองไว้ ให้ disable ที่นี่:
       HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
       HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
    */
}

void home_reset_sequence(void)
{
    /* เปิดโหมด homing เฉพาะช่วงนี้ */
    homing_arm_exti();

    hit_x_min = hit_y_min = 0;

    /* เตรียม PWM ทางเดิน */
    Stepper1_ResumePWM(HOMING_FREQ);
    Stepper2_ResumePWM(HOMING_FREQ);

    /* กรณีชิด X_MIN ตั้งแต่เริ่ม: ดัน X+ ออกเล็กน้อย */
    /* หมายเหตุ: เงื่อนไข cur_x==0 เป็นตรรกะ—ถ้าจริงๆ หัวชิดสวิตช์อยู่ ก็จะเจอ interrupt
       ทันทีอยู่ดี; หากไม่แน่ใจ ให้เพิ่มการอ่านสถานะสวิตช์ฮาร์ดแวร์ก่อนค่อย NUDGE */
    if (cur_x == 0) {
    	corexy_move_delta_steps_no_dis(+HOMING_NUDGE_XP_STEPS, 0, HOMING_FREQ, 30000);
    }

    /* หา Y_MIN: jog ลงเป็นช่วง ๆ จนชนสวิตช์ */
    while (!hit_y_min) {
        if (corexy_move_delta_steps_no_dis(0, -(int32_t)HOMING_JOG_STEPS, HOMING_FREQ, 30000) != 0) {
            break;
        }
    }

    /* หา X_MIN: jog ซ้ายเป็นช่วง ๆ จนชนสวิตช์ */
    while (!hit_x_min) {
        if (corexy_move_delta_steps_no_dis(-(int32_t)HOMING_JOG_STEPS, 0, HOMING_FREQ, 30000) != 0) {
            break;
        }
    }

    /* เมื่อชนแล้ว สวิตช์มักยังค้าง → backoff คลายออกเล็กน้อย */
    HAL_Delay(160);
    corexy_move_delta_steps_no_dis(0, +(int32_t)HOMING_BACKOFF_Y_STEPS, HOMING_BACKOFF_FREQ, 30000);
    HAL_Delay(160);
    corexy_move_delta_steps_no_dis(-(int32_t)HOMING_BACKOFF_X_STEPS, 0, HOMING_BACKOFF_FREQ, 30000);

    /* ตั้งตำแหน่งเป็น 0,0 */
    cur_x = 0; cur_y = 0;

    /* ปิดโหมด homing */
    homing_disarm_exti();

    /* คงแรงยึดนุ่ม ๆ */
    Stepper_SoftHoldBoth(10);
}
