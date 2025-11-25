#include "stepper.h"
#include "config.h"

#ifndef INVERT_MOTOR_A_DIR
#define INVERT_MOTOR_A_DIR 0
#endif
#ifndef INVERT_MOTOR_B_DIR
#define INVERT_MOTOR_B_DIR 0
#endif

#define STEP1_GPIO_Port GPIOB
#define STEP1_Pin       GPIO_PIN_4
#define STEP2_GPIO_Port GPIOB
#define STEP2_Pin       GPIO_PIN_6

#define DIR1_GPIO_Port  GPIOA
#define DIR1_Pin        GPIO_PIN_3
#define DIR2_GPIO_Port  GPIOC
#define DIR2_Pin        GPIO_PIN_0

#define EN1_GPIO_Port   GPIOC
#define EN1_Pin         GPIO_PIN_3
#define EN2_GPIO_Port   GPIOF
#define EN2_Pin         GPIO_PIN_3

extern TIM_HandleTypeDef htim3; /* Motor A */
extern TIM_HandleTypeDef htim4; /* Motor B */

volatile uint32_t stepCount1 = 0, targetSteps1 = 0;
volatile uint32_t stepCount2 = 0, targetSteps2 = 0;
volatile uint8_t  stepDone1  = 1, stepDone2  = 1;

static void _tim3_ch1_set_af(void) {
    GPIO_InitTypeDef gi = {0};
    gi.Pin       = STEP1_Pin;
    gi.Mode      = GPIO_MODE_AF_PP;
    gi.Pull      = GPIO_NOPULL;
    gi.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gi.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(STEP1_GPIO_Port, &gi);
}
static void _tim3_ch1_set_gpio_output(void) {
    GPIO_InitTypeDef gi = {0};
    gi.Pin   = STEP1_Pin;
    gi.Mode  = GPIO_MODE_OUTPUT_PP;
    gi.Pull  = GPIO_NOPULL;
    gi.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(STEP1_GPIO_Port, &gi);
}

static void _tim4_ch1_set_af(void) {
    GPIO_InitTypeDef gi = {0};
    gi.Pin       = STEP2_Pin;
    gi.Mode      = GPIO_MODE_AF_PP;
    gi.Pull      = GPIO_NOPULL;
    gi.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gi.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(STEP2_GPIO_Port, &gi);
}
static void _tim4_ch1_set_gpio_output(void) {
    GPIO_InitTypeDef gi = {0};
    gi.Pin   = STEP2_Pin;
    gi.Mode  = GPIO_MODE_OUTPUT_PP;
    gi.Pull  = GPIO_NOPULL;
    gi.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(STEP2_GPIO_Port, &gi);
}

void Stepper1_Enable(uint8_t en) {
    HAL_GPIO_WritePin(EN1_GPIO_Port, EN1_Pin, en ? GPIO_PIN_RESET : GPIO_PIN_SET);
}
void Stepper2_Enable(uint8_t en) {
    HAL_GPIO_WritePin(EN2_GPIO_Port, EN2_Pin, en ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

static inline void _set_dir1(uint8_t dir){
    uint8_t d = (dir ^ INVERT_MOTOR_A_DIR) ? 1 : 0;
    HAL_GPIO_WritePin(DIR1_GPIO_Port, DIR1_Pin, d ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
static inline void _set_dir2(uint8_t dir){
    uint8_t d = (dir ^ INVERT_MOTOR_B_DIR) ? 1 : 0;
    HAL_GPIO_WritePin(DIR2_GPIO_Port, DIR2_Pin, d ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static inline void _setup_freq(TIM_HandleTypeDef* htim, uint32_t channel, uint32_t freq_hz) {
    if (freq_hz == 0) freq_hz = 1;
    uint32_t arr = (1000000u / freq_hz);
    if (arr) arr -= 1u;
    if (arr < 1u) arr = 1u;
    __HAL_TIM_SET_AUTORELOAD(htim, arr);
    __HAL_TIM_SET_COMPARE(htim, channel, arr/2u);
}

void Stepper1_Start(uint32_t steps, uint32_t freq, uint8_t dir)
{
    Stepper1_Enable(1);
    _tim3_ch1_set_af();
    _set_dir1(dir);
    stepCount1 = 0; targetSteps1 = steps; stepDone1 = (steps==0);
    _setup_freq(&htim3, TIM_CHANNEL_1, freq);
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_1);
}
void Stepper2_Start(uint32_t steps, uint32_t freq, uint8_t dir)
{
    Stepper2_Enable(1);
    _tim4_ch1_set_af();
    _set_dir2(dir);
    stepCount2 = 0; targetSteps2 = steps; stepDone2 = (steps==0);
    _setup_freq(&htim4, TIM_CHANNEL_1, freq);
    __HAL_TIM_SET_COUNTER(&htim4, 0);
    HAL_TIM_PWM_Start_IT(&htim4, TIM_CHANNEL_1);
}

void Stepper1_Start_no_dis(uint32_t steps, uint32_t freq, uint8_t dir)
{
    _tim3_ch1_set_af();
    _set_dir1(dir);
    stepCount1 = 0; targetSteps1 = steps; stepDone1 = (steps==0);
    _setup_freq(&htim3, TIM_CHANNEL_1, freq);
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_1);
}
void Stepper2_Start_no_dis(uint32_t steps, uint32_t freq, uint8_t dir)
{
    _tim4_ch1_set_af();
    _set_dir2(dir);
    stepCount2 = 0; targetSteps2 = steps; stepDone2 = (steps==0);
    _setup_freq(&htim4, TIM_CHANNEL_1, freq);
    __HAL_TIM_SET_COUNTER(&htim4, 0);
    HAL_TIM_PWM_Start_IT(&htim4, TIM_CHANNEL_1);
}

void Stepper1_HardStop(void)
{
    HAL_TIM_PWM_Stop_IT(&htim3, TIM_CHANNEL_1);
    _tim3_ch1_set_gpio_output();
    HAL_GPIO_WritePin(STEP1_GPIO_Port, STEP1_Pin, GPIO_PIN_RESET);
    Stepper1_Enable(1);
}
void Stepper2_HardStop(void)
{
    HAL_TIM_PWM_Stop_IT(&htim4, TIM_CHANNEL_1);
    _tim4_ch1_set_gpio_output();
    HAL_GPIO_WritePin(STEP2_GPIO_Port, STEP2_Pin, GPIO_PIN_RESET);
    Stepper2_Enable(1);
}
void Stepper_StopAll(void)
{
    Stepper1_HardStop();
    Stepper2_HardStop();
}

void Stepper1_ResumePWM(uint32_t freq)
{
    _tim3_ch1_set_af();
    _setup_freq(&htim3, TIM_CHANNEL_1, freq);
    __HAL_TIM_SET_COUNTER(&htim3, 0);
}
void Stepper2_ResumePWM(uint32_t freq)
{
    _tim4_ch1_set_af();
    _setup_freq(&htim4, TIM_CHANNEL_1, freq);
    __HAL_TIM_SET_COUNTER(&htim4, 0);
}

static void _ramp_down(TIM_HandleTypeDef *htim, uint32_t channel, uint32_t ramp_ms)
{
    if (ramp_ms == 0) return;
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(htim);
    if (arr == 0) arr = 1;
    const int steps = 10;
    uint32_t wait = (ramp_ms + steps - 1) / steps;
    for (int i = 0; i < steps; ++i) {
        uint32_t next = arr + (arr >> 1) + 1;
        if (next > 0xFFFFu) next = 0xFFFFu;
        __HAL_TIM_SET_AUTORELOAD(htim, next);
        __HAL_TIM_SET_COMPARE(htim, channel, next / 2u);
        arr = next;
        if (wait) HAL_Delay(wait);
    }
}

void Stepper1_SoftHold(uint32_t ramp_ms)
{
    _ramp_down(&htim3, TIM_CHANNEL_1, ramp_ms);
    HAL_TIM_PWM_Stop_IT(&htim3, TIM_CHANNEL_1);
    _tim3_ch1_set_gpio_output();
    HAL_GPIO_WritePin(STEP1_GPIO_Port, STEP1_Pin, GPIO_PIN_RESET);
    Stepper1_Enable(1);
}
void Stepper2_SoftHold(uint32_t ramp_ms)
{
    _ramp_down(&htim4, TIM_CHANNEL_1, ramp_ms);
    HAL_TIM_PWM_Stop_IT(&htim4, TIM_CHANNEL_1);
    _tim4_ch1_set_gpio_output();
    HAL_GPIO_WritePin(STEP2_GPIO_Port, STEP2_Pin, GPIO_PIN_RESET);
    Stepper2_Enable(1);
}
void Stepper_SoftHoldBoth(uint32_t ramp_ms)
{
    Stepper1_SoftHold(ramp_ms);
    Stepper2_SoftHold(ramp_ms);
}

void Stepper1_Coast(void)
{
    HAL_TIM_PWM_Stop_IT(&htim3, TIM_CHANNEL_1);
    _tim3_ch1_set_gpio_output();
    HAL_GPIO_WritePin(STEP1_GPIO_Port, STEP1_Pin, GPIO_PIN_RESET);
    Stepper1_Enable(0);
}
void Stepper2_Coast(void)
{
    HAL_TIM_PWM_Stop_IT(&htim4, TIM_CHANNEL_1);
    _tim4_ch1_set_gpio_output();
    HAL_GPIO_WritePin(STEP2_GPIO_Port, STEP2_Pin, GPIO_PIN_RESET);
    Stepper2_Enable(0);
}

void Stepper1_LockStepHigh(void)
{
    HAL_TIM_PWM_Stop_IT(&htim3, TIM_CHANNEL_1);
    _tim3_ch1_set_gpio_output();
    HAL_GPIO_WritePin(STEP1_GPIO_Port, STEP1_Pin, GPIO_PIN_RESET);
    Stepper1_Enable(1);
}
void Stepper2_LockStepHigh(void)
{
    HAL_TIM_PWM_Stop_IT(&htim4, TIM_CHANNEL_1);
    _tim4_ch1_set_gpio_output();
    HAL_GPIO_WritePin(STEP2_GPIO_Port, STEP2_Pin, GPIO_PIN_RESET);
    Stepper2_Enable(1);
}

void Stepper1_IdleKeepAF(void)
{
    HAL_TIM_PWM_Stop_IT(&htim3, TIM_CHANNEL_1);
    Stepper1_Enable(1);
}
void Stepper2_IdleKeepAF(void)
{
    HAL_TIM_PWM_Stop_IT(&htim4, TIM_CHANNEL_1);
    Stepper2_Enable(1);
}
void Stepper_IdleBothKeepAF(void)
{
    Stepper1_IdleKeepAF();
    Stepper2_IdleKeepAF();
}
