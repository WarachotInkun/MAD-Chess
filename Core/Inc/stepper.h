#pragma once
#include <stdint.h>
#include "tim.h"
#include "gpio.h"

/* เคาน์เตอร์สเต็ป (ใช้ใน IRQ) */
extern volatile uint32_t stepCount1, targetSteps1;
extern volatile uint32_t stepCount2, targetSteps2;
extern volatile uint8_t  stepDone1, stepDone2;

/* ควบคุมพื้นฐาน */
void Stepper1_Enable(uint8_t en);   /* en: 1=enable (EN=LOW), 0=disable (EN=HIGH) */
void Stepper2_Enable(uint8_t en);

void Stepper1_Start(uint32_t steps, uint32_t freq, uint8_t dir);
void Stepper2_Start(uint32_t steps, uint32_t freq, uint8_t dir);

void Stepper1_Start_no_dis(uint32_t steps, uint32_t freq, uint8_t dir);
void Stepper2_Start_no_dis(uint32_t steps, uint32_t freq, uint8_t dir);

void Stepper1_HardStop(void);
void Stepper2_HardStop(void);
void Stepper_StopAll(void);

/* ตั้ง pin STEP กลับเป็น AF ของ TIM (สำหรับกลับมารัน PWM ต่อ) */
void Stepper1_ResumePWM(uint32_t freq);
void Stepper2_ResumePWM(uint32_t freq);

/* “หยุดนุ่ม” แล้วคงแรงยึด (EN=LOW) */
void Stepper1_SoftHold(uint32_t ramp_ms);
void Stepper2_SoftHold(uint32_t ramp_ms);
void Stepper_SoftHoldBoth(uint32_t ramp_ms);

/* ปล่อยลอย (EN=HIGH, ไม่มีแรงยึด) */
void Stepper1_Coast(void);
void Stepper2_Coast(void);

void Stepper1_LockStepHigh(void);
void Stepper2_LockStepHigh(void);

void Stepper1_IdleKeepAF(void);
void Stepper2_IdleKeepAF(void);
void Stepper_IdleBothKeepAF(void);
