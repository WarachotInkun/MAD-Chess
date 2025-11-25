#pragma once
#include <stdint.h>

void home_reset_sequence(void);
void homing_arm_exti(void);
void homing_disarm_exti(void);

/* NEW: เรียกจาก HAL_GPIO_EXTI_Callback */
void homing_on_exti(uint16_t pin);
