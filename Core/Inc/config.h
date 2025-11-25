/*
 * config.h
 *
 *  Created on: Sep 29, 2025
 *      Author: kim00
 */

#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_



#endif /* INC_CONFIG_H_ */

#pragma once

#define GRID_SIZE_X              8
#define GRID_SIZE_Y              8
#define STEPS_PER_CELL_X         238
#define STEPS_PER_CELL_Y         238
#define DEFAULT_FREQ             800
#define INVERT_X                 0
#define INVERT_Y                 0

#define HOMING_NUDGE_XP_STEPS    235
#define HOMING_JOG_STEPS         20
#define HOMING_FREQ              800

#define SPEED_KNIGHT_HZ          400
#define SPEED_FAST_NO_MAG_HZ    1200

#define X_MIN_PIN                GPIO_PIN_5
#define Y_MIN_PIN                GPIO_PIN_10

// ทิศทางรายมอเตอร์ (0 = ปกติ, 1 = กลับทิศ)
#define INVERT_MOTOR_A_DIR   0
#define INVERT_MOTOR_B_DIR   0

#define HOMING_BACKOFF_X_STEPS   (2 * HOMING_JOG_STEPS)
#define HOMING_BACKOFF_Y_STEPS   (5 * HOMING_JOG_STEPS)
#define HOMING_BACKOFF_FREQ   500
