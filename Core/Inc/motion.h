#pragma once
#ifndef INC_MOTION_H_
#define INC_MOTION_H_

#include <stdint.h>

extern int16_t cur_x;
extern int16_t cur_y;

typedef enum {
    MOTION_OK            =  0,
    MOTION_ERR           = -1,
    MOTION_ERR_OOB       = -2,
    MOTION_ERR_SAME_POS  = -3,
    MOTION_ERR_PATTERN   = -4
} MotionStatus;

int corexy_move_delta_steps(int32_t dx_steps,
                            int32_t dy_steps,
                            uint32_t freq,
                            uint32_t timeout_ms);

/* NEW: แบบ no_dis – ใช้ Start_no_dis เพื่อไม่ยุ่งกับ EN (เหมาะกับ homing ที่ lock/hold ไว้) */
int corexy_move_delta_steps_no_dis(int32_t dx_steps,
                                   int32_t dy_steps,
                                   uint32_t freq,
                                   uint32_t timeout_ms);

int corexy_move_to_cell(uint8_t x, uint8_t y, uint32_t freq);
int corexy_knight_to_cell(uint8_t target_x, uint8_t target_y, uint32_t freq);
int motion_castle_left(void);
int motion_castle_right(void);
int motion_capture_to_row8(uint8_t x, uint8_t y);

#endif /* INC_MOTION_H_ */
