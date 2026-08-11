#ifndef __SHARED_DATA_H__
#define __SHARED_DATA_H__

#include "zf_common_headfile.h"
#include "xunxian.h"
#include "pid.h"

/* 直道加速判定区间行号（CPU0 boost_update 与 CPU1 画线共用） */
#define STR_ROW_LO   25
#define STR_ROW_HI   30

extern PID pid_speed_left, pid_speed_right;
extern PID pid_steer;

extern uint8 left_bound[TRACK_IMAGE_H];
extern uint8 right_bound[TRACK_IMAGE_H];
extern uint8 mid_line[TRACK_IMAGE_H];

extern volatile int16 left_real_speed;
extern volatile int16 right_real_speed;

extern uint8 base_pwm_val;
extern uint16 target_speed_pulse;   // uint16：直线 Speed+boost 峰值可超 255
extern uint8 heibaival;
extern float err_scale;
extern volatile int16 steer_error;
extern int16 err_thresh;   // calc_mid_row 逐行误差判定阈值（可由按键调整）

extern const char* param_names[];
extern uint8 cur_param_index;

extern volatile uint8 cross_state;
extern uint8 cross_row_count;

/* 直线加速 / 弯道减速（speed_boost 加到速度环目标） */
extern uint16 speed_boost;        // 当前速度增量（0 ~ boost_max）
extern uint16 boost_max;          // 速度增量上限（按键可调）
extern uint16 boost_up_step;      // 直道加速步长（按键可调）
extern uint16 boost_down_step;    // 弯道减速步长（按键可调）
extern uint16 curve_thresh;       // 弯道判定阈值：|mid_line[curve_row]-79| 超过则判弯道（按键可调）
extern uint16 curve_row;          // 弯道判定行（按键可调）
extern uint16 straight_tol;       // 直道稳定容差（相对 79 的允许偏差）（按键可调）

/* 急弯内侧轮反转（差速回正，仅在转向打满时启用） */
extern volatile int16 rev_thresh;  // 反转启用阈值：|steer_error| 超过才反转
extern volatile int16 rev_max;     // 反转量上限
extern volatile int16 rev_cur;     // 当前反转量（每 10ms 渐进逼近目标，CPU1 可显示）

#endif