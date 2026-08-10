/*
 * control.c
 * 车身控制层：速度环 + 转向环 + PWM 合成 + 反转逻辑 + boost_update + calc_mid_row
 * 每 10ms 由 CPU0 主循环调用 control_loop()
 */

#include "zf_common_headfile.h"
#include "shared_data.h"
#include "motor.h"
#include "control.h"

#pragma section all "cpu0_dsram"

/* ---------- PID 实例 ---------- */
PID pid_speed_left, pid_speed_right;
PID pid_steer;

/* ---------- 寻线结果共享数组（CPU1 写入，CPU0 读取） ---------- */
uint8 left_bound[TRACK_IMAGE_H];
uint8 right_bound[TRACK_IMAGE_H];
uint8 mid_line[TRACK_IMAGE_H];

/* ---------- 控制参数 ---------- */
uint8 base_pwm_val = 5;          // 正转基础占空比(%)，动态维持值
uint8 target_speed_pulse = 200;  // 速度环目标（编码器脉冲/10ms）
float err_scale = 1.3f;          // 中线偏差 → 转向目标系数
volatile int16 steer_error = 0;  // 当前转向误差（平滑后）

volatile int16 rev_cur      = 0;  // 当前反转占空比(%)，每 10ms 渐进平滑（CPU1 显示用）

static int16 last_mid_error = 0;     // 上一帧转向目标（无有效行时保持）
static uint8 rev_active_flag = 0;    /* 反转进入/退出门槛标志 */
static uint8 straight_cnt = 0;       // 直线连续确认帧数

#pragma section all restore

/* ---------- PID 初始化（各环参数） ---------- */
void control_init(void)
{
    PID_Init(&pid_speed_left,  0.08, 0.07, 0.1, 50, 15);
    PID_Init(&pid_speed_right, 0.1,  0.06, 0.1, 50, 15);
    PID_Init(&pid_steer, 1.18, 0.0, 0.70, 0, 38);
}

void clear_integrals(void) {
    pid_speed_left.integral = 0;
    pid_speed_right.integral = 0;
    pid_steer.integral = 0;
}

/**
 * @brief 直线加速 / 弯道减速状态机（每 10ms 调用一次）
 * 当前被主循环注释（提速待重做），保留函数供以后启用。
 */
void boost_update(void)
{
    uint8 row;
    uint8 all_straight = 1;

    if (cross_state || is_learning())
    {
        speed_boost = 0;
        straight_cnt = 0;
        return;
    }

    if (mid_line[curve_row] != INVALID_BORDER)
    {
        int16 dev = (int16)mid_line[curve_row] - LINE_CENTER;
        if (dev < 0) dev = -dev;
        if (dev > (int16)curve_thresh)
        {
            speed_boost -= (int16)boost_down_step;
            if (speed_boost < 0) speed_boost = 0;
            straight_cnt = 0;
            return;
        }
    }

    for (row = STR_ROW_LO; row <= STR_ROW_HI; row++)
    {
        if (mid_line[row] == INVALID_BORDER) { all_straight = 0; break; }
        int16 d = (int16)mid_line[row] - LINE_CENTER;
        if (d < 0) d = -d;
        if (d > (int16)straight_tol) { all_straight = 0; break; }
    }

    if (all_straight)
    {
        if (straight_cnt < 255) straight_cnt++;
        if (straight_cnt >= STRAIGHT_REQ_FRAMES)
        {
            speed_boost += (int16)boost_up_step;
            if (speed_boost > (int16)boost_max) speed_boost = boost_max;
        }
    }
    else
    {
        straight_cnt = 0;
    }
}

/**
 * @brief 在 90~55 行范围内，比较"当前行"与"当前行-5行"的误差是否超阈值，
 *        选择当前误差计算行。全未超阈值时回落到 ERR_ROW_LO 行。
 */
int16 calc_mid_row(const uint8 mid_line[], uint16 thresh)
{
    int16 r;
    for (r = ERR_ROW_HI; r >= ERR_ROW_LO + ERR_ROW_STEP; r--)
    {
        uint8 m_r = mid_line[r];
        uint8 m_r5 = mid_line[r - ERR_ROW_STEP];
        if (m_r  >= TRACK_IMAGE_W) continue;   /* 本行无效 */
        if (m_r5 >= TRACK_IMAGE_W) continue;   /* r-5 行无效 */
        {
            int16 d = (int16)m_r - (int16)m_r5;
            if (d < 0) d = -d;
            if (d > (int16)thresh) return r;   /* 误差超阈值，err 用当前行 */
        }
    }
    return ERR_ROW_LO;   /* 全部未超阈值，选 55 行 */
}

/**
 * @brief 车身控制主循环（每 10ms 调用一次）
 *  1) 反转逻辑：|steer_error| > rev_thresh 触发，rev_cur 渐进到 rev_target
 *  2) 速度环目标分裂：正常 = +140，反转轮 = -rev_cur
 *  3) 转向误差：黑区用 calc_mid_row 选行，否则固定 60 行；十字路口强直
 *  4) 转向 PID 得 steer_delta，合成左右 PWM 并输出
 */
void control_loop(void)
{
    int16 rev_target = 0;

    /* -- 1. 反转触发：|steer_error| 超 rev_thresh，学习期/十字路口不反转 -- */
    if (!cross_state && !is_learning())
    {
        int16 ae = steer_error;
        if (ae < 0) ae = -ae;
        if (ae > rev_thresh)
        {
            rev_target = (int16)(ae - rev_thresh);
            if (rev_target < REV_MIN_BASE) rev_target = REV_MIN_BASE;
            if (rev_target > rev_max) rev_target = rev_max;
        }
    }
    if (rev_cur < rev_target) { rev_cur += REV_STEP; if (rev_cur > rev_target) rev_cur = rev_target; }
    else if (rev_cur > rev_target) { rev_cur -= REV_STEP; if (rev_cur < rev_target) rev_cur = rev_target; }

    /* 刚进入/退出反转时清速度环积分，避免反转轮受到旧偏差拉扯 */
    if (rev_cur > 0 && !rev_active_flag)
    {
        pid_speed_left.integral  = 0;
        pid_speed_right.integral = 0;
        rev_active_flag = 1;
    }
    else if (rev_cur == 0 && rev_active_flag)
    {
        pid_speed_left.integral  = 0;
        pid_speed_right.integral = 0;
        rev_active_flag = 0;
    }

    /* -- 2. 速度环目标分裂：正常=+140，内侧轮=反转 -- */
    int speed_target = (int)target_speed_pulse;
    int left_tgt  = speed_target;
    int right_tgt = speed_target;
    if (rev_cur > 0)
    {
        if      (steer_error > 0) left_tgt  = -(int)rev_cur;   /* 线在左(内侧轮为左)反转 */
        else if (steer_error < 0) right_tgt = -(int)rev_cur;   /* 线在右(内侧轮为右)反转 */
    }
    int left_corr  = (int)PID_Calc(&pid_speed_left,  (float)left_tgt,  (float)left_real_speed);
    int right_corr = (int)PID_Calc(&pid_speed_right, (float)right_tgt, (float)right_real_speed);

    /* -- 3. 转向误差计算：固定用 calc_mid_row 自适应选行 -- */
    int16 steer_target;
    {
        int16 sel = calc_mid_row(mid_line, (uint16)err_thresh);
        if (sel >= ERR_ROW_LO && sel <= ERR_ROW_HI && mid_line[sel] < TRACK_IMAGE_W)
        {
            steer_target = (int16)((LINE_CENTER - (int16)mid_line[sel]) * err_scale);
            last_mid_error = steer_target;
        }
        else
        {
            steer_target = last_mid_error;
        }
    }

    // 十字路口强制直行
    if (cross_state) {
        steer_error = 0;
        last_mid_error = 0;
    } else {
        // err 平滑：每周期最多变化 ERR_MAX_STEP
        int16 delta = steer_target - steer_error;
        if (delta >  ERR_MAX_STEP) delta =  ERR_MAX_STEP;
        if (delta < -ERR_MAX_STEP) delta = -ERR_MAX_STEP;
        steer_error += delta;
    }

    int steer_delta = (int)PID_Calc(&pid_steer, 0.0, (float)steer_error);

    /* -- 4. PWM 合成：反转轮开环 PWM=-rev_cur（跳过速度环与 steer_delta），非反转轮差速+钳0 -- */
    int left_pwm, right_pwm;
    if (left_tgt  < 0) left_pwm  = -(int)rev_cur;             /* 左轮在反转，开环反转 */
    else               left_pwm  = base_pwm_val + left_corr  + steer_delta;
    if (right_tgt < 0) right_pwm = -(int)rev_cur;             /* 右轮在反转，开环反转 */
    else               right_pwm = base_pwm_val + right_corr - steer_delta;
    /* 非反转轮钳 ≥0：避免差速打穿变成内侧轮停转 */
    if (left_tgt  >= 0 && left_pwm  < 0) left_pwm  = 0;
    if (right_tgt >= 0 && right_pwm < 0) right_pwm = 0;
    motor_set_pwm(left_pwm, right_pwm);
}
