/*
 * param.c
 * 上位机可调参数：索引表 + 变量定义 + 调参函数
 * 变量定义放本文件，extern 声明在 shared_data.h（约定：extern 只在 shared_data.h 声明）
 */

#include "zf_common_headfile.h"
#include "shared_data.h"
#include "param.h"

#pragma section all "cpu0_dsram"

/* 图像二值化阈值（CPU1 binary_image 使用） */
uint8 heibaival = 205;

/* calc_mid_row 逐行误差判定阈值（暂不纳入按键循环，屏幕显示用） */
int16 err_thresh = 10;

/* 十字路口识别（暂不纳入按键循环，屏幕显示用） */
uint8   cross_row_count     = 22;    // 全白行数阈值

const char* param_names[] = {
    "Speed", "BoostMax",
    "BoostUp", "BoostDn", "CurveTh", "Straight"
};
uint8 cur_param_index = PARAM_SPEED;

/* 直线加速 / 弯道减速（speed_boost 加到速度环目标，基础速度=弯道能过的上限） */
uint16 speed_boost       = 0;      // 当前速度增量（0 ~ boost_max）
uint16 boost_max         = 60;     // 速度增量上限（直线峰值 = Speed + boost_max）
uint16 boost_up_step     = 20;      // 直线加速步长（快爬升）
uint16 boost_down_step   = 40;      // 弯道减速步长（急回落）
uint16 curve_thresh      = 20;     // 弯道判断阈值（与79的偏差，超过此 boost 归 0）
uint16 curve_row         = 40;     // 弯道判断行（前方曲率扫描带 25~40 行）
uint16 straight_tol      = 12;     // 直线稳定容差（偏差低于此 boost 满档）

/* 弯道内侧轮反转：|steer_error| 超过 rev_thresh 才反转，量不超过 rev_max */
volatile int16 rev_thresh   = 40;   // 反转触发阈值（对应 pid_steer 输出限幅，按键可调）
volatile int16 rev_max      = 3;    // 反转占空比(%)上限（按键可调）
/* rev_cur（当前反转占空比）是运行状态，定义在 control.c */

#pragma section all restore

void param_change(uint8 direction) {
    int delta = (direction == 0) ? 1 : -1;

    switch (cur_param_index) {
        case PARAM_SPEED:
            {
                int v = (int)target_speed_pulse + delta * 10;
                if (v < 0) v = 0;
                if (v > 1000) v = 1000;
                target_speed_pulse = (uint16)v;
            } break;
        case PARAM_BOOST_MAX:
            {
                int v = (int)boost_max + delta;
                if (v < 0) v = 0;
                if (v > 255) v = 255;
                boost_max = (uint16)v;
            } break;
        case PARAM_BOOST_UP:
            {
                int v = (int)boost_up_step + delta;
                if (v < 0) v = 0;
                if (v > 255) v = 255;
                boost_up_step = (uint16)v;
            } break;
        case PARAM_BOOST_DOWN:
            {
                int v = (int)boost_down_step + delta;
                if (v < 0) v = 0;
                if (v > 255) v = 255;
                boost_down_step = (uint16)v;
            } break;
        case PARAM_CURVE_THRESH:
            {
                int v = (int)curve_thresh + delta;
                if (v < 1) v = 1;
                if (v > 100) v = 100;
                curve_thresh = (uint16)v;
            } break;
        case PARAM_STRAIGHT_TOL:
            {
                int v = (int)straight_tol + delta;
                if (v < 1) v = 1;
                if (v > 100) v = 100;
                straight_tol = (uint16)v;
            } break;
        default: break;
    }
}

/**
 * @brief 生成当前可调参数的显示字符串，如 "Speed=210" / "BoostMax=40"
 * @param buf 输出缓冲区（至少 32 字节）
 */
void param_display_str(char *buf)
{
    switch (cur_param_index) {
        case PARAM_SPEED:         sprintf(buf, "Speed=%d", (int)target_speed_pulse);  break;
        case PARAM_BOOST_MAX:     sprintf(buf, "BoostMax=%d", (int)boost_max);        break;
        case PARAM_BOOST_UP:      sprintf(buf, "BoostUp=%d", (int)boost_up_step);     break;
        case PARAM_BOOST_DOWN:    sprintf(buf, "BoostDn=%d", (int)boost_down_step);   break;
        case PARAM_CURVE_THRESH:  sprintf(buf, "CurveTh=%d", (int)curve_thresh);      break;
        case PARAM_STRAIGHT_TOL:  sprintf(buf, "Straight=%d", (int)straight_tol);     break;
        default:                  sprintf(buf, "---"); break;
    }
}
