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
    "Speed", "Rev Th",
    "Rev Max", "Steer KP", "Steer KD"
};
uint8 cur_param_index = PARAM_SPEED;

/* 直线加速 / 弯道减速（speed_boost 加到速度环目标） */
int16 speed_boost       = 0;      // 当前速度增量（0 ~ boost_max）
uint8 boost_max         = 6;      // 速度增量上限
uint8 boost_up_step     = 1;      // 直线加速步长
uint8 boost_down_step   = 2;      // 弯道减速步长
uint8 curve_thresh      = 20;     // 弯道判断阈值（与79的偏差）
uint8 curve_row         = 40;     // 弯道判断行
uint8 straight_tol      = 12;     // 直线稳定容差（25~30 行距中心 79 的偏差）

/* 弯道内侧轮反转：|steer_error| 超过 rev_thresh 才反转，量不超过 rev_max */
volatile int16 rev_thresh   = 40;   // 反转触发阈值（对应 pid_steer 输出限幅，按键可调）
volatile int16 rev_max      = 6;    // 反转占空比(%)上限（按键可调）
/* rev_cur（当前反转占空比）是运行状态，定义在 control.c */

#pragma section all restore

void param_change(uint8 direction) {
    int delta = (direction == 0) ? 1 : -1;

    switch (cur_param_index) {
        case PARAM_SPEED:
            {
                int v = (int)target_speed_pulse + delta * 10;
                if (v < 0) v = 0;
                if (v > 255) v = 255;
                target_speed_pulse = (uint8)v;
            } break;
        case PARAM_REV_THRESH:
            {
                int v = (int)rev_thresh + delta;
                if (v < 0) v = 0;
                if (v > 100) v = 100;
                rev_thresh = (int16)v;
            } break;
        case PARAM_REV_MAX:
            {
                int v = (int)rev_max + delta;
                if (v < 0) v = 0;
                if (v > 50) v = 50;
                rev_max = (int16)v;
            } break;
        case PARAM_STEER_KP:
            {
                float v = pid_steer.KP + ((direction == 0) ? 0.05f : -0.05f);
                if (v < 0.0f) v = 0.0f;
                if (v > 10.0f) v = 10.0f;
                pid_steer.KP = v;
            } break;
        case PARAM_STEER_KD:
            {
                float v = pid_steer.KD + ((direction == 0) ? 0.05f : -0.05f);
                if (v < 0.0f) v = 0.0f;
                if (v > 10.0f) v = 10.0f;
                pid_steer.KD = v;
            } break;
        default: break;
    }
}

/**
 * @brief 生成当前可调参数的显示字符串，如 "RevTh=38" / "SteerKP=1.08"
 * @param buf 输出缓冲区（至少 32 字节）
 */
void param_display_str(char *buf)
{
    switch (cur_param_index) {
        case PARAM_SPEED:       sprintf(buf, "Speed=%d", (int)target_speed_pulse);  break;
        case PARAM_REV_THRESH:  sprintf(buf, "RevTh=%d", (int)rev_thresh);          break;
        case PARAM_REV_MAX:     sprintf(buf, "RevMax=%d", (int)rev_max);            break;
        case PARAM_STEER_KP:    sprintf(buf, "SteerKP=%.2f", pid_steer.KP);         break;
        case PARAM_STEER_KD:    sprintf(buf, "SteerKD=%.2f", pid_steer.KD);         break;
        default:                sprintf(buf, "---"); break;
    }
}
