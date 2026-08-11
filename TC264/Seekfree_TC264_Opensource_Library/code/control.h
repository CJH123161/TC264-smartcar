#ifndef __CONTROL_H__
#define __CONTROL_H__

#include "zf_common_headfile.h"

/* 图像中线参考列（像素）：非 TRACK_IMAGE_W/2，是摄像头安装后的经验偏移值 */
#define LINE_CENTER   79

/* err 计算用宏：90~55 行，步进 5（注意：行号越小=越前方/越远，行号越大=越近车头，勿与"最远/最近"直译混淆） */
#define ERR_ROW_HI    100             // 较大行号 → 近处行（车头侧）
#define ERR_ROW_LO    55             // 较小行号 → 远处行（前方侧）
#define ERR_ROW_STEP  5

#define ERR_MAX_STEP  15             // 每 10ms 转向误差最大变化量

/* 反转触发步长与保底占空比（代码常量，非按键可调） */
#define REV_STEP      4              // 反转量每 10ms 变化量（从0到10%约 25ms）
#define REV_MIN_BASE  1             // 反转保底占空比(%)（刻意低于电机死区≈10%，高速下反转轮实际不动→等效禁用反转，防"转过了"）

/* 直线判定确认帧数 */
#define STRAIGHT_REQ_FRAMES  3       // 直线稳定确认帧数

void control_init(void);              /* 初始化 PID 实例（速度环/转向环参数） */
void control_loop(void);              /* 每 10ms 调用：速度环+转向环+PWM 合成 */
void clear_integrals(void);

#endif
