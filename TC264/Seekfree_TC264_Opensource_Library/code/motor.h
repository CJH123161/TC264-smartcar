#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "zf_common_headfile.h"

/* 电机与编码器引脚定义 */
#define MOTOR_LEFT_PWM      ATOM0_CH7_P02_7
#define MOTOR_RIGHT_PWM     ATOM0_CH5_P02_5
#define R1                  P02_6   /* 左轮方向脚：高=正转，低=反转 */
#define R2                  P02_4   /* 右轮方向脚：低=正转，高=反转 */

#define ENCODER_LEFT        TIM5_ENCODER
#define ENCODER_RIGHT       TIM6_ENCODER

void motor_init(void);
void motor_set_pwm(int left_duty, int right_duty);
void motor_speed_update(void);   /* 由 10ms 中断调用：读编码器并算实际速度 */

#endif
