/*
 * motor.c
 * 电机驱动：PWM 输出 + 方向脚 + 编码器测速
 */

#include "zf_common_headfile.h"
#include "motor.h"

#pragma section all "cpu0_dsram"

volatile int16 left_real_speed = 0;
volatile int16 right_real_speed = 0;

static int16 last_left_encoder = 0;
static int16 last_right_encoder = 0;

#pragma section all restore

void motor_init(void)
{
    gpio_init(R1, GPO, 1, GPO_PUSH_PULL);
    gpio_init(R2, GPO, 1, GPO_PUSH_PULL);
    pwm_init(MOTOR_LEFT_PWM,  17000, 0);
    pwm_init(MOTOR_RIGHT_PWM, 17000, 0);
    encoder_dir_init(ENCODER_LEFT,  TIM5_ENCODER_CH1_P10_3, TIM5_ENCODER_CH2_P10_1);
    encoder_dir_init(ENCODER_RIGHT, TIM6_ENCODER_CH1_P20_3, TIM6_ENCODER_CH2_P20_0);
    encoder_clear_count(ENCODER_LEFT);
    encoder_clear_count(ENCODER_RIGHT);
}

void motor_set_pwm(int left_duty, int right_duty) {
    if (left_duty  >  100) left_duty  =  100;
    if (left_duty  < -100) left_duty  = -100;
    if (right_duty >  100) right_duty =  100;
    if (right_duty < -100) right_duty = -100;
    /* 方向脚极性：左轮 高=正转/低=反转，右轮 低=正转/高=反转 */
    gpio_set_level(R1, (left_duty  < 0) ? 0 : 1);
    gpio_set_level(R2, (right_duty < 0) ? 1 : 0);
    if (left_duty  < 0) left_duty  = -left_duty;
    if (right_duty < 0) right_duty = -right_duty;
    pwm_set_duty(MOTOR_LEFT_PWM,  left_duty  * 100);
    pwm_set_duty(MOTOR_RIGHT_PWM, right_duty * 100);
}

void motor_speed_update(void)
{
    int16 cur_left  = encoder_get_count(TIM5_ENCODER);
    int16 cur_right = encoder_get_count(TIM6_ENCODER);
    left_real_speed  = cur_left  - last_left_encoder;
    right_real_speed = last_right_encoder - cur_right;
    last_left_encoder  = cur_left;
    last_right_encoder = cur_right;
}
