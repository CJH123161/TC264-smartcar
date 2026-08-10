/*
 * pid.c
 * PID 控制器：位置式，积分限幅 + 输出限幅
 */

#include "zf_common_headfile.h"
#include "pid.h"

void PID_Init(PID *pid, float kp, float ki, float kd, float ilim, float olim) {
    pid->KP = kp; pid->KI = ki; pid->KD = kd;
    pid->integral = 0; pid->last_error = 0;
    pid->integral_limit = ilim; pid->output_limit = olim;
}

float PID_Calc(PID *pid, float target, float actual) {
    float error = target - actual;
    float P = pid->KP * error;
    pid->integral += error;
    if (pid->integral > pid->integral_limit) pid->integral = pid->integral_limit;
    if (pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;
    float I = pid->KI * pid->integral;
    float D = pid->KD * (pid->last_error - error);
    pid->last_error = error;
    float out = P + I + D;
    if (out > pid->output_limit) out = pid->output_limit;
    if (out < -pid->output_limit) out = -pid->output_limit;
    return out;
}
