#ifndef __PID_H__
#define __PID_H__

#include "zf_common_headfile.h"

typedef struct {
    float KP, KI, KD;
    float integral, last_error;
    float integral_limit, output_limit;
} PID;

void PID_Init(PID *pid, float kp, float ki, float kd, float ilim, float olim);
float PID_Calc(PID *pid, float target, float actual);

#endif
