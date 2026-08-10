#ifndef __PARAM_H__
#define __PARAM_H__

#include "zf_common_headfile.h"

/* 上位机可调参数索引（与 param_names[] 对应；Cross Rows/Err Th 暂不纳入按键循环） */
typedef enum {
    PARAM_SPEED = 0,
    PARAM_REV_THRESH,
    PARAM_REV_MAX,
    PARAM_STEER_KP,
    PARAM_STEER_KD,
    PARAM_COUNT
} ParamIndex;

void param_change(uint8 direction);
void param_display_str(char *buf);   /* 生成"当前参数名=值"字符串，供屏幕显示 */

#endif
