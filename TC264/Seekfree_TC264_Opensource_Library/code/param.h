#ifndef __PARAM_H__
#define __PARAM_H__

#include "zf_common_headfile.h"

/* 按键可调参数索引（与 param_names[] 对应；Cross Rows/Err Th 暂不纳入按键循环） */
typedef enum {
    PARAM_SPEED = 0,
    PARAM_BOOST_MAX,
    PARAM_BOOST_UP,
    PARAM_BOOST_DOWN,
    PARAM_CURVE_THRESH,
    PARAM_STRAIGHT_TOL,
    PARAM_COUNT
} ParamIndex;

void param_change(uint8 direction);
void param_display_str(char *buf);   /* 生成"当前参数名=值"字符串，供屏幕显示 */

#endif
