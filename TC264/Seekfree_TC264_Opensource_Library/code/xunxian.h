#ifndef __XUNXIAN_H__
#define __XUNXIAN_H__

#include "zf_common_headfile.h"

#define TRACK_IMAGE_H  MT9V03X_H   // 120
#define TRACK_IMAGE_W  MT9V03X_W   // 188

#define INVALID_BORDER   255

#ifndef TRACK_START_ROW
#define TRACK_START_ROW  10
#endif
#ifndef TRACK_END_ROW
#define TRACK_END_ROW    110       // 不能超过119
#endif

/* 全白行判定阈值 */
#ifndef ALL_WHITE_MIN_WHITE
#define ALL_WHITE_MIN_WHITE  150    /* 该行白色像素至少需为 W-? 才可能是全白行 */
#endif
#ifndef ALL_WHITE_MAX_BLACK
#define ALL_WHITE_MAX_BLACK  5      /* 该行黑色像素数量低于该值才算全白行 */
#endif

/* 十字路口识别：只统计 50~110 行之间的全白行数量 */
#ifndef CROSS_ROW_LO
#define CROSS_ROW_LO  22
#endif
#ifndef CROSS_ROW_HI
#define CROSS_ROW_HI   110
#endif

extern uint8 mt9v03x_image[MT9V03X_H][MT9V03X_W];
// 原基于 update 后算法之间保持兼容
uint8 is_learning(void);   // 返回 1 表示还在学习阶段
uint8 row_is_all_white_pub(uint8 row);   // 全白行判定（供显示提示）
void track_init(void);
void binary_image(uint8 threshold);
void extract_mid_line(uint8 left_bound[], uint8 right_bound[], uint8 mid_line[]);
void fix_mid_line(uint8 mid_line[], uint16 start_row, uint16 end_row);
uint8 detect_crossroad(uint8 left_bound[], uint8 right_bound[]);

#endif
