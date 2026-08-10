/*
 * xunxian.c
 * 寻线算法 + 十字路口检测
 * 前方先学习 100 帧（约 2 秒），之后进入寻线稳定
 */

#include "zf_common_headfile.h"
#include "xunxian.h"

/* 外部变量 */
extern uint8 cross_row_count;

/*------------ 局部静态变量 ------------*/
static uint8 width_ref[MT9V03X_H];          // 每行参考赛道宽度（像素）
static uint8 border_state[MT9V03X_H];      // 0无效 1双边 2左边 3右边
static uint16 learn_frame_cnt = 0;         // 学习帧计数
#define LEARN_MAX_FRAMES  100              // 学习帧数 (50fps * 2s = 100帧)

/*------------ 初始化 ------------*/
void track_init(void)
{
    for (int i = 0; i < MT9V03X_H; i++) {
        width_ref[i] = 0;
        border_state[i] = 0;
    }
    learn_frame_cnt = 0;                    // 复位帧计数
}

/**
 * @brief 判断一行是否为"纯白行"（十字白带/缺线）
 *
 * 检测逻辑：
 *  1) 摄像头图像最左列(0)与最右列(W-1)一开始就是白色；
 *  2) 往中间数白色像素数量，若超过一定值(ALL_WHITE_MIN_W) → 标记为“可能全白行”；
 *  3) 扫描这一整行，若黑色像素数量小于一定值(ALL_WHITE_MAX_BLACK) → 判定为全白行。
 *
 * @return 1: 纯白行  0: 非纯白行
 */
static uint8 row_is_all_white(uint8 row)
{
    uint8 k;
    uint16 white_cnt = 0, black_cnt = 0;

    /* 1) 最左、最右一开始就是白色 */
    if (mt9v03x_image[row][0]        != 255 ||
        mt9v03x_image[row][TRACK_IMAGE_W - 3] != 255)
        return 0;

    /* 2) 往中间数白色像素，超过阈值 → “可能全白行” */
    for (k = 0; k < TRACK_IMAGE_W; k++)
        if (mt9v03x_image[row][k] == 255) white_cnt++;
    if (white_cnt < ALL_WHITE_MIN_WHITE)
        return 0;

    /* 3) 扫描这一整行，黑色像素小于一定数量 → 全白行 */
    for (k = 0; k < TRACK_IMAGE_W; k++)
        if (mt9v03x_image[row][k] == 0) black_cnt++;

    return (black_cnt < ALL_WHITE_MAX_BLACK) ? 1 : 0;
}

/**
 * @brief 对外公开的全白行判定（供 cpu1 显示提示用）
 * @param row 行号
 * @return 1: 纯白行  0: 非纯白行
 */
uint8 row_is_all_white_pub(uint8 row)
{
    return row_is_all_white(row);
}

/**
 * 提取中线（补充）：扫描左右边界，计算每行中线。
 * 纯白行处理：直接置 INVALID_BORDER，再交给 fix_mid_line 插值补线。
 * - 若该行被上述"纯白行"判定为全白，则视为缺线。
 * - 该行还需要在行内统计黑色像素：若 <5 判定为纯白行，否则按普通行处理。
 */
void extract_mid_line(uint8 left_bound[], uint8 right_bound[], uint8 mid_line[])
{
    uint8 row, col;

    for (row = TRACK_START_ROW; row <= TRACK_END_ROW; row++)
    {
        int left = INVALID_BORDER;
        int right = INVALID_BORDER;

        // 扫描左右边界
        for (col = 0; col < TRACK_IMAGE_W; col++) {
            if (mt9v03x_image[row][col] == 255) { left = col; break; }
        }
        for (col = TRACK_IMAGE_W - 3; col > 0; col--) {
            if (mt9v03x_image[row][col] == 255) { right = col; break; }
        }

        left_bound[row]  = (uint8)(left == INVALID_BORDER ? 0 : left);
        right_bound[row] = (uint8)(right == INVALID_BORDER ? 0 : right);

        if (row_is_all_white(row))
        {
            /* 纯白（十字白带/缺线）边缘扫描到的都是白色，直接取中线丢失，
               置 INVALID 交给 fix_mid_line 补线 */
            mid_line[row] = INVALID_BORDER;
            border_state[row] = 0;
        }
        else if (left != INVALID_BORDER && right != INVALID_BORDER && left < right)
        {
            /* 左右边界都有效 = 双边，取中间为中准线 */
            mid_line[row] = (uint8)((left + right) >> 1);
            border_state[row] = 1;
            /* 学习参考宽度：仅在双边界行平滑更新，抗闪烁。
               单边界丢线时用 width_ref 估计中线（透视下近宽远窄，按行存）。 */
            {
                int w = right - left;
                if (w >= 4)
                {
                    if (width_ref[row] == 0)
                        width_ref[row] = (uint8)w;
                    else
                        width_ref[row] = (uint8)(((uint16)width_ref[row] + (uint16)w) >> 1);
                }
            }
        }
        else if (right != INVALID_BORDER && left == INVALID_BORDER)
        {
            /* 只有右边 = 左转弯 / 丢线，利用学习的行宽估计 */
            border_state[row] = 3;
            if (width_ref[row] > 0 && width_ref[row] <= right)
            {
                int est_left = right - (int)width_ref[row];
                if (est_left >= 0)
                    mid_line[row] = (uint8)(right - (width_ref[row] >> 1));
                else
                    mid_line[row] = INVALID_BORDER;
            }
            else
                mid_line[row] = INVALID_BORDER;
        }
        else if (left != INVALID_BORDER && right == INVALID_BORDER)
        {
            /* 只有左边 = 右转弯 / 丢线 */
            border_state[row] = 2;
            if (width_ref[row] > 0 && (left + (int)width_ref[row]) <= (int)TRACK_IMAGE_W)
                mid_line[row] = (uint8)(left + (width_ref[row] >> 1));
            else
                mid_line[row] = INVALID_BORDER;
        }
        else
        {
            /* 左右边界都没有 = 全丢，保留 INVALID */
            mid_line[row] = INVALID_BORDER;
            border_state[row] = 0;
        }
    }

    // 每帧图像学习帧计数自加
    if (learn_frame_cnt < LEARN_MAX_FRAMES)
        learn_frame_cnt++;
}

/**
 * @brief 十字路口识别：只统计屏幕上 50~110 行之间的纯白行数量，
 *        只要全白行数 > 设定值(cross_row_count)，即判定为十字路口。
 * @return 1: 十字路口  0: 非十字路口
 */
uint8 detect_crossroad(uint8 left_bound[], uint8 right_bound[])
{
    uint8 row, cnt = 0;

    for (row = CROSS_ROW_LO; row <= CROSS_ROW_HI; row++)
    {
        if (row_is_all_white(row))
            cnt++;
    }

    return (cnt > cross_row_count) ? 1 : 0;
}

/**
 * 补线（丢线补值）：对 INVALID_BORDER 部分在有效行之间做线性插值。
 * 便于找中线，应在每帧调用 extract_mid_line() 之后调用。
 */
void fix_mid_line(uint8 mid_line[], uint16 start_row, uint16 end_row)
{
    uint16 row;
    for (row = start_row; row <= end_row; row++)
    {
        if (mid_line[row] == INVALID_BORDER)
        {
            int16 up = -1, dn = -1;
            uint16 r;

            for (r = row; r > start_row; r--)
                if (mid_line[r - 1] != INVALID_BORDER) { up = (int16)(r - 1); break; }

            for (r = row + 1; r <= end_row; r++)
                if (mid_line[r] != INVALID_BORDER) { dn = (int16)r; break; }

            if (up >= 0 && dn > up)
            {
                int32 num = (int32)(mid_line[dn] - mid_line[up]) * (int32)(row - up);
                mid_line[row] = (uint8)(mid_line[up] + (num / (dn - up)));
            }
            else if (up >= 0)
            {
                mid_line[row] = mid_line[up];
            }
            else if (dn >= 0)
            {
                mid_line[row] = mid_line[dn];
            }
        }
    }
}

/* 二值化 */
void binary_image(uint8 threshold){
    for (uint16 i = 0; i < TRACK_IMAGE_H; i++)
        for (uint16 j = 0; j < TRACK_IMAGE_W; j++)
            mt9v03x_image[i][j] = (mt9v03x_image[i][j] > threshold) ? 255 : 0;
}

/**
 * @brief 判断是否还在学习阶段
 * @return 1: 学习未完成  0: 已完成
 */
uint8 is_learning(void)
{
    return (learn_frame_cnt < LEARN_MAX_FRAMES) ? 1 : 0;
}
