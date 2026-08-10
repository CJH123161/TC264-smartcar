#include "zf_common_headfile.h"
#include "xunxian.h"
#include "shared_data.h"

#pragma section all "cpu1_dsram"

volatile uint8 cross_state = 0;

void core1_main(void)
{
    disable_Watchdog();
    interrupt_global_enable(0);

    ips200_init(IPS200_TYPE_SPI);
    mt9v03x_init();
    track_init();

    cpu_wait_event_ready();

    uint8_t show_counter = 0;
    uint8_t cross_enter_cnt = 0;
    uint8_t cross_exit_cnt = 0;

    while (TRUE)
    {
        if (mt9v03x_finish_flag)
        {
            mt9v03x_finish_flag = 0;
            binary_image(heibaival);
            extract_mid_line(left_bound, right_bound, mid_line);
            fix_mid_line(mid_line, TRACK_START_ROW, TRACK_END_ROW);
            if (detect_crossroad(left_bound, right_bound))
            {
                cross_exit_cnt = 0;
                if (++cross_enter_cnt >= 3) cross_state = 1;
            }
            else
            {
                cross_enter_cnt = 0;
                if (cross_state && ++cross_exit_cnt >= 3) cross_state = 0;
            }
        }

        if (++show_counter >= 5)
        {
            show_counter = 0;

            ips200_show_gray_image(0, 0, mt9v03x_image[0], MT9V03X_W, MT9V03X_H,
                                   MT9V03X_W, MT9V03X_H, heibaival);

            for (uint16 row = TRACK_START_ROW; row <= TRACK_END_ROW; row++)
            {
                if (row_is_all_white_pub((uint8)row))
                {
                    /* 全白行提示：该行图像中心打黄点 */
                    ips200_draw_point(TRACK_IMAGE_W / 2,     row, RGB565_YELLOW);
                    ips200_draw_point(TRACK_IMAGE_W / 2 - 6, row, RGB565_YELLOW);
                    ips200_draw_point(TRACK_IMAGE_W / 2 + 6, row, RGB565_YELLOW);
                }
                if (left_bound[row] != 0)
                    ips200_draw_point(left_bound[row], row, RGB565_GREEN);
                if (right_bound[row] != 0)
                    ips200_draw_point(right_bound[row], row, RGB565_GREEN);
                if (mid_line[row] != INVALID_BORDER)
                    ips200_draw_point(mid_line[row], row, RGB565_RED);
            }

            /* 弯道判定行（curve_row，按键可调）画紫色横线 */
            if (curve_row < TRACK_IMAGE_H)
                ips200_draw_line(0, curve_row, TRACK_IMAGE_W - 1, curve_row, RGB565_PURPLE);

            /* 直道加速判定区间两端（STR_ROW_LO / STR_ROW_HI，与 CPU0 共用宏）画横线 */
            if (STR_ROW_LO < TRACK_IMAGE_H)
                ips200_draw_line(0, STR_ROW_LO, TRACK_IMAGE_W - 1, STR_ROW_LO, RGB565_CYAN);
            if (STR_ROW_HI < TRACK_IMAGE_H)
                ips200_draw_line(0, STR_ROW_HI, TRACK_IMAGE_W - 1, STR_ROW_HI, RGB565_GREEN);

            char buf[48];

            // 第一行：当前可调参数名 + 值
            param_display_str(buf);
            ips200_show_string(0, 125, buf);

            // 第二行：转向 PID + 当前偏差
            sprintf(buf, "St:%.2f/%.2f Err:%4d", pid_steer.KP, pid_steer.KD, steer_error);
            ips200_show_string(0, 145, buf);

            // 第三行：左右实际速度 + 速度增量
            sprintf(buf, "L S:%4d R S:%4d B:%2d", left_real_speed, right_real_speed, speed_boost);
            ips200_show_string(0, 165, buf);

            // 第四行：十字识别参数 + 误差阈值(TH)
            sprintf(buf, "CR:%d TH:%d", cross_row_count, err_thresh);
            ips200_show_string(0, 185, buf);

            // 第五行：反转阈值 + 反转上限
            sprintf(buf, "RevTh:%d RevMax:%d", rev_thresh, rev_max);
            ips200_show_string(0, 205, buf);

            // 第六行：当前反转量
            sprintf(buf, "RevCur:%d", rev_cur);
            ips200_show_string(0, 225, buf);

            // 第七行：十字路口状态 + 学习提示
            sprintf(buf, "%-6s %s", cross_state ? "CROSS!" : "", is_learning() ? "" : "OK!");
            ips200_show_string(0, 245, buf);
        }
    }
}

#pragma section all restore
