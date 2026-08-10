#include "zf_common_headfile.h"
#include "shared_data.h"
#include "zf_device_key.h"
#include "motor.h"
#include "control.h"
#include "param.h"

#pragma section all "cpu0_dsram"

/* 10ms 节拍标志（由 PIT ISR 置位，主循环清零） */
volatile uint8_t flag_10ms = 0;

#pragma section all restore

int core0_main(void) {
    clock_init(); debug_init(); system_delay_init();
    motor_init();
    control_init();
    key_init(10);
    pit_ms_init(CCU60_CH0, 10);
    cpu_wait_event_ready();

    while (TRUE) {
        if (flag_10ms) {
            flag_10ms = 0;
            key_scanner();

            if (key_get_state(KEY_1) == KEY_SHORT_PRESS) {
                key_clear_state(KEY_1);
                cur_param_index = (cur_param_index + 1) % PARAM_COUNT;
            }

            if (key_get_state(KEY_2) == KEY_SHORT_PRESS) {
                key_clear_state(KEY_2);
                param_change(0);
            } else if (key_get_state(KEY_2) == KEY_LONG_PRESS) {
                static uint8_t up_cnt = 0;
                if (++up_cnt >= 5) { up_cnt = 0; param_change(0); }
            }
            if (key_get_state(KEY_3) == KEY_SHORT_PRESS) {
                key_clear_state(KEY_3);
                param_change(1);
            } else if (key_get_state(KEY_3) == KEY_LONG_PRESS) {
                static uint8_t down_cnt = 0;
                if (++down_cnt >= 5) { down_cnt = 0; param_change(1); }
            }
            if (key_get_state(KEY_4) == KEY_SHORT_PRESS) {
                key_clear_state(KEY_4);
                clear_integrals();
            }

            /* boost_update 已注释：排除直线加速影响，速度目标固定 */
            // boost_update();

            control_loop();
        }
    }
    return 0;
}

IFX_INTERRUPT(cc60_pit_ch0_isr, 0, CCU6_0_CH0_ISR_PRIORITY) {
    interrupt_global_enable(0);
    pit_clear_flag(CCU60_CH0);
    motor_speed_update();
    flag_10ms = 1;
}
