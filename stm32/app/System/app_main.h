/**
 * @file    app_main.h
 * @brief   应用入口：外设初始化 + 控制节拍 + 主循环
 *
 * 控制环跑在 TIM12 更新中断（app_tick_isr），主循环只跑 micro-ROS。
 */
#ifndef APP_MAIN_H
#define APP_MAIN_H

#include <stdint.h>

/** 上电调用一次（CubeMX 的 MX_xxx_Init 之后） */
void app_main_init(void);

/** TIM12 更新中断里调用 */
void app_tick_isr(void);

/** while(1) 里调用 */
void app_loop(void);

/** 取里程计位姿（x/y 单位 m，theta 单位 rad） */
void app_get_pose(float *x, float *y, float *theta);

extern volatile uint32_t g_ctrl_ticks;
extern volatile uint32_t g_boot_errors;

#endif /* APP_MAIN_H */
