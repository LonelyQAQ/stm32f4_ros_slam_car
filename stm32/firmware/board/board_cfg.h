/**
 * @file    board_cfg.h
 * @brief   天空星 STM32F407VGT6 载板 —— 底盘相关引脚/定时器/机械参数
 *
 * 引脚依据：载板原理图 H1/H2 解析（见 硬件_天空星F407引脚解析_2026-10-30.md）
 * 换板子只改这一个文件，motor/encoder 源码不用动。
 */
#ifndef BOARD_CFG_H
#define BOARD_CFG_H

#include "main.h"

/* ======================= 电机 PWM（TB6612） ======================= */
/* PWMA = PC8 = TIM3_CH3 或 TIM8_CH3；PWMB = PC9 = TIM3_CH4 或 TIM8_CH4
 * 原来的 tids 工程用的是 TIM3，这里保持一致 */
#define MOTOR_PWM_HTIM        htim3
#define MOTOR_A_CH            TIM_CHANNEL_3   /* PWMA -> 左轮 */
#define MOTOR_B_CH            TIM_CHANNEL_4   /* PWMB -> 右轮 */

/* ARR：TIM3 时钟 84 MHz，20 kHz => 84e6/20000-1 = 4199
 * 20 kHz 超出人耳，避免电机啸叫 */
#define MOTOR_PWM_ARR         4199U
#define MOTOR_MAX_DUTY        0.95f           /* 留 5% 余量，别贴死满占空比 */

/* 方向：AIN1/AIN2 = PC0/PC1，BIN1/BIN2 = PC2/PC3（TB6612） */
#define MOTOR_A_IN1_PORT      GPIOC
#define MOTOR_A_IN1_PIN       GPIO_PIN_0
#define MOTOR_A_IN2_PORT      GPIOC
#define MOTOR_A_IN2_PIN       GPIO_PIN_1

#define MOTOR_B_IN1_PORT      GPIOC
#define MOTOR_B_IN1_PIN       GPIO_PIN_2
#define MOTOR_B_IN2_PORT      GPIOC
#define MOTOR_B_IN2_PIN       GPIO_PIN_3

/* 电机装反了不用改代码，把对应 SIGN 改成 -1 */
#define MOTOR_A_SIGN          (+1)
#define MOTOR_B_SIGN          (+1)

/* ======================= 编码器（硬件正交解码） ======================= */
/* E1A/E1B = PA0/PA1 = TIM2_CH1/CH2；E2A/E2B = PD12/PD13 = TIM4_CH1/CH2 */
#define ENC_L_HTIM            htim2
#define ENC_R_HTIM            htim4

/* 轮子同向转时两路必须同号。
 * 2026-10-31 实测：两轮同向手转，raw L=+280 / raw R=+362 —— 原始同号，
 * 所以这里都用 +1；此前的 -1 是从 tids 工程继承的，那是另一套底盘/接线。 */
#define ENC_L_SIGN            (+1)
#define ENC_R_SIGN            (+1)

/* 编码器 ARR：驱动会读 __HAL_TIM_GET_AUTORELOAD 自适应（TIM2 32 位 / TIM4 16 位），
 * 这里只作记录。建议 TIM2 与 TIM4 都设 65535：16 位定时器最大就是它，
 * 两个一致后行为完全一样，也留足 ±32767 计数/周期的余量。 */
#define ENC_ARR               65535U

/* ======================= 机械参数（标定后改这里） =======================
 * 2026-11-01 按用户提供的电机铭牌核算：
 *   减速比 1:30、输出轴 320 rpm、轮毂直径 65 mm、编码器 11 线（四倍频）
 *   CPR      = 11 × 4 × 30 = 1320 counts/轮转
 *   周长     = π × 0.065 = 0.204204 m
 *   分辨率   = 0.204204 / 1320 = 0.15470 mm/count
 *   空载速度 = 320/60 × 0.204204 = 1.089 m/s（满载实测约 0.5 m/s）
 * 若 320 rpm 指的是减速前电机轴，输出只有 10.7 rpm（3.6 cm/s），与实测不符，
 * 因此这里按"输出轴 320 rpm"理解。仍需用 2 m 直行/原地 5 圈复核轮径与轮距。 */
#define WHEEL_DIAMETER_M      0.065f   /* 轮毂直径，用户实测 65 mm */
#define WHEEL_TRACK_M         0.180f   /* 左右轮接地中心距，待实测 */
#define ENCODER_LINES         11.0f    /* 编码器每转脉冲数（线数） */
#define GEAR_RATIO            30.0f    /* 减速比 1:30 */
#define ENCODER_CPR           (ENCODER_LINES * 4.0f * GEAR_RATIO)  /* 四倍频后的每转计数 */
#define WHEEL_CIRCUMFERENCE_M (3.14159265f * WHEEL_DIAMETER_M)
#define ENC_M_PER_COUNT       (WHEEL_CIRCUMFERENCE_M / ENCODER_CPR)

/* ======================= 控制参数 ======================= */
#define ENC_SPEED_LPF_ALPHA   0.30f    /* 轮速一阶低通系数，0~1，越大越跟手越抖 */
#define MOTOR_CMD_TIMEOUT_MS  300U     /* /cmd_vel 看门狗：超时自动停 */

/* ======================= 轮速 PID =======================
 * TIM12 仍是 1 kHz（位姿积分用），速度环每 WHEEL_SPEED_DIV 拍跑一次：
 * 10 ms 窗口累加编码器计数再换算速度，避开 1 ms 单拍只有 0/1 个计数的量化噪声。
 * 增益单位：Kp = duty/(m/s)，Ki = duty/(m/s·s)，Kd = duty/(m/s²)。
 * 前馈按 APP_V_MAX_MPS 给基础占空比，积分负责吃掉电池电压/负载造成的稳态误差。 */
#define WHEEL_SPEED_DIV         10U
#define WHEEL_PID_KP            5000.0f
#define WHEEL_PID_KI            30000.0f
#define WHEEL_PID_KD            30.0f
#define WHEEL_PID_D_TAU_S       0.030f   /* 微分低通时间常数，压编码器量化噪声 */
#define WHEEL_PID_DEADBAND_MPS  0.005f   /* 目标/实测都低于此值 → 滑行停 */

#endif /* BOARD_CFG_H */
