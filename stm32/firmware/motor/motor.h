/**
 * @file    motor.h
 * @brief   TB6612 双路电机驱动（速度占空比下发）
 *
 * 不含 PID —— 速度环应在调用者（1 kHz 任务）里做，这里只管"给多少转多少"。
 */
#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

/**
 * @brief  启动 PWM 通道，两轮置为滑行
 */
void motor_init(void);

/**
 * @brief  下发左右轮占空比
 * @param  left / right  范围 ±(MOTOR_PWM_ARR * MOTOR_MAX_DUTY)
 *                       正数 = 前进（方向由 board_cfg.h 的 SIGN 决定）
 *                       0 = 滑行
 */
void motor_set(int16_t left, int16_t right);

/**
 * @brief  滑行停止（IN1=IN2=0，电机自由转，PWM=0）
 */
void motor_stop(void);

/**
 * @brief  短接刹车（IN1=IN2=1，PWM=0）—— 急停用这个
 */
void motor_brake(void);

#endif /* MOTOR_H */
