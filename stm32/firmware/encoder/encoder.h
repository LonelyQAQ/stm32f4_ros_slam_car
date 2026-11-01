/**
 * @file    encoder.h
 * @brief   双轮正交编码器（TIM2 / TIM4 硬件解码）+ 轮速 + 里程计
 *
 * 采样方式：周期调用 encoder_update(dt)，内部读增量并清零计数器。
 * 不做插值、不做平滑以外的处理，EKF 融合交给 Jetson。
 */
#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

typedef struct {
    int32_t total;      /**< 累计计数（带符号，左正右正 = 前进） */
    int16_t delta;      /**< 上一次采样周期内的计数增量 */
    float   speed_mps;  /**< 低通后的线速度，m/s */
} wheel_t;

typedef struct {
    float linear_mps;   /**< 车体前进速度 */
    float yaw_rate_dps; /**< 由左右轮差速算出的角速度，deg/s */
    int32_t left_total; /**< 左轮累计计数（里程计用） */
    int32_t right_total;/**< 右轮累计计数 */
} odom_t;

extern wheel_t g_wheel_l;
extern wheel_t g_wheel_r;

/** 启动编码器定时器并清零 */
void encoder_init(void);

/**
 * @brief  周期采样（建议 5 ms 调用一次，与速度环同节拍）
 * @param  dt  距上次调用的时间，秒
 */
void encoder_update(float dt);

/** 清零累计量与速度（换场地 / 重新标定时用） */
void encoder_reset(void);

/** 取当前里程（线速度 + 差速角速度） */
void odom_get(odom_t *out);

#endif /* ENCODER_H */
