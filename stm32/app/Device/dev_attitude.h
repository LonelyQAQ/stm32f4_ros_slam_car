/**
 * @file    dev_attitude.h
 * @brief   BMI088 + VQF 姿态解算（6D：陀螺 + 加速度，无磁力计）
 *
 * 采样由 ACC_STATUS.DRDY 驱动（≈200 Hz，天然对齐 ODR）。yaw 不可观测，
 * 只作高频角速率参考；绝对航向由 Jetson 侧 EKF + 激光匹配提供。
 */
#ifndef DEV_ATTITUDE_H
#define DEV_ATTITUDE_H

#include <stdbool.h>

#include "dev_imu.h"

typedef struct {
    float            q[4];      /**< w,x,y,z：机体系 → 地理系（VQF 6D） */
    dev_imu_sample_t sample;    /**< 与 q 同一帧的原始 SI 数据 */
    uint32_t         updates;   /**< 已送入 VQF 的样本数 */
    float            bias[3];   /**< VQF 当前估计的陀螺零偏（三轴），rad/s */
    uint8_t          rest;      /**< VQF 静止检测是否成立 */
    uint8_t          zupt;      /**< 本模块的零速修正是否生效 */
} dev_attitude_t;

/** 必须在 dev_imu_init() 成功后调用 */
bool dev_attitude_init(void);

/** 主循环里反复调用：只有检测到新样本才做解算，无样本时直接返回 */
void dev_attitude_poll(void);

/** 取最新姿态（线程不安全，只在主循环里用） */
bool dev_attitude_get(dev_attitude_t *out);

/** 取最新一帧 IMU（遥测用，避免和 200 Hz 采样抢 SPI） */
bool dev_attitude_get_sample(dev_imu_sample_t *out);

extern volatile uint32_t g_att_updates;
extern volatile uint32_t g_att_errors;

#endif /* DEV_ATTITUDE_H */
