/**
 * @file    dev_imu.h
 * @brief   BMI088 封装：配置量程、读原始值、换算成 SI 单位
 *
 * 只负责「取数 + 单位换算」；姿态融合在 dev_attitude（VQF 6D）。
 */
#ifndef DEV_IMU_H
#define DEV_IMU_H

#include <stdbool.h>

typedef struct {
    float gyro_rps[3];   /**< rad/s，x/y/z */
    float acc_mps2[3];   /**< m/s^2，x/y/z，含重力 */
    float temp_c;
} dev_imu_sample_t;

bool dev_imu_init(void);

/** 读一帧并换算；失败返回 false 并累加 g_imu_errors */
bool dev_imu_read(dev_imu_sample_t *out);

extern volatile uint32_t g_imu_count;
extern volatile uint32_t g_imu_errors;

#endif /* DEV_IMU_H */
