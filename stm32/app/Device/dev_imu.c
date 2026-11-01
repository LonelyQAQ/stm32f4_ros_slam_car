/**
 * @file    dev_imu.c
 * @brief   BMI088 设备封装
 *
 * 依赖 bmi088/bmi088.h（SPI1 + PC4/PC5 双片选，读操作走 DMA）。
 */
#include "microros_app_includes.h"

extern SPI_HandleTypeDef hspi1;

volatile uint32_t g_imu_count  = 0;
volatile uint32_t g_imu_errors = 0;

static const BMI088_Config s_imu_cfg = {
    .accel_range = BMI088_ACC_RANGE_6G,
    .accel_odr   = BMI088_ACC_ODR_200_HZ,
    .accel_bwp   = BMI088_ACC_BWP_NORMAL,
    .gyro_range  = APP_GYRO_RANGE,
    .gyro_bw     = BMI088_GYRO_BW_400_116,
};

bool dev_imu_init(void)
{
    if (BMI088_Init(&hspi1, &s_imu_cfg) != BMI088_OK) {
        g_imu_errors++;
        return false;
    }
    return true;
}

bool dev_imu_read(dev_imu_sample_t *out)
{
    BMI088_Data raw;

    if (out == NULL) {
        return false;
    }
    if (BMI088_ReadAll(&raw) != BMI088_OK) {
        g_imu_errors++;
        return false;
    }

    const float dps_to_rps = 3.14159265f / 180.0f;
    const float lsb_to_g   = 1.0f / APP_ACC_LSB_PER_G;

    out->gyro_rps[0] = (float)raw.gyro.x / APP_GYRO_LSB_PER_DPS * dps_to_rps;
    out->gyro_rps[1] = (float)raw.gyro.y / APP_GYRO_LSB_PER_DPS * dps_to_rps;
    out->gyro_rps[2] = (float)raw.gyro.z / APP_GYRO_LSB_PER_DPS * dps_to_rps;

    out->acc_mps2[0] = (float)raw.accel.x * lsb_to_g * APP_G;
    out->acc_mps2[1] = (float)raw.accel.y * lsb_to_g * APP_G;
    out->acc_mps2[2] = (float)raw.accel.z * lsb_to_g * APP_G;

    /* 驱动已把寄存器右移 5 位成 11 位有符号数，按手册 0.125 °C/LSB */
    out->temp_c = 23.0f + (float)raw.temperature * 0.125f;

    g_imu_count++;
    return true;
}
