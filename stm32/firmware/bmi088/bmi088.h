/**
 * @file    bmi088.h
 * @brief   BMI088 6-Axis IMU Driver Header (Hardware SPI1)
 *
 * @details BMI088 是 Bosch Sensortec 的 6 轴惯性测量单元，包含一个 16 位
 *          三轴加速度计和一个 16 位三轴陀螺仪。两个传感器位于同一封装内，
 *          共用 SPI 总线（SCK/MOSI/MISO），但使用独立的片选引脚：
 *            - CSB1 (PC4) → 加速度计
 *            - CSB2 (PC5) → 陀螺仪
 *
 *          SPI 模式：Mode 0（CPOL=0, CPHA=0），时钟 ≤10 MHz。
 *
 * @warning 加速度计上电默认为 I²C 模式，必须通过一次 SPI 假读操作
 *          （CSB1 上升沿）才能切换到 SPI 模式。本驱动在 Init() 中自动处理。
 */

#ifndef __BMI088_H
#define __BMI088_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdbool.h>
#include <stdint.h>

/* ========================== 片选引脚宏（优先使用 CubeMX 生成的定义）======== */
#ifndef BMI088_ACC_CS_Pin
  #ifdef BMI088_CSB1_Pin
    #define BMI088_ACC_CS_Pin   BMI088_CSB1_Pin
    #define BMI088_ACC_CS_Port  BMI088_CSB1_GPIO_Port
  #else
    #define BMI088_ACC_CS_Pin   GPIO_PIN_4
    #define BMI088_ACC_CS_Port  GPIOC
  #endif
#endif
#ifndef BMI088_GYRO_CS_Pin
  #ifdef BMI088_CSB2_Pin
    #define BMI088_GYRO_CS_Pin  BMI088_CSB2_Pin
    #define BMI088_GYRO_CS_Port BMI088_CSB2_GPIO_Port
  #else
    #define BMI088_GYRO_CS_Pin  GPIO_PIN_5
    #define BMI088_GYRO_CS_Port GPIOC
  #endif
#endif

#define BMI088_ACC_CS_SET()     HAL_GPIO_WritePin(BMI088_ACC_CS_Port,  BMI088_ACC_CS_Pin,  GPIO_PIN_SET)
#define BMI088_ACC_CS_CLR()     HAL_GPIO_WritePin(BMI088_ACC_CS_Port,  BMI088_ACC_CS_Pin,  GPIO_PIN_RESET)
#define BMI088_GYRO_CS_SET()    HAL_GPIO_WritePin(BMI088_GYRO_CS_Port, BMI088_GYRO_CS_Pin, GPIO_PIN_SET)
#define BMI088_GYRO_CS_CLR()    HAL_GPIO_WritePin(BMI088_GYRO_CS_Port, BMI088_GYRO_CS_Pin, GPIO_PIN_RESET)

/* ========================== SPI 句柄 ======================================== */
/* SPI 句柄由 BMI088_Init() 传入，驱动内部存储为 bmi088_hspi */

/* ========================== 加速度计 寄存器地址（CSB1）====================== */
#define BMI088_ACC_CHIP_ID          0x00U
#define BMI088_ACC_ERR_REG          0x02U
#define BMI088_ACC_STATUS           0x03U
#define BMI088_ACC_X_LSB            0x12U
#define BMI088_ACC_X_MSB            0x13U
#define BMI088_ACC_Y_LSB            0x14U
#define BMI088_ACC_Y_MSB            0x15U
#define BMI088_ACC_Z_LSB            0x16U
#define BMI088_ACC_Z_MSB            0x17U
#define BMI088_ACC_SENSORTIME_0     0x18U
#define BMI088_ACC_SENSORTIME_1     0x19U
#define BMI088_ACC_SENSORTIME_2     0x1AU
#define BMI088_ACC_INT_STAT_1       0x1DU
#define BMI088_ACC_TEMP_MSB         0x22U
#define BMI088_ACC_TEMP_LSB         0x23U
#define BMI088_ACC_CONF             0x40U
#define BMI088_ACC_RANGE            0x41U
#define BMI088_ACC_INT1_IO_CTRL     0x53U
#define BMI088_ACC_INT2_IO_CTRL     0x54U
#define BMI088_ACC_SELF_TEST        0x6DU
#define BMI088_ACC_PWR_CONF         0x7CU
#define BMI088_ACC_PWR_CTRL         0x7DU

#define BMI088_ACC_CHIP_ID_VAL      0x1EU
#define BMI088_ACC_PWR_CTRL_SUSPEND 0x00U
#define BMI088_ACC_PWR_CTRL_ACTIVE  0x04U
#define BMI088_ACC_PWR_CONF_ACTIVE  0x00U
#define BMI088_ACC_PWR_CONF_SUSPEND 0x03U

#define BMI088_ACC_CONF_ODR_Pos     0U
#define BMI088_ACC_CONF_ODR_Msk     (0x0FU << BMI088_ACC_CONF_ODR_Pos)
#define BMI088_ACC_CONF_BWP_Pos     4U
#define BMI088_ACC_CONF_BWP_Msk     (0x07U << BMI088_ACC_CONF_BWP_Pos)

#define BMI088_ACC_ODR_12_5_HZ      0x05U
#define BMI088_ACC_ODR_25_HZ        0x06U
#define BMI088_ACC_ODR_50_HZ        0x07U
#define BMI088_ACC_ODR_100_HZ       0x08U
#define BMI088_ACC_ODR_200_HZ       0x09U
#define BMI088_ACC_ODR_400_HZ       0x0AU
#define BMI088_ACC_ODR_800_HZ       0x0BU
#define BMI088_ACC_ODR_1600_HZ      0x0CU

#define BMI088_ACC_BWP_OSR4         0x00U
#define BMI088_ACC_BWP_OSR2         0x01U
#define BMI088_ACC_BWP_NORMAL       0x02U

#define BMI088_ACC_RANGE_Pos        0U
#define BMI088_ACC_RANGE_Msk        0x03U
#define BMI088_ACC_RANGE_3G         0x00U
#define BMI088_ACC_RANGE_6G         0x01U
#define BMI088_ACC_RANGE_12G        0x02U
#define BMI088_ACC_RANGE_24G        0x03U

#define BMI088_ACC_SENS_3G          10920U
#define BMI088_ACC_SENS_6G          5461U
#define BMI088_ACC_SENS_12G         2730U
#define BMI088_ACC_SENS_24G         1365U

#define BMI088_ACC_STATUS_DRDY      0x80U
#define BMI088_ACC_ERR_FATAL        0x01U
#define BMI088_ACC_ERR_CMD          0x02U
#define BMI088_ACC_SELF_TEST_EN     0x0DU
#define BMI088_ACC_SELF_TEST_DIS    0x00U

/* ========================== 陀螺仪 寄存器地址（CSB2）======================== */
#define BMI088_GYRO_CHIP_ID         0x00U
#define BMI088_GYRO_RATE_X_LSB      0x02U
#define BMI088_GYRO_RATE_X_MSB      0x03U
#define BMI088_GYRO_RATE_Y_LSB      0x04U
#define BMI088_GYRO_RATE_Y_MSB      0x05U
#define BMI088_GYRO_RATE_Z_LSB      0x06U
#define BMI088_GYRO_RATE_Z_MSB      0x07U
#define BMI088_GYRO_INT_STAT_1      0x0AU
#define BMI088_GYRO_RANGE           0x0FU
#define BMI088_GYRO_BANDWIDTH       0x10U
#define BMI088_GYRO_LPM1            0x11U
#define BMI088_GYRO_SOFTRESET       0x14U
#define BMI088_GYRO_INT_CTRL        0x15U
#define BMI088_GYRO_INT3_INT4_IO    0x16U

#define BMI088_GYRO_CHIP_ID_VAL     0x0FU

#define BMI088_GYRO_RANGE_Pos       0U
#define BMI088_GYRO_RANGE_Msk       0x07U
#define BMI088_GYRO_RANGE_2000      0x00U
#define BMI088_GYRO_RANGE_1000      0x01U
#define BMI088_GYRO_RANGE_500       0x02U
#define BMI088_GYRO_RANGE_250       0x03U
#define BMI088_GYRO_RANGE_125       0x04U

#define BMI088_GYRO_SENS_2000       16U
#define BMI088_GYRO_SENS_1000       33U
#define BMI088_GYRO_SENS_500        66U
#define BMI088_GYRO_SENS_250        131U
#define BMI088_GYRO_SENS_125        262U

#define BMI088_GYRO_BW_Pos          0U
#define BMI088_GYRO_BW_Msk          0x0FU
#define BMI088_GYRO_BW_2000_532     0x00U
#define BMI088_GYRO_BW_2000_230     0x01U
#define BMI088_GYRO_BW_400_116      0x03U
#define BMI088_GYRO_BW_200_64       0x04U
#define BMI088_GYRO_BW_100_32       0x05U
#define BMI088_GYRO_BW_400_47       0x06U

#define BMI088_GYRO_LPM1_SUSPEND    0x80U
#define BMI088_GYRO_LPM1_DEEP_SUSP  0x20U
#define BMI088_GYRO_LPM1_NORMAL     0x00U
#define BMI088_GYRO_RESET_CMD       0xB6U
#define BMI088_GYRO_INT_CTRL_DATA   0x01U
#define BMI088_GYRO_STAT_DRDY       0x80U

/* ========================== 数据结构体 ====================================== */

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} BMI088_AxisRaw;

typedef struct {
    BMI088_AxisRaw accel;
    BMI088_AxisRaw gyro;
    int16_t        temperature;
} BMI088_Data;

typedef struct {
    uint8_t accel_range;
    uint8_t accel_odr;
    uint8_t accel_bwp;
    uint8_t gyro_range;
    uint8_t gyro_bw;
} BMI088_Config;

/* ========================== 状态枚举 ======================================== */

typedef enum {
    BMI088_OK            = 0x00U,
    BMI088_ERROR         = 0x01U,
    BMI088_ERR_SPI       = 0x02U,
    BMI088_ERR_CHIP_ID   = 0x03U,
    BMI088_ERR_SELF_TEST = 0x04U,
    BMI088_ERR_TIMEOUT   = 0x05U,
    BMI088_ERR_PARAM     = 0x06U,
    /* Extended error codes for diagnostics */
    BMI088_ERR_ACC_CHIP_ID = 0x10U,  /* Accelerometer CHIP_ID mismatch */
    BMI088_ERR_GYRO_CHIP_ID = 0x11U, /* Gyroscope CHIP_ID mismatch     */
} BMI088_Status;

/* ========================== API 函数声明 ==================================== */

BMI088_Status BMI088_Init(SPI_HandleTypeDef *hspi, const BMI088_Config *config);
BMI088_Status BMI088_DeInit(void);
BMI088_Status BMI088_ReadAccel(BMI088_AxisRaw *ax);
BMI088_Status BMI088_ReadAccelFiltered(BMI088_AxisRaw *ax);
BMI088_Status BMI088_ReadGyro(BMI088_AxisRaw *gx);
BMI088_Status BMI088_ReadAll(BMI088_Data *data);
BMI088_Status BMI088_ReadTemperature(float *temp_c);
/** 加速度计是否出新样本（读 ACC_STATUS 的 DRDY 位；读数据后自动清零） */
BMI088_Status BMI088_AccelDataReady(bool *ready);
BMI088_Status BMI088_SetAccelRange(uint8_t range);
BMI088_Status BMI088_SetGyroRange(uint8_t range);
BMI088_Status BMI088_SelfTest(void);
BMI088_Status BMI088_GyroCalibrate(void);
BMI088_Status BMI088_AccelCalibrate(void);
float BMI088_AccelToMps2(int16_t raw_value);
float BMI088_GyroToDps(int16_t raw_value);
uint16_t BMI088_GetAccelSensitivity(void);
uint16_t BMI088_GetGyroSensitivity(void);

/* Diagnostics */
void BMI088_GetLastChipIDs(uint8_t *acc_id, uint8_t *gyro_id);
void BMI088_GetAccelOffsets(int16_t *ox, int16_t *oy, int16_t *oz);

#define BMI088_GYRO_LPF_ALPHA        0.15f
#define BMI088_ACC_LPF_ALPHA         0.08f   /* 一阶低通 α, fc≈2.55Hz @ 200Hz ODR */

#ifdef __cplusplus
}
#endif

#endif /* __BMI088_H */
