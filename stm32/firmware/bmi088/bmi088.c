/**
 * @file    bmi088.c
 * @brief   BMI088 6-Axis IMU Driver (Hardware SPI1, DMA + Interrupt)
 *
 * @details 本驱动使用 STM32F407VGT6 的硬件 SPI1（DMA + 中断模式）与 BMI088 通信。
 *          两个内部传感器（加速度计 + 陀螺仪）共用 SPI 总线，
 *          通过独立 CS 引脚（PC4 / PC5）区分。
 *
 *          SPI 配置：Mode 0（CPOL=0, CPHA=0），5.25 MHz
 *          DMA：DMA2_Stream0 (RX) / DMA2_Stream3 (TX), 均为 Channel 3
 *
 *          关键流程：
 *            - 加速度计上电默认 I²C 模式，通过 CSB1 上升沿 + SPI 假读切换
 *            - 陀螺仪上电直接进入 SPI 模式（由 PS 引脚电平决定）
 *            - 读取 LSB 时 MSB 被硬件锁存（shadowing），保证多字节原子性
 *            - 读操作使用 HAL_SPI_TransmitReceive_DMA + 中断完成标志同步
 *            - 写操作使用阻塞 HAL_SPI_TransmitReceive（全双工，避免 OVR 污染）
 *            - 加速度计 SPI 读比陀螺仪多一个 dummy 字节
 *
 *          硬件目标：STM32F407VGT6 @ 168 MHz, SPI1 @ 5.25 MHz
 */

/* Includes ------------------------------------------------------------------*/
#include "bmi088.h"
#include <math.h>

/* ========================== 私有常量 ======================================== */

#define BMI088_SPI_TIMEOUT          100U
#define BMI088_STARTUP_DELAY_MS     50U
#define BMI088_POWERUP_DELAY_MS     50U
#define BMI088_ACC_SPI_SWITCH_DELAY 5U  /* 5 ms for reliable I2C→SPI transition */
#define BMI088_GYRO_RESET_DELAY_MS  30U
#define BMI088_GYRO_CHIP_ID_RETRIES 5U

/* ========================== 私有变量 ======================================== */

static uint8_t bmi088_accel_range = BMI088_ACC_RANGE_24G;
static uint8_t bmi088_gyro_range  = BMI088_GYRO_RANGE_2000;

static int16_t gyro_offset_x;
static int16_t gyro_offset_y;
static int16_t gyro_offset_z;
static uint8_t  gyro_calibrated;

static int16_t accel_offset_x;
static int16_t accel_offset_y;
static int16_t accel_offset_z;
static uint8_t  accel_calibrated;

/* ---- 一阶低通滤波状态 ---- */
static float   accel_filt_x, accel_filt_y, accel_filt_z;
static uint8_t accel_filt_init;

static SPI_HandleTypeDef *bmi088_hspi;

/* ---- 诊断：记录最后一次读到的 chip_id ---- */
static uint8_t bmi088_last_acc_chip_id  = 0x00;
static uint8_t bmi088_last_gyro_chip_id = 0x00;

/* ========================== DMA 同步变量 ==================================== */

static volatile uint8_t bmi088_dma_done;
static volatile uint8_t bmi088_dma_error;

/* ========================== HAL SPI 回调重写 ================================ */

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == bmi088_hspi)
    {
        bmi088_dma_done = 1;
    }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == bmi088_hspi)
    {
        bmi088_dma_done  = 1;
        bmi088_dma_error = 1;
    }
}

/* ========================== DMA 等待辅助函数 ================================ */

static BMI088_Status BMI088_DMA_WaitComplete(uint32_t timeout_ms)
{
    uint32_t tickstart = HAL_GetTick();

    while (!bmi088_dma_done)
    {
        if ((HAL_GetTick() - tickstart) > timeout_ms)
        {
            HAL_SPI_DMAStop(bmi088_hspi);
            return BMI088_ERR_TIMEOUT;
        }
    }

    if (bmi088_dma_error)
    {
        return BMI088_ERR_SPI;
    }

    return BMI088_OK;
}

/* ========================== 私有函数声明 ==================================== */

typedef enum {
    BMI088_CS_ACCEL = 0,
    BMI088_CS_GYRO  = 1,
} BMI088_CSType;

static void BMI088_CS_Select(BMI088_CSType sensor);
static void BMI088_CS_Deselect(BMI088_CSType sensor);
static BMI088_Status BMI088_SPI_ReadReg(BMI088_CSType sensor, uint8_t reg,
                                         uint8_t *data, uint8_t len);
static BMI088_Status BMI088_SPI_ReadRegBlocking(BMI088_CSType sensor, uint8_t reg,
                                                 uint8_t *data, uint8_t len);
static BMI088_Status BMI088_SPI_WriteReg(BMI088_CSType sensor, uint8_t reg,
                                          uint8_t data);
static BMI088_Status BMI088_VerifySPI(void);

/* ========================== 片选控制 ======================================== */

static void BMI088_CS_Select(BMI088_CSType sensor)
{
    if (sensor == BMI088_CS_ACCEL)
        BMI088_ACC_CS_CLR();
    else
        BMI088_GYRO_CS_CLR();
}

static void BMI088_CS_Deselect(BMI088_CSType sensor)
{
    if (sensor == BMI088_CS_ACCEL)
        BMI088_ACC_CS_SET();
    else
        BMI088_GYRO_CS_SET();
}

/* ========================== SPI 底层读写 ==================================== */

/**
 * @brief  SPI DMA 读取寄存器（加速度计多一个 dummy 字节）
 */
static BMI088_Status BMI088_SPI_ReadReg(BMI088_CSType sensor, uint8_t reg,
                                         uint8_t *data, uint8_t len)
{
    HAL_StatusTypeDef hal_ret;
    BMI088_Status     ret;
    uint8_t tx_buf[32];
    uint8_t rx_buf[32];
    uint8_t frame_len;
    uint8_t data_offset;

    if (len == 0 || data == NULL)
        return BMI088_ERR_PARAM;

    /* BMI088 加速度计比陀螺仪多一个 dummy 字节 */
    if (sensor == BMI088_CS_ACCEL)
    {
        frame_len   = (uint8_t)(len + 2U);
        data_offset = 2U;
    }
    else
    {
        frame_len   = (uint8_t)(len + 1U);
        data_offset = 1U;
    }

    if (frame_len > sizeof(tx_buf))
        return BMI088_ERR_PARAM;

    tx_buf[0] = reg | 0x80U;
    for (uint8_t i = 1; i < frame_len; i++)
        tx_buf[i] = 0xFFU;

    BMI088_CS_Select(sensor);

    bmi088_dma_done  = 0;
    bmi088_dma_error = 0;

    hal_ret = HAL_SPI_TransmitReceive_DMA(bmi088_hspi, tx_buf, rx_buf, frame_len);
    if (hal_ret != HAL_OK)
    {
        BMI088_CS_Deselect(sensor);
        return BMI088_ERR_SPI;
    }

    ret = BMI088_DMA_WaitComplete(BMI088_SPI_TIMEOUT);

    BMI088_CS_Deselect(sensor);

    if (ret != BMI088_OK)
        return ret;

    for (uint8_t i = 0; i < len; i++)
        data[i] = rx_buf[i + data_offset];

    return BMI088_OK;
}

/**
 * @brief  SPI 阻塞读取寄存器
 * @note   初始化阶段使用阻塞方式，避免把总线/模式问题与 DMA 中断问题混在一起。
 *         BMI088 加速度计读帧为：地址 + dummy + 数据；陀螺仪没有额外 dummy。
 */
static BMI088_Status BMI088_SPI_ReadRegBlocking(BMI088_CSType sensor, uint8_t reg,
                                                 uint8_t *data, uint8_t len)
{
    HAL_StatusTypeDef hal_ret;
    uint8_t tx_buf[32];
    uint8_t rx_buf[32] = {0};
    uint8_t frame_len;
    uint8_t data_offset;

    if ((len == 0U) || (data == NULL))
        return BMI088_ERR_PARAM;

    if (sensor == BMI088_CS_ACCEL)
    {
        frame_len   = (uint8_t)(len + 2U);
        data_offset = 2U;
    }
    else
    {
        frame_len   = (uint8_t)(len + 1U);
        data_offset = 1U;
    }

    if (frame_len > sizeof(tx_buf))
        return BMI088_ERR_PARAM;

    tx_buf[0] = reg | 0x80U;
    for (uint8_t i = 1U; i < frame_len; i++)
        tx_buf[i] = 0xFFU;

    BMI088_CS_Select(sensor);
    hal_ret = HAL_SPI_TransmitReceive(bmi088_hspi, tx_buf, rx_buf, frame_len,
                                      BMI088_SPI_TIMEOUT);
    BMI088_CS_Deselect(sensor);

    if (hal_ret != HAL_OK)
        return BMI088_ERR_SPI;

    for (uint8_t i = 0U; i < len; i++)
        data[i] = rx_buf[i + data_offset];

    return BMI088_OK;
}

/**
 * @brief  SPI 阻塞写入寄存器（全双工）
 */
static BMI088_Status BMI088_SPI_WriteReg(BMI088_CSType sensor, uint8_t reg,
                                          uint8_t data)
{
    HAL_StatusTypeDef hal_ret;
    uint8_t tx_buf[2];
    uint8_t rx_buf[2];

    tx_buf[0] = reg & 0x7FU;
    tx_buf[1] = data;

    BMI088_CS_Select(sensor);

    hal_ret = HAL_SPI_TransmitReceive(bmi088_hspi, tx_buf, rx_buf, 2,
                                       BMI088_SPI_TIMEOUT);

    BMI088_CS_Deselect(sensor);

    if (hal_ret != HAL_OK)
        return BMI088_ERR_SPI;

    HAL_Delay(1);  /* 寄存器写入后同步等待 */

    return BMI088_OK;
}

/* ========================== SPI 配置验证 ==================================== */

static BMI088_Status BMI088_VerifySPI(void)
{
    if (bmi088_hspi->Init.BaudRatePrescaler < SPI_BAUDRATEPRESCALER_16)
    {
        bmi088_hspi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
        if (HAL_SPI_Init(bmi088_hspi) != HAL_OK)
            return BMI088_ERR_SPI;
    }
    if (bmi088_hspi->Init.DataSize != SPI_DATASIZE_8BIT)
        return BMI088_ERR_SPI;
    if (bmi088_hspi->Init.Mode != SPI_MODE_MASTER)
        return BMI088_ERR_SPI;
    return BMI088_OK;
}

/* ========================== 初始化 / 反初始化 =============================== */

BMI088_Status BMI088_Init(SPI_HandleTypeDef *hspi, const BMI088_Config *config)
{
    BMI088_Status ret;
    BMI088_Config default_cfg;
    uint8_t chip_id = 0U;
    uint8_t reg_val;

    if (hspi == NULL) return BMI088_ERR_PARAM;
    bmi088_hspi = hspi;

    ret = BMI088_VerifySPI();
    if (ret != BMI088_OK) return ret;

    if (config == NULL)
    {
        default_cfg.accel_range = BMI088_ACC_RANGE_24G;
        default_cfg.accel_odr   = BMI088_ACC_ODR_100_HZ;
        default_cfg.accel_bwp   = BMI088_ACC_BWP_OSR4;
        default_cfg.gyro_range  = BMI088_GYRO_RANGE_2000;
        default_cfg.gyro_bw     = BMI088_GYRO_BW_100_32;
        config = &default_cfg;
    }

    bmi088_accel_range = config->accel_range;
    bmi088_gyro_range  = config->gyro_range;

    BMI088_CS_Deselect(BMI088_CS_ACCEL);
    BMI088_CS_Deselect(BMI088_CS_GYRO);
    bmi088_last_acc_chip_id  = 0U;
    bmi088_last_gyro_chip_id = 0U;

    /* 驱动自身保证传感器上电稳定，不依赖 LCD 等其他外设带来的偶然延时。 */
    HAL_Delay(BMI088_STARTUP_DELAY_MS);

    /* =================== 加速度计初始化 =================== */

    /* I²C → SPI 模式切换 */
    BMI088_CS_Deselect(BMI088_CS_ACCEL);
    HAL_Delay(1);

    {
        uint8_t dummy_id;

        /* 完整假读帧：地址 + dummy + 数据。返回值本身在切换期间无效。 */
        ret = BMI088_SPI_ReadRegBlocking(BMI088_CS_ACCEL,
                                         BMI088_ACC_CHIP_ID, &dummy_id, 1U);
        if (ret != BMI088_OK) return ret;
    }
    /* 上面的阻塞读取结束时 CSB1 上升沿使加速度计进入 SPI 模式。 */
    HAL_Delay(BMI088_ACC_SPI_SWITCH_DELAY);

    /* 验证 CHIP_ID (带重试) */
    {
        uint8_t acc_retry;
        for (acc_retry = 0; acc_retry < BMI088_GYRO_CHIP_ID_RETRIES; acc_retry++)
        {
            ret = BMI088_SPI_ReadRegBlocking(BMI088_CS_ACCEL,
                                              BMI088_ACC_CHIP_ID, &chip_id, 1U);
            bmi088_last_acc_chip_id = chip_id;  /* 保存诊断信息 */
            if ((ret == BMI088_OK) && (chip_id == BMI088_ACC_CHIP_ID_VAL))
                break;
            HAL_Delay(1);
        }
        if (ret != BMI088_OK) return ret;
        if (chip_id != BMI088_ACC_CHIP_ID_VAL)
        {
            bmi088_last_acc_chip_id = chip_id;
            return BMI088_ERR_ACC_CHIP_ID;
        }
    }

    /* 使能加速度计 */
    ret = BMI088_SPI_WriteReg(BMI088_CS_ACCEL, BMI088_ACC_PWR_CTRL,
                               BMI088_ACC_PWR_CTRL_ACTIVE);
    if (ret != BMI088_OK) return ret;
    HAL_Delay(BMI088_POWERUP_DELAY_MS);

    /* PWR_CONF */
    ret = BMI088_SPI_WriteReg(BMI088_CS_ACCEL, BMI088_ACC_PWR_CONF,
                               BMI088_ACC_PWR_CONF_ACTIVE);
    if (ret != BMI088_OK) return ret;

    /* ACC_CONF */
    reg_val = (config->accel_bwp << BMI088_ACC_CONF_BWP_Pos)
            | (config->accel_odr << BMI088_ACC_CONF_ODR_Pos);
    ret = BMI088_SPI_WriteReg(BMI088_CS_ACCEL, BMI088_ACC_CONF, reg_val);
    if (ret != BMI088_OK) return ret;

    /* ACC_RANGE */
    reg_val = config->accel_range & BMI088_ACC_RANGE_Msk;
    ret = BMI088_SPI_WriteReg(BMI088_CS_ACCEL, BMI088_ACC_RANGE, reg_val);
    if (ret != BMI088_OK) return ret;

    /* =================== 陀螺仪初始化 =================== */

    for (uint8_t retry = 0; retry < BMI088_GYRO_CHIP_ID_RETRIES; retry++)
    {
        ret = BMI088_SPI_ReadRegBlocking(BMI088_CS_GYRO,
                                         BMI088_GYRO_CHIP_ID, &chip_id, 1U);
        bmi088_last_gyro_chip_id = chip_id;
        if ((ret == BMI088_OK) && (chip_id == BMI088_GYRO_CHIP_ID_VAL))
            break;
        HAL_Delay(1);
    }
    if (ret != BMI088_OK) return ret;
    if (chip_id != BMI088_GYRO_CHIP_ID_VAL)
    {
        bmi088_last_gyro_chip_id = chip_id;
        return BMI088_ERR_GYRO_CHIP_ID;
    }

    reg_val = config->gyro_range & BMI088_GYRO_RANGE_Msk;
    ret = BMI088_SPI_WriteReg(BMI088_CS_GYRO, BMI088_GYRO_RANGE, reg_val);
    if (ret != BMI088_OK) return ret;

    reg_val = config->gyro_bw & BMI088_GYRO_BW_Msk;
    ret = BMI088_SPI_WriteReg(BMI088_CS_GYRO, BMI088_GYRO_BANDWIDTH, reg_val);
    if (ret != BMI088_OK) return ret;

    ret = BMI088_SPI_WriteReg(BMI088_CS_GYRO, BMI088_GYRO_LPM1,
                               BMI088_GYRO_LPM1_NORMAL);
    if (ret != BMI088_OK) return ret;

    BMI088_GyroCalibrate();

    /* 开机不做加速度标定：它会把"开机那一刻"的 X/Y 重力分量和 Z 的 1 g
     * 一起当零偏减掉，底盘姿态一变数据就整体偏移，且丢掉重力。
     * 移动底盘要的是含重力的 raw，故此处只保留陀螺零偏标定。 */
    accel_calibrated = 0;

    return BMI088_OK;
}

BMI088_Status BMI088_DeInit(void)
{
    BMI088_SPI_WriteReg(BMI088_CS_ACCEL, BMI088_ACC_PWR_CTRL,
                         BMI088_ACC_PWR_CTRL_SUSPEND);
    BMI088_SPI_WriteReg(BMI088_CS_GYRO, BMI088_GYRO_LPM1,
                         BMI088_GYRO_LPM1_DEEP_SUSP | BMI088_GYRO_LPM1_SUSPEND);
    BMI088_CS_Deselect(BMI088_CS_ACCEL);
    BMI088_CS_Deselect(BMI088_CS_GYRO);
    return BMI088_OK;
}

/* ========================== 数据读取 ======================================== */

BMI088_Status BMI088_ReadAccel(BMI088_AxisRaw *ax)
{
    BMI088_Status ret;
    uint8_t buf[6];

    if (ax == NULL) return BMI088_ERR_PARAM;

    ret = BMI088_SPI_ReadReg(BMI088_CS_ACCEL, BMI088_ACC_X_LSB, buf, 6);
    if (ret != BMI088_OK) return ret;

    ax->x = (int16_t)(((uint16_t)buf[1] << 8) | (uint16_t)buf[0]);
    ax->y = (int16_t)(((uint16_t)buf[3] << 8) | (uint16_t)buf[2]);
    ax->z = (int16_t)(((uint16_t)buf[5] << 8) | (uint16_t)buf[4]);

    if (accel_calibrated)
    {
        ax->x -= accel_offset_x;
        ax->y -= accel_offset_y;
        ax->z -= accel_offset_z;
    }

    return BMI088_OK;
}

/**
 * @brief  读取加速度计并应用一阶低通滤波
 * @param  ax  输出：滤波后的三轴加速度原始值（偏移已校准）
 * @note   滤波器采用 EMA: y[n] = α·x[n] + (1-α)·y[n-1]
 *          α = BMI088_ACC_LPF_ALPHA (0.08), fc ≈ 2.55 Hz @ 200 Hz ODR
 *          首次采样自动初始化滤波器状态（无阶跃响应）
 */
BMI088_Status BMI088_ReadAccelFiltered(BMI088_AxisRaw *ax)
{
    BMI088_Status ret;
    BMI088_AxisRaw raw;

    if (ax == NULL) return BMI088_ERR_PARAM;

    /* 读取已校准原始值 */
    ret = BMI088_ReadAccel(&raw);
    if (ret != BMI088_OK) return ret;

    if (!accel_filt_init)
    {
        /* 首次采样：直接初始化，避免阶跃 */
        accel_filt_x = (float)raw.x;
        accel_filt_y = (float)raw.y;
        accel_filt_z = (float)raw.z;
        accel_filt_init = 1;
    }
    else
    {
        accel_filt_x = BMI088_ACC_LPF_ALPHA * (float)raw.x
                     + (1.0f - BMI088_ACC_LPF_ALPHA) * accel_filt_x;
        accel_filt_y = BMI088_ACC_LPF_ALPHA * (float)raw.y
                     + (1.0f - BMI088_ACC_LPF_ALPHA) * accel_filt_y;
        accel_filt_z = BMI088_ACC_LPF_ALPHA * (float)raw.z
                     + (1.0f - BMI088_ACC_LPF_ALPHA) * accel_filt_z;
    }

    ax->x = (int16_t)accel_filt_x;
    ax->y = (int16_t)accel_filt_y;
    ax->z = (int16_t)accel_filt_z;

    return BMI088_OK;
}

BMI088_Status BMI088_ReadGyro(BMI088_AxisRaw *gx)
{
    BMI088_Status ret;
    uint8_t buf[6];

    if (gx == NULL) return BMI088_ERR_PARAM;

    ret = BMI088_SPI_ReadReg(BMI088_CS_GYRO, BMI088_GYRO_RATE_X_LSB, buf, 6);
    if (ret != BMI088_OK) return ret;

    gx->x = (int16_t)(((uint16_t)buf[1] << 8) | (uint16_t)buf[0]);
    gx->y = (int16_t)(((uint16_t)buf[3] << 8) | (uint16_t)buf[2]);
    gx->z = (int16_t)(((uint16_t)buf[5] << 8) | (uint16_t)buf[4]);

    if (gyro_calibrated)
    {
        gx->x -= gyro_offset_x;
        gx->y -= gyro_offset_y;
        gx->z -= gyro_offset_z;
    }

    return BMI088_OK;
}

BMI088_Status BMI088_GyroCalibrate(void)
{
    BMI088_Status ret;
    BMI088_AxisRaw raw;
    int16_t min_x, max_x, min_y, max_y, min_z, max_z;
    int32_t sum_x, sum_y, sum_z;

#define GYRO_CAL_SAMPLES    200
#define GYRO_CAL_STABLE_LSB 5
#define GYRO_CAL_RETRIES    3

    for (uint8_t attempt = 0; attempt < GYRO_CAL_RETRIES; attempt++)
    {
        sum_x = 0; sum_y = 0; sum_z = 0;
        min_x = 32767; max_x = -32768;
        min_y = 32767; max_y = -32768;
        min_z = 32767; max_z = -32768;

        for (uint16_t i = 0; i < GYRO_CAL_SAMPLES; i++)
        {
            ret = BMI088_ReadGyro(&raw);
            if (ret != BMI088_OK) return ret;

            sum_x += raw.x;
            sum_y += raw.y;
            sum_z += raw.z;

            if (raw.x < min_x) min_x = raw.x;
            if (raw.x > max_x) max_x = raw.x;
            if (raw.y < min_y) min_y = raw.y;
            if (raw.y > max_y) max_y = raw.y;
            if (raw.z < min_z) min_z = raw.z;
            if (raw.z > max_z) max_z = raw.z;
        }

        if (((max_x - min_x) < GYRO_CAL_STABLE_LSB) &&
            ((max_y - min_y) < GYRO_CAL_STABLE_LSB) &&
            ((max_z - min_z) < GYRO_CAL_STABLE_LSB))
        {
            gyro_offset_x = (int16_t)((float)sum_x / (float)GYRO_CAL_SAMPLES);
            gyro_offset_y = (int16_t)((float)sum_y / (float)GYRO_CAL_SAMPLES);
            gyro_offset_z = (int16_t)((float)sum_z / (float)GYRO_CAL_SAMPLES);
            gyro_calibrated = 1;
            return BMI088_OK;
        }
    }

    gyro_offset_x = 0;
    gyro_offset_y = 0;
    gyro_offset_z = 0;
    gyro_calibrated = 1;
    return BMI088_OK;
}

/**
 * @brief  加速度计零点校准（上电静止校准）
 * @note   设备必须水平静止放置（Z 轴朝上）。
 *         按 200 Hz ODR 间隔（5 ms）采集 500 个独立样本（2.5 秒）。
 *         使用加速度模长（±0.2g 容差）判断设备是否静止。
 *         X/Y 偏移 = 平均值；Z 偏移 = 平均值 - 1g_LSB。
 * @retval BMI088_OK        校准成功
 * @retval BMI088_ERROR     设备未静止，校准失败
 */
BMI088_Status BMI088_AccelCalibrate(void)
{
    BMI088_Status ret;
    BMI088_AxisRaw raw;
    int16_t min_x, max_x, min_y, max_y, min_z, max_z;
    int32_t sum_x, sum_y, sum_z;
    int16_t one_g;
    float   mag_tol;
    uint8_t motion_detected;

#define ACCEL_CAL_SAMPLES      500   /* 独立样本数 (2.5s @ 5ms) */
#define ACCEL_CAL_DELAY_MS     5     /* 匹配 200 Hz ODR           */
#define ACCEL_CAL_PP_THRESH    200   /* 峰峰值阈值 (LSB), 宽松    */
#define ACCEL_CAL_MAG_TOL_G    0.20f /* 模长容差 (g), ±0.2g      */
#define ACCEL_CAL_RETRIES      2     /* 重试次数                  */

    one_g   = (int16_t)BMI088_GetAccelSensitivity();
    mag_tol = ACCEL_CAL_MAG_TOL_G * (float)one_g;

    /* ---- 等待传感器和 OSR4 滤波器稳定 ---- */
    HAL_Delay(100);

    for (uint8_t attempt = 0; attempt < ACCEL_CAL_RETRIES; attempt++)
    {
        sum_x = 0; sum_y = 0; sum_z = 0;
        min_x = 32767; max_x = -32768;
        min_y = 32767; max_y = -32768;
        min_z = 32767; max_z = -32768;
        motion_detected = 0;

        for (uint16_t i = 0; i < ACCEL_CAL_SAMPLES; i++)
        {
            /* 直接读 SPI 寄存器原始值（绕过 ReadAccel 的 offset 应用） */
            {
                uint8_t buf[6];
                ret = BMI088_SPI_ReadReg(BMI088_CS_ACCEL,
                                          BMI088_ACC_X_LSB, buf, 6);
                if (ret != BMI088_OK) return ret;

                raw.x = (int16_t)(((uint16_t)buf[1] << 8)
                                  | (uint16_t)buf[0]);
                raw.y = (int16_t)(((uint16_t)buf[3] << 8)
                                  | (uint16_t)buf[2]);
                raw.z = (int16_t)(((uint16_t)buf[5] << 8)
                                  | (uint16_t)buf[4]);
            }

            /* ---- 模长检查：是否偏离 1g ---- */
            {
                float mag = sqrtf((float)raw.x * (float)raw.x
                                + (float)raw.y * (float)raw.y
                                + (float)raw.z * (float)raw.z);
                if (fabsf(mag - (float)one_g) > mag_tol)
                {
                    motion_detected = 1;
                }
            }

            sum_x += raw.x;
            sum_y += raw.y;
            sum_z += raw.z;

            if (raw.x < min_x) min_x = raw.x;
            if (raw.x > max_x) max_x = raw.x;
            if (raw.y < min_y) min_y = raw.y;
            if (raw.y > max_y) max_y = raw.y;
            if (raw.z < min_z) min_z = raw.z;
            if (raw.z > max_z) max_z = raw.z;

            /* 按 ODR 速率等待下一个独立样本 */
            HAL_Delay(ACCEL_CAL_DELAY_MS);
        }

        /* ---- 静止判断：无剧烈运动 + 峰峰值在容差内 ---- */
        if (!motion_detected
            && ((max_x - min_x) < ACCEL_CAL_PP_THRESH)
            && ((max_y - min_y) < ACCEL_CAL_PP_THRESH)
            && ((max_z - min_z) < ACCEL_CAL_PP_THRESH))
        {
            accel_offset_x = (int16_t)((float)sum_x
                                       / (float)ACCEL_CAL_SAMPLES);
            accel_offset_y = (int16_t)((float)sum_y
                                       / (float)ACCEL_CAL_SAMPLES);
            /* Z 轴扣除 1g 重力加速度 */
            accel_offset_z = (int16_t)((float)sum_z
                                       / (float)ACCEL_CAL_SAMPLES)
                           - one_g;

            /* 重置低通滤波器状态 */
            accel_filt_x    = 0.0f;
            accel_filt_y    = 0.0f;
            accel_filt_z    = 0.0f;
            accel_filt_init = 0;

            accel_calibrated = 1;
            return BMI088_OK;
        }
    }

    /* ---- 校准失败：offset 清零，标记未校准，返回错误 ---- */
    accel_offset_x   = 0;
    accel_offset_y   = 0;
    accel_offset_z   = 0;
    accel_calibrated = 0;
    return BMI088_ERROR;
}

BMI088_Status BMI088_ReadAll(BMI088_Data *data)
{
    BMI088_Status ret;

    if (data == NULL) return BMI088_ERR_PARAM;

    ret = BMI088_ReadAccel(&data->accel);
    if (ret != BMI088_OK) return ret;

    ret = BMI088_ReadGyro(&data->gyro);
    if (ret != BMI088_OK) return ret;

    {
        uint8_t buf[2];
        ret = BMI088_SPI_ReadReg(BMI088_CS_ACCEL, BMI088_ACC_TEMP_MSB, buf, 2);
        if (ret == BMI088_OK)
        {
            data->temperature = (int16_t)(((uint16_t)buf[0] << 3)
                                         | ((uint16_t)buf[1] >> 5));
            if (data->temperature & 0x0400)
                data->temperature |= 0xF800;
        }
    }

    return ret;
}

BMI088_Status BMI088_AccelDataReady(bool *ready)
{
    uint8_t        status = 0U;
    BMI088_Status  ret;

    if (ready == NULL) return BMI088_ERR_PARAM;

    ret = BMI088_SPI_ReadReg(BMI088_CS_ACCEL, BMI088_ACC_STATUS, &status, 1U);
    if (ret != BMI088_OK) return ret;

    *ready = ((status & BMI088_ACC_STATUS_DRDY) != 0U);
    return BMI088_OK;
}

BMI088_Status BMI088_ReadTemperature(float *temp_c)
{
    BMI088_Status ret;
    uint8_t buf[2];
    int16_t raw;

    if (temp_c == NULL) return BMI088_ERR_PARAM;

    ret = BMI088_SPI_ReadReg(BMI088_CS_ACCEL, BMI088_ACC_TEMP_MSB, buf, 2);
    if (ret != BMI088_OK) return ret;

    raw = (int16_t)(((uint16_t)buf[0] << 3) | ((uint16_t)buf[1] >> 5));
    if (raw & 0x0400) raw |= 0xF800;

    *temp_c = (float)raw * 0.125f + 23.0f;

    return BMI088_OK;
}

/* ========================== 动态配置 ======================================== */

BMI088_Status BMI088_SetAccelRange(uint8_t range)
{
    BMI088_Status ret;

    if (range > BMI088_ACC_RANGE_24G) return BMI088_ERR_PARAM;

    ret = BMI088_SPI_WriteReg(BMI088_CS_ACCEL, BMI088_ACC_RANGE,
                               range & BMI088_ACC_RANGE_Msk);
    if (ret == BMI088_OK) bmi088_accel_range = range;

    return ret;
}

BMI088_Status BMI088_SetGyroRange(uint8_t range)
{
    BMI088_Status ret;

    if (range > BMI088_GYRO_RANGE_125) return BMI088_ERR_PARAM;

    ret = BMI088_SPI_WriteReg(BMI088_CS_GYRO, BMI088_GYRO_RANGE,
                               range & BMI088_GYRO_RANGE_Msk);
    if (ret == BMI088_OK) bmi088_gyro_range = range;

    return ret;
}

/* ========================== 自检 ============================================ */

static BMI088_Status BMI088_AccelSelfTest(void)
{
    BMI088_Status  ret;
    BMI088_AxisRaw ref, test;
    float          diff, expected_min;

    HAL_Delay(10);
    ret = BMI088_ReadAccel(&ref);
    if (ret != BMI088_OK) return ret;

    ret = BMI088_SPI_WriteReg(BMI088_CS_ACCEL, BMI088_ACC_SELF_TEST,
                               BMI088_ACC_SELF_TEST_EN);
    if (ret != BMI088_OK) return ret;
    HAL_Delay(50);

    ret = BMI088_ReadAccel(&test);
    BMI088_SPI_WriteReg(BMI088_CS_ACCEL, BMI088_ACC_SELF_TEST,
                         BMI088_ACC_SELF_TEST_DIS);
    if (ret != BMI088_OK) return ret;

    diff = (float)(test.z - ref.z);
    if (diff < 0.0f) diff = -diff;
    expected_min = (float)BMI088_ACC_SENS_24G * 0.5f;

    if (diff < expected_min) return BMI088_ERR_SELF_TEST;
    return BMI088_OK;
}

static BMI088_Status BMI088_GyroSelfTest(void)
{
    BMI088_Status ret;
    uint8_t chip_id;

    ret = BMI088_SPI_WriteReg(BMI088_CS_GYRO, BMI088_GYRO_SOFTRESET,
                               BMI088_GYRO_RESET_CMD);
    if (ret != BMI088_OK) return ret;
    HAL_Delay(BMI088_GYRO_RESET_DELAY_MS);

    ret = BMI088_SPI_ReadReg(BMI088_CS_GYRO, BMI088_GYRO_CHIP_ID, &chip_id, 1);
    if (ret != BMI088_OK) return ret;
    if (chip_id != BMI088_GYRO_CHIP_ID_VAL) return BMI088_ERR_SELF_TEST;

    {
        uint8_t reg_val;
        reg_val = bmi088_gyro_range & BMI088_GYRO_RANGE_Msk;
        BMI088_SPI_WriteReg(BMI088_CS_GYRO, BMI088_GYRO_RANGE, reg_val);
        reg_val = BMI088_GYRO_BW_100_32 & BMI088_GYRO_BW_Msk;
        BMI088_SPI_WriteReg(BMI088_CS_GYRO, BMI088_GYRO_BANDWIDTH, reg_val);
        BMI088_SPI_WriteReg(BMI088_CS_GYRO, BMI088_GYRO_LPM1,
                             BMI088_GYRO_LPM1_NORMAL);
    }

    return BMI088_OK;
}

BMI088_Status BMI088_SelfTest(void)
{
    BMI088_Status ret;
    ret = BMI088_AccelSelfTest();
    if (ret != BMI088_OK) return ret;
    ret = BMI088_GyroSelfTest();
    return ret;
}

/* ========================== 诊断 API ======================================== */

void BMI088_GetLastChipIDs(uint8_t *acc_id, uint8_t *gyro_id)
{
    if (acc_id  != NULL) *acc_id  = bmi088_last_acc_chip_id;
    if (gyro_id != NULL) *gyro_id = bmi088_last_gyro_chip_id;
}

void BMI088_GetAccelOffsets(int16_t *ox, int16_t *oy, int16_t *oz)
{
    if (ox != NULL) *ox = accel_offset_x;
    if (oy != NULL) *oy = accel_offset_y;
    if (oz != NULL) *oz = accel_offset_z;
}

/* ========================== 单位转换 ======================================== */

uint16_t BMI088_GetAccelSensitivity(void)
{
    switch (bmi088_accel_range)
    {
        case BMI088_ACC_RANGE_3G:  return BMI088_ACC_SENS_3G;
        case BMI088_ACC_RANGE_6G:  return BMI088_ACC_SENS_6G;
        case BMI088_ACC_RANGE_12G: return BMI088_ACC_SENS_12G;
        default:                   return BMI088_ACC_SENS_24G;
    }
}

uint16_t BMI088_GetGyroSensitivity(void)
{
    switch (bmi088_gyro_range)
    {
        case BMI088_GYRO_RANGE_125:  return BMI088_GYRO_SENS_125;
        case BMI088_GYRO_RANGE_250:  return BMI088_GYRO_SENS_250;
        case BMI088_GYRO_RANGE_500:  return BMI088_GYRO_SENS_500;
        case BMI088_GYRO_RANGE_1000: return BMI088_GYRO_SENS_1000;
        default:                     return BMI088_GYRO_SENS_2000;
    }
}

float BMI088_AccelToMps2(int16_t raw_value)
{
    float sens = (float)BMI088_GetAccelSensitivity();
    return (float)raw_value / sens * 9.80665f;
}

float BMI088_GyroToDps(int16_t raw_value)
{
    float sens = (float)BMI088_GetGyroSensitivity();
    return (float)raw_value / sens;
}
