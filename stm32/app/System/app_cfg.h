/**
 * @file    app_cfg.h
 * @brief   micro-ROS 移植的全部可调参数（全工程唯一 *_cfg.h）
 *
 * 改参数只改这里；源码里不允许出现裸数字。
 * 对应 CubeMX 工程：ROS_STM32_Test（F407VGT6 / Keil AC6 / HAL F4 1.28.3）
 */
#ifndef APP_CFG_H
#define APP_CFG_H

#include "main.h"
#include "bmi088.h"   /* 量程宏 BMI088_GYRO_RANGE_xxx 在这里 */

/* ===================== 功能开关 ===================== */
/* 1 = 带 micro-ROS；0 = 纯硬件自检固件（电机/编码器/IMU 验证，不依赖 ros 源码）
 * 自检模式上电等 3 秒后：正转 1.5 s → 停 1 s → 反转 1.5 s → 停，并周期性打印计数。 */
#define APP_ENABLE_UROS         0
/* 1 = 自定义帧链路（配 Jetson 侧 frame_bridge.py，不依赖 micro-ROS 源码） */
#define APP_ENABLE_FRAME        1

/* ===================== 通信 ===================== */
#define UROS_UART_HANDLE        huart1
#define UROS_BAUD               921600U

/* 接收用 DMA 圆形缓冲，写指针由 DMA 自己走；读侧追尾指针。
 * 1024 字节 ≈ 11 ms @921600，够一帧 XRCE 报文。 */
#define UROS_RX_DMA_SIZE        1024U

/* 发送是 DMA Normal + 忙等完成。最大帧约 800 B，留 2 倍余量。 */
#define UROS_TX_BUF_SIZE        2048U
#define UROS_TX_TIMEOUT_MS      50U

/* ===================== 话题 ===================== */
#define UROS_NODE_NAME          "slam_car_mcu"
#define UROS_TOPIC_ODOM         "/odom"
#define UROS_TOPIC_IMU          "/imu/data_raw"
#define UROS_TOPIC_WHEEL        "/wheel_speed"
#define UROS_TOPIC_CMD_VEL      "/cmd_vel"

#define UROS_FRAME_ODOM         "odom"
#define UROS_FRAME_BASE         "base_link"
#define UROS_FRAME_IMU          "imu_link"

/* ===================== 发布频率 =====================
 * 带宽预算（921600 ≈ 92 KB/s）：
 *   Odometry 含两个 36 字节协方差矩阵，约 800 B/帧
 *   Imu      含三个协方差矩阵，约 350 B/帧
 *   30 Hz × 800 + 50 Hz × 350 ≈ 41.5 KB/s，占 45%，留给 XRCE 心跳还有余量。
 * 想上 100 Hz IMU 必须先把 odom 降到 20 Hz 或换 1.5 Mbps 链路。 */
#define UROS_ODOM_HZ            30U
#define UROS_IMU_HZ             50U
#define UROS_WHEEL_HZ           20U

/* ===================== 运动学 ===================== */
#define APP_V_MAX_MPS           0.35f   /* 满占空比对应的线速度，标定后改 */
#define APP_W_MAX_RPS           1.20f   /* 满占空比对应的角速度，rad/s */

/* ===================== 看门狗 ===================== */
#define APP_CMD_TIMEOUT_MS      300U    /* 超过此时间没有 /cmd_vel → 滑行停车 */
/* 制动只由"急停/避障"触发：每帧重新续期，超过这个时间没再收到制动请求就回滑行，
 * 防止 Jetson 掉线后 H 桥一直短路发热。 */
#define APP_BRAKE_HOLD_MS       500U
#define APP_LINK_TIMEOUT_MS     1000U   /* micro-ROS 会话超时 → 停车并重连 */
#define APP_IMU_FAIL_LIMIT      10U     /* 连续读失败次数 → 停发 /imu */

/* ===================== IMU ===================== */
#define APP_IMU_ODR_HZ          200U    /* 驱动配置的 ODR，读取频率不能超过它 */
#define APP_GYRO_RANGE          BMI088_GYRO_RANGE_1000   /* ±1000 dps */
#define APP_GYRO_LSB_PER_DPS    32.768f                  /* 对应上行的灵敏度 */
#define APP_ACC_LSB_PER_G       5461.0f                  /* ±6g */
#define APP_G               9.80665f

/* ===================== 姿态解算（VQF 6D） ===================== */
/* 采样由 ACC_STATUS.DRDY 驱动，Ts 必须与 APP_IMU_ODR_HZ 对齐 */
#define APP_VQF_TS              0.005f
/* 车体系 x 前 / y 左 / z 上；下面写的是"车体这一轴取自传感器哪一轴"：
 *   +1/-1 = ±X，+2/-2 = ±Y，+3/-3 = ±Z
 * 实测（2026-10-31 二次确认）：车体正方向是 BMI088 的 -Y 轴，Z 朝上。
 * 保持右手系：车体前←传感器 -Y，车体左←传感器 +X，车体上←+Z。
 * 此前按 +Y 前向的记录作废。 */
#define APP_IMU_BODY_X          (-2)    /* 车体前 ← 传感器 -Y */
#define APP_IMU_BODY_Y          (+1)    /* 车体左 ← 传感器 +X */
#define APP_IMU_BODY_Z          (+3)    /* 车体上 ← 传感器 +Z */

/* ---- 零速修正（ZUPT）：轮子不动时把陀螺 z 零偏重新钉死 ----
 * 判据只看编码器：线速度、角速度都低于阈值并持续一段时间才认为静止。
 * 注意：车被人抱着走、轮子不转时会被误判为静止，这是已知局限。 */
#define IMU_ZUPT_V_MPS          0.02f   /* m/s，轮速低于它算静止 */
#define IMU_ZUPT_W_RPS          0.05f   /* rad/s */
#define IMU_ZUPT_HOLD_S         0.5f    /* 连续静止多久才修正 */
#define IMU_ZUPT_MIN_SAMPLES    20U     /* 至少攒够这么多样本再求均值 */
#define IMU_ZUPT_BIAS_SIGMA     0.002f  /* 钉住零偏时给 VQF 的不确定度 */
#define IMU_ZUPT_GYRO_MAX       0.035f  /* rad/s ≈2°/s：三轴都要低于它才算"真静止"，
                                         * 防止手转车（轮子打滑不转）被当成静止而把角速度钉成零偏 */

/* ===================== 协方差（EKF 要用，不能给 0） ===================== */
#define APP_COV_ODOM_POS        1.0e-3f
#define APP_COV_ODOM_YAW        1.0e-2f
#define APP_COV_ODOM_VEL        1.0e-2f
#define APP_COV_ODOM_W          1.0e-2f
#define APP_COV_IMU_GYRO        2.0e-4f
#define APP_COV_IMU_ACC         4.0e-2f

/* ===================== 控制节拍 ===================== */
/* 控制环跑在 TIM12 更新中断里。dt 由 HAL_GetTick 实测，不依赖 TIM12 的周期，
 * 所以 Period 是 1 ms 还是 2 ms 都不会导致速度算错。 */
#define APP_TICK_DT_MIN         0.0005f
#define APP_TICK_DT_MAX         0.0200f

#endif /* APP_CFG_H */
