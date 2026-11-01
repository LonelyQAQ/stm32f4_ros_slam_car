/**
 * @file    microros_app_includes.h
 * @brief   工程唯一引用入口：所有 .c 只 include 本文件 + 自己的 .h
 *
 * 层序（只允许自上而下）：System → Protocol → Device → 驱动/CubeMX 生成代码
 */
#ifndef MICROROS_APP_INCLUDES_H
#define MICROROS_APP_INCLUDES_H

#include "main.h"

/* ---- 配置（全工程唯一 *_cfg.h） ---- */
#include "app_cfg.h"

/* ---- 板级参数（引脚/定时器/机械尺寸） ---- */
#include "board_cfg.h"

/* ---- Device ---- */
#include "dev_uart.h"
#include "dev_imu.h"
#include "dev_attitude.h"
#include "motor.h"
#include "encoder.h"
#include "bmi088.h"
#include "vqf.h"

/* ---- Protocol ---- */
#include "link.h"
#include "uros_transport.h"
#include "uros_app.h"

/* ---- System ---- */
#include "app_main.h"

#endif /* MICROROS_APP_INCLUDES_H */
