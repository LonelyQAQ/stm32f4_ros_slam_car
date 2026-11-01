/**
 * @file    uros_transport.h
 * @brief   micro-ROS 自定义传输层（挂在 USART1 上）
 *
 * 调用顺序：app_main 里先 uros_transport_install()，再初始化 rcl，
 * 否则 rcl 会用默认的串口 transport 去找 USART，这里没有。
 */
#ifndef UROS_TRANSPORT_H
#define UROS_TRANSPORT_H

#include <stdbool.h>

/** 注册自定义 transport（framing = true，带 HDLC 帧与 CRC） */
bool uros_transport_install(void);

#endif /* UROS_TRANSPORT_H */
