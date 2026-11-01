/**
 * @file    link.h
 * @brief   上位机链路统一接口（三选一实现，编译期决定）
 *
 *   APP_ENABLE_UROS=1  → uros_app.c     （micro-ROS over USART1）
 *   APP_ENABLE_FRAME=1 → frame_link.c   （自定义帧 + Jetson 侧 Python 桥接）
 *   都为 0              → uros_stub.c    （硬件自检）
 *
 * 上层（app_main）只认这三个函数，换链路不用改控制逻辑。
 */
#ifndef LINK_H
#define LINK_H

#include <stdbool.h>

/** 主循环每圈调用：收发、解析、发布 */
void link_loop(void);

/** 取最近一条速度指令（v: m/s，w: rad/s）；无有效指令返回 false */
bool link_get_cmd(float *v, float *w);

/** 链路是否就绪；false 时上层一律停机 */
bool link_ready(void);

/** 是否请求制动（急停 / 避障生效）；超时或链路断则自动为 false → 滑行 */
bool link_get_brake(void);

#endif /* LINK_H */
