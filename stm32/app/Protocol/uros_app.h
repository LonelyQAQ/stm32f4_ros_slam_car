/**
 * @file    uros_app.h
 * @brief   micro-ROS 节点：实体初始化、发布调度、cmd_vel 接收
 *
 * 状态机放在这里：WAIT_AGENT → CONNECTED → LOST → 回到 WAIT_AGENT。
 * 掉线不重启 MCU，只销毁实体重连。
 */
#ifndef UROS_APP_H
#define UROS_APP_H

#include <stdbool.h>

typedef enum {
    UROS_STATE_WAIT_AGENT = 0,
    UROS_STATE_CONNECTED,
} uros_state_t;

/** 主循环每圈调用：推进状态机、spin executor、到点发布 */
void uros_app_loop(void);

/** 取最近一条 cmd_vel（v: m/s，w: rad/s）；无有效指令时返回 false */
bool uros_app_get_cmd(float *v, float *w);


extern volatile uros_state_t g_uros_state;
extern volatile uint32_t g_uros_reconnects;
extern volatile uint32_t g_uros_pub_errors;
extern volatile uint32_t g_cmd_count;

#endif /* UROS_APP_H */
