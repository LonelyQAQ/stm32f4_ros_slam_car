/**
 * @file    uros_stub.c
 * @brief   APP_ENABLE_UROS = 0 时的硬件自检实现
 *
 * 不依赖任何 micro-ROS 源码，用来单独验证：
 *   电源 → TIM12 节拍 → 编码器计数 → TB6612 转向 → BMI088 出数 → USART1 收发
 *
 * 上电动作序列（3 s 延时是留给你按电源/扶稳的时间）：
 *   等待 3 s → 正转 1.5 s（30% 占空比）→ 停 1 s → 反转 1.5 s → 停车
 * 之后每 2 s 通过 USART1 打印一次：节拍数、左右轮累计计数、轮速。
 */
#include "microros_app_includes.h"
#include <stdio.h>
#include <string.h>

#if !APP_ENABLE_UROS && !APP_ENABLE_FRAME

extern UART_HandleTypeDef UROS_UART_HANDLE;

volatile uros_state_t g_uros_state      = UROS_STATE_WAIT_AGENT;
volatile uint32_t     g_uros_reconnects = 0;
volatile uint32_t     g_uros_pub_errors = 0;
volatile uint32_t     g_cmd_count       = 0;

#define TEST_DUTY   ((int16_t)(MOTOR_PWM_ARR * 30U / 100U))

typedef enum {
    TEST_IDLE = 0,
    TEST_FWD,
    TEST_GAP,
    TEST_BWD,
    TEST_DONE
} test_stage_t;

static test_stage_t s_stage = TEST_IDLE;
static uint32_t s_stage_ms;
static uint32_t s_print_ms;

/* 用阻塞发送，自检阶段 USART1 没跑 DMA 收发，不会互相踩 */
static void dbg(const char *s)
{
    HAL_UART_Transmit(&UROS_UART_HANDLE, (uint8_t *)s, (uint16_t)strlen(s), 100);
}

static void enter(test_stage_t st, uint32_t now)
{
    s_stage    = st;
    s_stage_ms = now;
}

void link_loop(void)
{
    uint32_t now = HAL_GetTick();

    switch (s_stage) {
    case TEST_IDLE:
        if (now > 3000U) {
            dbg("[test] forward\r\n");
            motor_set(TEST_DUTY, TEST_DUTY);
            enter(TEST_FWD, now);
        }
        break;

    case TEST_FWD:
        if ((now - s_stage_ms) > 1500U) {
            dbg("[test] stop\r\n");
            motor_stop();
            enter(TEST_GAP, now);
        }
        break;

    case TEST_GAP:
        if ((now - s_stage_ms) > 1000U) {
            dbg("[test] backward\r\n");
            motor_set(-TEST_DUTY, -TEST_DUTY);
            enter(TEST_BWD, now);
        }
        break;

    case TEST_BWD:
        if ((now - s_stage_ms) > 1500U) {
            dbg("[test] done\r\n");
            motor_stop();
            enter(TEST_DONE, now);
        }
        break;

    default:
        break;
    }

    if ((now - s_print_ms) > 2000U) {
        char buf[96];
        s_print_ms = now;
        (void)snprintf(buf, sizeof(buf), "tick=%lu err=%lu L=%ld R=%ld vL=%d vR=%d\r\n",
                       (unsigned long)g_ctrl_ticks, (unsigned long)g_boot_errors,
                       (long)g_wheel_l.total, (long)g_wheel_r.total,
                       (int)(g_wheel_l.speed_mps * 1000.0f),
                       (int)(g_wheel_r.speed_mps * 1000.0f));
        dbg(buf);
    }
}

bool link_get_cmd(float *v, float *w)
{
    (void)v;
    (void)w;
    return false;   /* 自检模式没有上层控制 */
}

bool link_ready(void)
{
    return false;   /* 自检模式禁止闭环输出 */
}

bool link_get_brake(void)
{
    return false;   /* 自检模式不制动 */
}

#endif /* !APP_ENABLE_UROS && !APP_ENABLE_FRAME */
