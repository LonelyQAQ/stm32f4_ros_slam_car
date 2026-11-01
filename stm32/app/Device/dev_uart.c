/**
 * @file    dev_uart.c
 * @brief   USART1 DMA 收发封装
 *
 * 依赖：CubeMX 已配置 USART1 921600 8N1，
 *       USART1_RX = DMA2_Stream2（Circular），USART1_TX = DMA2_Stream7（Normal）。
 *
 * 生命周期：DMA 接收只启动一次。app_main_init() 与 transport_open() 都会调用
 * dev_uart_init()，因此该函数必须**幂等** —— 已在收时直接返回成功，否则
 * HAL_UART_Receive_DMA 会返回 HAL_BUSY，transport open 失败。
 */
#include "microros_app_includes.h"
#include <string.h>

extern UART_HandleTypeDef UROS_UART_HANDLE;

volatile uint32_t g_uart_rx_bytes = 0;
volatile uint32_t g_uart_tx_bytes = 0;
volatile uint32_t g_uart_errors   = 0;
volatile uint32_t g_uart_overflow = 0;   /* 消费落后导致 DMA 覆盖未读数据的字节数 */

static uint8_t  s_rx_dma[UROS_RX_DMA_SIZE];
static uint16_t s_rx_tail;        /* 读侧已消费到的位置 */
static uint16_t s_rx_head_seen;   /* 上一次看到的 DMA 写指针 */
static uint16_t s_rx_pending;     /* 环形区里尚未读走的字节数 */
static uint8_t  s_tx_buf[UROS_TX_BUF_SIZE];
static volatile bool s_tx_busy;

/* DMA 写指针 = 缓冲区大小 − 剩余传输数 */
static uint16_t rx_write_pos(void)
{
    uint16_t remain = (uint16_t)__HAL_DMA_GET_COUNTER(UROS_UART_HANDLE.hdmarx);
    return (uint16_t)(UROS_RX_DMA_SIZE - remain) % UROS_RX_DMA_SIZE;
}

bool dev_uart_init(void)
{
    if (s_tx_busy) {
        s_tx_busy = false;
    }

    /* 幂等：已经在收就不再启动第二次 */
    if (UROS_UART_HANDLE.RxState == HAL_UART_STATE_BUSY_RX) {
        return true;
    }

    s_rx_tail      = 0;
    s_rx_head_seen = 0;
    s_rx_pending   = 0;

    if (HAL_UART_Receive_DMA(&UROS_UART_HANDLE, s_rx_dma, UROS_RX_DMA_SIZE) != HAL_OK) {
        g_uart_errors++;
        return false;
    }
    return true;
}

/* 把 DMA 新写入的字节计入 pending；一次性写入超过剩余空间说明消费者落后、数据已被覆盖 */
static uint16_t rx_account(void)
{
    uint16_t head  = rx_write_pos();
    uint16_t delta = (uint16_t)((head - s_rx_head_seen) % UROS_RX_DMA_SIZE);
    uint16_t free_space = (uint16_t)(UROS_RX_DMA_SIZE - 1U - s_rx_pending);

    s_rx_head_seen = head;

    if (delta > free_space) {
        g_uart_overflow += (uint32_t)(delta - free_space);
        s_rx_pending = (uint16_t)(UROS_RX_DMA_SIZE - 1U);
    } else {
        s_rx_pending = (uint16_t)(s_rx_pending + delta);
    }
    return s_rx_pending;
}

uint16_t dev_uart_read(uint8_t *buf, uint16_t len)
{
    uint16_t avail = rx_account();
    uint16_t got   = 0;

    while (got < len && got < avail) {
        buf[got++] = s_rx_dma[s_rx_tail];
        s_rx_tail = (uint16_t)((s_rx_tail + 1U) % UROS_RX_DMA_SIZE);
    }

    s_rx_pending = (uint16_t)(s_rx_pending - got);
    g_uart_rx_bytes += got;
    return got;
}

/* 覆盖之前必须确认上一帧已发完，否则会改到正在发送的缓冲区 */
static bool wait_tx_idle(uint32_t start_ms)
{
    while (s_tx_busy) {
        if ((HAL_GetTick() - start_ms) > UROS_TX_TIMEOUT_MS) {
            (void)HAL_UART_AbortTransmit(&UROS_UART_HANDLE);
            s_tx_busy = false;
            g_uart_errors++;
            return false;
        }
    }
    return true;
}

bool dev_uart_write(const uint8_t *data, uint16_t len)
{
    uint32_t t0 = HAL_GetTick();

    if (len == 0U || len > UROS_TX_BUF_SIZE) {
        g_uart_errors++;
        return false;
    }
    if (!wait_tx_idle(t0)) {
        return false;
    }

    memcpy(s_tx_buf, data, len);
    s_tx_busy = true;

    if (HAL_UART_Transmit_DMA(&UROS_UART_HANDLE, s_tx_buf, len) != HAL_OK) {
        s_tx_busy = false;
        g_uart_errors++;
        return false;
    }
    if (!wait_tx_idle(t0)) {
        return false;
    }

    g_uart_tx_bytes += len;
    return true;
}

void dev_uart_flush(void)
{
    uint16_t head = rx_write_pos();
    s_rx_tail      = head;
    s_rx_head_seen = head;
    s_rx_pending   = 0;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &UROS_UART_HANDLE) {
        s_tx_busy = false;
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &UROS_UART_HANDLE) {
        s_tx_busy = false;
        g_uart_errors++;
        /* 出错后 DMA 会停，必须重启接收，否则链路永久静默 */
        (void)HAL_UART_AbortReceive(&UROS_UART_HANDLE);
        s_rx_pending   = 0;
        s_rx_head_seen = 0;
        s_rx_tail      = 0;
        (void)HAL_UART_Receive_DMA(&UROS_UART_HANDLE, s_rx_dma, UROS_RX_DMA_SIZE);
    }
}
