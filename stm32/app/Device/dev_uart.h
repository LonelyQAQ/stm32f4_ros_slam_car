/**
 * @file    dev_uart.h
 * @brief   USART1 收发封装：DMA 圆形接收 + DMA 单次发送
 *
 * 接收不做中断解析：DMA 自己把字节搬进环形区，读侧用「DMA 剩余计数」推写指针。
 * 这样 ISR 里什么都不用做，不会因为串口负载高而丢节拍。
 */
#ifndef DEV_UART_H
#define DEV_UART_H

#include <stdint.h>
#include <stdbool.h>

/** 启动 DMA 接收（幂等：重复调用安全，已在收则直接返回 true） */
bool dev_uart_init(void);

/**
 * @brief  取走环形缓冲里的数据
 * @param  buf  目标缓冲
 * @param  len  最多取多少字节
 * @return 实际取到的字节数（0 = 当前没数据，立即返回，不阻塞）
 */
uint16_t dev_uart_read(uint8_t *buf, uint16_t len);

/**
 * @brief  阻塞发送一整块（等 DMA 发完或超时）
 * @retval true 发送完成
 */
bool dev_uart_write(const uint8_t *data, uint16_t len);

/** 只在调试/自检时用：把接收缓冲清空 */
void dev_uart_flush(void);

/* 只读计数器，给上层看链路健康度 */
extern volatile uint32_t g_uart_rx_bytes;
extern volatile uint32_t g_uart_tx_bytes;
extern volatile uint32_t g_uart_errors;
extern volatile uint32_t g_uart_overflow;   /* 消费落后导致数据被覆盖的字节数 */

#endif /* DEV_UART_H */
