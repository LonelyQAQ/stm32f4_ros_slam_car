#include "microros_app_includes.h"
#if APP_ENABLE_UROS

#include <rmw_microros/rmw_microros.h>
#include <time.h>

static bool transport_open(struct timespec *timeout, size_t *baudrate, void *args)
{
    (void)timeout;
    (void)baudrate;
    (void)args;
    return dev_uart_init();
}

static bool transport_close(void *args)
{
    (void)args;
    return true;
}

static bool transport_write(void *buf, size_t len, void *args)
{
    (void)args;
    return dev_uart_write((const uint8_t *)buf, (uint16_t)len);
}

/* 必须立即返回：XRCE 自己在循环里重试，阻塞会把控制节拍拖死 */
static bool transport_read(void *buf, size_t len, size_t *read_bytes, void *args)
{
    (void)args;
    *read_bytes = dev_uart_read((uint8_t *)buf, (uint16_t)len);
    return true;
}

bool uros_transport_install(void)
{
    return rmw_uros_set_custom_transport(
        true,                 /* framing：帧化 + CRC，与 agent 一致 */
        NULL,                 /* 传给回调的 args，这里不用 */
        transport_open,
        transport_close,
        transport_write,
        transport_read);
}

#endif /* APP_ENABLE_UROS */
