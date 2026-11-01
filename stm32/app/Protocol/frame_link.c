/**
 * @file    frame_link.c
 * @brief   自定义二进制帧链路（F407 ↔ Jetson）
 *
 * 帧格式（小端）：
 *   0xAA 0x55 | type(1) | len(1) | payload(len) | crc16(2)
 *   crc16 = CRC-16/CCITT-FALSE（初值 0xFFFF，多项式 0x1021），覆盖 type|len|payload
 *
 * 上行 TELEMETRY(0x01) 90 B @50 Hz：tick / 位姿 / 速度 / 编码器 / IMU / 四元数 / 零偏诊断
 * 下行 CMD_VEL(0x02)    9 B：线速度、角速度、标志位（bit0=制动请求）
 *
 * 带宽：66 B × 50 = 3.3 KB/s，921600 下占 3.6%，余量极大。
 */
#include "microros_app_includes.h"
#include <string.h>

#if APP_ENABLE_FRAME

extern UART_HandleTypeDef UROS_UART_HANDLE;

#define FRAME_H1          0xAAU
#define FRAME_H2          0x55U
#define FRAME_TYPE_TELEM  0x01U
#define FRAME_TYPE_CMD    0x02U

#define TELEM_PAYLOAD_LEN 90U
#define CMD_PAYLOAD_LEN   9U
#define CMD_FLAG_BRAKE    0x01U   /* flags bit0：急停/避障生效，请求制动 */
#define FRAME_MAX_PAYLOAD 96U
#define TX_PERIOD_MS      (1000U / 50U)

typedef struct __attribute__((packed)) {
    uint32_t tick_ms;
    float    x;
    float    y;
    float    theta;
    float    v;
    float    w;
    int32_t  left_ticks;
    int32_t  right_ticks;
    float    gyro[3];
    float    acc[3];
    float    temp_c;
    float    cmd_v;      /* 回显：MCU 实际收到的 v/w，用来验证下行 */
    float    cmd_w;
    float    q[4];       /* VQF 6D 姿态 w,x,y,z（机体系→地理系） */
    float    bias_z;     /* VQF 估计的陀螺 z 零偏，rad/s */
    uint8_t  rest;       /* VQF 静止检测 */
    uint8_t  zupt;       /* 零速修正在生效 */
} telem_t;

typedef struct __attribute__((packed)) {
    float   v;
    float   w;
    uint8_t flags;   /* CMD_FLAG_*，未知位忽略 */
} cmd_vel_t;

/* 解析状态机 */
typedef enum {
    RX_WAIT_H1 = 0,
    RX_WAIT_H2,
    RX_TYPE,
    RX_LEN,
    RX_PAYLOAD,
    RX_CRC_LO,
    RX_CRC_HI
} rx_state_t;

static rx_state_t s_rx_state;
static uint8_t    s_rx_type;
static uint8_t    s_rx_len;
static uint8_t    s_rx_payload[FRAME_MAX_PAYLOAD];
static uint16_t   s_rx_idx;
static uint16_t   s_rx_crc;

static float    s_cmd_v;
static float    s_cmd_w;
static uint32_t s_cmd_ms;
static bool     s_cmd_seen;
static uint32_t s_brake_ms;     /* 最近一次"请求制动"的时刻 */
static bool     s_brake_seen;

static uint32_t s_next_telem_ms;

volatile uint32_t g_frame_tx_frames;
volatile uint32_t g_frame_rx_frames;
volatile uint32_t g_frame_crc_errors;

/* ---------------- CRC ---------------- */

static uint16_t crc16_ccitt(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFU;

    while (len--) {
        crc ^= (uint16_t)(*data++) << 8;
        for (uint8_t i = 0; i < 8U; i++) {
            crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U)
                                  : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

/* ---------------- 发送 ---------------- */

static bool frame_send(uint8_t type, const uint8_t *payload, uint8_t len)
{
    uint8_t  buf[4 + FRAME_MAX_PAYLOAD + 2];
    uint16_t n = 0;
    uint16_t crc;

    if (len > FRAME_MAX_PAYLOAD) {
        return false;
    }

    buf[n++] = FRAME_H1;
    buf[n++] = FRAME_H2;
    buf[n++] = type;
    buf[n++] = len;
    memcpy(&buf[n], payload, len);
    n = (uint16_t)(n + len);

    crc = crc16_ccitt(&buf[2], (uint16_t)(2U + len));
    buf[n++] = (uint8_t)(crc & 0xFFU);
    buf[n++] = (uint8_t)(crc >> 8);

    if (!dev_uart_write(buf, n)) {
        return false;
    }
    g_frame_tx_frames++;
    return true;
}

static void send_telemetry(void)
{
    telem_t t;
    odom_t  od;

    memset(&t, 0, sizeof(t));
    odom_get(&od);

    t.tick_ms     = HAL_GetTick();
    t.v           = od.linear_mps;
    t.w           = od.yaw_rate_dps * 0.0174533f;
    t.left_ticks  = od.left_total;
    t.right_ticks = od.right_total;

    {
        float px, py, pth;
        app_get_pose(&px, &py, &pth);
        t.x     = px;
        t.y     = py;
        t.theta = pth;
    }

    /* 回显下行指令（未收到过则为 0） */
    t.cmd_v = s_cmd_seen ? s_cmd_v : 0.0f;
    t.cmd_w = s_cmd_seen ? s_cmd_w : 0.0f;

    {
        dev_attitude_t att;

        /* 200 Hz 采样在 dev_attitude_poll 里完成，这里只取缓存，避免抢 SPI */
        if (dev_attitude_get(&att)) {
            /* 发"已扣零偏"的角速度：VQF 只在内部四元数里扣，EKF 吃的是这里，
             * 不减的话 EKF 的 yaw 会按零偏速率持续漂（实测正好等于 bias_z）。 */
            t.gyro[0] = att.sample.gyro_rps[0] - att.bias[0];
            t.gyro[1] = att.sample.gyro_rps[1] - att.bias[1];
            t.gyro[2] = att.sample.gyro_rps[2] - att.bias[2];
            t.acc[0]  = att.sample.acc_mps2[0];
            t.acc[1]  = att.sample.acc_mps2[1];
            t.acc[2]  = att.sample.acc_mps2[2];
            t.temp_c  = att.sample.temp_c;
            t.q[0]    = att.q[0];
            t.q[1]    = att.q[1];
            t.q[2]    = att.q[2];
            t.q[3]    = att.q[3];
            t.bias_z  = att.bias[2];
            t.rest    = att.rest;
            t.zupt    = att.zupt;
        }
    }

    (void)frame_send(FRAME_TYPE_TELEM, (const uint8_t *)&t, (uint8_t)sizeof(t));
}

/* ---------------- 接收 ---------------- */

static void handle_frame(uint8_t type, const uint8_t *payload, uint8_t len)
{
    g_frame_rx_frames++;

    if (type == FRAME_TYPE_CMD && len == CMD_PAYLOAD_LEN) {
        cmd_vel_t c;
        memcpy(&c, payload, sizeof(c));
        s_cmd_v    = c.v;
        s_cmd_w    = c.w;
        s_cmd_ms   = HAL_GetTick();
        s_cmd_seen = true;

        if ((c.flags & CMD_FLAG_BRAKE) != 0U) {
            s_brake_ms   = s_cmd_ms;   /* 每帧重新续期，掉线后自然过期 */
            s_brake_seen = true;
        }
    }
}

static void rx_byte(uint8_t b)
{
    uint16_t crc;

    switch (s_rx_state) {
    case RX_WAIT_H1:
        if (b == FRAME_H1) s_rx_state = RX_WAIT_H2;
        break;

    case RX_WAIT_H2:
        s_rx_state = (b == FRAME_H2) ? RX_TYPE : RX_WAIT_H1;
        break;

    case RX_TYPE:
        s_rx_type  = b;
        s_rx_state = RX_LEN;
        break;

    case RX_LEN:
        if (b == 0U || b > FRAME_MAX_PAYLOAD) {
            s_rx_state = RX_WAIT_H1;
        } else {
            s_rx_len   = b;
            s_rx_idx   = 0;
            s_rx_state = RX_PAYLOAD;
        }
        break;

    case RX_PAYLOAD:
        s_rx_payload[s_rx_idx++] = b;
        if (s_rx_idx >= s_rx_len) s_rx_state = RX_CRC_LO;
        break;

    case RX_CRC_LO:
        s_rx_crc   = b;
        s_rx_state = RX_CRC_HI;
        break;

    case RX_CRC_HI:
        s_rx_crc |= (uint16_t)((uint16_t)b << 8);
        {
            uint8_t hdr[2] = { s_rx_type, s_rx_len };
            uint8_t tmp[2 + FRAME_MAX_PAYLOAD];
            memcpy(tmp, hdr, 2);
            memcpy(&tmp[2], s_rx_payload, s_rx_len);
            crc = crc16_ccitt(tmp, (uint16_t)(2U + s_rx_len));
        }
        if (crc == s_rx_crc) {
            handle_frame(s_rx_type, s_rx_payload, s_rx_len);
        } else {
            g_frame_crc_errors++;
        }
        s_rx_state = RX_WAIT_H1;
        break;

    default:
        s_rx_state = RX_WAIT_H1;
        break;
    }
}

/* ---------------- 对外 ---------------- */

void link_loop(void)
{
    uint8_t  buf[64];
    uint16_t n;
    uint32_t now = HAL_GetTick();

    while ((n = dev_uart_read(buf, sizeof(buf))) > 0U) {
        for (uint16_t i = 0; i < n; i++) {
            rx_byte(buf[i]);
        }
    }

    if ((uint32_t)(now - s_next_telem_ms) >= TX_PERIOD_MS) {
        s_next_telem_ms = now;
        send_telemetry();
    }
}

bool link_get_cmd(float *v, float *w)
{
    if (!s_cmd_seen || (HAL_GetTick() - s_cmd_ms) > APP_CMD_TIMEOUT_MS) {
        return false;
    }
    *v = s_cmd_v;
    *w = s_cmd_w;
    return true;
}

bool link_get_brake(void)
{
    /* 只有"急停/避障"才会置这个标志；超过保持时间自动回落滑行，
     * 避免 Jetson 掉线时 H 桥长期短路发热。 */
    if (!s_brake_seen) {
        return false;
    }
    return (HAL_GetTick() - s_brake_ms) <= APP_BRAKE_HOLD_MS;
}

bool link_ready(void)
{
    return true;   /* 帧链路的保护靠 cmd 超时，不靠连接状态 */
}

#endif /* APP_ENABLE_FRAME */
