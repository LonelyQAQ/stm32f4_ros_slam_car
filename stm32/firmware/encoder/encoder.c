/**
 * @file    encoder.c
 * @brief   双轮正交编码器（TIM2 / TIM4 硬件解码）
 *
 * 注意：编码器定时器的 ARR 必须是 65535。这里用 (int16_t) 强转做溢出展开，
 *       ARR 不是全 16 位时展开会错（原 tids 工程用 >=10000 → -30000 的补丁，
 *       说明那边 ARR 设成了 29999，换到本板要改回 65535）。
 */
#include "encoder.h"
#include "board_cfg.h"

extern TIM_HandleTypeDef ENC_L_HTIM;
extern TIM_HandleTypeDef ENC_R_HTIM;

wheel_t g_wheel_l;
wheel_t g_wheel_r;

#define ENC_PI 3.14159265f

static void wheel_reset(wheel_t *w)
{
    w->total     = 0;
    w->delta     = 0;
    w->speed_mps = 0.0f;
}

void encoder_init(void)
{
    HAL_TIM_Encoder_Start(&ENC_L_HTIM, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&ENC_R_HTIM, TIM_CHANNEL_ALL);

    __HAL_TIM_SET_COUNTER(&ENC_L_HTIM, 0);
    __HAL_TIM_SET_COUNTER(&ENC_R_HTIM, 0);

    wheel_reset(&g_wheel_l);
    wheel_reset(&g_wheel_r);
}

void encoder_reset(void)
{
    __HAL_TIM_SET_COUNTER(&ENC_L_HTIM, 0);
    __HAL_TIM_SET_COUNTER(&ENC_R_HTIM, 0);
    wheel_reset(&g_wheel_l);
    wheel_reset(&g_wheel_r);
}

/* 读增量并清零。
 * 自适应 ARR：TIM2 是 32 位、TIM4 是 16 位，CubeMX 默认值不同，
 * 这里按实际 ARR 把计数折回正负半周，不依赖具体位宽。 */
static int32_t read_delta(TIM_HandleTypeDef *h)
{
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(h);
    uint32_t cnt = __HAL_TIM_GET_COUNTER(h);
    __HAL_TIM_SET_COUNTER(h, 0);

    if (arr >= 0x7FFFFFFFU) {          /* 32 位定时器跑满量程 */
        return (int32_t)cnt;
    }
    int32_t period = (int32_t)arr + 1;
    int32_t d      = (int32_t)cnt;
    if (d > period / 2) d -= period;   /* 16 位定时器：越过半周就折成负数 */
    return d;
}

static void wheel_update(wheel_t *w, int32_t delta, float dt)
{
    w->delta  = (int16_t)delta;
    w->total += delta;

    float raw = (delta * ENC_M_PER_COUNT) / dt;
    w->speed_mps += ENC_SPEED_LPF_ALPHA * (raw - w->speed_mps);
}

void encoder_update(float dt)
{
    if (dt <= 0.0f) return;

    int32_t dl = read_delta(&ENC_L_HTIM) * ENC_L_SIGN;
    int32_t dr = read_delta(&ENC_R_HTIM) * ENC_R_SIGN;

    wheel_update(&g_wheel_l, dl, dt);
    wheel_update(&g_wheel_r, dr, dt);
}

void odom_get(odom_t *out)
{
    if (out == 0) return;

    const float vl = g_wheel_l.speed_mps;
    const float vr = g_wheel_r.speed_mps;

    out->linear_mps   = 0.5f * (vl + vr);
    out->yaw_rate_dps = ((vr - vl) / WHEEL_TRACK_M) * (180.0f / ENC_PI);
    out->left_total   = g_wheel_l.total;
    out->right_total  = g_wheel_r.total;
}
