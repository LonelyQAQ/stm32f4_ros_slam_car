/**
 * @file    dev_attitude.c
 * @brief   BMI088 + VQF 姿态解算（6D）+ 零速修正（ZUPT）
 *
 * 采样时机由加速度计的 DRDY 位决定：主循环空转时几乎不产生 SPI 流量，
 * 有数据才跑一次 VQF，等价于"200 Hz 中断里做解算"，但不用占中断。
 *
 * ZUPT：轮编码器判定"真的没动"时，用静止窗口内陀螺 z 的均值直接钉住零偏。
 * 这是 6D 无磁力计情况下压制 yaw 漂移最有效的手段。
 */
#include "microros_app_includes.h"
#include <math.h>
#include <string.h>

volatile uint32_t g_att_updates;
volatile uint32_t g_att_errors;
volatile uint32_t g_zupt_count;   /* 生效过多少次 ZUPT 修正 */

static dev_attitude_t s_att;
static bool           s_ready;

/* ---- ZUPT 状态 ---- */
static float    s_still_s;      /* 连续静止时长 */
static float    s_gz_sum;       /* 静止窗口内 gyro z 累加 */
static uint32_t s_gz_n;
static uint8_t  s_zupt;

bool dev_attitude_init(void)
{
    memset(&s_att, 0, sizeof(s_att));
    s_att.q[0] = 1.0f;                       /* 单位四元数 */
    initVqf(APP_VQF_TS, APP_VQF_TS, APP_VQF_TS);

    s_still_s = 0.0f;
    s_gz_sum  = 0.0f;
    s_gz_n    = 0;
    s_zupt    = 0;

    s_ready = true;
    return true;
}

/* 传感器轴向 → 车体轴向；只在符号/轴向不一致时需要改 app_cfg.h */
static float axis_pick(const float v[3], int code)
{
    int   idx = (code > 0 ? code : -code) - 1;
    float x   = v[idx];
    return (code > 0) ? x : -x;
}

static void apply_mount(float v[3])
{
    float s[3] = { v[0], v[1], v[2] };

    v[0] = axis_pick(s, APP_IMU_BODY_X);
    v[1] = axis_pick(s, APP_IMU_BODY_Y);
    v[2] = axis_pick(s, APP_IMU_BODY_Z);
}

/**
 * 零速修正：轮子连续静止一段时间后，把静止窗口内陀螺 z 的均值当作零偏钉给 VQF。
 * 边界：样本不足不修正；一有运动立刻退出并清零窗口。
 */
static void zupt_update(const dev_imu_sample_t *s)
{
    odom_t od;
    bool   still;
    bool   gyro_quiet;

    odom_get(&od);

    /* 三重判据：轮速静止 + 陀螺本身也静止。
     * 只判轮速是危险的：手转车辆时轮子打滑不转，会把"转动角速度"当成零偏钉死，
     * 之后陀螺一直偏（实测出现过 3.3°/s 的假角速度）。 */
    gyro_quiet = (fabsf(s->gyro_rps[0]) < IMU_ZUPT_GYRO_MAX)
              && (fabsf(s->gyro_rps[1]) < IMU_ZUPT_GYRO_MAX)
              && (fabsf(s->gyro_rps[2]) < IMU_ZUPT_GYRO_MAX);

    still = (fabsf(od.linear_mps) < IMU_ZUPT_V_MPS)
         && (fabsf(od.yaw_rate_dps * 0.0174533f) < IMU_ZUPT_W_RPS)
         && gyro_quiet;

    if (!still) {
        s_still_s = 0.0f;
        s_gz_sum  = 0.0f;
        s_gz_n    = 0;
        s_zupt    = 0;
        return;
    }

    s_still_s += APP_VQF_TS;
    s_gz_sum  += s->gyro_rps[2];
    s_gz_n++;

    if (s_still_s < IMU_ZUPT_HOLD_S) {
        return;
    }
    s_zupt = 1;

    if (s_gz_n < IMU_ZUPT_MIN_SAMPLES) {
        return;
    }

    {
        vqf_real_t bias[3];
        float      gz_mean = s_gz_sum / (float)s_gz_n;

        /* 均值超门限说明窗口里其实在转，丢弃这次修正 */
        if (fabsf(gz_mean) > IMU_ZUPT_GYRO_MAX) {
            s_gz_sum = 0.0f;
            s_gz_n   = 0;
            return;
        }

        getBiasEstimate(bias);
        bias[2] = gz_mean;
        setBiasEstimate(bias, IMU_ZUPT_BIAS_SIGMA);
        s_gz_sum = 0.0f;
        s_gz_n   = 0;
        g_zupt_count++;
    }
}

void dev_attitude_poll(void)
{
    dev_imu_sample_t s;
    bool             ready = false;

    if (!s_ready) {
        return;
    }
    if (BMI088_AccelDataReady(&ready) != BMI088_OK) {
        g_att_errors++;
        return;
    }
    if (!ready) {
        return;
    }
    if (!dev_imu_read(&s)) {
        g_att_errors++;
        return;
    }

    apply_mount(s.gyro_rps);
    apply_mount(s.acc_mps2);

    updateGyr(s.gyro_rps);
    updateAcc(s.acc_mps2);
    getQuat6D(s_att.q);

    zupt_update(&s);

    s_att.sample   = s;
    s_att.updates  = ++g_att_updates;
    s_att.rest     = getRestDetected() ? 1U : 0U;
    s_att.zupt     = s_zupt;
}

bool dev_attitude_get(dev_attitude_t *out)
{
    vqf_real_t bias[3];

    if ((out == NULL) || !s_ready) {
        return false;
    }
    getBiasEstimate(bias);
    s_att.bias[0] = bias[0];
    s_att.bias[1] = bias[1];
    s_att.bias[2] = bias[2];
    *out = s_att;
    return true;
}

bool dev_attitude_get_sample(dev_imu_sample_t *out)
{
    if ((out == NULL) || !s_ready) {
        return false;
    }
    if (s_att.updates == 0U) {
        return false;
    }
    *out = s_att.sample;
    return true;
}
