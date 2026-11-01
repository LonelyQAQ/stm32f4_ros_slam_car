/**
 * @file    app_main.c
 * @brief   应用入口实现
 *
 * 控制链：TIM12 1 kHz 读编码器/积分位姿 → 每 10 ms 跑一次轮速 PID → TB6612
 * 发布链：主循环 → frame_link / uros_app_loop 按频率发遥测
 */
#include "microros_app_includes.h"
#include <math.h>

volatile uint32_t g_ctrl_ticks = 0;
volatile uint32_t g_boot_errors = 0;

extern TIM_HandleTypeDef htim12;

static uint32_t s_last_tick_ms;

/* 轮速 PID 状态 */
typedef struct {
    float integ;      /* 积分项，单位 duty */
    float prev_meas;  /* 上次实测轮速，m/s */
    float d_filt;     /* 低通后的轮加速度，m/s² */
} wheel_pid_t;

static wheel_pid_t s_pid_l;
static wheel_pid_t s_pid_r;

/* 10 ms 测速窗口：1 kHz 里累加编码器增量，避开单拍 0/1 计数的量化噪声 */
static uint32_t s_speed_ticks;
static float    s_speed_dt;
static int32_t  s_speed_cnt_l;
static int32_t  s_speed_cnt_r;

/* 里程计位姿（1 kHz 积分，frame 链路上行给 Jetson） */
static float s_pose_x, s_pose_y, s_pose_th;

void app_get_pose(float *x, float *y, float *theta)
{
    if (x)     *x = s_pose_x;
    if (y)     *y = s_pose_y;
    if (theta) *theta = s_pose_th;
}

/* cmd_vel → 左右轮线速度（差速模型） */
static void cmd_to_wheel(const float v, const float w, float *vl, float *vr)
{
    const float half_track = WHEEL_TRACK_M * 0.5f;
    *vl = v - w * half_track;
    *vr = v + w * half_track;
}

static void pid_reset(wheel_pid_t *p)
{
    p->integ     = 0.0f;
    p->prev_meas = 0.0f;
    p->d_filt    = 0.0f;
}

/* 轮速 PID：目标/实测 m/s → 带符号占空比 */
static int16_t pid_step(wheel_pid_t *p, float target, float meas, float dt)
{
    const float duty_max = (float)MOTOR_PWM_ARR * MOTOR_MAX_DUTY;
    const float e = target - meas;
    float dmeas;
    float integ;
    float u;

    /* 目标接近零且轮子已停下：滑行，清积分，避免原地嗡鸣 */
    if (fabsf(target) < WHEEL_PID_DEADBAND_MPS &&
        fabsf(meas) < WHEEL_PID_DEADBAND_MPS) {
        pid_reset(p);
        return 0;
    }

    /* 微分对测量值求导，避免目标阶跃时的微分冲击 */
    dmeas        = (meas - p->prev_meas) / dt;
    p->prev_meas = meas;
    p->d_filt   += (dt / (WHEEL_PID_D_TAU_S + dt)) * (dmeas - p->d_filt);

    integ = p->integ + WHEEL_PID_KI * e * dt;
    if (integ >  duty_max) integ =  duty_max;
    if (integ < -duty_max) integ = -duty_max;

    u = target / APP_V_MAX_MPS * duty_max     /* 前馈 */
      + WHEEL_PID_KP * e
      + integ
      - WHEEL_PID_KD * p->d_filt;

    /* 输出饱和且误差同向时冻结积分，防止积分饱和 */
    if (u > duty_max) {
        u = duty_max;
        if (e > 0.0f) integ = p->integ;
    } else if (u < -duty_max) {
        u = -duty_max;
        if (e < 0.0f) integ = p->integ;
    }

    p->integ = integ;
    return (int16_t)u;
}

static void control_update(float dt, float meas_l, float meas_r)
{
    float v = 0.0f, w = 0.0f;
    float vl, vr;

    /* 制动优先：只有急停 / 避障生效时链路才给 brake（自带 500 ms 超时回落） */
    if (link_get_brake()) {
        motor_brake();
        pid_reset(&s_pid_l);
        pid_reset(&s_pid_r);
        return;
    }

    /* 链路未就绪或指令超时，一律滑行停车（两脚低电平，不短路 H 桥） */
    if (!link_ready() || !link_get_cmd(&v, &w)) {
        motor_stop();
        pid_reset(&s_pid_l);
        pid_reset(&s_pid_r);
        return;
    }

    /* 限幅，防止上位机手滑发一个巨大的速度 */
    if (v >  APP_V_MAX_MPS) v =  APP_V_MAX_MPS;
    if (v < -APP_V_MAX_MPS) v = -APP_V_MAX_MPS;
    if (w >  APP_W_MAX_RPS) w =  APP_W_MAX_RPS;
    if (w < -APP_W_MAX_RPS) w = -APP_W_MAX_RPS;

    cmd_to_wheel(v, w, &vl, &vr);
    motor_set(pid_step(&s_pid_l, vl, meas_l, dt),
              pid_step(&s_pid_r, vr, meas_r, dt));
}

void app_main_init(void)
{
    motor_init();
    encoder_init();

    pid_reset(&s_pid_l);
    pid_reset(&s_pid_r);
    s_speed_ticks = 0;
    s_speed_dt    = 0.0f;
    s_speed_cnt_l = 0;
    s_speed_cnt_r = 0;

    if (!dev_imu_init()) {
        g_boot_errors++;
    }
    dev_attitude_init();     /* 依赖 dev_imu_init 的 SPI/片选就绪 */
    if (!dev_uart_init()) {
        g_boot_errors++;
    }

    s_last_tick_ms = HAL_GetTick();
}

void app_tick_isr(void)
{
    uint32_t now = HAL_GetTick();
    float dt = (float)(now - s_last_tick_ms) / 1000.0f;

    /* 用实测 dt 而不是 TIM12 周期：Period 改成 1 ms 或 2 ms 都不会算错速度 */
    if (dt < APP_TICK_DT_MIN) {
        return;
    }
    if (dt > APP_TICK_DT_MAX) {
        dt = APP_TICK_DT_MAX;
    }
    s_last_tick_ms = now;

    encoder_update(dt);

#if APP_ENABLE_UROS || APP_ENABLE_FRAME
    /* 速度环 100 Hz：10 ms 窗口累加计数，再换算轮速 */
    s_speed_cnt_l += g_wheel_l.delta;
    s_speed_cnt_r += g_wheel_r.delta;
    s_speed_dt    += dt;

    if (++s_speed_ticks >= WHEEL_SPEED_DIV) {
        float win = s_speed_dt;
        float ml  = (float)s_speed_cnt_l * ENC_M_PER_COUNT / win;
        float mr  = (float)s_speed_cnt_r * ENC_M_PER_COUNT / win;

        control_update(win, ml, mr);

        s_speed_ticks = 0;
        s_speed_dt    = 0.0f;
        s_speed_cnt_l = 0;
        s_speed_cnt_r = 0;
    }
#endif
    {
        odom_t od;
        odom_get(&od);
        s_pose_x  += od.linear_mps * cosf(s_pose_th) * dt;
        s_pose_y  += od.linear_mps * sinf(s_pose_th) * dt;
        s_pose_th += (od.yaw_rate_dps * 0.0174533f) * dt;
    }

    g_ctrl_ticks++;
}

void app_loop(void)
{
    dev_attitude_poll();   /* 有新样本才解算，无样本立即返回 */
    link_loop();
}

/* TIM12 更新中断：CubeMX 没有生成这个回调，由应用层实现 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM12) {
        app_tick_isr();
    }
}
