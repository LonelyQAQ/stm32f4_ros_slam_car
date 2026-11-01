/**
 * @file    motor.c
 * @brief   TB6612 双路电机驱动
 */
#include "motor.h"
#include "board_cfg.h"

extern TIM_HandleTypeDef MOTOR_PWM_HTIM;

static int16_t clamp_duty(int32_t v)
{
    const int32_t lim = (int32_t)(MOTOR_PWM_ARR * MOTOR_MAX_DUTY);
    if (v >  lim) return (int16_t) lim;
    if (v < -lim) return (int16_t)-lim;
    return (int16_t)v;
}

/* 一个 H 桥：v>0 正转、v<0 反转、v==0 滑行 */
static void drive_channel(GPIO_TypeDef *in1_port, uint16_t in1_pin,
                          GPIO_TypeDef *in2_port, uint16_t in2_pin,
                          int32_t v, uint32_t ch)
{
    if (v > 0) {
        HAL_GPIO_WritePin(in1_port, in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(in2_port, in2_pin, GPIO_PIN_SET);
    } else if (v < 0) {
        HAL_GPIO_WritePin(in1_port, in1_pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(in2_port, in2_pin, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(in1_port, in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(in2_port, in2_pin, GPIO_PIN_RESET);
    }

    /* 用无符号幅值，方向已经由 IN1/IN2 决定 */
    uint32_t mag = (v >= 0) ? (uint32_t)v : (uint32_t)(-v);
    __HAL_TIM_SET_COMPARE(&MOTOR_PWM_HTIM, ch, mag);
}

void motor_init(void)
{
    HAL_TIM_PWM_Start(&MOTOR_PWM_HTIM, MOTOR_A_CH);
    HAL_TIM_PWM_Start(&MOTOR_PWM_HTIM, MOTOR_B_CH);
    motor_stop();
}

void motor_set(int16_t left, int16_t right)
{
    int32_t l = clamp_duty((int32_t)left  * MOTOR_A_SIGN);
    int32_t r = clamp_duty((int32_t)right * MOTOR_B_SIGN);

    drive_channel(MOTOR_A_IN1_PORT, MOTOR_A_IN1_PIN,
                  MOTOR_A_IN2_PORT, MOTOR_A_IN2_PIN, l, MOTOR_A_CH);
    drive_channel(MOTOR_B_IN1_PORT, MOTOR_B_IN1_PIN,
                  MOTOR_B_IN2_PORT, MOTOR_B_IN2_PIN, r, MOTOR_B_CH);
}

void motor_stop(void)
{
    __HAL_TIM_SET_COMPARE(&MOTOR_PWM_HTIM, MOTOR_A_CH, 0);
    __HAL_TIM_SET_COMPARE(&MOTOR_PWM_HTIM, MOTOR_B_CH, 0);

    HAL_GPIO_WritePin(MOTOR_A_IN1_PORT, MOTOR_A_IN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_A_IN2_PORT, MOTOR_A_IN2_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_B_IN1_PORT, MOTOR_B_IN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_B_IN2_PORT, MOTOR_B_IN2_PIN, GPIO_PIN_RESET);
}

void motor_brake(void)
{
    __HAL_TIM_SET_COMPARE(&MOTOR_PWM_HTIM, MOTOR_A_CH, 0);
    __HAL_TIM_SET_COMPARE(&MOTOR_PWM_HTIM, MOTOR_B_CH, 0);

    HAL_GPIO_WritePin(MOTOR_A_IN1_PORT, MOTOR_A_IN1_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_A_IN2_PORT, MOTOR_A_IN2_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_B_IN1_PORT, MOTOR_B_IN1_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_B_IN2_PORT, MOTOR_B_IN2_PIN, GPIO_PIN_SET);
}
