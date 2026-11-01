# SLAM 小车固件模块（STM32F407 天空星载板）

从你给的 BMI088 驱动 + tids 工程的电机/编码器代码整理而来，去掉原工程耦合，全部依赖
`board/board_cfg.h` 一处配置。

```
firmware/
├── board/board_cfg.h     引脚、定时器、机械参数（换板只改这里）
├── motor/motor.c,h       TB6612 双路（方向 + 占空比）
├── encoder/encoder.c,h   TIM2/TIM4 硬件正交解码 → 轮速 → 里程
└── bmi088/bmi088.c,h     SPI1 六轴 IMU（原驱动，已适配本板）
```

## 引脚映射（与载板原理图一致）

| 功能 | MCU | 定时器 / 说明 |
|---|---|---|
| PWMA | PC8 | TIM3_CH3（TIM8_CH3 也可） |
| PWMB | PC9 | TIM3_CH4（TIM8_CH4 也可） |
| AIN1/AIN2 | PC0 / PC1 | GPIO 推挽 |
| BIN1/BIN2 | PC2 / PC3 | GPIO 推挽 |
| E1A/E1B | PA0 / PA1 | TIM2 编码器模式 |
| E2A/E2B | PD12 / PD13 | TIM4 编码器模式 |
| IMU SCK/MISO/MOSI | PA5 / PA6 / PA7 | SPI1 |
| IMU CSB1/CSB2 | PC4 / PC5 | GPIO 推挽，默认高 |

## CubeMX 必须配好的几项

1. **TIM3**：PWM Generation CH3/CH4，`PSC=0`、`ARR=4199` → 20 kHz（TIM3 时钟 84 MHz）。
2. **TIM2 / TIM4**：`Combined Channels = Encoder Mode`，**`ARR = 65535`**，两路都开。
3. **SPI1**：`Mode 0`，8 bit，主机，`Prescaler ≥ 16`（≤ 10.5 MHz）。
   **必须开 SPI1 的 RX DMA** —— BMI088 驱动的读操作走 `HAL_SPI_TransmitReceive_DMA`，
   没配 DMA 会一直等完成标志。
4. **GPIO**：PC0–PC5 全部推挽输出（初值见上），不要开上下拉。

## 用法

```c
motor_init();
encoder_init();
BMI088_Config cfg = { BMI088_ACC_RANGE_6G, BMI088_ACC_ODR_200_HZ,
                      BMI088_ACC_BWP_NORMAL, BMI088_GYRO_RANGE_2000,
                      BMI088_GYRO_BW_400_116 };
BMI088_Init(&hspi1, &cfg);

/* 5 ms 周期任务 */
encoder_update(0.005f);

odom_t o; odom_get(&o);
motor_set(cmd_a, cmd_b);          /* ±(4199*0.95) */
```

## 三个必须知道的坑

1. **编码器 ARR 已经不是硬约束了**。原 tids 工程两个定时器都填 29999（TIM2 是 32 位、
   TIM4 是 16 位，俩都填了同一个数），所以代码里才有 `if(temp>=10000) temp-=30000;` 的补丁。
   本驱动改成读 `__HAL_TIM_GET_AUTORELOAD()` 自适应位宽，ARR 填多少都能正确展开。
   **仍建议 TIM2/TIM4 都设 65535**：两个一致、行为统一，也留足余量。

2. **BMI088 驱动重定义了 `HAL_SPI_TxRxCpltCallback` 和 `HAL_SPI_ErrorCallback`**。
   如果这个工程里还有别的 SPI 走 DMA，回调会互相覆盖 —— 目前 SPI1 只给 IMU，独占没问题。

3. **BMI088 驱动会改 `hspi->Init.BaudRatePrescaler`**（初始化时强制 ≥16 分频再 `HAL_SPI_Init`）。
   这是为了防止有人把 SPI 配到 21 MHz 以上，属于保护措施，不要删。

## 还没做的

- 速度 PID 环（1 kHz）：应放在调用 `encoder_update` 的任务里，本包只给底层读写。
- 里程计积分（x/y/θ）：放在 micro-ROS 节点里，用 `odom_t` 的线速度 + 角速度积分。
- 标定：`WHEEL_DIAMETER_M`、`WHEEL_TRACK_M`、`ENCODER_LINES`、`GEAR_RATIO` 四个值现在是
  占位，必须实测后改 `board_cfg.h`，否则 odom 的尺度和转向全是错的。
