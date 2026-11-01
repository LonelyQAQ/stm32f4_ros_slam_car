# micro-ROS 移植 · Keil 集成清单

> 目标工程：`ROS_STM32_Test`（STM32F407VGT6 / Keil MDK-ARM AC6 / HAL F4 1.28.3）
> 代码位置：`app/`（业务，CubeMX 不接管）+ `firmware/`（电机/编码器/IMU 驱动）
> 状态：**源码已通过 clang -Wall 语法检查；未上板验证**

---

## 1. 现在的目录

```
ROS_STM32_Test/
├── Core/                       ← CubeMX 管理，不要手改（除 USER CODE 区）
├── Drivers/                    ← HAL
├── app/                        ← 本次新增，业务代码
│   ├── microros_app_includes.h     ← 唯一总头
│   ├── System/  app_cfg.h  app_main.c/h
│   ├── Device/  dev_uart.c/h  dev_imu.c/h
│   └── Protocol/ uros_transport.c/h  uros_app.c/h
└── firmware/                   ← 从 SLAM_Car 复制过来的驱动
    ├── board/board_cfg.h
    ├── motor/motor.c/h
    ├── encoder/encoder.c/h
    └── bmi088/bmi088.c/h
```

## 2. Keil 里要加的 Include 路径

`Options for Target → C/C++ → Include Paths`，加这 7 条：

```
.\app
.\app\System
.\app\Device
.\app\Protocol
.\firmware\board
.\firmware\motor
.\firmware\encoder
.\firmware\bmi088
```

再加上 micro-ROS 源码树的头文件目录（生成后按实际路径填，一般形如）：

```
.\microros\include
.\microros\src\rcl\rcl\include
.\microros\src\rclc\rclc\include
.\microros\src\rmw_microxrcedds\rmw_microxrcedds\include
.\microros\src\microxrcedds_client\include
.\microros\src\microcdr\include
.\microros\src\rosidl_runtime_c\include
.\microros\src\rosidl_typesupport_microxrcedds_c\include
.\microros\src\builtin_interfaces\include
.\microros\src\nav_msgs\include
.\microros\src\sensor_msgs\include
.\microros\src\geometry_msgs\include
.\microros\src\std_msgs\include
```

> 具体有哪些取决于你生成 `mcu_ws` 时勾选了哪些包。**先加 `app/` 与 `firmware/` 的，
> 编译会报缺哪个 ros 头，再逐个补路径**，比一次性猜准。

## 3. 预处理宏

`Options for Target → C/C++ → Define`，新增：

```
RMW_UXRCE_TRANSPORT=custom
```

其余（`RMW_UXRCE_MAX_NODES` 等）用默认值即可，等出现资源不足再调。

## 4. 编译选项（关键）

| 项 | 值 |
|---|---|
| Floating Point Hardware | **Single Precision**（`--fpu=FPv4-SP`） |
| 浮点 ABI | **hard-float**（`--apcs=/hardfp`） |
| Optimization | `-O1` 或 `-Os`（先别 `-O0`，micro-ROS 代码量大） |

**必须和 micro_ros_lib 用同一套浮点 ABI**，否则 `float` 传参走错寄存器，现象是"能跑但数据乱"。

## 5. 要加入编译的源文件

新建两个 Group：

**Group `app`**
```
app/System/app_main.c
app/Device/dev_uart.c
app/Device/dev_imu.c
app/Protocol/uros_transport.c
app/Protocol/uros_app.c
```

**Group `firmware`**
```
firmware/motor/motor.c
firmware/encoder/encoder.c
firmware/bmi088/bmi088.c
```

**Group `microros`** —— 把 `mcu_ws` 里所有 `.c` 加进来（400+ 个），
建议按包分组建（rcl / rclc / rmw / uxrce / cdr / typesupport / msgs），否则以后定位文件很痛苦。

## 6. main.c 改两处（只在 USER CODE 区）

```c
/* USER CODE BEGIN Includes */
#include "microros_app_includes.h"
/* USER CODE END Includes */
```

```c
  /* USER CODE BEGIN 2 */
  app_main_init();
  HAL_TIM_Base_Start_IT(&htim12);      /* 启动控制节拍中断 */
  /* USER CODE END 2 */

  /* USER CODE BEGIN WHILE */
  while (1)
  {
    app_loop();                        /* micro-ROS 状态机 + 发布 */
  }
  /* USER CODE END WHILE */
```

> `HAL_TIM_Base_Start_IT(&htim12)` 这句必须加 —— CubeMX 只生成 `MX_TIM12_Init()`，
> 不会自动开中断。

## 7. 回调归属（避免重复定义）

| 回调 | 定义在 | 说明 |
|---|---|---|
| `HAL_UART_TxCpltCallback` | `dev_uart.c` | 只管 USART1 |
| `HAL_UART_ErrorCallback` | `dev_uart.c` | 出错后重启接收 |
| `HAL_TIM_PeriodElapsedCallback` | `app_main.c` | 只认 TIM12 |
| `HAL_SPI_TxRxCpltCallback` | `bmi088.c` | 只认 SPI1 |

**工程里不要再定义这四个**，否则链接报重复符号。

## 8. 上板验证顺序（M1）

**第 0 步：先不接 agent，单独确认底层**

1. 烧进去，Keil 里看 `g_boot_errors` 应为 0（说明 IMU 初始化和 UART DMA 都过了）
2. 看 `g_ctrl_ticks` 在涨（TIM12 中断在跑）
3. 给 `/cmd_vel` 前先看 `g_uros_state` 停在 `UROS_STATE_WAIT_AGENT`（正常，因为还没连）

**第 1 步：Jetson 起 agent**

```bash
source /opt/ros/humble/setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 921600 -v6
```

`-v6` 是关键：agent 会打印握手细节，连不上时能立刻看到卡在哪一步。

**第 2 步：看节点上线**

```bash
ros2 node list          # 应出现 /slam_car_mcu
ros2 topic list         # 应有 /odom /imu/data_raw /wheel_speed /cmd_vel
ros2 topic hz /odom     # 应约 30 Hz
```

**第 3 步：验证数据**

```bash
ros2 topic echo /imu/data_raw --once     # 静止时 z 轴加速度 ≈ 9.8
ros2 topic echo /wheel_speed
```

**第 4 步：验证控制（先架空轮子！）**

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.05}, angular: {z: 0.0}}"
```

轮子应正转；停发 0.3 s 后应自动停。

## 9. 已知未验证 / 需要你核对的点

| # | 项 | 怎么核 |
|---|---|---|
| 1 | 四个 transport 回调签名 | 对照你 `mcu_ws` 里 `rmw_microros/rmw_microros.h` 的 `rmw_uros_set_custom_transport` 原型 |
| 2 | `rclc_*_init` 家族签名 | 同上，看 `rclc/rclc.h`；Jazzy 分支有变化 |
| 3 | 消息结构体字段名 | 看 `nav_msgs/msg/odometry.h` 等生成头，本代码按 Humble 写 |
| 4 | 控制节拍频率 | TIM12 现在 Period=1679 → 500 Hz（2 ms）。想要 1 kHz 改成 839。代码读实测 dt，两种都不会算错速度 |
| 5 | 轮径/轮距/编码器线数/减速比 | `board_cfg.h` 里现在是占位值，不标定 odom 尺度全错 |
| 6 | 带宽 | odom 30 Hz + imu 50 Hz ≈ 41 KB/s，921600 下占 45%。想上 100 Hz IMU 要先降 odom |

## 10. 下一步（M2）

- 速度环 PID：在 `app_main.c` 的 `speed_to_duty()` 里换成闭环，`encoder_update` 已提供实测速度
- BMI088 零偏标定：上电静止 5 s 取平均，写进 `s_gyro_bias`
- `/odom` 增加 `twist` 的实测值（当前已填，来自轮速）
