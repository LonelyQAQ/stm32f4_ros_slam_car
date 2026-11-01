# SLAM 建图小车（STM32F407 + Jetson Orin Nano）

房间内的 2D SLAM 建图、静态地图定位、Nav2 自主导航与定点返航。
双轮差速底盘：STM32F407 负责电机闭环、编码器、IMU 与硬实时；Jetson Orin Nano 负责激光雷达、
Cartographer 建图、EKF、AMCL 定位、Nav2 导航和 Web 控制台。v1 无视觉。

**当前验收结果**（地图 `20261101_193038`，车位姿由 AMCL 给出）：

| 项目 | 结果 |
|---|---|
| 返航终点误差 | **7.3 cm**（dx -6.4 / dy 3.5），航向 **0.2°** |
| 停车后 AMCL 收敛跳变 | **3.1 cm** |
| 现场实测物理误差 | 约 **8 cm** |
| 任意位置全局定位 | inlier 0.89–0.95，平均距离 0.05–0.07 m |
| 静止陀螺零偏 | 0.0012 °/s（约 0.1 °/min），29 s 积分漂移 0.0° |

---

## 1. 系统结构

```
                    ┌──────────────────────────── Jetson Orin Nano (ROS 2 Humble) ───────────────────────────┐
 YDLIDAR T-mini Plus│  /scan ──► safety_guard ─┐                                                             │
      (USB)         │                          ├─► /cmd_vel_safe ──► frame_bridge ──► UART6 921600 ──┐        │
                    │  /cmd_vel_manual (webui) ┤                                                     │        │
                    │  /cmd_vel (Nav2) ──► cmd_mux ──► /cmd_vel_raw                                   │        │
                    │                                                                                 ▼        │
                    │  EKF: 轮速 vx + IMU vyaw ──► odom→base_link                      STM32F407VGT6      │
                    │  Cartographer（建图）或 AMCL（定位）──► map→odom                  TB6612 / 编码器     │
                    │  Nav2 / Web 控制台 :8080                                          BMI088 / VQF 6D     │
                    └─────────────────────────────────────────────────────────────────────────────────┘
```

- **上行**：MCU → Jetson 自定义二进制帧（TELEM 90 B @50 Hz）。
- **下行**：Jetson → MCU 速度指令（CMD 9 B @20 Hz，含制动标志）。
- **TF**：`map→odom`（Cartographer 或 AMCL）、`odom→base_link`（EKF）、`base_link→laser_frame`（静态，yaw=π）。
- **控制链**：`/cmd_vel_manual` + `/cmd_vel` → `cmd_mux` → `/cmd_vel_raw` → `safety_guard` → `/cmd_vel_safe` → `frame_bridge` → MCU。
  **少启动 `cmd_mux` 或 `safety_guard`，网页发出的速度到不了电机。**

---

## 2. 硬件

| 部件 | 型号 / 参数 | 说明 |
|---|---|---|
| 主控 | 天空星 STM32F407VGT6，168 MHz | 8 MHz HSE，PLL 168 MHz；APB1 /4、APB2 /2 |
| 电机驱动 | TB6612 | PWMA/PWMB 调速，AIN/BIN 方向 |
| 电机 | 1:30 减速，输出轴 320 rpm | 空载 1.089 m/s（65 mm 轮） |
| 编码器 | 11 线，四倍频，减速前 | CPR = 11×4×30 = **1320 counts/轮转** |
| 轮径 / 轮距 | 65 mm / 180 mm（轮距待实测） | 0.1547 mm/count |
| IMU | BMI088，SPI1，6 轴 | VQF 6D，200 Hz；无磁力计 |
| 激光雷达 | YDLIDAR T-mini Plus | 0.1–6.0 m，约 8–10 Hz，442 点 |
| 上位机 | Jetson Orin Nano，Ubuntu 22.04 + ROS 2 Humble | 通过 USB 网口 / WiFi 访问 Web |

### 2.1 MCU 引脚

| 功能 | MCU 引脚 | 资源 |
|---|---|---|
| 编码器 1 A/B | PA0 / PA1 | TIM2_CH1/CH2（硬件正交解码） |
| 编码器 2 A/B | PD12 / PD13 | TIM4_CH1/CH2 |
| 电机 A 调速 / 方向 | PC8 / PC0 / PC1 | TIM3_CH3；AIN1/AIN2 |
| 电机 B 调速 / 方向 | PC9 / PC2 / PC3 | TIM3_CH4；BIN1/BIN2 |
| BMI088 SPI1 | PA5 SCK / PA6 MISO / PA7 MOSI | CS：PC4（加速度）、PC5（陀螺） |
| Jetson 串口 | PC6 TX / PC7 RX | **USART6（端子 P2，3.3V）**，921600 |
| 调试串口 | PB6 / PB7 | USART1，5V 电平 |
| 屏幕（软件 SPI） | PE7–PE11 | SCLK/MOSI/DC/RST/CS |
| 按键 / 蜂鸣器 | PE0–PE3 / PE5 | 低有效 / NPN |

> 硬件说明：原理图里的 `BMI088_CSBI`（PB0）是废网络（拼写错误），BMI088 只用 PC4/PC5 两个片选。
> 雷达 0° 与车头**相反**，所以静态 TF 的 yaw = π；`safety_guard` 的 `scan_yaw_offset_rad` 也同步为 π。

---

## 3. MCU 固件（`stm32/`）

### 3.1 目录分层

```
stm32/
├── app/
│   ├── System/      app_main.c  app_cfg.h  控制环、状态机、配置
│   ├── Protocol/    frame_link.c           自定义二进制帧
│   └── Device/      电机、编码器、IMU 设备层
├── firmware/
│   ├── board/       board_cfg.h            轮径/轮距/PID/编码器
│   ├── motor/       TB6612 + PWM
│   ├── encoder/     正交解码与计数
│   └── vqf/         VQF 6D 姿态解算
├── Core/            CubeMX 生成的 HAL 初始化
├── Drivers/         STM32F4 HAL
└── MDK-ARM/         Keil 工程 ROS_STM32_Test.uvprojx
```

### 3.2 关键配置

`app/System/app_cfg.h`：

```c
#define APP_ENABLE_FRAME 1      // 自定义帧链路（当前使用）
#define APP_ENABLE_UROS  0      // micro-ROS 未启用
#define APP_CMD_TIMEOUT_MS 300  // 300 ms 收不到指令 → 滑行停车
#define APP_V_MAX_MPS 0.35f     // 满占空比折算速度（前馈用）
#define APP_W_MAX_RPS 1.20f
#define APP_IMU_BODY_X (-2)     // 车头 = 传感器 -Y
#define APP_IMU_BODY_Y (+1)
#define APP_IMU_BODY_Z (+3)
```

`firmware/board/board_cfg.h`：

```c
#define WHEEL_DIAMETER_M 0.065f     // 轮毂 65 mm（雷达法复核 k=1.0127）
#define WHEEL_TRACK_M    0.180f     // 待实测
#define ENCODER_LINES    11.0f
#define GEAR_RATIO       30.0f
#define WHEEL_PID_KP     5000.0f
#define WHEEL_PID_KI     30000.0f
#define WHEEL_PID_KD     30.0f
#define WHEEL_PID_DEADBAND_MPS 0.005f
```

### 3.3 控制环

- TIM12 1 kHz 节拍：读编码器、积分位姿。
- 每 10 ms（100 Hz）跑一次轮速 PID：10 ms 测速窗口、前馈 + 抗饱和 + 微分低通 + 零速死区。
- 速度指令超时 300 ms 自动滑行；制动只由 CMD 的 bit0 触发（最长 500 ms，防止 H 桥长时间短路）。
- VQF 6D：200 Hz、`Ts=0.005`、`tauAcc=3 s`；ZUPT 零速修正；静止陀螺零偏约 0.1 °/min。

### 3.4 二进制帧（`app/Protocol/frame_link.c`）

```
0xAA 0x55 | type(1) | len(1) | payload(len) | crc16(2, LE)
crc16 = CRC-16/CCITT-FALSE（初值 0xFFFF，多项式 0x1021），覆盖 type|len|payload

TELEM 0x01  90 B @50 Hz
  tick / x y th / v w / 左右编码器计数 / gyro[3] / acc[3] / temp
  / cmd_v cmd_w（回显）/ 四元数 q[4] / bias_z / rest / zupt

CMD   0x02   9 B @20 Hz
  float v, float w, uint8 flags（bit0 = 制动请求，未知位忽略）
```

### 3.5 编译与烧录

- 打开 `stm32/MDK-ARM/ROS_STM32_Test.uvprojx`（Keil MDK，ARM Compiler 6）。
- 命令行编译（示例）：

```bat
D:\Keil\Core\UV4\UV4.exe -j0 -b ROS_STM32_Test.uvprojx -o build.log
```

- 烧录：Keil 里选择 **CMSIS-DAP** 调试器（SWD），Download / Verify。
  本机没有 OpenOCD/GCC 工具链，统一用 Keil。
- 编译产物（`*.axf/*.hex/*.map` 等）已在 `.gitignore` 中忽略。

---

## 4. Jetson 软件栈（`jetson/`）

### 4.1 依赖

- Ubuntu 22.04 + ROS 2 Humble
- `cartographer_ros`、`nav2_bringup`、`robot_localization`、`ydlidar_ros2_driver`
- Python 3：`numpy`、`Pillow`（地图预览）

### 4.2 部署

把 `jetson/` 下的文件复制到 `/home/nvidia/`（脚本按绝对路径约定，配置文件名保持不变）：

```
frame_bridge.py  cmd_mux.py  safety_guard.py  nav_manager.py  webui.py  webui.js  webui.html
webui_state.py   slam_manager.py  localization_manager.py  map_tools.py  map_localize.py
data_logger.py   odom_tape.py  odom_lidar_calib.py
car_aux.yaml  ekf_tmini.yaml  ekf_tmini.launch.py
cartographer_tmini.lua  cartographer_tmini.launch.py
amcl_tmini.yaml  nav2_tmini.yaml  ydlidar_tmini.launch.py
carctl
```

### 4.3 启动顺序

**顺序很重要**（缺 `cmd_mux`/`safety_guard` 电机不动；缺 EKF 没有 `odom→base_link`）：

```bash
# 1) 基础栈：雷达 + 桥接 + EKF + 仲裁 + 安全层 + Web
bash ~/carctl up

# 2) 建图：网页点「开始建图」（Cartographer）
#    或手动：
#    ros2 launch ~/cartographer_tmini.launch.py

# 3) 定位：网页地图库选图 →「使用」（map_server + AMCL + Nav2）
```

`carctl` 命令：

```bash
bash ~/carctl up       # 雷达 + frame_bridge + EKF + cmd_mux + safety_guard + webui
bash ~/carctl down     # 全部停止（含 Cartographer / AMCL / Nav2）
bash ~/carctl nav on   # 单独启动 Nav2
bash ~/carctl nav off
bash ~/carctl status
```

浏览器打开 `http://<jetson-ip>:8080`。

### 4.4 节点与话题

| 节点/脚本 | 作用 | 主要输出 |
|---|---|---|
| `ydlidar_tmini.launch.py` | 雷达驱动 + `base_link→laser_frame`（yaw=π）+ 数据记录 | `/scan` |
| `frame_bridge.py` | UART ↔ ROS 桥接 | `/odom /imu/data_raw /imu/temp /imu/diag /wheel_speed /enc_ticks /cmd_echo` |
| `ekf_tmini.launch.py` | robot_localization | `/odometry/filtered`、`odom→base_link` |
| `cmd_mux.py` | 手动/自动仲裁 | `/cmd_vel_raw` |
| `safety_guard.py` | 0.5 m 近距避障 | `/cmd_vel_safe`、`/safety_guard/state` |
| `cartographer_tmini.launch.py` | 建图 | `/map`、`map→odom` |
| `localization_manager.py` | map_server + AMCL 启停 | `/map`、`map→odom`、`/amcl_pose` |
| `nav_manager.py` | 目标/返航/模式/初始位姿 | NavigateToPose、`/nav_mode`、`/initialpose` |
| `webui.py` | Web 控制台 + 状态汇总 | `/cmd_vel_manual`、`/webui/status` |
| `data_logger.py` | 30 Hz 本地记录 | `~/logs/car_*.txt` |

常用话题：

```
/odom /odometry/filtered /imu/data_raw /imu/temp /imu/diag /wheel_speed /enc_ticks /cmd_echo
/scan /map /plan /amcl_pose /initialpose /nav_mode
/cmd_vel_manual /cmd_vel /cmd_vel_raw /cmd_vel_safe /cmd_brake
/safety_guard/state /webui/status
```

### 4.5 EKF（`ekf_tmini.yaml`）

- 50 Hz，`two_d_mode`，`world_frame: odom`。
- `odom0 = /odom`：只吃 `vx`（轮速），不吃轮速差角速度。
- `imu0 = /imu/data_raw`：只吃 `vyaw`（陀螺）。
- 绝对航向不参与融合：6D IMU 无磁力计，轮速差在打滑时会跑飞；航向交给 `map→odom` 的 Cartographer/AMCL。

### 4.6 安全层（`car_aux.yaml`）

| 参数 | 值 | 说明 |
|---|---|---|
| `avoid_distance_m` | 0.5 | 前/后 40° 内障碍触发 |
| `release_distance_m` | 0.7 | 解除门限（滞后） |
| `confirm_scans` / `release_scans` | 3 / 3 | 连续帧确认，防抖 |
| `min_cluster_points` | 3 | 点簇最小点数 |
| `side_clearance_m` | 0.3 | 侧向净空 |
| `scan_yaw_offset_rad` | π | 雷达 0° 与车头相反 |

触发时只压制对应方向：能在障碍前停下；后退/转向仍可用。

### 4.7 Web 控制台

- 画布：地图、实时点云、路径、车位姿；滚轮缩放、拖动平移、点击设导航目标。
- 行驶：WASD/按钮，松手 400 ms 死手停车；急停会取消 Nav2、切手动、制动 0.5 s。
- 导航：启动/关闭 Nav2、手动/自动互斥、返航原点、设为原点、取消；显示剩余距离、原点、距原点、末次误差。
- 建图：开始建图、建图完成（导出 raw/map/preview/meta）、停止节点。
- 地图库：下拉选择、**使用**、校验、删除；显示「定位中 · 匹配 inlier/均距」。
- 定位：设初始位姿（点击地图）、**重新全局定位**（车必须静止）。
- IMU：显示层置零（不改 MCU 零偏）。

### 4.8 建图与地图库

- 建图引擎：Cartographer 2D + IMU + 轮速（`use_odometry=true`），`published_frame=odom`。
- 只允许 **1 段** 激光（实测 2/4 段会角度跳变/CPU 飙升）。
- 「建图完成」调用 `finish_trajectory` + `write_state`，导出：

```
~/maps/<YYYYmmdd_HHMMSS>/
├── raw.pgm / raw.yaml      建图原始产物
├── map.pgm / map.yaml      形态学降噪成品（导航用）
├── map.pbstream            Cartographer 位姿图
├── preview.png             降噪前后对比
└── meta.json               元数据 + 拟合检查
```

- 降噪：障碍格 8 邻域 ≤1 个邻居 → 去孤立点；空闲格 ≥6 个障碍邻居 → 补裂缝；只处理 occupied 层。
- 拟合检查：扫描点 vs 地图障碍，`inlier ≥ 0.60` 且平均距离 ≤0.15 m 通过；
  原始图与降噪图对比过（0.91/0.069 vs 0.90/0.073），**降噪不引入偏移**。

### 4.9 定位（AMCL）

- 地图模式：`map_server` + `nav2_amcl`，地图静态；建图模式与定位模式互斥。
- 首次「使用」：单帧扫描对地图做全局搜索（`map_localize.py`）：
  自由格抽稀 0.1 m、10° 粗搜 → 0.1 m/5°、0.05 m/2° 精修；
  接受条件 `inlier ≥ 0.55` 且 `mean ≤ 0.12 m`，否则回退 map 原点并提示手动设初始位姿。
- 连续跟踪：`update_min_d/a = 0.03`，AMCL `max_beams 120`、粒子 1000–3000、
  `sigma_hit 0.12`、`z_hit 0.7 / z_rand 0.2`。
- 同一张图重复点「使用」不会重启 AMCL；重复点击由 `_map_op` 串行化。
- 后台监控（`LocMonitor`）：1 Hz 用 AMCL 位姿把扫描投到地图上算吻合度；
  连续 3 帧失配且车静止、未导航 → 自动全局重定位（6/12/24/48/96 s 退避）。
- 低置信度不硬跳；可手动「重新全局定位」（行驶中拒绝）。
- 建图/定位切换：先停旧发布者再起新的，`map→odom` 永远只有一个发布者。

### 4.10 导航（Nav2）

- DWB 局部控制器：`max_vel_x 0.25`、`max_vel_theta 0.8`、`sim_time 1.7`。
- 到点判据：`xy_goal_tolerance 0.05`、`yaw_goal_tolerance 0.08`。
- 进度检查：`required_movement_radius 0.10`、`movement_time_allowance 15 s`
  （原来 0.3 m/10 s，会在终点反复 abort + spin）。
- 返航原点：1 s 内取 10 帧 `map→base_link` 中位数，要求 `loc_quality.ok` 且 `inlier ≥ 0.8`；
  home 与地图会话绑定，切图/重启后失效。
- 到点后 1 s 再测一次位姿，网页显示「末次误差 / 停车后跳变」。
- 自动微调默认 **关闭**（`home_refine_m=0`）：13 cm 级近距离目标 DWB 只生成旋转轨迹，会原地打转。

---

## 5. 标定

### 5.1 轮径（雷达当尺子）

用激光雷达测墙距变化作为真实位移：

```bash
source /opt/ros/humble/setup.bash
python3 ~/odom_lidar_calib.py
```

1. 车头对准一面平墙（偏角 <5°），距墙 1.2–2 m；
2. 记下 `d0`，慢速直行 0.8–1.5 m，停稳；
3. `Ctrl+C`，脚本给出 `k = ((d0-d1)/cos偏角) / 轮速位移` 和建议轮径。

实测：d0 1.978 → d1 0.857 m，轮速位移 1.110 m → **k=1.0127**（轮径 65.8 mm），
在雷达 ±1 cm 的误差内，**65 mm 保持不变**。

### 5.2 陀螺

```bash
python3 ~/gyro_drift.py     # 静止 30 s
```

实测 `gz` 均值 0.0012 °/s（0.1 °/min），29 s 积分漂移 0.0°；运动中陀螺标度仍建议用原地 5 圈复核。

### 5.3 激光安装角

`base_link→laser_frame` 为 `(0,0,0.02)`、yaw=π；雷达 x/y 偏移尚未实测，贴墙直行可复核。

---

## 6. 数据记录

- 随雷达 launch 自动启动（`~/data_logger.py`，单实例锁 `/tmp/car_data_logger.lock`）。
- **30 Hz**，一行一个采样，47 列，带表头；Jetson 本地时间（ISO + 毫秒）。
- 内容：IMU（姿态/角速度/加速度/温度）、MCU 位姿/速度/编码器/指令回显、EKF、
  雷达摘要（点数/最小距离/正前墙距）、`/imu/diag`、导航/定位/安全状态、CPU/负载/内存。
- 文件：`~/logs/car_YYYYmmdd_HHMMSS.txt`，每 **30 min** 换文件；
  **超过 30 min 的旧文件自动删除**；`~/logs/latest.txt` 指向当前文件。

```bash
tail -f ~/logs/latest.txt
cp ~/logs/latest.txt ~/logs/keep_$(date +%H%M).txt   # 需要留档时先复制
```

> 电机占空比目前不在 MCU 遥测里；需要的话在 `telem_t` 里加字段并同步 `frame_bridge.py`。

---

## 7. 目录结构

```
.
├── stm32/                 Keil MDK 工程（Core/Drivers/app/firmware）
├── jetson/
│   ├── frame_bridge.py    UART ↔ ROS
│   ├── cmd_mux.py         手动/自动仲裁
│   ├── safety_guard.py    近距避障
│   ├── nav_manager.py     Nav2 目标/返航/模式/初始位姿
│   ├── webui.py/.js/.html Web 控制台
│   ├── webui_state.py     共享状态
│   ├── slam_manager.py    Cartographer 启停 / 建图完成
│   ├── localization_manager.py  map_server + AMCL 启停
│   ├── map_tools.py       PGM/降噪/拟合检查/地图库
│   ├── map_localize.py    单帧全局定位
│   ├── data_logger.py     30 Hz 本地记录
│   ├── odom_tape.py       直行标定助手
│   ├── odom_lidar_calib.py 雷达墙距标定
│   ├── *_tmini.yaml/lua/launch.py  参数与启动
│   └── carctl            一键启停
└── README.md
```

---

## 8. 故障排查

| 现象 | 原因 / 处理 |
|---|---|
| 网页发速度但电机不动 | 确认 `cmd_mux`、`safety_guard`、`frame_bridge` 都在跑；`/cmd_echo` 是否收到；电机供电是否打开 |
| 地图上下颠倒 | OccupancyGrid 行 0 在底部，前端要翻转；已在 `webui` 处理 |
| 点云与地图错位 180° | 雷达 0° 与车头相反：静态 TF yaw=π，`safety_guard`/`webui` 的扫描偏移同步 π |
| 车前进方向与导航箭头相反 | 同上；另检查 IMU 安装矩阵 `-2/+1/+3` |
| 「使用」后再点一次把定位杀掉 | 已修：同图不重启 AMCL；lifecycle 用 `GetState` |
| 网页一直加载/刷新不了 | SSE 连接泄漏已修（重连先 close + 服务端上限 3 条）；关掉多余标签页 |
| 终点附近原地打转 | 近距离目标 DWB 只生成旋转轨迹；自动微调默认关闭 |
| Nav2 到点误差 15 cm | 旧 `xy_goal_tolerance=0.15`；已改 0.05，并修 `progress_checker` |
| AMCL 报 “cannot publish a pose” | 还没设初始位姿；点「使用」或「设初始位姿」 |
| 没有 `odom→base_link` | EKF 没启动：`ros2 launch ~/ekf_tmini.launch.py`（`carctl up` 已包含） |
| `ros2`/`rclpy` 找不到 | 先 `source /opt/ros/humble/setup.bash` |

---

## 9. 已知限制与下一步

- 地图是带轮速先验建的，局部可能有几厘米形变；要稳定进入 5 cm 以内，建议**重走闭合回路建图**。
- 轮径用雷达法复核到 1.3%；轮距/CPR 仍是标称值，原地转向比例未单独标定。
- BMI088 无磁力计，绝对航向只能靠 AMCL；长时间遮挡/相似场景可能歧义，需手动设初始位姿。
- 电机占空比未进遥测；数据记录里只有指令速度和实际轮速。
- 墙角掉头/窄道仍需实车多跑；安全层 0.5 m 避障与 0.35 m 膨胀会让贴墙原点难以精确靠近。
- 自动微调默认关闭；需要时可把 `home_refine_m` 设成 0.08，但近距离目标有原地打转风险。

---

## 10. 安全

- 调试时先架空或断开电机供电；首次上电先低速（0.1 m/s）。
- 急停：网页「急停」会取消 Nav2、切手动、制动 0.5 s；MCU 另有 300 ms 指令超时兜底。
- 激光雷达 0.1–6.0 m；近距避障只在确认 3 帧后触发，解除需 3 帧。
