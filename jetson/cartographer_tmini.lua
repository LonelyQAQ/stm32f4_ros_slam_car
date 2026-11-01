-- Cartographer 2D 配置：YDLIDAR T-mini Plus + BMI088(VQF 6D) + 轮式 EKF
--
-- 架构约定：
--   map->odom            ← Cartographer 发布（published_frame=odom，provide_odom_frame=false）
--   odom->base_link      ← robot_localization EKF（轮速 vx + IMU vyaw）
--   base_link->laser_frame ← 静态 TF（z=0.02，yaw=π）
--   IMU 数据已在固件里补偿 90° 安装偏置，frame_id 直接是 base_link
--
-- 用法见 cartographer_tmini.launch.py

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,

  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "odom",
  odom_frame = "odom",
  provide_odom_frame = false,      -- odom->base_link 由 EKF 提供，绝不重复发布
  publish_frame_projected_to_2d = true,

  use_odometry = true,             -- 订阅 /odom（桥接的轮速里程计）
  use_nav_sat = false,
  use_landmarks = false,

  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  -- 2026-11-01 实测结论：本机只能用 1 段。
  --   1 段 → CPU 11%，稳定
  --   2 段 → CPU 30~62%，**实车角度跳变剧烈**（拆段要求逐点时间戳与 IMU/odom 严格对齐，
  --          本驱动只填了 time_increment、header.stamp 与逐点时间不自洽）
  --   4 段 → CPU 239%，收益仅再降一半，不划算
  -- 抑制运动畸变改用「抬高雷达频率（10~12 Hz）」或「导航侧限速」。
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,

  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,

  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

-- ---- 2D 轨迹构建 ----
TRAJECTORY_BUILDER_2D.use_imu_data = true          -- 关键：2D 也吃 IMU（抑制旋转漂移）
TRAJECTORY_BUILDER_2D.min_range = 0.05             -- 与雷达 range_min 一致
TRAJECTORY_BUILDER_2D.max_range = 12.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 1.0
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.2)

-- 单线雷达 0.71°/点，360° 共 506 点：自适应体素过滤按 0.03 m 保留细节
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 0.3
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 200
TRAJECTORY_BUILDER_2D.loop_closure_adaptive_voxel_filter.max_length = 0.3

-- ---- 位姿图 ----
POSE_GRAPH.optimization_problem.huber_scale = 1e2
POSE_GRAPH.optimize_every_n_nodes = 90
POSE_GRAPH.constraint_builder.min_score = 0.55
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.6

return options
