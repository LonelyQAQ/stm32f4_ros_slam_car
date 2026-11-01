#!/usr/bin/env python3
"""T-mini Plus 雷达启动（本地版）

与厂商 ydlidar_launch.py 的唯一区别：
    base_link -> laser_frame 的 yaw = π（实测雷达 0° 与车头相反）。
（网页显示使用此 TF；安全层在扫描角度上独立应用 π 偏移。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node


def generate_launch_description():
    share_dir = get_package_share_directory('ydlidar_ros2_driver')
    parameter_file = LaunchConfiguration('params_file')
    params_declare = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(share_dir, 'params', 'ydlidar.yaml'),
        description='Path to the ROS2 parameters file to use.')

    driver_node = LifecycleNode(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        output='screen',
        emulate_tty=True,
        parameters=[parameter_file],
        namespace='/',
    )
    # x y z qx qy qz qw frame_id child_frame_id
    # 2026-11-01 实测：雷达 0° 与车头相反 → yaw=π（qz=1, qw=0）。
    # 判据：车头正前方 0.3 m 放物体，扫描里该物体落在 −108°~−180° 一侧，即车尾方向。
    # 注意 UI 必须一起用这个 TF（webui.on_scan 已按 map→laser 变换），
    # 否则前端再乘一次位姿会叠加成 360°/错乱。
    tf2_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_laser',
        arguments=['0', '0', '0.02', '0', '0', '1', '0', 'base_link', 'laser_frame'],
    )
    # 30 Hz 本地记录（~/logs/car_*.txt），随雷达启动；脚本内有单实例锁
    logger_node = ExecuteProcess(
        cmd=['python3', os.path.expanduser('~/data_logger.py')],
        output='screen',
        name='data_logger',
    )
    return LaunchDescription([params_declare, driver_node, tf2_node,
                              logger_node])
