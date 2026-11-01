#!/usr/bin/env python3
"""启动 robot_localization EKF（轮速 + IMU vyaw 融合，发布 odom->base_link）"""
import os

from launch import LaunchDescription
from launch_ros.actions import Node

PARAMS = os.path.expanduser('~/ekf_tmini.yaml')


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            parameters=[PARAMS],
            output='screen',
        ),
    ])
