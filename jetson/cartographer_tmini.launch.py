#!/usr/bin/env python3
"""Cartographer 2D 启动（建图 / 定位两种模式）

TF 分工：
  map->odom              本节点
  odom->base_link        robot_localization 的 ekf_node（必须已在跑）
  base_link->laser_frame 静态 TF（ydlidar launch 里给，z=0.02，yaw=π）

启动参数：
  config                配置文件名，默认 cartographer_tmini.lua
  load_state_filename   载入已有 .pbstream（给了就走定位模式）
  use_grid              是否启动栅格发布节点
  resolution            栅格分辨率
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = 'cartographer_ros'
CONFIG_DIR = os.path.expanduser('~')


def _node(context):
    config = LaunchConfiguration('config').perform(context)
    load_state = LaunchConfiguration('load_state_filename').perform(context)

    arguments = ['-configuration_directory', CONFIG_DIR,
                 '-configuration_basename', config]
    if load_state:
        arguments += ['-load_state_filename', load_state, '-load_future_trajectory']

    return [Node(
        package=PKG,
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        arguments=arguments,
        remappings=[('scan', '/scan'),
                    ('imu', '/imu/data_raw'),
                    ('odom', '/odom')],
    )]


def generate_launch_description():
    args = [
        DeclareLaunchArgument('config', default_value='cartographer_tmini.lua'),
        DeclareLaunchArgument('load_state_filename', default_value=''),
        DeclareLaunchArgument('use_grid', default_value='true'),
        DeclareLaunchArgument('resolution', default_value='0.05'),
    ]

    occupancy_grid_node = Node(
        package=PKG,
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        arguments=['-resolution', LaunchConfiguration('resolution'),
                   '-publish_period_sec', '1.0'],
        condition=IfCondition(LaunchConfiguration('use_grid')),
    )

    return LaunchDescription(args + [OpaqueFunction(function=_node),
                                     occupancy_grid_node])
