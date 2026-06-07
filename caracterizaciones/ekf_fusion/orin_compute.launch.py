#!/usr/bin/env python3
"""
Cómputo del lado ORIN para la arquitectura BRIDGE (la base/lidar/cámara corren en la
Nano y llegan por DDS). Aquí NO se lanza chassis/lidar; solo lo que el Orin debe calcular:
  - robot_state_publisher : URDF -> TF estático (base_footprint->base_link->lidar_frame, ...)
  - imu_filter_madgwick   : /imu/data_raw (de la Nano) -> /imu/data
  - ekf_filter_node       : /odom_raw (de la Nano) + /imu/data -> /odom + TF odom->base_footprint

Encima de esto se corre SLAM/Nav2 y el cerebro AGV (mision_jetauto.launch.py start_base:=false).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    ctrl = get_package_share_directory('jetauto_controller')
    desc = get_package_share_directory('jetauto_description')
    ekf_params = os.path.join(ctrl, 'config', 'ekf.yaml')

    return LaunchDescription([
        # URDF -> TF (robot_state_publisher)
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(desc, 'launch', 'description.launch.py'))),

        # IMU madgwick: consume /imu/data_raw (de la Nano) -> /imu/data
        Node(package='imu_filter_madgwick', executable='imu_filter_madgwick_node', name='imu_filter',
             output='screen',
             parameters=[{'use_mag': False, 'publish_tf': False, 'world_frame': 'enu'}]),

        # EKF: consume /odom_raw (de la Nano) + /imu/data -> /odom + TF odom->base_footprint
        Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node',
             output='screen', parameters=[ekf_params],
             remappings=[('odometry/filtered', 'odom')]),
    ])
