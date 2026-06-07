#!/usr/bin/env python3
"""robot_state_publisher con el URDF mecanum (árbol TF base_footprint->lidar_frame).
Pasa robot_description como string de Python (evita el parseo YAML de la CLI que
rompe con el XML multilínea). use_sim_time=true (Isaac /clock).
Uso:  ros2 launch isaac/rsp.launch.py
"""
import os

from launch import LaunchDescription
from launch_ros.actions import Node

HERE = os.path.dirname(os.path.abspath(__file__))


def generate_launch_description():
    urdf = os.path.join(HERE, 'assets', 'jetauto_mecanum.urdf')
    with open(urdf) as f:
        robot_description = f.read()
    return LaunchDescription([
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             output='screen',
             parameters=[{'robot_description': robot_description, 'use_sim_time': True}]),
    ])
