#!/usr/bin/env python3
"""
kalman.launch.py — TAREA 1 (Filtro de Kalman) sin argumentos.

Equivale a:
  ros2 launch tareas_gazebo tareas_gazebo.launch.py world:=vacio.sdf obstaculos:=false

Lanza el bringup principal con el mundo VACIO (sin laberinto ni obstaculos) para
que el seguimiento de trayectoria salga limpio. Aun acepta gui:=false / rviz:=false.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    base = os.path.join(get_package_share_directory('tareas_gazebo'),
                        'launch', 'tareas_gazebo.launch.py')
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(base),
            launch_arguments={
                'world': 'vacio.sdf',
                'obstaculos': 'false',
                'gui': LaunchConfiguration('gui'),
                'rviz': LaunchConfiguration('rviz'),
            }.items(),
        ),
    ])
