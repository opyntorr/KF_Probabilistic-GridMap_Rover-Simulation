#!/usr/bin/env python3
"""
tareas_gazebo.launch.py — Bringup MINIMO para las dos tareas en Gazebo.

Levanta:
  * Gazebo (mundo laberinto) + puente /clock
  * Robot JetAuto (jetauto_bringup) con odometria nativa de gz ENCENDIDA:
      -> /odom (ground truth)  +  TF odom->base_footprint
      -> /scan, /joint_states (encoders), /cmd_vel -> ruedas
  * (opcional) obstaculos variables para enriquecer el mapa
  * kf_control_node : Filtro de Kalman continuo + control de seguimiento (TAREA 1)
  * gridmap_node    : mapa de ocupacion probabilistico con /scan (TAREA 2)
  * (opcional) RViz

NO incluye teleop ni control_trayectoria (el /cmd_vel lo genera SOLO el KF).

Uso:
  ros2 launch tareas_gazebo tareas_gazebo.launch.py
  ros2 launch tareas_gazebo tareas_gazebo.launch.py rviz:=false obstaculos:=false
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable,
    AppendEnvironmentVariable, TimerAction, IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_sim = get_package_share_directory('mi_proyecto_sim')
    world_file = os.path.join(pkg_sim, 'worlds', 'laberinto.sdf')
    models_dir = os.path.join(pkg_sim, 'models')
    obstaculos_sdf = os.path.join(models_dir, 'obstaculos_var', 'model.sdf')

    use_rviz = LaunchConfiguration('rviz')
    use_obst = LaunchConfiguration('obstaculos')

    args = [
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('obstaculos', default_value='true'),
    ]

    set_env = SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH',
                                     models_dir + ':' + pkg_sim)
    plugin_env = AppendEnvironmentVariable('IGN_GAZEBO_SYSTEM_PLUGIN_PATH',
                                           '/opt/ros/humble/lib')

    gazebo = ExecuteProcess(cmd=['ign', 'gazebo', '-r', world_file], output='screen')

    clock_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
    )

    # Robot con odometria de gz ENCENDIDA (necesitamos /odom como ground truth).
    jetauto = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_sim, 'launch', 'jetauto_bringup.launch.py')),
        launch_arguments={
            'x': '-1.0', 'y': '-1.0', 'z': '0.08', 'yaw': '1.5708',
            'use_sim_time': 'true',
            'publish_sim_odom_tf': 'true',
        }.items(),
    )

    spawn_obst = Node(
        package='ros_gz_sim', executable='create', name='spawn_obstaculos_var',
        condition=IfCondition(use_obst),
        arguments=['-name', 'obstaculos_var', '-file', obstaculos_sdf,
                   '-x', '-2.294', '-y', '2.294', '-z', '0.0', '-R', '1.5708'],
        output='screen',
    )
    spawn_obst_diferido = TimerAction(period=6.0, actions=[spawn_obst])

    kf_node = Node(
        package='tareas_gazebo', executable='kf_control', name='kf_control_node',
        output='screen', parameters=[{'use_sim_time': True}],
    )
    grid_node = Node(
        package='tareas_gazebo', executable='gridmap', name='gridmap_node',
        output='screen', parameters=[{'use_sim_time': True}],
    )
    # Arrancar los nodos cuando el robot/sensores ya esten arriba.
    nodos_diferidos = TimerAction(period=10.0, actions=[kf_node, grid_node])

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        condition=IfCondition(use_rviz),
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription(args + [
        set_env, plugin_env, gazebo, clock_bridge, jetauto,
        spawn_obst_diferido, nodos_diferidos, rviz,
    ])
