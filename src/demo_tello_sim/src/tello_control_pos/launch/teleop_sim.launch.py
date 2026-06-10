"""
Launch teleop con Xbox para Tello en SIMULACION (Gazebo).

Uso:
  ros2 launch tello_control_pos teleop_sim.launch.py

Prerequisitos:
  - Simulacion Gazebo con el dron spawneado
  - Control Xbox conectado por USB o Bluetooth
"""
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory('tello_control_pos')
    config = os.path.join(pkg, 'config', 'xbox_tello.yaml')

    # Nodo joy: lee el control Xbox y publica a /joy
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'deadzone': 0.15,
            'autorepeat_rate': 20.0,
        }]
    )

    # Nodo teleop: traduce /joy a /drone1/cmd_vel
    teleop_node = Node(
        package='tello_control_pos',
        executable='tello_joy_teleop',
        name='tello_joy_teleop',
        output='screen',
        parameters=[
            ParameterFile(config, allow_substs=True),
            {
                'use_sim_time': True,
                'velocity_scale': 1.0,
                'takeoff_mode': 'service',
            }
        ]
    )

    camera_viewer = ExecuteProcess(
        cmd=['ros2', 'run', 'rqt_image_view', 'rqt_image_view', '/drone1/camera_down'],
        output='screen'
    )

    camera_info_node = Node(
        package='tello_control_pos',
        executable='camera_info_publisher',
        name='camera_info_publisher_down',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        joy_node,
        teleop_node,
        camera_viewer,
        camera_info_node,
    ])
