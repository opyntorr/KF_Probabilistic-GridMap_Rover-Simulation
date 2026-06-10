from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction


def generate_launch_description():
    return LaunchDescription([
        # 1. Ejecutar plotter
        Node(
            package='tello_control_pos',
            executable='plotter',
            name='plotter',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        
        # 2. Ejecutar position_controller usando solo odometría
        Node(
            package='tello_control_pos',
            executable='position_controller',
            name='position_controller',
            output='screen',
            remappings=[
                ('/odometry/filtered', '/drone1/odom')  # Remapear para que use odometría pura
            ],
            parameters=[
                {'use_sim_time': True},
                {'velocity_scale': 1.0},
                {'kp': 0.4},       # PID más suave para Gazebo
                {'kd': 0.15}       # PID más suave para Gazebo
            ]
        ),
        
        # 3. Visor cámara inferior (ventana separada)
        ExecuteProcess(
            cmd=['ros2', 'run', 'rqt_image_view', 'rqt_image_view', '/drone1/camera_down'],
            output='screen'
        ),

        # 4. Publicar camera_info de la cámara inferior
        Node(
            package='tello_control_pos',
            executable='camera_info_publisher',
            name='camera_info_publisher_down',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),

        # 5. Enviar comando de takeoff con 3 segundos de retraso
        TimerAction(
            period=3.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'ros2', 'service', 'call', 
                        '/drone1/tello_action', 
                        'tello_msgs/srv/TelloAction', 
                        "{cmd: 'takeoff'}"
                    ],
                    output='screen'
                )
            ]
        )
    ])
