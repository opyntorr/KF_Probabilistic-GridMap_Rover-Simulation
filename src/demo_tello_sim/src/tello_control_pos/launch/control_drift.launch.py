from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction


def generate_launch_description():
    return LaunchDescription([
        # 1. Fusionar poses de OptiTrack y Odometría
        Node(
            package='tello_control_pos',
            executable='pose_fuser',
            name='pose_fuser',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        
        # 3. Ejecutar plotter
        Node(
            package='tello_control_pos',
            executable='plotter',
            name='plotter',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        
        # 4. Ejecutar position_controller (remapeando salida para inyectar drift)
        Node(
            package='tello_control_pos',
            executable='position_controller',
            name='position_controller',
            output='screen',
            remappings=[
                ('/drone1/cmd_vel', '/drone1/cmd_vel_clean')
            ],
            parameters=[
                {'use_sim_time': True},
                {'velocity_scale': 1.0},
                {'kp': 0.5},
                {'ki': 0.06},
                {'kd': 0.35}
            ]
        ),
        
        # 4.5 Ejecutar simulador de drift
        Node(
            package='tello_control_pos',
            executable='drift_simulator',
            name='drift_simulator',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                {'drift_magnitude': 0.05}  # Viento masivo en simulación
            ]
        ),
        
        # 5. Visor cámara inferior (ventana separada)
        ExecuteProcess(
            cmd=['ros2', 'run', 'rqt_image_view', 'rqt_image_view', '/drone1/camera_down'],
            output='screen'
        ),

        # 7. Publicar camera_info de la cámara inferior sincronizado con las imágenes
        Node(
            package='tello_control_pos',
            executable='camera_info_publisher',
            name='camera_info_publisher_down',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),

        # 8. Ejecutar optitrack_simulator
        Node(
            package='tello_control_pos',
            executable='optitrack_simulator',
            name='optitrack_simulator',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                {'latency_sec': 0.005}
            ]
        ),
        
        # 9. Enviar comando de takeoff con 3 segundos de retraso para asegurar que la simulación esté lista
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
