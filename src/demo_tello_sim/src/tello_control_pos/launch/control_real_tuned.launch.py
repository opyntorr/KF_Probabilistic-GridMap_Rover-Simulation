from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction, RegisterEventHandler
from launch.event_handlers import OnShutdown

def generate_launch_description():
    return LaunchDescription([
        # 1. Driver del Tello Real (Se conecta por WiFi al dron físico)
        Node(
            package='tello',
            executable='tello',
            name='tello_driver',
            output='screen',
            parameters=[
                {'tello_ip': '192.168.10.1'}  # IP por defecto del Tello
            ]
        ),
        
        # 2. Fusionar poses de OptiTrack y Odometría
        Node(
            package='tello_control_pos',
            executable='pose_fuser',
            name='pose_fuser',
            output='screen',
            remappings=[
                ('/drone1/odom', '/odom'),      # Remapeo al tópico del dron real
                ('/drone_pose', '/optitrack/rigid_body')   # Remapeo al tópico del OptiTrack real
            ],
            parameters=[{'use_sim_time': False}]
        ),
        
        # 3. Graficador
        Node(
            package='tello_control_pos',
            executable='plotter',
            name='plotter',
            output='screen',
            parameters=[{'use_sim_time': False}]
        ),
        
        # 4. Controlador de Posición (¡Con las nuevas ganancias perfectas!)
        Node(
            package='tello_control_pos',
            executable='position_controller',
            name='position_controller',
            output='screen',
            remappings=[
                ('/drone1/cmd_vel', '/control')  # Remapeo al tópico del dron real
            ],
            parameters=[
                {'use_sim_time': False},
                {'velocity_scale': 100.0},  # Escala para el dron real [-100, 100]
                {'kp': 0.5},
                {'ki': 0.06},
                {'kd': 0.35}                 # Freno aerodinámico (filtro EMA interno a 240Hz)
            ]
        ),
        
        # 5. Enviar comando de takeoff al dron real (Retrasado para asegurar conexión)
        TimerAction(
            period=4.0,  # 4 segundos para darle tiempo al nodo tello de conectarse al WiFi
            actions=[
                ExecuteProcess(
                    cmd=[
                        'ros2', 'topic', 'pub', '--once', 
                        '/takeoff', 
                        'std_msgs/msg/Empty', 
                        '{}'
                    ],
                    output='screen'
                )
            ]
        ),
        
        # 6. Al cerrar con Ctrl+C, enviar comando de aterrizaje
        RegisterEventHandler(
            OnShutdown(
                on_shutdown=[
                    ExecuteProcess(
                        cmd=[
                            'ros2', 'topic', 'pub', '--once',
                            '/land',
                            'std_msgs/msg/Empty',
                            '{}'
                        ],
                        output='screen'
                    )
                ]
            )
        )
    ])
