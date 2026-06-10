"""
Launch teleop con Xbox para Tello REAL.

Uso:
  ros2 launch tello_control_pos teleop_real.launch.py

Prerequisitos:
  - PC conectado al WiFi del Tello (TELLO-XXXXXX)
  - Control Xbox conectado por USB o Bluetooth

Flujo:
  1. Se conecta al dron real via driver
  2. Presiona A para takeoff
  3. Sticks: izq=XY, der=yaw; gatillos=Z (RT sube, LT baja)
  4. Presiona Y para tomar foto (se guarda en PHOTO_DIR)
  5. Presiona B para land o X para emergency
  6. Ctrl+C envia land automaticamente
"""
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnShutdown
from ament_index_python.packages import get_package_share_directory

PHOTO_DIR = '/tmp/tello_fotos'


def generate_launch_description():
    pkg = get_package_share_directory('tello_control_pos')
    config = os.path.join(pkg, 'config', 'xbox_tello.yaml')

    # Borrar y recrear la carpeta de fotos en cada arranque
    setup_photo_dir = ExecuteProcess(
        cmd=['bash', '-c', f'rm -rf {PHOTO_DIR} && mkdir -p {PHOTO_DIR}'],
        output='screen'
    )

    # Driver del Tello real (escucha: takeoff, land, emergency, control)
    tello_driver = Node(
        package='tello',
        executable='tello',
        name='tello_driver',
        output='screen',
        parameters=[{'tello_ip': '192.168.10.1'}]
    )

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

    # Nodo teleop: traduce /joy a comandos del dron real
    # velocity_scale=100 porque el driver espera enteros -100..100
    teleop_node = Node(
        package='tello_control_pos',
        executable='tello_joy_teleop',
        name='tello_joy_teleop',
        output='screen',
        parameters=[
            ParameterFile(config, allow_substs=True),
            {
                'velocity_scale': 100.0,
                'takeoff_mode': 'topic',
                'photo_button': 3,        # Y
                'photo_save_dir': PHOTO_DIR,
            }
        ],
        remappings=[
            ('/drone1/cmd_vel', '/control'),
        ]
    )

    # Visor de video: muestra /image_raw en una ventana
    image_view_node = Node(
        package='image_view',
        executable='image_view',
        name='tello_video',
        output='screen',
        remappings=[('image', '/image_raw')],
        parameters=[{'autosize': True}]
    )

    # Al cerrar con Ctrl+C, enviar land de seguridad
    shutdown_handler = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[
                ExecuteProcess(
                    cmd=[
                        'ros2', 'topic', 'pub', '--once',
                        '/land', 'std_msgs/msg/Empty', '{}'
                    ],
                    output='screen'
                )
            ]
        )
    )

    return LaunchDescription([
        setup_photo_dir,
        tello_driver,
        joy_node,
        teleop_node,
        image_view_node,
        shutdown_handler,
    ])
