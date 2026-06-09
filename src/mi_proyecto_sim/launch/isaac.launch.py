import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction, IncludeLaunchDescription, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_sim = get_package_share_directory('mi_proyecto_sim')
    
    # SLAM Toolbox is the default for this mode
    ws_root = os.path.abspath(os.path.join(pkg_sim, '..', '..', '..', '..'))
    maps_dir = os.path.join(ws_root, 'src', 'mi_proyecto_sim', 'maps')
    mapa_pgm = os.path.join(maps_dir, 'mapa_mision.pgm')
    mapa_yaml = os.path.join(maps_dir, 'mapa_mision.yaml')
    arucos_yaml = os.path.join(maps_dir, 'arucos.yaml')

    limpiar_artefactos = ExecuteProcess(
        cmd=['rm', '-f', mapa_pgm, mapa_yaml, arucos_yaml],
        output='screen',
    )

    visor_dron = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='visor_dron',
        arguments=['/uav/camera/image'] 
    )

    visor_carrito = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='image_view_carrito',
        arguments=['/cam_1/image_aruco'],
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_sim, 'rviz', 'mi_config.rviz')],
        parameters=[{'use_sim_time': True}]
    )

    detector_aruco_node = Node(
        package='mi_proyecto_sim',
        executable='detector_aruco.py',
        name='detector_aruco',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'aruco_size_m': 0.11,
            'tamano_pixel_mapa': 440,
            'ancho_laberinto_m': 2.65,
            'alto_laberinto_m': 3.10,
            'invert_colors': True,  # Peticion del usuario
        }]
    )

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': mapa_yaml,
            'use_sim_time': True,
            'frame_id': 'map_dron_origin',
        }],
        remappings=[('/map', '/map_dron')]
    )

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['map_server']
        }]
    )

    filtro_lidar_node = Node(
        package='mi_proyecto_sim',
        executable='filtro_lidar.py',
        name='filtro_lidar',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch', 'online_async_launch.py'
            )
        ),
        launch_arguments={
            'slam_params_file': os.path.join(pkg_sim, 'config', 'mapper_params_online_async.yaml'),
            'use_sim_time': 'true',
        }.items(),
    )

    optitrack_sim = Node(
        package='tello_control_pos',
        executable='optitrack_simulator',
        name='optitrack_simulator',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'latency_sec': 0.005},
            {'publish_orientation': True},
        ],
    )

    pose_fuser = Node(
        package='tello_control_pos',
        executable='pose_fuser',
        name='pose_fuser',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    pid_controller = Node(
        package='tello_control_pos',
        executable='position_controller',
        name='position_controller',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'velocity_scale': 1.0},
            {'kp': 0.5},
            {'ki': 0.06},
            {'kd': 0.35},
            {'enable_yaw_control': True},
            {'kp_yaw': 1.5},
            {'kd_yaw': 0.15},
            {'max_yaw_rate': 0.8},
        ],
    )

    plotter = Node(
        package='tello_control_pos',
        executable='plotter',
        name='plotter',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # NUEVO: Shim para conectar TelloAction con Topics estandar de Isaac
    shim_dron = ExecuteProcess(
        cmd=['python3', os.path.join(ws_root, 'isaac', 'tello_action_shim.py')],
        output='screen'
    )

    mision_dron_node = Node(
        package='mi_proyecto_sim',
        executable='mision_dron.py',
        name='mision_dron',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'use_real_drone': False},
            {'camera_topic': '/uav/camera/image'},
            {'odom_topic': '/drone1/odom'},
            {'stitcher': 'pose'},
            {'stitch_resolution': 0.005},
            {'camera_yaml': '/ros2_ws/src/mi_proyecto_sim/config/camera_tello_sim.yaml'},
            {'invert_colors': True}, # Peticion del usuario
            {'use_optitrack_pose': True}, # Guardar pose real en la foto
            {'map_size_m': 3.9},
        ],
    )
    
    mision = TimerAction(
        period=5.0,
        actions=[mision_dron_node],
    )

    planificador_rrt_node = Node(
        package='mi_proyecto_sim',
        executable='planificador_rrt',
        name='planificador_rrt',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_radius_m': 0.22,
        }]
    )

    # NUEVO: El control de trayectoria Kelly & Diaz
    control_trayectoria = Node(
        package='mi_proyecto_sim',
        executable='control_trayectoria.py',
        name='control_trayectoria',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'disable_visual_modes': False,
        }]
    )

    publicador_tfs_node = Node(
        package='mi_proyecto_sim',
        executable='publicador_tfs_arucos.py',
        name='publicador_tfs_arucos',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'arucos_yaml': arucos_yaml}
        ],
    )

    convertir_mapa = ExecuteProcess(
        cmd=['python3', os.path.join(ws_root, 'src', 'mi_proyecto_sim', 'tools', 'pgm_to_pbstream.py'),
             '--pgm', mapa_pgm,
             '--yaml', mapa_yaml,
             '--out', os.path.join(maps_dir, 'mapa_mision.pbstream')],
        output='screen'
    )

    handler_mision = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=mision_dron_node,
            on_exit=[convertir_mapa]
        )
    )

    # Cuando termina de volar el dron y convierte el mapa, se lanza el coche:
    rrt_y_slam_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=convertir_mapa,
            on_exit=[
                map_server_node,
                lifecycle_manager_node,
                publicador_tfs_node,
                slam_toolbox_launch,
                planificador_rrt_node,
                control_trayectoria
            ]
        )
    )

    # Usar rsp.launch.py para que los TFs locales del carrito existan en ROS 2
    tf_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ws_root, 'isaac', 'rsp.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )

    return LaunchDescription([
        limpiar_artefactos,
        shim_dron,
        tf_robot,
        visor_dron,
        visor_carrito,
        rviz_node,
        detector_aruco_node,
        filtro_lidar_node,
        optitrack_sim,
        pose_fuser,
        pid_controller,
        plotter,
        mision,
        handler_mision,
        rrt_y_slam_handler,
    ])
