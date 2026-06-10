#!/usr/bin/env bash
# launch_ros_stack.sh — corre el stack ROS del proyecto Gazebo CONTRA Isaac Sim.
#
# La parte de PORT ya está hecha: Isaac (scene_mecanum.py) publica/consume EXACTAMENTE
# los topics que el stack ROS espera, en el dominio 30 + cyclone loopback (isaac_env.sh):
#   publica : /scan /odom /tf /joint_states /clock  /cam_1/{image,depth_image,camera_info}
#             /imu/data_raw  /uav/camera/{image,camera_info}  /drone1/odom
#   consume : /cmd_vel  /drone1/cmd_vel  /drone1/{takeoff,land}
# Los nodos ROS son sim-agnósticos y YA están construidos en el host:
#   mi_proyecto_sim  (mision_dron, control_trayectoria, planificador_rrt, filtro_lidar,
#                     detector_aruco)   -> /home/opyntorr/agv_uav_project_jetauto/install
#   tello_control_pos (position_controller, optitrack_simulator, pose_fuser_optitrack)
#
# Uso: arranca Isaac en UNA terminal y este script en OTRA (mismo dominio 30).
set -e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash   # mi_proyecto_sim + tello_control_pos
source "$HERE/isaac_env.sh"                                        # ROS_DOMAIN_ID=30 + cyclone loopback
DEMO="${1:-help}"

case "$DEMO" in
  # ---------------------------------------------------------------------------
  # DEMO STITCHING (dron): vuela un grid sobre el laberinto y arma el panorama.
  #   Isaac:  $ISAACSIM/python.sh isaac/scene_mecanum.py --drone --world laberinto --aruco
  # ADAPTACIÓN PENDIENTE: mision_dron usa el servicio TelloAction '/drone1/tello_action'
  #   para el takeoff; el dron de Isaac expone topics Empty '/drone1/{takeoff,land}'.
  #   Opción A: añadir un servicio TelloAction en drone.py (requiere tello_msgs en el
  #             python de Isaac).  Opción B: shim host -> publica /drone1/takeoff al
  #             recibir TelloAction.  cam_topic y odom_topic se pasan por parámetro.
  stitching)
    python3 "$HERE/tello_action_shim.py" &
    ros2 run tello_control_pos optitrack_simulator &
    ros2 run tello_control_pos pose_fuser_optitrack &
    ros2 run tello_control_pos position_controller &        # /drone1/target_position -> /drone1/cmd_vel
    ros2 run mi_proyecto_sim mision_dron.py --ros-args \
        -p camera_topic:=/uav/camera/image -p odom_topic:=/drone1/odom &
    
    # Herramientas visuales de depuración (como en Gazebo)
    ros2 run tello_control_pos plotter &
    ros2 run rqt_image_view rqt_image_view /uav/camera/image &
    rviz2 -d /ros2_ws/src/mi_proyecto_sim/rviz/mi_config.rviz --ros-args -p use_sim_time:=true &
    
    wait
    ;;

  # ---------------------------------------------------------------------------
  # DEMO NAV (carrito): SLAM + RRT + control con evasión por lidar en el laberinto.
  #   Isaac:  $ISAACSIM/python.sh isaac/scene_mecanum.py --world laberinto --lidar --camera --imu --aruco
  # REQUIERE: slam_toolbox instalado (apt install ros-humble-slam-toolbox) — no estaba
  #   en el host; es la única dep extra para este demo. filtro_lidar/RRT/control son host.
  nav)
    # Carrito: mapa stitcheado del DRON -> /map_dron -> RRT -> control hasta la meta.
    # PRECONDICIÓN: la misión del dron ya escribió maps/mapa_mision.{pgm,yaml} + maps/arucos.yaml.
    # DEPS host: ros-humble-slam-toolbox, ros-humble-nav2-lifecycle-manager (+ nav2-map-server).
    MAPS=/ros2_ws/src/mi_proyecto_sim/maps
    SIM_SRC=/ros2_ws/src/mi_proyecto_sim

    # 0) TF del robot (base_footprint->...->lidar_frame/cam_1/imu). Isaac solo da odom->base.
    ros2 launch "$HERE/rsp.launch.py" use_sim_time:=true &
    # 1) Filtro LiDAR: /scan -> /scan_filtered
    ros2 run mi_proyecto_sim filtro_lidar.py --ros-args -p use_sim_time:=true &
    # 2) SLAM (mapping): /scan_filtered -> /map + TF map->odom  (arg = slam_params_file)
    ros2 launch slam_toolbox online_async_launch.py \
        slam_params_file:="$SIM_SRC/config/mapper_params_online_async.yaml" \
        use_sim_time:=true &
    # 3) Mapa del DRON -> /map_dron (frame map_dron_origin, latched) + lifecycle autostart
    ros2 run nav2_map_server map_server --ros-args -r __node:=map_server \
        -p yaml_filename:="$MAPS/mapa_mision.yaml" -p frame_id:=map_dron_origin \
        -p use_sim_time:=true -r /map:=/map_dron &
    ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args \
        -r __node:=lifecycle_manager_map -p autostart:=true -p use_sim_time:=true \
        -p "node_names:=['map_server']" &
    # 4) Alinea map<->map_dron_origin con el carrito de arucos.yaml, ancla meta_aruco y
    #    LATCHEA /alignment_ready (sin esto el control nunca arranca, RRT no tiene meta).
    ros2 run mi_proyecto_sim publicador_tfs_arucos.py --ros-args \
        -p use_sim_time:=true -p arucos_yaml:="$MAPS/arucos.yaml" &
    # 5) RRT: /map_dron (+ /map) -> /rrt_path
    ros2 run mi_proyecto_sim planificador_rrt --ros-args \
        -p use_sim_time:=true -p robot_radius_m:=0.22 &
    # 6) Control (Kelly&Diaz + terminal + park visual ID1): /rrt_path + /scan_filtered + /cam_1 -> /cmd_vel
    ros2 run mi_proyecto_sim control_trayectoria.py --ros-args \
        -p use_sim_time:=true -p disable_visual_modes:=false &
    # 7) detector_aruco (dron, /uav/camera/image): opcional, no en la ruta crítica del park
    ros2 run mi_proyecto_sim detector_aruco.py --ros-args -p use_sim_time:=true &
    wait
    ;;

  *)
    echo "uso: $0 {stitching|nav}"
    echo "  stitching : dron vuela grid sobre el laberinto + panorama (needs Isaac --drone --world laberinto)"
    echo "  nav       : SLAM+RRT+control en el laberinto (needs Isaac --world laberinto --lidar --camera --imu)"
    echo ""
    echo "Arranca Isaac primero (otra terminal, mismo dominio 30):"
    echo "  source isaac/isaac_env.sh"
    echo "  \$ISAACSIM/python.sh isaac/scene_mecanum.py --world laberinto --lidar --camera --imu --aruco --drone"
    ;;
esac
