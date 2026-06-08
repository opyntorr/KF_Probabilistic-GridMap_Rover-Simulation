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
source /home/opyntorr/agv_uav_project_jetauto/install/setup.bash   # mi_proyecto_sim + tello_control_pos
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
    ros2 run tello_control_pos optitrack_simulator &
    ros2 run tello_control_pos pose_fuser_optitrack &
    ros2 run tello_control_pos position_controller &        # /drone1/target_position -> /drone1/cmd_vel
    ros2 run mi_proyecto_sim mision_dron.py --ros-args \
        -p camera_topic:=/uav/camera/image -p odom_topic:=/drone1/odom &
    wait
    ;;

  # ---------------------------------------------------------------------------
  # DEMO NAV (carrito): SLAM + RRT + control con evasión por lidar en el laberinto.
  #   Isaac:  $ISAACSIM/python.sh isaac/scene_mecanum.py --world laberinto --lidar --camera --imu --aruco
  # REQUIERE: slam_toolbox instalado (apt install ros-humble-slam-toolbox) — no estaba
  #   en el host; es la única dep extra para este demo. filtro_lidar/RRT/control son host.
  nav)
    ros2 run mi_proyecto_sim filtro_lidar.py &              # /scan -> /scan_filtered
    ros2 launch slam_toolbox online_async_launch.py \
        params_file:=/home/opyntorr/agv_uav_project_jetauto/src/mi_proyecto_sim/config/mapper_params_online_async.yaml \
        use_sim_time:=true &                                # /scan_filtered -> /map, map->odom
    ros2 run mi_proyecto_sim planificador_rrt &             # /map -> /rrt_path
    ros2 run mi_proyecto_sim control_trayectoria.py --ros-args \
        -p disable_visual_modes:=false &                    # /rrt_path + /scan_filtered -> /cmd_vel
    ros2 run mi_proyecto_sim detector_aruco.py &            # /cam_1 -> ArUco 3D
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
