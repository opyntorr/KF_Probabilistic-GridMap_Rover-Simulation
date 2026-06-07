cmake_minimum_required(VERSION 3.8)
project(mi_proyecto_sim)

# Recortado para las 2 tareas (KF + mapa) en Gazebo: solo se necesita el robot
# JetAuto (URDF + mundo + chassis_sim). Se eliminaron el nodo C++ planificador_rrt
# y los scripts del dron/nav/aruco que ya no se usan.
find_package(ament_cmake REQUIRED)

install(PROGRAMS
  mi_proyecto_sim/jetauto_chassis_sim.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  models
  worlds
  launch
  urdf
  rviz
  config
  maps
  meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
