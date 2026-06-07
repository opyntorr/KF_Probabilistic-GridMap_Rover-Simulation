FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive

# Solo lo necesario para las 2 tareas (filtro de Kalman + mapa de ocupacion) en Gazebo:
#   - Gazebo (Ignition) + puentes ROS<->gz
#   - ros2_control + controladores (las 4 ruedas mecanum del JetAuto)
#   - xacro (URDF del robot)
#   - numpy / matplotlib / scipy (los nodos KF y de mapa + las versiones Python puras)
# Se quito: nav2, slam_toolbox, apriltag, vision_msgs, cv_bridge, robot_localization,
#   imu_filter_madgwick, joy/teleop, rqt/rviz-imu, el SDK de YDLidar y el driver del Tello.
RUN apt-get update && apt-get install -y \
    ros-humble-ros-ign-gazebo \
    ros-humble-ros-ign-bridge \
    ros-humble-ros-gz \
    ros-humble-xacro \
    ros-humble-controller-manager \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-joint-state-broadcaster \
    ros-humble-velocity-controllers \
    ros-humble-gz-ros2-control \
    ros-humble-ign-ros2-control \
    python3-pip \
    python3-numpy \
    python3-matplotlib \
    python3-scipy \
    nano \
    tmux \
    git \
    && rm -rf /var/lib/apt/lists/*

# Configurar variables de entorno para NVIDIA
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics,utility,compute

# Darle un color distinto al prompt para saber que estás en Docker
RUN echo "PS1='\[\033[01;36m\](docker) \[\033[01;32m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ '" >> /root/.bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

WORKDIR /ros2_ws
CMD ["bash"]
