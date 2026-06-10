FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive

# Solo lo necesario para las 2 tareas (filtro de Kalman + mapa de ocupacion) en Gazebo:
#   - Gazebo (Ignition) + puentes ROS<->gz
#   - ros2_control + controladores (las 4 ruedas mecanum del JetAuto)
#   - xacro (URDF del robot)
#   - numpy / matplotlib / scipy (los nodos KF y de mapa + las versiones Python puras)
# Paquetes restaurados para soportar la simulación completa (Dron + Carrito):
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
    ros-humble-nav2-bringup \
    ros-humble-navigation2 \
    ros-humble-slam-toolbox \
    ros-humble-robot-localization \
    ros-humble-imu-filter-madgwick \
    ros-humble-apriltag-ros \
    ros-humble-apriltag-msgs \
    ros-humble-cv-bridge \
    ros-humble-vision-msgs \
    ros-humble-teleop-twist-joy \
    ros-humble-joy \
    ros-humble-urdf-tutorial \
    ros-humble-rqt-image-view \
    python3-pip \
    python3-numpy \
    python3-matplotlib \
    python3-scipy \
    python3-rosdep \
    nano \
    tmux \
    git \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-tf-transformations \
    python3-transforms3d \
    python3-protobuf \
    protobuf-compiler \
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
