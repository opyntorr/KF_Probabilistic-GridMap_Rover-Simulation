#!/usr/bin/env bash
# orin_record_static.sh  (vive en el Orin como ~/orin_record_static.sh)
# Graba /imu/data_raw + /scan a un rosbag2 para caracterizacion ESTATICA (Allan del IMU + stats del
# LiDAR). Lo lanza static_capture.sh por SSH (detached). Robot QUIETO, escena estatica, A1 desconectado.
#
# Uso:  bash ~/orin_record_static.sh <nombre> [horas_tope]   (default tope 12 h)
# El tope es de seguridad (timeout -s INT) para no llenar disco; normalmente se para con
# static_capture.sh stop (SIGINT -> cierra el bag limpio).
# NOTA: sin 'set -u' a proposito -> los setup.bash de ROS usan vars no definidas y abortarian.
NAME="${1:-static_$(date +%Y%m%d_%H%M%S)}"
HOURS="${2:-12}"
OUT="$HOME/static_runs/$NAME"
mkdir -p "$HOME/static_runs"

source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/jetson/cyclonedds-orin.xml

echo "[rec] $(date) grabando /imu/data_raw + /scan -> $OUT (tope ${HOURS}h)"
# -s INT: al llegar al tope cierra el bag limpio (igual que un Ctrl-C). exec para que el SIGINT
# de 'static_capture.sh stop' (pkill -INT) llegue directo a ros2 bag record.
exec timeout -s INT "${HOURS}h" ros2 bag record -o "$OUT" /imu/data_raw /scan
