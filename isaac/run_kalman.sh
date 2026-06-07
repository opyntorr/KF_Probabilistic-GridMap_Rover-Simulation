#!/usr/bin/env bash
# Tarea 1 (Kalman) — física. Requiere scene_mecanum.py ya corriendo.
set -e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
source "$HERE/isaac_env.sh"
echo ">> KF conduciendo el círculo por física (~90 s de sim); guarda isaac_01..03"
python3 "$HERE/kf_control_isaac.py"
