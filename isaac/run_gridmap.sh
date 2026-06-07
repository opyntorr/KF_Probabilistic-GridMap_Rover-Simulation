#!/usr/bin/env bash
# Tarea 2 (GridMap) — física. Requiere scene_mecanum.py --gridmap ya corriendo.
set -e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
source "$HERE/isaac_env.sh"
cleanup(){ kill -INT "$GRID" 2>/dev/null; sleep 1; kill "$RSP" "$GRID" 2>/dev/null; }
trap cleanup EXIT
ros2 launch "$HERE/rsp.launch.py" >/tmp/isaac_rsp.log 2>&1 & RSP=$!
sleep 4
python3 "$HERE/gridmap_isaac.py" >/tmp/isaac_gridmap_node.log 2>&1 & GRID=$!
sleep 2
echo ">> mapeando por física mientras el KF conduce (~90 s de sim)..."
python3 "$HERE/kf_control_isaac.py"
echo ">> mapa en $HERE/figs/isaac_10_mapa_ocupacion.png"
