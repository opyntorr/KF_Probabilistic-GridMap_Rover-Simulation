#!/usr/bin/env bash
# run.sh — Lanzador unificado de un solo clic (Isaac Sim host + ROS Docker)

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export DISPLAY="${DISPLAY:-:0}"
source "$HERE/isaac_env.sh"

cleanup() {
  echo; echo "[run] limpiando procesos…"
  pkill -f scene_mecanum.py 2>/dev/null
  echo "[run] cerrando contenedores de Docker si quedaron colgados…"
  cd /home/opyntorr/agv_uav_project_jetauto_Vilchis && docker compose down 2>/dev/null
  docker ps -q --filter "name=integration-simulacion-run" | xargs -r docker stop 2>/dev/null
  echo "[run] listo."
}
trap 'cleanup; exit 0' INT TERM

echo "=========================================================="
echo "🚀 INICIANDO ENTORNO UNIFICADO (ISAAC + DOCKER)"
echo "=========================================================="

echo "[run] 1. Arrancando Isaac Sim (Host)…"
"$ISAACSIM/python.sh" "$HERE/scene_mecanum.py" --world laberinto --drone --aruco --lidar --imu --no-roof >/dev/null 2>&1 &

echo "[run] Esperando 15 segundos a que Isaac cargue el entorno 3D…"
sleep 15

echo "[run] 2. Arrancando el orquestador automático en Docker…"
cd /home/opyntorr/agv_uav_project_jetauto_Vilchis
# Guardar el log en mission.log pero mostrarlo en pantalla usando tee
docker compose run --rm -e ROS_DOMAIN_ID=30 -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp -e CYCLONEDDS_URI="file:///ros2_ws/isaac/cyclonedds_local.xml" -e PYTHONUNBUFFERED=1 simulacion bash -c "source /ros2_ws/install/setup.bash && ros2 launch mi_proyecto_sim isaac.launch.py" | tee mission.log

cleanup
