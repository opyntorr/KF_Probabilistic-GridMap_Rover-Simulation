#!/usr/bin/env bash

# Script para compilar el workspace de ROS 2 dentro del contenedor Docker
# Se debe ejecutar desde la raíz del proyecto (agv_uav_project_jetauto_Vilchis)

echo "🛠️  Compilando el workspace dentro de Docker..."
docker compose run --rm simulacion bash -c "cd /ros2_ws && colcon build --symlink-install"
echo "✅ Compilación finalizada."
