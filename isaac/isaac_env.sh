# Entorno para la sim de Isaac Sim 4.5 — JetAuto mecanum POR FÍSICA (copia Vilchis).
# Uso:  source isaac/isaac_env.sh   (en CADA terminal)
export ROS_DOMAIN_ID=30
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# CycloneDDS solo-LOCAL (loopback): Isaac y el cerebro en la MISMA laptop.
export CYCLONEDDS_URI="file:///home/opyntorr/agv_uav_project_jetauto_Vilchis/isaac/cyclonedds_local.xml"
export ISAACSIM=/home/opyntorr/isaacsim
echo "[isaac_env] (Vilchis/física) ROS_DOMAIN_ID=$ROS_DOMAIN_ID RMW=$RMW_IMPLEMENTATION CYCLONEDDS=loopback"
