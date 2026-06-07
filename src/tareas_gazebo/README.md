# tareas_gazebo — Las dos tareas corriendo en Gazebo

Versión ROS2 de las dos tareas (que en `tareas/` eran simulación pura en Python).
Aquí **Gazebo es la planta y los sensores**; estos nodos solo aportan el filtro,
el control y el mapeo. La matemática (Kalman-Bucy, ley de control, log-odds) es
la misma; solo cambia la entrada/salida (tópicos en vez de simulación interna).

## Arquitectura SIM-TO-REAL (cómo se conecta a tu sim del JetAuto)

El ruido se inyecta con la caracterización del robot REAL (`tareas_gazebo/sensor_models.py`,
copiado de `caracterizaciones/`), y el mismo número sintoniza el filtro que lo corrige.

| Señal | Tópico / fuente | Uso |
|---|---|---|
| Estado real (ground truth) | `/odom` (odometría nativa de gz = pose del modelo) | **referencia** + base para sintetizar sensores |
| Predicción del KF (50 Hz) | `/joint_states` (encoders→vel. lineal) + IMU yaw-rate (GT + bias 0.199°/s + ARW) | odometría propioceptiva (deriva) |
| Corrección del KF (15 Hz) | pose absoluta LiDAR = `/odom` + ruido MS200 (tipo AMCL) | **ancla** la deriva |
| Salida de control | `/cmd_vel` (Twist) → `jetauto_chassis_sim` → 4 ruedas | mueve el robot |
| Mapa | `/scan` (gpu_lidar real) + ruido MS200 inyectado + TF `odom→lidar_frame` | mapa de ocupación |

### Decisiones de diseño
- **Robot mecanum, control diferencial:** el controlador del paper (offset `h≠0`)
  se comanda como twist `[v, ω] = A(θ)⁻¹[q̇_d + Kp·q̃]` (Modelo 1, ec. 7/33), solo
  `linear.x` y `angular.z`.
- **Suite de sensores realista (continuo-discreto):** predicción con IMU yaw-rate +
  encoders (con su ruido real); el **bias del giro hace derivar** el rumbo
  (dead-reckoning). El **LiDAR ancla** la pose a 15 Hz (replica tu fusión real
  lidar-dominante) y el KF **acota** la deriva.
- **Por qué GT+ruido para IMU/LiDAR:** así se inyecta exactamente el ruido
  *caracterizado* (no el del sensor genérico de Gazebo). Los encoders sí salen del
  `/joint_states` real + ruido de cuantización.
- **Límites del chasís** (0.10 m/s, 0.5 rad/s): trayectoria = círculo lento
  (`R=0.6 m`) que arranca en la pose inicial (error ~0) para no saturar.

## Nodos
- **`kf_control`** (`kf_control_node.py`) — TAREA 1: Kalman-Bucy continuo +
  control de seguimiento. Publica `/cmd_vel`. Tras `duration_s` (90 s) guarda
  `gz_01..03_*.png` en `figs/`.
- **`gridmap`** (`gridmap_node.py`) — TAREA 2: mapa de ocupación en log-odds con
  `/scan`. Publica `nav_msgs/OccupancyGrid` en `/mapa_probabilistico` (visible en
  RViz) y guarda `gz_10_mapa_ocupacion.png` al cerrar (Ctrl-C).

## Cómo correrlo (dentro del docker)

```bash
docker compose up -d
docker exec -it integration bash

# dentro del container:
cd /ros2_ws
colcon build --symlink-install        # construye todo el workspace
source install/setup.bash
ros2 launch tareas_gazebo tareas_gazebo.launch.py
```

Esto levanta Gazebo (mundo laberinto) + el JetAuto + LiDAR + ambos nodos + RViz.
En RViz añade un display **Map** con tópico `/mapa_probabilistico` (frame `odom`)
para ver el mapa construirse en vivo; y `/odom` + `/cmd_vel` para ver el control.

Argumentos útiles:
```bash
ros2 launch tareas_gazebo tareas_gazebo.launch.py rviz:=false obstaculos:=false
```

Las figuras de validación quedan en `src/tareas_gazebo/figs/` (montado al host).

## Validación
- **Tarea 1:** `gz_01_plano_xy.png` (deseada/GT/encoders/KF), `gz_02` (estados),
  `gz_03` (error KF vs GT con bandas ±2σ). El estimado del KF debe seguir al
  ground truth mejor/más suave que la odometría cruda de encoders, con la deriva
  esperada por ser propioceptivo.
- **Tarea 2:** el mapa en `/mapa_probabilistico` (RViz) y `gz_10_mapa_ocupacion.png`
  deben reproducir las paredes/obstáculos del laberinto; comparable contra el mapa
  de `slam_toolbox` si lo corres en paralelo.

## Parámetros (ros2 param)
`kf_control`: `h, kp, traj_radius, traj_omega, duration_s, encoder_sigma,
meas_sigma, proc_sigma, v_max, w_max, control_rate, output_dir`.
`gridmap`: `fixed_frame, resolution, size_m, max_range, p_occ, p_free, output_dir`.
