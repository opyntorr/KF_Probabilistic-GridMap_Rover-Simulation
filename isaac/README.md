# Las 2 tareas en Isaac Sim 4.5 — JetAuto mecanum POR FÍSICA

Tercera plataforma de las dos tareas del curso (Filtro de Kalman y Mapa de Ocupación),
**separada** de la versión Python pura (`../tareas/`) y de la de Gazebo (`../src/tareas_gazebo/`).

A diferencia del primer port a Isaac (que era cinemático: teleport del modelo completo), aquí
el robot se mueve **POR FÍSICA / FRICCIÓN**, igual que el JetAuto real:

- Las **4 ruedas mecanum giran** (drive de velocidad) a partir de `/cmd_vel` (IK mecanum).
- Cada rueda tiene **12 rodillos pasivos** (48 en total, esferas) modelados como cuerpos físicos.
- El movimiento (avance, giro y **strafe holonómico**) **emerge de la fricción** rueda→rodillo→suelo,
  no de un plugin cinemático. Es lo más sim-to-real de las tres plataformas → **es la de referencia**.
- Encoders **reales** desde `/joint_states` (las ruedas giran de verdad); `/odom` = pose física del chasís.

El slip real de las ruedas hace **derivar mucho** la odometría propioceptiva (hasta >1 rad en θ),
y el filtro de Kalman (ancla LiDAR) la **corrige a ~0.01 rad** — la demostración más realista del KF.

## Requisitos por terminal
`source /opt/ros/humble/setup.bash` **y** `source isaac/isaac_env.sh` (dominio 30 + cyclonedds
loopback). Los nodos corren en el **host**, NO en el docker `integration` (dominio 42, aislado).

## Tarea 1 — Filtro de Kalman
```bash
# terminal A — escena física (mundo vacío):
source isaac/isaac_env.sh
$ISAACSIM/python.sh isaac/scene_mecanum.py            # (--headless para sin ventana)
# terminal B:
./isaac/run_kalman.sh
```
Genera `isaac_01_plano_xy.png`, `isaac_02_estados_tiempo.png`, `isaac_03_error_estimacion.png`.

## Tarea 2 — Mapa de ocupación
```bash
# terminal A — escena física + cuarto 6x6 + RTX lidar 2D (/scan):
source isaac/isaac_env.sh
$ISAACSIM/python.sh isaac/scene_mecanum.py --gridmap  # (--headless para sin ventana)
# terminal B — robot_state_publisher + gridmap + KF (driver):
./isaac/run_gridmap.sh
```
Genera `isaac_10_mapa_ocupacion.png` (autoguardado cada ~10 s). En RViz: display **Map** en
`/mapa_probabilistico` (frame `odom`).

## Archivos
- `scene_mecanum.py` — escena de física (gravedad+fricción, drive de velocidad de ruedas, rodillos
  pasivos). `--gridmap` añade cuarto + RTX lidar. Importa `assets/jetauto_mecanum.urdf` (con rodillos).
- `kf_control_isaac.py` — Tarea 1 (encoders REALES de `/joint_states`).
- `gridmap_isaac.py` — Tarea 2 (log-odds desde `/scan`, submuestreo ~400 haces).
- `rsp.launch.py` — robot_state_publisher (árbol TF `base_footprint→lidar_frame`).
- `rover_model.py`, `sensor_models.py` — modelo cinemático + ruido caracterizado (compartidos).
- `assets/jetauto_mecanum.urdf` — URDF con 48 rodillos, reexportado de
  `src/mi_proyecto_sim/urdf/jetauto/jetauto_sim.urdf.xacro`.

## Notas de física / sim-to-real
- **Gravedad ON + material de fricción** (μ=1.0) en suelo y rodillos; paso de física a 120 Hz y CCD
  por los rodillos pequeños (esferas r=9.6 mm). La física de 48 rodillos es **pesada** → RTF bajo
  (igual que en Gazebo): la sim corre más lento que el tiempo real, pero `use_sim_time` lo maneja.
- **Ruedas = drive de velocidad** (kp=0, kd=`WHEEL_KD`); rodillos libres. La IK mecanum
  (`vx∓vy∓k·wz)/r`) convierte `/cmd_vel` en las 4 velocidades de rueda.
- Hay **slip** (las ruedas no alcanzan el 100% del target y hay algo de acoplamiento giro/strafe):
  esto es *esperable y más realista*; el control por realimentación + el KF cierran el lazo.
- TF: `odom→base_footprint` lo da la escena (pose física); `base_footprint→lidar_frame` el
  `robot_state_publisher`. El stamp del `/scan` del RTX está en otra época que `/clock` → el gridmap
  usa la última TF (con el callback rápido coincide con el barrido).

## Re-exportar el URDF con rodillos
```bash
SRC="$(pwd)/src/mi_proyecto_sim"; OV="$(pwd)/isaac/.ament_overlay"
mkdir -p "$OV/share/ament_index/resource_index/packages"
: > "$OV/share/ament_index/resource_index/packages/mi_proyecto_sim"
ln -sfn "$SRC" "$OV/share/mi_proyecto_sim"
source /opt/ros/humble/setup.bash
AMENT_PREFIX_PATH="$OV:$AMENT_PREFIX_PATH" \
  xacro "$SRC/urdf/jetauto/jetauto_sim.urdf.xacro" use_aruco:=false -o isaac/assets/jetauto_mecanum.urdf
sed -i "s#$OV/share/mi_proyecto_sim#$SRC#g" isaac/assets/jetauto_mecanum.urdf
```
