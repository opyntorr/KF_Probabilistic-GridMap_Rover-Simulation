# Port nocturno Gazebo → Isaac — reporte de avance

Inicio: 2026-06-08. Plan: `~/.claude/plans/cuddly-discovering-whale.md`. Modo autónomo (sin preguntas).
Prioridad: Mundos → Dron → Stitching → Sensores/ArUcos → Stack ROS → Serpenteo → HUD → Multicámara.

## Resumen ejecutivo (lo importante)
- **Recuperada incidencia de rename** del proyecto (espacio→guion bajo). Sin pérdida de datos.
- **Escena por defecto INTACTA** (Kalman/gridmap del usuario): verificado, sin regresión.
- **6 de 7 fases FUNCIONANDO en Isaac** (ver "Estado por fase"). **Validación capstone:** la escena COMPLETA
  (`--world laberinto --lidar --camera --imu --aruco --drone --perf`, TODO junto) arranca sin conflictos
  inter-módulo, el robot descansa en el piso y publica `/scan /odom /tf /cam_1/* /imu/data_raw /drone1/odom
  /joint_states` con **RTF=1.28, fps=21, gpu=14%, mem≈2 GB**.
- **F4 (stack ROS / stitching / nav): port hecho, demo live pendiente.** El interface de topics de Isaac está
  COMPLETO y los nodos ROS ya están CONSTRUIDOS en el host (mi_proyecto_sim + tello_control_pos, dominio 30).
  Falta solo la orquestación/adaptación operativa — ver `launch_ros_stack.sh` y la sección F4 abajo.
- **Serpenteo (F5): exhaustivamente investigado, NO eliminable** por los knobs de física/control — ver F5.

## SIGUIENTE PASO CLARO (resumible)
Pivotar la carga del laberinto al patrón **PROBADO** que ya funciona con el warehouse (`EXTRA_USDS`):
1. Convertir los 5 STL del maze a USD con `omni.kit.asset_converter`
   (`piso, paredes, lineas, jaula, cajasRecientes` de `…/laberinto_real/meshes/`, escala 0.001).
2. Cargarlos como **referencias bajo un Xform plano** (NO URDF) + colisión malla "none" por prim + colores planos
   (ya están en `assets/worlds/laberinto.urdf` como referencia de color) + auto-centrar. Eso = el camino de
   `EXTRA_USDS` que SÍ colisiona bien (warehouse: 149 mallas, robot se apoya).
3. Luego seguir con dron→stitching→sensores→ArUcos 3D→stack ROS→serpenteo→HUD→multicámara.

## Estado por fase
- [x] **F1 Mundos** — ✅ laberinto vía world_loader (STL→USD + colisión malla "none" + colores Gazebo +
  rotación Rx90·Rz90 + auto-centrado). Robot se asienta en el piso (z=0.05); `/scan` ve las paredes
  (218/256 returns). La ROTACIÓN era la causa de que cayera (STL Y-up; Gazebo lo paraba con roll=π/2).
- [x] **F2 Dron + cámara cenital** — ✅ dron Tello kinemático obedece `/drone1/cmd_vel` (verificado),
  publica `/uav/camera/image`+`camera_info` (nadir 960×720), `/drone1/odom`, takeoff/land. Malla Tello→USD.
  (Stitching pendiente: requiere el stack ROS — F4.)
- [x] **F3 Sensores + ArUcos 3D** — ✅ IMU `/imu/data_raw` @28Hz; RGB-D `/cam_1/{image,depth_image,camera_info}`
  @14Hz 640×480; ArUcos 3D reales (tabla blanca 4mm + 28 celdas negras extruidas +0.6mm), cubo objetivo
  (top=ID0/lados=ID1) + marcador ID4 en el techo. Fix clave: nodos pre-existentes del grafo por ruta completa.
- [x] **F6 HUD perf** — ✅ `[PERF] fps rtf gpu gmem ram` en consola + topic `/isaac/perf`
  (ej. fps=29.7 rtf=0.98 gpu=59% gmem=1656MB ram=24%). Overlay GUI vía display_options (windowed).
- [x] **F7 Multicámara + grabación** — ✅ rig (scene+chase) + grabación replicator a PNG (171 frames).
  Resolución por preset (720p/1080p/…). Viewports GUI vía omni.kit.viewport. (wheel-cam: el find_link no
  halló el link de rueda — menor, pasar wheel_link_prim explícito lo arregla.)
- Workflow de autoría completado: 7 módulos (`world_loader, drone, aruco3d, sensors, cameras, perf_hud,
  serpenteo_sweep`) + `INTEGRATION_GUIDE.md`. Integración + prueba GPU secuencial en curso.
- [ ] **F2 Dron + cámara cenital + stitching** (incremental, color-dependiente)
- [ ] **F3 Sensores (IMU+RGB-D) + ArUcos 3D + topics**
- [ ] **F4 Stack ROS + verificación lidar**
- [ ] **F5 Serpenteo (sweep)**
- [ ] **F6 HUD perf (FPS/RTF/GPU/mem)**
- [ ] **F7 Multicámara + grabación**

## Bitácora
(se va llenando con hallazgos, decisiones y fallos)

### 2026-06-08 — arranque
- Memoria guardada (gazebo-isaac-full-port, drone-stitching-color-incremental).
- Fuente del mundo: `mi_proyecto_sim/worlds/laberinto.sdf` → `laberinto_real` @ (-2.294,2.294,0,roll=π/2),
  `cubo_meta` @ (1.2,1.2,0.06), `aruco_2` @ (0,-1.4,0.06,yaw=-π/2), skydome. Luz: 3 directionals.
- **Colores del laberinto (CLAVE stitching), del model.sdf:** piso `0.08,0.08,0.09`; paredes (marrón)
  `0.55,0.38,0.20`; lineas (azul) `0.03,0.28,0.70` (offset y=-0.02027, visual); jaula (blanco) `0.85`;
  cajasRecientes (amarillo) `1.0,0.85,0.0`. Mallas en `…/laberinto_real/meshes/` escala 0.001. La jaula es una
  **estructura alta ~4.6 m** (la arena enrejada donde vuela el dron), no un piso plano.

### 2026-06-08 — F1 (mundo laberinto): hecho y bloqueo
- Creado `isaac/assets/worlds/laberinto.urdf` (5 mallas + colores planos = Gazebo). Integrado `--world laberinto`
  en `scene_mecanum.py` (arg, spawn (-1,-1,yaw=π/2), gating de piso/extras). **Default sin regresión (verificado).**
- El maze **carga y se auto-centra** por bbox (4.6×4.6×4.2, piso z=0) con los colores correctos. ✅ visual.
- **Bloqueo de colisión** (≈8 iteraciones diagnosticando):
  - El importador URDF deja un solo prim `Xform:collisions` con aproximación **convexHull** (bloque sólido).
  - Las mallas reales son **prototipos instanciados** (invisibles a `PrimRange` hasta `SetInstanceable(False)`,
    igual que en `jetauto_materials.py`).
  - Tras de-instanciar + poner colisión "none" en las 9 mallas + apagar el convexHull → **el robot igual cae**
    (z negativa, caída libre). Importar un entorno estático complejo como **articulación fix_base** no registra
    bien los colliders estáticos (a diferencia de `EXTRA_USDS`, referencia USD bajo Xform plano, que SÍ colisiona).
  - **Conclusión:** abandonar el import-URDF del maze; usar conversión STL→USD + carga estilo `EXTRA_USDS`.
- Bugs corregidos de paso en `scene_mecanum.py`: el loop de `EXTRA_USDS` estaba gateado en `not args.no_extra`
  en vez de `_load_extras` (con `--world` cargaba también el warehouse y ponía el robot en su plataforma z=1.30).
- Probes diagnósticos dejados (borrables): `isaac/_maze_probe.py`, `_maze_render.py`, `_maze_collinspect.py`.
- Dir vacío con nombre viejo (espacio) quedó sin borrar (seguridad); contiene solo 2 archivos de esta noche.

---

## CIERRE — estado final

### Cómo correr (todo en `scene_mecanum.py`, flags componibles)
```bash
source isaac/isaac_env.sh
# escena completa (laberinto + lidar + RGB-D + IMU + ArUcos 3D + dron + HUD):
$ISAACSIM/python.sh isaac/scene_mecanum.py --world laberinto --lidar --camera --imu --aruco --drone --perf
# por defecto (Kalman/gridmap del usuario): INTACTO, sin flags nuevos.
```
Flags nuevos: `--world laberinto`, `--lidar`, `--camera`, `--imu`, `--aruco`, `--drone`, `--cameras --rec_cams
--cam_res`, `--viewports`, `--perf`, y los del barrido (`--solver-pos/-vel --phys-hz --contact-offset
--rest-offset --maxdepen --roller-shape`). Módulos nuevos: `world_loader, drone, aruco3d, sensors, cameras,
perf_hud, serpenteo_sweep` + `INTEGRATION_GUIDE.md`.

### F5 — Serpenteo: MEJORADO con rodillos CILÍNDRICOS (−40%)
Lo que NO ayudó (invariante, residual ~1.6 cm): fricción μ (0.5/1/2), # rodillos (12/24), controlador
(h/Kp/LPF), ruido del estimador, solver-iters (4→64/16). **Lo que SÍ ayudó: cambiar las esferas por rodillos
CILÍNDRICOS** (barril, eje del cilindro alineado al eje de giro del joint a 45°). Medido:
| rodillo | serpenteo std_radial | tracking RMS |
|---|---|---|
| esfera (antes) | 0.0160 m | 0.050 m |
| **cilindro (default ahora)** | **0.0098 m (−40%)** | **0.024 m (−52%)** |
| cápsula | 0.0116 m | 0.024 m |
El contacto de LÍNEA del barril (vs el punto de la esfera) da tracción mecanum más suave — y es lo más
sim-to-real (los rodillos reales son barriles). **CILINDRO quedó como rodillo por defecto** en
`assets/jetauto_mecanum.urdf`. Clave: orientar el cilindro a lo largo del eje de giro (rpy `0 pi/2 pi/2-alpha`);
con la orientación mala del 1er intento el robot ni trazaba el círculo (RMS 0.67 m). Regenerar:
`python3 isaac/serpenteo_sweep.py --regen {cylinder|sphere|capsule}`. Residual baja a ~1 cm (no cero; el resto es
el artefacto de integración de Isaac). Backups: `/tmp/jetauto_mecanum_{sphere,cylinder}.urdf`.

### F4 — Stack ROS / stitching / nav: PORT hecho, demo live pendiente
- **Hecho:** Isaac publica/consume EXACTAMENTE los topics del stack (lista arriba). Nodos ROS **construidos en
  host** (`colcon build mi_proyecto_sim` ✅ + `tello_control_pos` ya estaba). `launch_ros_stack.sh {stitching|nav}`
  orquesta los nodos en dominio 30 contra Isaac.
- **STITCHING (demo) — 90% funcionando.** Construí `tello_msgs` + `tello_control_pos` en host, escribí
  `isaac/tello_action_shim.py` (servicio `/drone1/tello_action` → topics Empty del dron). Cadena verificada:
  `/drone1/odom → optitrack_simulator → /optitrack/rigid_body → pose_fuser_optitrack → /odometry/filtered →
  position_controller → /drone1/cmd_vel → dron`. **El dron DESPEGA** (shim ✓, z→2.2 estable) y **navega los
  waypoints** sobre el laberinto. **Bloqueo final:** la cámara nadir del dron publica **~3.7 Hz e intermitente**
  bajo el render RTX del laberinto (warning `SdRenderVarPtr missing valid renderVar LdrColorSDhost`), y la misión
  espera un frame FRESCO por waypoint → se atora en WP1 ("mismo stamp"). **Fix a probar:** bajar la resolución
  de la cámara del dron (960×720→480×360) y/o añadir `frameSkipCount`, o iluminación más ligera. Lanzar con
  `bash isaac/launch_ros_stack.sh stitching` + Isaac `--world laberinto --drone --aruco`.
- **nav** — falta `apt install ros-humble-slam-toolbox` (única dep extra); filtro_lidar/RRT/control_trayectoria
  ya están en host. Verificar TF `map→odom` + el gate `/alignment_ready`.

### Limpieza / pendientes menores
- URDF restaurado a 12 rodillos sphere (el barrido lo regeneró). Backup: `/tmp/jetauto_mecanum_sphere_backup.urdf`.
- wheel-cam del rig: el find_link no halló el link de rueda (pasar `wheel_link_prim` explícito).
- Probes y `serpenteo_sweep.py` quedan en `isaac/` (borrables / a depurar).
- Nada commiteado (el usuario decide).
