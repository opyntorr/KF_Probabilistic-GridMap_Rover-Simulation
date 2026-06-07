# EKF + fusión de sensores (robot_localization) — kit de implementación

Cómo está fusionada la odometría del JetAuto y dónde entran los números de la caracterización.
Complementa la carpeta `caracterizaciones/` (que dice **qué son** los sensores); esto dice **cómo
se fusionan**.

## Arquitectura (bridge Orin+Nano)

```
   NANO (contenedor, hardware)                    ORIN (cómputo)
   ┌──────────────────────────┐                  ┌─────────────────────────────────┐
   │ chassis_node ──/odom_raw──┼───DDS──────────► │ ekf_filter_node (robot_localiz.)│
   │   (integra cmd_vel,       │                  │   odom0 = /odom_raw  (pose x,y,yaw)
   │    publica cov)           │                  │   imu0  = /imu/data  (yaw + vyaw)│──/odom
   │ imu_node ───/imu/data_raw─┼───DDS──► imu_filter_madgwick ──/imu/data──►        │  + TF
   │   (MPU6050, publica cov)  │                  │   control = /cmd_vel             │  odom→base
   └──────────────────────────┘                  └─────────────────────────────────┘
```

- `chassis_node.py` ← cmd_vel → motores I2C; publica **/odom_raw** (dead-reckoning, lazo abierto) con covarianzas.
- `imu_node.py` → **/imu/data_raw** (MPU6050) con las covarianzas caracterizadas.
- `imu_filter_madgwick` → /imu/data_raw → **/imu/data** (añade orientación). Config inline en `orin_compute.launch.py`.
- `ekf_filter_node` → fusiona /odom_raw + /imu/data + **cmd_vel (control)** → **/odom** + TF odom→base_footprint.

## Archivos en este kit
| Archivo | Qué es |
|---|---|
| `ekf.yaml` | **La fusión.** Config de robot_localization (qué campo aporta cada sensor + control). |
| `orin_compute.launch.py` | **El wiring.** Lanza RSP + madgwick + EKF (en el Orin). |
| `imu_node.py` | Fuente IMU (publica /imu/data_raw con `gyro_variance` de la Allan). Corre en el Nano. |
| `chassis_node.py` | Fuente odom (publica /odom_raw con `ODOM_POSE_COV`/`ODOM_TWIST_COV`). Corre en el Nano. |
| `chassis_params.yaml` | Constantes del chasís (wheel_diameter, wheelbase, PPC, factors). |
| `robot_localization_REFERENCIA_ros1.yaml` | El config ROS1 original, **con comentarios de cada param** + `process_noise_covariance` e `initial_estimate_covariance` completos. Útil como referencia. |

## Quién aporta qué al EKF (de `ekf.yaml`)
- **odom0 = /odom_raw**, `odom0_config = [x,y,z, _,_,yaw, ...]` diferencial → POSE x,y,yaw.
- **imu0 = /imu/data**, `imu0_config = [_,_,_, _,_,yaw, _,_,_, _,_,vyaw, ...]` diferencial → yaw + vyaw.
- **control = /cmd_vel** (`use_control: true`, `control_config = [vx,_,_,_,_,vyaw]`), con `acceleration_limits
  = [1.3,0,0,0,0,3.4]` (= envelope real medido). Esto = la fusión **default del carro**.
- `two_d_mode: true`, `world_frame: odom`, `publish_tf: true`.

## Dónde entran los números de la caracterización
| Número (de `caracterizaciones/`) | Dónde se aplica |
|---|---|
| Cov. giro `[3e-6, 1.1e-5, 5e-6]` (rad/s)² (de ARW) | `imu_node.py` → `gyro_variance` (mensaje /imu/data_raw). El EKF la usa al ponderar vyaw. |
| Bias crudo del giro (~1.1°/s en z) | `imu_node.py` lo auto-resta al arranque (mediana+MAD). El residual deriva ~0.2°/s → el LiDAR ancla el yaw. |
| Cov. odom `ODOM_POSE_COV`/`ODOM_TWIST_COV` | `chassis_node.py` (yaw pose = 1e3 → odom casi no aporta yaw; lo da la IMU). |
| Envelope vel/accel (1.3 / 3.4) | `ekf.yaml` `acceleration_limits` (predicción con control). |
| Constantes de rueda (D, A, B, PPC) | `chassis_params.yaml` (cinemática de /odom_raw). |

> robot_localization lee las covarianzas **del mensaje** (no hay override por-sensor en el yaml) →
> por eso las cov. del giro van en `imu_node.py` y las de odom en `chassis_node.py`.

## Cómo se lanza
En el Orin: `ros2 launch jetauto_bringup orin_compute.launch.py` (RSP + madgwick + EKF). El Nano ya
publica /odom_raw + /imu/data_raw por su bringup. Verás `/odom` + TF `odom→base_footprint`.

## Mejoras pendientes para la fusión (de `motores/ENCODER_ODOMETRY.md`)
1. **Modelo de movimiento de AMCL (`alpha1..5`)**: NO se caracteriza quieto → manejar + comparar odom
   vs **OptiTrack** (flujo EVO). Es el ruido de odometría por unidad de movimiento.
2. **Si se reintentan encoders** (lazo cerrado, rama `encoder-odometry`): NO repetir el error del EKF
   (que puso odom0 = solo vx,vy). Config intermedia recomendada: **odom0 = POSE x,y (suave, sin yaw)**
   + **imu0 = yaw+vyaw**. Mantener `linear_correction_factor = 1.085`.
3. La covarianza del giro se puede inflar 2–3× si el filtro va nervioso (la deriva de bias no está en
   el ruido blanco).
