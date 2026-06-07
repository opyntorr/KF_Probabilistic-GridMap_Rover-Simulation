# Caracterizaciones del JetAuto (motores, IMU, LiDAR)

Archivo consolidado de las caracterizaciones del robot real JetAuto (arquitectura bridge
Orin+Nano). Reúne **parámetros**, **resultados** y **código para caracterizar** de los tres
subsistemas. Fuente original: repo `jetauto_bridge` (`~/jetauto_migration`).

```
caracterizaciones/
├── README.md            <- este resumen (parámetros + resultados + cómo reproducir)
├── motores/             <- motor_characterization.py + ENCODER_ODOMETRY.md
├── imu/                 <- imu_allan_analyze.py + summary + plots (Allan)
├── lidar/               <- lidar_static_analyze.py + summary + plots
├── captura/             <- grabador nocturno (Orin) + wrapper (laptop) + README
└── ekf_fusion/          <- IMPLEMENTACION: ekf.yaml + wiring + nodos fuente + guia
```

> Para **implementar el EKF / fusión de sensores**, ver `ekf_fusion/README_ekf_fusion.md`:
> tiene la config del EKF, el wiring (madgwick+EKF), los nodos fuente (imu/odom con sus
> covarianzas) y el mapa de dónde entra cada número de la caracterización.

---

## 1) MOTORES (cinemática mecanum + encoders)

**Código:** `motores/motor_characterization.py` (corre DENTRO del contenedor del Nano, robot
ELEVADO; barre cada motor_id y mide ΔEnc por canal). **Writeup completo:** `motores/ENCODER_ODOMETRY.md`.

> Nota: la odometría real quedó en **dead-reckoning** (integra `cmd_vel`); el experimento de
> encoders en lazo cerrado se REVIRTIÓ (empeoró el SLAM). El código de encoders vive en la rama
> `encoder-odometry` (`88455af`). Estas calibraciones siguen válidas/reutilizables.

### Parámetros físicos del chasís (en `chassis_params.yaml` / `nano_params.yaml`)
| Parámetro | Valor |
|---|---|
| `wheel_diameter` | 96.5 mm |
| `wheelbase_a` / `wheelbase_b` | 103 / 97 mm |
| `pulse_per_cycle` (PPC) | 4320 |
| `motor_type` | 3 (JGB37 encoder motor) |
| `go_factor` / `turn_factor` | 0.90 / 0.93 |
| `max_motor_duty` / `motor_slew_rate` | 85 (de 100) / 250 duty·s⁻¹ |
| `control_rate` | 50 Hz |

### Envelope real vel/accel (en `ekf.yaml` y `nav2_params.yaml`)
- Velocidad: lineal **0.10 m/s** (cap nav), angular **0.5 rad/s**.
- Aceleración: lineal **1.3 m/s²**, angular **3.4 rad/s²** (decel angular 4.5).

### Calibraciones de encoders (válidas)
- Mapeo encoder→rueda: `enc_wheel_channels = [2, 3, 0, 1]`, `enc_wheel_signs = [1,1,1,1]`.
- Cinemática directa verificada (cero acoplamiento vx/vy/wz).
- **`linear_correction_factor = 1.085`** (piso: avance real 1.07 m / odom 0.986 m).

### Hallazgos clave
1. **El giro por ruedas NO sirve en mecanum** (rodillos a 45° deslizan no-linealmente). Medido
   cmd→gyro: 0.30→0.305, 0.40→0.446, 0.50→0.522 → encoders subestiman ~12%. **Usar IMU para yaw.**
2. **Batería baja = giros erráticos** (wz=0.5 llegó a girar ~1.7×). Mapear/navegar con buena batería.
3. Empujar a mano NO mueve encoders (caja reductora bloquea). Calibrar manejando con motores.

---

## 2) IMU — MPU-6050 (`/imu/data_raw` @ 50 Hz)

**Código:** `imu/imu_allan_analyze.py` (bias robusto, varianza de Allan → ARW + bias instability,
drift de bias). **Resultados:** `imu/imu_summary.txt`, `imu/allan_gyro.png`, `imu/bias_drift.png`.
Capturado en **10 h** quieto (2026-06-04), robot inmóvil, escena estática.

### Resultados (por eje de giro)
| Eje | Bias residual* | ARW (ruido blanco) | Bias instability (piso) |
|---|---|---|---|
| x (roll) | −0.245 °/s | 0.83 °/√h | 8.3 °/h |
| y (pitch) | +0.155 °/s | 1.60 °/√h | 13.5 °/h |
| **z (yaw)** | **+0.199 °/s** | **1.06 °/√h** | **6.0 °/h** |

Accel z = 10.00 m/s² (~2% error de escala, normal). Curva de Allan: pendiente −½ limpia hasta
τ≈400 s, mínimo ~1.1 millideg/s, luego sube (RRW).

> *MATIZ IMPORTANTE: el bag grabó `/imu/data_raw`, que **ya sale con el bias restado** por
> `imu_node` (auto-calibración al arranque, bias CRUDO ~0.019 rad/s = 1.1 °/s en z). Esos
> "biases" son el **RESIDUAL que deriva** durante la sesión (bias instability/RRW), NO el crudo.
> Recién calibrado el residual es ~0 (medido z −0.008 °/s).

### Parámetros derivados y APLICADOS (en `imu_node.py`, repo jetauto_bridge)
- `gyro_variance` (cov. de medición del EKF, de AR²/dt): **[3e-6, 1.1e-5, 5e-6] (rad/s)²**.
  Antes iba en cero → el EKF la forzaba a ~1e-6 y confiaba ciegamente.
- Calibración de bias al arranque ahora **robusta** (mediana + rechazo MAD; el gyro y tiene ~7.6% spikes).
- El EKF (`ekf.yaml`) ya fusiona odom_raw + imu/data + **cmd_vel (control)** = fusión default del carro.
- Implicación: la gravedad no observa el bias de yaw + el residual deriva ~0.2 °/s/sesión → **el LiDAR
  ancla el rumbo** (decisión lidar-dominante).

---

## 3) LiDAR — Orbbec MS200 (`/scan` @ 15 Hz)

**Código:** `lidar/lidar_static_analyze.py` (σ(d) robusta, rango máx confiable, dropout/espurios,
drift térmico, sugerencias AMCL). **Resultados:** `lidar/lidar_summary.txt`, `lidar/sigma_vs_range.png`,
`lidar/thermal_drift.png`. Mismo bag de 10 h (153 k scans usables).

### Resultados
| Métrica | Valor |
|---|---|
| σ_range típico | **4.3 mm** (1.4 mm <0.5 m, ~5 mm @1–2 m, ~15–20 mm @2.5–4 m) |
| Cuantización | 1 mm |
| Drift térmico | 3 mm / 10 h (despreciable, sin warm-up) |
| Dropout / espurios | 4.3% / 3.5% |

### Parámetros APLICADOS
- **slam_toolbox** (`mapper_params_online_async.yaml`, config activo): `min_laser_range` 0.15→**0.05**
  (MS200 ve a 3 cm), `max_laser_range` 12.0→**8.0** (margen seguro).
- **AMCL** (`nav2_params.yaml`, modelo `likelihood_field`): `sigma_hit` 0.2→**0.015**, `z_hit` 0.5→**0.9**,
  `z_rand` 0.5→**0.08**, `z_max` 0.04, rango MS200. (Preparado en repo; aún no en un path activo.)

### Caveats
1. Rango máx **NO** caracterizado >~5 m (la escena no tenía blancos lejanos); el "12 m" no está
   confirmado → para interiores usar `max_laser_range ≈ 8 m`.
2. El MS200 publica scans de longitud variable → solo 28% (300 rayos) se usaron; σ sólido pero
   el dropout 4.3% está subestimado.

---

## 4) Cómo reproducir (IMU + LiDAR, robot quieto)

Código en `captura/` (`orin_record_static.sh` corre en el Orin; `static_capture.sh` lo controla
desde la laptop). Detalle en `captura/README_captura.md`. Necesita `rosbags + numpy + matplotlib`.

```bash
# 1) Capturar (robot QUIETO, escena estática, A1 desconectado, MS200 girando):
~/static_capture.sh start nocturno 10     # graba /imu/data_raw + /scan, tope 10 h, detached
~/static_capture.sh stop                  # SIGINT, cierra el bag limpio
~/static_capture.sh fetch nocturno        # trae el bag a ~/static_runs/nocturno

# 2) Analizar:
python3 imu/imu_allan_analyze.py     ~/static_runs/nocturno
python3 lidar/lidar_static_analyze.py ~/static_runs/nocturno
```

**Motores** (script `motores/motor_characterization.py`): correr DENTRO del contenedor del Nano,
robot ELEVADO. Imprime a stdout (mapeo de canales, signos, pulsos/s, patrón mecanum) — transcribir
a `ENCODER_ODOMETRY.md`.

## Datos crudos
El bag de 10 h (`nocturno_0.db3`, ~2.2 GB) NO se incluye aquí (tamaño). Vive en
`~/static_runs/nocturno/`. Motores: no quedó dataset crudo (el script imprime a stdout; los
resultados están transcritos en `ENCODER_ODOMETRY.md`).
