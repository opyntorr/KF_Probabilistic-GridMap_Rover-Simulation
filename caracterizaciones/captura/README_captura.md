# Caracterización estática del IMU + LiDAR MS200 (robot quieto)

Caracteriza el IMU (varianza de Allan → bias, ARW, bias instability) y el LiDAR MS200 (σ_range,
rango máx confiable, dropout, espurios, drift térmico) con el **robot quieto**, en una corrida larga
(toda la noche). Robusto a outliers (mediana/MAD/sigma-clipping) por si algo/alguien se mueve.

Sirve para: mejor odometría (prior de SLAM → menos drift) y, si haces **AMCL / slam_toolbox
localization**, los params del modelo de sensor (`sigma_hit`, `laser_max_range`, `z_rand`, `z_max`).

## Precondiciones (físicas)
- **A1 desconectado** (USB) → sin su vibración.
- **MS200 girando** (bringup normal), `/scan` @ 15 Hz; IMU publicando `/imu/data_raw` @ 50 Hz.
- Robot **perfectamente quieto**, superficie estable, **escena estática** (nada moviéndose en el
  campo del lidar). Para la Allan más limpia, idealmente el MS200 también detenido (su vibración a
  15 Hz infla el ARW; el bias del giro sale bien igual).

## Captura (desde la laptop)
```bash
~/static_capture.sh start nocturno 10     # graba /imu/data_raw + /scan en el Orin, tope 10 h, detached
# ...puedes cerrar la laptop. En la mañana:
~/static_capture.sh status                # ve si sigue grabando + tamaño
~/static_capture.sh stop                  # para LIMPIO (SIGINT -> cierra el bag)
~/static_capture.sh fetch nocturno        # trae el bag a ~/static_runs/nocturno
```
El bag queda en el Orin (`~/static_runs/<nombre>/`, .db3 + metadata) y se copia a la laptop con `fetch`.

## Análisis (en el venv, en la laptop)
```bash
source ~/evo_venv/bin/activate            # tiene rosbags + numpy + matplotlib
python3 ~/jetauto_migration/tools/characterization/imu_allan_analyze.py   ~/static_runs/nocturno
python3 ~/jetauto_migration/tools/characterization/lidar_static_analyze.py ~/static_runs/nocturno
```
Salidas (en la carpeta del bag):
- IMU: `imu_summary.txt`, `allan_gyro.png`, `bias_drift.png` → **bias del giro, ARW, bias instability, drift**.
- LiDAR: `lidar_summary.txt`, `sigma_vs_range.png`, `thermal_drift.png` → **σ(d), rango máx confiable,
  dropout, espurios, cuantización, drift térmico, + sugerencias de params AMCL**.

### Bias del LiDAR (opcional, captura corta aparte)
Contra una pared plana a distancia **medida con cinta** (un sesgo sistemático desplaza la localización
vs el mapa en AMCL/localization):
```bash
python3 lidar_static_analyze.py ~/static_runs/<captura_pared> --known-dist 1.00
```

## Dónde entran los parámetros
- **IMU** → `robot_localization` (imu0 noise + `process_noise_covariance`) y ganancias de madgwick →
  mejor yaw → menos drift de odom → **mapas SLAM más consistentes**.
- **LiDAR** → `slam_toolbox` `max_laser_range` (mapas más limpios); **AMCL** `sigma_hit`,
  `laser_max_range`/`laser_likelihood_max_dist`, `z_rand`, `z_max` (el script imprime sugerencias).

## NO se caracteriza quieto (queda para después, con movimiento)
- **Modelo de movimiento de AMCL (`alpha1..5`)** = ruido de la odometría por unidad de movimiento →
  manejando + comparando odom vs **OptiTrack** (usar el flujo EVO).
- Tuning de **cierre de lazo** de slam_toolbox y técnica de manejo (revisitar) → la palanca grande de
  consistencia, requiere movimiento.

## Archivos
- Orin: `~/orin_record_static.sh`.
- Laptop: `~/static_capture.sh` (= copia versionada aquí), `imu_allan_analyze.py`,
  `lidar_static_analyze.py`.
- venv: `rosbags` (lector de bags ROS2 puro-python).
