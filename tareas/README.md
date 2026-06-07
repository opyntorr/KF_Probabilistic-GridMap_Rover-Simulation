# Tareas de Navegación Probabilística

Implementación mínima de dos tareas, montadas en el **mismo entorno** (el docker
de este proyecto) y compartiendo el mismo modelo cinemático del rover.

```
tareas/
├── comun/
│   ├── rover_model.py          # modelo cinemático generalizado de Kelly (compartido)
│   └── sensor_models.py        # ruido REAL de los sensores (de ../../caracterizaciones)
├── 1_filtro_kalman_rover/      # TAREA 1: Filtro de Kalman continuo + control
│   ├── kf_rover.py             #   versión base (medición de pose, C=I) + figuras 01–04
│   ├── cross_validate.py       #   cross-validación Monte Carlo (figura 05)
│   ├── kf_rover_sim2real.py    #   versión SIM-TO-REAL: IMU+encoders + ancla LiDAR (figs 06–08)
│   ├── REPORTE.md
│   └── figs/                   #   figuras generadas
└── 2_mapa_ocupacion/           # TAREA 2: Mapa de ocupación probabilístico
    ├── gridmap.py              #   entorno + lidar MS200 (ruido real) + log-odds + validación
    ├── REPORTE.md
    └── figs/
```

### Sim-to-real
`comun/sensor_models.py` es la **única fuente de verdad** del ruido de los sensores,
tomada de las caracterizaciones del robot REAL (`../caracterizaciones/`): IMU MPU-6050
(bias 0.199 °/s, ARW 1.06 °/√h), encoders mecanum (escala 1.085, cuantización) y LiDAR
MS200 (σ_range 4.3 mm, dropout 4.3 %, espurios 3.5 %). El **mismo número** inyecta el
ruido en simulación y sintoniza (Q, R) el filtro que lo corrige. Ver `1_filtro_kalman_rover/REPORTE.md` §7.

Ambas tareas usan `comun/rover_model.py`, que implementa el *Robot Diferencial
Generalizado* del paper de Díaz & Kelly (2016). La cinemática se reutiliza tal
cual; la parte teórica/derivación del documento queda fuera del alcance de esta
implementación (se pidió ignorarla por ahora).

## Requisitos
Solo `numpy`, `scipy` y `matplotlib`, que ya vienen instalados en la imagen del
docker. Las figuras se guardan a disco (backend `Agg`), así que **corre headless**
sin necesidad de pantalla.

## Cómo ejecutar

### Dentro del docker (entorno objetivo)
La carpeta se monta en `/ros2_ws/tareas` (ver `compose.yaml`). El container se
llama **`integration`**.

```bash
# levantar el container
docker compose up -d
docker exec -it integration bash

# dentro del container:
cd /ros2_ws/tareas/1_filtro_kalman_rover && python3 kf_rover.py && python3 cross_validate.py
cd /ros2_ws/tareas/2_mapa_ocupacion   && python3 gridmap.py
```

### En el host (si tiene numpy/scipy/matplotlib)
```bash
cd tareas/1_filtro_kalman_rover && python3 kf_rover.py && python3 cross_validate.py
cd ../2_mapa_ocupacion          && python3 gridmap.py
```

Cada `REPORTE.md` describe la tarea, las ecuaciones usadas y los resultados.
