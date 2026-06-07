# Tarea 2 — Mapa de ocupación probabilístico

## 1. Objetivo
Implementar un **mapa de ocupación (occupancy grid) probabilístico** que represente
la probabilidad de ocupación del entorno y se actualice conforme llegan nuevas
mediciones de un sensor de rango simulado.

## 2. Componentes

### Entorno 2D con obstáculos (`World`)
Cuarto de `6 × 6 m` discretizado a `5 cm/celda` (120 × 120). Se construye una
rejilla de **verdad de terreno** (`gt`, 1 = ocupado / 0 = libre) con paredes
perimetrales y cuatro obstáculos internos (dos cajas y dos muros cortos). Esta
verdad sirve para (a) simular el sensor y (b) validar el resultado.

### Trayectoria del rover
El rover recorre un círculo dentro del cuarto usando el **mismo modelo cinemático
de la Tarea 1** (`comun/rover_model.py`) y su ley de control. De cada paso se
toma la pose `(x, y, θ)` desde la que se hace un escaneo (700 escaneos en total).

### Sensor de rango simulado (`RangeSensor`) — ruido REAL del MS200 (sim-to-real)
Lidar de **360 haces** en 360°, alcance máximo **8 m**. El rango verdadero se obtiene
por **ray-casting vectorizado** sobre la verdad de terreno y luego se le inyecta el
**ruido caracterizado del Orbbec MS200** (`../comun/sensor_models.py`, de
`caracterizaciones/`):
- **σ(rango)** dependiente de la distancia (1.4 mm <0.5 m … ~16–20 mm @2.5–4 m),
- **cuantización** de 1 mm,
- **dropout 4.3%** (rayos sin retorno → se omiten, no aportan información),
- **espurios 3.5%** (retorno falso *más cercano* — una pared opaca no se ve "a través").

Modelar el espurio como retorno más cercano (no a través del obstáculo) y omitir los
dropout es lo físicamente correcto y evita que esos rayos "borren" el interior de los
obstáculos.

### Actualización probabilística en log-odds (`OccupancyGrid`)
Cada celda guarda su **log-odds** `l = log(p/(1−p))`, con prior `p = 0.5` → `l = 0`.
Por cada haz se aplica el **modelo inverso de sensor** recorriendo la línea
robot→impacto con **Bresenham**:

```
celdas antes del impacto  ->  l += l_free   (l_free = log(0.4/0.6) < 0)
celda del impacto         ->  l += l_occ    (l_occ  = log(0.7/0.3) > 0)
```

Las actualizaciones son **aditivas** (acumulación bayesiana de evidencia) y se
limita `l` a `±6` para evitar saturación. La probabilidad se recupera con
`p = 1 − 1/(1 + e^l)`.

## 3. Visualización (carpeta `figs/`)
- **`10_evolucion_mapa.png`** — el mapa a 35, 105, 210, 385, 560 y 700 escaneos.
  Se ve cómo el gris (desconocido) se va resolviendo en blanco (libre) y negro
  (ocupado) conforme el rover (trayectoria roja) avanza.
- **`11_validacion_mapa.png`** — verdad de terreno vs mapa estimado vs mapa de
  aciertos (verde) / errores (rojo).

## 4. Validación (resultados)
Comparando el mapa final contra la verdad de terreno, **sobre las celdas
observadas y decididas** (`p ≥ 0.65` ocupado, `p ≤ 0.35` libre):

| Métrica | Valor (con ruido real del MS200) |
|---|---|
| Celdas observadas | 12 912 |
| **Exactitud** (celdas decididas) | **99.9 %** |
| Precisión (clase ocupado) | 99.4 % |
| Recall (clase ocupado) | 99.4 % |

Aun con el ruido realista del MS200 (σ por rango + dropout + espurios), la
**acumulación bayesiana en log-odds** recupera el mapa: un retorno espurio aislado no
fija una celda; se necesita evidencia repetida. Esa robustez al ruido real es la prueba
sim-to-real del enfoque probabilístico.

Las regiones libres y ocupadas se identifican correctamente. Las zonas que quedan
en **gris (~0.5)** son las **sombras de oclusión** detrás de los obstáculos y los
interiores de las cajas: celdas que ningún haz alcanzó, por lo que el filtro
—correctamente— las deja en "desconocido" en vez de inventar un valor.

## 5. Conclusiones
1. La acumulación en log-odds identifica correctamente libre vs ocupado (99.6 %).
2. El mapa **converge con el tiempo**: más escaneos desde más puntos de vista
   reducen la incertidumbre (menos gris).
3. El esquema probabilístico maneja el ruido del sensor de forma natural: una
   medición errónea aislada no fija una celda; se necesita evidencia repetida.

## 6. Cómo ejecutar
```bash
cd /ros2_ws/tareas/2_mapa_ocupacion   # dentro del docker
python3 gridmap.py                    # ~5 s -> figuras 10 y 11
```
