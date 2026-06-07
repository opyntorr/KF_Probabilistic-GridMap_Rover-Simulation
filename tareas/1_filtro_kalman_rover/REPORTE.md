# Tarea 1 — Integración del Filtro de Kalman continuo en la simulación del rover

## 1. Objetivo
Integrar un **Filtro de Kalman en tiempo continuo** (forma de Kalman–Bucy) a la
simulación del rover ya existente, de modo que el **controlador de seguimiento de
trayectoria use el estado estimado por el filtro** (no el estado real ni la
medición cruda).

## 2. Modelo del rover
Se usa el modelo cinemático del *Robot Diferencial Generalizado* de Díaz & Kelly
(2016), "Modelo 2" (entradas = velocidades angulares de las ruedas), implementado
en `../comun/rover_model.py`:

```
xi = [x, y, theta]^T          u = [w_r, w_l]^T
xi_dot = [ D(theta) ; phi^T ] u
```

con `D(theta)` (ec. 28) no singular porque `h ≠ 0` (`det D = -h r²/d`), lo que
permite la ley de control por inversión de matriz. Parámetros del paper:
`r = 4 cm`, `h = 15 cm`, `d = 10 cm`.

## 3. Cómo se integró el Filtro de Kalman
El sistema es no lineal, así que se usa el **Filtro de Kalman–Bucy extendido**
(KF continuo linealizado alrededor del estimado). Todo se integra con **Euler**.

**Propagación del estado real (planta + ruido de proceso, Euler–Maruyama):**
```
xi_{k+1} = xi_k + dt·f(xi_k, u_k) + sqrt(dt)·chol(Q)·w_k
```

**Sensor simulado:** se mide la postura completa con ruido blanco gaussiano,
```
y_k = C·xi_k + v_k ,   C = I₃ ,   v_k ~ N(0, R)
```
con desviaciones `σ = [5 cm, 5 cm, 4°]`.

**Filtro de Kalman continuo (un paso de Euler):**
```
F   = ∂f/∂xi |_(xi_hat, u)              (Jacobiano, jacobian_f)
L   = P·Cᵀ·R⁻¹                           (ganancia de Kalman)
xi_hat ← xi_hat + dt·[ f(xi_hat,u) + L·(y − C·xi_hat) ]
P      ← P + dt·[ F·P + P·Fᵀ + Q − P·Cᵀ·R⁻¹·C·P ]   (ecuación de Riccati)
```

**Lazo de control con el estado estimado (ec. 33 del paper):**
```
u = D⁻¹(theta_hat)·[ qd_dot + Kp·(qd − q_hat) ]
```
Es decir, `q_hat` y `theta_hat` provienen del filtro. La trayectoria deseada es el
círculo del escenario 2 del paper, `qd = R[sin(ωt), cos(ωt)]`.

### Nota de implementación (estabilidad numérica)
La ecuación de Riccati continua integrada con Euler es **rígida** cuando `P(0)`
está lejos del estado estacionario (el término `−P R⁻¹ P` puede volver `P`
negativa en un paso). Se resolvió usando un paso fino `dt = 2 ms` y una `P(0)`
moderada; con eso la transición es estable y el filtro converge. También se
satura `u` como red de seguridad numérica (no se activa en operación normal).

## 4. Validación
Figuras en `figs/`:

| Figura | Qué muestra |
|---|---|
| `01_plano_xy.png` | Plano XY: deseada, real, **nube de mediciones**, estimada. El estimado corta limpio la nube de ruido. |
| `02_estados_tiempo.png` | `x, y, θ` vs tiempo: real (azul) vs medición (gris) vs estimado (rojo). |
| `03_error_estimacion.png` | Error de estimación dentro de las bandas **±2σ** del propio filtro (consistencia). Las bandas arrancan anchas y se cierran al converger. |
| `04_error_seguimiento.png` | Error de seguimiento `q̃ = qd − q` → tiende a cero. |
| `05_cross_validation.png` | Monte Carlo (40 corridas), resumen comparativo. |

### Resultados de cross-validación (40 corridas, `cross_validate.py`)

| Estado | RMSE medición | RMSE KF | Mejora | % dentro de ±2σ |
|---|---|---|---|---|
| x | 0.0500 m | 0.0100 m | **5.0×** | 100 % |
| y | 0.0500 m | 0.0101 m | **4.9×** | 100 % |
| θ | 0.0698 rad | 0.0290 rad | **2.4×** | 99 % |

**Error de seguimiento (RMS de la norma):**
- Control con estado **estimado (KF)**: `0.0895 ± 0.0009 m`
- Control con estado **real (ideal)**: `0.0896 ± 0.0003 m`

## 5. Conclusiones
1. El KF **reduce el error de estado ~5×** en posición frente a la medición cruda.
2. El filtro es **consistente**: el error real cae dentro de las bandas ±2σ que
   predice su propia covarianza `P` (de hecho es ligeramente conservador, ~100 %).
3. Cerrar el lazo de control con el estado **estimado** da un seguimiento
   prácticamente idéntico a usar el estado **real** (0.0895 vs 0.0896 m): el KF
   permite controlar el rover correctamente aun con sensores ruidosos.

## 6. Cómo ejecutar
```bash
# dentro del docker (o en cualquier python con numpy/scipy/matplotlib):
cd /ros2_ws/tareas/1_filtro_kalman_rover
python3 kf_rover.py            # version base: simulación + figuras 01–04
python3 cross_validate.py      # Monte Carlo + figura 05
python3 kf_rover_sim2real.py   # version SIM-TO-REAL (abajo) + figuras 06–08
```

## 7. Variante SIM-TO-REAL (ruido real de los sensores)
`kf_rover_sim2real.py` reemplaza la medición idealizada (pose completa, C=I) por el
**conjunto de sensores REAL del JetAuto**, con el ruido medido en `caracterizaciones/`
(centralizado en `../comun/sensor_models.py`). Es la versión "lo más sim-to-real posible".

**Arquitectura (multi-tasa, continuo-discreto):**
- **Predicción (propioceptiva, 50 Hz):** odometría con
  *IMU yaw-rate* (bias residual **0.199 °/s** + ruido **ARW 1.06 °/√h**) +
  *encoders* (velocidad lineal calibrada, ruido de cuantización PPC 4320). Como el
  bias del giro **no** se modela en el filtro, el rumbo **deriva** (igual que el robot real).
- **Corrección (exteroceptiva, 15 Hz):** *pose absoluta del LiDAR MS200* tipo
  scan-match/AMCL (σ derivada del σ_range de **4.3 mm**). **Ancla** la deriva — replica
  la decisión "lidar-dominante" del robot real.
- `Q` se deriva del ruido de IMU+encoders; `R` del MS200. Mismo número que inyecta el
  ruido sintoniza el filtro (puente sim-to-real). Verificado: el ARW caracterizado da
  varianza ≈ **5e-6 (rad/s)²**, idéntica a la del EKF real (`ekf_fusion/ekf.yaml`).

**Resultados (RMSE vs estado real, figuras 06–08):**

| Estado | Odometría sola (deriva) | KF (con ancla LiDAR) | Mejora |
|---|---|---|---|
| x | 0.067 m | 0.007 m | ~9× |
| y | 0.071 m | 0.007 m | ~10× |
| θ | 0.141 rad (~8°) | 0.006 rad (~0.3°) | ~24× |

El KF **acota** la deriva del dead-reckoning: la odometría sola se va en espiral
(fig. 06) y el rumbo se va ~8° (fig. 07, θ naranja); el ancla LiDAR lo corrige a ~0.3°.
El error queda dentro de las bandas ±2σ (fig. 08): filtro consistente.
