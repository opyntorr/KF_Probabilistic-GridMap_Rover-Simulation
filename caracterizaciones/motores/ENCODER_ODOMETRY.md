# Odometría por ENCODERS (lazo cerrado) — experimento revertido (2026-06-02)

> **Estado: REVERTIDO a dead-reckoning.** Este documento guarda el trabajo, las
> calibraciones y los hallazgos por si se quiere reactivar/mejorar más adelante.
> El **código completo** vive en la rama **`encoder-odometry`** (commit `88455af`).
> `master` quedó en el estado que funcionaba: dead-reckoning (integra `cmd_vel`) + EKF
> original (pose `x,y,yaw` de odom + yaw/vyaw de IMU) + caps de velocidad 0.25 m/s.

## Por qué se hizo

La odom del robot real era **lazo abierto**: `chassis_node.py` integraba el `cmd_vel`
**comandado** (no medía las ruedas) → drift. Objetivo: medir el movimiento REAL con los
encoders de la placa I2C 0x34 (lazo cerrado).

## Por qué se REVIRTIÓ

Tras activar encoders + reconfigurar el EKF, **el mapa SLAM se desconfiguró MÁS** que con
el dead-reckoning. Hipótesis: el EKF reconfigurado integraba **solo velocidades** (`vx,vy`
de encoders), y esas velocidades a 50 Hz tienen ruido de cuantización (~20 pulsos/ciclo);
al integrarlas, la TF `odom→base` resultó **menos suave** que cuando el EKF usaba la pose
→ peor predicción para el scan-matching de slam_toolbox. (Descartado: el yaw NO derivaba en
reposo — bias del giroscopio mínimo, −0.03°/s.)

## Calibraciones (VÁLIDAS — reutilizables)

Caracterizadas con `nano_docker/motor_characterization.py` (robot elevado) + pruebas en piso:

- **Mapeo encoder→rueda** (de `motor_characterization.py`): `chan_for_id={1:0,2:1,3:2,4:3}`
  (el array de `read_encoders()` está en orden motor_id, todos signo +). Combinado con
  `_INDEX_TO_MOTOR_ID={0:3,1:4,2:1,3:2}` (de `motor_board.py`):
  - `enc_wheel_channels = [2, 3, 0, 1]`   (rueda lógica i → índice en read_encoders)
  - `enc_wheel_signs = [1, 1, 1, 1]`
- **Cinemática directa** (verificada con vx+/vy+/wz+, cero acoplamiento):
  - `wi_mm_s = signo_i · (Δpulsos[canal_i] / pulse_per_cycle) · (π·wheel_diameter) / dt`
  - internas: `v2=-w0, v3=w1, v1=w2, v4=-w3`
  - `vx = (v1+v2+v3+v4)/4/1000`, `vy = (v2+v4-v1-v3)/4/1000`, `wz = ((v2+v3-v1-v4)/4)/(a+b)`
- **linear_correction_factor = 1.085** (calibrado en piso: avance real 1.07 m / odom 0.986 m).
  La parte LINEAL de los encoders funcionó bien (cero acoplamiento, escala calibrada).

## Hallazgos clave (importantes aunque no se use encoders)

1. **El giro por ruedas NO sirve en mecanum.** Los rodillos a 45° deslizan de forma variable
   con la velocidad y la superficie → el factor angular NO es constante. El **giroscopio
   (IMU MPU6050) mide el giro real fielmente**: cmd 0.30→gyro 0.305; cmd 0.40→gyro 0.446;
   cmd 0.50→gyro 0.522. Los encoders subestiman ~12% (y de forma no-lineal).
   → Para el yaw, usar la IMU, no las ruedas.
2. **Batería baja = giros erráticos.** Con batería baja, comando wz=0.5 hizo girar al robot
   ~1.7× (las "5 vueltas" raras). Con cargador/batería estable, consistente ~1.10-1.15.
   → **Mapear/navegar siempre con buena batería.**
3. Empujar el robot a mano NO mueve los encoders (la caja reductora bloquea las ruedas);
   calibrar manejando con motores.
4. Un motor puede quedar girando tras pruebas (estado residual de la placa); se para
   escribiendo 0 directo al I2C 0x34 (regs 50-53) dentro del contenedor.

## Cómo reactivar (si se reintenta, CON MEJORAS)

El código está en la rama `encoder-odometry`. Para activarlo: `odom_source: encoder` en
`nano_params.yaml` + desplegar `chassis_node.py` de esa rama (docker cp al contenedor + restart).

**NO repetir el error del EKF:** la reconfig que empeoró el SLAM puso `odom0` = solo `vx,vy`
e `imu0` = solo `vyaw`. Mejores caminos a probar:
- **Config intermedia (recomendada):** `odom0` aporta la POSE `x,y` (suave, como el original)
  pero SIN yaw; `imu0` aporta `yaw`+`vyaw`. Así el yaw viene de la IMU pero la posición usa
  la pose suave de odom (no velocidades ruidosas).
- O subir las covarianzas de `vx,vy` / bajar la frecuencia de lectura de encoders / filtrar.
- Mantener `linear_correction_factor=1.085` (encoders lineales) — ese sí ayudó.

## Despliegue (recordatorio)

Firmware del Nano: fuente de verdad `~/nano_docker/src/` EN EL NANO; el contenedor tiene el
código HORNEADO (solo monta /dev). Desplegar = `docker cp` a `/nano_ws/src/...` (+ build para
yaml) + `docker restart`. Persistir = re-hornear `jetauto-nano:humble` desde `~/nano_docker`.
El EKF corre en el Orin (`jetauto_controller/config/ekf.yaml`, servicio `jetauto-orin.service`).
