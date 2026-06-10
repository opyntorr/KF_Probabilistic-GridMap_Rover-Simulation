# Mesh del DJI Tello — atribución

Los archivos `tello.dae` (visual) y `tello.stl` (colisión, opcional) provienen de:

- **Repositorio:** https://github.com/bingyo/tello_ros_gazebo
- **Ruta original:** `tello_description/meshes/tello.{dae,stl}`
- **Licencia:** Apache License 2.0

La licencia Apache-2.0 permite redistribución y uso, conservando esta nota de
atribución y el aviso de copyright del proyecto original. Texto completo de la
licencia: https://www.apache.org/licenses/LICENSE-2.0

## Detalles técnicos del mesh
- Unidades: metros (`<unit name="meter" meter="1"/>`, `up_axis = Z_UP`).
- Bounding box ≈ 0.142 m (X, adelante/atrás) × 0.128 m (Y, izq/der) × 0.0315 m (Z, alto).
- El nodo `Tello_V5` lleva una rotación +90° en X que, junto con `Z_UP`, deja el
  dron plano en el plano X-Y con +X hacia adelante. Por eso se usa con
  `scale = 1` y `rpy = 0 0 0` (igual que el URDF original de bingyo).
- Material embebido único (gris oscuro, diffuse ≈ 0.028). Sin texturas externas:
  el `.dae` es autocontenido.
