#!/usr/bin/env python3
"""
Valida un /mapa_probabilistico (nav_msgs/OccupancyGrid) contra la verdad de
terreno del cuarto (misma geometria que tareas_room.sdf): cobertura, exactitud,
precision y recall, con los MISMOS criterios que la validacion de Python
(occ>=0.65, free<=0.35; exactitud sobre celdas decididas).

La rejilla de Gazebo/Isaac es 12x12 m (mas grande que el cuarto 6x6), asi que las
metricas se restringen al rectangulo del cuarto (region relevante), igual que en
Python (donde la rejilla ERA el cuarto).

Uso (con un gridmap publicando /mapa_probabilistico):
    source /opt/ros/humble/setup.bash && source isaac/isaac_env.sh
    python3 isaac/validate_map.py
"""
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from nav_msgs.msg import OccupancyGrid

# cuerpos ocupados (cx, cy, sx, sy) [m] — = tareas_room.sdf / ROOM de scene_mecanum
ROOM = {
    "wall_n": (-1.5, 2.25, 6.0, 0.1), "wall_s": (-1.5, -3.75, 6.0, 0.1),
    "wall_e": (1.5, -0.75, 0.1, 6.0), "wall_w": (-4.5, -0.75, 0.1, 6.0),
    "box1": (0.4, 1.1, 0.6, 0.6), "box2": (-3.5, 0.8, 0.6, 0.6),
    "box3": (-0.3, -3.0, 0.9, 0.6), "box4": (0.6, -2.0, 0.5, 1.4),
}
# rectangulo del cuarto (region relevante = paredes hacia adentro)
RXMIN, RXMAX, RYMIN, RYMAX = -4.55, 1.55, -3.80, 2.30
OCC_TH, FREE_TH = 0.65, 0.35


class Validator(Node):
    def __init__(self):
        super().__init__("validate_map",
                         parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.msg = None
        self.create_subscription(OccupancyGrid, "/mapa_probabilistico",
                                 lambda m: setattr(self, "msg", m), 1)


def main():
    rclpy.init()
    n = Validator()
    t = time.time()
    while n.msg is None and time.time() - t < 20:
        rclpy.spin_once(n, timeout_sec=0.2)
    if n.msg is None:
        print("NO llego /mapa_probabilistico (¿gridmap vivo? ¿dominio/DDS?)")
        rclpy.shutdown(); return

    m = n.msg
    W, H, res = m.info.width, m.info.height, m.info.resolution
    ox, oy = m.info.origin.position.x, m.info.origin.position.y
    data = np.array(m.data, dtype=np.int16).reshape(H, W)   # row-major, -1=desconocido

    X = ox + (np.arange(W) + 0.5) * res
    Y = oy + (np.arange(H) + 0.5) * res
    XX, YY = np.meshgrid(X, Y)

    gt_occ = np.zeros((H, W), bool)
    for (cx, cy, sx, sy) in ROOM.values():
        gt_occ |= (np.abs(XX - cx) <= sx / 2) & (np.abs(YY - cy) <= sy / 2)

    relevant = (XX >= RXMIN) & (XX <= RXMAX) & (YY >= RYMIN) & (YY <= RYMAX)
    observed = (data != -1) & relevant
    prob = np.where(data < 0, 0.5, data / 100.0)
    pred_occ = (prob >= OCC_TH) & observed
    pred_free = (prob <= FREE_TH) & observed
    decided = observed & (pred_occ | pred_free)
    correct = ((pred_occ & gt_occ) | (pred_free & ~gt_occ)) & decided

    acc = correct.sum() / max(1, decided.sum())
    tp = (pred_occ & gt_occ).sum()
    fp = (pred_occ & ~gt_occ).sum()
    fn = (pred_free & gt_occ).sum()
    prec = tp / max(1, tp + fp)
    rec = tp / max(1, tp + fn)
    reltot = int(relevant.sum())

    print("\n=== Validacion del mapa de ocupacion (vs verdad de terreno) ===")
    print(f"  celdas relevantes (cuarto) : {reltot}")
    print(f"  observadas                 : {int(observed.sum())}  -> cobertura {100*observed.sum()/reltot:.1f}%")
    print(f"  decididas                  : {int(decided.sum())}  ({100*decided.sum()/reltot:.1f}% del cuarto)")
    print(f"  Exactitud (decididas)      : {100*acc:.2f}%")
    print(f"  Precision (ocupado)        : {100*prec:.2f}%")
    print(f"  Recall    (ocupado)        : {100*rec:.2f}%")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
