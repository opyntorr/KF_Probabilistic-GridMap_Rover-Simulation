#!/usr/bin/env python3
"""
TAREA 2 en Gazebo — Mapa de ocupacion probabilistico con el LiDAR real.

A diferencia de la version Python pura (donde el lidar se simulaba por
ray-casting), aqui el sensor es el LiDAR REAL del JetAuto en Gazebo (/scan).
El nodo:
  * suscribe /scan (sensor_msgs/LaserScan),
  * obtiene la pose del lidar en el frame fijo 'odom' via TF,
  * actualiza una rejilla en LOG-ODDS con el mismo modelo inverso de sensor
    (Bresenham: libre a lo largo del haz, ocupado en el impacto),
  * publica nav_msgs/OccupancyGrid en /mapa_probabilistico (se ve en RViz),
  * guarda una figura del mapa al cerrar.

Es "mapeo con poses conocidas": la pose la da la TF (odom->lidar_frame).
"""

import math
import os

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import tf2_ros

from tareas_gazebo import sensor_models as sm


def yaw_from_quat(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def bresenham(i0, j0, i1, j1):
    cells = []
    di, dj = abs(i1 - i0), abs(j1 - j0)
    si = 1 if i0 < i1 else -1
    sj = 1 if j0 < j1 else -1
    err = di - dj
    i, j = i0, j0
    while True:
        cells.append((i, j))
        if i == i1 and j == j1:
            break
        e2 = 2 * err
        if e2 > -dj:
            err -= dj
            i += si
        if e2 < di:
            err += di
            j += sj
    return cells


class GridMapNode(Node):
    def __init__(self):
        super().__init__('gridmap_node')
        gp = lambda n, v: self.declare_parameter(n, v).value

        self.fixed_frame = gp('fixed_frame', 'odom')
        self.res = gp('resolution', 0.05)
        size_m = gp('size_m', 12.0)
        self.origin_x = gp('origin_x', -size_m / 2.0)
        self.origin_y = gp('origin_y', -size_m / 2.0)
        self.nx = int(size_m / self.res)
        self.ny = int(size_m / self.res)
        self.max_range = gp('max_range', sm.LIDAR_MAX_RANGE)   # 8 m (caracterizacion MS200)
        self.clamp = gp('clamp', 6.0)
        self.inject_ms200 = gp('inject_ms200', True)           # ruido MS200 sim-to-real sobre /scan
        self.out_dir = gp('output_dir', '/ros2_ws/src/tareas_gazebo/figs')
        self.rng = np.random.default_rng(0)

        p_occ, p_free = gp('p_occ', 0.7), gp('p_free', 0.4)
        self.l_occ = math.log(p_occ / (1 - p_occ))
        self.l_free = math.log(p_free / (1 - p_free))

        self.L = np.zeros((self.ny, self.nx))
        self.path = []

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(LaserScan, '/scan', self.scan_cb, qos_profile_sensor_data)
        self.pub = self.create_publisher(OccupancyGrid, '/mapa_probabilistico', 1)
        self.create_timer(0.5, self.publish_map)   # 2 Hz
        self.n_scans = 0
        self.get_logger().info('gridmap_node listo (esperando /scan y TF odom->lidar)...')

    def w2g(self, x, y):
        j = int((x - self.origin_x) / self.res)
        i = int((y - self.origin_y) / self.res)
        return i, j

    def in_bounds(self, i, j):
        return 0 <= i < self.ny and 0 <= j < self.nx

    def scan_cb(self, scan: LaserScan):
        # pose del lidar en el frame fijo
        try:
            tf = self.tf_buffer.lookup_transform(
                self.fixed_frame, scan.header.frame_id, rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return
        tx = tf.transform.translation.x
        ty = tf.transform.translation.y
        tyaw = yaw_from_quat(tf.transform.rotation)
        self.path.append((tx, ty))

        i0, j0 = self.w2g(tx, ty)
        if not self.in_bounds(i0, j0):
            return

        rmax = min(self.max_range, scan.range_max)
        ang = scan.angle_min
        for r in scan.ranges:
            a = ang
            ang += scan.angle_increment
            # sim-to-real: inyecta ruido del MS200 (sigma(rango)+dropout+espurios) al /scan real
            if self.inject_ms200 and math.isfinite(r) and r < rmax:
                r = sm.corrupt_lidar_range(r, self.rng, rmax)
            hit = math.isfinite(r) and scan.range_min <= r < rmax
            if not hit:
                continue          # sin retorno (dropout) -> no aporta informacion
            end_r = r
            ex = tx + end_r * math.cos(tyaw + a)
            ey = ty + end_r * math.sin(tyaw + a)
            i1, j1 = self.w2g(ex, ey)
            i1 = min(max(i1, 0), self.ny - 1)
            j1 = min(max(j1, 0), self.nx - 1)
            cells = bresenham(i0, j0, i1, j1)
            for (ci, cj) in cells[:-1]:
                self.L[ci, cj] += self.l_free
            ci, cj = cells[-1]
            self.L[ci, cj] += self.l_occ if hit else self.l_free
        np.clip(self.L, -self.clamp, self.clamp, out=self.L)
        self.n_scans += 1

    def prob(self):
        return 1.0 - 1.0 / (1.0 + np.exp(self.L))

    def publish_map(self):
        if self.n_scans == 0:
            return
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.fixed_frame
        msg.info.resolution = self.res
        msg.info.width = self.nx
        msg.info.height = self.ny
        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.info.origin.orientation.w = 1.0

        data = np.full(self.L.shape, -1, dtype=np.int8)   # desconocido
        seen = self.L != 0.0
        prob = self.prob()
        data[seen] = (prob[seen] * 100).astype(np.int8)
        msg.data = data.flatten().tolist()                # row-major (i*nx + j)
        self.pub.publish(msg)

    def save_plot(self):
        if self.n_scans == 0:
            return
        os.makedirs(self.out_dir, exist_ok=True)
        extent = [self.origin_x, self.origin_x + self.nx * self.res,
                  self.origin_y, self.origin_y + self.ny * self.res]
        plt.figure(figsize=(8, 8))
        plt.imshow(self.prob(), origin='lower', extent=extent, cmap='Greys', vmin=0, vmax=1)
        if self.path:
            p = np.array(self.path)
            plt.plot(p[:, 0], p[:, 1], 'r-', lw=1, label='trayectoria lidar')
            plt.legend()
        plt.title(f'Gazebo: mapa de ocupacion ({self.n_scans} escaneos)\n'
                  'negro=ocupado, blanco=libre, gris=desconocido')
        plt.xlabel('x [m]'); plt.ylabel('y [m]')
        plt.tight_layout()
        plt.savefig(os.path.join(self.out_dir, 'gz_10_mapa_ocupacion.png'), dpi=130)
        plt.close()
        self.get_logger().info(f'Mapa guardado ({self.n_scans} escaneos) en {self.out_dir}')


def main():
    rclpy.init()
    node = GridMapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.save_plot()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
