#!/usr/bin/env python3
# encoding: utf-8
"""
ROS2 IMU node: reads the MPU-6050 over I2C and publishes sensor_msgs/Imu (raw).
Orientation is left empty (covariance[0] = -1); fuse with imu_filter_madgwick to get
orientation on /imu/data. Port of mpu_6050_driver/imu_node.py + jetauto_sdk/imu.py.
"""
import statistics

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

from .mpu6050 import MPU6050


def _robust_mean(vals, k=8.0):
    """Media robusta del bias: rechaza |x-mediana| > k*1.4826*MAD y promedia los inliers.
    El gyro y mostro ~7.6% de spikes en la caracterizacion -> una media simple se sesgaria."""
    med = statistics.median(vals)
    mad = statistics.median([abs(v - med) for v in vals])
    if mad > 0.0:
        thr = k * 1.4826 * mad
        inl = [v for v in vals if abs(v - med) <= thr]
        if inl:
            return sum(inl) / len(inl)
    return med


class ImuNode(Node):
    def __init__(self):
        super().__init__('imu_node')
        self.declare_parameter('i2c_bus', 7)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('freq', 50.0)
        self.declare_parameter('calibrate_gyro', True)
        self.declare_parameter('gyro_calib_samples', 200)
        # Covarianzas (diagonal) del IMU, de la caracterizacion estatica de 10 h (2026-06-04,
        # tools/characterization). gyro: varianza de ruido blanco por eje = ARW^2/dt @50Hz, de
        # ARW [x 0.83, y 1.60, z 1.06 deg/sqrt(hr)] -> ~[2.9e-6, 1.1e-5, 4.8e-6] (rad/s)^2. El EKF
        # solo fusiona vyaw (z); subir 2-3x si el filtro va nervioso. accel NO lo fusiona el EKF
        # (imu0 accel=false) -> valor nominal, no caracterizado.
        self.declare_parameter('gyro_variance', [3.0e-6, 1.1e-5, 5.0e-6])
        self.declare_parameter('accel_variance', [2.0e-3, 2.0e-3, 2.0e-3])

        i2c_bus = self.get_parameter('i2c_bus').value
        self.frame_id = self.get_parameter('frame_id').value
        freq = float(self.get_parameter('freq').value)
        gv = list(self.get_parameter('gyro_variance').value)
        av = list(self.get_parameter('accel_variance').value)
        self.ang_cov = [gv[0], 0.0, 0.0, 0.0, gv[1], 0.0, 0.0, 0.0, gv[2]]
        self.acc_cov = [av[0], 0.0, 0.0, 0.0, av[1], 0.0, 0.0, 0.0, av[2]]

        try:
            self.imu = MPU6050(i2c_bus=i2c_bus)
        except Exception as e:
            self.get_logger().error(
                f'No pude abrir el IMU (MPU6050 0x68) en I2C bus {i2c_bus}: {e}')
            raise

        # Auto-calibracion de bias de gyro al arranque (robot QUIETO). Mide y resta el bias CRUDO del
        # MPU6050 (~0.019 rad/s = 1.1 deg/s en z, medido por arranque). La caracterizacion de 10 h
        # (2026-06-04) mostro que /imu/data_raw (ya con el bias restado) deja un RESIDUAL que deriva a
        # ~0.2 deg/s a lo largo de una sesion (bias instability/RRW) -> por eso el LiDAR sigue anclando
        # el yaw. Estimador ROBUSTO (mediana + rechazo MAD): el gyro y tiene ~7.6% de spikes que
        # sesgarian una media simple.
        import time as _t
        self.gbx = self.gby = self.gbz = 0.0
        if self.get_parameter('calibrate_gyro').value:
            n = int(self.get_parameter('gyro_calib_samples').value)
            xs, ys, zs = [], [], []
            for _ in range(n):
                try:
                    _, _, _, gx, gy, gz = self.imu.read()
                    xs.append(gx); ys.append(gy); zs.append(gz)
                except OSError:
                    pass
                _t.sleep(1.0 / freq)
            if xs:
                self.gbx = _robust_mean(xs)
                self.gby = _robust_mean(ys)
                self.gbz = _robust_mean(zs)
            self.get_logger().info(
                'gyro bias (rad/s, robusto): x=%.5f y=%.5f z=%.5f (n=%d, el robot debia estar quieto)'
                % (self.gbx, self.gby, self.gbz, len(xs)))

        self.pub = self.create_publisher(Imu, 'imu/data_raw', 10)
        self.create_timer(1.0 / freq, self.tick)
        self.get_logger().info(f'imu_node listo (I2C bus {i2c_bus}, {freq:.0f} Hz)')

    def tick(self):
        try:
            ax, ay, az, gx, gy, gz = self.imu.read()
        except OSError as e:
            self.get_logger().warn(f'I2C IMU read fallo: {e}')
            return
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az
        msg.angular_velocity.x = gx - self.gbx
        msg.angular_velocity.y = gy - self.gby
        msg.angular_velocity.z = gz - self.gbz
        # Covarianzas de la caracterizacion (antes iban en cero -> el EKF las forzaba a ~1e-6 y
        # confiaba ciegamente en el giro). Ahora el EKF pondera vyaw con su ruido real.
        msg.angular_velocity_covariance = self.ang_cov
        msg.linear_acceleration_covariance = self.acc_cov
        msg.orientation_covariance[0] = -1.0  # orientation unknown (sensor_msgs/Imu convention)
        self.pub.publish(msg)

    def destroy_node(self):
        try:
            self.imu.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
