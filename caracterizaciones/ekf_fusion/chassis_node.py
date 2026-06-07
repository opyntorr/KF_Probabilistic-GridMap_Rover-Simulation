#!/usr/bin/env python3
# encoding: utf-8
"""
ROS2 chassis node for the JetAuto mecanum base.

Subscribes /cmd_vel, drives the I2C encoder-motor board, integrates wheel-odometry
from the commanded velocity (faithful to the ROS1 `odom_publisher.py`), and publishes
/odom plus the odom->base_footprint TF.

LOW-LEVEL MOTOR SAFETY FILTER (protects the motors against ANY command source —
teleop, the AGV brain, bugs, runaways):
  1. cmd_vel input clamp: |vx|,|vy| <= max_linear_speed ; |wz| <= max_angular_speed.
  2. per-wheel duty cap: each motor duty limited to +/- max_motor_duty (of 100).
  3. slew-rate limit: each motor duty ramps toward its target at <= motor_slew_rate
     duty/s, so commands never jump (e.g. full forward -> full reverse) — this is what
     prevents the current spikes / gear shock that actually damage geared motors.
  4. watchdog: if /cmd_vel goes stale (> cmd_vel_timeout), the target ramps to 0.
The board firmware already hard-clamps each speed to [-100, 100] as a final backstop.

Motor writes happen ONLY in the 50 Hz control loop (single writer), at a steady rate.

Port of jetauto_controller_main.py + odom_publisher.py to ROS2 Humble.
"""
import math
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

from .motor_board import MotorBoard, MOTOR_TYPE_JGB37
from .mecanum import MecanumChassis

# Covariances copied verbatim from the ROS1 odom_publisher.py
ODOM_POSE_COV = [1e-9, 0., 0., 0., 0., 0.,
                 0., 1e-3, 1e-9, 0., 0., 0.,
                 0., 0., 1e6, 0., 0., 0.,
                 0., 0., 0., 1e6, 0., 0.,
                 0., 0., 0., 0., 1e6, 0.,
                 0., 0., 0., 0., 0., 1e3]
ODOM_TWIST_COV = [1e-9, 0., 0., 0., 0., 0.,
                  0., 1e-3, 1e-9, 0., 0., 0.,
                  0., 0., 1e6, 0., 0., 0.,
                  0., 0., 0., 1e6, 0., 0.,
                  0., 0., 0., 0., 1e6, 0.,
                  0., 0., 0., 0., 0., 0.1]


def yaw_to_quaternion(yaw):
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def _clamp(v, lim):
    return max(-lim, min(lim, v))


class ChassisNode(Node):
    def __init__(self):
        super().__init__('jetauto_chassis')

        self.declare_parameter('i2c_bus', 7)
        self.declare_parameter('motor_type', MOTOR_TYPE_JGB37)
        self.declare_parameter('wheelbase_a', 103.0)
        self.declare_parameter('wheelbase_b', 97.0)
        self.declare_parameter('wheel_diameter', 96.5)
        self.declare_parameter('pulse_per_cycle', 4320.0)
        self.declare_parameter('go_factor', 1.0)
        self.declare_parameter('turn_factor', 1.0)
        self.declare_parameter('linear_correction_factor', 1.0)
        self.declare_parameter('angular_correction_factor', 1.0)
        self.declare_parameter('control_rate', 50.0)
        self.declare_parameter('cmd_vel_timeout', 0.5)
        # --- low-level motor safety filter ---
        self.declare_parameter('max_linear_speed', 0.5)    # m/s  : tope de |vx|,|vy|
        self.declare_parameter('max_angular_speed', 0.5)   # rad/s: tope de |wz| (cap conservador)
        self.declare_parameter('max_motor_duty', 85.0)     # tope por rueda (de 100)
        self.declare_parameter('motor_slew_rate', 250.0)   # duty/s: rampa de aceleracion
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_footprint')
        self.declare_parameter('publish_tf', True)

        gp = self.get_parameter
        i2c_bus = gp('i2c_bus').value
        self.go_factor = gp('go_factor').value
        self.turn_factor = gp('turn_factor').value
        self.lin_corr = gp('linear_correction_factor').value
        self.ang_corr = gp('angular_correction_factor').value
        self.rate = float(gp('control_rate').value)
        self.cmd_timeout = gp('cmd_vel_timeout').value
        self.max_lin = float(gp('max_linear_speed').value)
        self.max_ang = float(gp('max_angular_speed').value)
        self.max_duty = float(gp('max_motor_duty').value)
        self.slew_rate = float(gp('motor_slew_rate').value)
        self.odom_frame = gp('odom_frame_id').value
        self.base_frame = gp('base_frame_id').value
        self.publish_tf = gp('publish_tf').value

        try:
            self.board = MotorBoard(i2c_bus=i2c_bus, motor_type=gp('motor_type').value)
        except Exception as e:
            self.get_logger().error(
                f'No pude abrir la placa de motores en I2C bus {i2c_bus} (0x34): {e}')
            raise
        self.mecanum = MecanumChassis(
            self.board, a=gp('wheelbase_a').value, b=gp('wheelbase_b').value,
            wheel_diameter=gp('wheel_diameter').value, pulse_per_cycle=gp('pulse_per_cycle').value)

        self._lock = threading.Lock()
        self.cmd_vx = self.cmd_vy = self.cmd_wz = 0.0
        self.x = self.y = self.yaw = 0.0
        self.last_cmd_time = self.get_clock().now()
        self.cur_duties = [0.0, 0.0, 0.0, 0.0]   # estado de la rampa (duty actual por rueda)
        self.last_written = None
        self.watchdog_active = False

        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_cb, 10)
        self.dt = 1.0 / self.rate
        self.create_timer(self.dt, self.update)
        self.get_logger().info(
            f'jetauto_chassis listo (I2C bus {i2c_bus}, {self.rate:.0f} Hz). '
            f'Limites: lin<={self.max_lin} m/s, ang<={self.max_ang} rad/s, '
            f'duty<={self.max_duty:.0f}, rampa {self.slew_rate:.0f} duty/s.')

    def cmd_vel_cb(self, msg: Twist):
        """Solo guarda el objetivo (con clamp de entrada). NO escribe motores aqui:
        eso lo hace el lazo de control a 50 Hz, con tope de duty + rampa."""
        with self._lock:
            self.cmd_vx = self.go_factor * _clamp(msg.linear.x, self.max_lin)
            self.cmd_vy = self.go_factor * _clamp(msg.linear.y, self.max_lin)
            self.cmd_wz = self.turn_factor * _clamp(msg.angular.z, self.max_ang)
            self.last_cmd_time = self.get_clock().now()

    def update(self):
        now = self.get_clock().now()
        with self._lock:
            age = (now - self.last_cmd_time).nanoseconds * 1e-9
            stale = age > self.cmd_timeout
            vx, vy, wz = (0.0, 0.0, 0.0) if stale else (self.cmd_vx, self.cmd_vy, self.cmd_wz)
        if stale and not self.watchdog_active:
            self.get_logger().warn('cmd_vel viejo; rampa de motores a 0 (watchdog)')
            self.watchdog_active = True
        elif not stale:
            self.watchdog_active = False

        # --- filtro de seguridad de bajo nivel: tope de duty por rueda + rampa (slew) ---
        target = self.mecanum.duties_xyz(vx, vy, wz)
        target = [_clamp(d, self.max_duty) for d in target]
        step = self.slew_rate * self.dt
        for i in range(4):
            delta = target[i] - self.cur_duties[i]
            if delta > step:
                self.cur_duties[i] += step
            elif delta < -step:
                self.cur_duties[i] -= step
            else:
                self.cur_duties[i] = target[i]
        out = [int(round(d)) for d in self.cur_duties]
        if out != self.last_written:
            try:
                self.board.set_speeds(out)   # set_speeds re-satura a [-100,100] (backstop firmware)
                self.last_written = out
            except OSError as e:
                self.get_logger().warn(f'I2C write fallo: {e}')

        # dead-reckoning odometry from commanded body velocity (REP-103: x fwd, y left)
        self.x += (math.cos(self.yaw) * vx - math.sin(self.yaw) * vy) * self.dt * self.lin_corr
        self.y += (math.sin(self.yaw) * vx + math.cos(self.yaw) * vy) * self.dt * self.lin_corr
        self.yaw += wz * self.dt * self.ang_corr

        stamp = now.to_msg()
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = yaw_to_quaternion(self.yaw)
        odom.pose.covariance = ODOM_POSE_COV
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz
        odom.twist.covariance = ODOM_TWIST_COV
        self.odom_pub.publish(odom)

        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.rotation = yaw_to_quaternion(self.yaw)
            self.tf_broadcaster.sendTransform(t)

    def destroy_node(self):
        try:
            self.board.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ChassisNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
