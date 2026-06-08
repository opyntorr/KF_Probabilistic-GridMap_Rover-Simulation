#!/usr/bin/env python3
"""
TAREA 1 en Isaac Sim (FÍSICA) — Filtro de Kalman sim-to-real.

Versión para la escena POR FÍSICA (scene_mecanum.py): como las ruedas giran de
verdad, los encoders salen de `/joint_states` REAL (FK mecanum), igual que en
Gazebo (más sim-to-real que tomar el twist). El resto es idéntico:

  PREDICCIÓN (50 Hz): encoders (/joint_states -> FK mecanum) + ruido caracterizado,
                      IMU yaw-rate (de /odom.twist) + bias 0.199°/s + ARW  -> deriva.
  CORRECCIÓN (15 Hz): pose absoluta LiDAR (= /odom + ruido MS200) -> ancla la deriva.
  GROUND TRUTH: /odom (pose física del chasís). No entra al control.

Trayectoria circular alineada al rumbo inicial. Control = Modelo 1 de Kelly (twist).

Correr:  source /opt/ros/humble/setup.bash && source isaac/isaac_env.sh
         python3 isaac/kf_control_isaac.py     # con scene_mecanum.py corriendo
"""

import math
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import rover_model as rm
import sensor_models as sm

HERE = os.path.dirname(os.path.abspath(__file__))
WHEELS = ['front_left_wheel_joint', 'front_right_wheel_joint',
          'back_left_wheel_joint', 'back_right_wheel_joint']


def yaw_from_quat(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class KFControlIsaac(Node):
    def __init__(self):
        super().__init__('kf_control_isaac',
                         parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        gp = lambda n, v: self.declare_parameter(n, v).value

        self.h = gp('h', 0.30)   # offset Kelly (= robot real); ganancia angular ~Kp/h
        kp = gp('kp', 0.8)
        self.Kp = np.diag([kp, kp])
        self.cmd_alpha = gp('cmd_lpf_alpha', 0.8)  # LPF de salida (= robot real); 1.0 = sin filtro
        self.v_cmd_prev = 0.0
        self.w_cmd_prev = 0.0
        self.ctrl_on_gt = gp('ctrl_on_gt', False)  # DIAGNOSTICO: controlar sobre GT limpio (sin ruido KF)
        self.open_loop = gp('open_loop', False)     # DIAGNOSTICO: círculo con twist constante, sin realimentación
        self.R_traj = gp('traj_radius', 0.6)
        self.omega_t = gp('traj_omega', 2 * np.pi / 70.0)
        self.duration = gp('duration_s', 90.0)
        self.rate = gp('control_rate', sm.IMU_RATE_HZ)
        self.out_dir = gp('output_dir', os.path.join(HERE, 'figs'))
        self.dt = 1.0 / self.rate
        self.lidar_every = max(1, int(round(self.rate / sm.LIDAR_RATE_HZ)))

        self.sig_v = sm.enc_linear_std(self.dt)
        self.sig_w = sm.imu_arw_rate_std(self.dt)
        self.R_lidar = np.diag([sm.LIDAR_POSE_SIGMA_XY**2,
                                sm.LIDAR_POSE_SIGMA_XY**2,
                                sm.LIDAR_POSE_SIGMA_YAW**2])

        self.xi_hat = None
        self.xi_odo = None
        self.P = np.diag([0.02, 0.02, 0.02])
        self.gt = None
        self.gt_w = 0.0
        self.wheel_vel = None
        self.center = None
        self.phi = 0.0
        self.t0 = None
        self.k = 0
        self.rng = np.random.default_rng(0)
        self.log = []
        self.saved = False

        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_timer(self.dt, self.step)
        self.get_logger().info('kf_control_isaac (física, encoders reales) listo...')

    def odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        self.gt = np.array([p.x, p.y, yaw_from_quat(msg.pose.pose.orientation)])
        self.gt_w = msg.twist.twist.angular.z

    def joint_cb(self, msg: JointState):
        idx = {n: i for i, n in enumerate(msg.name)}
        if all(w in idx for w in WHEELS) and len(msg.velocity) >= len(msg.name):
            self.wheel_vel = np.array([msg.velocity[idx[w]] for w in WHEELS])

    def desired(self, t):
        R, w, phi = self.R_traj, self.omega_t, self.phi
        qd = self.center + np.array([R * np.cos(w * t + phi), R * np.sin(w * t + phi)])
        qd_dot = np.array([-R * w * np.sin(w * t + phi), R * w * np.cos(w * t + phi)])
        return qd, qd_dot

    def step(self):
        if self.gt is None or self.wheel_vel is None:
            return
        if self.xi_hat is None:
            self.xi_hat = self.gt.copy()
            self.xi_odo = self.gt.copy()
            th0 = self.gt[2]
            self.phi = th0 - np.pi / 2.0
            qp0 = rm.point_p(self.gt, self.h)
            self.center = qp0 - self.R_traj * np.array([np.cos(self.phi), np.sin(self.phi)])
            self.t0 = self.get_clock().now().nanoseconds * 1e-9
            return

        t = self.get_clock().now().nanoseconds * 1e-9 - self.t0
        dt = self.dt

        # --- SENSORES con ruido caracterizado ---
        vx, _, _ = rm.mecanum_forward(*self.wheel_vel, r=sm.WHEEL_RADIUS, k=sm.WHEEL_K)
        v_enc = sm.corrupt_encoder_forward(vx, self.rng, dt)         # encoders REALES + ruido
        w_imu = sm.corrupt_imu_yaw_rate(self.gt_w, self.rng, dt)     # IMU (bias+ARW)

        # --- PREDICCIÓN del KF ---
        F = rm.jacobian_unicycle(self.xi_hat, v_enc, w_imu)
        th = self.xi_hat[2]
        G = np.array([[np.cos(th), 0.0], [np.sin(th), 0.0], [0.0, 1.0]])
        Q = G @ np.diag([self.sig_v**2, self.sig_w**2]) @ G.T / dt
        Q[2, 2] += sm.IMU_GYRO_Z_BIAS**2
        Q[0, 0] += 0.02**2
        Q[1, 1] += 0.02**2
        self.xi_hat = self.xi_hat + dt * rm.f_unicycle(self.xi_hat, v_enc, w_imu)
        self.P = self.P + dt * (F @ self.P + self.P @ F.T + Q)
        self.P = 0.5 * (self.P + self.P.T)
        self.xi_odo = self.xi_odo + dt * rm.f_unicycle(self.xi_odo, v_enc, w_imu)

        # --- CORRECCIÓN del KF (LiDAR @ 15 Hz) ---
        if self.k % self.lidar_every == 0:
            y = sm.corrupt_lidar_pose(self.gt.copy(), self.rng)
            S = self.P + self.R_lidar
            Kk = self.P @ np.linalg.inv(S)
            innov = y - self.xi_hat
            innov[2] = math.atan2(math.sin(innov[2]), math.cos(innov[2]))
            self.xi_hat = self.xi_hat + Kk @ innov
            self.P = (np.eye(3) - Kk) @ self.P
            self.P = 0.5 * (self.P + self.P.T)
        self.k += 1

        # --- CONTROL con estado estimado (o GT/lazo-abierto si diagnóstico) ---
        qd, qd_dot = self.desired(t)
        if self.open_loop:
            v, w = self.R_traj * self.omega_t, self.omega_t   # círculo SIN realimentación
            q_p = rm.point_p(self.xi_hat, self.h)
        else:
            xi_ctrl = self.gt if self.ctrl_on_gt else self.xi_hat
            q_p = rm.point_p(xi_ctrl, self.h)
            v, w = rm.controller_twist(q_p, xi_ctrl[2], qd, qd_dot, self.Kp, self.h)
        # saturar y luego SUAVIZAR (LPF exponencial), igual que el robot real:
        # es lo que rompe el ciclo límite (serpenteo).
        v = float(np.clip(v, -sm.V_MAX, sm.V_MAX))
        w = float(np.clip(w, -sm.W_MAX, sm.W_MAX))
        a = self.cmd_alpha
        self.v_cmd_prev = a * v + (1.0 - a) * self.v_cmd_prev
        self.w_cmd_prev = a * w + (1.0 - a) * self.w_cmd_prev
        msg = Twist()
        msg.linear.x = self.v_cmd_prev
        msg.angular.z = self.w_cmd_prev
        self.pub.publish(msg)

        gp_ = rm.point_p(self.gt, self.h)
        op_ = rm.point_p(self.xi_odo, self.h)
        self.log.append([t, *gp_, self.gt[2], *op_, self.xi_odo[2],
                         *q_p, self.xi_hat[2], *qd, *np.diag(self.P)])

        if t >= self.duration and not self.saved:
            self.pub.publish(Twist())
            self.save_plots()
            self.saved = True
            self.get_logger().info('Validación física terminada; figuras guardadas.')

    def save_plots(self):
        if not self.log:
            return
        os.makedirs(self.out_dir, exist_ok=True)
        d = np.array(self.log)
        np.save(os.path.join(self.out_dir, 'traj.npy'), d)
        t = d[:, 0]; g = d[:, 1:4]; o = d[:, 4:7]; h = d[:, 7:10]; qd = d[:, 10:12]; Pd = d[:, 12:15]
        names = ['x [m]', 'y [m]', 'theta [rad]']

        def wrap(a):
            return np.arctan2(np.sin(a), np.cos(a))

        plt.figure(figsize=(7.5, 7.5))
        plt.plot(qd[:, 0], qd[:, 1], 'g--', lw=2, label='Deseada')
        plt.plot(g[:, 0], g[:, 1], 'b-', lw=1.6, label='Ground truth (Isaac física)')
        plt.plot(o[:, 0], o[:, 1], color='orange', lw=1.2, label='Odometria sola (deriva)')
        plt.plot(h[:, 0], h[:, 1], 'r-', lw=1.1, label='Estimada KF (IMU+enc+LiDAR)')
        plt.axis('equal'); plt.grid(True, alpha=0.3); plt.legend(fontsize=9)
        plt.xlabel('x [m]'); plt.ylabel('y [m]')
        plt.title('Isaac Sim (física mecanum): KF vs odometria sola')
        plt.tight_layout()
        plt.savefig(os.path.join(self.out_dir, 'isaac_01_plano_xy.png'), dpi=130); plt.close()

        fig, ax = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
        for i in range(3):
            gi, oi, hi = g[:, i], o[:, i], h[:, i]
            if i == 2:
                gi, oi, hi = np.unwrap(gi), np.unwrap(oi), np.unwrap(hi)
            ax[i].plot(t, gi, 'b-', lw=1.4, label='Ground truth')
            ax[i].plot(t, oi, color='orange', lw=1.0, label='Odometria sola')
            ax[i].plot(t, hi, 'r-', lw=1.0, label='Estimado KF')
            ax[i].set_ylabel(names[i]); ax[i].grid(True, alpha=0.3)
        ax[0].legend(loc='upper right', ncol=3); ax[2].set_xlabel('t [s]')
        fig.suptitle('Isaac Sim (física): real vs odometria(deriva) vs KF')
        fig.tight_layout()
        fig.savefig(os.path.join(self.out_dir, 'isaac_02_estados_tiempo.png'), dpi=130); plt.close(fig)

        err = g - h
        err[:, 2] = wrap(g[:, 2] - h[:, 2])
        two = 2.0 * np.sqrt(Pd)
        fig, ax = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
        for i in range(3):
            ax[i].plot(t, err[:, i], 'k-', lw=1.0, label='Error GT-KF')
            ax[i].plot(t, two[:, i], 'r--', lw=1.0, label=r'$\pm2\sigma$')
            ax[i].plot(t, -two[:, i], 'r--', lw=1.0)
            ax[i].set_ylabel('err ' + names[i]); ax[i].grid(True, alpha=0.3)
        ax[0].legend(loc='upper right'); ax[2].set_xlabel('t [s]')
        fig.suptitle('Isaac Sim (física): error de estimacion del KF y covarianza')
        fig.tight_layout()
        fig.savefig(os.path.join(self.out_dir, 'isaac_03_error_estimacion.png'), dpi=130); plt.close(fig)

        def rmse(a, b):
            return float(np.sqrt(np.mean((a - b) ** 2)))

        def rmse_ang(a, b):
            return float(np.sqrt(np.mean(wrap(a - b) ** 2)))

        # --- métricas de seguimiento / serpenteo (GT del punto vs deseada) ---
        track = np.sqrt(((g[:, :2] - qd) ** 2).sum(1))            # error punto->deseada
        rad = np.sqrt(((g[:, :2] - self.center) ** 2).sum(1)) - self.R_traj
        dr = rad - np.median(rad)
        ncross = int((np.diff(np.sign(dr)) != 0).sum())           # zig-zags (cruces)
        self.get_logger().info(
            f'h={self.h:.2f} Kp={self.Kp[0,0]:.2f} | seguimiento RMS={np.sqrt(np.mean(track**2)):.4f} m '
            f'| serpenteo: std_radial={rad.std():.4f} m, cruces={ncross}')
        self.get_logger().info(
            f'RMSE vs GT  x: odo={rmse(o[:,0],g[:,0]):.3f} kf={rmse(h[:,0],g[:,0]):.3f} | '
            f'y: odo={rmse(o[:,1],g[:,1]):.3f} kf={rmse(h[:,1],g[:,1]):.3f} | '
            f'th: odo={rmse_ang(o[:,2],g[:,2]):.3f} kf={rmse_ang(h[:,2],g[:,2]):.3f}')


def main():
    rclpy.init()
    node = KFControlIsaac()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        if not node.saved:
            node.save_plots()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
