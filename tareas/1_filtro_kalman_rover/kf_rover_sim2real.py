"""
TAREA 1 (sim-to-real) — Filtro de Kalman con ruido REAL de los sensores.

A diferencia de kf_rover.py (que medía la postura completa con C=I), aquí se
modela el conjunto de sensores REAL del JetAuto, con el ruido caracterizado en
caracterizaciones/ (ver comun/sensor_models.py):

  PREDICCION (propioceptiva, 50 Hz):
    * IMU MPU-6050  -> yaw rate (con bias residual 0.199°/s + ruido ARW)
    * Encoders      -> velocidad lineal (calibrada, con ruido de cuantizacion)
    -> dead-reckoning: el bias del giro hace DERIVAR el rumbo (como en el robot real).

  CORRECCION (exteroceptiva, 15 Hz):
    * LiDAR MS200   -> pose absoluta tipo scan-match/AMCL (sigma del MS200)
    -> ANCLA la deriva (replica la decision "lidar-dominante" del robot real).

Es un KF de tiempo continuo con prediccion continua (Euler) + correccion discreta
del LiDAR (forma continuo-discreta), porque los sensores son multi-tasa.

El control de seguimiento usa el estado estimado. Se compara contra la odometria
SIN correccion (dead-reckoning puro) para mostrar cuanto ancla el LiDAR.

Ejecutar:  python3 kf_rover_sim2real.py   -> figuras en ./figs/
"""

import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "comun")))
import rover_model as rm        # noqa: E402
import sensor_models as sm      # noqa: E402

FIGS = os.path.join(os.path.dirname(__file__), "figs")


def desired_trajectory(t, R=0.6, omega=2 * np.pi / 70.0):
    qd = np.array([R * np.sin(omega * t), -R * np.cos(omega * t)])
    qd_dot = np.array([R * omega * np.cos(omega * t), R * omega * np.sin(omega * t)])
    return qd, qd_dot


def run_simulation(seed=0, T=70.0, dt=1.0 / sm.IMU_RATE_HZ, h=0.30, Kp_gain=0.8):
    rng = np.random.default_rng(seed)
    Kp = np.diag([Kp_gain, Kp_gain])
    lidar_every = max(1, int(round(sm.IMU_RATE_HZ / sm.LIDAR_RATE_HZ)))  # 50/15 ~ 3

    # --- ruido / covarianzas DERIVADAS de la caracterizacion ---
    sig_v = sm.enc_linear_std(dt)           # std velocidad lineal encoders [m/s]
    sig_w = sm.imu_arw_rate_std(dt)         # std ruido blanco yaw IMU [rad/s]
    R_lidar = np.diag([sm.LIDAR_POSE_SIGMA_XY**2,
                       sm.LIDAR_POSE_SIGMA_XY**2,
                       sm.LIDAR_POSE_SIGMA_YAW**2])

    # Estados
    xi = np.array([0.0, 0.0, np.pi / 2])    # REAL (chasis, punto a)
    xi_hat = xi.copy()                      # estimado (KF)
    xi_odo = xi.copy()                      # dead-reckoning SIN correccion (referencia)
    P = np.diag([0.02, 0.02, 0.02])

    # centro del circulo para que qd(0) = punto p inicial (error inicial ~0)
    R_traj = 0.6
    q_p0 = rm.point_p(xi, h)
    center = q_p0 + np.array([0.0, R_traj])
    v_cmd = w_cmd = 0.0

    N = int(T / dt)
    L = {k: [] for k in ("t", "xi", "hat", "odo", "lidar", "qd", "Pdiag")}

    for k in range(N):
        t = k * dt
        # trayectoria deseada (relativa al centro)
        qd_rel, qd_dot = desired_trajectory(t, R=R_traj)
        qd = center + qd_rel

        # --- control con estado ESTIMADO ---
        q_p = rm.point_p(xi_hat, h)
        v_cmd, w_cmd = rm.controller_twist(q_p, xi_hat[2], qd, qd_dot, Kp, h)
        v_cmd = float(np.clip(v_cmd, -sm.V_MAX, sm.V_MAX))
        w_cmd = float(np.clip(w_cmd, -sm.W_MAX, sm.W_MAX))

        # --- planta REAL: el chasis ejecuta el comando con go/turn factor + ruido ---
        v_act = sm.GO_FACTOR * v_cmd
        w_act = sm.TURN_FACTOR * w_cmd
        xi = xi + dt * rm.f_unicycle(xi, v_act, w_act)
        xi = xi + np.sqrt(dt) * np.array([0.0, 0.0, 0.0])  # planta casi deterministica

        # --- SENSORES propioceptivos (con ruido real) ---
        v_enc = sm.corrupt_encoder_forward(v_act, rng, dt)
        w_imu = sm.corrupt_imu_yaw_rate(w_act, rng, dt)   # incluye bias 0.199°/s

        # --- PREDICCION del KF (continua, Euler) con la odometria ruidosa ---
        F = rm.jacobian_unicycle(xi_hat, v_enc, w_imu)
        # Q derivada del ruido de los sensores: G diag(sig_v^2, sig_w^2) G^T
        th = xi_hat[2]
        G = np.array([[np.cos(th), 0.0], [np.sin(th), 0.0], [0.0, 1.0]])
        Q = G @ np.diag([sig_v**2, sig_w**2]) @ G.T / dt
        Q[2, 2] += (sm.IMU_GYRO_Z_BIAS**2)   # cubre el bias no modelado del giro
        Q[0, 0] += 0.02**2                    # piso: slip lateral / error de modelo no modelado
        Q[1, 1] += 0.02**2
        xi_hat = xi_hat + dt * rm.f_unicycle(xi_hat, v_enc, w_imu)
        P = P + dt * (F @ P + P @ F.T + Q)
        P = 0.5 * (P + P.T)

        # dead-reckoning SIN correccion (misma odometria, sin lidar)
        xi_odo = xi_odo + dt * rm.f_unicycle(xi_odo, v_enc, w_imu)

        # --- CORRECCION del KF (discreta) con la pose del LiDAR @ 15 Hz ---
        lidar_meas = np.array([np.nan, np.nan, np.nan])
        if k % lidar_every == 0:
            y = sm.corrupt_lidar_pose(xi.copy(), rng)
            S = P + R_lidar
            Kk = P @ np.linalg.inv(S)
            innov = y - xi_hat
            innov[2] = np.arctan2(np.sin(innov[2]), np.cos(innov[2]))  # wrap yaw
            xi_hat = xi_hat + Kk @ innov
            P = (np.eye(3) - Kk) @ P
            P = 0.5 * (P + P.T)
            lidar_meas = y

        L["t"].append(t)
        L["xi"].append(rm.point_p(xi, h).tolist() + [xi[2]])
        L["hat"].append(rm.point_p(xi_hat, h).tolist() + [xi_hat[2]])
        L["odo"].append(rm.point_p(xi_odo, h).tolist() + [xi_odo[2]])
        L["lidar"].append(rm.point_p(lidar_meas, h).tolist() + [lidar_meas[2]]
                          if np.isfinite(lidar_meas[0]) else [np.nan, np.nan, np.nan])
        L["qd"].append(qd.tolist())
        L["Pdiag"].append(np.diag(P).tolist())

    return {k: np.array(v) for k, v in L.items()}


def rmse(a, b):
    return float(np.sqrt(np.mean((a - b) ** 2)))


def make_plots(d):
    os.makedirs(FIGS, exist_ok=True)
    t = d["t"]
    xi, hat, odo, lid, qd, Pd = d["xi"], d["hat"], d["odo"], d["lidar"], d["qd"], d["Pdiag"]

    # 1) plano XY
    plt.figure(figsize=(7.5, 7.5))
    plt.plot(qd[:, 0], qd[:, 1], "g--", lw=2, label="Deseada $q_d$")
    plt.plot(xi[:, 0], xi[:, 1], "b-", lw=1.8, label="Real")
    plt.plot(odo[:, 0], odo[:, 1], color="orange", lw=1.3, label="Odometria sin correccion (deriva)")
    plt.plot(hat[:, 0], hat[:, 1], "r-", lw=1.2, label="Estimada KF (IMU+enc+LiDAR)")
    m = np.isfinite(lid[:, 0])
    plt.scatter(lid[m, 0], lid[m, 1], s=8, c="0.6", alpha=0.5, label="Ancla LiDAR (15 Hz)")
    plt.plot(xi[0, 0], xi[0, 1], "ko", ms=6)
    plt.axis("equal"); plt.grid(True, alpha=0.3)
    plt.xlabel("x [m]"); plt.ylabel("y [m]")
    plt.title("Sim-to-real: KF (IMU+encoders + ancla LiDAR) vs odometria sola")
    plt.legend(loc="best", fontsize=9); plt.tight_layout()
    plt.savefig(os.path.join(FIGS, "06_s2r_plano_xy.png"), dpi=130); plt.close()

    # 2) estados vs tiempo
    names = ["x [m]", "y [m]", r"$\theta$ [rad]"]
    fig, ax = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    for i in range(3):
        ax[i].plot(t, xi[:, i], "b-", lw=1.5, label="Real")
        ax[i].plot(t, odo[:, i], color="orange", lw=1.1, label="Odometria sola")
        ax[i].plot(t, hat[:, i], "r-", lw=1.1, label="Estimado KF")
        ax[i].set_ylabel(names[i]); ax[i].grid(True, alpha=0.3)
    ax[0].legend(loc="upper right", ncol=3); ax[2].set_xlabel("t [s]")
    fig.suptitle("Sim-to-real: real vs odometria(deriva) vs estimado KF")
    fig.tight_layout()
    fig.savefig(os.path.join(FIGS, "07_s2r_estados_tiempo.png"), dpi=130); plt.close(fig)

    # 3) error de estimacion KF con bandas +/-2sigma
    err = xi - hat
    two = 2.0 * np.sqrt(Pd)
    fig, ax = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    for i in range(3):
        ax[i].plot(t, err[:, i], "k-", lw=1.0, label="Error real-KF")
        ax[i].plot(t, two[:, i], "r--", lw=1.0, label=r"$\pm 2\sigma$")
        ax[i].plot(t, -two[:, i], "r--", lw=1.0)
        ax[i].set_ylabel("err " + names[i]); ax[i].grid(True, alpha=0.3)
    ax[0].legend(loc="upper right"); ax[2].set_xlabel("t [s]")
    fig.suptitle("Sim-to-real: error de estimacion del KF y covarianza")
    fig.tight_layout()
    fig.savefig(os.path.join(FIGS, "08_s2r_error_estimacion.png"), dpi=130); plt.close(fig)

    print("Figuras sim-to-real guardadas en", FIGS)
    print("\n--- RMSE vs estado real (una corrida) ---")
    for i, nm in enumerate(["x", "y", "theta"]):
        print(f"  {nm:5s}: odometria sola = {rmse(odo[:, i], xi[:, i]):.4f}   "
              f"KF (con LiDAR) = {rmse(hat[:, i], xi[:, i]):.4f}")


def main():
    d = run_simulation(seed=1)
    make_plots(d)


if __name__ == "__main__":
    main()
