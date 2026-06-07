"""
TAREA 1 - Integracion del Filtro de Kalman continuo + simulacion del rover.

Arquitectura (todo en tiempo continuo, integrado con Euler):

  1. Modelo cinematico generalizado del rover (comun/rover_model.py, paper Kelly).
  2. Propagacion del estado REAL con Euler + ruido de proceso (Euler-Maruyama).
  3. Sensor simulado: medicion ruidosa de la postura completa  y = xi + ruido.
  4. Filtro de Kalman continuo (Kalman-Bucy), linealizado en el estimado:

         xi_hat_dot = f(xi_hat,u) + L (y - C xi_hat)
         L          = P C^T R^-1                       (ganancia de Kalman)
         P_dot      = F P + P F^T + Q - P C^T R^-1 C P  (ec. de Riccati)

  5. Controlador de seguimiento de trayectoria que usa el ESTADO ESTIMADO:
         u = D^-1(theta_hat) [ qd_dot + Kp (qd - q_hat) ]

  6. Graficas de validacion: real vs medicion vs estimado vs deseada.

Ejecutar:  python3 kf_rover.py
Genera las figuras en ./figs/
"""

import os
import sys

import numpy as np
import matplotlib

matplotlib.use("Agg")  # backend sin ventana -> corre headless en el docker
import matplotlib.pyplot as plt

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "comun")))
import rover_model as rm  # noqa: E402

FIGS = os.path.join(os.path.dirname(__file__), "figs")


# --------------------------------------------------------------------------- #
# Trayectoria deseada (escenario 2 del paper: circulo), en metros.
# --------------------------------------------------------------------------- #
def desired_trajectory(t, R=0.8, omega=2 * np.pi / 40.0):
    qd = np.array([R * np.sin(omega * t), R * np.cos(omega * t)])
    qd_dot = np.array([R * omega * np.cos(omega * t), -R * omega * np.sin(omega * t)])
    return qd, qd_dot


# --------------------------------------------------------------------------- #
# Simulacion completa. Devuelve un diccionario con los registros temporales.
# --------------------------------------------------------------------------- #
def run_simulation(seed=0, T=40.0, dt=0.002,
                   sigma_meas=(0.05, 0.05, np.deg2rad(4.0)),
                   sigma_proc=(0.005, 0.005, np.deg2rad(1.0)),
                   Kp_gain=1.0, u_max=60.0, use_estimate=True):
    """
    use_estimate=True  -> el control usa el estado del filtro de Kalman.
    use_estimate=False -> el control usa el estado real (caso ideal de referencia).
    """
    rng = np.random.default_rng(seed)
    p = rm.RoverParams()
    Kp = np.diag([Kp_gain, Kp_gain])

    # Covarianzas (densidades) del filtro de Kalman.
    Qc = np.diag(np.array(sigma_proc) ** 2)   # ruido de proceso
    Rc = np.diag(np.array(sigma_meas) ** 2)   # ruido de medicion
    C = np.eye(3)                              # se mide la postura completa
    Rc_inv = np.linalg.inv(Rc)
    cholQ = np.linalg.cholesky(Qc)
    cholR = np.linalg.cholesky(Rc)

    # Condiciones iniciales.
    xi = np.array([0.0, 0.0, np.pi / 2])          # estado REAL (theta0 = pi/2)
    xi_hat = np.array([0.15, -0.15, np.pi / 3])   # estimado inicial CON error
    P = np.diag([0.1, 0.1, 0.1])                  # incertidumbre inicial

    N = int(T / dt)
    log = {k: np.zeros((N, dim)) for k, dim in
           [("t", 1), ("xi", 3), ("y", 3), ("xhat", 3), ("qd", 2),
            ("err_est", 3), ("err_track", 2), ("Pdiag", 3)]}

    for k in range(N):
        t = k * dt
        qd, qd_dot = desired_trajectory(t)

        # --- Control (usa estimado o real segun bandera) ---
        x_for_ctrl = xi_hat if use_estimate else xi
        u = rm.controller(x_for_ctrl, qd, qd_dot, Kp, p)
        u = np.clip(u, -u_max, u_max)  # saturacion de las ruedas (seguridad)

        # --- Sensor: medicion ruidosa de la postura ---
        y = xi + cholR @ rng.standard_normal(3)

        # --- Filtro de Kalman continuo (Kalman-Bucy), un paso de Euler ---
        F = rm.jacobian_f(xi_hat, u, p)
        L = P @ C.T @ Rc_inv
        xi_hat = xi_hat + dt * (rm.f_kinematics(xi_hat, u, p) + L @ (y - C @ xi_hat))
        Pdot = F @ P + P @ F.T + Qc - P @ C.T @ Rc_inv @ C @ P
        P = P + dt * Pdot
        P = 0.5 * (P + P.T)  # mantener simetria

        # --- Registro (antes de propagar la verdad) ---
        log["t"][k] = t
        log["xi"][k] = xi
        log["y"][k] = y
        log["xhat"][k] = xi_hat
        log["qd"][k] = qd
        log["err_est"][k] = xi - xi_hat
        log["err_track"][k] = qd - xi[0:2]
        log["Pdiag"][k] = np.diag(P)

        # --- Propagacion del estado REAL: Euler + ruido de proceso ---
        xi = xi + dt * rm.f_kinematics(xi, u, p) + np.sqrt(dt) * (cholQ @ rng.standard_normal(3))

    log["params"] = dict(T=T, dt=dt, sigma_meas=sigma_meas, sigma_proc=sigma_proc,
                         Kp=Kp_gain, use_estimate=use_estimate)
    return log


# --------------------------------------------------------------------------- #
# Graficas de validacion
# --------------------------------------------------------------------------- #
def make_plots(log):
    os.makedirs(FIGS, exist_ok=True)
    t = log["t"][:, 0]
    xi, y, xhat, qd = log["xi"], log["y"], log["xhat"], log["qd"]

    # 1) Plano XY: deseada / real / medicion / estimada
    plt.figure(figsize=(7, 7))
    s = slice(None, None, 5)  # submuestreo de las mediciones para no saturar
    plt.scatter(y[s, 0], y[s, 1], s=6, c="0.7", alpha=0.5, label="Medicion (sensor)")
    plt.plot(qd[:, 0], qd[:, 1], "g--", lw=2, label="Deseada $q_d$")
    plt.plot(xi[:, 0], xi[:, 1], "b-", lw=1.8, label="Real")
    plt.plot(xhat[:, 0], xhat[:, 1], "r-", lw=1.2, label="Estimada (KF)")
    plt.plot(xi[0, 0], xi[0, 1], "ko", ms=7, label="Inicio real")
    plt.axis("equal")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.title("Seguimiento de trayectoria con estado estimado por KF")
    plt.legend(loc="best")
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(os.path.join(FIGS, "01_plano_xy.png"), dpi=130)
    plt.close()

    # 2) Series de tiempo de cada estado
    names = ["x [m]", "y [m]", r"$\theta$ [rad]"]
    fig, ax = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    for i in range(3):
        ax[i].plot(t, y[:, i], ".", ms=1.5, c="0.7", alpha=0.5, label="Medicion")
        ax[i].plot(t, xi[:, i], "b-", lw=1.5, label="Real")
        ax[i].plot(t, xhat[:, i], "r-", lw=1.2, label="Estimado")
        ax[i].set_ylabel(names[i])
        ax[i].grid(True, alpha=0.3)
    ax[0].legend(loc="upper right", ncol=3)
    ax[2].set_xlabel("t [s]")
    fig.suptitle("Estados: real vs medicion vs estimado")
    fig.tight_layout()
    fig.savefig(os.path.join(FIGS, "02_estados_tiempo.png"), dpi=130)
    plt.close(fig)

    # 3) Error de estimacion con bandas +/- 2 sigma (consistencia del filtro)
    err = log["err_est"]
    two_sig = 2.0 * np.sqrt(log["Pdiag"])
    fig, ax = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    for i in range(3):
        ax[i].plot(t, err[:, i], "k-", lw=1.0, label="Error real-estimado")
        ax[i].plot(t, two_sig[:, i], "r--", lw=1.0, label=r"$\pm 2\sigma$ (KF)")
        ax[i].plot(t, -two_sig[:, i], "r--", lw=1.0)
        ax[i].set_ylabel("err " + names[i])
        ax[i].grid(True, alpha=0.3)
    ax[0].legend(loc="upper right")
    ax[2].set_xlabel("t [s]")
    fig.suptitle("Error de estimacion del KF y bandas de covarianza")
    fig.tight_layout()
    fig.savefig(os.path.join(FIGS, "03_error_estimacion.png"), dpi=130)
    plt.close(fig)

    # 4) Error de seguimiento q_tilde = qd - q (real)
    et = log["err_track"]
    plt.figure(figsize=(10, 4))
    plt.plot(t, et[:, 0], label=r"$\tilde{x}$")
    plt.plot(t, et[:, 1], label=r"$\tilde{y}$")
    plt.axhline(0, c="k", lw=0.6)
    plt.xlabel("t [s]")
    plt.ylabel("error de seguimiento [m]")
    plt.title("Error de seguimiento de posicion (control con estado estimado)")
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(os.path.join(FIGS, "04_error_seguimiento.png"), dpi=130)
    plt.close()

    print("Figuras guardadas en", FIGS)


def rmse(a, b):
    return float(np.sqrt(np.mean((a - b) ** 2)))


def main():
    log = run_simulation(seed=1)
    make_plots(log)

    # Resumen numerico rapido (RMSE medicion vs estimado).
    xi, y, xhat = log["xi"], log["y"], log["xhat"]
    print("\n--- RMSE (una corrida) ---")
    for i, nm in enumerate(["x", "y", "theta"]):
        print(f"  {nm:5s}: medicion = {rmse(y[:, i], xi[:, i]):.4f}   "
              f"estimado KF = {rmse(xhat[:, i], xi[:, i]):.4f}")
    et = log["err_track"]
    print(f"  RMS error de seguimiento (norma) = "
          f"{np.sqrt(np.mean(np.sum(et**2, axis=1))):.4f} m")


if __name__ == "__main__":
    main()
