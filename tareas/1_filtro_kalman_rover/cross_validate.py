"""
TAREA 1 - Cross-validacion (Monte Carlo) del filtro de Kalman.

Se corren N simulaciones independientes (distintas semillas de ruido) y se
comparan tres metricas, promediadas sobre las corridas:

  * RMSE de la MEDICION cruda contra el estado real  (linea base del sensor).
  * RMSE del ESTIMADO del KF contra el estado real    (debe ser menor).
  * Consistencia: porcentaje de muestras dentro de la banda +/- 2 sigma del KF
    (un filtro consistente deja ~95% de los errores dentro de la banda).

Tambien se compara el error de SEGUIMIENTO cuando el control usa el estado
estimado vs cuando usa el estado real (ideal), para mostrar que cerrar el lazo
con el KF es casi tan bueno como tener el estado perfecto.

Ejecutar:  python3 cross_validate.py
"""

import os

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from kf_rover import run_simulation, rmse, FIGS


def main(n_runs=40):
    rmse_meas = np.zeros((n_runs, 3))
    rmse_kf = np.zeros((n_runs, 3))
    inside_2s = np.zeros((n_runs, 3))
    track_est = np.zeros(n_runs)
    track_true = np.zeros(n_runs)

    for n in range(n_runs):
        # Control con estado estimado (KF).
        log = run_simulation(seed=100 + n, use_estimate=True)
        xi, y, xhat = log["xi"], log["y"], log["xhat"]
        for i in range(3):
            rmse_meas[n, i] = rmse(y[:, i], xi[:, i])
            rmse_kf[n, i] = rmse(xhat[:, i], xi[:, i])
            two_sig = 2.0 * np.sqrt(log["Pdiag"][:, i])
            inside_2s[n, i] = np.mean(np.abs(log["err_est"][:, i]) <= two_sig) * 100.0
        et = log["err_track"]
        track_est[n] = np.sqrt(np.mean(np.sum(et ** 2, axis=1)))

        # Mismo escenario pero control con estado real (referencia ideal).
        log_ref = run_simulation(seed=100 + n, use_estimate=False)
        et_ref = log_ref["err_track"]
        track_true[n] = np.sqrt(np.mean(np.sum(et_ref ** 2, axis=1)))

    names = ["x", "y", "theta"]
    print(f"\n=== Cross-validacion Monte Carlo ({n_runs} corridas) ===\n")
    print(f"{'estado':>7} | {'RMSE medicion':>14} | {'RMSE KF':>10} | "
          f"{'mejora':>7} | {'% dentro 2sigma':>15}")
    print("-" * 66)
    for i, nm in enumerate(names):
        m = rmse_meas[:, i].mean()
        k = rmse_kf[:, i].mean()
        print(f"{nm:>7} | {m:>14.4f} | {k:>10.4f} | {m / k:>6.2f}x | "
              f"{inside_2s[:, i].mean():>14.1f}%")

    print("\n--- Error de seguimiento (norma RMS, m) ---")
    print(f"  control con estado ESTIMADO (KF): {track_est.mean():.4f} "
          f"+/- {track_est.std():.4f}")
    print(f"  control con estado REAL (ideal) : {track_true.mean():.4f} "
          f"+/- {track_true.std():.4f}")

    # Grafica resumen.
    os.makedirs(FIGS, exist_ok=True)
    fig, ax = plt.subplots(1, 2, figsize=(12, 4.5))

    x = np.arange(3)
    w = 0.35
    ax[0].bar(x - w / 2, rmse_meas.mean(0), w, yerr=rmse_meas.std(0),
              capsize=4, label="Medicion cruda", color="0.7")
    ax[0].bar(x + w / 2, rmse_kf.mean(0), w, yerr=rmse_kf.std(0),
              capsize=4, label="Estimado KF", color="tab:red")
    ax[0].set_xticks(x)
    ax[0].set_xticklabels(["x [m]", "y [m]", "theta [rad]"])
    ax[0].set_ylabel("RMSE")
    ax[0].set_title(f"RMSE medicion vs KF ({n_runs} corridas)")
    ax[0].legend()
    ax[0].grid(True, axis="y", alpha=0.3)

    ax[1].bar([0, 1], [track_est.mean(), track_true.mean()],
              yerr=[track_est.std(), track_true.std()], capsize=5,
              color=["tab:red", "tab:green"])
    ax[1].set_xticks([0, 1])
    ax[1].set_xticklabels(["control con\nestado KF", "control con\nestado real"])
    ax[1].set_ylabel("RMS error de seguimiento [m]")
    ax[1].set_title("Costo de usar el estimado en el lazo")
    ax[1].grid(True, axis="y", alpha=0.3)

    fig.tight_layout()
    fig.savefig(os.path.join(FIGS, "05_cross_validation.png"), dpi=130)
    plt.close(fig)
    print("\nFigura de cross-validacion guardada en", FIGS)


if __name__ == "__main__":
    main()
