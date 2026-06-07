#!/usr/bin/env python3
"""
imu_allan_analyze.py — caracteriza el IMU (MPU-6050) desde un rosbag estatico:
  - Bias del giroscopio (robusto, mediana por eje).
  - Varianza de Allan (overlapping) por eje de giro -> ARW y bias instability.
  - Drift del bias en el tiempo (ventanas deslizantes) = deriva termica/largo plazo.
  - Rechazo de outliers (golpes/movimientos) por MAD antes de la Allan.

Corre en el venv:  source ~/evo_venv/bin/activate  (necesita rosbags + numpy + matplotlib)
    python3 imu_allan_analyze.py ~/static_runs/<nombre>  [--topic /imu/data_raw] [--k 8]

Salida en la carpeta del bag: imu_summary.txt, allan_gyro.png, bias_drift.png
"""
import argparse, sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore

R2D = 180.0 / np.pi
# Los bags de ROS2 Humble no embeben las definiciones de tipos -> hay que darle un typestore.
TYPESTORE = get_typestore(Stores.ROS2_HUMBLE)


def read_imu(bagdir, topic):
    ts, g, a = [], [[], [], []], [[], [], []]
    with AnyReader([Path(bagdir)], default_typestore=TYPESTORE) as reader:
        conns = [c for c in reader.connections if c.topic == topic]
        if not conns:
            sys.exit(f"No hay topic {topic} en el bag. Topics: {[c.topic for c in reader.connections]}")
        for conn, _t, raw in reader.messages(connections=conns):
            m = reader.deserialize(raw, conn.msgtype)
            ts.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
            w, l = m.angular_velocity, m.linear_acceleration
            g[0].append(w.x); g[1].append(w.y); g[2].append(w.z)
            a[0].append(l.x); a[1].append(l.y); a[2].append(l.z)
    return np.asarray(ts), np.asarray(g), np.asarray(a)


def reject_outliers(x, k):
    """Reemplaza |x-mediana|>k*MAD por la mediana (mantiene continuidad para la Allan). Devuelve (x_limpio, frac)."""
    med = np.median(x)
    mad = np.median(np.abs(x - med)) * 1.4826
    if mad <= 0:
        return x.copy(), 0.0
    bad = np.abs(x - med) > k * mad
    out = x.copy(); out[bad] = med
    return out, bad.mean()


def overlapping_adev(rate, fs):
    N = len(rate)
    theta = np.cumsum(rate) / fs            # angulo (integral de la tasa)
    maxm = (N - 1) // 2
    if maxm < 2:
        return np.array([]), np.array([])
    ms = np.unique(np.floor(np.logspace(0, np.log10(maxm), 80)).astype(np.int64))
    ms = ms[(ms >= 1) & (ms <= maxm)]
    taus = ms / fs
    adev = np.empty(len(ms))
    for i, m in enumerate(ms):
        tau = m / fs
        d = theta[2 * m:] - 2.0 * theta[m:N - m] + theta[:N - 2 * m]
        adev[i] = np.sqrt(np.sum(d * d) / (2.0 * tau * tau * (N - 2 * m)))
    return taus, adev


def arw_and_bi(taus, adev):
    # ARW = adev en tau=1s (deg/sqrt(hr)); bias instability = min(adev)/0.664 (deg/hr)
    arw = np.interp(1.0, taus, adev) if taus[0] <= 1.0 <= taus[-1] else adev[np.argmin(np.abs(taus - 1.0))]
    arw_dsh = arw * R2D * 60.0
    bi = adev.min() / 0.664
    bi_dh = bi * R2D * 3600.0
    return arw_dsh, bi_dh, taus[np.argmin(adev)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag', help='carpeta del rosbag (~/static_runs/<nombre>)')
    ap.add_argument('--topic', default='/imu/data_raw')
    ap.add_argument('--k', type=float, default=8.0, help='umbral MAD para outliers')
    args = ap.parse_args()
    bag = Path(args.bag).expanduser()

    ts, g, a = read_imu(bag, args.topic)
    if len(ts) < 100:
        sys.exit(f"Muy pocas muestras IMU ({len(ts)}).")
    dur = ts[-1] - ts[0]
    fs = 1.0 / np.median(np.diff(ts))
    axes = ['x', 'y', 'z']
    L = []
    L.append(f"IMU {args.topic}: {len(ts)} muestras, {dur/60:.1f} min, fs~{fs:.1f} Hz")
    L.append("")

    # --- bias robusto + outliers + Allan por eje ---
    fig, axp = plt.subplots(figsize=(7, 5))
    for i, ax_name in enumerate(axes):
        raw = g[i]
        clean, frac = reject_outliers(raw, args.k)
        bias = np.median(clean)
        taus, adev = overlapping_adev(clean - bias, fs)  # quita bias DC para la Allan
        if len(taus) == 0:
            continue
        arw, bi, tau_bi = arw_and_bi(taus, adev)
        L.append(f"GYRO {ax_name}: bias = {bias*R2D:.4f} deg/s ({bias:.6f} rad/s) | "
                 f"outliers descartados = {100*frac:.2f}%")
        L.append(f"         ARW = {arw:.3f} deg/sqrt(hr) | bias instability = {bi:.2f} deg/hr (@tau={tau_bi:.1f}s)")
        axp.loglog(taus, adev * R2D, label=f'gyro {ax_name}')
    axp.set_xlabel('tau (s)'); axp.set_ylabel('Allan deviation (deg/s)')
    axp.set_title('Allan deviation - giroscopio'); axp.grid(True, which='both', alpha=0.3); axp.legend()
    fig.tight_layout(); fig.savefig(bag / 'allan_gyro.png', dpi=110); plt.close(fig)

    # --- accel: bias/gravedad robusto (referencia) ---
    L.append("")
    for i, ax_name in enumerate(axes):
        clean, _ = reject_outliers(a[i], args.k)
        L.append(f"ACCEL {ax_name}: mediana = {np.median(clean):.4f} m/s^2")

    # --- drift de bias del giro en el tiempo (ventanas) ---
    L.append("")
    L.append("=== Drift de bias del giro (mediana por ventana de 60 s) ===")
    win = max(int(60 * fs), 100)
    fig2, ax2 = plt.subplots(figsize=(7, 4))
    for i, ax_name in enumerate(axes):
        clean, _ = reject_outliers(g[i], args.k)
        nb = len(clean) // win
        if nb < 2:
            continue
        seg = clean[:nb * win].reshape(nb, win)
        med = np.median(seg, axis=1) * R2D
        tw = (np.arange(nb) * win / fs) / 60.0
        ax2.plot(tw, med, label=f'gyro {ax_name}')
        L.append(f"  gyro {ax_name}: rango del bias = {med.max()-med.min():.4f} deg/s a lo largo de la captura")
    ax2.set_xlabel('tiempo (min)'); ax2.set_ylabel('bias (deg/s)')
    ax2.set_title('Drift de bias del giro'); ax2.grid(True, alpha=0.3)
    if ax2.get_legend_handles_labels()[0]:  # solo si hubo >=2 ventanas (corrida larga)
        ax2.legend()
    fig2.tight_layout(); fig2.savefig(bag / 'bias_drift.png', dpi=110); plt.close(fig2)

    L.append("")
    L.append("USO: el bias (sobre todo gyro z) -> restar/EKF; ARW+bias instability -> robot_localization")
    L.append("     imu0 noise + process_noise_covariance y ganancias de madgwick.")
    txt = "\n".join(L) + "\n"
    (bag / 'imu_summary.txt').write_text(txt)
    print(txt)
    print(f"plots: {bag/'allan_gyro.png'} , {bag/'bias_drift.png'}")


if __name__ == '__main__':
    main()
