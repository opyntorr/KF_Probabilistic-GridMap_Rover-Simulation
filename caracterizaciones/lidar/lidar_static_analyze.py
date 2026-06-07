#!/usr/bin/env python3
"""
lidar_static_analyze.py — caracteriza el LiDAR MS200 desde un rosbag estatico (escena quieta).
Robusto a outliers (alguien/algo que se mueva): usa mediana + MAD / sigma-clipping por rayo.

Sirve para:
  - SLAM mapping: max_laser_range confiable + limpieza (sabe hasta donde confiar).
  - AMCL / slam_toolbox localization: sigma_hit (de sigma_range), z_rand/z_max (espurios/dropout),
    laser_max_range. (Imprime sugerencias de params de AMCL.)

Mide: sigma_range por rayo, sigma(d), rango max confiable, tasa de dropout, tasa de espurios,
cuantizacion del rango, y drift termico/largo plazo.

Modo bias (opcional, captura corta contra pared plana a distancia MEDIDA con cinta):
    python3 lidar_static_analyze.py <bag> --known-dist 1.00   # compara mediana del frente vs 1.00 m

Corre en el venv:  source ~/evo_venv/bin/activate  (rosbags + numpy + matplotlib)
    python3 lidar_static_analyze.py ~/static_runs/<nombre>  [--topic /scan] [--sigma-max 0.05]
Salida en la carpeta del bag: lidar_summary.txt, sigma_vs_range.png, thermal_drift.png
"""
import argparse, sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore

# Los bags de ROS2 Humble no embeben las definiciones de tipos -> hay que darle un typestore.
TYPESTORE = get_typestore(Stores.ROS2_HUMBLE)


def read_scan(bagdir, topic):
    rows, ts, meta = [], [], None
    with AnyReader([Path(bagdir)], default_typestore=TYPESTORE) as reader:
        conns = [c for c in reader.connections if c.topic == topic]
        if not conns:
            sys.exit(f"No hay topic {topic} en el bag. Topics: {[c.topic for c in reader.connections]}")
        for conn, _t, raw in reader.messages(connections=conns):
            m = reader.deserialize(raw, conn.msgtype)
            if meta is None:
                meta = dict(angle_min=m.angle_min, angle_inc=m.angle_increment,
                            range_min=m.range_min, range_max=m.range_max, n=len(m.ranges))
            r = np.asarray(m.ranges, dtype=np.float64)
            if len(r) == meta['n']:
                rows.append(r); ts.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
    return np.asarray(ts), np.vstack(rows), meta


def robust_ray(col, rmin, rmax, k=4, iters=4):
    """Devuelve (mediana, sigma, frac_outlier_espurio, frac_dropout). col = serie temporal de un rayo."""
    n = len(col)
    valid = np.isfinite(col) & (col > rmin) & (col < rmax)
    drop = 1.0 - valid.mean()
    x = col[valid]
    if len(x) < max(10, 0.2 * n):
        return np.nan, np.nan, np.nan, drop
    keep = np.ones(len(x), bool)
    for _ in range(iters):
        med = np.median(x[keep])
        mad = np.median(np.abs(x[keep] - med)) * 1.4826
        if mad <= 0:
            break
        keep = np.abs(x - med) <= k * mad
    inl = x[keep]
    return np.median(inl), np.std(inl), 1.0 - keep.mean(), drop


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--topic', default='/scan')
    ap.add_argument('--sigma-max', type=float, default=0.05, help='umbral de sigma para "rango confiable" (m)')
    ap.add_argument('--known-dist', type=float, default=None, help='modo bias: distancia real a la pared frontal (m)')
    args = ap.parse_args()
    bag = Path(args.bag).expanduser()

    ts, R, meta = read_scan(bag, args.topic)
    ns, nr = R.shape
    dur = ts[-1] - ts[0]
    rmin, rmax = meta['range_min'], meta['range_max']
    angs = (meta['angle_min'] + np.arange(nr) * meta['angle_inc'])
    L = []
    L.append(f"LiDAR {args.topic}: {ns} scans, {dur/60:.1f} min, {nr} rayos, range {rmin:.2f}-{rmax:.2f} m")

    med = np.full(nr, np.nan); sig = np.full(nr, np.nan)
    out = np.full(nr, np.nan); drop = np.full(nr, np.nan)
    for j in range(nr):
        med[j], sig[j], out[j], drop[j] = robust_ray(R[:, j], rmin, rmax)

    good = np.isfinite(sig)
    # --- sigma(d): binnear por rango mediano ---
    L.append("")
    L.append("=== sigma_range por distancia ===")
    edges = np.arange(0, np.ceil(rmax) + 0.5, 0.5)
    for lo, hi in zip(edges[:-1], edges[1:]):
        sel = good & (med >= lo) & (med < hi)
        if sel.sum() >= 3:
            sm = np.median(sig[sel])
            L.append(f"  {lo:.1f}-{hi:.1f} m: sigma~{sm*1000:.1f} mm  (n_rayos={int(sel.sum())})")
    # rango max confiable: ultimo bin con sigma < sigma_max
    reliable = rmax
    for lo, hi in zip(edges[:-1], edges[1:]):
        sel = good & (med >= lo) & (med < hi)
        if sel.sum() >= 3 and np.median(sig[sel]) > args.sigma_max:
            reliable = lo; break
    L.append(f"  -> rango max confiable (sigma<{args.sigma_max*1000:.0f}mm): ~{reliable:.1f} m")

    # --- dropout / espurios / cuantizacion ---
    drop_overall = np.nanmean(drop)
    out_overall = np.nanmean(out[good])
    # cuantizacion: paso minimo entre valores de rango distintos en el rayo mas estable
    jbest = np.nanargmin(np.where(good, sig, np.inf))
    col = R[:, jbest]; col = col[np.isfinite(col) & (col > rmin)]
    uq = np.unique(np.round(col, 5))
    quant = np.median(np.diff(uq)) if len(uq) > 3 else np.nan
    L.append("")
    L.append(f"sigma_range tipico (rayos buenos): mediana {np.nanmedian(sig[good])*1000:.1f} mm")
    L.append(f"tasa de DROPOUT (rayos sin retorno, escena estatica): {100*drop_overall:.1f}%")
    L.append(f"tasa de ESPURIOS (retornos validos saltados/outliers): {100*out_overall:.2f}%")
    L.append(f"cuantizacion del rango (paso minimo): ~{quant*1000:.1f} mm")

    # --- bias (modo opcional) ---
    if args.known_dist is not None:
        front = good & (np.abs(angs) < np.radians(5))  # +-5 deg alrededor de 0 (frente)
        if front.sum():
            meas = np.median(med[front])
            L.append("")
            L.append(f"=== BIAS (frente, {front.sum()} rayos) ===")
            L.append(f"  real={args.known_dist:.3f} m  medido={meas:.3f} m  bias={meas-args.known_dist:+.3f} m "
                     f"({100*(meas-args.known_dist)/args.known_dist:+.1f}%)")

    # --- sugerencias AMCL ---
    sig_typ = np.nanmedian(sig[good])
    L.append("")
    L.append("=== SUGERENCIAS para AMCL (sensor model) ===")
    L.append(f"  sigma_hit ~ {max(sig_typ, (quant if np.isfinite(quant) else 0)):.3f} m "
             f"(>= sigma_range y >= cuanto; a menudo se sube 2-3x para robustez)")
    L.append(f"  laser_max_range / laser_likelihood_max_dist <= {reliable:.1f} m")
    z_rand = min(0.1, max(0.01, out_overall * 5))
    L.append(f"  z_rand ~ {z_rand:.2f} (de la tasa de espurios)  | z_hit ~ {1-z_rand:.2f}")
    L.append(f"  z_max ~ acorde a dropout {100*drop_overall:.1f}% (rayos sin retorno)")
    L.append("  NOTA: el modelo de MOVIMIENTO de AMCL (alpha1..5) NO se caracteriza quieto -> con manejo + OptiTrack.")

    # --- plots ---
    fig, ax = plt.subplots(figsize=(7, 4))
    ax.scatter(med[good], sig[good]*1000, s=6, alpha=0.4)
    ax.axhline(args.sigma_max*1000, color='r', ls='--', label=f'umbral {args.sigma_max*1000:.0f}mm')
    ax.set_xlabel('rango (m)'); ax.set_ylabel('sigma (mm)'); ax.set_title('sigma_range vs distancia')
    ax.grid(True, alpha=0.3); ax.legend(); fig.tight_layout()
    fig.savefig(bag / 'sigma_vs_range.png', dpi=110); plt.close(fig)

    # drift termico: rayo mas estable, mediana por ventana
    fig2, ax2 = plt.subplots(figsize=(7, 4))
    rate = ns / dur if dur > 0 else 15.0
    win = max(int(60 * rate), 30)
    colj = R[:, jbest].astype(float); colj[~(np.isfinite(colj) & (colj > rmin))] = np.nan
    nb = ns // win
    if nb >= 2:
        seg = colj[:nb*win].reshape(nb, win)
        mw = np.nanmedian(seg, axis=1) * 1000
        tw = (np.arange(nb) * win / rate) / 60.0
        ax2.plot(tw, mw - np.nanmedian(mw))
        L.append("")
        L.append(f"drift termico (rayo estable @ {np.nanmedian(colj):.2f}m): rango {np.nanmax(mw)-np.nanmin(mw):.1f} mm en la captura")
    ax2.set_xlabel('tiempo (min)'); ax2.set_ylabel('desviacion de la mediana (mm)')
    ax2.set_title('Drift termico/largo plazo del rango'); ax2.grid(True, alpha=0.3)
    fig2.tight_layout(); fig2.savefig(bag / 'thermal_drift.png', dpi=110); plt.close(fig2)

    txt = "\n".join(L) + "\n"
    (bag / 'lidar_summary.txt').write_text(txt)
    print(txt)
    print(f"plots: {bag/'sigma_vs_range.png'} , {bag/'thermal_drift.png'}")


if __name__ == '__main__':
    main()
