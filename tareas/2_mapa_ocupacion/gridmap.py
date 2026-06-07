"""
TAREA 2 - Mapa de ocupacion probabilistico (occupancy grid).

Pipeline:
  1. Entorno 2D con obstaculos (rejilla "verdad de terreno").
  2. Un rover recorre el entorno (mismo modelo cinematico de la Tarea 1).
  3. En cada pose simula un sensor de rango tipo lidar (ray-casting + ruido).
  4. Actualiza la probabilidad de ocupacion de cada celda con el modelo
     inverso de sensor en LOG-ODDS (mapeo probabilistico visto en clase):

         l(c) <- l(c) + l_inv(c) - l_0          (l_0 = 0, prior = 0.5)
         p(c) = 1 - 1 / (1 + exp(l(c)))

     A lo largo de cada haz: celdas antes del impacto -> libres (l_free),
     celda del impacto -> ocupada (l_occ).
  5. Visualiza la evolucion del mapa y valida contra la verdad de terreno.

Ejecutar:  python3 gridmap.py
Genera las figuras en ./figs/
"""

import os
import sys

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "comun")))
import rover_model as rm    # noqa: E402  (modelo compartido con la Tarea 1)
import sensor_models as sm  # noqa: E402  (ruido REAL del MS200, sim-to-real)

FIGS = os.path.join(os.path.dirname(__file__), "figs")


# --------------------------------------------------------------------------- #
# 1) Entorno: rejilla de verdad de terreno (1 = ocupado, 0 = libre).
# --------------------------------------------------------------------------- #
class World:
    def __init__(self, w=6.0, h=6.0, res=0.05):
        self.w, self.h, self.res = w, h, res
        self.nx = int(round(w / res))
        self.ny = int(round(h / res))
        self.gt = np.zeros((self.ny, self.nx), dtype=np.uint8)
        self._build()

    def _rect(self, x0, y0, x1, y1):
        i0, j0 = self.world_to_grid(x0, y0)
        i1, j1 = self.world_to_grid(x1, y1)
        self.gt[min(i0, i1):max(i0, i1) + 1, min(j0, j1):max(j0, j1) + 1] = 1

    def _build(self):
        t = self.res * 2  # grosor de pared
        # Paredes perimetrales.
        self._rect(0, 0, self.w, t)
        self._rect(0, self.h - t, self.w, self.h)
        self._rect(0, 0, t, self.h)
        self._rect(self.w - t, 0, self.w, self.h)
        # Obstaculos internos.
        self._rect(1.5, 1.5, 2.2, 2.2)          # caja
        self._rect(3.8, 3.5, 4.6, 4.9)          # caja
        self._rect(3.6, 1.2, 3.8, 2.6)          # muro corto
        self._rect(1.2, 3.8, 2.8, 4.0)          # muro corto

    def world_to_grid(self, x, y):
        j = int(np.clip(x / self.res, 0, self.nx - 1))
        i = int(np.clip(y / self.res, 0, self.ny - 1))
        return i, j

    def in_bounds(self, i, j):
        return 0 <= i < self.ny and 0 <= j < self.nx


# --------------------------------------------------------------------------- #
# 2) Trayectoria del rover (usa el modelo cinematico de la Tarea 1).
# --------------------------------------------------------------------------- #
def rover_path(world, T=70.0, dt=0.1):
    """
    Recorre un circulo dentro del cuarto siguiendo la ley de control del paper.
    Devuelve un arreglo de poses [x, y, theta].
    """
    p = rm.RoverParams()
    Kp = np.diag([1.5, 1.5])
    cx, cy, R, omega = world.w / 2, world.h / 2, 1.4, 2 * np.pi / T

    xi = np.array([cx, cy - R, np.pi / 2])  # arranca sobre el circulo
    poses = []
    for k in range(int(T / dt)):
        t = k * dt
        qd = np.array([cx + R * np.sin(omega * t), cy - R * np.cos(omega * t)])
        qd_dot = np.array([R * omega * np.cos(omega * t), R * omega * np.sin(omega * t)])
        u = rm.controller(xi, qd, qd_dot, Kp, p)
        xi = xi + dt * rm.f_kinematics(xi, u, p)
        poses.append(xi.copy())
    return np.array(poses)


# --------------------------------------------------------------------------- #
# 3) Sensor de rango simulado (lidar) por ray-casting sobre la verdad.
# --------------------------------------------------------------------------- #
class RangeSensor:
    """LiDAR simulado con el RUIDO REAL del Orbbec MS200 (sensor_models.py)."""

    def __init__(self, n_beams=360, fov=2 * np.pi, max_range=sm.LIDAR_MAX_RANGE, seed=0):
        self.angles = np.linspace(-fov / 2, fov / 2, n_beams, endpoint=False)
        self.max_range = max_range
        self.rng = np.random.default_rng(seed)

    def scan(self, pose, world):
        """Ray-casting vectorizado (rango verdadero) + ruido caracterizado del MS200."""
        x, y, th = pose
        angs = th + self.angles                      # (B,)
        ca, sa = np.cos(angs), np.sin(angs)
        rr = np.arange(0.0, self.max_range, world.res * 0.5)  # (S,)
        X = x + np.outer(ca, rr)                      # (B, S)
        Y = y + np.outer(sa, rr)
        J = np.clip((X / world.res).astype(int), 0, world.nx - 1)
        I = np.clip((Y / world.res).astype(int), 0, world.ny - 1)
        oob = (X < 0) | (X >= world.w) | (Y < 0) | (Y >= world.h)
        blocked = (world.gt[I, J] == 1) | oob        # (B, S) celda que detiene el haz

        hit = blocked.any(axis=1)
        first = np.argmax(blocked, axis=1)           # primer indice bloqueado por haz
        r_true = np.where(hit, rr[first], self.max_range)
        B = self.angles.shape[0]

        # --- ruido REAL del MS200: sigma(rango) + dropout + espurios + cuantizacion ---
        sig = sm.lidar_sigma(r_true)
        noisy = r_true + self.rng.normal(0.0, 1.0, B) * sig
        noisy = np.round(noisy / sm.LIDAR_QUANT_M) * sm.LIDAR_QUANT_M
        # espurio: retorno FALSO mas CERCANO (una pared opaca no se ve "a traves")
        spurious = self.rng.uniform(sm.LIDAR_MIN_RANGE,
                                    np.maximum(r_true, sm.LIDAR_MIN_RANGE + 1e-3))
        u = self.rng.random(B)
        ranges = np.where(u < sm.LIDAR_DROPOUT, self.max_range,
                          np.where(u < sm.LIDAR_DROPOUT + sm.LIDAR_SPURIOUS,
                                   spurious, noisy))
        ranges[~hit] = self.max_range                # sin obstaculo -> sin retorno
        return np.clip(ranges, 0.0, self.max_range)


# --------------------------------------------------------------------------- #
# 4) Mapa de ocupacion en log-odds + modelo inverso de sensor.
# --------------------------------------------------------------------------- #
class OccupancyGrid:
    def __init__(self, world, p_occ=0.7, p_free=0.4, clamp=6.0):
        self.world = world
        self.L = np.zeros((world.ny, world.nx))     # log-odds (prior 0.5 -> 0)
        self.l_occ = np.log(p_occ / (1 - p_occ))
        self.l_free = np.log(p_free / (1 - p_free))
        self.clamp = clamp

    @staticmethod
    def _bresenham(i0, j0, i1, j1):
        """Celdas de la linea (i0,j0)->(i1,j1)."""
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

    def update(self, pose, ranges, sensor):
        w = self.world
        x, y, th = pose
        i0, j0 = w.world_to_grid(x, y)
        for a, r in zip(sensor.angles, ranges):
            if r >= sensor.max_range:
                continue          # haz sin retorno (dropout) -> no aporta informacion
            hit = r < sensor.max_range
            end_r = min(r, sensor.max_range)
            ex = x + end_r * np.cos(th + a)
            ey = y + end_r * np.sin(th + a)
            i1, j1 = w.world_to_grid(ex, ey)
            cells = self._bresenham(i0, j0, i1, j1)
            for (ci, cj) in cells[:-1]:           # camino libre
                self.L[ci, cj] += self.l_free
            ci, cj = cells[-1]                     # extremo del haz
            self.L[ci, cj] += self.l_occ if hit else self.l_free
        np.clip(self.L, -self.clamp, self.clamp, out=self.L)

    def prob(self):
        return 1.0 - 1.0 / (1.0 + np.exp(self.L))


# --------------------------------------------------------------------------- #
# 5) Visualizacion y validacion
# --------------------------------------------------------------------------- #
def plot_evolution(world, grid, sensor, poses, snapshots):
    os.makedirs(FIGS, exist_ok=True)
    fig, axes = plt.subplots(2, 3, figsize=(15, 10))
    axes = axes.ravel()
    extent = [0, world.w, 0, world.h]
    snap_idx = 0
    for k in range(len(poses)):
        grid.update(poses[k], sensor.scan(poses[k], world), sensor)
        if snap_idx < len(snapshots) and k == snapshots[snap_idx]:
            ax = axes[snap_idx]
            ax.imshow(grid.prob(), origin="lower", extent=extent,
                      cmap="Greys", vmin=0, vmax=1)
            ax.plot(poses[:k + 1, 0], poses[:k + 1, 1], "r-", lw=1)
            ax.plot(poses[k, 0], poses[k, 1], "ro", ms=5)
            ax.set_title(f"{k + 1} escaneos")
            ax.set_xlabel("x [m]")
            ax.set_ylabel("y [m]")
            snap_idx += 1
    fig.suptitle("Evolucion del mapa de ocupacion probabilistico "
                 "(negro = ocupado, blanco = libre, gris = desconocido)")
    fig.tight_layout()
    fig.savefig(os.path.join(FIGS, "10_evolucion_mapa.png"), dpi=120)
    plt.close(fig)
    print("Figura de evolucion guardada en", FIGS)


def validate(world, grid, occ_th=0.65, free_th=0.35):
    """Compara el mapa final con la verdad de terreno sobre celdas observadas."""
    prob = grid.prob()
    observed = grid.L != 0.0                     # celdas tocadas por algun haz
    pred_occ = prob >= occ_th
    pred_free = prob <= free_th
    gt_occ = world.gt == 1

    # Solo celdas observadas y decididas (no las que quedaron en ~0.5).
    decided = observed & (pred_occ | pred_free)
    correct = ((pred_occ & gt_occ) | (pred_free & ~gt_occ)) & decided
    acc = correct.sum() / max(1, decided.sum())

    tp = (pred_occ & gt_occ & observed).sum()
    fp = (pred_occ & ~gt_occ & observed).sum()
    fn = (pred_free & gt_occ & observed).sum()
    precision = tp / max(1, tp + fp)
    recall = tp / max(1, tp + fn)

    print("\n=== Validacion del mapa de ocupacion ===")
    print(f"  Celdas observadas        : {observed.sum()}")
    print(f"  Exactitud (celdas decididas): {acc * 100:.2f}%")
    print(f"  Precision (ocupado)      : {precision * 100:.2f}%")
    print(f"  Recall    (ocupado)      : {recall * 100:.2f}%")

    # Figura comparativa: verdad / mapa final / aciertos-errores.
    extent = [0, world.w, 0, world.h]
    fig, ax = plt.subplots(1, 3, figsize=(16, 5.5))
    ax[0].imshow(world.gt, origin="lower", extent=extent, cmap="Greys", vmin=0, vmax=1)
    ax[0].set_title("Verdad de terreno")

    ax[1].imshow(prob, origin="lower", extent=extent, cmap="Greys", vmin=0, vmax=1)
    ax[1].set_title("Mapa estimado (prob. ocupacion)")

    # Mapa de aciertos/errores.
    vis = np.full(prob.shape, 0.5)
    vis[correct] = 1.0                            # acierto -> verde
    vis[decided & ~correct] = 0.0                 # error  -> rojo
    rgb = np.zeros(prob.shape + (3,))
    rgb[..., 0] = (decided & ~correct)            # rojo = error
    rgb[..., 1] = correct                         # verde = acierto
    ax[2].imshow(rgb, origin="lower", extent=extent)
    ax[2].set_title(f"Aciertos (verde) / errores (rojo)\nexactitud = {acc*100:.1f}%")
    for a in ax:
        a.set_xlabel("x [m]")
        a.set_ylabel("y [m]")
    fig.tight_layout()
    fig.savefig(os.path.join(FIGS, "11_validacion_mapa.png"), dpi=120)
    plt.close(fig)
    print("Figura de validacion guardada en", FIGS)
    return acc, precision, recall


def main():
    world = World()
    poses = rover_path(world)
    sensor = RangeSensor(seed=7)
    grid = OccupancyGrid(world)

    n = len(poses)
    snapshots = [int(f * (n - 1)) for f in (0.05, 0.15, 0.30, 0.55, 0.80, 1.0)]
    plot_evolution(world, grid, sensor, poses, snapshots)
    validate(world, grid)


if __name__ == "__main__":
    main()
