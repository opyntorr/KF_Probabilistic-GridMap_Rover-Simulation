#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
serpenteo_sweep.py — arnés AUTOMÁTICO para ELIMINAR el "serpenteo" del mecanum
(~1.3 cm de tejido circular) en la escena POR FÍSICA (scene_mecanum.py).

Sesiones previas demostraron que el serpenteo es INVARIANTE a la fricción μ, al
número de rodillos y al ajuste del controlador. Este arnés barre las perillas de
ALTO VALOR todavía SIN PROBAR, en este ORDEN de prioridad:

  (1) iteraciones del solver de la articulación: posición 4→16→32→64, velocidad 1→4→8
  (2) TimeStepsPerSecond de la física 120→240→480 (+ substeps efectivos)
  (3) contact_offset≈0.001 y rest_offset≈0.0005 en los colliders de rueda/rodillo
  (4) maxDepenetrationVelocity≈5
  (5) geometría del rodillo  esfera → cilindro/cápsula (rodillos de barril, eje = eje
      del joint a 45°)  -> regenera el URDF (regen_wheel_urdf)
  (6) aplicar la acción de la articulación en un callback POR PASO DE FÍSICA vs a tasa
      de render

CÓMO FUNCIONA
-------------
Para cada `config` (dict), el arnés:
  A. (si toca) regenera assets/jetauto_mecanum.urdf con la geometría de rodillo pedida.
  B. lanza scene_mecanum.py como SUBPROCESO ($ISAACSIM/python.sh) pasándole los NUEVOS
     flags de física que el orquestador debe haber añadido a scene_mecanum.py
     (--solver-pos, --solver-vel, --phys-hz, --contact-offset, --rest-offset,
      --maxdepen, --roller-shape, --apply-perstep) + los flags que YA existen
     (--world '' / 'laberinto', --headless, --no-extra).
  C. lanza kf_control_isaac.py (que YA corre el círculo y guarda figs/traj.npy) como
     un segundo subproceso ROS2, esperando a que termine la rutina (duration_s).
  D. lee figs/traj.npy, calcula el RESIDUAL DE AJUSTE DE CÍRCULO (std de |p-c|-R sobre
     la trayectoria del PUNTO de control en GROUND TRUTH, columnas 1:3) y el RTF.
  E. añade {config, residual_std_mm, p2p_mm, rtf, ...} a isaac/serpenteo_results.csv.

Al final elige el MEJOR: residual ≤ ~4 mm; desempate por mayor RTF y luego "más
sim-to-real" (menos iteraciones / menor Hz / esfera antes que cápsula).

NO LANZA Isaac por sí mismo al importarse: TODO el trabajo está dentro de funciones.
El orquestador (que corre en la GPU) llama a run_sweep(). Este archivo solo valida con
`python3 -m py_compile`.

Uso (en la máquina con GPU, NO aquí):
    source isaac/isaac_env.sh
    python3 isaac/serpenteo_sweep.py                 # barrido completo
    python3 isaac/serpenteo_sweep.py --quick         # subconjunto rápido
    python3 isaac/serpenteo_sweep.py --regen capsule # solo regenerar el URDF
"""

import argparse
import csv
import datetime
import os
import re
import shutil
import signal
import subprocess
import sys
import tempfile
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PROJECT = os.path.dirname(HERE)                       # .../agv_uav_project_jetauto_Vilchis
SCENE = os.path.join(HERE, "scene_mecanum.py")
KF = os.path.join(HERE, "kf_control_isaac.py")
URDF_OUT = os.path.join(HERE, "assets", "jetauto_mecanum.urdf")
TRAJ_NPY = os.path.join(HERE, "figs", "traj.npy")
RESULTS_CSV = os.path.join(HERE, "serpenteo_results.csv")

# Pipeline de regen del URDF (igual al README "Re-exportar el URDF con rodillos"):
#   xacro src/mi_proyecto_sim/urdf/jetauto/jetauto_sim.urdf.xacro use_aruco:=false -o ...
GAZEBO_SRC = os.path.join(os.path.dirname(PROJECT), "agv_uav_project_jetauto", "src", "mi_proyecto_sim")
SIM_XACRO = os.path.join(GAZEBO_SRC, "urdf", "jetauto", "jetauto_sim.urdf.xacro")
WHEEL_XACRO = os.path.join(GAZEBO_SRC, "urdf", "jetauto", "jetauto_mecanum_wheel.urdf.xacro")
AMENT_OVERLAY = os.path.join(HERE, ".ament_overlay")

# Geometría del rodillo (del macro jetauto_roller). En el xacro el rodillo es una
# esfera de radio roller_radius=0.0096 sobre el aro, eje del joint = (sin α, cos α, 0)
# con α=±45°. Para "barril" usamos un cilindro/cápsula CO-AXIAL al eje del joint:
# en el frame del link el eje del joint es local +X tras girar el visual; pero como
# el <axis> del joint vive en el frame del PADRE (la rueda), la forma de colisión del
# rodillo debe orientarse para que su eje largo quede a lo largo del eje de rodadura.
# El rodillo gira sobre su propio eje del joint -> su eje de simetría = eje de rodadura
# = (sin α, cos α, 0) en el frame de la rueda. En el frame LOCAL del link de rodillo
# ese eje es +X (URDF: cylinder/capsule por defecto a lo largo de +Z, así que rpy gira
# Z->X = (0, +pi/2, 0)). Longitud del barril ≈ 0.018 m (cabe entre rodillos vecinos).
ROLLER_RADIUS = 0.0096
ROLLER_LENGTH = 0.018


# ---------------------------------------------------------------------------
# 1) REGENERACIÓN PARAMÉTRICA DE LA GEOMETRÍA DEL RODILLO (xacro -> URDF)
# ---------------------------------------------------------------------------
def _patched_wheel_xacro(shape):
    """Devuelve el texto del jetauto_mecanum_wheel.urdf.xacro con la geometría de
    colisión del rodillo cambiada a `shape` ∈ {sphere, cylinder, capsule}.

    Solo toca el bloque <collision> del macro jetauto_roller. La esfera es el original
    (no toca nada). Cilindro/cápsula = "barril": eje de simetría a lo largo del eje del
    joint (eje de rodadura), conseguido con rpy='0 ${pi/2} 0' (gira +Z local -> +X local).
    """
    with open(WHEEL_XACRO, "r") as f:
        txt = f.read()
    if shape == "sphere":
        return txt

    if shape == "cylinder":
        geo = '<cylinder radius="${roller_radius}" length="%s"/>' % ROLLER_LENGTH
    elif shape == "capsule":
        # URDF estándar no tiene <capsule>; el importador de URDF de Isaac SÍ lo acepta
        # (sdf/urdf extendido). Si fallara, el orquestador puede caer a cylinder.
        geo = '<capsule radius="${roller_radius}" length="%s"/>' % ROLLER_LENGTH
    else:
        raise ValueError("shape debe ser sphere|cylinder|capsule, no %r" % shape)

    # El bloque original (idéntico al del archivo leído):
    #   <origin xyz="0 0 0" rpy="0 0 0"/>
    #   <geometry>
    #     <sphere radius="${roller_radius}"/>
    #   </geometry>
    old = re.compile(
        r'(<collision>\s*)'
        r'<origin xyz="0 0 0" rpy="0 0 0"/>\s*'
        r'<geometry>\s*'
        r'<sphere radius="\$\{roller_radius\}"/>\s*'
        r'</geometry>',
        re.MULTILINE)
    # Orientar el barril a lo largo del EJE DE GIRO del rodillo (joint axis = (sinα,cosα,0)
    # en el frame del link). Cilindro/cápsula por defecto van en +Z; mapear +Z->(sinα,cosα,0)
    # = Ry(pi/2) [+Z->+X] luego Rz(pi/2 - alpha) [+X->(sinα,cosα,0)]. rpy=(0, pi/2, pi/2-alpha).
    new = (r'\g<1>'
           '<origin xyz="0 0 0" rpy="0 ${pi/2} ${pi/2 - alpha}"/>'
           '<geometry>' + geo + '</geometry>')
    patched, n = old.subn(new, txt)
    if n != 1:
        raise RuntimeError(
            "no pude parchear el bloque <collision> del rodillo (encontré %d, esperaba 1). "
            "¿Cambió jetauto_mecanum_wheel.urdf.xacro?" % n)
    return patched


def regen_wheel_urdf(shape="sphere", urdf_out=URDF_OUT, verbose=True):
    """Regenera assets/jetauto_mecanum.urdf con rodillos `shape` ∈ {sphere,cylinder,capsule}.

    Replica el pipeline del README (overlay de ament -> xacro -> sed a ruta absoluta),
    pero copiando ANTES el wheel xacro parcheado a /tmp y haciendo que el sim xacro lo
    incluya desde ahí. Devuelve la ruta del URDF generado.

    Estrategia (mínimamente invasiva, no edita los fuentes de Gazebo):
      1. crea un dir temporal espejo con jetauto_sim.urdf.xacro + los includes,
         sustituyendo SOLO jetauto_mecanum_wheel.urdf.xacro por la versión parcheada;
      2. crea el overlay de ament para que $(find mi_proyecto_sim) resuelva al SRC real
         (mallas, etc.), pero apuntando el include del wheel al temporal;
      3. corre xacro y reescribe las rutas file:// del overlay al SRC absoluto (sed).
    """
    shape = shape.lower()
    if shape not in ("sphere", "cylinder", "capsule"):
        raise ValueError("shape debe ser sphere|cylinder|capsule")

    tmpdir = tempfile.mkdtemp(prefix="jetauto_roller_%s_" % shape)
    # 1) escribe el wheel xacro parcheado en el temporal
    patched_wheel = os.path.join(tmpdir, "jetauto_mecanum_wheel.urdf.xacro")
    with open(patched_wheel, "w") as f:
        f.write(_patched_wheel_xacro(shape))

    # copia el sim xacro al temporal y reapunta SOLO el include del wheel al parcheado.
    # Los demás includes ($(find mi_proyecto_sim)/...) los resuelve el overlay de ament.
    with open(SIM_XACRO, "r") as f:
        sim_txt = f.read()
    sim_txt = sim_txt.replace(
        '<xacro:include filename="$(find mi_proyecto_sim)/urdf/jetauto/jetauto_mecanum_wheel.urdf.xacro"/>',
        '<xacro:include filename="%s"/>' % patched_wheel)
    tmp_sim = os.path.join(tmpdir, "jetauto_sim.urdf.xacro")
    with open(tmp_sim, "w") as f:
        f.write(sim_txt)

    # 2) overlay de ament para $(find mi_proyecto_sim) (mallas/meshes reales)
    os.makedirs(os.path.join(AMENT_OVERLAY, "share", "ament_index",
                             "resource_index", "packages"), exist_ok=True)
    open(os.path.join(AMENT_OVERLAY, "share", "ament_index",
                      "resource_index", "packages", "mi_proyecto_sim"), "w").close()
    link = os.path.join(AMENT_OVERLAY, "share", "mi_proyecto_sim")
    if os.path.islink(link) or os.path.exists(link):
        try:
            os.remove(link)
        except OSError:
            pass
    os.symlink(GAZEBO_SRC, link)

    # 3) xacro (en una shell que primero hace source de ROS2 humble + el overlay)
    env_cmd = (
        'set -e; source /opt/ros/humble/setup.bash; '
        'export AMENT_PREFIX_PATH="%s:$AMENT_PREFIX_PATH"; '
        'xacro "%s" use_aruco:=false -o "%s"; '
        # reescribe las rutas del overlay al SRC absoluto (igual que el README)
        'sed -i "s#%s/share/mi_proyecto_sim#%s#g" "%s"'
    ) % (AMENT_OVERLAY, tmp_sim, urdf_out, AMENT_OVERLAY, GAZEBO_SRC, urdf_out)
    if verbose:
        print("[regen] xacro -> %s (rodillo=%s)" % (urdf_out, shape), flush=True)
    subprocess.run(["bash", "-lc", env_cmd], check=True)

    # verificación rápida: 48 colliders de rodillo de la forma pedida
    with open(urdf_out, "r") as f:
        body = f.read()
    tag = {"sphere": "<sphere", "cylinder": "<cylinder", "capsule": "<capsule"}[shape]
    n = body.count(tag)
    if verbose:
        print("[regen] %s: %d colliders '%s' en %s" %
              (shape, n, tag, os.path.basename(urdf_out)), flush=True)
    shutil.rmtree(tmpdir, ignore_errors=True)
    return urdf_out


# ---------------------------------------------------------------------------
# 2) MÉTRICA: ajuste de círculo (Kåsa) + residual sobre la trayectoria GT del punto
# ---------------------------------------------------------------------------
def circle_fit_residual(traj_npy=TRAJ_NPY, settle_frac=0.15):
    """Lee figs/traj.npy y devuelve (residual_std_m, p2p_m, R_m, n_used).

    traj.npy (de kf_control_isaac.save_plots) tiene 15 columnas:
        [t, gp_x, gp_y, gt_yaw, op_x, op_y, odo_yaw, qp_x, qp_y, kf_yaw, qd_x, qd_y, P0,P1,P2]
    El GROUND TRUTH del PUNTO de control (lo que teje el serpenteo) es columnas 1:3.
    Descarta el primer `settle_frac` (transitorio de enganche del lazo) y ajusta un
    círculo algebraico (Kåsa); el residual = |p-c| - R. std(residual) ≈ amplitud del
    serpenteo (el baseline da ~16 mm con transitorio, ~13 mm en régimen).
    """
    d = np.load(traj_npy)
    if d.ndim != 2 or d.shape[1] < 3 or len(d) < 50:
        raise RuntimeError("traj.npy con forma inesperada: %r" % (d.shape,))
    i0 = int(len(d) * settle_frac)
    g = d[i0:, 1:3].astype(float)            # GT del punto de control (x, y)
    x, y = g[:, 0], g[:, 1]
    # ajuste algebraico de círculo (Kåsa): min ||A·[2cx,2cy,c2] - (x^2+y^2)||
    A = np.c_[2.0 * x, 2.0 * y, np.ones(len(x))]
    b = x * x + y * y
    sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    cx, cy, c2 = sol
    R = float(np.sqrt(max(c2 + cx * cx + cy * cy, 0.0)))
    r = np.sqrt((x - cx) ** 2 + (y - cy) ** 2)
    resid = r - R
    return float(resid.std()), float(resid.max() - resid.min()), R, len(g)


def _rtf_from_traj(traj_npy, wall_secs):
    """RTF = tiempo_sim_cubierto / tiempo_de_pared del proceso del controlador.
    El tiempo de sim cubierto = último t de traj.npy (usa /clock = reloj de Isaac)."""
    try:
        d = np.load(traj_npy)
        sim_secs = float(d[-1, 0] - d[0, 0])
    except Exception:
        return float("nan")
    if wall_secs <= 0:
        return float("nan")
    return sim_secs / wall_secs


# ---------------------------------------------------------------------------
# 3) LANZAMIENTO de un config: scene_mecanum.py (subproc) + kf_control (subproc)
# ---------------------------------------------------------------------------
def _scene_cmd(cfg, isaacsim, extra_scene_args):
    """Construye el argv para $ISAACSIM/python.sh scene_mecanum.py con los flags del cfg.

    NOTA: estos flags NUEVOS deben existir en scene_mecanum.py (los añade el orquestador;
    ver `how_to_integrate`). Si scene_mecanum.py aún no los conoce, usa parse_known_args
    así que los IGNORA sin romper, pero entonces el barrido no surte efecto -> por eso
    se reportan exactamente abajo.
    """
    pysh = os.path.join(isaacsim, "python.sh")
    cmd = [pysh, SCENE, "--headless", "--no-extra", "--world", ""]
    cmd += ["--solver-pos", str(cfg["solver_pos"])]
    cmd += ["--solver-vel", str(cfg["solver_vel"])]
    cmd += ["--phys-hz", str(cfg["phys_hz"])]
    cmd += ["--contact-offset", str(cfg["contact_offset"])]
    cmd += ["--rest-offset", str(cfg["rest_offset"])]
    cmd += ["--maxdepen", str(cfg["maxdepen"])]
    cmd += ["--roller-shape", str(cfg["roller_shape"])]
    if cfg.get("apply_perstep"):
        cmd += ["--apply-perstep"]
    cmd += list(extra_scene_args)
    return cmd


def _kf_cmd(duration_s):
    """argv para correr el círculo (kf_control_isaac) bajo el python del sistema con ROS2.
    Pasa duration_s + output_dir por parámetros ROS2; figs/traj.npy se sobrescribe."""
    return [
        "bash", "-lc",
        ('source /opt/ros/humble/setup.bash; source "%s"; '
         'exec python3 "%s" --ros-args -p duration_s:=%g -p output_dir:=%s'
         ) % (os.path.join(HERE, "isaac_env.sh"), KF, duration_s, os.path.join(HERE, "figs"))
    ]


def run_one(cfg, isaacsim, duration_s=90.0, scene_warmup_s=45.0,
            extra_scene_args=(), verbose=True):
    """Corre UN config de principio a fin y devuelve un dict de resultados.

    Pasos: (regen URDF si la forma cambió) -> lanza la escena -> espera warmup ->
    lanza el controlador (círculo) y espera a que termine -> mide residual + RTF ->
    mata la escena. Robusto a fallos: captura excepciones y las reporta en 'error'.
    """
    res = dict(cfg)
    res["error"] = ""
    scene_proc = None
    kf_proc = None
    t_wall0 = None
    try:
        # A) geometría del rodillo: regen del URDF solo si cambia respecto al actual
        if cfg.get("roller_shape", "sphere") != "sphere" or cfg.get("force_regen"):
            regen_wheel_urdf(cfg["roller_shape"], verbose=verbose)

        # borra traj.npy viejo para no leer resultados rancios si el control falla
        if os.path.exists(TRAJ_NPY):
            os.remove(TRAJ_NPY)

        # B) lanza la escena (subproceso, su propia sesión para poder matar el grupo)
        scmd = _scene_cmd(cfg, isaacsim, extra_scene_args)
        if verbose:
            print("[run] escena: %s" % " ".join(scmd), flush=True)
        scene_proc = subprocess.Popen(scmd, cwd=PROJECT, start_new_session=True)

        # C) espera a que la escena publique /clock y /odom (warmup fijo; el orquestador
        #    puede subirlo). Si la escena muere antes, aborta este config.
        t_end = time.time() + scene_warmup_s
        while time.time() < t_end:
            if scene_proc.poll() is not None:
                raise RuntimeError("scene_mecanum.py terminó durante el warmup (rc=%s)"
                                   % scene_proc.returncode)
            time.sleep(1.0)

        # D) corre el círculo y espera a que escriba figs/traj.npy
        kcmd = _kf_cmd(duration_s)
        if verbose:
            print("[run] controlador (círculo) %.0fs ..." % duration_s, flush=True)
        t_wall0 = time.time()
        kf_proc = subprocess.Popen(kcmd, cwd=PROJECT, start_new_session=True)
        # margen: la rutina dura duration_s; damos +60s de cortesía (arranque ROS2)
        kf_proc.wait(timeout=duration_s + 60.0)
        wall_secs = time.time() - t_wall0

        # E) métricas
        if not os.path.exists(TRAJ_NPY):
            raise RuntimeError("el controlador no generó figs/traj.npy")
        rstd, p2p, R, n = circle_fit_residual(TRAJ_NPY)
        res["residual_std"] = rstd
        res["residual_std_mm"] = rstd * 1000.0
        res["p2p"] = p2p
        res["p2p_mm"] = p2p * 1000.0
        res["fit_R"] = R
        res["n_used"] = n
        res["rtf"] = _rtf_from_traj(TRAJ_NPY, wall_secs)
        if verbose:
            print("[run] residual_std=%.2f mm  p2p=%.2f mm  R=%.3f  RTF=%.3f" %
                  (res["residual_std_mm"], res["p2p_mm"], R, res["rtf"]), flush=True)
    except subprocess.TimeoutExpired:
        res["error"] = "timeout del controlador"
    except Exception as e:  # noqa: BLE001 — robusto: un config malo no tumba el barrido
        res["error"] = "%s: %s" % (type(e).__name__, e)
    finally:
        for p in (kf_proc, scene_proc):
            _kill_group(p)
    # rellena claves de métricas si hubo error (CSV homogéneo)
    for k in ("residual_std", "residual_std_mm", "p2p", "p2p_mm", "fit_R", "n_used", "rtf"):
        res.setdefault(k, float("nan"))
    return res


def _kill_group(proc):
    """Mata el proceso y su grupo (Isaac arranca hijos); tolerante a None/ya-muerto."""
    if proc is None:
        return
    try:
        if proc.poll() is None:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            try:
                proc.wait(timeout=20.0)
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                proc.wait(timeout=10.0)
    except (ProcessLookupError, OSError):
        pass


# ---------------------------------------------------------------------------
# 4) DEFINICIÓN DEL BARRIDO (en el orden de prioridad de la tarea)
# ---------------------------------------------------------------------------
def _base_cfg():
    """Config base = scene_mecanum.py actual (sphere, 120 Hz, solver por defecto).
    El default del solver de articulación en Isaac suele ser pos=4/vel=1; lo fijamos
    explícito para que el baseline sea reproducible y comparable."""
    return dict(
        solver_pos=4, solver_vel=1, phys_hz=120,
        contact_offset=-1.0,   # <0 = "no tocar el offset por defecto del importador"
        rest_offset=-1.0,
        maxdepen=-1.0,         # <0 = no tocar
        roller_shape="sphere",
        apply_perstep=False,
        force_regen=False,
        tag="baseline",
    )


def build_configs(quick=False):
    """Lista ORDENADA de configs. Cada etapa parte del MEJOR conocido a priori (greedy
    coordinate descent): primero solver, luego Hz, offsets, depen, geometría, per-step.
    El orquestador puede reordenar/recortar; aquí damos un barrido completo y sensato.
    """
    cfgs = []

    def add(tag, **kw):
        c = _base_cfg()
        c.update(kw)
        c["tag"] = tag
        cfgs.append(c)

    # 0) baseline
    add("baseline")

    # (1) iteraciones del solver de posición (4→16→32→64) y velocidad (1→4→8)
    add("solverpos16", solver_pos=16)
    add("solverpos32", solver_pos=32)
    add("solverpos64", solver_pos=64)
    add("solvervel4", solver_pos=32, solver_vel=4)
    add("solvervel8", solver_pos=32, solver_vel=8)
    # mejor solver supuesto a priori para encadenar las siguientes etapas:
    best_solver = dict(solver_pos=32, solver_vel=4)

    # (2) TimeStepsPerSecond 120→240→480
    add("phys240", phys_hz=240, **best_solver)
    add("phys480", phys_hz=480, **best_solver)
    best_phys = dict(phys_hz=240, **best_solver)

    # (3) contact_offset≈0.001 & rest_offset≈0.0005 en colliders de rueda/rodillo
    add("offsets", contact_offset=0.001, rest_offset=0.0005, **best_phys)
    best_off = dict(contact_offset=0.001, rest_offset=0.0005, **best_phys)

    # (4) maxDepenetrationVelocity≈5
    add("maxdepen5", maxdepen=5.0, **best_off)
    best_dep = dict(maxdepen=5.0, **best_off)

    # (5) geometría del rodillo: cilindro y cápsula (barril)
    add("cylinder", roller_shape="cylinder", **best_dep)
    add("capsule", roller_shape="capsule", **best_dep)

    # (6) aplicar la acción por paso de física (vs render-rate)
    add("perstep", apply_perstep=True, **best_dep)

    if quick:
        keep = {"baseline", "solverpos32", "phys240", "offsets", "cylinder"}
        cfgs = [c for c in cfgs if c["tag"] in keep]
    return cfgs


# ---------------------------------------------------------------------------
# 5) CSV + selección del mejor
# ---------------------------------------------------------------------------
_CSV_FIELDS = [
    "ts", "tag", "solver_pos", "solver_vel", "phys_hz",
    "contact_offset", "rest_offset", "maxdepen", "roller_shape", "apply_perstep",
    "residual_std_mm", "p2p_mm", "fit_R", "n_used", "rtf", "error",
]


def append_result(res, csv_path=RESULTS_CSV):
    """Añade una fila a isaac/serpenteo_results.csv (crea cabecera si no existe)."""
    new = not os.path.exists(csv_path)
    row = {k: res.get(k, "") for k in _CSV_FIELDS}
    row["ts"] = datetime.datetime.now().isoformat(timespec="seconds")
    with open(csv_path, "a", newline="") as f:
        w = csv.DictWriter(f, fieldnames=_CSV_FIELDS)
        if new:
            w.writeheader()
        w.writerow(row)


def _simtoreal_score(res):
    """Penalización "menos sim-to-real": preferimos menos iteraciones, menor Hz y
    esfera (lo más fiel/barato). Menor = mejor (desempate tras RTF)."""
    shape_rank = {"sphere": 0, "cylinder": 1, "capsule": 1}.get(res.get("roller_shape", "sphere"), 2)
    return (res.get("solver_pos", 999) + res.get("solver_vel", 999)
            + res.get("phys_hz", 9999) / 120.0
            + 10.0 * shape_rank
            + (5.0 if res.get("apply_perstep") else 0.0))


def pick_best(results, residual_target_mm=4.0):
    """Elige el mejor: residual ≤ target (mm); desempate por mayor RTF y luego más
    sim-to-real. Si NINGUNO cumple el target, devuelve el de menor residual."""
    ok = [r for r in results
          if not r.get("error") and np.isfinite(r.get("residual_std_mm", np.nan))]
    if not ok:
        return None
    feas = [r for r in ok if r["residual_std_mm"] <= residual_target_mm]
    pool = feas if feas else ok
    if feas:
        # cumplen target -> max RTF, luego más sim-to-real
        pool.sort(key=lambda r: (-(r.get("rtf") or 0.0), _simtoreal_score(r)))
    else:
        # ninguno cumple -> el de menor residual (desempate por RTF)
        pool.sort(key=lambda r: (r["residual_std_mm"], -(r.get("rtf") or 0.0)))
    return pool[0]


# ---------------------------------------------------------------------------
# 6) ORQUESTACIÓN
# ---------------------------------------------------------------------------
def run_sweep(isaacsim=None, quick=False, duration_s=90.0, scene_warmup_s=45.0,
              extra_scene_args=(), residual_target_mm=4.0, verbose=True):
    """Corre el barrido completo (un config a la vez; la GPU está serializada).
    Devuelve (best_cfg, all_results). Cada resultado se va guardando en el CSV."""
    isaacsim = isaacsim or os.environ.get("ISAACSIM", "/home/opyntorr/isaacsim")
    cfgs = build_configs(quick=quick)
    if verbose:
        print("[sweep] %d configs -> %s" % (len(cfgs), RESULTS_CSV), flush=True)
    results = []
    for i, cfg in enumerate(cfgs):
        if verbose:
            print("\n[sweep] === %d/%d  tag=%s ===" % (i + 1, len(cfgs), cfg["tag"]), flush=True)
        r = run_one(cfg, isaacsim, duration_s=duration_s,
                    scene_warmup_s=scene_warmup_s, extra_scene_args=extra_scene_args,
                    verbose=verbose)
        append_result(r)
        results.append(r)
    best = pick_best(results, residual_target_mm=residual_target_mm)
    if verbose:
        if best is None:
            print("\n[sweep] sin resultados válidos.", flush=True)
        else:
            print("\n[sweep] MEJOR: tag=%s residual=%.2f mm RTF=%.3f shape=%s "
                  "solver=%d/%d hz=%d offsets=(%g,%g) depen=%g perstep=%s" % (
                      best["tag"], best["residual_std_mm"], best.get("rtf", float("nan")),
                      best["roller_shape"], best["solver_pos"], best["solver_vel"],
                      best["phys_hz"], best["contact_offset"], best["rest_offset"],
                      best["maxdepen"], best["apply_perstep"]), flush=True)
    return best, results


def main(argv=None):
    ap = argparse.ArgumentParser(description="Barrido para eliminar el serpenteo del mecanum.")
    ap.add_argument("--quick", action="store_true", help="subconjunto rápido de configs")
    ap.add_argument("--duration", type=float, default=90.0, help="segundos del círculo por config")
    ap.add_argument("--warmup", type=float, default=45.0, help="segundos de arranque de la escena")
    ap.add_argument("--target", type=float, default=4.0, help="residual objetivo (mm)")
    ap.add_argument("--isaacsim", default=os.environ.get("ISAACSIM", "/home/opyntorr/isaacsim"))
    ap.add_argument("--regen", choices=["sphere", "cylinder", "capsule"], default=None,
                    help="solo regenerar el URDF con esta geometría de rodillo y salir")
    ap.add_argument("--scene-arg", action="append", default=[],
                    help="flag extra para scene_mecanum.py (repetible), p.ej. --scene-arg --gridmap")
    args = ap.parse_args(argv)

    if args.regen:
        regen_wheel_urdf(args.regen)
        return 0

    run_sweep(isaacsim=args.isaacsim, quick=args.quick, duration_s=args.duration,
              scene_warmup_s=args.warmup, extra_scene_args=tuple(args.scene_arg),
              residual_target_mm=args.target)
    return 0


if __name__ == "__main__":
    sys.exit(main())
