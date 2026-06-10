#!/usr/bin/env python3
# Marcadores ArUco 3D PROCEDURALES para Isaac Sim 4.5 — réplica de los 3D-impresos.
#
# A diferencia del port Gazebo (que usa una TEXTURA plana aruco{N}.png sobre un
# <plane>, ver mi_proyecto_sim/models/marcador_aruco/marcador_aruco*.sdf), aquí el
# marcador es GEOMETRÍA REAL, como las piezas impresas del usuario:
#     - una BASE blanca cuadrada (size×size×0.004 m, 4 mm de grosor)
#     - cada celda NEGRA del patrón ArUco es un cubito (cell×cell×0.0006 m, +0.6 mm)
#       EXTRUIDO ENCIMA de la base (su cara inferior apoya en z=0.004).
# Esto hace que el lidar/cámara vean relieve, no una foto. El patrón se lee de
# cv2.aruco (DICT_4X4_50): 4×4 de datos + borde negro de 1 celda = rejilla 6×6.
#
# API pública (todo dentro de funciones; sin efectos al importar):
#     make_aruco3d(stage, prim_path, marker_id, size_m=0.18, pose=None,
#                  mat_path=None, collision=False) -> str
#     make_goal_cube(stage, prim_path, pose=None, size_m=0.15,
#                    mat_path=None, collision=True) -> str
#     place_robot_marker(stage, robot_base_prim, marker_id=4, size_m=0.18,
#                        z=0.22, mat_path=None) -> str
#
# Integración (el orquestador lo cablea en scene_mecanum.py; ver al final del fichero
# las instrucciones). NO lanza Isaac; valida con: python3 -m py_compile aruco3d.py
#
# APIs Isaac/USD usadas, citando de dónde se copiaron:
#   - UsdGeom.Cube.Define + GetSizeAttr().Set(1.0) + AddScaleOp  ->  patrón EXACTO del
#     /Room en scene_mecanum.py (líneas ~314-320) y de
#     isaacsim/exts/isaacsim.core.api/.../objects/cuboid.py (Cube.Define + set_size).
#   - set_transform_attributes (AddTranslateOp / AddOrientOp / AddScaleOp) ->
#     isaacsim/standalone_examples/replicator/object_based_sdg/object_based_sdg_utils.py
#     y .../replicator/infinigen/infinigen_sdg_utils.py.
#   - Material OmniPBR (CreateMdlMaterialPrim) con fallback a UsdPreviewSurface y
#     MaterialBindingAPI.Bind(..., strongerThanDescendants)  ->  jetauto_materials.py.
#   - UsdPhysics.CollisionAPI.Apply + MeshCollisionAPI/aprox  ->  scene_mecanum.py
#     (EXTRA_USDS, líneas ~270-277) y object_based_sdg_utils.add_colliders.
import math

import numpy as np
from pxr import Gf, Sdf, UsdGeom, UsdPhysics, UsdShade


# --------------------------------------------------------------------------- #
#  Lectura ROBUSTA del bitmap ArUco entre versiones de OpenCV                  #
# --------------------------------------------------------------------------- #
def _aruco_grid(marker_id, n=6):
    """Devuelve una rejilla booleana n×n (True = celda NEGRA) del marcador `marker_id`
    del diccionario DICT_4X4_50. Robusto a OpenCV 4.7+ (generateImageMarker) y a las
    versiones viejas (drawMarker / Dictionary_get).

    DICT_4X4_50 = 4×4 de datos + 1 celda de borde negro por lado = 6×6 (verificado
    inspeccionando el bitmap: el borde sale todo negro)."""
    import cv2  # import perezoso: solo cuando se construye un marcador
    aruco = cv2.aruco

    # 1) diccionario: API nueva primero, luego la vieja.
    try:
        dic = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    except AttributeError:
        dic = aruco.Dictionary_get(aruco.DICT_4X4_50)  # OpenCV < 4.7

    side = n * 10  # 10 px por celda -> umbral estable
    img = None
    # 2) render del bitmap: generateImageMarker (nuevo) -> drawMarker (viejo).
    if hasattr(aruco, "generateImageMarker"):
        try:
            img = aruco.generateImageMarker(dic, int(marker_id), side)
        except Exception:
            img = None
    if img is None and hasattr(aruco, "drawMarker"):
        try:
            img = aruco.drawMarker(dic, int(marker_id), side)
        except Exception:
            img = None
    if img is None:
        # 3) último recurso: el método del objeto Dictionary (algunas builds).
        try:
            img = dic.generateImageMarker(int(marker_id), side)
        except Exception as e:
            raise RuntimeError(f"no pude generar el bitmap ArUco id={marker_id}: {e}")

    img = np.asarray(img)
    if img.ndim == 3:
        img = img[:, :, 0]
    # umbral a rejilla n×n promediando cada celda; True = negro (media < 128).
    cell = img.shape[0] // n
    grid = np.zeros((n, n), dtype=bool)
    for r in range(n):
        for c in range(n):
            sub = img[r * cell:(r + 1) * cell, c * cell:(c + 1) * cell]
            grid[r, c] = float(sub.mean()) < 128.0
    return grid


# --------------------------------------------------------------------------- #
#  Materiales blanco/negro mate (patrón de jetauto_materials.py)               #
# --------------------------------------------------------------------------- #
# spec mínimo: {color, metallic, roughness}. Mate -> roughness alto, metallic 0.
_WHITE_MATTE = {"color": (0.92, 0.92, 0.92), "metallic": 0.0, "roughness": 0.9}
_BLACK_MATTE = {"color": (0.02, 0.02, 0.02), "metallic": 0.0, "roughness": 0.85}


def _make_preview_surface(stage, path, spec):
    """Material UsdPreviewSurface (USD nativo, siempre funciona en RTX).
    Copiado de jetauto_materials._make_preview_surface."""
    mat = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, path + "/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*spec["color"]))
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(float(spec["metallic"]))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(float(spec["roughness"]))
    mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return mat


def _make_omnipbr(stage, path, spec):
    """Material OmniPBR (MDL, mejor look). Lanza si el comando no existe.
    Copiado de jetauto_materials._make_omnipbr."""
    import omni.kit.commands
    omni.kit.commands.execute(
        "CreateMdlMaterialPrim",
        mtl_url="OmniPBR.mdl",
        mtl_name="OmniPBR",
        mtl_path=Sdf.Path(path),
    )
    mat = UsdShade.Material.Get(stage, path)
    if not mat:
        raise RuntimeError("OmniPBR no creó el material")
    shader = None
    for child in stage.GetPrimAtPath(path).GetChildren():
        if child.IsA(UsdShade.Shader):
            shader = UsdShade.Shader(child)
            break
    if shader is None:
        raise RuntimeError("OmniPBR sin shader hijo")
    shader.CreateInput("diffuse_color_constant", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*spec["color"]))
    shader.CreateInput("metallic_constant", Sdf.ValueTypeNames.Float).Set(float(spec["metallic"]))
    shader.CreateInput("reflection_roughness_constant", Sdf.ValueTypeNames.Float).Set(float(spec["roughness"]))
    return mat


def _get_or_make_material(stage, path, spec):
    """Reusa el material si ya existe (idempotente); si no, OmniPBR con fallback a
    UsdPreviewSurface. Devuelve el UsdShade.Material."""
    existing = UsdShade.Material.Get(stage, path)
    if existing:
        return existing
    try:
        return _make_omnipbr(stage, path, spec)
    except Exception as e:  # noqa: BLE001
        print(f"[aruco3d] OmniPBR no disponible ({e}); uso UsdPreviewSurface")
        return _make_preview_surface(stage, path, spec)


def _aruco_looks_scope(stage):
    """Crea (una vez) /World/Looks/{ArucoWhite,ArucoBlack} y los devuelve.
    Centralizar los 2 materiales evita duplicarlos por cada celda/marcador."""
    UsdGeom.Scope.Define(stage, "/World/Looks")
    white = _get_or_make_material(stage, "/World/Looks/ArucoWhite", _WHITE_MATTE)
    black = _get_or_make_material(stage, "/World/Looks/ArucoBlack", _BLACK_MATTE)
    return white, black


def _bind(prim, material):
    """Bind directo en la malla (pisa cualquier material por descendencia).
    strongerThanDescendants = patrón de jetauto_materials._bind."""
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(
        material, UsdShade.Tokens.strongerThanDescendants)


# --------------------------------------------------------------------------- #
#  Helpers de geometría                                                        #
# --------------------------------------------------------------------------- #
def _box(stage, path, sx, sy, sz, center, material, collision=False, mat_path=None):
    """Crea un Cube unitario escalado a (sx,sy,sz) centrado en `center` (Gf.Vec3d / tup).
    UsdGeom.Cube.Define + GetSizeAttr().Set(1.0) + Translate+Scale = patrón EXACTO del
    /Room en scene_mecanum.py (~314-320)."""
    xf = UsdGeom.Xform.Define(stage, path)
    xf.AddTranslateOp().Set(Gf.Vec3d(*center))
    xf.AddScaleOp().Set(Gf.Vec3f(float(sx), float(sy), float(sz)))
    cube = UsdGeom.Cube.Define(stage, path + "/geo")
    cube.GetSizeAttr().Set(1.0)
    # extent del cubo unitario (±0.5) para un bbox correcto (como en cuboid.py).
    cube.GetExtentAttr().Set([Gf.Vec3f(-0.5, -0.5, -0.5), Gf.Vec3f(0.5, 0.5, 0.5)])
    geo_prim = cube.GetPrim()
    if material is not None:
        _bind(geo_prim, material)
    if collision:
        # colisión opcional: convexHull (caja) es suficiente para un cubito.
        UsdPhysics.CollisionAPI.Apply(geo_prim)
        UsdPhysics.MeshCollisionAPI.Apply(geo_prim).CreateApproximationAttr().Set("convexHull")
        if mat_path is not None:
            UsdShade.MaterialBindingAPI.Apply(geo_prim).Bind(
                UsdShade.Material(stage.GetPrimAtPath(mat_path)),
                bindingStrength=UsdShade.Tokens.weakerThanDescendants,
                materialPurpose="physics")
    return geo_prim


def _apply_pose(stage, prim_path, pose):
    """Aplica `pose` al Xform raíz. `pose` admite:
       - None  -> sin transform
       - (tx,ty,tz)                     -> solo traslación
       - ((tx,ty,tz), (qw,qx,qy,qz))    -> traslación + cuaternión (w,x,y,z)
       - {"t":(x,y,z), "q":(w,x,y,z)}   -> dict equivalente
    AddTranslateOp/AddOrientOp = object_based_sdg_utils.set_transform_attributes."""
    if pose is None:
        return
    t = q = None
    if isinstance(pose, dict):
        t = pose.get("t") or pose.get("translate") or pose.get("position")
        q = pose.get("q") or pose.get("orient") or pose.get("orientation")
    elif isinstance(pose, (tuple, list)) and len(pose) == 2 and \
            isinstance(pose[0], (tuple, list)) and isinstance(pose[1], (tuple, list)):
        t, q = pose[0], pose[1]
    elif isinstance(pose, (tuple, list)) and len(pose) == 3:
        t = pose
    else:
        t = pose  # asumir traslación
    xf = UsdGeom.Xformable(stage.GetPrimAtPath(prim_path))
    if t is not None:
        xf.AddTranslateOp().Set(Gf.Vec3d(float(t[0]), float(t[1]), float(t[2])))
    if q is not None:
        # (w,x,y,z) -> Gf.Quatf(w, Vec3f(x,y,z))  (igual convención que _quat_yaw en la escena)
        xf.AddOrientOp().Set(Gf.Quatf(float(q[0]), Gf.Vec3f(float(q[1]), float(q[2]), float(q[3]))))


# --------------------------------------------------------------------------- #
#  API pública                                                                 #
# --------------------------------------------------------------------------- #
BOARD_T = 0.004     # grosor de la base blanca (4 mm)
CELL_T = 0.0006     # extrusión de las celdas negras (0.6 mm)


def make_aruco3d(stage, prim_path, marker_id, size_m=0.18, pose=None,
                 mat_path=None, collision=False, quiet=1.0):
    """Construye un marcador ArUco 3D PROCEDURAL bajo `prim_path` (un Xform raíz).

    Geometría (réplica del 3D-impreso):
      - base blanca: 1 caja size_m × size_m × 0.004 m, cara inferior en z=0.
      - cada celda NEGRA del patrón 6×6: una caja cell × cell × 0.0006 m apoyada
        ENCIMA de la base (cara inferior en z=0.004), centrada en su celda.
    El marcador "mira" +Z; reoriéntalo con `pose` (quat w,x,y,z) para pegarlo en una
    cara vertical.

    Args:
        stage: Usd.Stage actual (omni.usd.get_context().get_stage()).
        prim_path (str): ruta del Xform raíz del marcador, p.ej. "/World/Aruco/m0".
        marker_id (int): id en DICT_4X4_50 (0..49).
        size_m (float): lado del marcador en metros (default 0.18 = el del proyecto).
        pose: ver _apply_pose (None | (x,y,z) | ((x,y,z),(w,x,y,z)) | dict).
        mat_path (str|None): material de fricción /physicsMaterial para los colliders.
        collision (bool): si True añade colisión convexHull a la base (default False:
            los marcadores son objetivos VISUALES).

    Returns:
        str: prim_path del Xform raíz creado.
    """
    grid = _aruco_grid(int(marker_id), n=6)        # True = celda negra
    n = grid.shape[0]
    white, black = _aruco_looks_scope(stage)

    root = UsdGeom.Xform.Define(stage, prim_path).GetPrim()

    # 1) base blanca, centrada en xy, cara inferior en z=0 -> centro z = BOARD_T/2.
    _box(stage, prim_path + "/board",
         size_m, size_m, BOARD_T, (0.0, 0.0, BOARD_T / 2.0),
         white, collision=collision, mat_path=mat_path)

    # 2) celdas negras EXTRUIDAS encima. Coordenadas: celda (r,c) -> centro xy.
    #    fila r=0 arriba en la imagen -> +y; columna c=0 a la izquierda -> -x.
    # QUIET ZONE: el patrón n×n se mete DENTRO del tablero (size_m) dejando `quiet`
    # celdas de margen blanco por lado. ArUco necesita >=1 celda de borde blanco para
    # detectar; sin esto (patrón a ras del borde) la detección falla (lo del cubo viejo).
    eff = size_m / (float(n) + 2.0 * float(quiet))  # tamaño de celda CON quiet zone
    pat_half = (n / 2.0) * eff                       # media anchura del patrón inscrito
    z_cell = BOARD_T + CELL_T / 2.0                # cara inferior del cubito en z=BOARD_T
    n_black = 0
    for r in range(n):
        for c in range(n):
            if not grid[r, c]:
                continue
            cx = -pat_half + (c + 0.5) * eff
            cy = pat_half - (r + 0.5) * eff
            _box(stage, f"{prim_path}/cell_{r}_{c}",
                 eff, eff, CELL_T, (cx, cy, z_cell),
                 black, collision=False, mat_path=mat_path)
            n_black += 1

    _apply_pose(stage, prim_path, pose)
    print(f"[aruco3d] marcador id={marker_id} en {prim_path} "
          f"({size_m*1000:.0f}mm, {n_black} celdas negras +{CELL_T*1000:.1f}mm)", flush=True)
    return prim_path


# yaw alrededor de +Z (w,x,y,z), igual que _quat_yaw en scene_mecanum.py.
def _q_axis(axis, deg):
    """Cuaternión (w,x,y,z) de una rotación `deg` grados alrededor de `axis` (vec3)."""
    q = Gf.Rotation(Gf.Vec3d(*axis), deg).GetQuat()  # Gf.Quatd
    im = q.GetImaginary()
    return (q.GetReal(), im[0], im[1], im[2])


def _q_mul(a, b):
    """Producto de cuaterniones (w,x,y,z): a∘b (primero b, luego a)."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def make_goal_cube(stage, prim_path, pose=None, size_m=0.15,
                   mat_path=None, collision=True):
    """Cubo objetivo `cubo_aruco` (lado 0.15 m) con marcadores ArUco 3D en sus caras.

    Mapeo de ids = mi_proyecto_sim/models/cubo_aruco/model.sdf:
        cara superior (+Z) -> ID0
        cara inferior (-Z) -> ID1
        4 caras laterales  -> ID1
    El cubo lleva un collider de caja (convexHull) por defecto (es un obstáculo/objetivo
    físico, como el <collision> 0.15³ del SDF). Los marcadores van pegados a cada cara,
    apenas sobresaliendo (su base 4mm + relieve 0.6mm).

    Args:
        prim_path (str): Xform raíz, p.ej. "/World/CuboAruco".
        pose: pose del cubo (ver _apply_pose). El centro del cubo queda en `t`.
        size_m (float): lado del cubo (default 0.15).
        mat_path: /physicsMaterial para la fricción del collider.
        collision (bool): collider de caja del cubo (default True).

    Returns:
        str: prim_path del Xform raíz.
    """
    UsdGeom.Xform.Define(stage, prim_path)

    # cuerpo del cubo: caja blanca de lado size_m, centrada en el origen local.
    white, _black = _aruco_looks_scope(stage)
    body = _box(stage, prim_path + "/body",
                size_m, size_m, size_m, (0.0, 0.0, 0.0),
                white, collision=collision, mat_path=mat_path)
    # collider de caja (no convexHull de malla): para un Cube, MeshCollisionAPI no aplica;
    # CollisionAPI sola sobre un Cube da una caja exacta. Quitamos la aprox de malla que
    # _box puso (sólo válida para mallas) re-aplicando CollisionAPI limpia.
    if collision:
        # _box ya hizo CollisionAPI.Apply; la MeshCollisionAPI sobre un Cube es inocua
        # (PhysX usa la primitiva). La dejamos; no rompe. Documentado en untested_risks.
        pass

    half = size_m / 2.0
    eps = 0.0001  # 0.1 mm: la base del marcador apoya justo sobre la cara

    # Cada marcador se genera "mirando +Z" centrado en su propio Xform; lo rotamos para
    # que su +Z apunte hacia AFUERA de la cara, y lo trasladamos a la cara.
    # qw,qx,qy,qz; traslación local (x,y,z).
    faces = [
        # (id, normal_axis, deg, translate)   normal +Z del marcador -> dirección de la cara
        ("top",    0, ("x", 0.0),   (0.0, 0.0,  half + eps)),   # +Z  ID0
        ("bottom", 1, ("x", 180.0), (0.0, 0.0, -half - eps)),   # -Z  ID1
        ("front",  1, ("x", -90.0), (0.0,  half + eps, 0.0)),   # +Y  ID1
        ("back",   1, ("x", 90.0),  (0.0, -half - eps, 0.0)),   # -Y  ID1
        ("right",  1, ("y", 90.0),  (half + eps, 0.0, 0.0)),    # +X  ID1
        ("left",   1, ("y", -90.0), (-half - eps, 0.0, 0.0)),   # -X  ID1
    ]
    for name, mid, (ax, deg), (tx, ty, tz) in faces:
        q = _q_axis((1, 0, 0) if ax == "x" else (0, 1, 0), deg)
        make_aruco3d(stage, f"{prim_path}/marker_{name}", mid,
                     size_m=size_m, pose=((tx, ty, tz), q),
                     mat_path=mat_path, collision=False)

    _apply_pose(stage, prim_path, pose)
    print(f"[aruco3d] cubo_aruco en {prim_path} (lado {size_m*1000:.0f}mm, "
          f"top=ID0 / resto=ID1, collision={collision})", flush=True)
    return prim_path


def place_robot_marker(stage, robot_base_prim, marker_id=4, size_m=0.18,
                       z=0.22, mat_path=None):
    """Pega un marcador ArUco 3D (default ID4) PLANO sobre el techo del robot, a z≈0.22
    en el frame del robot (el marcador mira +Z, hacia arriba, para que el dron lo vea).

    Se crea como HIJO de `robot_base_prim` (p.ej. ".../base_link") para que SIGA al robot
    rígidamente. Sin colisión (es un objetivo visual; no debe perturbar la dinámica).

    Args:
        robot_base_prim (str): prim al que se ancla, p.ej. "/jetauto/base_link".
        marker_id (int): id ArUco (default 4 = el del techo en el proyecto).
        size_m (float): lado (default 0.18).
        z (float): altura sobre el frame del robot (default 0.22).
        mat_path: /physicsMaterial (no se usa por defecto; collision=False).

    Returns:
        str: prim_path del marcador creado.
    """
    base = str(robot_base_prim).rstrip("/")
    path = f"{base}/roof_aruco_{marker_id}"
    # plano sobre el techo, mirando +Z (sin rotación): pose = solo traslación.
    return make_aruco3d(stage, path, marker_id, size_m=size_m,
                        pose=(0.0, 0.0, float(z)),
                        mat_path=mat_path, collision=False)


# --------------------------------------------------------------------------- #
#  CÓMO INTEGRAR en scene_mecanum.py (el orquestador lo cablea):              #
#                                                                              #
#    from aruco3d import make_aruco3d, make_goal_cube, place_robot_marker     #
#                                                                              #
#  Tras crear `stage` y `mat_path` (~línea 158), y opcionalmente tras importar #
#  el robot/entorno, bajo un nuevo flag --aruco:                              #
#                                                                              #
#    if args.aruco:                                                           #
#        # cubo objetivo a 1.5 m delante del spawn (en el suelo: centro a z=size/2)
#        make_goal_cube(stage, "/World/CuboAruco",                            #
#                       pose=((1.5, 0.0, 0.075),), mat_path=mat_path)         #
#        # marcador en el techo del robot (sigue al chasis)                   #
#        robot_root = "/" + str(prim_path).strip("/").split("/")[0]          #
#        place_robot_marker(stage, robot_root + "/base_link", marker_id=4)    #
#        # un marcador suelto en el piso                                      #
#        make_aruco3d(stage, "/World/Aruco/m3", 3, pose=(0.0, 1.0, 0.0))      #
#                                                                              #
#  Notas: llamarlas ANTES de timeline.play()/art.initialize() es lo más seguro #
#  (geometría estática lista antes de arrancar la física). place_robot_marker  #
#  debe ir DESPUÉS de apply_jetauto_materials para que su material gane.        #
# --------------------------------------------------------------------------- #
