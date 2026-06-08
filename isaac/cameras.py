#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# ============================================================================
# cameras.py — multi-camera rig + recording for show/debug (Isaac Sim 4.5)
# ============================================================================
#
# Módulo COMPLEMENTARIO de scene_mecanum.py (port mecanum por física). Aquí NO
# se simula nada nuevo: se añaden cámaras para *enseñar* y *depurar* la escena:
#
#   - wheel-cam  : cámara HIJA de un link de rueda (se mueve y gira con la rueda)
#   - scene-cam  : cámara FIJA en el mundo (vista panorámica de la escena)
#   - chase-cam  : cámara que SIGUE al robot (3ra persona, detrás y arriba)
#   - top-cam    : cenital opcional (mira hacia abajo desde arriba del robot)
#
# y permite:
#   - crear un viewport (ventana GUI) por cámara para verlas en vivo,
#   - cambiar la resolución de un render_product por script (presets 720p/1080p/4K),
#   - GRABAR N cámaras a disco a la vez con omni.replicator.core (BasicWriter -> PNG).
#
# El módulo NO tiene efectos al importarse: todo el trabajo vive dentro de
# funciones/clases. El orquestador llama a setup_cameras(...) tras crear el robot,
# y opcionalmente create_viewports(...) (GUI) y/o un Recorder (disco).
#
# ----------------------------------------------------------------------------
# CÓMO SE GRABA / SE MUESTRA (3 caminos, de menor a mayor coste):
# ----------------------------------------------------------------------------
#   (A) GUI en vivo  -> create_viewports(): una ventana por cámara. NO escribe a
#       disco; sirve para *mirar*. Requiere --headless OFF.
#   (B) Recorder     -> render_product por cámara + BasicWriter(rgb=True) que
#       vuelca PNG por cámara y frame a disco. Patrón EXACTO de:
#         standalone_examples/api/isaacsim.replicator.examples/multi_camera.py
#       (rep.create.render_product(cam, resolution) + WriterRegistry/BasicWriter
#        .initialize(output_dir=..., rgb=True) + .attach([rp...])). Para hacer un
#       MP4 al final: ffmpeg sobre la secuencia de PNG (helper pngs_to_mp4()).
#   (C) OBS / captura de pantalla  -> ver docstring "OBS ALTERNATIVE" abajo.
#
# ----------------------------------------------------------------------------
# OBS ALTERNATIVE (grabar la ventana de Isaac sin tocar el código):
# ----------------------------------------------------------------------------
#   La ventana de Isaac Sim es una ventana X11/Wayland normal, así que cualquier
#   capturador de pantalla la graba sin que el script haga nada:
#     * OBS Studio: fuente "Screen Capture (XSHM)" en X11, o "PipeWire / Screen
#       Capture" en Wayland; o "Window Capture (Xcomposite)" apuntando a la
#       ventana "Isaac Sim". Graba a MP4/MKV con audio opcional. Ideal para la
#       demo de Vilchis (varios viewports a la vez en una sola grabación).
#     * Sin OBS:  wf-recorder -o <salida.mp4>   (Wayland) ó
#                 ffmpeg -f x11grab -framerate 30 -i :0.0 out.mp4   (X11).
#   Ventajas: cero coste de GPU extra para SDG, captura *exactamente* lo que se ve
#   (incluyendo varios viewports/HUD). Desventaja: resolución limitada al tamaño
#   de la ventana y no hay imágenes "limpias" por cámara (para eso usa el Recorder).
#   En --headless NO sirve OBS (no hay ventana): usa el Recorder (camino B).
# ============================================================================

import math
import os
from typing import Dict, List, Optional, Sequence, Tuple

# NOTA: NO importar isaacsim/omni/pxr a nivel de módulo. Igual que scene_mecanum.py,
# esos módulos solo existen DESPUÉS de crear SimulationApp. Se importan DENTRO de
# las funciones para que `python3 -m py_compile cameras.py` (python del sistema)
# valide la sintaxis sin Isaac.


# ----------------------------------------------------------------------------
# Presets de resolución (el menú "Resolution" del viewport en la GUI de Isaac).
# Son los mismos valores que ofrece el dropdown del viewport. Úsalos por nombre
# en setup_cameras(specs) o en set_resolution(rp_path, RESOLUTION_PRESETS["1080p"]).
# El viewport de la GUI tiene además "Fill Viewport" / "Custom"; por script lo que
# importa es el (width, height) del render_product, que es lo que estos presets dan.
# ----------------------------------------------------------------------------
RESOLUTION_PRESETS: Dict[str, Tuple[int, int]] = {
    # nombre amigable          (width, height)
    "icon": (256, 256),
    "qvga": (320, 240),
    "vga": (640, 480),
    "svga": (800, 600),
    "720p": (1280, 720),        # HD            (default del Recorder/viewports)
    "1080p": (1920, 1080),      # Full HD
    "1440p": (2560, 1440),      # QHD / 2K
    "4k": (3840, 2160),         # UHD / 4K
    "1_1": (1024, 1024),        # cuadrada (útil para debug por cámara)
    "square_512": (512, 512),
}


# ============================================================================
# Helper de resolución
# ============================================================================
def set_resolution(render_product_path: str, resolution):
    """Fija la resolución (width, height) de un render_product por script.

    `resolution` puede ser una tupla (w, h) o el NOMBRE de un preset
    (clave de RESOLUTION_PRESETS, p.ej. "1080p", "4k").

    Envuelve isaacsim.core.utils.render_product.set_resolution(rp_path, (w,h)),
    que escribe el atributo UsdRender.Product.resolution en la session layer.
    Citas:
      - isaacsim/core/utils/render_product.py: def set_resolution(render_product_path, resolution)
        (usa UsdRender.Product(...).GetResolutionAttr().Set(Gf.Vec2i(w, h)))
      - exts/.../sensors/camera/camera.py: Camera.set_resolution() llama a esta misma utilidad.

    Presets (= dropdown de resolución del viewport en la GUI):
        720p=1280x720, 1080p=1920x1080, 1440p=2560x1440, 4k=3840x2160, ...
    Ver el dict RESOLUTION_PRESETS para la lista completa.
    """
    from isaacsim.core.utils.render_product import set_resolution as _set_res

    if isinstance(resolution, str):
        if resolution not in RESOLUTION_PRESETS:
            raise KeyError(
                f"preset de resolución desconocido: {resolution!r}; "
                f"opciones: {sorted(RESOLUTION_PRESETS)}"
            )
        resolution = RESOLUTION_PRESETS[resolution]
    _set_res(render_product_path, (int(resolution[0]), int(resolution[1])))
    return tuple(resolution)


def _resolve_res(res, default=(1280, 720)) -> Tuple[int, int]:
    """Normaliza un valor de resolución (preset-string | (w,h) | None) a (w,h)."""
    if res is None:
        return default
    if isinstance(res, str):
        return RESOLUTION_PRESETS.get(res, default)
    return (int(res[0]), int(res[1]))


# ============================================================================
# Búsqueda de prims (rueda / robot) — robusta a instancing
# ============================================================================
def find_link_prim(stage, name_contains: str, under_path: Optional[str] = None):
    """Devuelve la RUTA (str) del primer prim cuyo nombre contiene `name_contains`.

    Útil para localizar un link de rueda (p.ej. "front_left_wheel_link") sin
    saber su ruta exacta tras el import del URDF. Si `under_path` se da, busca solo
    bajo ese subárbol. Patrón = scene_mecanum.py (recorre stage.Traverse() buscando
    p.GetName() == "lidar_frame").
    """
    from pxr import Usd

    root = stage.GetPrimAtPath(under_path) if under_path else None
    it = Usd.PrimRange(root) if (root and root.IsValid()) else stage.Traverse()
    for p in it:
        if name_contains in p.GetName():
            return p.GetPath().pathString
    return None


# ============================================================================
# Construcción del rig de cámaras
# ============================================================================
def setup_cameras(
    stage,
    simulation_app,
    robot_prim: str,
    wheel_link_prim: Optional[str] = None,
    specs: Optional[Dict[str, dict]] = None,
) -> Dict[str, object]:
    """Crea el rig de cámaras y devuelve un dict {nombre: isaacsim.sensors.camera.Camera}.

    Args:
        stage:           Usd.Stage actual (omni.usd.get_context().get_stage()).
        simulation_app:  el SimulationApp (para .update() y calentar el render).
        robot_prim:      ruta del articulation root del robot
                         (p.ej. "/jetauto/base_footprint").
        wheel_link_prim: ruta del LINK de rueda donde colgar la wheel-cam. Si es
                         None, se busca un prim cuyo nombre contenga "wheel"
                         (front_left_wheel_link, etc.) bajo el robot.
        specs:           dict opcional para sobreescribir defaults por cámara.
                         Claves reconocidas: "wheel", "scene", "chase", "top".
                         Cada valor es un sub-dict con cualquiera de:
                            "enabled" (bool, default True salvo "top"=False),
                            "resolution" (preset-str o (w,h)),
                            "translation" (x,y,z) local/world según la cámara,
                            "focal_length" (mm).
                         La chase-cam usa además "offset" y "height" (ver abajo).

    Devuelve: dict {nombre: Camera}. La chase-cam y la wheel-cam adjuntan metadatos
    en el objeto (atributos _chase_*) que update_chase_camera() consume cada frame.

    APIs (citadas del install local):
      - Camera(prim_path, name, resolution, translation/position, orientation):
        exts/isaacsim.sensors.camera/.../camera.py  (class Camera, __init__ firma).
        Crea el UsdGeom.Camera si no existe en prim_path.
      - cam.initialize(): exts/.../camera.py def initialize() -> crea el
        render_product (rep.create.render_product) y engancha el annotator "rgb".
      - cam.set_world_pose / set_local_pose(translation, orientation, camera_axes):
        exts/.../camera.py. camera_axes="world" => (+Z up, +X forward) como ROS/robótica.
      - colgar la cámara de un link: se crea el prim Camera COMO HIJO del link
        (prim_path = <link>/<cam_name>), igual que scene_mecanum.py cuelga el
        RTX lidar del prim "lidar_frame". Como hijo, hereda el transform del link
        (se mueve y gira con la rueda).
    """
    from isaacsim.sensors.camera import Camera
    from pxr import Gf, Sdf, UsdGeom

    specs = specs or {}
    cams: Dict[str, object] = {}

    def _spec(key, default):
        s = dict(default)
        s.update(specs.get(key, {}) or {})
        return s

    # ---- helper: orientación "mirar hacia (target) desde (eye)" en quat (w,x,y,z) world.
    def _look_quat_world(eye, target, up=(0.0, 0.0, 1.0)):
        # Matriz de "mirar a" (USD). La invertimos para obtener el transform de la
        # cámara, y de ahí extraemos el quaternion. camera_axes="world" en
        # set_world_pose espera (w,x,y,z) scalar-first.
        m = Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*target), Gf.Vec3d(*up))
        q = m.GetInverse().ExtractRotationQuat()  # Gf.Quatd
        im = q.GetImaginary()
        return [q.GetReal(), im[0], im[1], im[2]]

    # ------------------------------------------------------------------
    # 1) WHEEL-CAM — hija de un link de rueda (se mueve con la rueda)
    # ------------------------------------------------------------------
    wspec = _spec("wheel", {
        "enabled": True, "resolution": "720p",
        "translation": (0.12, 0.0, 0.06),   # un poco delante y arriba del eje de la rueda
        "focal_length": 12.0,
    })
    if wspec.get("enabled", True):
        link = wheel_link_prim or find_link_prim(stage, "wheel_link", under_path=robot_prim) \
            or find_link_prim(stage, "wheel", under_path=robot_prim)
        if link is None:
            print("[CAM] aviso: no encontré un link de rueda; wheel-cam omitida", flush=True)
        else:
            wpath = link.rstrip("/") + "/wheel_cam"
            w, h = _resolve_res(wspec.get("resolution"), (1280, 720))
            # translation = LOCAL respecto al link (al pasar translation, no position).
            wheel_cam = Camera(
                prim_path=wpath, name="wheel_cam",
                resolution=(w, h),
                translation=tuple(wspec.get("translation", (0.12, 0.0, 0.06))),
            )
            wheel_cam.set_focal_length(float(wspec.get("focal_length", 12.0)))
            cams["wheel"] = wheel_cam
            print(f"[CAM] wheel-cam (hija de {link}) en {wpath}  {w}x{h}", flush=True)

    # ------------------------------------------------------------------
    # 2) SCENE-CAM — fija en el mundo, vista panorámica
    # ------------------------------------------------------------------
    sspec = _spec("scene", {
        "enabled": True, "resolution": "1080p",
        "eye": (5.0, -5.0, 4.0), "target": (0.0, 0.0, 0.3),
        "focal_length": 18.0,
    })
    if sspec.get("enabled", True):
        spath = "/World/SceneCam"
        # /World puede no existir (la escena vacía de Kalman no lo crea); asegúralo.
        if not stage.GetPrimAtPath("/World").IsValid():
            UsdGeom.Xform.Define(stage, "/World")
        w, h = _resolve_res(sspec.get("resolution"), (1920, 1080))
        scene_cam = Camera(prim_path=spath, name="scene_cam", resolution=(w, h))
        scene_cam.set_focal_length(float(sspec.get("focal_length", 18.0)))
        cams["scene"] = scene_cam
        # pose world: mirar al origen/robot desde un punto elevado.
        eye = tuple(sspec.get("eye", (5.0, -5.0, 4.0)))
        tgt = tuple(sspec.get("target", (0.0, 0.0, 0.3)))
        scene_cam.set_world_pose(
            position=list(eye),
            orientation=_look_quat_world(eye, tgt),
            camera_axes="world",
        )
        print(f"[CAM] scene-cam fija en {spath} {w}x{h} (eye={eye})", flush=True)

    # ------------------------------------------------------------------
    # 3) CHASE-CAM — sigue al robot (3ra persona). Se actualiza por frame
    #    con update_chase_camera(); aquí solo se crea y se guardan params.
    # ------------------------------------------------------------------
    cspec = _spec("chase", {
        "enabled": True, "resolution": "1080p",
        "offset": (-1.6, 0.0, 0.9),    # detrás (-x) y arriba (+z) en marco del robot
        "look_height": 0.2,            # mira a z=0.2 sobre el robot
        "focal_length": 18.0,
    })
    if cspec.get("enabled", True):
        cpath = "/World/ChaseCam"
        w, h = _resolve_res(cspec.get("resolution"), (1920, 1080))
        chase_cam = Camera(prim_path=cpath, name="chase_cam", resolution=(w, h))
        chase_cam.set_focal_length(float(cspec.get("focal_length", 18.0)))
        # metadatos para update_chase_camera()
        chase_cam._chase_robot_prim = robot_prim
        chase_cam._chase_offset = tuple(cspec.get("offset", (-1.6, 0.0, 0.9)))
        chase_cam._chase_look_height = float(cspec.get("look_height", 0.2))
        cams["chase"] = chase_cam
        print(f"[CAM] chase-cam en {cpath} {w}x{h} (offset={chase_cam._chase_offset})", flush=True)

    # ------------------------------------------------------------------
    # 4) TOP-CAM — cenital opcional (off por defecto)
    # ------------------------------------------------------------------
    tspec = _spec("top", {
        "enabled": False, "resolution": "1080p",
        "height": 4.0, "follow": True, "focal_length": 15.0,
    })
    if tspec.get("enabled", False):
        tpath = "/World/TopCam"
        w, h = _resolve_res(tspec.get("resolution"), (1920, 1080))
        top_cam = Camera(prim_path=tpath, name="top_cam", resolution=(w, h))
        top_cam.set_focal_length(float(tspec.get("focal_length", 15.0)))
        top_cam._top_height = float(tspec.get("height", 4.0))
        top_cam._top_follow = bool(tspec.get("follow", True))
        top_cam._top_robot_prim = robot_prim
        # pose inicial: cenital sobre (0,0). Mirar hacia abajo (up=+Y para que el
        # "arriba" de la imagen sea +Y del mundo). camera_axes="world".
        h_z = top_cam._top_height
        top_cam.set_world_pose(
            position=[0.0, 0.0, h_z],
            orientation=_look_quat_world((0.0, 0.0, h_z), (0.0, 0.0, 0.0), up=(0.0, 1.0, 0.0)),
            camera_axes="world",
        )
        cams["top"] = top_cam
        print(f"[CAM] top-cam cenital en {tpath} {w}x{h} (z={h_z})", flush=True)

    # ------------------------------------------------------------------
    # initialize() de cada cámara: crea su render_product y annotator "rgb".
    # DEBE hacerse con la timeline ya en play (scene_mecanum.py llama
    # timeline.play() antes); luego unos update() para "calentar" el render
    # (igual que scene_mecanum.py hace 5 updates tras crear su TopCam).
    # ------------------------------------------------------------------
    for nm, cam in cams.items():
        try:
            cam.initialize()
        except Exception as e:  # no-fatal: una cámara mala no debe tumbar la escena
            print(f"[CAM] aviso: initialize() de '{nm}' falló: {e}", flush=True)
    for _ in range(5):
        simulation_app.update()

    print(f"[CAM] rig listo: {sorted(cams)}", flush=True)
    return cams


# ============================================================================
# Seguimiento por frame (chase-cam / top-cam que siguen al robot)
# ============================================================================
def _robot_pose(stage, robot_prim):
    """(pos xyz, yaw) del robot leyendo su transform world del stage. No-fatal."""
    import omni.timeline
    from pxr import Gf, UsdGeom

    prim = stage.GetPrimAtPath(robot_prim)
    if not prim or not prim.IsValid():
        return None, None
    t = omni.timeline.get_timeline_interface().get_current_time() * stage.GetTimeCodesPerSecond()
    m = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(t)
    pos = m.ExtractTranslation()
    # yaw a partir de la matriz de rotación (eje +X del robot proyectado en XY).
    rot = m.ExtractRotationMatrix()
    fwd = Gf.Vec3d(rot[0][0], rot[0][1], rot[0][2])  # fila 0 = imagen de +X
    yaw = math.atan2(fwd[1], fwd[0])
    return (float(pos[0]), float(pos[1]), float(pos[2])), float(yaw)


def update_chase_camera(stage, cams: Dict[str, object]):
    """Reposiciona chase-cam (y top-cam si follow=True) detrás/encima del robot.

    Llamar UNA VEZ por iteración del bucle principal, junto a simulation_app.update().
    Usa set_world_pose(camera_axes="world") de la Camera (exts/.../camera.py).
    No-fatal: si algo falla, no rompe el bucle de simulación.
    """
    from pxr import Gf

    def _look_quat_world(eye, target, up=(0.0, 0.0, 1.0)):
        m = Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*target), Gf.Vec3d(*up))
        q = m.GetInverse().ExtractRotationQuat()
        im = q.GetImaginary()
        return [q.GetReal(), im[0], im[1], im[2]]

    try:
        chase = cams.get("chase")
        if chase is not None:
            pos, yaw = _robot_pose(stage, chase._chase_robot_prim)
            if pos is not None:
                ox, oy, oz = chase._chase_offset
                c, s = math.cos(yaw), math.sin(yaw)
                # rota el offset del marco del robot al mundo (solo en XY)
                ex = pos[0] + (c * ox - s * oy)
                ey = pos[1] + (s * ox + c * oy)
                ez = pos[2] + oz
                tgt = (pos[0], pos[1], pos[2] + chase._chase_look_height)
                chase.set_world_pose(
                    position=[ex, ey, ez],
                    orientation=_look_quat_world((ex, ey, ez), tgt),
                    camera_axes="world",
                )
    except Exception as e:
        print(f"[CAM] update chase falló (lo ignoro): {e}", flush=True)

    try:
        top = cams.get("top")
        if top is not None and getattr(top, "_top_follow", False):
            pos, _ = _robot_pose(stage, top._top_robot_prim)
            if pos is not None:
                z = pos[2] + top._top_height
                top.set_world_pose(
                    position=[pos[0], pos[1], z],
                    orientation=_look_quat_world((pos[0], pos[1], z), (pos[0], pos[1], pos[2]),
                                                 up=(0.0, 1.0, 0.0)),
                    camera_axes="world",
                )
    except Exception as e:
        print(f"[CAM] update top falló (lo ignoro): {e}", flush=True)


# ============================================================================
# Viewports (GUI): una ventana por cámara
# ============================================================================
def create_viewports(camera_paths, resolution=None, tile: bool = True) -> Dict[str, object]:
    """Crea un viewport (ventana GUI) por cada cámara y lo apunta a esa cámara.

    Args:
        camera_paths: dict {nombre: ruta-de-prim-de-cámara}  ó  lista de rutas.
                      Acepta también un dict {nombre: Camera}; usa cam.prim_path.
        resolution:   resolución del viewport (preset-str o (w,h)) aplicada a TODOS;
                      None deja el default del viewport (1280x720).
        tile:         si True, reparte las ventanas en una rejilla para no apilarlas.

    Devuelve: dict {nombre: viewport_window} (omni.kit.viewport.window.ViewportWindow).

    APIs (citadas):
      - omni.kit.viewport.utility.create_viewport_window(name, width, height,
        position_x, position_y, camera_path=Sdf.Path):
        extscache/omni.kit.viewport.utility-*/omni/kit/viewport/utility/__init__.py
        (def create_viewport_window(...): crea un ViewportWindow y, si camera_path,
         setea window.viewport_api.camera_path).
      - get_active_viewport_and_window / get_viewport_from_window_name: misma utilidad.
      - para cambiar la cámara de un viewport ya creado: vp_window.viewport_api.camera_path = Sdf.Path(...)

    Nota: en --headless NO hay ventanas; esta función no tiene efecto útil ahí
    (usa el Recorder para grabar a disco). Requiere la GUI (headless=False).
    """
    from omni.kit.viewport.utility import create_viewport_window
    from pxr import Sdf

    # normaliza la entrada a dict {nombre: ruta-str}
    paths: Dict[str, str] = {}
    if isinstance(camera_paths, dict):
        for nm, v in camera_paths.items():
            paths[nm] = v.prim_path if hasattr(v, "prim_path") else str(v)
    else:
        for i, v in enumerate(camera_paths):
            p = v.prim_path if hasattr(v, "prim_path") else str(v)
            paths[os.path.basename(str(p)) or f"cam{i}"] = p

    w, h = _resolve_res(resolution, (640, 360)) if resolution is not None else (640, 360)

    windows: Dict[str, object] = {}
    n = len(paths)
    cols = max(1, int(math.ceil(math.sqrt(n)))) if tile else 1
    for i, (nm, cam_path) in enumerate(paths.items()):
        px = (i % cols) * (w + 30) if tile else 0
        py = (i // cols) * (h + 60) if tile else 0
        try:
            win = create_viewport_window(
                name=f"cam:{nm}",
                width=w, height=h,
                position_x=px, position_y=py,
                camera_path=Sdf.Path(cam_path),
            )
            windows[nm] = win
            print(f"[CAM] viewport 'cam:{nm}' -> {cam_path}", flush=True)
        except Exception as e:
            print(f"[CAM] no pude crear viewport para '{nm}': {e}", flush=True)
    return windows


def set_viewport_camera(window_name: str, camera_path: str):
    """Cambia, por script, la cámara mostrada en un viewport existente.

    Usa get_viewport_from_window_name(window_name).camera_path = Sdf.Path(...)
    (extscache/omni.kit.viewport.utility-*/.../__init__.py). Útil para reutilizar
    un solo viewport y rotar entre cámaras durante la demo.
    """
    from omni.kit.viewport.utility import get_viewport_from_window_name
    from pxr import Sdf

    vp = get_viewport_from_window_name(window_name)
    if vp is None:
        print(f"[CAM] no hay viewport llamado {window_name!r}", flush=True)
        return False
    vp.camera_path = Sdf.Path(camera_path)
    return True


# ============================================================================
# Recorder: graba N cámaras a la vez a disco vía omni.replicator.core
# ============================================================================
class Recorder:
    """Graba varias cámaras a la vez a disco (PNG por cámara y frame).

    Patrón EXACTO de:
      standalone_examples/api/isaacsim.replicator.examples/multi_camera.py
        rp = rep.create.render_product(str(cam_prim_path), resolution=(w,h))
        writer = rep.WriterRegistry.get("BasicWriter")   # o un Writer custom
        writer.initialize(output_dir=..., rgb=True)
        writer.attach([rp1, rp2, ...])
        rep.orchestrator.set_capture_on_play(False)       # capturar manualmente
        for i in range(N): rep.orchestrator.step(rt_subframes=...)

    Aquí cada cámara recibe su PROPIO render_product (vía un 'name' único, lo que
    fuerza force_new=True en rep.create.render_product) y su PROPIO BasicWriter con
    un output_dir distinto -> así cada cámara escribe a su carpeta sin mezclar.

    Uso típico en el bucle de scene_mecanum.py:
        rec = Recorder({"chase": chase_cam, "wheel": wheel_cam}, out_dir, fps=30)
        rec.start()                      # cuando empiece el movimiento
        ...
        while running:
            simulation_app.update()
            rec.step()                   # captura un frame si toca (según fps)
        rec.stop()
        rec.to_mp4()                     # opcional: PNG -> MP4 con ffmpeg

    Args:
        cameras:  dict {nombre: Camera}  ó  {nombre: ruta-de-prim-str}.
        out_dir:  carpeta base; cada cámara escribe en out_dir/<nombre>/.
        fps:      frames por segundo a volcar (se decima respecto al render real).
        resolution: opcional, fuerza la resolución del render_product de cada cámara
                    (preset-str o (w,h)); None usa la resolución actual de la cámara.
        image_format: "png" (default) o "jpeg" (BasicWriter.image_output_format).
        rt_subframes: subframes de raytracing por captura (calidad vs. velocidad);
                      el ejemplo usa 4. Solo se usa con use_orchestrator=True.
        use_orchestrator: si True, captura con rep.orchestrator.step() (escena
                      controlada por replicator). Si False (default, recomendado
                      para esta escena que ya corre su propio bucle de física),
                      solo se confía en set_capture_on_play(True): el writer
                      adjunto vuelca cada frame renderizado mientras la timeline
                      está en play; step() solo lleva la cuenta de fps/decimado.
    """

    def __init__(
        self,
        cameras: Dict[str, object],
        out_dir: str,
        fps: float = 30.0,
        resolution=None,
        image_format: str = "png",
        rt_subframes: int = 4,
        use_orchestrator: bool = False,
    ):
        self.cameras = cameras
        self.out_dir = out_dir
        self.fps = float(fps)
        self.resolution = resolution
        self.image_format = image_format
        self.rt_subframes = int(rt_subframes)
        self.use_orchestrator = bool(use_orchestrator)

        self._writers: Dict[str, object] = {}
        self._rps: Dict[str, object] = {}
        self._names: List[str] = []
        self._on = False
        self._t0 = None
        self._next_t = 0.0
        self._frames = 0
        self._built = False

    # -- construir render_products + writers (perezoso: en start()) ----------
    def _build(self):
        import omni.replicator.core as rep

        os.makedirs(self.out_dir, exist_ok=True)
        for nm, cam in self.cameras.items():
            cam_path = cam.prim_path if hasattr(cam, "prim_path") else str(cam)
            # resolución del render_product
            if self.resolution is not None:
                res = _resolve_res(self.resolution, (1280, 720))
            elif hasattr(cam, "get_resolution"):
                try:
                    res = tuple(cam.get_resolution())
                except Exception:
                    res = (1280, 720)
            else:
                res = (1280, 720)

            # render_product propio por cámara. Pasar name -> force_new=True
            # (create.py: si name no es None, force_new=True) => un rp único.
            rp = rep.create.render_product(cam_path, resolution=(int(res[0]), int(res[1])),
                                           name=f"rec_{nm}")
            sub = os.path.join(self.out_dir, nm)
            os.makedirs(sub, exist_ok=True)
            # BasicWriter: rgb=True -> escribe rgb_XXXX.<fmt> por frame.
            # (writers_default/basicwriter.py: __init__(output_dir, rgb, image_output_format,...))
            writer = rep.WriterRegistry.get("BasicWriter")
            writer.initialize(output_dir=sub, rgb=True, image_output_format=self.image_format)
            writer.attach([rp])
            self._writers[nm] = writer
            self._rps[nm] = rp
            self._names.append(nm)
            print(f"[REC] cámara '{nm}' -> {sub}  ({res[0]}x{res[1]}, {self.image_format})", flush=True)
        self._built = True

    def start(self, t0: Optional[float] = None):
        """Arranca la grabación. Construye writers la 1ra vez. t0 = tiempo de sim."""
        import omni.replicator.core as rep
        import omni.timeline

        if not self._built:
            self._build()
        # con use_orchestrator=False dejamos que el writer capture en cada frame
        # renderizado (set_capture_on_play True). Con True, capturamos manualmente.
        try:
            rep.orchestrator.set_capture_on_play(not self.use_orchestrator)
        except Exception as e:
            print(f"[REC] set_capture_on_play falló (sigo): {e}", flush=True)
        self._on = True
        self._t0 = t0 if t0 is not None else omni.timeline.get_timeline_interface().get_current_time()
        self._next_t = self._t0
        self._frames = 0
        print(f"[REC] ► grabando {sorted(self._names)} @ {self.fps:.0f} fps -> {self.out_dir}", flush=True)

    def step(self, t: Optional[float] = None):
        """Avanza el recorder; captura un frame si toca según fps. No-fatal.

        Devuelve True si en esta llamada se capturó un frame.
        """
        if not self._on:
            return False
        import omni.timeline

        try:
            if t is None:
                t = omni.timeline.get_timeline_interface().get_current_time()
            if t < self._next_t:
                return False
            if self.use_orchestrator:
                import omni.replicator.core as rep
                # step() entrega datos a annotators, dispara writers y avanza el render.
                rep.orchestrator.step(rt_subframes=self.rt_subframes)
            # con capture_on_play=True el writer ya vuelca el frame actual; aquí
            # solo contabilizamos y programamos el siguiente instante.
            self._frames += 1
            self._next_t += 1.0 / self.fps
            return True
        except Exception as e:
            print(f"[REC] error en step (desactivo recorder): {e}", flush=True)
            self._on = False
            return False

    def stop(self):
        """Detiene la grabación y suelta los render_products (liberan VRAM)."""
        import omni.replicator.core as rep

        if not self._built:
            return
        try:
            rep.orchestrator.set_capture_on_play(False)
        except Exception:
            pass
        for nm, writer in list(self._writers.items()):
            try:
                writer.detach()
            except Exception:
                pass
        for nm, rp in list(self._rps.items()):
            # destruir el render_product libera VRAM (create.py recomienda destruirlos).
            # rp es un HydraTexture (viewport_manager.py) con .destroy() y .path.
            try:
                rp.destroy()
            except Exception:
                pass
        self._on = False
        print(f"[REC] ■ detenido; {self._frames} frames/cámara en {self.out_dir}", flush=True)

    def to_mp4(self, fps: Optional[float] = None) -> Dict[str, str]:
        """Convierte la secuencia PNG de cada cámara a un MP4 (requiere ffmpeg en PATH).

        Devuelve {nombre: ruta_mp4}. No-fatal: si ffmpeg falla, lo reporta y sigue.
        """
        outs: Dict[str, str] = {}
        for nm in self._names:
            sub = os.path.join(self.out_dir, nm)
            mp4 = os.path.join(self.out_dir, f"{nm}.mp4")
            ok = pngs_to_mp4(sub, mp4, fps=fps or self.fps, pattern="rgb_%04d." + self.image_format)
            if ok:
                outs[nm] = mp4
        return outs


# ============================================================================
# Utilidad: PNG -> MP4 con ffmpeg (igual estilo que el _rec_step de scene_mecanum)
# ============================================================================
def pngs_to_mp4(png_dir: str, mp4_path: str, fps: float = 30.0,
                pattern: str = "rgb_%04d.png") -> bool:
    """Une la secuencia <png_dir>/<pattern> en un MP4 con ffmpeg. No-fatal.

    BasicWriter nombra los archivos rgb_0000.png, rgb_0001.png, ... (frame_padding=4),
    por eso el patrón por defecto es 'rgb_%04d.png'. Devuelve True si ffmpeg salió 0.
    """
    import subprocess

    inp = os.path.join(png_dir, pattern)
    cmd = [
        "ffmpeg", "-y", "-loglevel", "error",
        "-framerate", str(fps), "-i", inp,
        "-c:v", "libx264", "-pix_fmt", "yuv420p", mp4_path,
    ]
    try:
        r = subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
        if r.returncode == 0:
            print(f"[REC] MP4 -> {mp4_path}", flush=True)
            return True
        print(f"[REC] ffmpeg falló ({r.returncode}): {r.stderr.decode(errors='ignore')[:200]}", flush=True)
        return False
    except FileNotFoundError:
        print("[REC] ffmpeg no está en PATH; deja los PNG sin convertir", flush=True)
        return False
    except Exception as e:
        print(f"[REC] error al hacer MP4: {e}", flush=True)
        return False
