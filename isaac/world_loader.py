#!/usr/bin/env python3
# world_loader.py — carga el laberinto Gazebo en Isaac Sim 4.5 por el patrón PROBADO
# de REFERENCIA USD + colisión de malla "none" (NO por URDF; ver KEY LESSON del port).
#
# Dos cosas:
#   1) convierte las 5 mallas STL del laberinto a USD con omni.kit.asset_converter
#      (cacheado en isaac/assets/worlds/usd/<name>.usd; se salta si ya existe).
#   2) load_laberinto(): referencia cada USD bajo /World/Maze/<name>, escala 0.001,
#      des-instancia, aplica CollisionAPI + MeshCollisionAPI("none") + fricción a cada
#      Mesh (igual que el bloque EXTRA_USDS de scene_mecanum.py) EXCEPTO 'lineas'
#      (solo visual), colorea PLANO igual que Gazebo, y auto-centra en xy + piso a z=0.
#
# También expone load_vacio() = solo un plano de piso (AddGroundPlaneCommand).
#
# Sin efectos colaterales en import: todo el trabajo Isaac ocurre dentro de funciones,
# y los imports pesados de Isaac (omni.*, pxr) van dentro de las funciones, para poder
# `python3 -m py_compile` con el python del sistema (los del sistema sí: os/asyncio).
#
# APIs Isaac usadas (citadas en cada uso):
#  - omni.kit.asset_converter.get_instance().create_converter_task / AssetConverterContext
#      -> isaacsim/standalone_examples/api/omni.kit.asset_converter/asset_usd_converter.py
#      -> extscache/omni.kit.asset_converter-2.8.3/omni/kit/asset_converter/impl/{extension,task_manager,context}.py
#  - add_reference_to_stage, compute_aabb, create_bbox_cache, AddGroundPlaneCommand,
#    SetInstanceable(False), CollisionAPI/MeshCollisionAPI, MaterialBindingAPI
#      -> scene_mecanum.py (bloques 'laberinto'/EXTRA_USDS + groundPlane), reutilizados aquí.

import os

HERE = os.path.dirname(os.path.abspath(__file__))

# STLs fuente (proyecto Gazebo, escala 0.001 igual que model.sdf de laberinto_real).
_MESH_DIR = "/home/opyntorr/agv_uav_project_jetauto/src/mi_proyecto_sim/models/laberinto_real/meshes"
_USD_CACHE = os.path.join(HERE, "assets", "worlds", "usd")

# Escala de las mallas (model.sdf: <scale>0.001 0.001 0.001</scale>).
MAZE_SCALE = 0.001

# Las 5 piezas del laberinto. 'collision' False = SOLO visual (lineas, como en model.sdf,
# que NO define <collision> para las líneas). 'offset' = pose del <visual> en model.sdf
# (las líneas llevan 0 -0.02027 0; ya en metros, NO se re-escala). 'color' = color PLANO
# = Gazebo (CRÍTICO para el stitching por color del dron; ver model.sdf de laberinto_real).
#   piso   diffuse 0.08 0.08 0.09
#   paredes diffuse 0.55 0.38 0.20
#   lineas diffuse 0.03 0.28 0.70   (solo visual, offset y=-0.02027)
#   jaula  diffuse 0.85 0.85 0.85   (la jaula alta ~4.6 m; jaula.stl ~3.3 MB)
#   cajas  diffuse 1.0 0.85 0.0
_MAZE_PIECES = [
    {"name": "piso", "stl": "piso.stl", "collision": True,
     "color": (0.08, 0.08, 0.09), "offset": (0.0, 0.0, 0.0)},
    {"name": "paredes", "stl": "paredes.stl", "collision": True,
     "color": (0.55, 0.38, 0.20), "offset": (0.0, 0.0, 0.0)},
    {"name": "lineas", "stl": "lineas.stl", "collision": False,
     "color": (0.03, 0.28, 0.70), "offset": (0.0, -0.02027, 0.0)},
    {"name": "jaula", "stl": "jaula.stl", "collision": True,
     "color": (0.85, 0.85, 0.85), "offset": (0.0, 0.0, 0.0)},
    {"name": "cajas", "stl": "cajasRecientes.stl", "collision": True,
     "color": (1.0, 0.85, 0.0), "offset": (0.0, 0.0, 0.0)},
]


# ---------------------------------------------------------------------------
# 1) Conversión STL -> USD (omni.kit.asset_converter), cacheada en disco.
# ---------------------------------------------------------------------------
def _convert_one_stl(in_stl, out_usd, simulation_app):
    """Convierte un STL a USD de forma síncrona pumpeando el loop async de kit.

    Patrón: create_converter_task() devuelve un AssetConverterFutureWrapper cuyo
    wait_until_finished() es CORUTINA (ver impl/task_manager.py). En un SimulationApp
    en marcha el event-loop de kit ya está corriendo, así que NO usamos
    run_until_complete (chocaría); en su lugar lanzamos la corutina con
    asyncio.ensure_future y avanzamos simulation_app.update() hasta is_finished(),
    tal como pide la guía del port. (Ref: asset_usd_converter.py usa la misma
    create_converter_task/wait_until_finished; aquí solo cambiamos el driver del loop.)
    """
    import asyncio

    import omni.kit.asset_converter as converter  # import local: evita choques (ver ejemplo)

    ctx = converter.AssetConverterContext()        # impl/context.py
    # STL no trae materiales -> los colores los ponemos NOSOTROS (flat, = Gazebo).
    ctx.ignore_materials = True
    ctx.ignore_animations = True
    ctx.ignore_camera = True
    ctx.ignore_light = True
    # NO crear /World como root: queremos que el defaultPrim del USD sea la malla, para
    # que add_reference_to_stage(...) la componga limpia bajo /World/Maze/<name>.
    ctx.create_world_as_default_root_prim = False
    # El STL es adimensional; aplicamos la escala 0.001 NOSOTROS al referenciar (igual
    # que Gazebo). Dejamos use_meter_as_world_unit en su default (False).

    instance = converter.get_instance()            # impl/extension.get_instance()
    if instance is None:
        raise RuntimeError("omni.kit.asset_converter no está habilitado "
                           "(enable_extension('omni.kit.asset_converter') antes de convertir)")

    def _progress(_p, _t):                          # firma del ejemplo asset_usd_converter.py
        return

    task = instance.create_converter_task(in_stl, out_usd, _progress, ctx)

    # Lanzar la corutina en el loop de kit y bombear updates hasta que termine.
    fut = asyncio.ensure_future(task.wait_until_finished())
    # tope de seguridad: jaula.stl (~3.3 MB) es la más pesada; 12000 updates ~ minutos.
    for _ in range(12000):
        if task.is_finished() or fut.done():
            break
        simulation_app.update()
    if not task.is_finished():
        raise RuntimeError(f"conversión STL->USD no terminó a tiempo: {in_stl}")

    # Éxito = OmniConverterStatus.OK (lo que devuelve wait_until_finished()/get_status();
    # ver impl/task_manager.py:26 y tests/test_asset_converter.py:179). Si por alguna
    # razón no podemos importar el enum, caemos al bool del future (= success).
    success = bool(fut.result())
    try:
        from omni.kit.asset_converter.native_bindings import OmniConverterStatus
        success = task.get_status() == OmniConverterStatus.OK
    except Exception:
        pass
    if not success:
        raise RuntimeError(f"falló la conversión {in_stl} -> {out_usd}: "
                           f"{task.get_error_message()}")
    return out_usd


def ensure_maze_usds(simulation_app, mesh_dir=_MESH_DIR, usd_cache=_USD_CACHE):
    """Asegura que existan los 5 USD del laberinto; convierte los que falten.

    Devuelve dict {name: ruta_usd}. Cacheado: si el .usd ya existe, NO reconvierte.
    """
    os.makedirs(usd_cache, exist_ok=True)
    out = {}
    for piece in _MAZE_PIECES:
        name = piece["name"]
        in_stl = os.path.join(mesh_dir, piece["stl"])
        out_usd = os.path.join(usd_cache, f"{name}.usd")
        if not os.path.exists(out_usd):
            if not os.path.exists(in_stl):
                raise FileNotFoundError(f"STL fuente no encontrado: {in_stl}")
            print(f"[WORLD] convirtiendo {piece['stl']} -> {os.path.basename(out_usd)} ...",
                  flush=True)
            _convert_one_stl(in_stl, out_usd, simulation_app)
            print(f"[WORLD]   listo {out_usd}", flush=True)
        else:
            print(f"[WORLD] USD en caché: {os.path.basename(out_usd)} (sin reconvertir)",
                  flush=True)
        out[name] = out_usd
    return out


# ---------------------------------------------------------------------------
# helper: material UsdPreviewSurface de color plano (para fidelidad de render).
# ---------------------------------------------------------------------------
def _make_flat_material(stage, mat_root, name, rgb):
    """Crea un UsdPreviewSurface mate de color rgb bajo mat_root/<name>_mat.

    Ref: patrón UsdShade estándar (mismo estilo que jetauto_materials.py usa para
    los materiales del robot). Roughness alto + metallic 0 = mate (= pbr de Gazebo).
    """
    from pxr import Gf, Sdf, UsdShade
    mpath = f"{mat_root}/{name}_mat"
    material = UsdShade.Material.Define(stage, mpath)
    shader = UsdShade.Shader.Define(stage, f"{mpath}/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.9)
    shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return material


# ---------------------------------------------------------------------------
# 2) load_laberinto: referencia los USD, colisión "none", color plano, auto-centra.
# ---------------------------------------------------------------------------
def load_laberinto(stage, simulation_app, mat_path="/physicsMaterial",
                   parent="/World/Maze", mesh_dir=_MESH_DIR, usd_cache=_USD_CACHE,
                   scale=MAZE_SCALE):
    """Carga el laberinto Gazebo en `stage` por referencia-USD + colisión de malla 'none'.

    Espejo del bloque EXTRA_USDS / 'laberinto' de scene_mecanum.py:
      * add_reference_to_stage cada pieza bajo parent/<name>
      * escala global `scale` (0.001) en cada pieza
      * SetInstanceable(False) (des-instanciar; el converter exporta instanciado/proto,
        invisible a Usd.PrimRange -> mismo truco que jetauto_materials.py)
      * por cada Mesh: CollisionAPI + MeshCollisionAPI('none') + bind material de
        fricción `mat_path`, EXCEPTO 'lineas' (solo visual, sin colisión)
      * color PLANO = Gazebo (DisplayColor + UsdPreviewSurface)
      * auto-centra en xy y baja el piso a z=0 (bbox sobre todo el laberinto)

    Requiere que /physicsScene y `mat_path` (UsdPhysics.MaterialAPI) ya existan
    (los crea scene_mecanum.py antes de llamar aquí).

    Devuelve (parent_prim_path:str, bbox:np.ndarray[xmin,ymin,zmin,xmax,ymax,zmax])
    del laberinto YA centrado.
    """
    from isaacsim.core.utils.bounds import compute_aabb, create_bbox_cache  # scene_mecanum.py
    from isaacsim.core.utils.stage import add_reference_to_stage             # scene_mecanum.py
    from pxr import Gf, Usd, UsdGeom, UsdPhysics, UsdShade                    # scene_mecanum.py

    # asegurar los USD (convierte si faltan)
    usds = ensure_maze_usds(simulation_app, mesh_dir=mesh_dir, usd_cache=usd_cache)

    # Xform raíz del laberinto + raíz para los materiales planos.
    UsdGeom.Xform.Define(stage, parent)
    mat_root = f"{parent}/Looks"
    UsdGeom.Scope.Define(stage, mat_root)

    # 1) referenciar cada pieza bajo parent/<name> con escala + offset (igual a model.sdf).
    piece_paths = {}
    for piece in _MAZE_PIECES:
        name = piece["name"]
        ppath = f"{parent}/{name}"
        UsdGeom.Xform.Define(stage, ppath)                       # contenedor (le ponemos scale/offset)
        # add_reference_to_stage(usd_path, prim_path) — firma de stage.py (scene_mecanum.py EXTRA_USDS).
        add_reference_to_stage(usd_path=usds[name], prim_path=f"{ppath}/ref")
        xf = UsdGeom.Xformable(stage.GetPrimAtPath(ppath))
        xf.ClearXformOpOrder()
        ox, oy, oz = piece["offset"]
        # translate (offset del visual de Gazebo, ya en metros) luego scale 0.001.
        xf.AddTranslateOp().Set(Gf.Vec3d(ox, oy, oz))
        xf.AddScaleOp().Set(Gf.Vec3f(scale, scale, scale))
        piece_paths[name] = ppath

    simulation_app.update()                                       # poblar geometría (para PrimRange/bbox)

    # 2) des-instanciar (el converter puede exportar proto instanciado, invisible a
    #    Usd.PrimRange). EXACTAMENTE como scene_mecanum.py: PrimRange SIMPLE (NO
    #    TraverseInstanceProxies; los instance-proxies son read-only y SetInstanceable
    #    fallaría sobre ellos). Tras apagar el flag, una segunda pasada SÍ ve los hijos.
    for q in Usd.PrimRange(stage.GetPrimAtPath(parent)):
        if q.IsInstanceable():
            q.SetInstanceable(False)
    simulation_app.update()

    # 3) por pieza: colisión 'none' (salvo lineas) + fricción + color plano.
    mat_prim = stage.GetPrimAtPath(mat_path)
    total_col = 0
    for piece in _MAZE_PIECES:
        name = piece["name"]
        ppath = piece_paths[name]
        flat = _make_flat_material(stage, mat_root, name, piece["color"])
        ncol = 0
        for q in Usd.PrimRange(stage.GetPrimAtPath(ppath)):
            if q.GetTypeName() != "Mesh":
                continue
            # color PLANO = Gazebo: DisplayColor (vista rápida) + UsdPreviewSurface (RTX).
            gprim = UsdGeom.Gprim(q)
            gprim.CreateDisplayColorAttr([Gf.Vec3f(*piece["color"])])
            UsdShade.MaterialBindingAPI.Apply(q).Bind(
                flat, bindingStrength=UsdShade.Tokens.weakerThanDescendants)
            # colisión de TRIÁNGULOS estática (igual que EXTRA_USDS) salvo 'lineas'.
            if piece["collision"]:
                UsdPhysics.CollisionAPI.Apply(q)
                UsdPhysics.MeshCollisionAPI.Apply(q).CreateApproximationAttr().Set("none")
                if mat_prim and mat_prim.IsValid():
                    UsdShade.MaterialBindingAPI.Apply(q).Bind(
                        UsdShade.Material(mat_prim),
                        bindingStrength=UsdShade.Tokens.weakerThanDescendants,
                        materialPurpose="physics")
                ncol += 1
        total_col += ncol
        print(f"[WORLD]   {name}: color={piece['color']} colisión 'none' en {ncol} mallas"
              + ("" if piece["collision"] else "  (SOLO visual)"), flush=True)

    # 4) ORIENTACIÓN + AUTO-CENTRAR. Las STL del laberinto NO vienen Z-up: Gazebo las
    #    paraba con el roll=π/2 del <include> (∘ yaw=π/2 del <model>). El asset_converter
    #    NO añade ese flip Y-up→Z-up (sí lo hacía el importador URDF), así que SIN esta
    #    rotación el piso queda VERTICAL y el robot cae al vacío. Aplicamos Rx90·Rz90 al
    #    Xform raíz y luego auto-centramos en xy + piso a z=0 sobre la geometría YA rotada.
    xf_root = UsdGeom.Xformable(stage.GetPrimAtPath(parent))
    xf_root.ClearXformOpOrder()
    _rotq = (Gf.Rotation(Gf.Vec3d(1, 0, 0), 90) * Gf.Rotation(Gf.Vec3d(0, 0, 1), 90)).GetQuat()
    _Mrot = Gf.Matrix4d().SetRotate(Gf.Quatd(_rotq))
    _root_op = xf_root.AddTransformOp()
    _root_op.Set(_Mrot)
    simulation_app.update()
    bb = compute_aabb(create_bbox_cache(), parent, include_children=True)  # [xmin..zmax] YA rotado
    cx, cy = 0.5 * (bb[0] + bb[3]), 0.5 * (bb[1] + bb[4])
    _Mfull = Gf.Matrix4d(_Mrot)
    _Mfull.SetTranslateOnly(Gf.Vec3d(-cx, -cy, -bb[2]))   # rotación + centrado xy + piso z=0
    _root_op.Set(_Mfull)
    simulation_app.update()

    # bbox YA centrado (para que el llamador sepa el footprint final).
    bb2 = compute_aabb(create_bbox_cache(), parent, include_children=True)
    print(f"[WORLD] laberinto centrado: xy=({bb2[3]-bb2[0]:.2f}x{bb2[4]-bb2[1]:.2f}) "
          f"alto={bb2[5]-bb2[2]:.2f}  piso~z=0  ({total_col} mallas con colisión 'none')",
          flush=True)
    return parent, bb2


# ---------------------------------------------------------------------------
# load_vacio: mundo vacío = solo un plano de piso con fricción.
# ---------------------------------------------------------------------------
def load_vacio(stage, mat_path="/physicsMaterial", plane_path="/groundPlane",
               size=100.0):
    """Mundo vacío: un plano de piso en z=0 (AddGroundPlaneCommand), con fricción.

    Reutiliza EXACTAMENTE el patrón del 'plano de piso propio' de scene_mecanum.py:
    omni.kit.commands.execute('AddGroundPlaneCommand', stage, planePath, axis='Z',
    size, position, color) y luego enlaza el material de fricción al CollisionPlane.
    (AddGroundPlaneCommand: extsPhysics/omni.physx.commands/__init__.py:248.)

    Devuelve la ruta del plano creado.
    """
    import omni.kit.commands
    from pxr import Gf, UsdShade

    omni.kit.commands.execute(
        "AddGroundPlaneCommand", stage=stage, planePath=plane_path, axis="Z",
        size=size, position=Gf.Vec3f(0, 0, 0.0), color=Gf.Vec3f(0.5),
    )
    gp = stage.GetPrimAtPath(f"{plane_path}/CollisionPlane")
    if gp and gp.IsValid():
        mat_prim = stage.GetPrimAtPath(mat_path)
        if mat_prim and mat_prim.IsValid():
            UsdShade.MaterialBindingAPI.Apply(gp).Bind(
                UsdShade.Material(mat_prim),
                bindingStrength=UsdShade.Tokens.weakerThanDescendants,
                materialPurpose="physics")
    print(f"[WORLD] mundo vacío: plano de piso en {plane_path} (z=0)", flush=True)
    return plane_path
