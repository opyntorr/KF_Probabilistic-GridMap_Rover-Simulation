#!/usr/bin/env python3
# JetAuto mecanum en Isaac Sim 4.5 — MOVIMIENTO POR FÍSICA (no teleport).
#
# A diferencia del port kinemático (../../agv_uav_project_jetauto/isaac), aquí el
# robot se mueve por FRICCIÓN: las 4 ruedas mecanum giran (drive de velocidad) y los
# 12 rodillos pasivos por rueda (48 en total, esferas) transmiten la tracción contra
# el suelo. El strafe holonómico emerge de la física (igual que en Gazebo). Más
# sim-to-real: encoders REALES desde /joint_states, /odom = pose física del chasis.
#
#   /cmd_vel --(IK mecanum)--> velocidades de las 4 ruedas --(fricción)--> el robot avanza/gira/strafe
#
# Correr:   source isaac/isaac_env.sh
#           $ISAACSIM/python.sh isaac/scene_mecanum.py            # Kalman (mundo vacío)
#           $ISAACSIM/python.sh isaac/scene_mecanum.py --gridmap  # GridMap (cuarto + RTX lidar)
#           (--headless para sin ventana)
import argparse
import math
import os

from isaacsim import SimulationApp

HERE = os.path.dirname(os.path.abspath(__file__))
URDF_PATH = os.path.join(HERE, "assets", "jetauto_mecanum.urdf")
LIDAR_CONFIG = "RPLIDAR_S2E"

# --- USDs extra que se cargan POR DEFECTO (entornos/props) -----------------
# Se añaden como referencias al stage. Ajusta "at" (x,y,z) para colocarlos sin
# que pisen al robot. Vacía la lista (o pasa --no-extra) para no cargar ninguno.
# "collision": True añade colisión de malla (triángulos, estática) a las mallas del
# asset, porque estos USD vienen SIN colisión (el warehouse trae 0; el grid solo un
# plano de piso) -> sin esto el robot los atraviesa. OJO: con colisión, un asset
# centrado en el origen atraparía al robot -> usa "at" para apartarlo (p.ej. el
# warehouse a (0,8,0), fuera del círculo de la tarea).
# OJO (--gridmap): el lidar VERÁ estas geometrías -> el mapa las incluirá pero
# validate_map.py NO las conoce -> la exactitud bajará. Para validar usa --no-extra.
_ASSETS = "/home/opyntorr/isaacsim_assets/Assets/Isaac/4.5/Isaac/Environments"
EXTRA_USDS = [
    {"path": f"{_ASSETS}/Grid/gridroom_curved.usd", "at": (0.0, 0.0, 0.0), "collision": True},
    {"path": f"{_ASSETS}/Modular_Warehouse/Props/warehouse_h10m_center.usd", "at": (0.0, 0.0, 0.0), "collision": True, "spawn_on": True, "spawn_z": 1.1},
]

# cinemática mecanum (misma que rover_model/sensor_models)
WHEEL_R = 0.04825
WHEEL_K = 0.20
WHEEL_KD = 600.0          # damping del drive de velocidad de rueda (tune)
WHEELS = ['front_left_wheel_joint', 'front_right_wheel_joint',
          'back_left_wheel_joint', 'back_right_wheel_joint']

parser = argparse.ArgumentParser()
parser.add_argument("--headless", action="store_true")
parser.add_argument("--gridmap", action="store_true", help="añade cuarto 6x6 + RTX lidar /scan")
parser.add_argument("--lidar", action="store_true", help="RTX lidar /scan SIN el cuarto 6x6 (mundos como laberinto)")
# --- módulos del port (drone, aruco3d, sensors, cameras, perf_hud) ---
parser.add_argument("--aruco", action="store_true", help="marcadores ArUco 3D + cubo objetivo")
parser.add_argument("--drone", action="store_true", help="dron Tello kinemático en /drone1/cmd_vel")
parser.add_argument("--imu", action="store_true", help="IMU en /imu/data_raw")
parser.add_argument("--camera", action="store_true", help="cámara RGBD Astra en /cam_1/*")
parser.add_argument("--cameras", action="store_true", help="rig multi-cámara wheel/scene/chase[/top]")
parser.add_argument("--viewports", action="store_true", help="un viewport GUI por cámara (no --headless)")
parser.add_argument("--rec_cams", action="store_true", help="graba el rig multicámara a disco")
parser.add_argument("--cam_res", default="1080p", help="preset resolución rig (720p/1080p/1440p/4k)")
parser.add_argument("--perf", action="store_true", help="HUD stats + [PERF] fps/rtf/gpu/ram")
# --- knobs de física del barrido de serpenteo (default = no tocar) ---
parser.add_argument("--solver-pos", type=int, default=-1)
parser.add_argument("--solver-vel", type=int, default=-1)
parser.add_argument("--phys-hz", type=float, default=-1.0)
parser.add_argument("--contact-offset", type=float, default=-1.0)
parser.add_argument("--rest-offset", type=float, default=-1.0)
parser.add_argument("--maxdepen", type=float, default=-1.0)
parser.add_argument("--roller-shape", default="sphere")
parser.add_argument("--apply-perstep", action="store_true")
parser.add_argument("--no-extra", action="store_true", help="no cargar los USDs de EXTRA_USDS")
parser.add_argument("--ground", action="store_true", help="forzar el plano de piso propio aunque haya escenario")
parser.add_argument("--no-ground", action="store_true", help="no crear plano de piso propio (usa el piso del escenario)")
parser.add_argument("--urdf", default=URDF_PATH)
parser.add_argument("--x", type=float, default=None)
parser.add_argument("--y", type=float, default=None)
parser.add_argument("--z", type=float, default=0.05)
parser.add_argument("--yaw", type=float, default=None)
parser.add_argument("--friction", type=float, default=1.0, help="μ estática=dinámica de ruedas/suelo")
parser.add_argument("--world", default="", help="mundo a cargar: 'laberinto' (URDF estático con mallas STL) o ''")
parser.add_argument("--record", action="store_true",
                    help="graba una cámara cenital a mp4 durante la rutina (Kelly)")
parser.add_argument("--record_secs", type=float, default=90.0,
                    help="segundos a grabar tras iniciar el movimiento (= duration_s de Kelly)")
parser.add_argument("--record_fps", type=float, default=30.0)
parser.add_argument("--cam_z", type=float, default=3.0, help="altura de la cámara cenital sobre (0,0)")
parser.add_argument("--rec_out", default=None, help="ruta del .mp4 (default: isaac/topdown_<ts>.mp4)")
args, _ = parser.parse_known_args()

# spawn por defecto segun la tarea (igual que el port kinematico)
_spawn_corner = args.gridmap or args.world == "laberinto"
if args.x is None:
    args.x = -1.0 if _spawn_corner else 0.0
if args.y is None:
    args.y = -1.0 if _spawn_corner else 0.0
if args.yaw is None:
    args.yaw = math.pi / 2 if _spawn_corner else 0.0

simulation_app = SimulationApp({"renderer": "RaytracedLighting", "headless": args.headless})

import numpy as np
import omni.graph.core as og
import omni.kit.commands
import omni.timeline
import usdrt.Sdf
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.bounds import compute_aabb, create_bbox_cache
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.types import ArticulationAction
from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade

enable_extension("isaacsim.ros2.bridge")
enable_extension("isaacsim.asset.importer.urdf")
if args.gridmap or args.lidar:
    enable_extension("isaacsim.sensors.rtx")
if args.imu:
    enable_extension("isaacsim.sensors.physics")    # sensor IMU (sensors.add_imu)
if args.record or args.cameras or args.rec_cams or args.viewports:
    enable_extension("isaacsim.sensors.camera")
if args.world == "laberinto" or args.drone:
    enable_extension("omni.kit.asset_converter")   # STL/DAE->USD (laberinto / malla del dron)
simulation_app.update()

# --- 1) Importar el URDF (con rodillos) ------------------------------------
_, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
import_config.merge_fixed_joints = False
import_config.convex_decomp = False
import_config.import_inertia_tensor = True
import_config.fix_base = False                 # base libre: la mueve la física
import_config.distance_scale = 1.0
import_config.density = 0.0                     # usa las inercias del URDF
_, prim_path = omni.kit.commands.execute(
    "URDFParseAndImportFile", urdf_path=args.urdf,
    import_config=import_config, get_articulation_root=True,
)
print(f"[FIS] URDF importado en: {prim_path}")

stage = omni.usd.get_context().get_stage()

# --- 1b) Materiales: verde -> aluminio anodizado, negro -> plástico mate -------
# (el importador de URDF deja el robot blanco; ver jetauto_materials.py)
from jetauto_materials import apply_jetauto_materials
apply_jetauto_materials(stage, prim_path)

# --- cámara cenital opcional (para grabar la rutina desde arriba) ---
if args.record:
    try:
        _cam = UsdGeom.Camera.Define(stage, Sdf.Path("/World/TopCam"))
        _cam.CreateFocalLengthAttr(15.0)            # gran angular: abarca el área a ~3 m
        _xf = UsdGeom.Xformable(_cam.GetPrim())
        _xf.ClearXformOpOrder()
        _xf.AddTransformOp().Set(
            Gf.Matrix4d().SetLookAt(Gf.Vec3d(0, 0, args.cam_z),
                                    Gf.Vec3d(0, 0, 0), Gf.Vec3d(0, 1, 0)).GetInverse())
        print(f"[REC] cámara cenital /World/TopCam en (0,0,{args.cam_z}) mirando abajo", flush=True)
    except Exception as _e:
        print(f"[REC] no pude crear la cámara cenital ({_e}); grabación off", flush=True)
        args.record = False

# --- 2) Física CON gravedad + fricción -------------------------------------
scene = UsdPhysics.Scene.Define(stage, Sdf.Path("/physicsScene"))
scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
scene.CreateGravityMagnitudeAttr().Set(9.81)
PhysxSchema.PhysxSceneAPI.Apply(stage.GetPrimAtPath("/physicsScene"))
physx = PhysxSchema.PhysxSceneAPI.Get(stage, "/physicsScene")
physx.CreateEnableCCDAttr(True)
physx.CreateEnableStabilizationAttr(True)
physx.CreateEnableGPUDynamicsAttr(False)
physx.CreateBroadphaseTypeAttr("MBP")
physx.CreateSolverTypeAttr("TGS")
physx.CreateTimeStepsPerSecondAttr(args.phys_hz if args.phys_hz > 0 else 120.0)  # --phys-hz (sweep)

# material de fricción (suelo + rodillos)
mat_path = "/physicsMaterial"
UsdShade.Material.Define(stage, mat_path)
mat = UsdPhysics.MaterialAPI.Apply(stage.GetPrimAtPath(mat_path))
mat.CreateStaticFrictionAttr(args.friction)
mat.CreateDynamicFrictionAttr(args.friction)
mat.CreateRestitutionAttr(0.0)
print(f"[FIS] fricción μ={args.friction}", flush=True)

# --- mundo: laberinto (mallas STL del proyecto Gazebo via REFERENCIA USD) ----
# Patrón PROBADO (= EXTRA_USDS): STL->USD (asset_converter) + colisión de malla "none"
# + colores planos = Gazebo (clave para el stitching del dron). NO por URDF (un import
# URDF dejaba un convexHull combinado y el robot caía; ver world_loader.py).
if args.world == "laberinto":
    from world_loader import load_laberinto
    _maze_path, _maze_bb = load_laberinto(stage, simulation_app, mat_path=mat_path)

# Plano de piso PROPIO: solo cuando NO hay escenario importado (si hay, el robot se
# apoya en el piso del escenario; un plano propio en z=0 lo pisaría/enterraría).
_load_extras = (not args.no_extra) and len(EXTRA_USDS) > 0 and not args.world
_make_ground = args.ground or (not args.no_ground and not _load_extras and not args.world)
if _make_ground:
    omni.kit.commands.execute(
        "AddGroundPlaneCommand", stage=stage, planePath="/groundPlane", axis="Z",
        size=100.0, position=Gf.Vec3f(0, 0, 0.0), color=Gf.Vec3f(0.5),
    )
    gp = stage.GetPrimAtPath("/groundPlane/CollisionPlane")
    if gp and gp.IsValid():
        UsdShade.MaterialBindingAPI.Apply(gp).Bind(
            UsdShade.Material(stage.GetPrimAtPath(mat_path)),
            bindingStrength=UsdShade.Tokens.weakerThanDescendants, materialPurpose="physics")
    print("[FIS] plano de piso propio en z=0")
else:
    print("[FIS] sin plano propio: el robot se apoya en el piso del escenario")

light = UsdLux.DistantLight.Define(stage, Sdf.Path("/DistantLight"))
light.CreateIntensityAttr(1000)

# Dome light: entorno uniforme para que los materiales METÁLICOS (aluminio anodizado)
# reflejen algo y no se vean negros. Sin esto, con solo el DistantLight el metal sale oscuro.
dome = UsdLux.DomeLight.Define(stage, Sdf.Path("/DomeLight"))
dome.CreateIntensityAttr(700)
dome.CreateColorAttr(Gf.Vec3f(0.9, 0.93, 1.0))

# --- 2c) USDs extra (entornos/props) cargados por defecto ------------------
# Muchos assets tienen el pivote en una ESQUINA (la geometría arranca en el origen),
# así que por defecto se CENTRAN en xy (por su bbox) y se bajan para que su piso
# quede en z=0. "at" desplaza ADEMÁS del centrado; "center": False usa "at" tal cual.
spawn_on_pose = None   # (x,y,z_top) si algún asset tiene "spawn_on": el robot va encima
if _load_extras:
    for i, e in enumerate(EXTRA_USDS):
        UsdGeom.Xform.Define(stage, f"/World/Extra_{i}")
        add_reference_to_stage(usd_path=e["path"], prim_path=f"/World/Extra_{i}/ref")
    simulation_app.update()                       # poblar geometría para el bbox
    bbcache = create_bbox_cache()
    for i, e in enumerate(EXTRA_USDS):
        p = f"/World/Extra_{i}"
        ax, ay, az = e.get("at", (0.0, 0.0, 0.0))
        if e.get("center", True):
            bb = compute_aabb(bbcache, p, include_children=True)  # [xmin,ymin,zmin,xmax,ymax,zmax]
            tx, ty, tz = ax - 0.5 * (bb[0] + bb[3]), ay - 0.5 * (bb[1] + bb[4]), az - bb[2]
        else:
            tx, ty, tz = ax, ay, az
        UsdGeom.Xformable(stage.GetPrimAtPath(p)).AddTranslateOp().Set(Gf.Vec3d(tx, ty, tz))
        # spawn_on: el robot irá ENCIMA de este asset (xy = su centro). La altura es
        # "spawn_z" (sobre el piso del asset) si se da; si no, el tope (bbox).
        if e.get("spawn_on") and e.get("center", True):
            spawn_on_pose = (ax, ay, az + e.get("spawn_z", bb[5] - bb[2]))
        # colisión estática (malla de triángulos) + fricción, porque los assets
        # vienen sin colisión -> el robot los atravesaría.
        ncol = 0
        if e.get("collision", True):
            for q in Usd.PrimRange(stage.GetPrimAtPath(p)):
                if q.GetTypeName() == "Mesh":
                    UsdPhysics.CollisionAPI.Apply(q)
                    UsdPhysics.MeshCollisionAPI.Apply(q).CreateApproximationAttr().Set("none")
                    UsdShade.MaterialBindingAPI.Apply(q).Bind(
                        UsdShade.Material(stage.GetPrimAtPath(mat_path)),
                        bindingStrength=UsdShade.Tokens.weakerThanDescendants, materialPurpose="physics")
                    ncol += 1
        _sz = (f"size=({bb[3]-bb[0]:.1f}x{bb[4]-bb[1]:.1f}x{bb[5]-bb[2]:.1f})"
               if e.get("center", True) else "")
        print(f"[FIS] USD extra: {os.path.basename(e['path'])} {_sz} -> trasl "
              f"({tx:.2f},{ty:.2f},{tz:.2f})  colisión en {ncol} mallas", flush=True)

# fricción a TODOS los colliders de rodillo (match por RUTA: el collider puede ser
# un sub-prim cuyo nombre no contiene "roller" pero su ruta sí, p.ej. .../roller_0_link/collisions)
n_roller = 0
_dbgcol = []
for p in stage.Traverse():
    if p.HasAPI(UsdPhysics.CollisionAPI):
        pth = p.GetPath().pathString
        if len(_dbgcol) < 6:
            _dbgcol.append(pth)
        if "roller" in pth:
            UsdShade.MaterialBindingAPI.Apply(p).Bind(
                UsdShade.Material(stage.GetPrimAtPath(mat_path)),
                bindingStrength=UsdShade.Tokens.weakerThanDescendants, materialPurpose="physics")
            if args.contact_offset >= 0.0 or args.rest_offset >= 0.0:
                _capi = PhysxSchema.PhysxCollisionAPI.Apply(p)   # --contact-offset/--rest-offset (sweep)
                if args.contact_offset >= 0.0:
                    _capi.CreateContactOffsetAttr(args.contact_offset)
                if args.rest_offset >= 0.0:
                    _capi.CreateRestOffsetAttr(args.rest_offset)
            n_roller += 1
print(f"[FIS] fricción aplicada a {n_roller} colliders de rodillo")
if n_roller == 0:
    print(f"[FIS] DEBUG colliders (muestra): {_dbgcol}")

# --- 2b) (gridmap) cuarto 6x6 + cajas (geometría = tareas_room.sdf) ---------
if args.gridmap:
    ROOM = {
        "wall_n": ((-1.5, 2.25, 0.3), (6.0, 0.1, 0.6)),
        "wall_s": ((-1.5, -3.75, 0.3), (6.0, 0.1, 0.6)),
        "wall_e": ((1.5, -0.75, 0.3), (0.1, 6.0, 0.6)),
        "wall_w": ((-4.5, -0.75, 0.3), (0.1, 6.0, 0.6)),
        "box1": ((0.4, 1.1, 0.3), (0.6, 0.6, 0.6)),
        "box2": ((-3.5, 0.8, 0.3), (0.6, 0.6, 0.6)),
        "box3": ((-0.3, -3.0, 0.3), (0.9, 0.6, 0.6)),
        "box4": ((0.6, -2.0, 0.3), (0.5, 1.4, 0.6)),
    }
    UsdGeom.Xform.Define(stage, "/Room")
    for nm, (c, s) in ROOM.items():
        xf = UsdGeom.Xform.Define(stage, f"/Room/{nm}")
        xf.AddTranslateOp().Set(Gf.Vec3d(*c))
        xf.AddScaleOp().Set(Gf.Vec3f(*s))
        cube = UsdGeom.Cube.Define(stage, f"/Room/{nm}/geo")
        cube.GetSizeAttr().Set(1.0)
        cube.CreateDisplayColorAttr([Gf.Vec3f(0.6, 0.6, 0.65)])
    print(f"[FIS] cuarto con {len(ROOM)} cuerpos")

# --- ArUcos 3D (tabla blanca 4mm + negro extruido 0.6mm) + cubo objetivo ----
if args.aruco:
    from aruco3d import make_goal_cube, place_robot_marker
    make_goal_cube(stage, "/World/CuboAruco", pose=(1.5, 0.0, 0.075), mat_path=mat_path)
    _robot_root = "/" + str(prim_path).strip("/").split("/")[0]
    place_robot_marker(stage, _robot_root + "/base_link", marker_id=4)
    print("[FIS] ArUcos 3D: cubo objetivo + marcador ID4 en el techo del robot", flush=True)

# --- 3) Grafo ROS2: clock + odom(GT físico) + TF + joint_states ------------
og.Controller.edit(
    {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
    {
        og.Controller.Keys.CREATE_NODES: [
            ("OnTick", "omni.graph.action.OnPlaybackTick"),
            ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
            ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ("ComputeOdom", "isaacsim.core.nodes.IsaacComputeOdometry"),
            ("PublishOdom", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
            ("PublishRawTF", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
            ("PublishJoint", "isaacsim.ros2.bridge.ROS2PublishJointState"),
        ],
        og.Controller.Keys.CONNECT: [
            ("OnTick.outputs:tick", "PublishClock.inputs:execIn"),
            ("OnTick.outputs:tick", "ComputeOdom.inputs:execIn"),
            ("OnTick.outputs:tick", "PublishOdom.inputs:execIn"),
            ("OnTick.outputs:tick", "PublishRawTF.inputs:execIn"),
            ("OnTick.outputs:tick", "PublishJoint.inputs:execIn"),
            ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
            ("ReadSimTime.outputs:simulationTime", "PublishOdom.inputs:timeStamp"),
            ("ReadSimTime.outputs:simulationTime", "PublishRawTF.inputs:timeStamp"),
            ("ReadSimTime.outputs:simulationTime", "PublishJoint.inputs:timeStamp"),
            ("ComputeOdom.outputs:position", "PublishOdom.inputs:position"),
            ("ComputeOdom.outputs:orientation", "PublishOdom.inputs:orientation"),
            ("ComputeOdom.outputs:linearVelocity", "PublishOdom.inputs:linearVelocity"),
            ("ComputeOdom.outputs:angularVelocity", "PublishOdom.inputs:angularVelocity"),
            ("ComputeOdom.outputs:position", "PublishRawTF.inputs:translation"),
            ("ComputeOdom.outputs:orientation", "PublishRawTF.inputs:rotation"),
        ],
        og.Controller.Keys.SET_VALUES: [
            ("PublishClock.inputs:topicName", "clock"),
            ("ComputeOdom.inputs:chassisPrim", [usdrt.Sdf.Path(prim_path)]),
            ("PublishOdom.inputs:topicName", "odom"),
            ("PublishOdom.inputs:odomFrameId", "odom"),
            ("PublishOdom.inputs:chassisFrameId", "base_footprint"),
            ("PublishRawTF.inputs:topicName", "/tf"),
            ("PublishRawTF.inputs:parentFrameId", "odom"),
            ("PublishRawTF.inputs:childFrameId", "base_footprint"),
            ("PublishJoint.inputs:topicName", "joint_states"),
            ("PublishJoint.inputs:targetPrim", [usdrt.Sdf.Path(prim_path)]),
        ],
    },
)

# --- sensores extra (paridad con Gazebo): IMU /imu/data_raw + RGB-D /cam_1 --
if args.imu or args.camera:
    from sensors import add_imu, add_rgbd_camera
    _rroot = "/" + str(prim_path).strip("/").split("/")[0]   # /jetauto (raíz: busca en TODO el robot)
    if args.imu:
        add_imu(stage, simulation_app, base_link_prim=_rroot,
                topic="/imu/data_raw", frame_id="imu_link")
    if args.camera:
        add_rgbd_camera(stage, simulation_app, base_link_prim=_rroot, topic_base="/cam_1",
                        resolution=(640, 480), frame_id="cam_1_optical_frame")

# --- 3b) RTX lidar 2D en lidar_frame -> /scan (gridmap o --lidar) ----------
if args.gridmap or args.lidar:
    import omni.replicator.core as rep
    lidar_parent = prim_path
    for p in stage.Traverse():
        if p.GetName() == "lidar_frame":
            lidar_parent = p.GetPath().pathString
            break
    _, lidar = omni.kit.commands.execute(
        "IsaacSensorCreateRtxLidar", path="rtx_lidar", parent=lidar_parent,
        config=LIDAR_CONFIG, translation=(0.0, 0.0, 0.0),
        orientation=Gf.Quatd(1.0, 0.0, 0.0, 0.0))
    rp = rep.create.render_product(lidar.GetPath(), [1, 1], name="JetautoLidar")
    w = rep.writers.get("RtxLidar" + "ROS2PublishLaserScan")
    w.initialize(topicName="scan", frameId="lidar_frame")
    w.attach([rp])
    print(f"[FIS] RTX lidar ({LIDAR_CONFIG}) -> /scan en {lidar_parent}")

# --- 4) ROS2: suscriptor de /cmd_vel ---------------------------------------
import rclpy
from geometry_msgs.msg import Twist

rclpy.init()
node = rclpy.create_node("isaac_mecanum_drive")
cmd = {"vx": 0.0, "vy": 0.0, "wz": 0.0}
node.create_subscription(Twist, "/cmd_vel",
                         lambda m: cmd.update(vx=m.linear.x, vy=m.linear.y, wz=m.angular.z), 10)

# --- 5) Arrancar física + articulación + drives de rueda -------------------
timeline = omni.timeline.get_timeline_interface()
timeline.play()
simulation_app.update()

art = SingleArticulation(prim_path)
art.initialize()
# knobs anti-serpenteo (sweep): iteraciones del solver de la articulación + maxDepen
if args.solver_pos >= 0:
    art.set_solver_position_iteration_count(args.solver_pos)
if args.solver_vel >= 0:
    art.set_solver_velocity_iteration_count(args.solver_vel)
if args.maxdepen >= 0.0:
    PhysxSchema.PhysxRigidBodyAPI.Apply(stage.GetPrimAtPath(prim_path)).CreateMaxDepenetrationVelocityAttr(args.maxdepen)


def _quat_yaw(yaw):
    return np.array([math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0)])


if spawn_on_pose is not None:
    # La plataforma real está a una altura que no sabemos a priori (el bbox del
    # warehouse da el TECHO, no la plataforma). La hallamos con un raycast hacia
    # abajo en (x,y) del centro y soltamos el robot ~12 cm por encima para que caiga
    # limpio (si nace por debajo de la superficie, las ruedas quedan enterradas).
    rx, ry, zhint = spawn_on_pose
    art.set_world_pose(position=np.array([rx, ry, 60.0]), orientation=_quat_yaw(args.yaw))
    for _ in range(3):
        simulation_app.update()
    surf = None
    try:
        import carb
        from omni.physx import get_physx_scene_query_interface
        # origen del rayo POR DEBAJO del techo (zhint+2) para no engancharlo:
        hit = get_physx_scene_query_interface().raycast_closest(
            carb.Float3(rx, ry, zhint + 2.0), carb.Float3(0.0, 0.0, -1.0), 12.0)
        if hit and hit.get("hit"):
            zh = float(hit["position"][2])
            if 0.0 < zh < zhint + 1.5:          # descarta golpes al techo
                surf = zh
    except Exception as ex:
        print(f"[FIS] raycast de plataforma falló ({ex}); uso spawn_z", flush=True)
    if surf is not None:
        sx, sy, sz = rx, ry, surf + 0.12
        print(f"[FIS] plataforma detectada a z={surf:.3f}; suelto el robot desde z={sz:.2f}", flush=True)
    else:
        sx, sy, sz = rx, ry, zhint + 0.12
        print(f"[FIS] sin raycast: robot desde z={sz:.2f} (spawn_z={zhint:.2f})", flush=True)
else:
    sx, sy, sz = args.x, args.y, args.z
art.set_world_pose(position=np.array([sx, sy, sz]), orientation=_quat_yaw(args.yaw))

# índices DOF de las 4 ruedas
dof_names = list(art.dof_names)
wheel_idx = [dof_names.index(w) for w in WHEELS]
print(f"[FIS] DOFs={len(dof_names)}  ruedas en idx {wheel_idx}")

# gains: ruedas = drive de velocidad (kp=0, kd=WHEEL_KD); todo lo demás libre
ndof = len(dof_names)
kps = np.zeros(ndof)
kds = np.zeros(ndof)
for i in wheel_idx:
    kds[i] = WHEEL_KD
art.get_articulation_controller().set_gains(kps=kps, kds=kds)


def ik(vx, vy, wz):
    return np.array([
        (vx - vy - WHEEL_K * wz) / WHEEL_R,   # fl
        (vx + vy + WHEEL_K * wz) / WHEEL_R,   # fr
        (vx + vy - WHEEL_K * wz) / WHEEL_R,   # rl
        (vx - vy + WHEEL_K * wz) / WHEEL_R,   # rr
    ])


# asentar el robot unos pasos antes de aceptar comandos
for _ in range(60):
    simulation_app.update()
_rz = float(art.get_world_pose()[0][2])
print(f"[FIS] robot asentado en z={_rz:.3f} (contacto rueda ~{_rz - 0.00317:.3f})", flush=True)

# --- recorder cenital opcional (no-fatal: si algo falla, la sim sigue) ----------
rec = None
if args.record:
    try:
        import datetime
        from isaacsim.sensors.camera import Camera
        RW, RH = 1280, 720
        _topcam = Camera(prim_path="/World/TopCam", resolution=(RW, RH))
        _topcam.initialize()
        for _ in range(5):
            simulation_app.update()                  # calentar el render de la cámara
        _out = args.rec_out or os.path.join(
            HERE, f"topdown_{datetime.datetime.now():%Y%m%d_%H%M%S}.mp4")
        rec = {"cam": _topcam, "out": _out, "ff": None, "on": False, "done": False,
               "t0": 0.0, "next": 0.0, "W": RW, "H": RH}
        print(f"[REC] listo; grabaré {args.record_secs:.0f}s al iniciar el movimiento -> {_out}",
              flush=True)
    except Exception as _e:
        print(f"[REC] grabación deshabilitada ({_e})", flush=True)
        rec = None


# --- dron Tello kinemático opcional (obedece /drone1/cmd_vel) -----------------
drone = None
if args.drone:
    from drone import IsaacDrone
    drone = IsaacDrone(stage, simulation_app, prim_path="/World/drone1",
                       spawn=(args.x, args.y, 0.1), mat_path=mat_path)
    drone.post_play_init()                            # crea Camera+render product (timeline ya en play)
    drone.takeoff()                                   # sube a z=2.2 (gate z>0.8 del position_controller)


# --- HUD de rendimiento opcional (FPS / RTF / GPU / memoria) -----------------
perf = None
if args.perf:
    from perf_hud import PerfMonitor, enable_stats_overlay
    if not args.headless:
        enable_stats_overlay()                        # overlay de stats del viewport (GUI)
    perf = PerfMonitor(simulation_app, timeline=timeline, publish_ros=True, ros_node=node)

# --- rig multicámara opcional (wheel/scene/chase) + grabación a disco --------
CAMS = {}
cam_rec = None
_cams = None
if args.cameras or args.rec_cams or args.viewports:
    import cameras as _cams
    CAMS = _cams.setup_cameras(stage, simulation_app, robot_prim=prim_path, wheel_link_prim=None,
                               specs={"scene": {"resolution": args.cam_res},
                                      "chase": {"resolution": args.cam_res},
                                      "wheel": {"resolution": args.cam_res}})
    if args.viewports and not args.headless:
        _cams.create_viewports(CAMS, resolution=(640, 360))
    if args.rec_cams:
        import datetime as _dt
        _outd = os.path.join(HERE, f"cams_{_dt.datetime.now():%Y%m%d_%H%M%S}")
        cam_rec = _cams.Recorder(CAMS, _outd, fps=args.record_fps, resolution=args.cam_res)
        cam_rec.start()


def _rec_step():
    """Captura un frame si toca; arranca al moverse, termina a record_secs. No-fatal."""
    if rec is None or rec["done"]:
        return
    try:
        import subprocess
        import numpy as _np
        moving = (abs(cmd["vx"]) + abs(cmd["vy"]) + abs(cmd["wz"])) > 1e-4
        t = timeline.get_current_time()
        if not rec["on"]:
            if moving:
                rec["ff"] = subprocess.Popen(
                    ["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo",
                     "-pix_fmt", "rgb24", "-s", f'{rec["W"]}x{rec["H"]}',
                     "-r", str(args.record_fps), "-i", "-", "-an",
                     "-c:v", "libx264", "-pix_fmt", "yuv420p", rec["out"]],
                    stdin=subprocess.PIPE)
                rec["on"] = True; rec["t0"] = t; rec["next"] = t
                print("[REC] ► grabando…", flush=True)
            return
        if t - rec["t0"] >= args.record_secs:
            rec["ff"].stdin.close(); rec["ff"].wait(); rec["done"] = True
            print(f"[REC] ■ guardado {rec['out']}", flush=True)
            return
        if t >= rec["next"]:
            fr = rec["cam"].get_rgba()
            if fr is not None and getattr(fr, "size", 0):
                rgb = fr[:, :, :3]
                if rgb.dtype != _np.uint8:
                    rgb = (_np.clip(rgb, 0.0, 1.0) * 255).astype(_np.uint8)
                rec["ff"].stdin.write(_np.ascontiguousarray(rgb).tobytes())
            rec["next"] += 1.0 / args.record_fps
    except Exception as _e:
        print(f"[REC] error de grabación, la desactivo: {_e}", flush=True)
        rec["done"] = True


print("[FIS] Listo. /cmd_vel mueve el robot POR FRICCIÓN (ruedas+rodillos).")
try:
    while simulation_app.is_running():
        rclpy.spin_once(node, timeout_sec=0.0)
        if drone is not None:
            drone.spin_once()
            drone.step(1.0 / 120.0)
        targets = ik(cmd["vx"], cmd["vy"], cmd["wz"])
        art.apply_action(ArticulationAction(joint_velocities=targets,
                                            joint_indices=np.array(wheel_idx)))
        simulation_app.update()
        if drone is not None:
            drone.publish_odom()
        if perf is not None:
            perf.tick()
        if CAMS and _cams is not None:
            _cams.update_chase_camera(stage, CAMS)
        if cam_rec is not None:
            cam_rec.step()
        _rec_step()
except KeyboardInterrupt:
    pass
finally:
    if rec is not None and rec.get("ff") is not None and not rec["done"]:
        try:
            rec["ff"].stdin.close(); rec["ff"].wait()
            print(f"[REC] ■ guardado (parcial) {rec['out']}", flush=True)
        except Exception:
            pass
    if drone is not None:
        drone.shutdown()
    if perf is not None:
        perf.shutdown()
    if cam_rec is not None:
        try:
            cam_rec.stop()
            cam_rec.to_mp4()
        except Exception:
            pass
    rclpy.shutdown()
    timeline.stop()
    simulation_app.close()
