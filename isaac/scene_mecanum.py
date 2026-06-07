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

# cinemática mecanum (misma que rover_model/sensor_models)
WHEEL_R = 0.04825
WHEEL_K = 0.20
WHEEL_KD = 600.0          # damping del drive de velocidad de rueda (tune)
WHEELS = ['front_left_wheel_joint', 'front_right_wheel_joint',
          'back_left_wheel_joint', 'back_right_wheel_joint']

parser = argparse.ArgumentParser()
parser.add_argument("--headless", action="store_true")
parser.add_argument("--gridmap", action="store_true", help="añade cuarto 6x6 + RTX lidar /scan")
parser.add_argument("--urdf", default=URDF_PATH)
parser.add_argument("--x", type=float, default=None)
parser.add_argument("--y", type=float, default=None)
parser.add_argument("--z", type=float, default=0.05)
parser.add_argument("--yaw", type=float, default=None)
args, _ = parser.parse_known_args()

# spawn por defecto segun la tarea (igual que el port kinematico)
if args.x is None:
    args.x = -1.0 if args.gridmap else 0.0
if args.y is None:
    args.y = -1.0 if args.gridmap else 0.0
if args.yaw is None:
    args.yaw = math.pi / 2 if args.gridmap else 0.0

simulation_app = SimulationApp({"renderer": "RaytracedLighting", "headless": args.headless})

import numpy as np
import omni.graph.core as og
import omni.kit.commands
import omni.timeline
import usdrt.Sdf
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.core.utils.types import ArticulationAction
from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdLux, UsdPhysics, UsdShade

enable_extension("isaacsim.ros2.bridge")
enable_extension("isaacsim.asset.importer.urdf")
if args.gridmap:
    enable_extension("isaacsim.sensors.rtx")
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
physx.CreateTimeStepsPerSecondAttr(120.0)      # paso fino: rodillos pequeños

# material de fricción (suelo + rodillos)
mat_path = "/physicsMaterial"
UsdShade.Material.Define(stage, mat_path)
mat = UsdPhysics.MaterialAPI.Apply(stage.GetPrimAtPath(mat_path))
mat.CreateStaticFrictionAttr(1.0)
mat.CreateDynamicFrictionAttr(1.0)
mat.CreateRestitutionAttr(0.0)

omni.kit.commands.execute(
    "AddGroundPlaneCommand", stage=stage, planePath="/groundPlane", axis="Z",
    size=100.0, position=Gf.Vec3f(0, 0, 0.0), color=Gf.Vec3f(0.5),
)
# fricción al suelo
gp = stage.GetPrimAtPath("/groundPlane/CollisionPlane")
if gp and gp.IsValid():
    UsdShade.MaterialBindingAPI.Apply(gp).Bind(
        UsdShade.Material(stage.GetPrimAtPath(mat_path)),
        bindingStrength=UsdShade.Tokens.weakerThanDescendants, materialPurpose="physics")

light = UsdLux.DistantLight.Define(stage, Sdf.Path("/DistantLight"))
light.CreateIntensityAttr(1000)

# fricción a TODOS los colliders de rodillo
n_roller = 0
for p in stage.Traverse():
    nm = p.GetName()
    if "roller" in nm and p.HasAPI(UsdPhysics.CollisionAPI):
        UsdShade.MaterialBindingAPI.Apply(p).Bind(
            UsdShade.Material(stage.GetPrimAtPath(mat_path)),
            bindingStrength=UsdShade.Tokens.weakerThanDescendants, materialPurpose="physics")
        n_roller += 1
print(f"[FIS] fricción aplicada a {n_roller} colliders de rodillo")

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

# --- 3b) (gridmap) RTX lidar 2D en lidar_frame -> /scan --------------------
if args.gridmap:
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


def _quat_yaw(yaw):
    return np.array([math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0)])


art.set_world_pose(position=np.array([args.x, args.y, args.z]), orientation=_quat_yaw(args.yaw))

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

print("[FIS] Listo. /cmd_vel mueve el robot POR FRICCIÓN (ruedas+rodillos).")
try:
    while simulation_app.is_running():
        rclpy.spin_once(node, timeout_sec=0.0)
        targets = ik(cmd["vx"], cmd["vy"], cmd["wz"])
        art.apply_action(ArticulationAction(joint_velocities=targets,
                                            joint_indices=np.array(wheel_idx)))
        simulation_app.update()
except KeyboardInterrupt:
    pass
finally:
    rclpy.shutdown()
    timeline.stop()
    simulation_app.close()
