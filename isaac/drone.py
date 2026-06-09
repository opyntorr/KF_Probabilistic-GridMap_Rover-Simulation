#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# drone.py — dron Tello KINEMATICO para Isaac Sim 4.5.
#
# A diferencia del JetAuto mecanum (scene_mecanum.py), que se mueve POR FISICA
# (friccion de ruedas), el dron se mueve de forma KINEMATICA: NO hay aerodinamica.
# El cuerpo es un RigidBody con `kinematicEnabled=True`, y en cada paso de fisica
# le ESCRIBIMOS la pose objetivo (integrando el twist de /drone1/cmd_vel). Asi el
# PID de posicion existente del usuario (tello_control_pos/controller.py) funciona
# SIN CAMBIOS: publica /drone1/cmd_vel (Twist en marco cuerpo) y lee /odometry/filtered
# (que pose_fuser arma a partir de /drone1/odom que aqui publicamos).
#
#   /drone1/cmd_vel (Twist, marco cuerpo) --(integracion)--> pose del cuerpo (world)
#   pose del cuerpo --> /drone1/odom (nav_msgs/Odometry)  --> pose_fuser --> /odometry/filtered
#   camara nadir 960x720 --> /uav/camera/image (+ /uav/camera/camera_info)
#   /drone1/takeoff (Empty) -> target z=2.2   ;   /drone1/land (Empty) -> target z=0.1
#
# CONTRATO con scene_mecanum.py (el orquestador integra; este modulo NO se autoejecuta):
#   - Importar DESPUES de crear el SimulationApp (igual que el resto de imports del scene).
#   - No hay efectos globales en import: todo ocurre dentro de IsaacDrone(...).
#   - Reusa el /physicsScene y (opcional) el /physicsMaterial ya creados por el scene.
#
# APIs ancladas a ejemplos/exts locales de Isaac 4.5 (citadas en los comentarios):
#   - omni.kit.asset_converter:  standalone_examples/api/omni.kit.asset_converter/asset_usd_converter.py
#   - add_reference_to_stage:    standalone_examples/tutorials/getting_started_robot.py
#   - Camera (+ set_local_pose / set_focal_length / get_render_product_path):
#       exts/isaacsim.sensors.camera/isaacsim/sensors/camera/camera.py
#   - ROS2CameraHelper / ROS2CameraInfoHelper (renderProductPath/frameId/topicName/type):
#       standalone_examples/api/isaacsim.ros2.bridge/camera_manual.py  (+ los .ogn de la ext)
#   - de-instanciar mallas referenciadas:  isaac/jetauto_materials.py (mismo truco SetInstanceable(False))

import math
import os

import numpy as np

# Imports de Isaac/USD: validos solo DESPUES de crear el SimulationApp (el scene ya lo hizo).
import omni.graph.core as og
import omni.usd
import usdrt.Sdf
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.core.utils.stage import add_reference_to_stage
from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

# --- rutas del proyecto Gazebo (solo lectura) ---------------------------------
_TELLO_DAE = ("/home/opyntorr/agv_uav_project_jetauto/src/demo_tello_sim/src/"
              "tello-ros2-gazebo-master/tello_ros/tello_gazebo/models/tello/meshes/tello.dae")
_TELLO_STL = _TELLO_DAE[:-4] + ".stl"   # respaldo: el .stl del mismo modelo

# Caja de colision/visual del Tello (model.sdf: <box>0.18 0.18 0.05</box>) — usada como
# respaldo visual si la conversion de malla falla, y como tamano del cuerpo kinematico.
_BODY_HALF = (0.09, 0.09, 0.025)

# Orientacion de la malla visual del Tello. La tello.dae viene Y-up: referenciada tal
# cual en un stage Z-up aparece DE PIE/ladeada (Gazebo la endereza por su loader). Rx+90
# la deja plana (hélices horizontales). Si queda panza-arriba, cambia a (-90,0,0); si la
# nariz apunta de lado, añade yaw en el 3er valor (p.ej. (90,0,90)).
_VISUAL_RPY = (90.0, 0.0, 0.0) # Restaurado a 90 porque el USDZ si venia Y-up
_VISUAL_SCALE = (0.00015, 0.00015, 0.00015) # Reducido aun mas

# Parametros de camara nadir (tello_con_espejo/model.sdf -> sensor camera_espejo):
#   pose (0.1, 0, -0.05) respecto a base_link, pitch 90deg (mira hacia abajo).
#   horizontal_fov ~ 0.940651 rad (el del espejo en tello/model.sdf), 960x720.
_CAM_LOCAL_POS = (0.1, 0.0, -0.05)
# HFOV del Tello que Gazebo REALMENTE voló = el sensor 'camera_espejo' de
# tello_con_espejo/model.sdf = 1.4416 rad (82.6°), fx≈546 = camera_tello_sim.yaml que usa
# el stitcher. (Antes estaba 0.940651/53.9° = el sensor base camera_down EQUIVOCADO → el
# stitcher escalaba los tiles ×1.73 y el footprint era 2.2m en vez de 3.87m.) 82.6° da
# footprint 3.87m a z=2.2 → ~75% overlap, cobertura completa del laberinto por foto.
_CAM_HFOV = 1.4432   # fx = (960/2)/tan(HFOV/2) = 546.4 px, casa con camera_tello_sim.yaml
_CAM_W, _CAM_H = 960, 720   # resolución original (el fix de publicación manual evita el estancamiento)
_CAM_NEAR, _CAM_FAR = 0.1, 10.0

# Alturas objetivo de los stubs (la mision/PID esperan z>0.8 para empezar a controlar;
# el controller usa target_z=2.5 como hover por defecto, asi que despegamos a 2.2).
_TAKEOFF_Z = 2.2
_LAND_Z = 0.1


def _convert_mesh_to_usd(in_path, out_path):
    """Convierte una malla (dae/stl/obj/fbx) a USD usando omni.kit.asset_converter.
    Patron copiado de standalone_examples/api/omni.kit.asset_converter/asset_usd_converter.py
    (create_converter_task + wait_until_finished en un bucle). Devuelve True si OK.
    Es no-fatal: si falla, el llamador usa un respaldo de UsdGeom."""
    import asyncio

    import omni.kit.asset_converter as _conv

    async def _run():
        ctx = _conv.AssetConverterContext()
        ctx.ignore_materials = False
        ctx.use_meter_as_world_unit = True       # el .dae viene en metros (asset/unit meter=1)
        inst = _conv.get_instance()
        task = inst.create_converter_task(in_path, out_path, lambda a, b: None, ctx)
        ok = False
        for _ in range(600):                      # ~ proteccion contra cuelgues
            ok = await task.wait_until_finished()
            if ok:
                break
            await asyncio.sleep(0.1)
        return ok

    try:
        return asyncio.get_event_loop().run_until_complete(_run())
    except Exception as ex:
        print(f"[DRONE] conversion de malla fallo ({ex})", flush=True)
        return False


def _yaw_to_quat(yaw):
    """Quaternion (w,x,y,z) de un giro `yaw` alrededor de +Z (mismo orden que set_world_pose)."""
    return np.array([math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0)], dtype=float)


def _quat_to_gf(q_wxyz):
    """np (w,x,y,z) -> Gf.Quatd(w, (x,y,z))."""
    w, x, y, z = (float(v) for v in q_wxyz)
    return Gf.Quatd(w, Gf.Vec3d(x, y, z))


class IsaacDrone:
    """Dron Tello KINEMATICO controlado por /drone1/cmd_vel.

    Uso (lo hace el orquestador en scene_mecanum.py, NO este modulo):
        drone = IsaacDrone(stage, simulation_app, prim_path="/World/drone1",
                           spawn=(0.0, 0.0, 0.1), mat_path="/physicsMaterial")
        # ... timeline.play() + simulation_app.update() (como ya hace el scene) ...
        drone.post_play_init()                 # tras play(): inicializa la camara + grafo ROS2
        while simulation_app.is_running():
            drone.spin_once()                  # procesa /drone1/cmd_vel, takeoff/land
            ...                                # (resto del loop del scene)
            drone.step(dt)                     # integra el twist y escribe la pose kinematica
            simulation_app.update()
            drone.publish_odom()               # publica /drone1/odom desde la pose

    Metodos publicos clave:  step(dt), spin_once(), post_play_init(), publish_odom(),
    takeoff(), land(), get_pose(), shutdown().
    """

    def __init__(self, stage, simulation_app, prim_path="/World/drone1",
                 spawn=(0.0, 0.0, 0.1), mat_path="/physicsMaterial",
                 cmd_topic="/drone1/cmd_vel", odom_topic="/drone1/odom",
                 image_topic="/uav/camera/image", info_topic="/uav/camera/camera_info",
                 camera_frame="drone1_camera", odom_frame="odom",
                 child_frame="base_link", max_climb=0.6, z0=0.0):
        self._stage = stage
        self._app = simulation_app
        self._prim_path = prim_path
        self._mat_path = mat_path
        self._cmd_topic = cmd_topic
        self._odom_topic = odom_topic
        self._image_topic = image_topic
        self._info_topic = info_topic
        self._camera_frame = camera_frame
        self._odom_frame = odom_frame
        self._child_frame = child_frame
        self._max_climb = float(max_climb)      # m/s techo de subida del stub takeoff/land
        # z0 = altura ABSOLUTA del piso del laberinto (cuando el laberinto se sube sobre el
        # warehouse). El dron vuela/odometriza RELATIVO a este piso: odom z = pos_abs - z0,
        # y el target de takeoff/land es z0 + (2.2 / 0.1). Así la misión/controlador siguen
        # en z relativa al piso (2.2 m) sin saber que la escena subió. z0=0 -> sin offset.
        self._z0 = float(z0)

        # Estado cinematico (pose en el mundo).
        self._pos = np.array([float(spawn[0]), float(spawn[1]), float(spawn[2])], dtype=float)
        self._yaw = 0.0
        # Twist comandado (marco cuerpo): vx, vy, vz, wz.
        self._cmd = {"vx": 0.0, "vy": 0.0, "vz": 0.0, "wz": 0.0}
        # velocidad mundial real del ultimo step (para el twist del odom).
        self._world_vel = np.zeros(3, dtype=float)
        self._target_z = None                   # si no es None, los stubs fuerzan vz hacia el

        # Atributos creados despues (post_play_init / _init_ros2). Se declaran ANTES de
        # construir nada para que cualquier metodo invocado durante el build los vea.
        self._camera = None
        self._cam_graph_built = False
        self._node = None
        self._odom_pub = None

        # Asegura extensiones (el scene ya habilita ros2.bridge; la camara puede faltar).
        # enable_extension es idempotente — ver scene_mecanum.py (mismo patron).
        enable_extension("isaacsim.ros2.bridge")
        enable_extension("isaacsim.sensors.camera")

        self._build_body()
        self._build_camera_prim()
        self._init_ros2()

    # ------------------------------------------------------------------ cuerpo
    def _build_body(self):
        """Crea /World/drone1 como Xform con RigidBody KINEMATICO + visual Tello.
        Sin colision activa (kinematico, sin aerodinamica): no debe interactuar con el
        suelo ni con paredes; solo lo ven las camaras/lidar."""
        stage = self._stage
        body = UsdGeom.Xform.Define(stage, self._prim_path)
        prim = body.GetPrim()

        # Op de transformacion UNICA (matriz) que reescribiremos cada step. ClearXformOpOrder
        # evita conflictos si el prim trae ops previos. Usamos AddTransformOp -> Set(Matrix4d).
        xf = UsdGeom.Xformable(prim)
        xf.ClearXformOpOrder()
        self._xform_op = xf.AddTransformOp()
        self._write_pose_to_stage()             # pose inicial

        # RigidBody KINEMATICO: PhysX lo trata como cuerpo cuya pose la fija el usuario
        # (no cae por gravedad, no lo empujan colisiones).
        #   - kinematicEnabled vive en UsdPhysics.RigidBodyAPI -> patron EXACTO de
        #     extsPhysics/omni.physx.demos/.../KinematicBodyDemo.py:
        #       UsdPhysics.RigidBodyAPI.Apply(prim).CreateKinematicEnabledAttr().Set(True)
        #   - disableGravity vive en PhysxSchema.PhysxRigidBodyAPI (refuerzo; un cuerpo
        #     kinematico ya ignora la gravedad). Ver physxRigidBodyAPI.h:CreateDisableGravityAttr.
        rb_api = UsdPhysics.RigidBodyAPI.Apply(prim)
        rb_api.CreateKinematicEnabledAttr().Set(True)
        UsdPhysics.MassAPI.Apply(prim)
        UsdPhysics.MassAPI(prim).CreateMassAttr(0.1)   # masa nominal (tello/model.sdf: 0.1 kg)
        physx_rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        physx_rb.CreateDisableGravityAttr().Set(True)

        # Visual: malla Tello (convertida a USD) referenciada bajo /World/drone1/visual.
        # Respaldo: una caja UsdGeom del tamano del cuerpo si la conversion falla.
        self._attach_visual(prim)

    def _attach_visual(self, body_prim):
        stage = self._stage
        vis_path = self._prim_path + "/visual"
        # USD convertido EN DIRECTORIO ESCRIBIBLE (el arbol de Gazebo es de solo lectura).
        # Preferimos isaac/assets/ del proyecto destino; si no se puede, /tmp.
        _assets_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "assets")
        usd_out = os.path.join(_assets_dir if os.path.isdir(_assets_dir) else "/tmp",
                               "tello_isaac.usdz")

        mesh_usd = None
        if os.path.exists(usd_out):
            mesh_usd = usd_out
        else:
            # intentar convertir el .dae; si no, el .stl (asset_converter soporta ambos).
            for src in (_TELLO_DAE, _TELLO_STL):
                if os.path.exists(src) and _convert_mesh_to_usd(src, usd_out):
                    mesh_usd = usd_out
                    break

        if mesh_usd is not None:
            try:
                # Xform-wrapper que ROTA la malla a plano (la tello.dae viene Y-up).
                # La referencia va a un hijo /mesh para no chocar con el xformOp del
                # wrapper (referenciar sobre el mismo prim mezcla los xformOpOrder).
                vxf = UsdGeom.Xform.Define(stage, vis_path)
                vxf.AddRotateXYZOp().Set(Gf.Vec3d(*_VISUAL_RPY))
                vxf.AddScaleOp().Set(Gf.Vec3d(*_VISUAL_SCALE))
                ref_path = vis_path + "/mesh"
                UsdGeom.Xform.Define(stage, ref_path)
                add_reference_to_stage(usd_path=mesh_usd, prim_path=ref_path)   # tutorials/getting_started_robot.py
                self._app.update()                                             # poblar la referencia
                # De-instanciar (las mallas referenciadas llegan como instancias, invisibles a
                # PrimRange) — mismo truco que jetauto_materials.py (SetInstanceable(False)).
                self._prop_ops = []
                for q in Usd.PrimRange(stage.GetPrimAtPath(vis_path)):
                    if q.IsInstanceable():
                        q.SetInstanceable(False)
                    if 'pervane' in q.GetName().lower():
                        xform = UsdGeom.Xformable(q)
                        if xform:
                            op = xform.AddRotateYOp(UsdGeom.XformOp.PrecisionDouble, "spin")
                            self._prop_ops.append(op)
                
                # No aplicamos colision a la visual (cuerpo kinematico sin aerodinamica).
                print(f"[DRONE] visual Tello desde {os.path.basename(mesh_usd)} (rpy={_VISUAL_RPY})", flush=True)
                return
            except Exception as ex:
                print(f"[DRONE] no pude referenciar la malla ({ex}); uso caja", flush=True)

        # Respaldo: caja del tamano del cuerpo (0.18 x 0.18 x 0.05 m).
        cube = UsdGeom.Cube.Define(stage, vis_path)
        cube.GetSizeAttr().Set(1.0)
        cube.CreateDisplayColorAttr([Gf.Vec3f(0.15, 0.15, 0.17)])
        cxf = UsdGeom.Xformable(cube.GetPrim())
        cxf.ClearXformOpOrder()
        cxf.AddScaleOp().Set(Gf.Vec3f(2 * _BODY_HALF[0], 2 * _BODY_HALF[1], 2 * _BODY_HALF[2]))
        print("[DRONE] visual = caja (respaldo, sin malla)", flush=True)

    # --------------------------------------------------------------- camara prim
    def _build_camera_prim(self):
        """Crea SOLO el prim UsdGeom.Camera hijo del cuerpo, mirando -Z (nadir).
        El objeto Camera de isaacsim.sensors.camera + el grafo ROS2 se crean en
        post_play_init() (necesitan el render activo tras timeline.play())."""
        self._cam_path = self._prim_path + "/camera_down"
        cam = UsdGeom.Camera.Define(self._stage, self._cam_path)
        prim = cam.GetPrim()

        # Pose local: (0.1,0,-0.05) y orientacion para que el eje optico (-Z de la camara)
        # apunte hacia abajo (-Z del mundo). En convencion USD la camara mira -Z; con el
        # cuerpo nivelado, un giro de +180deg en X deja -Z_cam apuntando a -Z_world (nadir),
        # con +X_cam = +X_world (frente del dron) -> imagen "hacia adelante" arriba.
        xf = UsdGeom.Xformable(prim)
        xf.ClearXformOpOrder()
        xf.AddTranslateOp().Set(Gf.Vec3d(*_CAM_LOCAL_POS))
        # NADIR: en USD la cámara mira su -Z local; con el cuerpo nivelado, -Z local = -Z
        # mundo = ABAJO, así que la orientación IDENTIDAD ya es nadir. El RotateX 180 previo
        # la volteaba hacia ARRIBA (veía el domo/cielo) -> /uav/camera salía gris uniforme.
        xf.AddRotateXYZOp().Set(Gf.Vec3d(0.0, 0.0, 0.0))

        # Intrinsecos: hFOV ~0.94 rad. Con focal f y apertura h:  hFOV = 2*atan(h/(2f)).
        # Fijamos f=24 (mm en escala USD) y despejamos h. set_focal_length/aperture viven en
        # el objeto Camera, pero podemos fijar los atributos USD directamente igual que
        # camera_manual.py (GetFocalLengthAttr().Set / GetHorizontalApertureAttr().Set).
        focal = 24.0
        h_ap = 2.0 * focal * math.tan(_CAM_HFOV / 2.0)
        v_ap = h_ap * (float(_CAM_H) / float(_CAM_W))
        cam.GetProjectionAttr().Set("perspective")
        cam.GetFocalLengthAttr().Set(focal)
        cam.GetHorizontalApertureAttr().Set(h_ap)
        cam.GetVerticalApertureAttr().Set(v_ap)
        cam.GetClippingRangeAttr().Set(Gf.Vec2f(_CAM_NEAR, _CAM_FAR))

    def post_play_init(self):
        """Llamar UNA vez DESPUES de timeline.play()+simulation_app.update().
        Crea el objeto Camera (genera su render product) y el grafo ROS2 de imagen+info."""
        if self._cam_graph_built:
            return
        try:
            # Cámara por get_rgba() MANUAL (objeto isaacsim Camera). Es el ÚNICO método que
            # publica de forma fiable aquí: el ROS2CameraHelper (async, vía grafo) NO publica
            # datos para esta cámara (render-var stall, antes y después de play; un límite de
            # Isaac). get_rgba es ~1s/frame -> el loop llama publish_camera con throttle y la
            # cámara se mantiene caliente con el render de cada paso. (Cuello del RTF.)
            from isaacsim.sensors.camera import Camera
            self._camera = Camera(prim_path=self._cam_path, resolution=(_CAM_W, _CAM_H))
            self._camera.initialize()
            for _ in range(5):
                self._app.update()                       # calentar el render
            self._cam_graph_built = True
            print(f"[DRONE] camara nadir {_CAM_W}x{_CAM_H} -> {self._image_topic} "
                  f"(get_rgba manual; ROS2CameraHelper no publica datos aquí)", flush=True)
        except Exception as ex:
            import traceback
            print(f"[DRONE] camara ROS2 deshabilitada ({ex})\n{traceback.format_exc()}", flush=True)

    def _build_camera_graph(self, camera_prim_path):
        """Grafo ROS2 de cámara: IsaacCreateRenderProduct(camera_prim) + ROS2CameraHelper
        (rgb) + ROS2CameraInfoHelper, añadidos al /ActionGraph (OnPlaybackTick) para que
        publiquen en CADA render. Patrón de sensors.py (cam_1), que publica continuo."""
        # Patrón EXACTO de sensors.py (cam_1, que publica continuo): IsaacCreateRenderProduct
        # crea el render-product OFFSCREEN desde el PRIM de la cámara, y la cadena execIn es
        # OnTick -> CamRP.execIn -> CamRP.execOut -> CamHelper.execIn. Todo en /ActionGraph
        # (STRING path = AÑADE al grafo existente OnPlaybackTick).
        import usdrt
        keys = og.Controller.Keys
        og.Controller.edit(
            "/ActionGraph",
            {
                keys.CREATE_NODES: [
                    ("DroneCamRP", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                    ("DroneCamHelperRgb", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                    ("DroneCamHelperInfo", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
                ],
                keys.CONNECT: [
                    ("/ActionGraph/OnTick.outputs:tick", "DroneCamRP.inputs:execIn"),
                    ("DroneCamRP.outputs:execOut", "DroneCamHelperRgb.inputs:execIn"),
                    ("DroneCamRP.outputs:execOut", "DroneCamHelperInfo.inputs:execIn"),
                    ("DroneCamRP.outputs:renderProductPath", "DroneCamHelperRgb.inputs:renderProductPath"),
                    ("DroneCamRP.outputs:renderProductPath", "DroneCamHelperInfo.inputs:renderProductPath"),
                ],
                keys.SET_VALUES: [
                    ("DroneCamRP.inputs:cameraPrim", [usdrt.Sdf.Path(camera_prim_path)]),
                    ("DroneCamRP.inputs:width", _CAM_W),
                    ("DroneCamRP.inputs:height", _CAM_H),
                    ("DroneCamHelperRgb.inputs:type", "rgb"),
                    ("DroneCamHelperRgb.inputs:topicName", self._image_topic),
                    ("DroneCamHelperRgb.inputs:frameId", self._camera_frame),
                    ("DroneCamHelperInfo.inputs:topicName", self._info_topic),
                    ("DroneCamHelperInfo.inputs:frameId", self._camera_frame),
                ],
            },
        )
        self._app.update()

    # ----------------------------------------------------------------- ROS2 IO
    def _init_ros2(self):
        """rclpy node propio: suscribe /drone1/cmd_vel + takeoff/land, publica /drone1/odom.
        Mismo patron que scene_mecanum.py (rclpy.init + create_node + create_subscription).
        rclpy.init es idempotente con try/except por si el scene ya lo inicializo."""
        import rclpy
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from std_msgs.msg import Empty

        self._rclpy = rclpy
        self._Odometry = Odometry
        try:
            rclpy.init()
        except Exception:
            pass   # ya inicializado por el scene

        self._node = rclpy.create_node("isaac_drone")
        self._node.create_subscription(Twist, self._cmd_topic, self._on_cmd_vel, 10)
        self._node.create_subscription(Empty, "/drone1/takeoff", lambda _m: self.takeoff(), 1)
        self._node.create_subscription(Empty, "/drone1/land", lambda _m: self.land(), 1)
        self._odom_pub = self._node.create_publisher(Odometry, self._odom_topic, 10)
        from sensor_msgs.msg import CameraInfo, Image   # publicación MANUAL de la cámara
        self._Image, self._CameraInfo = Image, CameraInfo
        self._img_pub = self._node.create_publisher(Image, self._image_topic, 2)
        self._info_pub = self._node.create_publisher(CameraInfo, self._info_topic, 2)
        self._cam_ctr = 0
        print(f"[DRONE] ROS2 listo: sub {self._cmd_topic} + /drone1/takeoff,/land ; "
              f"pub {self._odom_topic} + {self._image_topic} (manual)", flush=True)

    def _on_cmd_vel(self, m):
        # linear.x/y/z en MARCO CUERPO (el PID rota global->cuerpo con el yaw); angular.z=yaw rate.
        self._cmd["vx"] = float(m.linear.x)
        self._cmd["vy"] = float(m.linear.y)
        self._cmd["vz"] = float(m.linear.z)
        self._cmd["wz"] = float(m.angular.z)
        # Si el PID manda Z explicita, sale del modo takeoff/land automatico.
        if abs(m.linear.z) > 1e-4:
            self._target_z = None

    def spin_once(self, timeout_sec=0.0):
        """Procesa callbacks ROS2 pendientes (igual que rclpy.spin_once en el scene)."""
        self._rclpy.spin_once(self._node, timeout_sec=timeout_sec)

    # ------------------------------------------------------------- stubs takeoff
    def takeoff(self):
        """TelloAction 'takeoff': fija un objetivo de altura (sube a 2.2 m SOBRE el piso)."""
        self._target_z = self._z0 + _TAKEOFF_Z
        print(f"[DRONE] TAKEOFF -> z={_TAKEOFF_Z} sobre piso (abs={self._target_z:.2f})", flush=True)

    def land(self):
        """TelloAction 'land': fija objetivo de altura al piso (0.1 m sobre el piso)."""
        self._target_z = self._z0 + _LAND_Z
        print(f"[DRONE] LAND -> z={_LAND_Z} sobre piso (abs={self._target_z:.2f})", flush=True)

    # ----------------------------------------------------------------- dinamica
    def step(self, dt):
        """Integra el twist (marco cuerpo) en la pose mundial y la ESCRIBE en el prim
        kinematico. Llamar cada paso, ANTES de simulation_app.update()."""
        dt = float(dt)
        if dt <= 0.0:
            return

        # Animar helices si esta despegado (z > 0.05) con Frame Skipping
        if hasattr(self, '_prop_ops') and self._prop_ops and self._pos[2] > 0.05:
            if not hasattr(self, '_prop_timer'):
                self._prop_timer = 0.0
                self._prop_angle = 0.0
            
            self._prop_timer += dt
            # Solo actualizar visualmente unas 20 veces por segundo (cada 0.05s)
            if self._prop_timer >= 0.05:
                # Girar muy rapido (miles de grados por segundo en simulacion)
                self._prop_angle += 2000.0 * self._prop_timer
                if self._prop_angle > 360.0:
                    self._prop_angle %= 360.0
                for op in self._prop_ops:
                    op.Set(self._prop_angle)
                self._prop_timer = 0.0

        # 1) yaw (angular.z).
        self._yaw += self._cmd["wz"] * dt

        # 2) velocidad lineal: marco cuerpo -> mundo (giro plano por yaw). vz es vertical.
        c, s = math.cos(self._yaw), math.sin(self._yaw)
        vx_b, vy_b, vz = self._cmd["vx"], self._cmd["vy"], self._cmd["vz"]
        vx_w = c * vx_b - s * vy_b
        vy_w = s * vx_b + c * vy_b

        # 3) si hay objetivo de altura (takeoff/land) y el PID no manda vz, generar vz hacia el.
        if self._target_z is not None and abs(vz) < 1e-6:
            dz = self._target_z - self._pos[2]
            vz = max(-self._max_climb, min(self._max_climb, dz))   # P simple saturado
            if abs(dz) < 0.02:
                vz = 0.0

        # 4) integrar posicion.
        self._world_vel[:] = (vx_w, vy_w, vz)
        self._pos = self._pos + self._world_vel * dt
        if self._pos[2] < self._z0:            # no bajar del piso del laberinto (abs z0)
            self._pos[2] = self._z0
            self._world_vel[2] = 0.0

        # 5) escribir la pose objetivo en el cuerpo kinematico (PhysX la respeta exacta).
        self._write_pose_to_stage()

    def _write_pose_to_stage(self):
        """Reescribe la op de transformacion del cuerpo con la pose actual (pos + yaw)."""
        q = _quat_to_gf(_yaw_to_quat(self._yaw))
        M = Gf.Matrix4d()
        M.SetRotateOnly(q)
        M.SetTranslateOnly(Gf.Vec3d(float(self._pos[0]), float(self._pos[1]), float(self._pos[2])))
        self._xform_op.Set(M)

    # ------------------------------------------------------------------- odom
    def publish_odom(self):
        """Publica /drone1/odom (nav_msgs/Odometry) desde la pose integrada.
        frame_id='odom', child_frame_id='base_link' (lo que pose_fuser espera).
        El twist es la velocidad MUNDIAL del ultimo step (pose_fuser lo pasa tal cual)."""
        if self._odom_pub is None:
            return
        msg = self._Odometry()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = self._odom_frame
        msg.child_frame_id = self._child_frame
        msg.pose.pose.position.x = float(self._pos[0])
        msg.pose.pose.position.y = float(self._pos[1])
        msg.pose.pose.position.z = float(self._pos[2] - self._z0)   # RELATIVO al piso del laberinto
        w, x, y, z = _yaw_to_quat(self._yaw)
        msg.pose.pose.orientation.w = float(w)
        msg.pose.pose.orientation.x = float(x)
        msg.pose.pose.orientation.y = float(y)
        msg.pose.pose.orientation.z = float(z)
        msg.twist.twist.linear.x = float(self._world_vel[0])
        msg.twist.twist.linear.y = float(self._world_vel[1])
        msg.twist.twist.linear.z = float(self._world_vel[2])
        msg.twist.twist.angular.z = float(self._cmd["wz"])
        self._odom_pub.publish(msg)
        # La cámara NO se publica aquí: el loop llama publish_camera() SOLO en frames de
        # render (donde hay un frame fresco). Llamar get_rgba en pasos física-solo devolvía
        # None y forzaba renders extra -> mataba el RTF. Odom sí va cada paso (es barato).

    def publish_camera(self):
        """Publica /uav/camera/image (+camera_info) con get_rgba(). El orquestador la llama
        SOLO en frames de RENDER (tras simulation_app.update()), donde hay un frame fresco;
        así no se fuerza render en pasos física-solo (clave para el RTF). La misión solo
        necesita 1 frame por waypoint y el dron espera ahí > el periodo de render."""
        if getattr(self, "_img_pub", None) is None or self._camera is None:
            return
        try:
            import numpy as np
            rgba = self._camera.get_rgba()
            # El render-var de la cámara LAG ~1 frame: tras pasos sin captura queda frío y
            # get_rgba() sale None. Un update() extra justo antes de releer lo deja listo.
            # Como publish_camera SOLO se llama en hover (waypoint), este costo NO afecta el
            # RTF del vuelo. Reintenta hasta 6 veces (recupera la cámara aunque venga fría).
            for _ in range(6):
                if rgba is not None and getattr(rgba, "size", 0) > 0:
                    break
                self._app.update()
                rgba = self._camera.get_rgba()
        except Exception:
            return
        if rgba is None or getattr(rgba, "size", 0) == 0:
            return
        rgb = rgba[:, :, :3]
        if rgb.dtype != np.uint8:
            rgb = (np.clip(rgb, 0.0, 1.0) * 255).astype(np.uint8)
        rgb = np.ascontiguousarray(rgb)
        h, w = int(rgb.shape[0]), int(rgb.shape[1])
        stamp = self._node.get_clock().now().to_msg()
        img = self._Image()
        img.header.stamp = stamp
        img.header.frame_id = self._camera_frame
        img.height, img.width = h, w
        img.encoding = "rgb8"
        img.is_bigendian = 0
        img.step = w * 3
        img.data = rgb.tobytes()
        self._img_pub.publish(img)
        # camera_info con intrínsecos del FOV (fx=fy, cx/cy al centro)
        fx = (w / 2.0) / math.tan(_CAM_HFOV / 2.0)
        info = self._CameraInfo()
        info.header = img.header
        info.height, info.width = h, w
        info.distortion_model = "plumb_bob"
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.k = [fx, 0.0, w / 2.0, 0.0, fx, h / 2.0, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, w / 2.0, 0.0, 0.0, fx, h / 2.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        self._info_pub.publish(info)

    # ------------------------------------------------------------------ varios
    def get_pose(self):
        """Devuelve (np.array([x,y,z]), yaw) actuales del dron."""
        return self._pos.copy(), self._yaw

    def shutdown(self):
        try:
            if self._node is not None:
                self._node.destroy_node()
        except Exception:
            pass
