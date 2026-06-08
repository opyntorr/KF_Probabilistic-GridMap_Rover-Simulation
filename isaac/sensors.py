#!/usr/bin/env python3
# sensors.py — añade al JetAuto los SENSORES que faltan en el port por física
# (scene_mecanum.py ya publica /clock, /odom, /tf, /joint_states y, con --gridmap,
# el RTX lidar en /scan). Aquí se añaden, igual que en Gazebo:
#
#   * IMU (MPU-6050)  -> sensor_msgs/Imu  en /imu/data_raw   (50 Hz reales)
#   * Cámara Astra    -> /cam_1/image (rgb), /cam_1/depth_image, /cam_1/camera_info
#                        (alimenta el visual servoing con ArUco de control_trayectoria)
#
# Diseño (ver INTEGRATION CONTRACT): funciones puras, SIN efectos al importar. Reciben
# (stage, simulation_app, ...) y AÑADEN nodos al grafo existente "/ActionGraph" usando
# og.Controller.edit({"graph_path": "/ActionGraph"...}), igual que scene_mecanum.py.
# El orquestador las llama tras crear el grafo ROS2 y antes (o después) de timeline.play().
#
# Las APIs están copiadas de los ejemplos/exts locales de Isaac 4.5 (citados en línea).

import math

import omni.graph.core as og
import omni.kit.commands
import usdrt.Sdf
from pxr import Gf, Usd, UsdGeom

# rpy/xyz del URDF jetauto_mecanum.urdf (líneas ~1740 imu_joint, ~1814 cam_1_joint):
#   imu_joint   : xyz=(-0.0849, 0, 0.080964)  rpy=(0,0,π)     parent=base_link
#   cam_1_joint : xyz=( 0.028858, 0, 0.202093) rpy=(0,0.261,0) parent=base_link
#   cam_1_optical_frame: REP-103 (z fwd, x right, y down) bajo cam_1_link.
# Como el importador de URDF YA crea estos links como prims (imu_link, cam_1_link,
# cam_1_optical_frame), los localizamos por NOMBRE en el stage (igual que scene_mecanum.py
# busca "lidar_frame" en stage.Traverse(), líneas ~372-375) en vez de recolocarlos a mano.


def _find_prim_by_name(stage, name, under=None):
    """Devuelve el pathString del primer prim llamado `name` (opcionalmente bajo `under`).
    Mismo patrón que la búsqueda de 'lidar_frame' en scene_mecanum.py (stage.Traverse)."""
    root = stage.GetPrimAtPath(under) if under else None
    it = Usd.PrimRange(root) if (root and root.IsValid()) else stage.Traverse()
    for p in it:
        if p.GetName() == name:
            return p.GetPath().pathString
    return None


# ---------------------------------------------------------------------------
# IMU: IsaacImuSensor en el link del IMU + IsaacReadIMU + ROS2PublishImu
# ---------------------------------------------------------------------------
def add_imu(stage, simulation_app,
            base_link_prim="/jetauto/base_link",
            topic="/imu/data_raw",
            frame_id="imu_link",
            imu_link_name="imu_link",
            graph_path="/ActionGraph"):
    """Crea un IMU físico en imu_link y publica sensor_msgs/Imu en `topic`.

    Cableado (OnPlaybackTick -> IsaacReadIMU -> ROS2PublishImu; ReadSimTime -> timeStamp).
    APIs copiadas de:
      - IsaacSensorCreateImuSensor: exts/isaacsim.sensors.physics/.../scripts/commands.py
        (clase IsaacSensorCreateImuSensor, args path/parent/translation/orientation/...).
      - IsaacReadIMU (inputs:imuPrim target, inputs:readGravity; outputs:linAcc/angVel/
        orientation): exts/isaacsim.sensors.physics/docs/ogn/OgnIsaacReadIMU.rst.
      - ROS2PublishImu (inputs: execIn, timeStamp, frameId, topicName, linearAcceleration,
        angularVelocity, orientation): exts/isaacsim.ros2.bridge/docs/ogn/OgnROS2PublishImu.rst.

    Devuelve el pathString del prim del sensor IMU, o None si no se halló imu_link.
    """
    # 1) localizar el link del IMU (lo crea el importador de URDF). Si se pasó una ruta
    #    de base_link válida, buscamos bajo el robot; si no, en todo el stage.
    under = base_link_prim
    if not (stage.GetPrimAtPath(under) and stage.GetPrimAtPath(under).IsValid()):
        under = None
    imu_link_path = _find_prim_by_name(stage, imu_link_name, under=under)
    if imu_link_path is None:
        # respaldo: directamente bajo base_link_prim (nombre conocido del URDF)
        cand = f"{base_link_prim}/{imu_link_name}"
        if stage.GetPrimAtPath(cand) and stage.GetPrimAtPath(cand).IsValid():
            imu_link_path = cand
    if imu_link_path is None:
        print(f"[SENS] add_imu: no encontré '{imu_link_name}'; IMU NO creado", flush=True)
        return None

    # 2) crear el sensor IMU físico como hijo del imu_link (offset 0: el link YA está
    #    en la pose del URDF). Comando de commands.py (IsaacSensorCreateImuSensor).
    #    sensor_period=-1 -> mide cada paso de física (lo limita el publisher/gate).
    _res = omni.kit.commands.execute(
        "IsaacSensorCreateImuSensor",
        path="/imu_sensor",
        parent=imu_link_path,
        translation=Gf.Vec3d(0.0, 0.0, 0.0),
        orientation=Gf.Quatd(1.0, 0.0, 0.0, 0.0),
        sensor_period=-1.0,
        linear_acceleration_filter_size=1,
        angular_velocity_filter_size=1,
        orientation_filter_size=1,
    )
    imu_sensor_path = f"{imu_link_path}/imu_sensor"
    simulation_app.update()
    print(f"[SENS] IMU sensor en {imu_sensor_path}", flush=True)

    # 3) añadir los nodos del grafo ROS2 al "/ActionGraph" existente. Reusa el patrón de
    #    og.Controller.edit de scene_mecanum.py (mismo graph_path/evaluator).
    #    OJO topic: con barra inicial ("/imu/data_raw") se publica tal cual (absoluto).
    og.Controller.edit(
        graph_path,  # ruta STRING -> AÑADE nodos al grafo existente (NO lo recrea)
        {
            og.Controller.Keys.CREATE_NODES: [
                ("ReadIMU", "isaacsim.sensors.physics.IsaacReadIMU"),
                ("PublishIMU", "isaacsim.ros2.bridge.ROS2PublishImu"),
            ],
            og.Controller.Keys.CONNECT: [
                # el grafo ya tiene "OnTick" (OnPlaybackTick) y "ReadSimTime"
                (f"{graph_path}/OnTick.outputs:tick", "ReadIMU.inputs:execIn"),
                ("ReadIMU.outputs:execOut", "PublishIMU.inputs:execIn"),
                ("ReadIMU.outputs:linAcc", "PublishIMU.inputs:linearAcceleration"),
                ("ReadIMU.outputs:angVel", "PublishIMU.inputs:angularVelocity"),
                ("ReadIMU.outputs:orientation", "PublishIMU.inputs:orientation"),
                (f"{graph_path}/ReadSimTime.outputs:simulationTime", "PublishIMU.inputs:timeStamp"),
            ],
            og.Controller.Keys.SET_VALUES: [
                # imuPrim es 'target' -> mismo form [usdrt.Sdf.Path(...)] que chassisPrim/
                # targetPrim en scene_mecanum.py (líneas 355/363).
                ("ReadIMU.inputs:imuPrim", [usdrt.Sdf.Path(imu_sensor_path)]),
                ("ReadIMU.inputs:readGravity", True),
                ("PublishIMU.inputs:topicName", topic),
                ("PublishIMU.inputs:frameId", frame_id),
            ],
        },
    )
    print(f"[SENS] IMU -> {topic} (frame '{frame_id}')", flush=True)
    return imu_sensor_path


# ---------------------------------------------------------------------------
# Cámara RGBD (Astra): Camera en cam_1_optical_frame + RenderProduct + 3 CameraHelper
# ---------------------------------------------------------------------------
def add_rgbd_camera(stage, simulation_app,
                    parent_prim=None,
                    topic_base="/cam_1",
                    resolution=(640, 480),
                    frame_id="cam_1_optical_frame",
                    optical_frame_name="cam_1_optical_frame",
                    cam_link_name="cam_1_link",
                    base_link_prim="/jetauto/base_link",
                    horizontal_fov_deg=60.0,
                    clipping=(0.1, 15.0),
                    graph_path="/ActionGraph"):
    """Crea una Camera USD en la pose del frame óptico de la Astra y publica
    rgb (/cam_1/image), depth (/cam_1/depth_image) y camera_info (/cam_1/camera_info).

    Patrón EXACTO del atajo oficial de Isaac (la herramienta del menú "ROS2 Camera"):
      exts/isaacsim.ros2.bridge/.../og_shortcuts/og_rtx_sensors.py
        IsaacCreateRenderProduct(cameraPrim, width, height) -> renderProductPath
          -> ROS2CameraHelper(type="rgb"/"depth") y ROS2CameraInfoHelper.
    Atributos de los nodos confirmados en:
      - OgnIsaacCreateRenderProduct.rst (inputs:cameraPrim[target], width, height;
        outputs:execOut, renderProductPath).
      - OgnROS2CameraHelper.rst (inputs:execIn, renderProductPath, topicName, type,
        frameId; allowedTokens de type incluyen rgb, depth, camera_info).
    Y el flujo OnTick->RenderProduct->Helpers replica camera_periodic.py / camera_manual.py
    (standalone_examples/api/isaacsim.ros2.bridge/).

    Si la Astra apunta a control_trayectoria (ArUco), los topics /cam_1/image y
    /cam_1/camera_info son los que consume el visual servoing.

    Devuelve el pathString de la Camera, o None si no se halló el frame óptico.
    """
    w, h = int(resolution[0]), int(resolution[1])
    topic_base = topic_base.rstrip("/")

    # 1) localizar el frame óptico de la cámara (REP-103) que creó el importador de URDF.
    under = base_link_prim
    if not (stage.GetPrimAtPath(under) and stage.GetPrimAtPath(under).IsValid()):
        under = None
    if parent_prim and stage.GetPrimAtPath(parent_prim) and stage.GetPrimAtPath(parent_prim).IsValid():
        cam_parent = parent_prim
    else:
        cam_parent = _find_prim_by_name(stage, optical_frame_name, under=under)
        if cam_parent is None:
            # respaldo: el link físico de la cámara (no óptico). Mejor que nada.
            cam_parent = _find_prim_by_name(stage, cam_link_name, under=under)
    if cam_parent is None:
        print(f"[SENS] add_rgbd_camera: no encontré '{optical_frame_name}'/'{cam_link_name}'; "
              f"cámara NO creada", flush=True)
        return None

    # 2) crear el prim Camera bajo el frame óptico. Como el padre YA está en la
    #    convención óptica REP-103 (z adelante), la Camera local va sin rotación: en
    #    USD una UsdGeom.Camera mira por su -Z, mientras que el frame óptico tiene +Z
    #    hacia adelante -> rotamos 180° en Y para alinear el -Z de la cámara con el +Z
    #    óptico (x sigue a la derecha, y queda hacia abajo, como REP-103).
    camera_path = f"{cam_parent}/jetauto_camera"
    cam_geom = UsdGeom.Camera.Define(stage, camera_path)
    cam_prim = cam_geom.GetPrim()
    # parámetros de la cámara (apertura/foco). horizontal_fov del URDF = 1.0472 rad = 60°.
    # Mantenemos la apertura horizontal por defecto (20.955) y derivamos la focal del FOV:
    #   focal = (h_aperture/2) / tan(fov/2)
    h_aperture = 20.955
    v_aperture = h_aperture * (float(h) / float(w))
    focal = (h_aperture * 0.5) / math.tan(math.radians(horizontal_fov_deg) * 0.5)
    cam_geom.GetHorizontalApertureAttr().Set(h_aperture)
    cam_geom.GetVerticalApertureAttr().Set(v_aperture)
    cam_geom.GetProjectionAttr().Set("perspective")
    cam_geom.GetFocalLengthAttr().Set(focal)
    cam_geom.GetClippingRangeAttr().Set(Gf.Vec2f(float(clipping[0]), float(clipping[1])))
    # alinear -Z de la cámara con +Z del frame óptico (rot 180° sobre Y).
    xf = UsdGeom.Xformable(cam_prim)
    xf.ClearXformOpOrder()
    xf.AddRotateXYZOp().Set(Gf.Vec3f(0.0, 180.0, 0.0))
    simulation_app.update()
    print(f"[SENS] Camera en {camera_path} ({w}x{h}, fov~{horizontal_fov_deg:.0f}°)", flush=True)

    # 3) nodos del grafo: RenderProduct (offscreen, sin viewport) + 3 helpers.
    #    nombres de nodo únicos para no chocar con otros grafos de cámara.
    og.Controller.edit(
        graph_path,  # ruta STRING -> AÑADE nodos al grafo existente (NO lo recrea)
        {
            og.Controller.Keys.CREATE_NODES: [
                ("CamRenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("CamHelperRgb", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("CamHelperDepth", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("CamHelperInfo", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
            ],
            og.Controller.Keys.CONNECT: [
                # el grafo ya tiene "OnTick" (OnPlaybackTick).
                (f"{graph_path}/OnTick.outputs:tick", "CamRenderProduct.inputs:execIn"),
                ("CamRenderProduct.outputs:execOut", "CamHelperRgb.inputs:execIn"),
                ("CamRenderProduct.outputs:execOut", "CamHelperDepth.inputs:execIn"),
                ("CamRenderProduct.outputs:execOut", "CamHelperInfo.inputs:execIn"),
                ("CamRenderProduct.outputs:renderProductPath", "CamHelperRgb.inputs:renderProductPath"),
                ("CamRenderProduct.outputs:renderProductPath", "CamHelperDepth.inputs:renderProductPath"),
                ("CamRenderProduct.outputs:renderProductPath", "CamHelperInfo.inputs:renderProductPath"),
            ],
            og.Controller.Keys.SET_VALUES: [
                # cameraPrim es 'target' -> form [usdrt.Sdf.Path(...)] como en el resto.
                ("CamRenderProduct.inputs:cameraPrim", [usdrt.Sdf.Path(camera_path)]),
                ("CamRenderProduct.inputs:width", w),
                ("CamRenderProduct.inputs:height", h),
                # RGB -> /cam_1/image
                ("CamHelperRgb.inputs:type", "rgb"),
                ("CamHelperRgb.inputs:topicName", f"{topic_base}/image"),
                ("CamHelperRgb.inputs:frameId", frame_id),
                # Depth -> /cam_1/depth_image (float32 distancia al plano imagen)
                ("CamHelperDepth.inputs:type", "depth"),
                ("CamHelperDepth.inputs:topicName", f"{topic_base}/depth_image"),
                ("CamHelperDepth.inputs:frameId", frame_id),
                # CameraInfo -> /cam_1/camera_info
                ("CamHelperInfo.inputs:topicName", f"{topic_base}/camera_info"),
                ("CamHelperInfo.inputs:frameId", frame_id),
            ],
        },
    )
    print(f"[SENS] Camera -> {topic_base}/image (rgb), {topic_base}/depth_image (depth), "
          f"{topic_base}/camera_info (frame '{frame_id}')", flush=True)
    return camera_path
