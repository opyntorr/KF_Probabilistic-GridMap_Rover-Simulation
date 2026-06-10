#!/usr/bin/env python3
"""Diagnóstico: render TOP-DOWN limpio del laberinto (cámara alta mirando -Z) para ver
los COLORES/estructura que el stitcher necesita. Sin dron, sin env. Lights por argv.
  uso: python.sh _maze_topdown.py [dome] [distant] [z_cam] [auto_exp 0/1]
Guarda /tmp/maze_top.png."""
import sys
from isaacsim import SimulationApp

dome = float(sys.argv[1]) if len(sys.argv) > 1 else 300.0
distant = float(sys.argv[2]) if len(sys.argv) > 2 else 600.0
z_cam = float(sys.argv[3]) if len(sys.argv) > 3 else 8.0
auto_exp = (len(sys.argv) > 4 and sys.argv[4] == "1")

simulation_app = SimulationApp({"renderer": "RaytracedLighting", "headless": True})

import carb
import numpy as np
import omni.kit.commands
from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdPhysics, PhysxSchema
import omni.usd
from isaacsim.core.utils.stage import add_reference_to_stage  # noqa

if not auto_exp:
    cs = carb.settings.get_settings()
    cs.set("/rtx/post/histogram/enabled", False)

stage = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)

# physicsScene + material (world_loader los requiere)
UsdPhysics.Scene.Define(stage, Sdf.Path("/physicsScene"))
mat_path = "/physicsMaterial"
UsdShade = __import__("pxr", fromlist=["UsdShade"]).UsdShade
UsdPhysics.MaterialAPI.Apply(
    __import__("pxr", fromlist=["UsdShade"]).UsdShade.Material.Define(stage, mat_path).GetPrim())

# luces
light = UsdLux.DistantLight.Define(stage, Sdf.Path("/DistantLight"))
light.CreateIntensityAttr(distant)
if dome > 0:
    d = UsdLux.DomeLight.Define(stage, Sdf.Path("/DomeLight"))
    d.CreateIntensityAttr(dome)
    d.CreateColorAttr(Gf.Vec3f(0.9, 0.93, 1.0))
print(f"[TOP] dome={dome} distant={distant} z_cam={z_cam} auto_exp={auto_exp}", flush=True)

# laberinto
import world_loader
parent, bb2, floor_top = world_loader.load_laberinto(stage, simulation_app, mat_path=mat_path)

# cámara top-down nadir en (0,0,z_cam), identidad -> mira -Z (abajo). FOV ancho.
import omni.timeline
tl = omni.timeline.get_timeline_interface()
cam = UsdGeom.Camera.Define(stage, "/TopCam")
cxf = UsdGeom.Xformable(cam.GetPrim()); cxf.ClearXformOpOrder()
cxf.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, z_cam))
focal = 18.0
import math
hfov = math.radians(60.0)
h_ap = 2.0 * focal * math.tan(hfov / 2.0)
cam.GetFocalLengthAttr().Set(focal)
cam.GetHorizontalApertureAttr().Set(h_ap)
cam.GetVerticalApertureAttr().Set(h_ap)
cam.GetClippingRangeAttr().Set(Gf.Vec2f(0.1, 100.0))

tl.play()
for _ in range(10):
    simulation_app.update()

from isaacsim.sensors.camera import Camera
c = Camera(prim_path="/TopCam", resolution=(900, 900))
c.initialize()
for _ in range(15):
    simulation_app.update()
rgba = c.get_rgba()
rgb = np.asarray(rgba)[:, :, :3].astype(np.uint8)
import cv2
cv2.imwrite("/tmp/maze_top.png", cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
# stats de color: azul (B>>R), marrón/amarillo (R>>B)
b, g, r = rgb[:, :, 2].astype(int), rgb[:, :, 1].astype(int), rgb[:, :, 0].astype(int)
azul = ((b - r) > 30).mean() * 100
calido = ((r - b) > 30).mean() * 100
print(f"[TOP] mean_rgb=({r.mean():.0f},{g.mean():.0f},{b.mean():.0f}) "
      f"p5={np.percentile(rgb,5):.0f} p95={np.percentile(rgb,95):.0f} "
      f"azul%={azul:.2f} calido%={calido:.2f} floor_top={floor_top:.3f} -> /tmp/maze_top.png", flush=True)
simulation_app.close()
