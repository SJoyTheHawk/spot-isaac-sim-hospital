"""Dump body-relative transforms of Spot sensor prims from a USD file.

Run with Isaac Sim's bundled Python:

    cd ~/isaac-sim
    ./python.sh ws/export_tf_pose.py

Launches Kit headless to access the USD bindings.
"""

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": True})

import omni.usd
from pxr import Usd, UsdGeom, Gf

USD_PATH = "/home/lighthouse/ros2_ws/src/lighthouse/lighthouse_isaac_hospital_scene/isaac_hospital_scene_spot.usd"
BODY_PATH = "/World/spot/body"
SENSOR_PATHS = [
    "/World/spot/body/XT_32/PandarXT_32_10hz",
    "/World/spot/body/Camera_SG2_OX03CC_5200_GMSL2_H60YA",
    "/World/spot/body/rsd455",
    "/World/spot/rsd455",
]

ctx = omni.usd.get_context()
ctx.open_stage(USD_PATH)
simulation_app.update()
stage = ctx.get_stage()

body = stage.GetPrimAtPath(BODY_PATH)
if not body or not body.IsValid():
    spot = stage.GetPrimAtPath("/World/spot")
    print("body prim not found; children of /World/spot:")
    if spot and spot.IsValid():
        for c in spot.GetChildren():
            print(" ", c.GetPath(), c.GetTypeName())
    simulation_app.close()
    raise SystemExit(1)

bw = UsdGeom.Xformable(body).ComputeLocalToWorldTransform(Usd.TimeCode.Default())

for path in SENSOR_PATHS:
    p = stage.GetPrimAtPath(path)
    if not p or not p.IsValid():
        print(f"{path}: NOT FOUND")
        continue
    pw = UsdGeom.Xformable(p).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    rel = pw * bw.GetInverse()
    t = rel.ExtractTranslation()
    q = Gf.Rotation(rel.ExtractRotationMatrix()).GetQuat()
    im = q.GetImaginary()
    print(f"{path}")
    print(f"  trans (x,y,z) = ({t[0]:.6f}, {t[1]:.6f}, {t[2]:.6f})")
    print(f"  quat  (x,y,z,w) = ({im[0]:.6f}, {im[1]:.6f}, {im[2]:.6f}, {q.GetReal():.6f})")

simulation_app.close()

BODY_PATH = "/World/spot/body"
SENSOR_PATHS = [
    "/World/spot/body/XT_32/PandarXT_32_10hz",
    "/World/spot/body/Camera_SG2_OX03CC_5200_GMSL2_H60YA",
    "/World/spot/body/rsd455",
    "/World/spot/rsd455",
]

stage = Usd.Stage.Open(USD_PATH)
if stage is None:
    raise SystemExit(f"Could not open USD: {USD_PATH}")

body = stage.GetPrimAtPath(BODY_PATH)
if not body or not body.IsValid():
    spot = stage.GetPrimAtPath("/World/spot")
    print("body prim not found; children of /World/spot:")
    if spot and spot.IsValid():
        for c in spot.GetChildren():
            print(" ", c.GetPath(), c.GetTypeName())
    raise SystemExit(1)

bw = UsdGeom.Xformable(body).ComputeLocalToWorldTransform(Usd.TimeCode.Default())

for path in SENSOR_PATHS:
    p = stage.GetPrimAtPath(path)
    if not p or not p.IsValid():
        print(f"{path}: NOT FOUND")
        continue
    pw = UsdGeom.Xformable(p).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    rel = pw * bw.GetInverse()
    t = rel.ExtractTranslation()
    q = Gf.Rotation(rel.ExtractRotationMatrix()).GetQuat()
    im = q.GetImaginary()
    print(f"{path}")
    print(f"  trans (x,y,z) = ({t[0]:.6f}, {t[1]:.6f}, {t[2]:.6f})")
    print(f"  quat  (x,y,z,w) = ({im[0]:.6f}, {im[1]:.6f}, {im[2]:.6f}, {q.GetReal():.6f})")
