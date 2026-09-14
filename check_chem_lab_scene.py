"""Headless physics smoke test for chem_lab_scene.usda on the Isaac server (Isaac Sim 6.0.1).

    cd ~/Desktop/sj/ysj && conda activate isaac
    source ~/Desktop/sj/isaac_env.sh 1      # GPU1 = RTX PRO 5000
    ulimit -n 65535
    python check_chem_lab_scene.py 2>&1 | tee chem_lab_check.log

Pass = the robot base stays put on the lab floor for 3 s of physics.
Both samples come from PhysX (the pose before play is not a PhysX object yet).
The ROS2 ActionGraph warnings are expected: the ROS2 bridge extension is not loaded here.
"""
import os
import sys

from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

import omni.timeline
import omni.usd
from omni.physx import get_physx_interface
from pxr import Gf, UsdGeom

SCENE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "chem_lab_scene.usda")
BASE = "/moebius_mecanum_base/base_link"
WHEEL = "/moebius_mecanum_base/left_front_wheel_link"
FLOOR_Z = -2.7815  # floor top written by build_chem_lab_scene.py

ctx = omni.usd.get_context()
if not ctx.open_stage(SCENE):
    print("FAIL could not open", SCENE)
    app.close()
    sys.exit(1)
for _ in range(600):  # wait for textures/payloads
    app.update()
    msg, loaded, total = ctx.get_stage_loading_status()
    if total == 0 or loaded >= total:
        break
stage = ctx.get_stage()
print("stage:", stage.GetRootLayer().identifier, "| metersPerUnit", UsdGeom.GetStageMetersPerUnit(stage))
print("lab children:", len(stage.GetPrimAtPath("/ChemLab").GetChildren()))
usd_origin = UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath(BASE)).ExtractTranslation()

physx = get_physx_interface()


def pose(path):
    t = physx.get_rigidbody_transformation(path)
    return Gf.Vec3d(*t["position"]) if t.get("ret_val") else None


tl = omni.timeline.get_timeline_interface()
tl.play()
app.update()  # first physics step creates the PhysX objects
p0, w0 = pose(BASE), pose(WHEEL)
zs = []
for _ in range(180):  # ~3 s at 60 Hz
    app.update()
    p = pose(BASE)
    if p is not None:
        zs.append(p[2])
p1, w1 = pose(BASE), pose(WHEEL)
tl.stop()
app.update()

if p0 is None or p1 is None:
    print("FAIL could not read base_link pose from PhysX")
    app.close()
    sys.exit(1)


def fmt(v):
    return tuple(round(x, 4) for x in v)


dz, dxy = p1[2] - p0[2], ((p1[0] - p0[0]) ** 2 + (p1[1] - p0[1]) ** 2) ** 0.5
print(f"base_link USD origin     {fmt(usd_origin)}   (reference only, not compared)")
print(f"base_link PhysX step 1   {fmt(p0)}")
print(f"base_link PhysX step 181 {fmt(p1)}   dz={dz:+.4f} m  dxy={dxy:.4f} m  z range over run {min(zs) - p0[2]:+.4f}..{max(zs) - p0[2]:+.4f}")
if w0 is not None and w1 is not None:
    print(f"wheel centre z  step 1 {w0[2]:.4f} -> {w1[2]:.4f}  (floor {FLOOR_Z}, wheel radius 0.05)")
if p1[2] < FLOOR_Z - 0.1:
    print("FAIL robot fell through the floor")
elif abs(dz) > 0.005 or dxy > 0.01:
    print("WARN robot moved more than 5 mm / 1 cm - check contacts in the GUI")
else:
    print("PASS robot rests on the lab floor")
app.close()
