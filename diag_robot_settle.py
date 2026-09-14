"""A/B 진단: 로봇이 바닥에 가라앉는 게 실험실 씬 탓인지, 로봇 원본에서도 그런지 비교한다.

    cd ~/Desktop/sj/ysj && conda activate isaac
    source ~/Desktop/sj/isaac_env.sh 1
    ulimit -n 65535
    python diag_robot_settle.py chem_lab_scene.usda 2>&1 | tee diag_lab.log
    python diag_robot_settle.py mecanum_room3.usd   2>&1 | tee diag_hospital.log

0.5초마다 base_link 높이·기울기, 바퀴 4개 중심 높이, 가장 낮은 바퀴 바닥 - 기준 바닥을 찍는다.
"""
import math
import os
import sys
import time
import functools

print = functools.partial(print, flush=True)  # tee 로 파이프해도 바로 찍히게

from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

import omni.timeline
import omni.usd
from omni.physx import get_physx_interface
from pxr import Gf, UsdGeom

name = sys.argv[1] if len(sys.argv) > 1 else "chem_lab_scene.usda"
SCENE = os.path.join(os.path.dirname(os.path.abspath(__file__)), name)
BASE = "/moebius_mecanum_base/base_link"
WHEELS = [f"/moebius_mecanum_base/{w}_wheel_link" for w in ("left_front", "left_rear", "right_front", "right_rear")]
WHEEL_R = 0.05
FLOOR_Z = -2.7815 if name.startswith("chem_lab") else -2.7805  # 실험실: 빌드값 / 병원: 원래 바퀴 바닥

T0 = time.time()
print(f"[diag] opening {SCENE}")
ctx = omni.usd.get_context()
if not ctx.open_stage(SCENE):
    print("FAIL could not open", SCENE)
    app.close()
    sys.exit(1)
for i in range(1200):  # 병원은 S3 payload 라 오래 걸릴 수 있다
    app.update()
    _, loaded, total = ctx.get_stage_loading_status()
    if i % 60 == 0:
        print(f"[diag] loading {loaded}/{total}  {time.time() - T0:.0f}s")
    if total == 0 or loaded >= total:
        break
print(f"[diag] stage ready in {time.time() - T0:.0f}s, starting physics")

physx = get_physx_interface()


def pose(path):
    t = physx.get_rigidbody_transformation(path)
    if not t.get("ret_val"):
        return None, None
    q = t["rotation"]  # (x, y, z, w)
    return Gf.Vec3d(*t["position"]), Gf.Quatd(q[3], q[0], q[1], q[2])


def tilt_deg(q):
    up = Gf.Rotation(q).TransformDir(Gf.Vec3d(0, 0, 1))
    return math.degrees(math.acos(max(-1.0, min(1.0, up[2]))))


tl = omni.timeline.get_timeline_interface()
tl.play()
app.update()
print(f"{'t[s]':>5} {'base_z':>8} {'tilt':>6} {'dxy':>6} | " + " ".join(f"{w.split('/')[-1][:11]:>11}" for w in WHEELS) + " | lowest_bottom-floor")
b0, _ = pose(BASE)
for step in range(0, 301):
    if step % 30 == 0:
        b, bq = pose(BASE)
        wz = [pose(w)[0][2] if pose(w)[0] is not None else float("nan") for w in WHEELS]
        low = min(wz) - WHEEL_R - FLOOR_Z
        dxy = math.hypot(b[0] - b0[0], b[1] - b0[1])
        print(f"{step / 60:5.2f} {b[2]:8.4f} {tilt_deg(bq):6.2f} {dxy:6.3f} | " + " ".join(f"{z:11.4f}" for z in wz) + f" | {low * 100:+7.2f} cm")
    app.update()
tl.stop()
app.update()
app.close()
