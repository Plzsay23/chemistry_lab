"""Build chem_lab_scene.usda: mecanum + dual SO-101 robot in the Chemistry3D lab,
with a physics beaker lying on the floor ahead of the robot.

Runs anywhere with `pxr` (usd-core or Isaac Sim's python). Re-runnable.
    uv run --python 3.12 --with usd-core python build_chem_lab_scene.py
"""
import math
import os
import shutil

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

ROOT = os.path.dirname(os.path.abspath(__file__))
LAB = os.path.join(ROOT, "chem_lab", "lab.usd")
SCENE = os.path.join(ROOT, "chem_lab_scene.usda")
ROBOT_SRC = os.path.join(ROOT, "mecanum_room3.usd")
ROBOT = os.path.join(ROOT, "mecanum_dualarm.usd")   # copy with L_/R_ arm joint names

SPOT = (0.38, 2.30)             # lab-local: aisle between the two ScienceDesk01 benches (0.80 m clearance)
ROBOT_XY = (-6.2155, 3.1528)    # /moebius_mecanum_base translate in mecanum_room3.usd
ROBOT_YAW_DEG = 98.78           # base_link +X (forward) direction in world
WHEEL_Z = -2.7805               # lowest wheel point in mecanum_room3.usd
FLOOR_GAP = 0.001               # floor 1 mm below wheels -> no initial penetration
LAB_FLOOR = -0.88               # ST_Ground tiles z in lab.usd (/root/Lab translate z = -0.88)
ROBOT_H = 0.45                  # meshes reaching below floor+this get colliders
WHEELS = ("left_front", "left_rear", "right_front", "right_rear")
WHEEL_R = 0.05                  # wheel link origin = wheel centre

# both arms have identical joint names; the articulation controller addresses joints by name
ARM_PREFIX = {"/so101_new_calib": "L_", "/so101_new_calib_0": "R_"}  # +Y / -Y side of base_link

BEAKER_SRC = "/root/Lab/ST_Acc_BechersGlass03_mo"   # 6.8 cm dia x 7.55 cm, pivot at bottom centre, axis +Z
BEAKER_MESH = "ST_Acc_BechersGlass03_md"
BEAKER_GLASS = "/ChemLab/materials/ST_Acc_ScienceGlass01"
BEAKER_AHEAD = 1.0              # m ahead of base_link along its +X
BEAKER_R, BEAKER_H = 0.0341, 0.0755
BEAKER_MASS = 0.05              # kg, small glass beaker


def build_robot_copy():
    """mecanum_room3.usd -> mecanum_dualarm.usd with arm joints renamed L_*/R_*."""
    shutil.copyfile(ROBOT_SRC, ROBOT)
    layer = Sdf.Layer.FindOrOpen(ROBOT)
    renames = {}
    edit = Sdf.BatchNamespaceEdit()
    for arm, prefix in ARM_PREFIX.items():
        for spec in list(layer.GetPrimAtPath(arm + "/joints").nameChildren):
            new = spec.path.GetParentPath().AppendChild(prefix + spec.name)
            edit.Add(spec.path, new)
            renames[spec.path] = new
    if not layer.Apply(edit):
        raise RuntimeError("joint rename failed")

    # Apply() does not retarget relationships (e.g. isaac:physics:robotJoints) or connections
    def retarget(spec):
        for child in spec.nameChildren:
            for rel in child.relationships:
                for old, new in renames.items():
                    rel.targetPathList.ReplaceItemEdits(old, new)
            for attr in child.attributes:
                for old, new in renames.items():
                    attr.connectionPathList.ReplaceItemEdits(old, new)
            retarget(child)

    retarget(layer.pseudoRoot)
    layer.Save()
    return len(renames)


n_joints = build_robot_copy()

# 1) lab.usd geometry is in metres but its layer says metersPerUnit=0.01 -> Kit would shrink it 100x
lab_stage = Usd.Stage.Open(LAB, Usd.Stage.LoadNone)
if UsdGeom.GetStageMetersPerUnit(lab_stage) != 1.0:
    UsdGeom.SetStageMetersPerUnit(lab_stage, 1.0)
    lab_stage.GetRootLayer().Save()

# floor extent + floor-level obstacle meshes (lab coords)
bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"])
tiles, obstacles = [], []
for p in lab_stage.Traverse():
    path = str(p.GetPath())
    if not p.IsA(UsdGeom.Mesh) or not path.startswith("/root/Lab/"):
        continue
    r = bc.ComputeWorldBound(p).ComputeAlignedRange()
    mn, mx = r.GetMin(), r.GetMax()
    top = path.split("/")[3]
    if "Ground" in top and abs(mn[2] - LAB_FLOOR) < 0.02 and abs(mx[2] - LAB_FLOOR) < 0.02:
        tiles.append((mn[0], mn[1], mx[0], mx[1]))
    elif mn[2] < LAB_FLOOR + ROBOT_H and mx[2] > LAB_FLOOR + 0.03 and not top.startswith("Campus_Corridor"):
        obstacles.append(path)

# 2) scene layer: robot file as sublayer (its links reference root-level /visuals,/colliders,/meshes)
if os.path.exists(SCENE):
    os.remove(SCENE)
layer = Sdf.Layer.CreateNew(SCENE)
layer.subLayerPaths.append("./" + os.path.basename(ROBOT))
st = Usd.Stage.Open(layer, Usd.Stage.LoadNone)
st.SetEditTarget(layer)
UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(st, 1.0)
st.SetDefaultPrim(st.GetPrimAtPath("/World"))
layer.customLayerData = {"comment": "built by build_chem_lab_scene.py"}

for p in ("/Environment/has_env", "/Environment/Fire", "/Environment/Smoke"):
    st.OverridePrim(p).SetActive(False)          # Hospital + fire/smoke placed for the hospital

# 3) lab, moved so SPOT lands under the robot and the floor meets the wheels
off = Gf.Vec3d(ROBOT_XY[0] - SPOT[0], ROBOT_XY[1] - SPOT[1], WHEEL_Z - FLOOR_GAP - (LAB_FLOOR + 0.88))
lab = st.DefinePrim("/ChemLab", "Xform")
lab.GetReferences().AddReference("./chem_lab/lab.usd", "/root/Lab")
lab.GetAttribute("xformOp:translate").Set(off)

# 4) colliders: lab ships with almost none (3 desks only)
n_new = 0
for path in obstacles:
    prim = st.GetPrimAtPath("/ChemLab" + path[len("/root/Lab"):])
    if not prim or prim.HasAPI(UsdPhysics.CollisionAPI):
        continue
    UsdPhysics.CollisionAPI.Apply(prim)
    UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr(UsdPhysics.Tokens.none)
    n_new += 1

x0 = min(t[0] for t in tiles)
y0 = min(t[1] for t in tiles)
x1 = max(t[2] for t in tiles)
y1 = max(t[3] for t in tiles)
floor_z = WHEEL_Z - FLOOR_GAP
fc = UsdGeom.Cube.Define(st, "/ChemLabPhysics/FloorCollider")
fc.CreateSizeAttr(1.0)
fc.AddTranslateOp().Set(Gf.Vec3d((x0 + x1) / 2 + off[0], (y0 + y1) / 2 + off[1], floor_z - 0.05))
fc.AddScaleOp().Set(Gf.Vec3f(x1 - x0 + 2.0, y1 - y0 + 2.0, 0.1))
fc.CreateVisibilityAttr(UsdGeom.Tokens.invisible)
UsdPhysics.CollisionAPI.Apply(fc.GetPrim())

# 5) wheel contact spheres: PhysX ignores the robot's instanced wheel Cylinder colliders
#    (GUI collider view 2026-09-14: no wheel colliders, wheels sank until the chassis box hit the floor)
for w in WHEELS:
    s = UsdGeom.Sphere.Define(st, f"/moebius_mecanum_base/{w}_wheel_link/wheel_contact")
    s.CreateRadiusAttr(WHEEL_R)
    s.CreatePurposeAttr(UsdGeom.Tokens.guide)
    UsdPhysics.CollisionAPI.Apply(s.GetPrim())

# 6) beaker lying on the floor ahead of the robot, axis along the robot's forward direction
yaw = math.radians(ROBOT_YAW_DEG)
fwd = Gf.Vec3d(math.cos(yaw), math.sin(yaw), 0.0)
centre = Gf.Vec3d(ROBOT_XY[0], ROBOT_XY[1], floor_z + BEAKER_R + 0.001) + fwd * BEAKER_AHEAD
pivot = centre - fwd * (BEAKER_H / 2)            # source pivot is the bottom centre
beaker = st.DefinePrim("/Beaker", "Xform")
beaker.GetReferences().AddReference("./chem_lab/lab.usd", BEAKER_SRC)
beaker.GetAttribute("xformOp:translate").Set(pivot)
beaker.GetAttribute("xformOp:rotateXYZ").Set(Gf.Vec3f(-90.0, 0.0, ROBOT_YAW_DEG - 90.0))  # local +Z -> fwd
UsdPhysics.RigidBodyAPI.Apply(beaker)
UsdPhysics.MassAPI.Apply(beaker).CreateMassAttr(BEAKER_MASS)

mesh = st.GetPrimAtPath(f"/Beaker/{BEAKER_MESH}")
UsdPhysics.CollisionAPI.Apply(mesh)
UsdPhysics.MeshCollisionAPI.Apply(mesh).CreateApproximationAttr(UsdPhysics.Tokens.convexHull)
binding = UsdShade.MaterialBindingAPI.Apply(mesh)
binding.Bind(UsdShade.Material(st.GetPrimAtPath(BEAKER_GLASS)))   # source binding points outside the reference
grip = UsdShade.Material.Define(st, "/PhysicsMaterials/BeakerGrip")
grip_api = UsdPhysics.MaterialAPI.Apply(grip.GetPrim())
grip_api.CreateStaticFrictionAttr(1.0)
grip_api.CreateDynamicFrictionAttr(0.9)
grip_api.CreateRestitutionAttr(0.0)
binding.Bind(grip, UsdShade.Tokens.weakerThanDescendants, "physics")

layer.Save()
print(f"wrote {ROBOT} ({n_joints} arm joints renamed)")
print(f"wrote {SCENE}")
print(f"  lab offset {tuple(round(v, 4) for v in off)}  floor z {floor_z:.4f}")
print(f"  colliders added to {n_new} lab meshes (+ floor box {x1 - x0 + 2:.1f} x {y1 - y0 + 2:.1f} m)")
print(f"  beaker centre {tuple(round(v, 4) for v in centre)} ({BEAKER_AHEAD} m ahead, lying along robot +X)")
