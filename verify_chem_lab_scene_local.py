"""Offline checks for chem_lab_scene.usda (no Isaac Sim needed).
    uv run --python 3.12 --with usd-core python verify_chem_lab_scene_local.py
"""
import math
import os

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

ROOT = os.path.dirname(os.path.abspath(__file__))
st = Usd.Stage.Open(os.path.join(ROOT, "chem_lab_scene.usda"), Usd.Stage.LoadNone)
ok = True


def check(cond, msg):
    global ok
    ok &= bool(cond)
    print(("  PASS " if cond else "  FAIL ") + msg)


lab_mpu = Sdf.Layer.FindOrOpen(os.path.join(ROOT, "chem_lab", "lab.usd")).pseudoRoot.GetInfo("metersPerUnit")
check(lab_mpu == 1.0, f"lab.usd metersPerUnit = {lab_mpu}")
check(UsdGeom.GetStageMetersPerUnit(st) == 1.0 and UsdGeom.GetStageUpAxis(st) == "Z", "scene Z-up, metres")
check(list(st.GetRootLayer().subLayerPaths) == ["./mecanum_dualarm.usd"], f"robot sublayer {list(st.GetRootLayer().subLayerPaths)}")
for p in ("/Environment/has_env", "/Environment/Fire", "/Environment/Smoke"):
    check(not st.GetPrimAtPath(p).IsActive(), f"{p} inactive")
check(st.GetPrimAtPath("/moebius_mecanum_base/base_footprint").HasAPI(UsdPhysics.ArticulationRootAPI),
      "robot articulation root present")
lab_children = len(st.GetPrimAtPath("/ChemLab").GetChildren())
check(lab_children > 100, f"lab composed ({lab_children} children)")

# arm joint names: unique, L_/R_ prefixed, and every relationship target still resolves
joints = [p for p in st.Traverse() if p.IsA(UsdPhysics.Joint) and str(p.GetPath()).startswith(("/moebius", "/so101"))]
movable = [p.GetName() for p in joints if not p.IsA(UsdPhysics.FixedJoint)]
dups = sorted({n for n in movable if movable.count(n) > 1})
check(not dups, f"movable joint names unique ({len(movable)}) {dups}")
arm_names = sorted(n for n in movable if n.startswith(("L_", "R_")))
check(len(arm_names) == 12, f"arm joints {arm_names}")
dangling = []
for p in st.Traverse():
    if not str(p.GetPath()).startswith(("/moebius", "/so101")):
        continue
    for r in p.GetRelationships():
        for t in r.GetTargets():
            if t.IsPrimPath() and not st.GetPrimAtPath(t):
                dangling.append(f"{r.GetPath()} -> {t}")
check(not dangling, f"no dangling robot relationship targets {dangling[:3]}")

bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"])
rb = bc.ComputeWorldBound(st.GetPrimAtPath("/moebius_mecanum_base")).ComputeAlignedRange()
for arm in ("/so101_new_calib", "/so101_new_calib_0"):
    rb.UnionWith(bc.ComputeWorldBound(st.GetPrimAtPath(arm)).ComputeAlignedRange())
wheel_z = bc.ComputeWorldBound(st.GetPrimAtPath("/moebius_mecanum_base/left_front_wheel_link")).ComputeAlignedRange().GetMin()[2]
tile = next(p for p in st.GetPrimAtPath("/ChemLab").GetChildren() if "Ground" in p.GetName())
floor_z = bc.ComputeWorldBound(tile).ComputeAlignedRange().GetMin()[2]
fc_top = bc.ComputeWorldBound(st.GetPrimAtPath("/ChemLabPhysics/FloorCollider")).ComputeAlignedRange().GetMax()[2]
print(f"  wheel bottom z {wheel_z:.4f} | visual floor z {floor_z:.4f} | floor collider top {fc_top:.4f}")
check(0 <= wheel_z - floor_z < 0.01 and abs(fc_top - floor_z) < 1e-4, "wheels sit on the lab floor (gap < 1 cm)")

xc0 = UsdGeom.XformCache()
for w in ("left_front", "left_rear", "right_front", "right_rear"):
    sp = st.GetPrimAtPath(f"/moebius_mecanum_base/{w}_wheel_link/wheel_contact")
    ok_s = bool(sp) and sp.IsA(UsdGeom.Sphere) and sp.HasAPI(UsdPhysics.CollisionAPI)
    r = UsdGeom.Sphere(sp).GetRadiusAttr().Get() if ok_s else None
    c = xc0.GetLocalToWorldTransform(sp).ExtractTranslation() if ok_s else Gf.Vec3d(0)
    link_c = xc0.GetLocalToWorldTransform(st.GetPrimAtPath(f"/moebius_mecanum_base/{w}_wheel_link")).ExtractTranslation()
    check(ok_s and abs(r - 0.05) < 1e-6 and (c - link_c).GetLength() < 1e-6 and 0 <= c[2] - r - floor_z < 0.01,
          f"{w} wheel contact sphere (r {r}, bottom-floor {(c[2] - (r or 0) - floor_z) * 1000:.1f} mm)")

lab_colliders, hits, n_col = [], [], 0
for p in Usd.PrimRange(st.GetPrimAtPath("/ChemLab")):
    if not p.HasAPI(UsdPhysics.CollisionAPI):
        continue
    n_col += 1
    r = bc.ComputeWorldBound(p).ComputeAlignedRange()
    lab_colliders.append((str(p.GetPath()), r))
    if all(r.GetMin()[k] < rb.GetMax()[k] and r.GetMax()[k] > rb.GetMin()[k] + (0.005 if k == 2 else 0) for k in range(3)):
        hits.append(str(p.GetPath()))
check(n_col > 50, f"{n_col} collider meshes in lab")
check(not hits, f"robot bbox overlaps no lab collider {hits[:3]}")

off = st.GetPrimAtPath("/ChemLab").GetAttribute("xformOp:translate").Get()
c = rb.GetMidpoint() - Gf.Vec3d(off[0], off[1], 0)
check(-3.5 < c[0] < 6.0 and -7.36 < c[1] < 10.29, f"robot centre in classroom interior (lab-local {c[0]:.2f}, {c[1]:.2f})")

# beaker: physics, lying along the robot's forward axis, on the floor, clear of everything
bk = st.GetPrimAtPath("/Beaker")
bmesh = st.GetPrimAtPath("/Beaker/ST_Acc_BechersGlass03_md")
check(bk.HasAPI(UsdPhysics.RigidBodyAPI) and abs(UsdPhysics.MassAPI(bk).GetMassAttr().Get() - 0.05) < 1e-6, "beaker rigid body, 0.05 kg")
check(bmesh.HasAPI(UsdPhysics.CollisionAPI)
      and UsdPhysics.MeshCollisionAPI(bmesh).GetApproximationAttr().Get() == "convexHull", "beaker convexHull collider")
vis, _ = UsdShade.MaterialBindingAPI(bmesh).ComputeBoundMaterial()
phys, _ = UsdShade.MaterialBindingAPI(bmesh).ComputeBoundMaterial(materialPurpose="physics")
check(vis and vis.GetPrim().IsValid() and "ScienceGlass" in vis.GetPath().name, f"beaker glass material {vis.GetPath() if vis else None}")
check(phys and UsdPhysics.MaterialAPI(phys.GetPrim()).GetStaticFrictionAttr().Get() == 1.0, "beaker grip physics material")

xc = UsdGeom.XformCache()
M = xc.GetLocalToWorldTransform(bmesh)
axis = M.TransformDir(Gf.Vec3d(0, 0, 1)).GetNormalized()
bl = xc.GetLocalToWorldTransform(st.GetPrimAtPath("/moebius_mecanum_base/base_link"))
fwd = bl.TransformDir(Gf.Vec3d(1, 0, 0)).GetNormalized()
br = bc.ComputeWorldBound(bmesh).ComputeAlignedRange()
bcen = br.GetMidpoint()
ahead = Gf.Dot(bcen - bl.ExtractTranslation(), fwd)
side = Gf.Dot(bcen - bl.ExtractTranslation(), bl.TransformDir(Gf.Vec3d(0, 1, 0)).GetNormalized())
print(f"  beaker centre {tuple(round(v, 3) for v in bcen)} size {tuple(round(v, 3) for v in br.GetSize())} "
      f"axis·fwd {Gf.Dot(axis, fwd):.3f} ahead {ahead:.3f} m side {side:+.3f} m bottom-floor {(br.GetMin()[2] - floor_z) * 1000:.1f} mm")
check(abs(Gf.Dot(axis, fwd)) > 0.99 and abs(axis[2]) < 0.01, "beaker lying, axis along robot forward")
check(0 < br.GetMin()[2] - floor_z < 0.005, "beaker resting on floor (gap < 5 mm)")
check(abs(ahead - 1.0) < 0.01 and abs(side) < 0.01, "beaker 1.0 m straight ahead of base_link")
bhits = [p for p, r in lab_colliders
         if all(r.GetMin()[k] < br.GetMax()[k] and r.GetMax()[k] > br.GetMin()[k] + (0.002 if k == 2 else 0) for k in range(3))]
check(not bhits, f"beaker overlaps no lab collider {bhits[:3]}")

total, missing = 0, set()
for p in Usd.PrimRange(st.GetPrimAtPath("/ChemLab")):
    for a in p.GetAttributes():
        if a.GetTypeName() == Sdf.ValueTypeNames.Asset:
            v = a.Get()
            if v and v.path and not v.path.endswith(".mdl") and "/" in v.path:
                total += 1
                if not v.resolvedPath:
                    missing.add(v.path)
check(not missing, f"lab textures resolve ({total - len(missing)}/{total}) {sorted(missing)[:3]}")
used = sorted({os.path.basename(l.identifier) for l in st.GetUsedLayers() if not l.anonymous})
print("  layers used:", used[:12])
print("\nRESULT:", "ALL PASS" if ok else "SOME CHECKS FAILED")
