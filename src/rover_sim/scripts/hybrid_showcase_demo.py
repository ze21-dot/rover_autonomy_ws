from isaacsim import SimulationApp
app = SimulationApp({"headless": True, "renderer": "RaytracedLighting"})
import numpy as np, random, os, omni.kit.commands
import omni.replicator.core as rep
import xml.etree.ElementTree as ET
from isaacsim.core.api import World
from isaacsim.core.api.objects import GroundPlane
from isaacsim.core.prims import Articulation
from isaacsim.core.utils.stage import add_reference_to_stage
from omni.usd import get_context
from pxr import UsdPhysics, UsdGeom, UsdLux, UsdShade, Sdf, Gf, Usd

HYB = os.environ.get("HYB", "1") == "1"
N_FRAMES = int(os.environ.get("FRAMES", "750"))
SPAWN_X = float(os.environ.get("SPAWN_X", "6.5"))

world = World(stage_units_in_meters=1.0, physics_dt=1/120.0, rendering_dt=1/30.0)
stage = get_context().get_stage()
sun = UsdLux.DistantLight.Define(stage, "/World/Sun")
sun.CreateIntensityAttr(3400); sun.CreateColorAttr(Gf.Vec3f(1.0, 0.97, 0.94))
UsdGeom.Xformable(sun.GetPrim()).AddRotateXYZOp().Set(Gf.Vec3f(-28, 45, 0))
dome = UsdLux.DomeLight.Define(stage, "/World/Sky")
dome.CreateIntensityAttr(300); dome.CreateColorAttr(Gf.Vec3f(0.82, 0.50, 0.33))
mars = np.array([0.50, 0.25, 0.13])
GroundPlane("/World/floor", z_position=-1.35, color=mars*0.9)  # safety net below terrain min
wrap = UsdGeom.Xform.Define(stage, "/World/terrain_wrap")
add_reference_to_stage("/workspace/assets/terrain.usd", "/World/terrain_wrap/terrain")
tp = stage.GetPrimAtPath("/World/terrain_wrap/terrain")
# referans icindeki converter op'larini olc, tersini ust-Xform'a yaz
cache0 = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"])
b0 = cache0.ComputeWorldBound(tp).ComputeAlignedRange()
span_x = float(b0.GetMax()[0]-b0.GetMin()[0])
corr = 60.0/span_x if span_x > 1e-6 else 100.0
wxf = UsdGeom.Xformable(wrap.GetPrim())
wxf.AddRotateXOp().Set(-90.0)   # OBJ Y-up -> stage Z-up
wxf.AddScaleOp().Set(Gf.Vec3f(corr, corr, corr))
print(f"terrain wrapper scale correction: x{corr:.1f} (raw span {span_x:.2f} m -> 60 m)")
mesh_n = 0
for pp in Usd.PrimRange(tp):
    if pp.GetTypeName() == "Mesh":
        UsdPhysics.CollisionAPI.Apply(pp)
        UsdPhysics.MeshCollisionAPI.Apply(pp).CreateApproximationAttr("meshSimplification")
        UsdGeom.Gprim(pp).CreateDisplayColorAttr([Gf.Vec3f(0.42, 0.17, 0.08)])
        mesh_n += 1
print(f"terrain meshes with collision: {mesh_n}")

s, cfg = omni.kit.commands.execute("URDFCreateImportConfig")
cfg.merge_fixed_joints=False; cfg.fix_base=False; cfg.make_default_prim=False
omni.kit.commands.execute("URDFParseAndImportFile",
    urdf_path="/workspace/robot/karasimsek_isaac.urdf", import_config=cfg)
for _ in range(8): app.update()

rock_native = {0:1.754, 1:1.840, 2:1.784, 3:1.724, 4:1.840}
target_heights = [0.06, 0.08, 0.10, 0.13, 0.16, 0.19, 0.22, 0.26, 0.30, 0.35]
x = 21.0; CY = 0.0; random.seed(3)
for k, th in enumerate(target_heights):
    y = CY + random.uniform(-0.30, 0.30)
    rp_path = f"/World/rock_{k}"
    add_reference_to_stage(f"/workspace/assets/rock{k%5}.usd", rp_path)
    rp = stage.GetPrimAtPath(rp_path)
    UsdPhysics.CollisionAPI.Apply(rp)
    UsdPhysics.MeshCollisionAPI.Apply(rp).CreateApproximationAttr("meshSimplification")
    sc = th / rock_native[k%5]
    xf = UsdGeom.Xformable(rp)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(x, y, th*0.32 + 2.26))  # corridor ground ~-0.11  # settle onto local terrain  # above local terrain, will settle
    xf.AddScaleOp().Set(Gf.Vec3f(sc, sc, sc))
    for pp in Usd.PrimRange(rp):
        if pp.GetTypeName() == "Mesh":
            UsdGeom.Gprim(pp).CreateDisplayColorAttr([Gf.Vec3f(0.36, 0.18, 0.10)])
    x -= (1.2 + th*2.0)
print(f"rocks placed: {len(target_heights)}")

deinst = 0
for p in stage.Traverse():
    if p.IsInstance():
        p.SetInstanceable(False); deinst += 1
for _ in range(4): app.update()
print(f"instances disabled: {deinst}")

tree = ET.parse("/workspace/robot/karasimsek_isaac.urdf")
mass_total = sum(float(m.get("value")) for m in tree.getroot().findall("link/inertial/mass"))
wheel_vel = None
for j in tree.getroot().findall("joint"):
    if j.get("name") == "FLWheelJoint":
        wheel_vel = float(j.find("limit").get("velocity"))
print(f"CHECK mass_total={mass_total:.2f} kg | wheel_vel_limit={wheel_vel} rad/s | cmd 4.5 within: {4.5<=wheel_vel}")
assert abs(mass_total-59.07) < 1.0 and 4.5 <= wheel_vel


root = [str(p.GetPath()) for p in stage.Traverse() if p.HasAPI(UsdPhysics.ArticulationRootAPI)][0]
world.reset()
r = Articulation(root); r.initialize()
for _ in range(4): app.update()
link_color = {}
for l in tree.getroot().findall("link"):
    v = l.find("visual")
    if v is None: continue
    vm = v.find("material")
    if vm is None: continue
    c = vm.find("color")
    if c is not None:
        link_color[l.get("name")] = [float(q)**2.2 for q in c.get("rgba").split()[:3]]
print(f"URDF materials found for {len(link_color)} links; sample={list(link_color.items())[:2]}")
inst_now = [str(p.GetPath()) for p in stage.Traverse() if p.IsInstance()]
print(f"INSTANCES still active at paint time: {len(inst_now)}; first3={inst_now[:3]}")
cl = stage.GetPrimAtPath("/karasimsek_base/chassis_link")
sub = [(str(p.GetPath()), p.GetTypeName(), p.IsInstance()) for p in Usd.PrimRange(cl)][:10]
print(f"CHASSIS subtree: {sub}")
all_meshes = [str(p.GetPath()) for p in stage.Traverse() if p.GetTypeName()=="Mesh"]
print(f"MESH COUNT at paint time: {len(all_meshes)}; first5={all_meshes[:5]}")
total = 0
for link, rgb in link_color.items():
    for p in stage.Traverse(Usd.TraverseInstanceProxies()):
        if p.GetTypeName() == "Mesh" and link.lower() in str(p.GetPath()).lower():
            UsdShade.MaterialBindingAPI.Apply(p).UnbindAllBindings()
            UsdGeom.Gprim(p).CreateDisplayColorAttr([Gf.Vec3f(*rgb)])
            total += 1
print(f"painted {total} meshes")

SPAWN_Y = float(os.environ.get("SPAWN_Y", "0.0"))
r.set_world_poses(positions=np.array([[SPAWN_X, SPAWN_Y, 3.26]]))  # drop onto corridor (ground ~-0.11..0.9)  # drop onto terrain
n = list(r.dof_names)
W = [i for i,z in enumerate(n) if "Wheel" in z]
S = [i for i,z in enumerate(n) if "Steer" in z]
iL, iR = n.index("leftjoint"), n.index("rightjoint")
kps = np.zeros((1,len(n))); kds = np.zeros((1,len(n)))
for i in W: kds[0,i] = 800.0
for i in S: kps[0,i], kds[0,i] = 5000.0, 200.0
r.set_gains(kps=kps, kds=kds)

for _ in range(360):
    q0 = r.get_joint_positions()[0]
    tau0 = float(np.clip(-400.0*(q0[iL]+q0[iR]), -60, 60))
    r.set_joint_efforts(np.array([[tau0, tau0]]), joint_indices=np.array([iL, iR]))
    world.step(render=False)
wp = r.get_world_poses()[0][0]
print(f"WORLD rover after settle: x={float(wp[0]):.2f} y={float(wp[1]):.2f} z={float(wp[2]):.2f}")
tcache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"])
tb = tcache.ComputeWorldBound(stage.GetPrimAtPath("/World/terrain_wrap")).ComputeAlignedRange()
print(f"WORLD terrain bbox: min={tuple(round(v,2) for v in tb.GetMin())} max={tuple(round(v,2) for v in tb.GetMax())}")
print(f"settle done: qL+qR={float(r.get_joint_positions()[0][iL]+r.get_joint_positions()[0][iR]):.4f}")

cam = rep.create.camera(position=(31.5, 5.0, 4.86), look_at=(18.0, 0.0, 2.26), focal_length=24.0)
rprod = rep.create.render_product(cam, (1920,1080))
for _ in range(240):
    qw = r.get_joint_positions()[0]
    tw = float(np.clip(-400.0*(qw[iL]+qw[iR]), -60, 60))
    r.set_joint_efforts(np.array([[tw, tw]]), joint_indices=np.array([iL, iR]))
    world.step(render=True)
wr = rep.WriterRegistry.get("BasicWriter")
wr.initialize(output_dir="/workspace/frames", rgb=True); wr.attach([rprod])

p0 = r.get_world_poses()[0][0].copy(); emax = 0.0
stall = 0; last_x = None; stopped = False
for t in range(N_FRAMES):
    q = r.get_joint_positions()[0]; v = r.get_joint_velocities()[0]
    e = q[iL]+q[iR]; emax = max(emax, abs(float(e)))
    if HYB:
        tau = float(np.clip(-(400.0*e + 20.0*(v[iL]+v[iR])), -60, 60))
        r.set_joint_efforts(np.array([[tau, tau]]), joint_indices=np.array([iL, iR]))
    vel = -4.5 * min(1.0, t/120.0)
    if stopped: vel = 0.0
    r.set_joint_velocity_targets(np.full((1,len(W)), vel), joint_indices=np.array(W))
    r.set_joint_position_targets(np.zeros((1,len(S))), joint_indices=np.array(S))
    cx = float(r.get_world_poses()[0][0][0])
    if t > 150 and last_x is not None and abs(cx-last_x) < 0.0004: stall += 1
    else: stall = 0
    last_x = cx
    if stall > 100: stopped = True
    world.step(render=True)
d = r.get_world_poses()[0][0]-p0
print(f"MODE={'ON' if HYB else 'OFF'}  distance={-d[0]:.2f} m  max|e|={emax:.3f} rad")
print("=== SHOWCASE RUN COMPLETE ===")
app.close()
