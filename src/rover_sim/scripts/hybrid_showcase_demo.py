from isaacsim import SimulationApp
app = SimulationApp({"headless": True, "renderer": "RaytracedLighting"})
import numpy as np, random, os, omni.kit.commands
import omni.replicator.core as rep
from isaacsim.core.api import World
from isaacsim.core.api.objects import GroundPlane, FixedSphere, FixedCuboid
from isaacsim.core.prims import Articulation
from omni.usd import get_context
from pxr import UsdPhysics, UsdGeom, UsdLux, UsdShade, Sdf, Gf

HYB = os.environ.get("HYB", "1") == "1"
world = World(stage_units_in_meters=1.0, physics_dt=1/120.0, rendering_dt=1/30.0)
stage = get_context().get_stage()

sun = UsdLux.DistantLight.Define(stage, "/World/Sun")
sun.CreateIntensityAttr(3500); sun.CreateColorAttr(Gf.Vec3f(1.0, 0.62, 0.42))
UsdGeom.Xformable(sun.GetPrim()).AddRotateXYZOp().Set(Gf.Vec3f(-28, 45, 0))
dome = UsdLux.DomeLight.Define(stage, "/World/Sky")
dome.CreateIntensityAttr(350); dome.CreateColorAttr(Gf.Vec3f(0.85, 0.5, 0.35))
mars = np.array([0.50, 0.25, 0.13])
GroundPlane("/World/floor", z_position=0.0, color=mars)

heights = [0.03, 0.04, 0.06, 0.08, 0.10, 0.13, 0.16, 0.20, 0.23, 0.26]
x = 1.5; random.seed(3)
for k, h in enumerate(heights):
    y = random.uniform(-0.25, 0.25)
    if k % 3 == 2:
        FixedCuboid(f"/World/bump_{k}", position=np.array([x, y, h*0.5]),
                    scale=np.array([0.45, 1.2, h]), color=mars*1.02)
    else:
        FixedSphere(f"/World/bump_{k}", position=np.array([x, y, h*0.15]),
                    radius=h*1.4, color=mars*1.02)
    x -= (1.3 + h*3)

s, cfg = omni.kit.commands.execute("URDFCreateImportConfig")
cfg.merge_fixed_joints=False; cfg.fix_base=False; cfg.make_default_prim=False
omni.kit.commands.execute("URDFParseAndImportFile",
    urdf_path="/workspace/robot/karasimsek_isaac.urdf", import_config=cfg)
for _ in range(8): app.update()
root = [str(p.GetPath()) for p in stage.Traverse() if p.HasAPI(UsdPhysics.ArticulationRootAPI)][0]
rtop = root.rsplit("/",1)[0]

import xml.etree.ElementTree as ET
_tree = ET.parse("/workspace/robot/karasimsek_isaac.urdf")
_mats = {}
for _m in _tree.getroot().findall("material"):
    _c = _m.find("color")
    if _c is not None:
        _mats[_m.get("name")] = [float(v) for v in _c.get("rgba").split()[:3]]
_link_color = {}
for _l in _tree.getroot().findall("link"):
    _v = _l.find("visual")
    if _v is None: continue
    _vm = _v.find("material")
    if _vm is None: continue
    _c = _vm.find("color")
    rgb = [float(v) for v in _c.get("rgba").split()[:3]] if _c is not None else _mats.get(_vm.get("name"))
    if rgb: _link_color[_l.get("name")] = rgb
print(f"URDF materials found for {len(_link_color)} links")
def paint_link(link, rgb):
    m = UsdShade.Material.Define(stage, f"/World/mat_{link}")
    sh = UsdShade.Shader.Define(stage, f"/World/mat_{link}/s")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.5)
    m.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    c = 0
    for p in stage.Traverse():
        pp = str(p.GetPath())
        if "/visuals/" in pp and p.GetTypeName() == "Mesh" and link.lower() in pp.lower():
            UsdShade.MaterialBindingAPI.Apply(p).Bind(m); c += 1
    return c
_total = 0
for _link, _rgb in _link_color.items():
    _total += paint_link(_link, _rgb)
print(f"painted {_total} meshes from URDF materials")

world.reset()
r = Articulation(root); r.initialize()
r.set_world_poses(positions=np.array([[6.5, 0.0, 0.45]]))
n = list(r.dof_names)
W = [i for i,x_ in enumerate(n) if "Wheel" in x_]
S = [i for i,x_ in enumerate(n) if "Steer" in x_]
iL, iR = n.index("leftjoint"), n.index("rightjoint")
kps = np.zeros((1,len(n))); kds = np.zeros((1,len(n)))
for i in W: kds[0,i] = 800.0
for i in S: kps[0,i], kds[0,i] = 5000.0, 200.0
r.set_gains(kps=kps, kds=kds)
try: r.set_max_efforts(np.full((1,len(n)), 120.0))
except Exception: pass

for _ in range(360):
    q0 = r.get_joint_positions()[0]
    tau0 = float(np.clip(-400.0*(q0[iL]+q0[iR]), -60, 60))
    r.set_joint_efforts(np.array([[tau0, tau0]]), joint_indices=np.array([iL, iR]))
    world.step(render=False)

cam = rep.create.camera(position=(8.6, 2.1, 0.65), look_at=(-2.0, 0, 0.30), focal_length=35.0)
rprod = rep.create.render_product(cam, (1920,1080))
wr = rep.WriterRegistry.get("BasicWriter")
wr.initialize(output_dir="/workspace/frames", rgb=True); wr.attach([rprod])

p0 = r.get_world_poses()[0][0].copy(); emax = 0.0
stall = 0; last_x = None; stopped = False
for t in range(750):
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
print(f"MOD={'ON' if HYB else 'OFF'}  distance={-d[0]:.2f} m  max|e|={emax:.3f} rad")
print("=== SHOWCASE RUN COMPLETE ===")
app.close()
