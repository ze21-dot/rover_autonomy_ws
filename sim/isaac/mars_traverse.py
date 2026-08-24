"""
SuRover KARASIMSEK - Isaac Sim 6.0.1.0 Mars traverse (showcase + test matrix)
One scene, one seed, one camera. Only the rover mode changes between runs.

Parameters (environment variables):
  RUN     run name -> /workspace/runs/<RUN>/        (default: test)
  MODEL   rigid | revolute                          (default: rigid)
  MODE    rigid | passive | hybrid                  (default: rigid; forced rigid if MODEL=rigid)
  SPEED   wheel target, rad/s                       (default: 7.0)
  LANE    1 = lane keeping on, 0 = off              (default: 1)
  TOTAL   max physics steps (60 Hz)                 (default: 3600 = 60 s)
  CAP     capture every CAP steps                   (default: 5 -> 720 frames = 30 s @ 24 fps)
  X_END   course end, run stops when x >= X_END     (default: 110.0)
  KP KD TAU  hybrid PD gains                        (default: 400 / 40 / 40)
"""
from isaacsim import SimulationApp
app = SimulationApp({"headless": True, "renderer": "RayTracedLighting"})

import numpy as np, csv, os, math, json
import omni.kit.app
em = omni.kit.app.get_app().get_extension_manager()
em.set_extension_enabled_immediate("omni.replicator.core", True)
for _ in range(20): app.update()

import omni.usd, omni.replicator.core as rep
from pxr import Usd, UsdGeom, UsdPhysics, UsdShade, Sdf, Gf, UsdLux, PhysxSchema
from isaacsim.core.api import World
from isaacsim.core.prims import Articulation
from isaacsim.core.utils.stage import add_reference_to_stage

# ---------------- parameters ----------------
RUN     = os.environ.get("RUN", "test")
MODEL   = os.environ.get("MODEL", "rigid")
MODE    = os.environ.get("MODE", "rigid")
SPEED   = float(os.environ.get("SPEED", "7.0"))
LANE    = os.environ.get("LANE", "1") == "1"
TOTAL   = int(os.environ.get("TOTAL", "3600"))
CAP     = int(os.environ.get("CAP", "5"))
X_END   = float(os.environ.get("X_END", "110.0"))
KP      = float(os.environ.get("KP", "400"))
KD      = float(os.environ.get("KD", "40"))
TAU_MAX = float(os.environ.get("TAU", "40"))
if MODEL == "rigid": MODE = "rigid"
if MODE != "hybrid": KP = KD = TAU_MAX = 0.0
DT = 1.0/60.0
RAMP = 900                       # steps to reach SPEED (15 s)
X_SPAWN = 20.0
MODEL_PATH = {"rigid":    "/workspace/karasimsek_rigid.usd/karasimsek_rigid/karasimsek_rigid.usda",
              "revolute": "/workspace/karasimsek.usd/karasimsek/karasimsek.usda"}[MODEL]
RUN_DIR = f"/workspace/runs/{RUN}"
os.makedirs(f"{RUN_DIR}/frames", exist_ok=True)
print(f"[cfg] RUN={RUN} MODEL={MODEL} MODE={MODE} SPEED={SPEED} LANE={LANE} TOTAL={TOTAL} CAP={CAP} X_END={X_END}")

stage = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)

# ---------------- terrain field (fixed seed, identical for every run) ----------------
N, EXT = 361, 240.0
xs = np.linspace(-EXT/2, EXT/2, N)
X, Y = np.meshgrid(xs, xs, indexing="ij")
Z = np.zeros_like(X)

def gauss(cx, cy, h, sigma):
    return h*np.exp(-(((X-cx)**2 + (Y-cy)**2)/(2*sigma**2)))

# distant scenery hills (never inside the lane)
Z += 6.0*np.exp(-(((X+40)**2)/2500 + ((Y-70)**2)/2500))
Z += 4.5*np.exp(-(((X+20)**2)/2000 + ((Y+75)**2)/2200))
Z += 7.0*np.exp(-(((X-80)**2)/3000 + ((Y-85)**2)/2800))
Z += 5.0*np.exp(-(((X-150)**2)/3000 + ((Y+60)**2)/2600))
# mid-distance rolling scenery (|y| 14-24 m), wide and low, blends smoothly into the lane
_scen = [(10,16,1.2,18),(22,-18,0.9,16),(36,20,1.5,20),(50,-16,1.1,18),(64,18,1.3,20),(78,-20,1.0,18),
         (92,17,1.4,20),(106,-17,1.2,18),(120,20,1.0,18),(30,-24,0.8,16),(70,24,0.9,16),(110,24,1.1,18)]
for _dx,_dy,_dh,_dw in _scen: Z += gauss(_dx, _dy, _dh, _dw/2.4)

# lane features - three layers, all deterministic:
#  1. swell    : wide gentle mounds, whole chassis rises/falls (showcase)
#  2. ridges   : narrow (sigma 1.0 m) offset to ONE wheel track -> asymmetric excitation (suspension test)
#  3. hollows  : narrow, opposite side of the neighbouring ridge
swell, ridges, hollows = [], [], []
_x, _k = 30.0, 0
while _x < 108.0:
    swell.append((_x, (0.8 if _k % 2 == 0 else -0.8), 0.25 + 0.25*abs(math.sin(_k*1.7)), 8.0 + 6.0*abs(math.cos(_k*1.3))))
    _x += 6.5 + 1.5*abs(math.sin(_k*2.1)); _k += 1
_x, _k = 32.0, 0
while _x < 108.0:
    side = 1.0 if _k % 2 == 0 else -1.0
    ridges.append((_x, side*0.95, 0.16 + 0.12*abs(math.sin(_k*1.3 + 0.4)), 1.0))
    hollows.append((_x + 2.6, -side*0.95, 0.10 + 0.08*abs(math.cos(_k*0.9)), 1.1))
    _x += 5.2 + 0.6*abs(math.sin(_k*1.9)); _k += 1

_Zc = np.zeros_like(Z)
for cx,cy,h,w in swell:   _Zc += gauss(cx, cy, h, w/2.4)
for cx,cy,h,s in ridges:  _Zc += gauss(cx, cy, h, s)
for cx,cy,d,s in hollows: _Zc -= gauss(cx, cy, d, s)
_edge = np.clip((30.0 - np.abs(Y)) / 20.0, 0.0, 1.0)   # lane (|y|<10) sees only its own features; 20 m blend to scenery
Z = Z*(1.0-_edge) + _Zc*_edge
print(f"[scene] lane: {len(swell)} swells (25-50 cm), {len(ridges)} ridges (16-28 cm, one-track), {len(hollows)} hollows")
_ci = lambda x: int((x+EXT/2)/EXT*(N-1))
print(f"[scene] lane z(x=20)={float(Z[_ci(20),_ci(0)]):.2f} z(x=58)={float(Z[_ci(58),_ci(0)]):.2f} z(x=108)={float(Z[_ci(108),_ci(0)]):.2f}")

def ground_z(px, py):
    i = np.clip((px + EXT/2)/EXT*(N-1), 0, N-1)
    j = np.clip((py + EXT/2)/EXT*(N-1), 0, N-1)
    i0,j0 = int(i), int(j); i1,j1 = min(i0+1,N-1), min(j0+1,N-1)
    fi,fj = i-i0, j-j0
    return float(Z[i0,j0]*(1-fi)*(1-fj) + Z[i1,j0]*fi*(1-fj) + Z[i0,j1]*(1-fi)*fj + Z[i1,j1]*fi*fj)

# ---------------- visual terrain mesh ----------------
UV_REPEAT = 20.0
pts, uvs, idx, cnt = [], [], [], []
for i in range(N):
    for j in range(N):
        pts.append(Gf.Vec3f(float(X[i,j]), float(Y[i,j]), float(Z[i,j])))
        uvs.append(Gf.Vec2f(i/(N-1)*UV_REPEAT, j/(N-1)*UV_REPEAT))
for i in range(N-1):
    for j in range(N-1):
        a,b,c,d = i*N+j, i*N+j+1, (i+1)*N+j+1, (i+1)*N+j
        idx += [a,b,c,d]; cnt.append(4)
m = UsdGeom.Mesh.Define(stage, "/World/terrain")
m.CreatePointsAttr(pts)
m.CreateFaceVertexIndicesAttr(idx); m.CreateFaceVertexCountsAttr(cnt)
UsdGeom.PrimvarsAPI(m.GetPrim()).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex).Set(uvs)

mat = UsdShade.Material.Define(stage, "/World/Looks/ground")
sh = UsdShade.Shader.Define(stage, "/World/Looks/ground/sh")
sh.CreateIdAttr("UsdPreviewSurface")
sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.95)
sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
st = UsdShade.Shader.Define(stage, "/World/Looks/ground/st")
st.CreateIdAttr("UsdPrimvarReader_float2")
st.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
st.CreateOutput("result", Sdf.ValueTypeNames.Float2)
def uvtex(path, file, rgb_scale=None, rgb_bias=None):
    t = UsdShade.Shader.Define(stage, path)
    t.CreateIdAttr("UsdUVTexture")
    t.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(file)
    t.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(st.GetOutput("result"))
    t.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("repeat")
    t.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("repeat")
    if rgb_scale: t.CreateInput("scale", Sdf.ValueTypeNames.Float4).Set(Gf.Vec4f(*rgb_scale))
    if rgb_bias:  t.CreateInput("bias",  Sdf.ValueTypeNames.Float4).Set(Gf.Vec4f(*rgb_bias))
    t.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
    return t
tx = uvtex("/World/Looks/ground/tx", "/workspace/assets/mars_ground.jpg")
sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(tx.GetOutput("rgb"))
try:
    tn = uvtex("/World/Looks/ground/tn", "/workspace/assets/mars_normal.jpg", (2,2,2,1), (-1,-1,-1,0))
    sh.CreateInput("normal", Sdf.ValueTypeNames.Normal3f).ConnectToSource(tn.GetOutput("rgb"))
except Exception as _e: print("[scene] normal map skipped:", _e)
mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(mat)
print("[scene] terrain mesh + texture ready")

# ---------------- rocks (outside the lane, |y| >= 9) ----------------
_rng = np.random.default_rng(53)
def rock(path, seed, scale, collide=True):
    r = np.random.default_rng(seed)
    sp, st_ = 10, 14
    P, I, C = [], [], []
    for a in range(sp+1):
        th = np.pi*a/sp
        for b in range(st_):
            ph = 2*np.pi*b/st_
            rad = scale*(1.0 + r.normal(0, 0.16))
            P.append(Gf.Vec3f(rad*np.sin(th)*np.cos(ph), rad*np.sin(th)*np.sin(ph), rad*np.cos(th)*0.72))
    for a in range(sp):
        for b in range(st_):
            v0 = a*st_+b; v1 = a*st_+(b+1)%st_; v2 = (a+1)*st_+(b+1)%st_; v3 = (a+1)*st_+b
            I += [v0,v1,v2,v3]; C.append(4)
    mm = UsdGeom.Mesh.Define(stage, path)
    mm.CreatePointsAttr(P); mm.CreateFaceVertexIndicesAttr(I); mm.CreateFaceVertexCountsAttr(C)
    mm.CreateDisplayColorAttr([Gf.Vec3f(0.42, 0.24, 0.17)])
    if collide: UsdPhysics.CollisionAPI.Apply(mm.GetPrim())
    return mm
nrock = 0
clusters = [(31,11,2.6),(38,-11,3.0),(45,12,2.2),(52,-11,3.4),(57,12,2.0),(12,-11,3.0),(20,11,2.6),(66,12,2.8),(75,-11,3.0),(84,12,2.4),(93,-11,3.2),(102,11,2.6)]
for k in range(330):
    if _rng.random() < 0.72:
        cx,cy,cs = clusters[_rng.integers(0,len(clusters))]
        px, py = cx + _rng.normal(0,cs), cy + _rng.normal(0,cs*0.7)
    else:
        px, py = _rng.uniform(6,114), _rng.uniform(-14,14)
    if abs(py) < 9.0 and 8.0 < px < 114.0: continue
    sc = _rng.uniform(0.05,0.20) if _rng.random() > 0.4 else _rng.uniform(0.04,0.10)
    rr = rock(f"/World/rocks/r{k}", 1000+k, sc, True)
    xf = UsdGeom.Xformable(rr.GetPrim())
    xf.AddTranslateOp().Set(Gf.Vec3d(px, py, ground_z(px,py) + sc*0.20))
    xf.AddRotateZOp().Set(float(_rng.uniform(0,360)))
    nrock += 1
ndeco = 0
for k in range(260):   # decorative lane-side rocks: no collision, never on the wheel tracks (|y| >= 1.0)
    px = _rng.uniform(22, 114); py = _rng.choice([-1, 1]) * _rng.uniform(1.0, 8.8)
    sc = _rng.uniform(0.03, 0.09)
    rr = rock(f"/World/rocks/d{k}", 5000+k, sc, False)
    xf = UsdGeom.Xformable(rr.GetPrim())
    xf.AddTranslateOp().Set(Gf.Vec3d(px, py, ground_z(px,py) + sc*0.15))
    xf.AddRotateZOp().Set(float(_rng.uniform(0,360)))
    ndeco += 1
print(f"[scene] rocks: {nrock} colliding (|y|>=9) + {ndeco} decorative (lane side, no collision)")

# ---------------- sky + lights ----------------
dome = UsdLux.DomeLight.Define(stage, "/World/sky")
dome.CreateIntensityAttr(420); dome.CreateColorAttr((1.0, 0.95, 0.90))
dome.CreateTextureFileAttr("/workspace/assets/mars_sky.jpg"); dome.CreateTextureFormatAttr("latlong")
sun = UsdLux.DistantLight.Define(stage, "/World/sun")
sun.CreateIntensityAttr(360); sun.CreateAngleAttr(2.5)
UsdLux.LightAPI(sun.GetPrim()).CreateColorAttr((1.0, 0.88, 0.76))
UsdGeom.XformCommonAPI(sun.GetPrim()).SetRotate((55.0, 0.0, 35.0))
fill = UsdLux.DistantLight.Define(stage, "/World/fill")
fill.CreateIntensityAttr(150); fill.CreateAngleAttr(15.0)
UsdLux.LightAPI(fill.GetPrim()).CreateColorAttr((0.9, 0.72, 0.62))
UsdGeom.XformCommonAPI(fill.GetPrim()).SetRotate((40.0, 0.0, -125.0))
print("[scene] lighting ready")

# ---------------- physics world: 60 Hz, TGS, CCD ----------------
world = World(stage_units_in_meters=1.0, physics_dt=DT)
try:
    _sc = stage.GetPrimAtPath("/physicsScene")
    _ps = PhysxSchema.PhysxSceneAPI.Apply(_sc)
    _ps.CreateSolverTypeAttr("TGS"); _ps.CreateEnableCCDAttr(True)
    print("[physics] scene: TGS + CCD")
except Exception as _e: print("[physics] scene api:", _e)

# physics ground: 1 m slope-aligned static boxes, raw USD (fast). Terrain mesh collision does not cook in PhysX.
gm = UsdShade.Material.Define(stage, "/World/Looks/ground_phys")
pm = UsdPhysics.MaterialAPI.Apply(gm.GetPrim())
pm.CreateRestitutionAttr(0.0); pm.CreateStaticFrictionAttr(1.4); pm.CreateDynamicFrictionAttr(1.2)
UsdGeom.Xform.Define(stage, "/World/phys")
ntile = 0
for tx_ in np.arange(4.0, 116.0, 1.0):
    for ty_ in np.arange(-16.0, 16.0, 1.0):
        cx, cy = tx_+0.5, ty_+0.5
        zc = ground_z(cx, cy)
        dzdx = (ground_z(cx+0.25,cy) - ground_z(cx-0.25,cy))*2.0
        dzdy = (ground_z(cx,cy+0.25) - ground_z(cx,cy-0.25))*2.0
        p_, r_ = math.atan2(dzdx,1.0), math.atan2(dzdy,1.0)
        cp, sp_ = math.cos(p_/2), math.sin(p_/2)
        cr, sr = math.cos(r_/2), math.sin(r_/2)
        q = np.array([cr*cp, sr*cp, cr*sp_, 0.0]); q /= np.linalg.norm(q)
        cube = UsdGeom.Cube.Define(stage, f"/World/phys/t{ntile}")
        cube.CreateSizeAttr(1.0)
        xf = UsdGeom.Xformable(cube.GetPrim())
        xf.AddTranslateOp().Set(Gf.Vec3d(cx, cy, zc-0.15))
        xf.AddOrientOp().Set(Gf.Quatf(float(q[0]), float(q[1]), float(q[2]), float(q[3])))
        xf.AddScaleOp().Set(Gf.Vec3f(1.4, 1.4, 0.3))
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        UsdGeom.Imageable(cube.GetPrim()).MakeInvisible()
        UsdShade.MaterialBindingAPI.Apply(cube.GetPrim()).Bind(gm, materialPurpose="physics")
        ntile += 1
print(f"[physics] tiles: {ntile}")

# ---------------- rover ----------------
add_reference_to_stage(MODEL_PATH, "/World/rover")
UsdGeom.XformCommonAPI(stage.GetPrimAtPath("/World/rover")).SetTranslate((X_SPAWN, 0.0, ground_z(X_SPAWN,0.0)+0.42))
wm_ = UsdShade.Material.Define(stage, "/World/Looks/wheel_phys")
wpm = UsdPhysics.MaterialAPI.Apply(wm_.GetPrim())
wpm.CreateStaticFrictionAttr(1.5); wpm.CreateDynamicFrictionAttr(1.3); wpm.CreateRestitutionAttr(0.0)

def pbr(path, rgb, rough, metal):
    mt = UsdShade.Material.Define(stage, path)
    s_ = UsdShade.Shader.Define(stage, path+"/sh"); s_.CreateIdAttr("UsdPreviewSurface")
    s_.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
    s_.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(rough)
    s_.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metal)
    mt.CreateSurfaceOutput().ConnectToSource(s_.ConnectableAPI(), "surface")
    return mt
body_mat  = pbr("/World/Looks/rover_body",  (0.80, 0.81, 0.83), 0.45, 0.35)
wheel_mat = pbr("/World/Looks/rover_wheel", (0.10, 0.10, 0.11), 0.85, 0.0)
nb = nv = 0
for pr in Usd.PrimRange(stage.GetPrimAtPath("/World/rover")):
    p = pr.GetPath().pathString
    if "Wheel_Link" in p:
        try: UsdShade.MaterialBindingAPI.Apply(pr).Bind(wm_, materialPurpose="physics"); nb += 1
        except Exception: pass
    if pr.GetName().endswith("_Link") or pr.GetName().endswith("_link"):
        try:
            UsdShade.MaterialBindingAPI.Apply(pr).Bind(wheel_mat if "Wheel" in pr.GetName() else body_mat,
                                                       bindingStrength=UsdShade.Tokens.strongerThanDescendants); nv += 1
        except Exception: pass
print(f"[rover] wheel physics material on {nb} prims, visual material on {nv} links")
try:
    _nc = 0
    for pr in Usd.PrimRange(stage.GetPrimAtPath("/World/rover")):
        if pr.HasAPI(UsdPhysics.RigidBodyAPI):
            PhysxSchema.PhysxRigidBodyAPI.Apply(pr).CreateEnableCCDAttr(True); _nc += 1
    print(f"[rover] CCD enabled on {_nc} bodies")
except Exception as _e: print("[rover] ccd:", _e)

# rocker joints: importer creates no usable drive; create a force drive (stiffness 0, damping set later by mode)
_nd = 0
for _pr in Usd.PrimRange(stage.GetPrimAtPath("/World/rover")):
    if _pr.GetName() in ("leftjoint", "rightjoint"):
        _d = UsdPhysics.DriveAPI.Apply(_pr, "angular")
        _d.CreateTypeAttr("force"); _d.CreateStiffnessAttr(0.0); _d.CreateDampingAttr(0.0)
        _d.CreateMaxForceAttr(5000.0); _d.CreateTargetPositionAttr(0.0)
        _nd += 1
print(f"[drive] rocker drives created: {_nd}")

world.reset()
rov = Articulation("/World/rover")
rov.initialize()
names = list(rov.dof_names)
print("[rover] dofs:", names)
iL = iR = None
if "leftjoint" in names and MODEL == "revolute":
    iL, iR = names.index("leftjoint"), names.index("rightjoint")
S = [names.index(n) for n in ["FLSteerJoint","RLSteerJoint","FRSteerJoint","RRSteerJoint"]]
W = [names.index(n) for n in ["FLWheelJoint","RLWheelJoint","FRWheelJoint","RRWheelJoint"]]
print("[mode]", MODE, "(articulated)" if iL is not None else "(rigid chassis)")

kp = np.zeros((1,len(names))); kd = np.zeros((1,len(names)))
for i in S: kp[0,i], kd[0,i] = 5e4, 1e3
for i in W: kp[0,i], kd[0,i] = 0.0, 2e3
if iL is not None:
    if MODE == "passive":                       # free rocker with viscous damping only
        kp[0,iL] = kp[0,iR] = 0.0; kd[0,iL] = kd[0,iR] = 900.0
    else:                                       # hybrid: effort mode, software differential
        kp[0,iL] = kp[0,iR] = 0.0; kd[0,iL] = kd[0,iR] = 0.0
        try: rov.switch_control_mode("effort", joint_indices=np.array([iL, iR])); print("[mode] rocker -> effort")
        except Exception as _e: print("[mode] switch failed:", _e)
try: rov.set_gains(kps=kp, kds=kd); print("[rover] gains set")
except Exception as e: print("gains-err", e)
try:
    eff = np.full((1,len(names)), 1e5)
    for i in W: eff[0,i] = 48.0
    rov.set_max_efforts(eff); print("[rover] wheel effort cap 48 Nm")
except Exception as e: print("effort-err", e)

# ---------------- camera ----------------
cam = UsdGeom.Camera.Define(stage, "/World/cam")
cam.CreateFocalLengthAttr(18.0); cam.CreateClippingRangeAttr(Gf.Vec2f(0.05, 4000.0))
rp = rep.create.render_product("/World/cam", (1280, 720))
wr = rep.WriterRegistry.get("BasicWriter")
wr.initialize(output_dir=f"{RUN_DIR}/frames", rgb=True)
wr.attach([rp])
SHOTS = [(-11.0,-7.0,3.2),(-5.0,-4.5,1.6),(7.0,-5.0,2.2),(-3.0,-4.0,1.2),(-14.0,4.5,4.5),(0.0,-12.0,3.0),
         (-3.5,5.0,1.5),(9.0,5.5,2.8),(-8.0,1.5,1.8),(-6.0,-9.0,4.0),(5.0,-3.5,1.3),(-16.0,-9.0,6.0)]
def set_cam(prim, eye, tgt):
    eye_v = Gf.Vec3d(*eye); tgt_v = Gf.Vec3d(*tgt)
    fwd = tgt_v - eye_v
    if fwd.GetLength() < 1e-6: return
    fwd.Normalize()
    up = Gf.Vec3d(0.0, 0.0, 1.0)
    if abs(Gf.Dot(fwd, up)) > 0.999: up = Gf.Vec3d(0.0, 1.0, 0.0)
    right = Gf.Cross(fwd, up); right.Normalize()
    tup = Gf.Cross(right, fwd); tup.Normalize()
    mtx = Gf.Matrix4d(right[0], right[1], right[2], 0.0,
                      tup[0], tup[1], tup[2], 0.0,
                      -fwd[0], -fwd[1], -fwd[2], 0.0,
                      eye_v[0], eye_v[1], eye_v[2], 1.0)
    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder(); xf.AddTransformOp().Set(mtx)

def plan(f):
    return min(0.5 + f*(SPEED-0.5)/RAMP, SPEED) if f < RAMP else SPEED

# ---------------- config + telemetry ----------------
json.dump({"run": RUN, "model": MODEL, "mode": MODE, "speed_rad_s": SPEED, "speed_m_s": round(SPEED*0.145, 3),
           "lane_keeping": LANE, "total_steps": TOTAL, "cap": CAP, "x_end": X_END, "physics_hz": 60, "solver": "TGS", "ccd": True,
           "gravity": 9.81, "kp": KP, "kd": KD, "tau_max": TAU_MAX, "wheel_effort_cap_nm": 48.0,
           "friction": {"wheel": [1.5, 1.3], "ground": [1.4, 1.2]}, "tiles": ntile, "rocks": nrock,
           "ridges": ridges, "hollows": hollows, "swell": swell},
          open(f"{RUN_DIR}/config.json", "w"), indent=1)
tf = open(f"{RUN_DIR}/run.csv", "w", newline="")
tw = csv.writer(tf)
tw.writerow(["f","t","cmd_vel","x","y","z","roll","pitch","yaw","wheel_mean",
             "e","edot_f","tau_pd_raw","tau","qL","qR","wvFL","wvRL","wvFR","wvRR"])

# rocker neutral calibration: settle under gravity with zero constraint torque
for _ in range(600):
    if iL is not None and MODE == "hybrid": rov.set_joint_efforts(np.array([[0.0, 0.0]]), joint_indices=np.array([iL, iR]))
    rov.set_joint_velocity_targets(np.zeros((1,len(W))), joint_indices=np.array(W))
    world.step(render=False)
_q0 = rov.get_joint_positions()[0]
E0 = float(_q0[iL] + _q0[iR]) if iL is not None else 0.0
D0 = float(_q0[iL] - _q0[iR]) if iL is not None else 0.0
print(f"[calib] E0={E0:+.4f} D0={D0:+.4f}")
for _ in range(10): app.update()      # renderer warm-up so frame 0 is clean
print("[run] starting: max", TOTAL, "steps ->", TOTAL//CAP, "frames, stop at x >=", X_END)

hist, vf, tau_prev = [], 0.0, 0.0
fx = fy = fz = None
nframes = 0; end_reason = "timeout"
for f in range(TOTAL):
    q = rov.get_joint_positions()[0]; v = rov.get_joint_velocities()[0]
    e = (float(q[iL] + q[iR]) - E0) if iL is not None else 0.0
    hist.append((f*DT, e))
    if len(hist) > 8: hist.pop(0)
    if len(hist) >= 4:
        t0,e0 = hist[0]; t1,e1 = hist[-1]
        dt_ = t1-t0; edot = (e1-e0)/dt_ if dt_ > 1e-6 else 0.0
    else: edot = 0.0
    vf += 0.12*(edot - vf)
    tau_pd = -KP*e - KD*vf
    tau_db = 0.0 if (abs(e) < 0.005 and abs(vf) < 0.03) else tau_pd
    tau_sl = float(np.clip(tau_db, tau_prev-3.0, tau_prev+3.0))
    tau = float(np.clip(tau_sl, -TAU_MAX, TAU_MAX))
    tau_prev = tau
    if iL is not None and MODE == "hybrid":
        rov.set_joint_efforts(np.array([[tau, tau]]), joint_indices=np.array([iL, iR]))
    cv = plan(f)
    rov.set_joint_velocity_targets(np.full((1,len(W)), cv), joint_indices=np.array(W))

    pos, ori = rov.get_world_poses()
    rx, ry, rz = float(pos[0][0]), float(pos[0][1]), float(pos[0][2])
    w_,x_,y_,z_ = [float(a) for a in ori[0]]
    roll  = math.degrees(math.atan2(2*(w_*x_+y_*z_), 1-2*(x_*x_+y_*y_)))
    pitch = math.degrees(math.asin(max(-1,min(1,2*(w_*y_-z_*x_)))))
    yaw   = math.degrees(math.atan2(2*(w_*z_+x_*y_), 1-2*(y_*y_+z_*z_)))

    _st = np.zeros((1,len(S)))
    if LANE:
        _corr = float(np.clip(-0.18*ry - 0.60*math.sin(math.radians(yaw)), -0.35, 0.35))
        _st[0,0] = _st[0,2] = _corr
        _st[0,1] = _st[0,3] = -_corr
    rov.set_joint_position_targets(_st, joint_indices=np.array(S))

    if fx is None: fx, fy, fz = rx, ry, rz
    fx += 0.22*(rx-fx); fy += 0.22*(ry-fy); fz += 0.015*(rz-fz)
    shot = SHOTS[min(int(f/(TOTAL/len(SHOTS))), len(SHOTS)-1)]
    ex, ey = rx+shot[0], ry+shot[1]
    ez = max(fz + 0.55*shot[2], ground_z(ex, ey) + 0.9)
    _hd = math.radians(yaw)
    set_cam(cam.GetPrim(), (ex, ey, ez), (fx + 0.6*math.cos(_hd), fy + 0.6*math.sin(_hd), fz + 0.95))

    wm = float(np.mean([v[i] for i in W]))
    tw.writerow([f, round(f*DT,4), round(cv,3), round(rx,3), round(ry,3), round(rz,4),
                 round(roll,2), round(pitch,2), round(yaw,2), round(wm,3),
                 round(e,5), round(vf,5), round(float(tau_pd),2), round(tau,2),
                 (round(float(q[iL]),5) if iL is not None else 0.0), (round(float(q[iR]),5) if iL is not None else 0.0)]
                + [round(float(v[i]),2) for i in W])
    if f % 300 == 0:
        tf.flush(); print(f"[run] f={f} t={f*DT:.1f}s x={rx:.2f} y={ry:.2f} z={rz:.3f} roll={roll:+.1f} tau={tau:+.1f} e={e:+.4f}")

    cap = (f % CAP == 0)
    world.step(render=cap)
    if cap:
        try: rep.orchestrator.step(delta_time=0.0, pause_timeline=False)
        except Exception: rep.orchestrator.step()
        nframes += 1
    if rx >= X_END: end_reason = "course_end"; break
    if abs(roll) > 60.0 or abs(pitch) > 60.0: end_reason = "rollover"; break

tf.close()
print(f"SESSION-ENDED reason={end_reason} steps={f+1} t={(f+1)*DT:.1f}s frames={nframes} x_final={rx:.2f}")
app.close()
