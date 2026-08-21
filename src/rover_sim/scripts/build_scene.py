"""
Mars showcase scene builder — SuRover 'karasimsek' hybrid-suspension demo.

Builds an Opportunity-style Mars environment:
  * Heightmap terrain (120 x 120 m): horizon mountain ridge, midfield
    bumps/craters, flattened drive corridor to preserve traverse physics.
  * Photogrammetric ground material (ambientCG Ground054, CC0) tinted to
    Mars red-brown; graceful fallback to displayColor primvars.
  * Layered rust sky via lat-long gradient texture on a dome light.
  * Procedural rock field (displaced icospheres) — guaranteed renderable,
    replacing the legacy converter USDs whose render visibility was unreliable.

Usage:
  PREVIEW=1 python build_scene.py    # build scene, render 1 frame, exit
"""
import os, numpy as np
from PIL import Image

# ---------------------------------------------------------------- sky texture
SKY = "/workspace/assets/sky_latlong.png"
W, H = 1024, 512
img = np.zeros((H, W, 3), np.float32)
top = np.array([0.55, 0.26, 0.10]); hor = np.array([0.95, 0.55, 0.28]); gnd = np.array([0.70, 0.38, 0.20])
for r in range(H):
    t = r / (H - 1)
    if t < 0.48:   img[r] = gnd
    elif t < 0.56: img[r] = gnd + (hor - gnd) * ((t - 0.48) / 0.08)
    else:          img[r] = hor + (top - hor) * (((t - 0.56) / 0.44) ** 0.75)
Image.fromarray((np.clip(img,0,1)*255).astype(np.uint8)[::-1]).save(SKY)
print("[scene] sky texture written")

# ------------------------------------------------------------------ sim start
from isaacsim import SimulationApp
app = SimulationApp({"headless": True, "width": 1920, "height": 1080})
import omni.replicator.core as rep
from pxr import Usd, UsdGeom, UsdLux, UsdShade, UsdPhysics, Sdf, Gf
import omni.usd

stage = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)

# ------------------------------------------------------------------- lighting
dome = UsdLux.DomeLight.Define(stage, "/World/Sky")
dome.CreateIntensityAttr(560)
dome.CreateColorAttr(Gf.Vec3f(1.0, 0.72, 0.48))          # dusty rust light bath
dome.CreateTextureFileAttr(SKY)
dome.CreateTextureFormatAttr("latlong")

# ---------------------------------------------------------- terrain heightmap
N, EXT = 241, 120.0                                       # 241x241 verts, 120 m
xs = np.linspace(24 - EXT/2, 24 + EXT/2, N)               # centered on corridor
ys = np.linspace(-EXT/2, EXT/2, N)
X, Y = np.meshgrid(xs, ys, indexing="ij")
Z = np.zeros_like(X)
rng = np.random.default_rng(42)
# gentle rolling base
for _ in range(24):
    cx, cy = rng.uniform(-30, 78), rng.uniform(-55, 55)
    amp, sig = rng.uniform(-0.5, 0.7), rng.uniform(4, 12)
    Z += amp * np.exp(-((X-cx)**2 + (Y-cy)**2) / (2*sig*sig))
# horizon mountain range — distant, layered, separated peaks (Opportunity ref)
for _ in range(14):
    cx, cy = rng.uniform(-30, -18), rng.uniform(-55, 55)
    amp, sig = rng.uniform(1.2, 3.2), rng.uniform(2.5, 5.5)   # smaller, distinct peaks
    Z += amp * np.exp(-((X-cx)**2 + (Y-cy)**2) / (2*sig*sig))
for _ in range(6):                                            # taller back row
    cx, cy = rng.uniform(-34, -26), rng.uniform(-50, 50)
    amp, sig = rng.uniform(3.5, 5.5), rng.uniform(4.0, 7.0)
    Z += amp * np.exp(-((X-cx)**2 + (Y-cy)**2) / (2*sig*sig))
# craters midfield
for _ in range(7):
    cx, cy = rng.uniform(0, 40), rng.uniform(-25, 25)
    amp, sig = rng.uniform(0.3, 0.8), rng.uniform(2.0, 4.5)
    d2 = (X-cx)**2 + (Y-cy)**2
    Z += -amp*np.exp(-d2/(2*sig*sig)) + 0.4*amp*np.exp(-d2/(2*(1.8*sig)**2))
# --- mountains as a separate visual layer (dust-beige, smooth) -----------
MOUNT = Z.copy()          # snapshot: ridge-only heights (before corridor flatten)
# flatten drive corridor so traverse physics stay clean
corr = np.exp(-((np.clip(np.abs(Y)-1.8, 0, None))**2) / (2*2.0**2)) * ((X > 16) & (X < 44))
Z *= (1.0 - 0.85 * corr)

def ground_z(px, py):
    i = int(np.clip((px - xs[0]) / (xs[-1]-xs[0]) * (N-1), 0, N-1))
    j = int(np.clip((py - ys[0]) / (ys[-1]-ys[0]) * (N-1), 0, N-1))
    return float(Z[i, j])

mesh = UsdGeom.Mesh.Define(stage, "/World/terrain")
pts = np.stack([X, Y, Z], -1).reshape(-1, 3)
mesh.CreatePointsAttr([Gf.Vec3f(*p) for p in pts.astype(float)])
idx, cts = [], []
for i in range(N-1):
    for j in range(N-1):
        a = i*N+j
        idx += [a, a+1, a+N+1, a+N]; cts.append(4)
mesh.CreateFaceVertexIndicesAttr(idx); mesh.CreateFaceVertexCountsAttr(cts)
uv = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex)
uv.Set([Gf.Vec2f(float(p[0])/6.0, float(p[1])/6.0) for p in pts])   # 6 m tiling
UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
print(f"[scene] terrain built: {N}x{N} verts, ridge+craters, corridor flattened")

# --------------------------------------------------- terrain material (tinted)
mat = UsdShade.Material.Define(stage, "/World/Looks/mars_ground")
sh  = UsdShade.Shader.Define(stage, "/World/Looks/mars_ground/pbr")
sh.CreateIdAttr("UsdPreviewSurface")
tex = UsdShade.Shader.Define(stage, "/World/Looks/mars_ground/tex")
tex.CreateIdAttr("UsdUVTexture")
tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set("/workspace/assets/ground/Ground054_4K-JPG_Color.jpg")
tex.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("repeat")
tex.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("repeat")
rdr = UsdShade.Shader.Define(stage, "/World/Looks/mars_ground/st")
rdr.CreateIdAttr("UsdPrimvarReader_float2")
rdr.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(rdr.ConnectableAPI(), "result")
# multiply-tint toward Mars red is not supported in preview surface directly;
# scale input approximates a warm tint
tex.CreateInput("scale", Sdf.ValueTypeNames.Float4).Set(Gf.Vec4f(1.15, 0.62, 0.38, 1.0))
sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(tex.ConnectableAPI(), "rgb")
sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.95)
mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(mat)

# distant-hills mesh: only where ridge height dominates (x < -6), pushed 0.15 m up
hill_mask = (X < -6)
hm = UsdGeom.Mesh.Define(stage, "/World/hills")
hp = np.stack([X, Y, Z + 0.15], -1)[hill_mask.any(axis=1)].reshape(-1, 3) if False else None
# simpler: full grid duplicate, but flatten non-hill zone below ground so it hides
_und = 0.35*np.sin(X*0.55 + 1.7)*np.cos(Y*0.38) + 0.22*np.sin(Y*0.9 + 0.6)
Zh = np.where(X < -6, Z + 0.12 + _und*np.clip((-6 - X)/8.0, 0, 1), Z - 5.0)
hpts = np.stack([X, Y, Zh], -1).reshape(-1, 3)
hm.CreatePointsAttr([Gf.Vec3f(*p) for p in hpts.astype(float)])
hm.CreateFaceVertexIndicesAttr(idx); hm.CreateFaceVertexCountsAttr(cts)
huv = UsdGeom.PrimvarsAPI(hm).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex)
huv.Set([Gf.Vec2f(float(p[0])/18.0, float(p[1])/18.0) for p in hpts])
UsdGeom.Gprim(hm.GetPrim()).CreateDisplayColorAttr([Gf.Vec3f(0.52, 0.34, 0.20)])   # dust-beige
hmat = UsdShade.Material.Define(stage, "/World/Looks/dust_hills")
hsh = UsdShade.Shader.Define(stage, "/World/Looks/dust_hills/pbr")
hsh.CreateIdAttr("UsdPreviewSurface")
htex = UsdShade.Shader.Define(stage, "/World/Looks/dust_hills/tex")
htex.CreateIdAttr("UsdUVTexture")
htex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set("/workspace/assets/ground/Ground054_4K-JPG_Color.jpg")
htex.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("repeat")
htex.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("repeat")
hrdr = UsdShade.Shader.Define(stage, "/World/Looks/dust_hills/st")
hrdr.CreateIdAttr("UsdPrimvarReader_float2")
hrdr.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
htex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(hrdr.ConnectableAPI(), "result")
htex.CreateInput("scale", Sdf.ValueTypeNames.Float4).Set(Gf.Vec4f(1.05, 0.78, 0.58, 1.0))   # pale sand-beige
hsh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(htex.ConnectableAPI(), "rgb")
hsh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(1.0)
hmat.CreateSurfaceOutput().ConnectToSource(hsh.ConnectableAPI(), "surface")
UsdShade.MaterialBindingAPI.Apply(hm.GetPrim()).Bind(hmat)
print("[scene] distant hills layered with dust-beige material")
# fallback primvar in case the preview surface is dropped by the renderer
# vertex-varying mottling: dust patches (bright), crack bands (dark), slope shading
_gx, _gy = np.gradient(Z)
_slope = np.clip(np.hypot(_gx, _gy) * 3.0, 0, 1)
_r3 = np.random.default_rng(11)
_patch = np.zeros_like(Z)
for _ in range(26):                                   # bright dust pools
    pcx, pcy = _r3.uniform(0, 44), _r3.uniform(-30, 30)
    _patch += _r3.uniform(0.10, 0.28) * np.exp(-((X-pcx)**2 + (Y-pcy)**2) / (2*_r3.uniform(2.0, 6.0)**2))
_crack = 0.12 * (np.sin(X*1.7 + Y*2.3) * np.sin(X*0.7 - Y*1.1) < -0.72)   # dark crack bands
_mod = np.clip(1.0 + _patch - _crack - 0.25*_slope, 0.55, 1.45)
_base = np.array([0.42, 0.20, 0.10])
_cols = (_base[None, :] * _mod.reshape(-1, 1)).astype(float)
_dc = UsdGeom.Gprim(mesh.GetPrim()).CreateDisplayColorAttr()
_dc.Set([Gf.Vec3f(*c) for c in _cols])
UsdGeom.Primvar(_dc).SetInterpolation(UsdGeom.Tokens.vertex)
print("[scene] ground mottling: dust pools + crack bands + slope shading (vertex-varying)")
print("[scene] ground material bound (photogrammetric color, mars tint)")

# ----------------------------------------------------------- procedural rocks
def make_rock(path, seed, radius):
    """Displaced icosphere — guaranteed-renderable rock stand-in."""
    r = np.random.default_rng(seed)
    n_lat, n_lon = 10, 14
    ps, ii, cc = [], [], []
    for a in range(n_lat+1):
        th = np.pi * a / n_lat
        for b in range(n_lon):
            ph = 2*np.pi * b / n_lon
            noise = 1.0 + r.uniform(-0.28, 0.28)
            v = np.array([np.sin(th)*np.cos(ph), np.sin(th)*np.sin(ph), np.cos(th)*0.72]) * radius * noise
            ps.append(Gf.Vec3f(*v.astype(float)))
    for a in range(n_lat):
        for b in range(n_lon):
            p0=a*n_lon+b; p1=a*n_lon+(b+1)%n_lon; p2=(a+1)*n_lon+(b+1)%n_lon; p3=(a+1)*n_lon+b
            ii += [p0,p1,p2,p3]; cc.append(4)
    m = UsdGeom.Mesh.Define(stage, path)
    m.CreatePointsAttr(ps); m.CreateFaceVertexIndicesAttr(ii); m.CreateFaceVertexCountsAttr(cc)
    shade = 0.10 + r.uniform(0, 0.06)
    UsdGeom.Gprim(m.GetPrim()).CreateDisplayColorAttr([Gf.Vec3f(shade, shade*0.42, shade*0.22)])
    return m

rng2 = np.random.default_rng(7)
count = 0
for k in range(140):                                   # scatter field in frustum
    px, py = rng2.uniform(2, 30), rng2.uniform(-9, 6)
    if abs(py) < 1.6 and 18 < px < 34: continue        # keep corridor clear
    rad = rng2.uniform(0.04, 0.22)
    rk = make_rock(f"/World/rocks/r{k}", 100+k, rad)
    xf = UsdGeom.Xformable(rk.GetPrim())
    xf.AddTranslateOp().Set(Gf.Vec3d(px, py, rad*0.55 + ground_z(px, py)))
    xf.AddRotateZOp().Set(rng2.uniform(0, 360)); count += 1
for k, (bx, by, br) in enumerate([(21.0, -3.2, 0.55), (14.5, 2.4, 0.75), (9.0, -4.5, 0.95), (25.5, 3.4, 0.45)]):
    bd = make_rock(f"/World/boulders/b{k}", 500+k, br)
    xf = UsdGeom.Xformable(bd.GetPrim())
    xf.AddTranslateOp().Set(Gf.Vec3d(bx, by, br*0.5 + ground_z(bx, by)))
    xf.AddRotateZOp().Set(rng2.uniform(0, 360))
    UsdPhysics.CollisionAPI.Apply(bd.GetPrim()); count += 1
print(f"[scene] rocks placed: {count} (procedural, corridor kept clear)")

# --------------------------------------------------------------------- camera
cam = rep.create.camera(position=(33.0, 5.5, 2.6 + ground_z(33, 5.5)),
                        look_at=(8.0, -1.0, 2.6), focal_length=21.0)   # lower, horizon at upper third
rprod = rep.create.render_product(cam, (1920, 1080))
wr = rep.WriterRegistry.get("BasicWriter")
os.makedirs("/workspace/frames", exist_ok=True)
wr.initialize(output_dir="/workspace/frames", rgb=True); wr.attach([rprod])

for _ in range(90): app.update()                        # RTX warm-up
rep.orchestrator.step()
for _ in range(10): app.update()
print("[scene] PREVIEW FRAME WRITTEN -> /workspace/frames/")
app.close()
