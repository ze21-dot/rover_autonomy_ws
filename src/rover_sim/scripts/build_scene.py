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
SKY = "/tmp/sky_unused.png"   # in-script generator neutralized; dome uses sky_v6.png
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
dome.CreateIntensityAttr(320)
dome.CreateColorAttr(Gf.Vec3f(1.0, 0.72, 0.48))          # dusty rust light bath
dome.CreateTextureFileAttr("/workspace/assets/sky_v6.png")
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
# signature crater: raised rim, camera-visible midfield (Mars Yard zone-4 style)
_cd2 = (X - 16.0)**2 + (Y + 4.0)**2
Z += -1.55*np.exp(-_cd2/(2*2.6**2)) + 0.85*np.exp(-_cd2/(2*4.1**2))
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
uv.Set([Gf.Vec2f(float(p[0])/24.0, float(p[1])/24.0) for p in pts])   # 6 m tiling
UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
print(f"[scene] terrain built: {N}x{N} verts, ridge+craters, corridor flattened")

# --------------------------------------------------- terrain material (tinted)
mat = UsdShade.Material.Define(stage, "/World/Looks/mars_ground")
sh  = UsdShade.Shader.Define(stage, "/World/Looks/mars_ground/pbr")
sh.CreateIdAttr("UsdPreviewSurface")
tex = UsdShade.Shader.Define(stage, "/World/Looks/mars_ground/tex")
tex.CreateIdAttr("UsdUVTexture")
tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set("/workspace/assets/ground/mars_ground_baked.jpg")  # pattern baked in
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
_blend = np.clip((-6 - X)/6.0, 0, 1)          # 0 at edge -> 1 deep in hills
Zh = Z + (0.30 + 0.9*np.clip(_und, -0.15, 0.6)*_blend)*_blend - 0.55*(1.0 - np.clip(_blend*2.2, 0, 1))
hpts = np.stack([X, Y, Zh], -1).reshape(-1, 3)
hm.CreatePointsAttr([Gf.Vec3f(*p) for p in hpts.astype(float)])
hm.CreateFaceVertexIndicesAttr(idx); hm.CreateFaceVertexCountsAttr(cts)
huv = UsdGeom.PrimvarsAPI(hm).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex)
huv.Set([Gf.Vec2f(float(p[0])/8.0, float(p[1])/8.0) for p in hpts])
UsdGeom.Gprim(hm.GetPrim()).CreateDisplayColorAttr([Gf.Vec3f(0.52, 0.34, 0.20)])   # dust-beige
hmat = UsdShade.Material.Define(stage, "/World/Looks/dust_hills")
hsh = UsdShade.Shader.Define(stage, "/World/Looks/dust_hills/pbr")
hsh.CreateIdAttr("UsdPreviewSurface")
htex = UsdShade.Shader.Define(stage, "/World/Looks/dust_hills/tex")
htex.CreateIdAttr("UsdUVTexture")
htex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set("/workspace/assets/ground/hills_clean.jpg")  # hills: clean texture
htex.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("repeat")
htex.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("repeat")
hrdr = UsdShade.Shader.Define(stage, "/World/Looks/dust_hills/st")
hrdr.CreateIdAttr("UsdPrimvarReader_float2")
hrdr.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
htex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(hrdr.ConnectableAPI(), "result")
htex.CreateInput("scale", Sdf.ValueTypeNames.Float4).Set(Gf.Vec4f(1.30, 0.95, 0.70, 1.0))   # pale sand-beige
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
# wind-swept banding: low-frequency stripes flowing across the view, gently curved
_band_axis = X*0.32 + Y*0.9                            # stripe direction
_wobble = 1.8*np.sin(X*0.18 + 1.1) + 1.2*np.sin(Y*0.11 - 0.7)
_patch = (0.16*np.sin(_band_axis*0.55 + _wobble)
        + 0.10*np.sin(_band_axis*1.35 + _wobble*1.7 + 2.4)
        + 0.05*np.sin(_band_axis*3.1 + 0.9))
_c1 = (np.sin(X*1.7 + Y*2.3) * np.sin(X*0.7 - Y*1.1) < -0.55)
_c2 = (np.sin(X*3.1 - Y*1.9 + 2.2) * np.sin(X*1.3 + Y*2.7) < -0.62)
_crack = 0.30*_c1 + 0.22*_c2                      # deeper, denser crack network   # dark crack bands
# Voronoi-edge plate cracks (Curiosity mudstone reference), kept subtle
_vr = np.random.default_rng(23)
_seeds = np.stack([_vr.uniform(xs[0], xs[-1], 260), _vr.uniform(ys[0], ys[-1], 260)], 1)
_P2 = np.stack([X.ravel(), Y.ravel()], 1)
_d = np.sqrt(((_P2[:, None, :] - _seeds[None, :, :])**2).sum(-1))
_d.sort(axis=1)
_edge = np.clip(1.0 - (_d[:, 1] - _d[:, 0]) / 0.28, 0, 1).reshape(X.shape)   # 1 at plate borders
_plates = 0.20 * (_edge ** 2.2)
_mod = np.clip(1.0 + _patch - _crack - _plates - 0.25*_slope, 0.55, 1.45)
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
    n_lat, n_lon = 18, 24
    ps, ii, cc = [], [], []
    for a in range(n_lat+1):
        th = np.pi * a / n_lat
        for b in range(n_lon):
            ph = 2*np.pi * b / n_lon
            noise = (1.0 + 0.15*np.sin(2*th + 3*ph + seed) * r.uniform(0.6, 1.0)
                         + r.uniform(-0.05, 0.05))          # two-octave: bulk + fine
            v = np.array([np.sin(th)*np.cos(ph), np.sin(th)*np.sin(ph), np.cos(th)*0.72]) * radius * noise
            ps.append(Gf.Vec3f(*v.astype(float)))
    for a in range(n_lat):
        for b in range(n_lon):
            p0=a*n_lon+b; p1=a*n_lon+(b+1)%n_lon; p2=(a+1)*n_lon+(b+1)%n_lon; p3=(a+1)*n_lon+b
            ii += [p0,p1,p2,p3]; cc.append(4)
    m = UsdGeom.Mesh.Define(stage, path)
    m.CreatePointsAttr(ps); m.CreateFaceVertexIndicesAttr(ii); m.CreateFaceVertexCountsAttr(cc)
    m.CreateSubdivisionSchemeAttr("catmullClark")       # smooth silhouettes
    shade = 0.10 + r.uniform(0, 0.06)
    UsdGeom.Gprim(m.GetPrim()).CreateDisplayColorAttr([Gf.Vec3f(shade, shade*0.42, shade*0.22)])
    return m

rng2 = np.random.default_rng(7)
count = 0
for k in range(620):                                   # scatter field in frustum
    px, py = rng2.uniform(2, 30), rng2.uniform(-9, 9)
    if abs(py) < 1.6 and 18 < px < 34: continue
    if (px-16.0)**2 + (py+4.0)**2 < 4.5**2: continue   # keep crater pit clean        # keep corridor clear
    rad = rng2.uniform(0.015, 0.20) if rng2.random() > 0.55 else rng2.uniform(0.012, 0.045)
    rk = make_rock(f"/World/rocks/r{k}", 100+k, rad)
    xf = UsdGeom.Xformable(rk.GetPrim())
    xf.AddTranslateOp().Set(Gf.Vec3d(px, py, rad*0.38 + ground_z(px, py)))   # seated deeper
    xf.AddRotateZOp().Set(rng2.uniform(0, 360))
    xf.AddRotateXOp().Set(rng2.uniform(-18, 18)); xf.AddRotateYOp().Set(rng2.uniform(-18, 18))
    xf.AddScaleOp().Set(Gf.Vec3f(rng2.uniform(0.75, 1.35), rng2.uniform(0.75, 1.35), rng2.uniform(0.45, 0.95)))  # squash
    count += 1
for _ck, (_ccx, _ccy) in enumerate([(19.5, 2.0), (12.0, -6.5), (24.5, -2.5)]):   # rock clusters
    for _cj in range(rng2.integers(10, 16)):
        _pxc = _ccx + rng2.normal(0, 0.9); _pyc = _ccy + rng2.normal(0, 0.9)
        if abs(_pyc) < 1.6 and 18 < _pxc < 34: continue
        if (_pxc-16.0)**2 + (_pyc+4.0)**2 < 4.5**2: continue
        _rc = rng2.uniform(0.02, 0.14)
        _rk = make_rock(f"/World/rocks/c{_ck}_{_cj}", 700 + _ck*10 + _cj, _rc)
        _xfc = UsdGeom.Xformable(_rk.GetPrim())
        _xfc.AddTranslateOp().Set(Gf.Vec3d(_pxc, _pyc, _rc*0.38 + ground_z(_pxc, _pyc)))
        _xfc.AddRotateZOp().Set(rng2.uniform(0, 360))
        _xfc.AddRotateXOp().Set(rng2.uniform(-12, 12)); _xfc.AddRotateYOp().Set(rng2.uniform(-12, 12))
        _xfc.AddScaleOp().Set(Gf.Vec3f(1.0, rng2.uniform(0.85, 1.15), rng2.uniform(0.55, 0.85)))
        count += 1
for k, (bx, by, br) in enumerate([(21.0, -3.2, 0.34), (14.5, 2.4, 0.47), (9.0, -4.5, 0.59), (25.5, 3.4, 0.28), (11.5, 4.8, 0.78), (16.5, -7.5, 0.68)]):
    bd = make_rock(f"/World/boulders/b{k}", 500+k, br)
    xf = UsdGeom.Xformable(bd.GetPrim())
    xf.AddTranslateOp().Set(Gf.Vec3d(bx, by, br*0.36 + ground_z(bx, by)))
    xf.AddRotateZOp().Set(rng2.uniform(0, 360))
    xf.AddRotateXOp().Set(rng2.uniform(-10, 10)); xf.AddRotateYOp().Set(rng2.uniform(-10, 10))
    xf.AddScaleOp().Set(Gf.Vec3f(1.0, rng2.uniform(0.9, 1.1), rng2.uniform(0.6, 0.85)))
    UsdPhysics.CollisionAPI.Apply(bd.GetPrim()); count += 1
hero = make_rock("/World/boulders/hero", 4242, 0.85)
hxf = UsdGeom.Xformable(hero.GetPrim())
hxf.AddTranslateOp().Set(Gf.Vec3d(18.5, 5.8, 0.85*0.34 + ground_z(18.5, 5.8)))
hxf.AddRotateZOp().Set(215.0)
hxf.AddRotateXOp().Set(-7.0); hxf.AddRotateYOp().Set(5.0)
hxf.AddScaleOp().Set(Gf.Vec3f(1.15, 0.95, 0.62))          # broad, low-slung monolith
UsdPhysics.CollisionAPI.Apply(hero.GetPrim()); count += 1
print(f"[scene] rocks placed: {count} (procedural, corridor kept clear; incl. hero boulder 1.4 m)")

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
