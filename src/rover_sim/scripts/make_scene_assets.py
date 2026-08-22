"""Regenerate the sealed showcase textures (scene v31 look, deterministic).

Outputs:
  /workspace/assets/sky_v6.png                     latlong sky, red palette, dust veils
  /workspace/assets/ground/hills_clean.jpg         clean hill texture (separate file by design)
  /workspace/assets/ground/mars_ground_baked.jpg   ground albedo with baked Voronoi plate cracks
  /workspace/assets/ground/mars_ground_normal.png  crack normal map (sobel)

Base surface: ambientCG Ground054 4K (CC0 photogrammetric scan), fetched if absent.
"""
import glob
import os
import subprocess

import numpy as np
from PIL import Image
from scipy import ndimage as ndi

ASSETS = "/workspace/assets"
GROUND = f"{ASSETS}/ground"
os.makedirs(GROUND, exist_ok=True)


def fetch_base():
    color = glob.glob(f"{ASSETS}/ground054/*Color.jpg")
    if not color:
        os.makedirs(f"{ASSETS}/ground054", exist_ok=True)
        subprocess.run(
            ["wget", "-q", "https://ambientcg.com/get?file=Ground054_4K-JPG.zip",
             "-O", f"{ASSETS}/ground054.zip"], check=True)
        subprocess.run(["unzip", "-q", "-o", f"{ASSETS}/ground054.zip",
                        "-d", f"{ASSETS}/ground054"], check=True)
        color = glob.glob(f"{ASSETS}/ground054/*Color.jpg")
    assert color, "Ground054 color map not found"
    return color[0]


def make_hills_clean(base_path):
    img = Image.open(base_path).convert("RGB")
    img.save(f"{GROUND}/hills_clean.jpg", quality=92)


def bake_ground():
    src = Image.open(f"{GROUND}/hills_clean.jpg").convert("RGB")
    a = np.asarray(src).astype(np.float32) / 255.0
    H, W = a.shape[:2]
    yy, xx = np.meshgrid(np.linspace(0, 24, H), np.linspace(0, 24, W), indexing="ij")
    rngp = np.random.default_rng(31)
    nx = ndi.gaussian_filter(rngp.normal(0, 1, (H, W)), 18) * 1.4
    ny = ndi.gaussian_filter(rngp.normal(0, 1, (H, W)), 18) * 1.4
    xw, yw = xx + nx, yy + ny
    vr = np.random.default_rng(23)
    seeds = np.stack([vr.uniform(-2, 26, 70), vr.uniform(-2, 26, 70)], 1)
    P = np.stack([xw.ravel(), yw.ravel()], 1)
    step = 3
    Ps = P[::step]
    d = np.sqrt(((Ps[:, None, :] - seeds[None, :, :]) ** 2).sum(-1))
    d.sort(axis=1)
    crack_s = np.clip(1.0 - (d[:, 1] - d[:, 0]) / 0.14, 0, 1) ** 1.8
    depth = np.zeros(P.shape[0], np.float32)
    depth[::step] = crack_s
    depth = ndi.grey_dilation(depth.reshape(H, W), size=(4, 4))
    depth = ndi.gaussian_filter(depth, 1.6)
    alb = np.clip(a * (1.0 - 0.22 * depth)[:, :, None], 0, 1)
    Image.fromarray((alb * 255).astype(np.uint8)).save(
        f"{GROUND}/mars_ground_baked.jpg", quality=92)
    gx = ndi.sobel(depth, axis=1)
    gy = ndi.sobel(depth, axis=0)
    nrm = np.dstack([-gx * 2.4, -gy * 2.4, np.ones_like(depth)])
    nrm /= np.linalg.norm(nrm, axis=2, keepdims=True)
    Image.fromarray(((nrm * 0.5 + 0.5) * 255).astype(np.uint8)).save(
        f"{GROUND}/mars_ground_normal.png")


def bake_sky():
    W, H = 2048, 1024
    img = np.zeros((H, W, 3), np.float32)
    top = np.array([0.55, 0.26, 0.10])
    hor = np.array([0.95, 0.55, 0.28])
    gnd = np.array([0.70, 0.38, 0.20])
    for r in range(H):
        t = r / (H - 1)
        if t < 0.48:
            img[r] = gnd
        elif t < 0.56:
            img[r] = gnd + (hor - gnd) * ((t - 0.48) / 0.08)
        else:
            img[r] = hor + (top - hor) * (((t - 0.56) / 0.44) ** 0.75)
    rng = np.random.default_rng(9)
    m1 = ndi.gaussian_filter(rng.normal(0, 1, (H, W)), sigma=(14, 120), mode="wrap")
    m2 = ndi.gaussian_filter(rng.normal(0, 1, (H, W)), sigma=(3, 55), mode="wrap")
    for r in range(H):
        m1[r] = np.roll(m1[r], int(r * 0.05))
        m2[r] = np.roll(m2[r], int(r * 0.09))
    m1 = (m1 - m1.mean()) / (m1.std() + 1e-6)
    m2 = (m2 - m2.mean()) / (m2.std() + 1e-6)
    sky0 = int(H * 0.56)
    for r in range(sky0, H):
        t = (r - sky0) / (H - sky0)
        fade = 1.0 - 0.45 * t
        v = (np.clip(m1[r], -1.1, 1.8) * 0.085 + np.clip(m2[r], -1.1, 1.8) * 0.045) * fade
        img[r, :, 0] += v
        img[r, :, 1] += v * 0.75
        img[r, :, 2] += v * 0.55
    img += rng.normal(0, 0.004, img.shape)
    Image.fromarray((np.clip(img, 0, 1) * 255).astype(np.uint8)[::-1]).save(
        f"{ASSETS}/sky_v6.png")


if __name__ == "__main__":
    base = fetch_base()
    make_hills_clean(base)
    bake_ground()
    bake_sky()
    print("ASSETS-OK")
