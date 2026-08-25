"""
SuRover Mars ground baker - seamless procedural albedo + normal map.
Output: /workspace/assets/mars_ground.jpg, /workspace/assets/mars_normal.jpg (2048x2048, tileable)
All noise is generated in the frequency domain, so the texture tiles without seams.
"""
import numpy as np
from PIL import Image
S, SEED, OUT = 2048, 7, "/workspace/assets"

def fnoise(beta, seed):
    """1/f^beta coloured noise, periodic, normalised to [0,1]."""
    r = np.random.default_rng(seed)
    F = np.fft.fft2(r.normal(size=(S, S)))
    fx = np.fft.fftfreq(S)
    f = np.sqrt(fx[:, None]**2 + fx[None, :]**2); f[0, 0] = 1.0
    n = np.real(np.fft.ifft2(F / f**beta))
    n -= n.min(); n /= (n.max() + 1e-9)
    return n

def bandnoise(f_lo, f_hi, seed):
    """Band-passed white noise (cycles per texture), periodic, unit std."""
    r = np.random.default_rng(seed)
    F = np.fft.fft2(r.normal(size=(S, S)))
    fx = np.fft.fftfreq(S) * S
    f = np.sqrt(fx[:, None]**2 + fx[None, :]**2)
    n = np.real(np.fft.ifft2(F * ((f >= f_lo) & (f <= f_hi))))
    return (n - n.mean()) / (n.std() + 1e-9)

low = fnoise(2.2, SEED)        # dust patches, metres
mid = fnoise(1.6, SEED + 1)    # mottling
hi  = fnoise(1.0, SEED + 2)    # grain

# cracks: zero-crossings of a band-passed field form a cellular network; gated into patches
cb = bandnoise(12, 30, SEED + 3)
crack = np.exp(-(cb / 0.08)**2)
gate = np.clip((fnoise(2.0, SEED + 4) - 0.30) / 0.35, 0, 1)
crack *= gate
print(f"crack coverage: {float((crack > 0.3).mean()):.3f}")

peb = (bandnoise(150, 400, SEED + 5) > 2.6).astype(float)   # sparse light pebbles

warm = np.array([0.84, 0.52, 0.31])
dark = np.array([0.50, 0.25, 0.14])
base = np.array([0.70, 0.36, 0.20])
t = 0.40*low + 0.42*mid + 0.18*hi
t = (t - t.min()) / (t.max() - t.min())
col = dark*(1 - t)[..., None] + warm*t[..., None]
col = 0.55*col + 0.45*base
col *= (0.96 + 0.08*hi)[..., None]
col *= (1.0 - 0.42*crack)[..., None]
col = col*(1 - 0.5*peb[..., None]) + np.array([0.82, 0.64, 0.50])*(0.5*peb[..., None])
Image.fromarray((np.clip(col, 0, 1)*255).astype(np.uint8)).save(f"{OUT}/mars_ground.jpg", quality=93)

h = 0.55*low + 0.30*mid + 0.15*hi - 0.35*crack + 0.25*peb
gy, gx = np.gradient(h)
s = 60.0
nx, ny, nz = -gx*s, -gy*s, np.ones_like(h)
L = np.sqrt(nx*nx + ny*ny + nz*nz)
nrm = np.stack([nx/L, ny/L, nz/L], -1)*0.5 + 0.5
Image.fromarray((nrm*255).astype(np.uint8)).save(f"{OUT}/mars_normal.jpg", quality=95)
print("GROUND-OK")
