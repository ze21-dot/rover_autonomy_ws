"""Mars sky dome (equirect 2048x1024): zenith dusty red -> bright butterscotch horizon, dust haze band,
soft cloud/dust streaks, sun glow. Seamless in azimuth (FFT noise)."""
import numpy as np
from PIL import Image
W, H, SEED = 2048, 1024, 11
v = np.linspace(0, 1, H)[:, None]            # 0 zenith, 0.5 horizon, 1 nadir
u = np.linspace(0, 1, W)[None, :]
def fnoise(beta, seed, shape=(H, W), stretch=6.0):
    r = np.random.default_rng(seed)
    F = np.fft.fft2(r.normal(size=shape))
    fy = np.fft.fftfreq(shape[0]); fx = np.fft.fftfreq(shape[1])
    f = np.sqrt(fy[:, None]**2 + (fx[None, :]*stretch)**2); f[0, 0] = 1.0
    n = np.real(np.fft.ifft2(F / f**beta)); n -= n.min(); n /= n.max() + 1e-9
    return n
zen = np.array([0.50, 0.33, 0.27]); hor = np.array([0.96, 0.74, 0.54]); grd = np.array([0.50, 0.30, 0.21])
t = np.clip(v*2, 0, 1)**1.4
sky = zen*(1-t)[..., None] + hor*t[..., None]
b = np.clip((v-0.5)*2, 0, 1)**0.5
col = np.broadcast_to(sky*(1-b)[..., None] + grd*b[..., None], (H, W, 3)).copy()
clouds = fnoise(2.2, SEED)*0.20 + fnoise(1.5, SEED+1)*0.10        # streaky dust clouds
band = np.exp(-((v-0.30)/0.16)**2)                                  # only in the upper-mid sky
col *= (1.0 + (clouds - 0.15)*band*2.2)[..., None]
haze = np.exp(-((v-0.5)/0.05)**2)                                   # horizon dust haze
col = col*(1-0.35*haze)[..., None] + np.array([0.98, 0.80, 0.60])*(0.35*haze)[..., None]
az = (u-0.5)*2*np.pi; el = (0.5-v)*np.pi
sa, se = np.radians(55), np.radians(35)
cosang = np.sin(el)*np.sin(se) + np.cos(el)*np.cos(se)*np.cos(az-sa)
col += np.exp(-(1-cosang)*30)[..., None]*np.array([0.55, 0.40, 0.25])
Image.fromarray((np.clip(col, 0, 1)*255).astype(np.uint8)).save("/workspace/assets/mars_sky.jpg", quality=92)
print("SKY-OK")
