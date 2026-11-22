"""Real ares playback (SDL disk driver, f32 stereo 48 kHz): dropouts and level."""
import sys, numpy as np
from sndlib import Checks
raw, skip = sys.argv[1], int(sys.argv[2]) if len(sys.argv) > 2 else 40
C = Checks(); rate = 48000
x = np.fromfile(raw, '<f4'); x = x[:len(x) // 2 * 2].reshape(-1, 2).mean(axis=1)
secs = len(x) // rate; gaps = []; audible = 0
for i in range(skip, secs):
    seg = x[i * rate:(i + 1) * rate]
    if np.sqrt((seg ** 2).mean()) < 1e-6: continue
    audible += 1
    z = np.abs(seg) < 1e-6
    if z.any():
        edges = np.diff(np.r_[0, z.astype(np.int8), 0]); runs = np.flatnonzero(edges == -1) - np.flatnonzero(edges == 1)
        longest = runs.max() * 1000 / rate
        if 20 < longest < 100: gaps.append((i, longest))     # dropout-sized holes; longer ones are musical rests
print(f"      {secs} s captured, {audible} audible after {skip} s; peak {np.abs(x).max():.3f}")
C.check("no dropout-sized gaps (20-100 ms) in audible audio", len(gaps) <= 2, f"{len(gaps)} seconds affected {gaps[:5]}")
C.check("no clipping", np.abs(x).max() < 0.999, f"peak {np.abs(x).max():.3f}")
sys.exit(C.fail)
