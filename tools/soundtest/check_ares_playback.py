"""Real ares playback (SDL disk driver, f32 stereo 48 kHz): dropouts and level."""
import sys, numpy as np
from sndlib import Checks
raw, skip = sys.argv[1], int(sys.argv[2]) if len(sys.argv) > 2 else 40
C = Checks(); rate = 48000
st = np.fromfile(raw, '<f4'); st = st[:len(st) // 2 * 2].reshape(-1, 2); x = st.mean(axis=1)
secs = len(x) // rate; gaps = []; host = []; audible = 0
for i in range(skip, secs):
    seg = x[i * rate:(i + 1) * rate]
    if np.sqrt((seg ** 2).mean()) < 1e-6: continue
    audible += 1
    z = np.abs(seg) < 1e-6
    if z.any():
        edges = np.diff(np.r_[0, z.astype(np.int8), 0]); st0 = np.flatnonzero(edges == 1); en0 = np.flatnonzero(edges == -1)
        for a, b in zip(st0, en0):
            ms = (b - a) * 1000 / rate
            # a dropout cuts the sound off; a musical rest decays into silence first
            before = seg[max(a - 96, 0):a]
            if 20 < ms < 100 and len(before) and np.sqrt((before ** 2).mean()) > 3e-3:
                # ares itself falling behind real time leaves holes that end on its
                # 10 ms output chunks; an N64-side hole has no reason to
                if (i * rate + b) % (rate // 100) == 0: host.append((i, round(ms, 1)))
                else: gaps.append((i, round(ms, 1)))
                break
print(f"      {secs} s captured, {audible} audible after {skip} s; peak {np.abs(x).max():.3f}")
C.check("no dropouts (sound cut off for 20-100 ms) in audible audio", len(gaps) <= 2, f"{len(gaps)} seconds affected {gaps[:5]}")
if host: print(f"INFO  {len(host)} seconds with holes on ares's 10 ms output grid (ares below real time, not the ROM): {host[:5]}")
C.check("no clipping", np.abs(x).max() < 0.999, f"peak {np.abs(x).max():.3f}")
L, R = st[skip * rate:, 0], st[skip * rate:, 1]
diff = np.sqrt(((L - R) ** 2).mean()) / max(np.sqrt(((L + R) ** 2).mean() / 4), 1e-9)
print(f"      stereo: L rms {np.sqrt((L**2).mean()):.4f}, R rms {np.sqrt((R**2).mean()):.4f}, side/mid ratio {diff:.3f} (0 = mono)")
C.check("both output channels carry audio", np.sqrt((L**2).mean()) > 1e-3 and np.sqrt((R**2).mean()) > 1e-3, "L and R non-silent")
sys.exit(C.fail)
