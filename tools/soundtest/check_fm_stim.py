"""FM note stimuli through the C model of rsp_fm.S at 55.5 kHz vs ymfm."""
import sys, numpy as np
from sndlib import *
W = sys.argv[1]; C = Checks()
m = resample(load(f"{W}/model55_stim.pcm"), 55555.5555, OUT_RATE); r = 0.5 * resample(load(f"{W}/ref_fm_l.pcm"), YM_RATE, OUT_RATE)
n = min(len(m), len(r)); m, r = m[:n], r[:n]
labs = [(int(a), b) for a, b in (l.split() for l in open(f"{W}/fm_labels.txt"))] + [(n, 'end')]
mR = resample(load(f"{W}/model55_stim_r.pcm"), 55555.5555, OUT_RATE)[:n]; rR = 0.5 * resample(load(f"{W}/ref_fm_r.pcm"), YM_RATE, OUT_RATE)[:n]
for (s, name), (e, _) in zip(labs, labs[1:]):
    a, b = s + 10 * 368 + 2000, e - 500
    if name.startswith('F_pan'):
        C.check(f"FM {name}", rms(m[a:b]) > 500 and rms(mR[a:b]) < 1 and rms(rR[a:b]) < 1 and abs(rms(m[a:b]) / rms(r[a:b]) - 1) < 0.25,
                f"left {rms(m[a:b]):.0f} (ymfm {rms(r[a:b]):.0f}), right {rms(mR[a:b]):.1f} (ymfm {rms(rR[a:b]):.1f})")
        continue
    x, y = m[a:b], r[a:b]
    if name.startswith(('G_', 'H_')):
        fx, fy = peak_freq(x), peak_freq(y)
        C.check(f"FM {name}", abs(fx - fy) < 1.0 and abs(rms(x) / rms(y) - 1) < 0.15,
                f"pitch {fx:.2f} Hz (ymfm {fy:.2f}), level {rms(x) / rms(y):.2f}")
        continue
    ratio = rms(x) / max(rms(y), 1e-9)
    px, py = peaks(x), peaks(y); common = len(set(px) & set(py))
    lo = 0.75 if name.startswith('E_') else 0.85
    C.check(f"FM {name}", lo < ratio < 1.15 and common >= 3, f"level {ratio:.2f}, partials ours {px} ymfm {py}")
sys.exit(C.fail)
