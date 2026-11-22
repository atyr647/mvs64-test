"""FM note stimuli through the C model of rsp_fm.S at 55.5 kHz vs ymfm."""
import sys, numpy as np
from sndlib import *
W = sys.argv[1]; C = Checks()
m = resample(load(f"{W}/model55_stim.pcm"), 55555.5555, OUT_RATE); r = 0.5 * resample(load(f"{W}/ref_fm_l.pcm"), YM_RATE, OUT_RATE)
n = min(len(m), len(r)); m, r = m[:n], r[:n]
labs = [(int(a), b) for a, b in (l.split() for l in open(f"{W}/fm_labels.txt"))] + [(n, 'end')]
for (s, name), (e, _) in zip(labs, labs[1:]):
    a, b = s + 10 * 368 + 2000, e - 500
    x, y = m[a:b], r[a:b]
    ratio = rms(x) / max(rms(y), 1e-9)
    px, py = peaks(x), peaks(y); common = len(set(px) & set(py))
    lo = 0.75 if name.startswith('E_') else 0.85
    C.check(f"FM {name}", lo < ratio < 1.15 and common >= 3, f"level {ratio:.2f}, partials ours {px} ymfm {py}")
sys.exit(C.fail)
