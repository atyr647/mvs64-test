"""SSG register stimulus (all volumes, noise, envelopes): ours vs ymfm."""
import sys, numpy as np
from sndlib import *
W = sys.argv[1]; C = Checks()
o = highpass(load(f"{W}/ours_ssgstim.pcm")); r = highpass(resample(load(f"{W}/ref_ssgstim.pcm"), YM_RATE, OUT_RATE))
n = min(len(o), len(r)); o, r = o[:n], r[:n]
labs = [(int(a), b) for a, b in (l.split() for l in open(f"{W}/ssg_labels.txt"))] + [(n, 'end')]
target = 0.84 / 0.98 * 0.5      # MAME Neo Geo SSG/FM routing x our 0.5
for (s, name), (e, _) in zip(labs, labs[1:]):
    a, b = o[s + 2000:e], r[s + 2000:e]
    if name in ('silence', 'envA_shape00') or len(a) < 1000: continue
    c, lag = best_corr(a, b); ratio = rms(a) / max(rms(b), 1e-9)
    need = 0.70 if name.startswith('3tones') else 0.90   # 3 channels at different phases cannot all align
    C.check(f"SSG {name}", c > need and abs(ratio - target) < 0.04, f"corr {c:.3f}, level {ratio:.3f} (target {target:.3f})")
sys.exit(C.fail)
