"""ADPCM-A/B and SSG, driven by the real M1 driver: ours vs ymfm."""
import sys, numpy as np
from sndlib import *
W = sys.argv[1]; C = Checks()
for name, ours, ref, hp, ratio_lo, ratio_hi in [
        ("ADPCM-A+B left", "ours_ab.pcm", "ref_ab.pcm", False, 0.49, 0.51),
        ("ADPCM-A+B right", "ours_ab_r.pcm", "ref_ab_r.pcm", False, 0.49, 0.51)]:
    o = load(f"{W}/{ours}"); r = resample(load(f"{W}/{ref}"), YM_RATE, OUT_RATE)
    n = min(len(o), len(r)); o, r = o[:n], r[:n]
    cs, ratios = [], []
    for s in range(0, n - 11025, 11025):
        a, b = o[s:s + 11025], r[s:s + 11025]
        if rms(b) < 20 and rms(a) < 20: continue
        cs.append(best_corr(a, b)[0]); ratios.append(rms(a) / max(rms(b), 1e-9))
    C.check(f"{name} correlation", np.median(cs) > 0.995 and min(cs) > 0.9, f"median {np.median(cs):.4f} over {len(cs)} windows (min {min(cs):.3f})")
    C.check(f"{name} level", ratio_lo < np.median(ratios) < ratio_hi, f"ours/ymfm {np.median(ratios):.3f} (expected 0.500: our deliberate x0.5)")
sys.exit(C.fail)
