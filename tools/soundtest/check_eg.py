"""FM envelope generator: amplitude envelopes (dB) of single carriers, RSP model vs ymfm."""
import sys, numpy as np
from sndlib import *
W = sys.argv[1]; C = Checks()
m = resample(load(f"{W}/model55_eg.pcm"), 55555.5555, OUT_RATE); r = 0.5 * resample(load(f"{W}/ref_eg_l.pcm"), YM_RATE, OUT_RATE)
n = min(len(m), len(r)); m, r = m[:n], r[:n]
labs = [(int(a), b) for a, b in (l.split() for l in open(f"{W}/eg_labels.txt"))] + [(n, 'end')]
win = 110   # 5 ms
def env_db(x):
    k = len(x) // win
    e = np.sqrt((x[:k * win].reshape(k, win) ** 2).mean(axis=1))
    return 20 * np.log10(np.maximum(e, 1.0))
for (s, name), (e, _) in zip(labs, labs[1:]):
    a, b = env_db(m[s:e]), env_db(r[s:e])
    live = (a > 6) | (b > 6)                         # ignore both-silent stretches
    d = np.abs(a - b)[live]
    # time for the envelope to fall below -40 dB of its peak (release / decay timing)
    def t_below(x, thr=40):
        p = x.argmax(); idx = np.flatnonzero(x[p:] < x[p] - thr); return (p + idx[0]) * 5 if len(idx) else None
    ta, tb = t_below(a), t_below(b)
    C.check(f"EG {name}", np.median(d) < 1.5 and np.percentile(d, 90) < 4,
            f"envelope diff median {np.median(d):.2f} dB, p90 {np.percentile(d,90):.2f} dB; peak->-40dB {ta} ms vs ymfm {tb} ms")
sys.exit(C.fail)
