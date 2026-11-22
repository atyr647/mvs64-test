"""The jingle as the real RSP rendered it in ares (SND_SELFTEST log) vs ymfm."""
import sys, numpy as np
from sndlib import *
LOG, W = sys.argv[1], sys.argv[2]; C = Checks()
ours = wav_from_log(LOG)
if len(ours) < 44100:
    C.check("ares capture", False, f"only {len(ours)} samples in {LOG}"); sys.exit(1)
L = resample(load(f"{W}/ref_st_l.pcm"), YM_RATE, OUT_RATE); G = resample(load(f"{W}/ref_st_ssg.pcm"), YM_RATE, OUT_RATE)
ref = 0.5 * L + (0.84 / 0.98 * 0.5) * G
t0 = 159 * 368; seg = ref[t0 - 2000:t0 + len(ours) + 4000]
env = lambda x: np.convolve(np.abs(x), np.ones(110) / 110, 'same')
eo = env(ours[:44100]); best = (-1, 0)
for off in range(0, 6000, 5):
    c = np.corrcoef(eo, env(seg[off:off + 44100]))[0, 1]
    if c > best[0]: best = (c, off)
r = seg[best[1]:best[1] + len(ours)]; o = ours[:len(r)]
bands = [(0, 500), (500, 1000), (1000, 2000), (2000, 4000), (4000, 11025)]
bo, br = band_shares(o, bands), band_shares(r, bands)
ratio = rms(o) / rms(r); lsc = log_spec_corr(o, r)
C.check("ares RSP jingle", 0.8 < ratio < 1.2 and lsc > 0.7 and max(abs(a - b) for a, b in zip(bo, br)) < 8,
        f"level {ratio:.2f}, envelope corr {best[0]:.2f}, log-spectrum corr {lsc:.3f}, bands {['%.0f%%' % v for v in bo]} vs ymfm {['%.0f%%' % v for v in br]}")
sys.exit(C.fail)
