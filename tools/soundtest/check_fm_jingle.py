"""Blazing Star FM jingle (cmd $5F) through the RSP C model vs ymfm."""
import sys, numpy as np
from sndlib import *
W = sys.argv[1]; C = Checks()
t0, N = 159 * 368, 88200
bands = [(0, 500), (500, 1000), (1000, 2000), (2000, 4000), (4000, 11025)]
ref = 0.5 * resample(load(f"{W}/ref_st_l.pcm"), YM_RATE, OUT_RATE)[t0:t0 + N]
for rate, path in ((22050, "model22.pcm"), (55555.5555, "model55.pcm")):
    x = load(f"{W}/{path}"); x = (x if rate == 22050 else resample(x, rate, OUT_RATE))[t0:t0 + N]
    bo, br = band_shares(x, bands), band_shares(ref, bands)
    worst = max(abs(a - b) for a, b in zip(bo, br)); lsc = log_spec_corr(x, ref); ratio = rms(x) / rms(ref)
    # at 22050 Hz strong FM partials alias, so the spectrum match is looser there
    C.check(f"FM jingle model@{int(rate)}", worst < 4 and lsc > (0.9 if rate == 22050 else 0.95) and 0.95 < ratio < 1.05,
            f"level {ratio:.2f}, log-spectrum corr {lsc:.3f}, band shares {['%.0f%%' % v for v in bo]} vs ymfm {['%.0f%%' % v for v in br]}")
# right channel: Blazing Star pans FM channels 3-4 left-only during this jingle
refR = 0.5 * resample(load(f"{W}/ref_st_r.pcm"), YM_RATE, OUT_RATE)[t0:t0 + N]
xR = load(f"{W}/model22_r.pcm")[t0:t0 + N]
ratio = rms(xR) / rms(refR); ratioL = rms(load(f"{W}/model22.pcm")[t0:t0 + N]) / rms(ref)
C.check("FM jingle stereo (right channel)", 0.95 < ratio < 1.05 and log_spec_corr(xR, refR) > 0.9,
        f"right level {ratio:.2f} vs left {ratioL:.2f}; ymfm R/L rms {rms(refR)/rms(ref):.2f}, ours {rms(xR)/rms(load(f'{W}/model22.pcm')[t0:t0+N]):.2f}")
sys.exit(C.fail)
