#!/usr/bin/env python3
"""Pick AOT entry points from a full interpreter profile
(-DPROFILER_PCS -DPROFILER_CALLS with a minimal HLE_PCS).
Every BSR/JSR target is a candidate function; it is credited with the
interpreted instructions from its entry up to the next candidate.
usage: autopick.py <ares log> [min share %] [exclude lo-hi ...]   -> prints an HLE_PCS list"""
import sys, bisect
log = sys.argv[1]; minshare = float(sys.argv[2]) if len(sys.argv) > 2 else 0.3
excl = [tuple(int(x, 16) for x in a.split('-')) for a in sys.argv[3:]]
blocks = sorted((int(l.split()[1], 16), int(l.split()[2])) for l in open(log, errors='replace') if l.startswith('[PROFPC]'))
calls = sorted({int(l.split()[1], 16) for l in open(log, errors='replace') if l.startswith('[PROFCALL]')})
total = sum(n for _, n in blocks); score = {}
for pc, n in blocks:
    # a 32-byte block belongs to the last call target at or before its end
    i = bisect.bisect_right(calls, pc + 31) - 1
    if i < 0: continue
    t = calls[i]
    if pc + 31 - t > 0x600: continue            # too far from any entry: not this function
    score[t] = score.get(t, 0) + n
picked = [(n, t) for t, n in score.items() if 100 * n / total >= minshare and not any(lo <= t < hi for lo, hi in excl)]
picked.sort(reverse=True); acc = 0
for n, t in picked:
    acc += n; print(f"# {t:06x}  {100 * n / total:5.2f}%  cum {100 * acc / total:5.1f}%", file=sys.stderr)
print(' '.join(f"{t:x}" for _, t in picked))
