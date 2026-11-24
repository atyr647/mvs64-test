#!/usr/bin/env python3
"""[PROFPC] lines (interpreted 68K instructions per 32-byte block, -DPROFILER_PCS)
-> hottest contiguous regions.  usage: pcs.py <ares log> [N]"""
import sys
rows = sorted((int(l.split()[1], 16), int(l.split()[2])) for l in open(sys.argv[1], errors='replace') if l.startswith('[PROFPC]'))
total = sum(n for _, n in rows); regions = []
for pc, n in rows:
    if regions and pc - regions[-1][1] <= 64: regions[-1][1] = pc; regions[-1][2] += n
    else: regions.append([pc, pc, n])
regions.sort(key=lambda r: -r[2]); acc = 0
print(f"{total} interpreted instructions, {len(rows)} blocks, {len(regions)} regions")
for a, b, n in regions[:int(sys.argv[2]) if len(sys.argv) > 2 else 30]:
    acc += n; print(f"{a:06x}-{b + 31:06x}  {100 * n / total:5.1f}%  cum {100 * acc / total:5.1f}%")
