#!/usr/bin/env python3
"""Symbolize a cache-miss profile written by the patched ares (-DPROFILER_MISS).
usage: missprof.py <missprof.txt> <elf> [nm] [frames]"""
import sys, subprocess, bisect, collections
path, elf = sys.argv[1], sys.argv[2]
nm = sys.argv[3] if len(sys.argv) > 3 else 'mips64-elf-nm'
frames = int(sys.argv[4]) if len(sys.argv) > 4 else 1500
syms = sorted((int(p[0][-8:], 16), p[2]) for p in (l.split() for l in subprocess.run([nm, '-n', elf], capture_output=True, text=True).stdout.splitlines()) if len(p) == 3)
addrs = [a for a, _ in syms]
def sym(a):
    i = bisect.bisect_right(addrs, a) - 1
    return syms[i][1].split('.lto_priv')[0] if i >= 0 and a - syms[i][0] < 0x100000 else f'<{a:08x}>'
tabs = {'IC': collections.Counter(), 'DPC': collections.Counter(), 'DAD': collections.Counter()}
pairs = collections.Counter()
for l in open(path):
    f = l.split()
    if f[0] == 'DPR': pairs[(sym(int(f[1], 16)), sym(int(f[2], 16)))] += int(f[3])
    else: tabs[f[0]][sym(int(f[1], 16))] += int(f[2])
for t, title in (('IC', 'I-cache misses by function'), ('DPC', 'D-cache misses by the code that accessed'), ('DAD', 'D-cache misses by the data accessed')):
    tot = sum(tabs[t].values()); acc = 0
    print(f"\n{title}: {tot / frames:.0f} per frame")
    for name, n in tabs[t].most_common(22):
        acc += n; print(f"  {n / frames:7.1f}/frame {100 * n / tot:5.1f}%  cum {100 * acc / tot:5.1f}%  {name}")
tot = sum(pairs.values()); acc = 0
print(f"\nD-cache misses by (code, data):")
for (a, b), n in pairs.most_common(30):
    acc += n; print(f"  {n / frames:7.1f}/frame {100 * n / tot:5.1f}%  cum {100 * acc / tot:5.1f}%  {a} -> {b}")
