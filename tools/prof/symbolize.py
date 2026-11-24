#!/usr/bin/env python3
"""Symbolize a [PROFN]/[PROFM] dump from a -DPROFILER build.
usage: symbolize.py <ares log> <elf> [nm binary]"""
import sys, subprocess, bisect, collections
log, elf = sys.argv[1], sys.argv[2]
nm = sys.argv[3] if len(sys.argv) > 3 else 'mips64-elf-nm'
syms = []
for l in subprocess.run([nm, '-n', elf], capture_output=True, text=True).stdout.splitlines():
    p = l.split()
    if len(p) == 3 and p[1] in 'tT': syms.append((int(p[0][-8:], 16), p[2]))
addrs = [a for a, _ in syms]
text = total = other = 0; fn = collections.Counter(); m68k = collections.Counter()
for l in open(log, errors='replace'):
    if l.startswith('[PROF] total='):
        f = dict(x.split('=') for x in l.split()[1:]); total = int(f['total']); other = int(f['other']); text = int(f['text'], 16)
    elif l.startswith('[PROFN]'):
        _, off, n = l.split(); a = text + int(off, 16) + 64
        i = bisect.bisect_right(addrs, a) - 1
        fn[syms[i][1] if i >= 0 else '?'] += int(n)
    elif l.startswith('[PROFM]'):
        _, pc, n = l.split(); m68k[int(pc, 16)] += int(n)
print(f"samples {total}, outside text {other} ({100*other/max(total,1):.1f}%)")
acc = 0
for name, n in fn.most_common(60):
    acc += n; print(f"{100*n/total:6.2f}%  {100*acc/total:6.2f}%  {name}")
print("\n68K PC (top 40):")
for pc, n in m68k.most_common(40): print(f"{100*n/total:6.2f}%  {pc:06x}")
