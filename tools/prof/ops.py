#!/usr/bin/env python3
"""Per-handler execution counts ([PROFOP] lines) -> coverage table.
usage: ops.py <ares log> <elf> [nm] [--list N]   (--list prints the top N handler names, hottest first)"""
import sys, subprocess
log, elf = sys.argv[1], sys.argv[2]
nm = sys.argv[3] if len(sys.argv) > 3 and not sys.argv[3].startswith('--') else 'mips64-elf-nm'
syms = {}
for l in subprocess.run([nm, '-S', elf], capture_output=True, text=True).stdout.splitlines():
    p = l.split()
    if len(p) == 4 and p[2] in 'tT': syms[int(p[0][-8:], 16)] = (p[3], int(p[1], 16))
rows = []
for l in open(log, errors='replace'):
    if l.startswith('[PROFOP]'):
        _, a, n = l.split(); a = int(a, 16); name, size = syms.get(a, (hex(a), 0)); rows.append((int(n), name, size))
rows.sort(reverse=True); total = sum(r[0] for r in rows)
if '--list' in sys.argv:
    for n, name, size in rows[:int(sys.argv[sys.argv.index('--list') + 1])]: print(name.split('.lto_priv')[0])
    sys.exit()
acc = sz = 0
print(f"{len(rows)} handlers used, {total} instructions")
for i, (n, name, size) in enumerate(rows, 1):
    acc += n; sz += size
    if i <= 25 or i in (32, 48, 64, 96, 128, 160, 200, 250, len(rows)):
        print(f"{i:4d} {100*n/total:6.2f}% cum {100*acc/total:6.2f}%  size {size:5d}  cum size {sz:6d}  {name}")
