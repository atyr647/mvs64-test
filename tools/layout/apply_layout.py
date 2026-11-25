#!/usr/bin/env python3
"""apply_layout.py BASE.ld LAYOUT.ld OUT.ld: merge realize2.py's cache-coloured
placement into the linker script generated from libdragon's n64.ld (mvs64.ld).

LAYOUT.ld holds `.text : { ... }`, `.rodata : { ... }`, `.data : { ... }` and
`.bss : { ... }` blocks of input-section patterns and padding statements
(`. += (color - .) & mask;`). Each block's body is inserted at the start of the
same output section of BASE.ld: in .text right after *(.boot), replacing the
SORT(.text.m68k.*) hot block (the colouring places those sections itself); in
the data sections before their wildcard patterns. A section matched by an
earlier pattern is not matched again, so the coloured entries win and every
other input section falls through to the usual wildcards."""
import re, sys

base, layout, out = sys.argv[1:4]
blocks = dict(re.findall(r'^(\.\w+) : \{\n(.*?)\n\}', open(layout).read(), re.S | re.M))
s = open(base).read()

def insert_after(s, anchor, body):
    i = s.index(anchor) + len(anchor)
    return s[:i] + '\n' + body + s[i:]

if '.text' in blocks:
    s = re.sub(r'\n *\*\(SORT\(\.text\.m68k\.\*\)\)', '', s)
    s = insert_after(s, '*(.boot)', blocks['.text'])
if '.rodata' in blocks:
    s = insert_after(s, '.rodata : {', blocks['.rodata'])
if '.data' in blocks:
    s = insert_after(s, '__data_start = .;', blocks['.data'])
if '.bss' in blocks:
    s = insert_after(s, '.bss : {', blocks['.bss'])
open(out, 'w').write(s)
print('applied', ', '.join('%s (%d lines)' % (k, v.count('\n') + 1) for k, v in blocks.items()))
