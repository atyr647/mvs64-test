#!/usr/bin/env python3
"""m68k_abi.py m68kops.c m68kops_layout.h OUT.c

Rewrite Musashi's handlers for a leaner calling convention, as a build step
(genhle keeps reading the original m68kops.c):

    static void m68k_op_X(void)   ->   static int m68k_op_X(uint __ir)

The opcode arrives in a register instead of being reloaded from the CPU
struct (REG_IR is still stored by the dispatch loop, for the helpers and the
exception code that read it), and the handler returns the cycles it used,
counted in a local, instead of a load/subtract/store of the global counter
per USE_CYCLES. Exactly equivalent to the original accounting:
  - USE_CYCLES(n) adds to the local; everything outside the handler
    (exceptions, interrupts) still adjusts the global, and the dispatch loop
    subtracts the local afterwards, so additive changes commute;
  - USE_ALL_CYCLES and m68k_op_stop's assignments are rewritten in terms of
    G = global - local, the value the original would have seen;
  - the idle skip (m68k_consume_timeslice in the branch helpers) sets the
    local to the global, so the loop's subtraction leaves exactly 0 plus
    whatever the handler uses after the branch (m68kcpu.h, M68K_HANDLER_ABI);
  - m68k_end_timeslice works on initial - remaining, invariant under the
    reordering.
"""
import re, sys

src, layout, out = sys.argv[1:4]
s = open(src).read()
lay = open(layout).read()

# the layout header's prototypes, with the new signature, inlined
lay = re.sub(r'static void (m68k_op_\w+)\(void\)', r'static int \1(uint __ir)', lay)
assert '#include "m68kops_layout.h"' in s
s = s.replace('#include "m68kcpu.h"', '#define M68K_HANDLER_ABI 1\n#include "m68kcpu.h"', 1)
s = s.replace('#include "m68kops_layout.h"', lay + '''
/* handler ABI (tools/m68k_abi.py): cycles counted in a local, opcode in a register */
#undef USE_CYCLES
#define USE_CYCLES(A) (__cyc += (A))
#undef USE_ALL_CYCLES
#define USE_ALL_CYCLES() (__cyc = m68ki_remaining_cycles - ((m68ki_remaining_cycles - __cyc) % CYC_INSTRUCTION[REG_IR]))
#undef REG_IR
#define REG_IR __ir
''', 1)

# m68k_op_stop assigns the counter directly
old_stop = '''		if(m68ki_remaining_cycles >= CYC_INSTRUCTION[REG_IR])
			m68ki_remaining_cycles = CYC_INSTRUCTION[REG_IR];'''
assert s.count(old_stop) == 1
s = s.replace(old_stop, '''		if(m68ki_remaining_cycles - __cyc >= CYC_INSTRUCTION[REG_IR])
			__cyc = m68ki_remaining_cycles - CYC_INSTRUCTION[REG_IR];''')

lines = s.split('\n')
o = []; in_h = False; n = 0
for i, l in enumerate(lines):
    m = re.match(r'^static void (m68k_op_\w+)\(void\)$', l)
    if m:
        o.append('static int %s(uint __ir)' % m.group(1)); in_h = True; n += 1
        continue
    if in_h and l == '{':
        o.append('{'); o.append('\tint __cyc = 0;'); continue
    if in_h and l == '}':
        o.append('\treturn __cyc;'); o.append('}'); in_h = False; continue
    if in_h:
        l = re.sub(r'\breturn;', 'return __cyc;', l)
        assert 'm68ki_remaining_cycles' not in l or '__cyc' in l, (i, l)
    o.append(l)
s = '\n'.join(o)

old_tab = '\nvoid (*m68ki_instruction_jump_table[0x10000])(void) = {'
assert s.count(old_tab) == 1
s = s.replace(old_tab, '\nint (*m68ki_instruction_jump_table_abi[0x10000])(uint) = {')
open(out, 'w').write(s)
print('m68k_abi: %d handlers rewritten' % n)
