#!/usr/bin/env bash
# Regenerate the cache-coloured layout for one ROM (see README.md).
#   ARES=/path/to/ares-with-cachetrace BIOS=bios.rom tools/layout/regen.sh WORKDIR [ROM.zip]
# Builds two trace ROMs (light section from frame 3000, heavy from 6500, 30
# frames each), records exact cache traces in the patched ares, anneals I-cache
# and D-cache positions, and writes tools/layout/<rom>.ld. ~35 min on 4 cores.
set -e
W=$(mkdir -p "$1" && cd "$1" && pwd); GAME=${2:-blazstar.zip}; ROMNAME=$(basename "${GAME%.zip}")
: "${ARES:?set ARES to an ares built with tools/layout/ares-cachetrace.patch}"
: "${BIOS:?set BIOS to the NeoGeo BIOS}"
REPO=$(cd "$(dirname "$0")/../.." && pwd); T=$REPO/tools/layout
cd "$REPO"
for start in 3000 6500; do
  rm -rf build/mvs64 hle_*.c hle_index.h m68kops_layout.h mvs64.ld
  # trace the plain layout: colours do not depend on where sections start
  make -f Makefile.mvs64 USE_HLE=1 LTO_SECTIONS=1 CACHE_LAYOUT= BIOS="$BIOS" ROM="$GAME" \
       SND_CFLAGS="-DAUTOPLAY -DCACHE_TRACE -DTRACE_START=$start" -j"$(nproc)" > "$W/build$start.log" 2>&1
  cp mvs64-"$ROMNAME".z64 "$W/t$start.z64"; cp build/mvs64/mvs64-"$ROMNAME".map "$W/t$start.map"
  cp build/mvs64/mvs64-"$ROMNAME".elf "$W/t$start.elf"
done
rm -f mvs64.ld
cd "$W"
python3 "$T/map_sections.py" t3000.map t3000.secs; python3 "$T/map_sections.py" t6500.map t6500.secs
# both windows must come from identically linked builds (only an immediate differs)
cmp -s <(awk '{print $1,$2,$3}' t3000.secs) <(awk '{print $1,$2,$3}' t6500.secs) || { echo "trace builds differ"; exit 1; }
(ARES="$ARES" bash "$T/ares_trace.sh" t3000.z64 tr3000 3000 &
 ARES="$ARES" bash "$T/ares_trace.sh" t6500.z64 tr6500 3000 & wait)
cat tr3000.i tr6500.i > all.i
python3 "$T/map_data.py" t6500.map base.data
# data objects; m68ki_hot and z80_blk are 8 KB-aligned wrappers whose padding
# already pins their hot part, so they are not moved
/opt/libdragon/bin/mips64-elf-nm -S -n t6500.elf | python3 -c "
import sys
for l in sys.stdin:
    p = l.split()
    if len(p) == 4 and p[2] in 'bBdDrR':
        print(p[0][-8:], int(p[1], 16), 'x' if p[3] in ('m68ki_hot', 'z80_blk') else p[2], p[3])" > base.objs
gcc -O2 -o icolor "$T/icolor.c" -lm; gcc -O2 -o dcolor "$T/dcolor.c" -lm
./icolor t6500.secs all.i 30000 1 i.colors > icolor.out 2> icolor.err &
./dcolor base.objs 4000 1 d.colors tr3000.d tr6500.d > dcolor.out 2> dcolor.err
wait
cat icolor.out dcolor.out
python3 "$T/realize2.py" --text t6500.secs i.colors --data base.objs base.data d.colors "$T/$ROMNAME.ld"
echo "wrote $T/$ROMNAME.ld"
