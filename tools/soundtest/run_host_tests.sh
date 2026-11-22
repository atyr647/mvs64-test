#!/usr/bin/env bash
# tools/soundtest/run_host_tests.sh [workdir]
#
# Host-side sound verification against ymfm (the YM2610 core ares and MAME use).
# Builds the real sound code (sound/ym2610_intf.c + CZ80) for the host, runs
# Blazing Star's actual M1 driver and register stimuli, and compares against
# ymfm fed the identical register writes.  Exits non-zero if any check fails.
#
# Needs: gcc, g++, python3 with numpy, the ymfm sources (YMFM_DIR; ares ships
# them under thirdparty/ymfm/src — build ares with the Pak repo's
# tools/build_ares.sh), and the game zip (GAME_ZIP, default ./blazstar.zip).
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd); REPO=$(cd "$HERE/../.." && pwd)
W=${1:-$(mktemp -d)}; mkdir -p "$W"; W=$(cd "$W" && pwd)
YMFM=${YMFM_DIR:-/opt/pak-ares/src/ares/thirdparty/ymfm/src}
GAME=${GAME_ZIP:-$REPO/blazstar.zip}
PY=${PYTHON:-}
if [ -z "$PY" ]; then for p in python3 python3.13 python3.12; do
  command -v $p >/dev/null 2>&1 && $p -c 'import numpy' 2>/dev/null && { PY=$p; break; }; done; fi
[ -n "$PY" ] || { echo "SKIP: no python with numpy"; exit 0; }
[ -f "$YMFM/ymfm_opn.cpp" ] || { echo "SKIP: ymfm sources not found (set YMFM_DIR)"; exit 0; }
[ -f "$GAME" ] || { echo "SKIP: $GAME not found (set GAME_ZIP)"; exit 0; }
echo "work dir: $W"

# ROM data from the game zip: M1 (Z80 driver) and V1..Vn concatenated (ADPCM samples)
$PY -I - "$GAME" "$W" <<'PYEOF'
import sys, zipfile, re
z = zipfile.ZipFile(sys.argv[1]); w = sys.argv[2]
names = z.namelist()
m1 = [n for n in names if re.search(r'm1', n.lower())][0]
open(w + '/m1.bin', 'wb').write(z.read(m1))
vs = sorted((n for n in names if re.search(r'\.v\d$|-v\d\.', n.lower())), key=lambda n: int(re.findall(r'v(\d)', n.lower())[-1]))
open(w + '/vrom.bin', 'wb').write(b''.join(z.read(n) for n in vs))
PYEOF
# the RSP's exp table, so the C model uses exactly what the N64 build uploads
$PY -I - "$REPO/sound/rsp_fm.c" > "$W/exp_tab.h" <<'PYEOF'
import sys, re
s = open(sys.argv[1]).read(); a = s.index('opn_power_table[256] = {'); b = s.index('};', a)
v = re.findall(r'\d+', s[a + len('opn_power_table[256] = {'):b]); assert len(v) == 256
print('static const uint16_t m_exp[256]={' + ','.join(v) + '};')
PYEOF

CF=(-O2 -w -DCPU_IS_LE=1 -I"$REPO/sound" -I"$REPO/cpu/cz80" -I"$REPO/pico" -I"$W" "-DYM_SRC=\"$REPO/sound/ym2610_intf.c\"")
CZ80="$REPO/cpu/cz80/cz80.c"
echo "building host tools..."
g++ -O2 -std=c++17 -w -I"$YMFM" "$HERE/ymfm_play.cpp" "$YMFM"/ymfm_opn.cpp "$YMFM"/ymfm_adpcm.cpp "$YMFM"/ymfm_ssg.cpp "$YMFM"/ymfm_misc.cpp -o "$W/ymfm_play"
for t in trace trace_st ssgstim; do gcc "${CF[@]}" "$HERE/$t.c" "$CZ80" -o "$W/$t" -lm; done
for t in fmstim fmmodel; do gcc "${CF[@]}" "$HERE/$t.c" "$CZ80" -o "$W/$t" -lm; done
gcc "${CF[@]}" "$HERE/equiv.c" "$CZ80" -o "$W/equiv"
gcc "${CF[@]}" "$HERE/timers.c" "$CZ80" -o "$W/timers"
g++ -O2 -std=c++17 -w -I"$YMFM" "$HERE/ymfm_timers.cpp" "$YMFM"/ymfm_opn.cpp "$YMFM"/ymfm_adpcm.cpp "$YMFM"/ymfm_ssg.cpp "$YMFM"/ymfm_misc.cpp -o "$W/ymfm_timers"
gcc "${CF[@]}" -DZ80_NO_IDLE_SKIP "$HERE/equiv.c" "$CZ80" -o "$W/equiv_noidle"

cd "$W"; M=m1.bin; V=vrom.bin; FAIL=0
echo; echo "== Z80 idle-loop skipping must not change the output"
./equiv $M $V out_idle.pcm && ./equiv_noidle $M $V out_noidle.pcm
if cmp -s out_idle.pcm out_noidle.pcm; then echo "PASS  idle skip: bit-identical over $(( $(stat -c %s out_idle.pcm) / 4 )) samples"; else echo "FAIL  idle skip changes the output"; FAIL=$((FAIL+1)); fi

echo; echo "== Timer A/B periods vs ymfm (they set the music tempo)"
if diff <(./timers) <(./ymfm_timers) >/dev/null; then echo "PASS  timer periods identical to ymfm for TA 0/1/512/900/1023, TB 0/128/192/255"
else echo "FAIL  timer periods differ from ymfm:"; diff <(./timers) <(./ymfm_timers) | head; FAIL=$((FAIL+1)); fi

echo; echo "== ADPCM-A/B with the real driver vs ymfm"
./trace $M $V 0 regs.txt ours_ab.pcm 2>/dev/null
./ymfm_play $V regs.txt ref_ab.pcm /dev/null $((3000*368)) 0
PYTHONPATH="$HERE" $PY "$HERE/check_adpcm_ssg_game.py" "$W" || FAIL=$((FAIL+$?))

echo; echo "== SSG register stimulus vs ymfm"
./ssgstim
./ymfm_play $V ssg_regs.txt /dev/null ref_ssgstim.pcm $(( $(stat -c %s ours_ssgstim.pcm) / 2 )) 0
PYTHONPATH="$HERE" $PY "$HERE/check_ssg_stim.py" "$W" || FAIL=$((FAIL+$?))

echo; echo "== FM note stimuli: C model of rsp_fm.S vs ymfm"
./fmstim 55555.5555 fm_regs.txt model55_stim.pcm fm_labels.txt
./ymfm_play $V fm_regs.txt ref_fm_l.pcm /dev/null $(( $(stat -c %s model55_stim.pcm) / 2 * 22050 / 55555 + 2000 )) 36
PYTHONPATH="$HERE" $PY "$HERE/check_fm_stim.py" "$W" || FAIL=$((FAIL+$?))

echo; echo "== Blazing Star FM jingle (cmd \$5F): RSP model vs ymfm"
./trace_st $M $V 0 regs_st.txt /dev/null 2>/dev/null
./ymfm_play $V regs_st.txt ref_st_l.pcm ref_st_ssg.pcm $((500*368)) 36
./fmmodel $M $V 22050 model22.pcm && ./fmmodel $M $V 55555.5555 model55.pcm
PYTHONPATH="$HERE" $PY "$HERE/check_fm_jingle.py" "$W" || FAIL=$((FAIL+$?))

echo; [ $FAIL -eq 0 ] && echo "ALL HOST SOUND CHECKS PASSED" || echo "$FAIL CHECK(S) FAILED"
exit $FAIL
