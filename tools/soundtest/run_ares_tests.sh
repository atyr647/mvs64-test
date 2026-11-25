#!/usr/bin/env bash
# tools/soundtest/run_ares_tests.sh [workdir]
#
# End-to-end sound checks in ares (headless: Xvfb + Mesa software rendering).
#   1. SND_SELFTEST ROM: replays Blazing Star's startup sound commands and the
#      FM jingle ($5F); the jingle as rendered by the real RSP code is compared
#      with ymfm (needs a prior run_host_tests.sh in the same workdir).
#   2. AUTOPLAY ROM: scripted controller plays Stage 1 for ~6 minutes while the
#      SDL "disk" audio driver records exactly what ares plays.  Checks: no
#      crashes, no dropouts, no clipping, AI underruns, sound CPU cost, FPS.
#
# Env: BIOS (NeoGeo BIOS file, sfix.sfix next to it), GAME_ZIP (default
# ./blazstar.zip), ARES (default /opt/pak-ares/bin/ares, from the Pak repo's
# tools/build_ares.sh), N64_INST (libdragon toolchain), AUTOPLAY_SECONDS (380).
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd); REPO=$(cd "$HERE/../.." && pwd)
W=${1:-$(mktemp -d)}; mkdir -p "$W"; W=$(cd "$W" && pwd)
ARES=${ARES:-/opt/pak-ares/bin/ares}; GAME=${GAME_ZIP:-$REPO/blazstar.zip}; SECS=${AUTOPLAY_SECONDS:-380}
: "${BIOS:?set BIOS to a NeoGeo BIOS file (with sfix.sfix in the same directory)}"
: "${N64_INST:?set N64_INST to the libdragon toolchain}"
export PATH="$N64_INST/bin:$PATH"
PY=${PYTHON:-}
if [ -z "$PY" ]; then for p in python3 python3.13 python3.12; do
  command -v $p >/dev/null 2>&1 && $p -c 'import numpy' 2>/dev/null && { PY=$p; break; }; done; fi
for t in "$ARES" Xvfb; do command -v "$t" >/dev/null 2>&1 || [ -x "$t" ] || { echo "SKIP: $t not found"; exit 0; }; done

DISP=:97; rm -f /tmp/.X97-lock; Xvfb $DISP -screen 0 1280x960x24 -ac -nolisten tcp >"$W/xvfb.log" 2>&1 & XPID=$!
trap 'kill $XPID 2>/dev/null || true' EXIT; sleep 2

build() {   # $1 = SND_CFLAGS, $2 = output rom
  ( cd "$REPO" && touch platform_n64.c emu.c sound/ym2610_intf.c sound/rsp_fm.c &&
    make -f Makefile.mvs64 USE_HLE=1 SND_CFLAGS="$1" BIOS="$BIOS" ROM="$GAME" -j"$(nproc)" >"$W/build_$(basename "$2").log" 2>&1 &&
    cp mvs64-$(basename "${GAME%.zip}").z64 "$2" )
}
ares_run() {   # $1 = rom, $2 = seconds, $3 = log, [$4 = raw audio capture]
  local audio=(SDL_AUDIO_DRIVER=dummy) set=(--setting Audio/Mute=true)
  if [ -n "${4:-}" ]; then audio=(SDL_AUDIO_DRIVER=disk SDL_AUDIO_DISK_OUTPUT_FILE="$4"); set=(--setting Audio/Mute=false --setting Audio/Frequency=48000 --setting Audio/Blocking=false --setting Audio/Volume=1.0); fi
  env DISPLAY=$DISP LIBGL_ALWAYS_SOFTWARE=1 "${audio[@]}" timeout "$2" "$ARES" --system "Nintendo 64" --no-file-prompt --fullscreen "${set[@]}" "$1" >"$3" 2>&1 || true
}
FAIL=0
echo "== building ROMs"; build "-DSND_SELFTEST -DSND_STATS" "$W/selftest.z64"; build "-DAUTOPLAY -DSND_STATS" "$W/autoplay.z64"

echo "== self-test ROM: FM jingle as rendered by the RSP"
ares_run "$W/selftest.z64" 150 "$W/ares_selftest.log"
if [ -f "$W/ref_st_l.pcm" ] && [ -n "$PY" ]; then
  PYTHONPATH="$HERE" $PY "$HERE/check_ares_jingle.py" "$W/ares_selftest.log" "$W" || FAIL=$((FAIL+$?))
else echo "SKIP  jingle comparison (run run_host_tests.sh with the same workdir first)"; fi

echo "== autoplay: Stage 1 for ${SECS}s with real playback captured"
ares_run "$W/autoplay.z64" "$SECS" "$W/ares_autoplay.log" "$W/autoplay_audio.raw"
L="$W/ares_autoplay.log"
n=$(grep -cE 'ASSERT|EXCEPTION' "$L" || true); [ "$n" -eq 0 ] && echo "PASS  no crashes/asserts" || { echo "FAIL  $n crash/assert lines"; FAIL=$((FAIL+1)); }
grep -E "^\[SND\] cmds" "$L" | grep -qE " c0| f5| 18 " && echo "PASS  gameplay sound commands seen (c0/f5/18 xx)" || { echo "FAIL  game never reached gameplay"; FAIL=$((FAIL+1)); }
und=$(grep -E "^\[SND\] cost" "$L" | tail -1 | sed 's/.*underruns=\([0-9]*\).*/\1/'); ub=$(grep -E "^\[SND\] cost" "$L" | sed -n 10p | sed 's/.*underruns=\([0-9]*\).*/\1/')
echo "INFO  N64 AI underruns: ${ub:-?} in the first 10 s (boot frames are slow in ares), $(( ${und:-0} - ${ub:-0} )) after"
clip=$(grep -E "^\[SND\] peaks" "$L" | awk -F'clipped=' '{s+=$2} END{print s+0}'); [ "$clip" -eq 0 ] && echo "PASS  no clipped samples in the mixer" || { echo "FAIL  $clip clipped samples"; FAIL=$((FAIL+1)); }
grep -E "^FPS" "$L" | awk 'NR>10{s+=$2;n++} END{if(n) printf "INFO  mean FPS after boot: %.1f\n", s/n}'
grep -E "^\[SND\] cost" "$L" | awk -F'[=%| ]+' 'NR>40{z+=$4;r+=$6;m+=$8;n++} END{if(n) printf "INFO  sound CPU cost: %.1f%% of real time (Z80 %.1f, RSP FM %.1f, mix %.1f)\n",(z+r+m)/n,z/n,r/n,m/n}'
[ -n "$PY" ] && { PYTHONPATH="$HERE" $PY "$HERE/check_ares_playback.py" "$W/autoplay_audio.raw" 40 || FAIL=$((FAIL+$?)); }

echo; [ $FAIL -eq 0 ] && echo "ALL ARES SOUND CHECKS PASSED" || echo "$FAIL CHECK(S) FAILED"
exit $FAIL
