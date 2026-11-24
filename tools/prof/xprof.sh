#!/bin/sh
# Average the [XPROF]/[PROF68K] lines of a -DPROFILER run over seconds [$2,$3)
a=${2:-70}; b=${3:-180}
for k in all 68k sound draw; do grep "^\[XPROF\] $k:" "$1" | sed -n "${a},${b}p" | sed 's/[a-z_/]*=//g' | awk -v k=$k '{c+=$3;i+=$4;d+=$5;n++} END{if(n) printf "%-6s %8.0f cycles/frame (%5.1f%% of 60Hz)  icache misses %6.0f (%4.1f%% of cycles at 48)  dcache misses %6.0f (%4.1f%% at 40)\n", k, c/n, c/n/15625, i/n, 4800*i/c, d/n, 4000*d/c}'; done
grep "PROF68K" "$1" | sed -n "${a},${b}p" | sed 's/.*insn=\([0-9]*\) hle_calls=\([0-9]*\) skipped_cycles=\([0-9]*\).*/\1 \2 \3/' | awk '{i+=$1;h+=$2;s+=$3;n++} END{if(n) printf "68K per frame: %.0f interpreted insns, %.0f HLE calls, %.0f%% of cycles idle-skipped\n", i/n,h/n,s/n/2000}'
