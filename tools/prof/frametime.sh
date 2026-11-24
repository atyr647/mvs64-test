#!/bin/sh
# Mean frame cost (% of a 60 Hz frame) over emulated frames [$2,$3) of an ares log: comparable across builds
# because AUTOPLAY input is frame-based.
grep "^\[PROFILE\]" "$1" | awk -v a="${2:-1500}" -v b="${3:-3000}" -F'[:% ]+' 'NR>=a && NR<b {c+=$3; d+=$7; n++} END{ if(n) printf "frames %d: mean frame cost %.1f%% of 60Hz (= %.1f FPS), draw %.1f%%\n", n, c/n, 6000/(c/n), d/n; else print "not enough frames"}'
