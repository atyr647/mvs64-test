#!/bin/sh
# Real FPS over fixed ranges of emulated frames (same game content in every
# build, since AUTOPLAY input is frame-based), from the per-second FPS lines
# of a production build's ares log.   usage: fps_by_frames.sh <log> [ranges]
grep "^FPS" "$1" | awk -v r="${2:-2400 6000 9000 12000}" '{f+=$2; t++; F[t]=f} END{
  n = split(r, m, " ");
  for (k = 1; k < n; k++) { a = m[k]; b = m[k+1]; ta = 0; tb = 0
    for (i = 1; i <= t; i++) { if (!ta && F[i] >= a) ta = i; if (!tb && F[i] >= b) tb = i }
    if (ta && tb) printf "frames %d-%d: %.1f FPS\n", a, b, (b - a) / (tb - ta)
    else printf "frames %d-%d: not reached\n", a, b }
}'
