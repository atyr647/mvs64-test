#!/bin/sh
# Real FPS over fixed ranges of emulated frames (same game content in every
# build, since AUTOPLAY input is frame-based), from the per-second FPS lines
# of a production build's ares log (the time a frame count is reached is
# interpolated within its second).   usage: fps_by_frames.sh <log> [ranges]
grep "^FPS" "$1" | awk -v r="${2:-2400 6000 9000 12000}" '{t++; F[t] = F[t-1] + $2} END{
  n = split(r, m, " ");
  for (k = 1; k < n; k++) { a = m[k]; b = m[k+1]; ta = -1; tb = -1
    for (i = 1; i <= t; i++) {
      if (ta < 0 && F[i] >= a) ta = (i - 1) + (a - F[i-1]) / (F[i] - F[i-1])
      if (tb < 0 && F[i] >= b) tb = (i - 1) + (b - F[i-1]) / (F[i] - F[i-1]) }
    if (ta >= 0 && tb >= 0) printf "frames %d-%d: %.2f FPS\n", a, b, (b - a) / (tb - ta)
    else printf "frames %d-%d: not reached\n", a, b }
}'
