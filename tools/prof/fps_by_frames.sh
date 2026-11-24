#!/bin/sh
# Real FPS over fixed ranges of emulated frames (same game content in every
# build, since AUTOPLAY input is frame-based), from the per-second FPS lines
# of a production build's ares log (the time a frame count is reached is
# interpolated within its second), and the rate of frames actually drawn
# (adaptive frameskip) when the log has it.   usage: fps_by_frames.sh <log> [ranges]
grep "^FPS" "$1" | awk -v r="${2:-2400 6000 9000 12000}" '{t++; F[t] = F[t-1] + $2; D[t] = D[t-1] + ($3 == "drawn" ? $4 : 0)} END{
  n = split(r, m, " ");
  for (k = 1; k < n; k++) { a = m[k]; b = m[k+1]; ta = -1; tb = -1
    for (i = 1; i <= t; i++) {
      if (ta < 0 && F[i] >= a) ta = (i - 1) + (a - F[i-1]) / (F[i] - F[i-1])
      if (tb < 0 && F[i] >= b) tb = (i - 1) + (b - F[i-1]) / (F[i] - F[i-1]) }
    if (ta >= 0 && tb >= 0) {
      ia = int(ta); ib = int(tb); da = D[ia] + (ta - ia) * (D[ia+1] - D[ia]); db = D[ib] + (tb - ib) * (D[ib+1] - D[ib])
      printf "frames %d-%d: %.2f FPS", a, b, (b - a) / (tb - ta)
      if (db > da) printf ", drawn %.2f/s", (db - da) / (tb - ta)
      printf "\n" }
    else printf "frames %d-%d: not reached\n", a, b }
}'
