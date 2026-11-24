# Profiling tools (ares)

Measure where the N64's time goes while Blazing Star plays itself
(`-DAUTOPLAY`: scripted input, so every build sees the same game frames).

Build with `SND_CFLAGS="-DAUTOPLAY -DPROFILER [extra]"` and run ares with
`--setting Developer/HomebrewMode=true`: the profiler reads ares's own cycle
and cache-miss counters (the "emux" `XPROFREAD` pseudo-instruction), so the
numbers are exact, not sampled. `-DPROFILER` builds only run in ares.

| Flag | Output | Read with |
|---|---|---|
| `-DPROFILER` | `[XSUM]`: clocks, I-cache and D-cache misses per emulated frame for 68K / sound / draw, over frames 1500–3000 | `xsum.sh <log>` |
| `-DPROFILER_PCS` | `[PROFPC]`: interpreted 68K instructions per 32-byte block of game code | `pcs.py <log>` |
| `-DPROFILER_OPS` | `[PROFOP]`: executions per interpreter handler | `ops.py <log> <elf>` (`--list N` regenerates `m68kops_hot.txt`) |
| `-DPROFILER_DISP` | `[XDISP]`: 68K cost split into dispatch (fetch + table) and handler bodies | grep |
| `-DPROFILER_STATE` | `[STATE]`: WORK RAM checksum every 500 frames. Equal between two builds = same emulation result | diff |
| `-DPROFILER_SAMPLE` | `[PROFN]`/`[PROFM]`: 2.3 kHz timer samples of the N64 PC and 68K PC | `symbolize.py <log> <elf>` |

The extra flags perturb the caches (their tables are several KB), so take the
`[XSUM]` numbers from a plain `-DPROFILER` build. A miss costs 96 clocks
(I-cache) or 80 (D-cache) in ares; one emulated frame at 60 Hz is 1,562,500.

`[XSUM]` counts the CPU's own work (68K, sound, draw). It does not count
time spent waiting for the display, the RDP or the RSP outside the sound
code, so a change that shifts work onto those can look faster there and
still lower the frame rate: always confirm with the real FPS of a
production build (`grep ^FPS`, mean from 40 s on). Moving the audio to the
RSP high-priority queue measured 1.5% better in `[XSUM]` and 33% worse in
FPS.

`frametime.sh <log> [from to]` gives the mean frame cost from the `[PROFILE]`
lines of any N64 build. `dasm68k.c` disassembles a range of the P-ROM
(`gcc -I<stub dir> dasm68k.c m68kdasm.c`).

## Workflow for a new game or scene

1. `-DPROFILER_PCS` run, `pcs.py`: which 68K code is still interpreted.
2. Disassemble the top regions, find the function entry points, add them to
   `HLE_PCS` in `Makefile.mvs64` (keep unrolled straight-line code out: native
   code that runs once per call only trades D-cache for I-cache misses).
3. `-DPROFILER_STATE` run with and without: the checksums must match (a frame
   may differ when the 68K is caught mid-frame, but it must re-converge).
4. Plain `-DPROFILER` run, `xsum.sh`: confirm the gain.
