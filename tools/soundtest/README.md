# Sound tests

Verification for the YM2610/Z80 sound emulation (`sound/ym2610_intf.c`,
`sound/rsp_fm.c`, `rsp_fm.S`). Two suites:

| Script | Runs on | What it proves |
|---|---|---|
| `run_host_tests.sh [workdir]` | Linux host, ~1 min | The sound code matches **ymfm** (the YM2610 core ares and MAME use) on identical register writes |
| `run_ares_tests.sh [workdir]` | ares (headless), ~15 min | The real N64 build, including the RSP microcode and libdragon audio path, plays correctly, without dropouts, through real gameplay |

Both exit non-zero when a check fails. Run the host suite first with the same
workdir, because the ares jingle check reuses its ymfm reference.

## Host suite (`run_host_tests.sh`)

The real `sound/ym2610_intf.c` and CZ80 are compiled for the host. Blazing
Star's actual M1 sound driver runs on them, and every YM2610 register write is
recorded and replayed into ymfm.

| Check | Method | Pass criteria |
|---|---|---|
| Z80 idle-loop skipping | 40 s of driver output with and without `Z80_NO_IDLE_SKIP` | bit-identical |
| ADPCM-A + ADPCM-B | real driver command stream (effects, BGM) vs ymfm | median correlation > 0.99; level 0.500 (our deliberate ×0.5) |
| SSG | register stimulus: tone at volumes 15→1, noise, 3 tones, envelope shapes | correlation > 0.90 per segment (> 0.70 for 3 tones, whose phases differ); level = MAME's 0.84/0.98 × 0.5 |
| FM operators | single notes (pure sine, 1 modulator, 4-op cascade, feedback 6, algorithm 4) through a C model of `rsp_fm.S` at 55.5 kHz | level within 15%; ≥3 of the 5 strongest partials identical |
| FM pan | algorithm 4 note panned left only | right channel silent in both; left level within 25% |
| FM detune | DT ±3 with MUL 5 at 1760 Hz (detune is added before the multiplier, as on the chip) | pitch within 1 Hz of ymfm (the old order is off by 4 Hz) |
| FM envelopes | attack, decay, sustain, release and key scaling | envelope difference: median < 1.5 dB, 90th percentile < 4 dB |
| FM in-game | the driver's FM jingle (command `$5F`) through the RSP model at 22050 and 55555 Hz, both channels | level within 15%, log-spectrum correlation > 0.75, band energy shares within 8 points |
| Timers | Timer A/B periods for several register values | identical to ymfm |

`rspfm_model.h` (used by `fmmodel.c`, `fmstim.c` and `egstim.c`) is the C model of the RSP routine. It mirrors
`rsp_fm.S` operation by operation and uses the exact tables the N64 build
uploads. The ares suite checks that the model and the real RSP agree.

Needs gcc, g++, python3 with numpy, `YMFM_DIR` (default: the ymfm sources in
an ares tree built by the Pak repo's `tools/build_ares.sh`), and `GAME_ZIP`
(default `./blazstar.zip`).

## ares suite (`run_ares_tests.sh`)

Builds two ROM variants with `make -f Makefile.mvs64 SND_CFLAGS=...`:

* `-DSND_SELFTEST` ignores the 68K. It replays the game's startup sound
  commands, the FM jingle, ADPCM-A effects and the ADPCM-B track at fixed Z80
  cycle times (so the timeline matches the host harness at any FPS), then streams
  4 s of output after `$5F` to the debug log as `[WAV]` hex. The jingle is
  compared with ymfm, whose SSG part is DC-blocked first like our output.
  `[ST]` lines log when each command and the capture happened.
* `-DAUTOPLAY` drives the controller from a script: coin, start, autofire,
  weaving and bombs. In about 6 minutes it plays Stage 1 to the boss. SDL's
  `disk` audio driver records exactly what ares plays.

Checks: no crashes/asserts, gameplay commands reached, no mixer clipping, no
20–100 ms holes in the real playback (longer silences are musical rests),
no clipping at the output. It also prints FPS, AI underruns and the sound CPU
cost split.

ares must have the machine to itself: the playback is real time, so other
heavy jobs (e.g. the host suite) cause host-side underruns that the gap check
reports as dropouts.

Env: `BIOS` (NeoGeo BIOS; `sfix.sfix` beside it), `N64_INST`, `ARES`
(default `/opt/pak-ares/bin/ares`), `GAME_ZIP`, `AUTOPLAY_SECONDS`.

## Other switches

* `SND_CFLAGS=-DSND_DISABLE` turns the sound CPU off, for A/B performance runs.
* `SND_CFLAGS=-DSOUND_DEBUG_MODE=3` plays a hardcoded FM tone through the RSP.
* The `[SND]` debug lines (once per second) report command/NMI counts, register
  writes, per-source peaks and clipping, the AI underrun count, and the sound
  CPU cost split into Z80, RSP FM and mixing.
