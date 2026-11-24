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
recorded, stamped with its exact Z80 time, and replayed into ymfm. (Stamping
writes per frame would merge the driver's key-off/key-on pairs, which are
about 100 µs apart, and make ymfm skip note retriggers that a real chip does.)

| Check | Method | Pass criteria |
|---|---|---|
| Z80 idle-loop skipping | 40 s of driver output with and without `Z80_NO_IDLE_SKIP` | bit-identical audio, and every register write at the identical Z80 cycle |
| ADPCM-A + ADPCM-B | real driver command stream (effects, BGM) vs ymfm | median correlation > 0.995, every window > 0.9; level 0.500 (our deliberate ×0.5) |
| SSG | register stimulus: tone at volumes 15→1, noise, 3 tones, envelope shapes | correlation > 0.90 per segment (> 0.70 for 3 tones, whose phases differ); level = MAME's 0.84/0.98 × 0.5 |
| FM operators | single notes (pure sine, 1 modulator, 4-op cascade, feedback 6, algorithm 4) through a C model of `rsp_fm.S` at 55.5 kHz | level within 6%; ≥3 of the 5 strongest partials identical |
| FM pan | algorithm 4 note panned left only | right channel silent in both; left level within 6% |
| FM detune | DT ±3 with MUL 5 at 1760 Hz (detune is added before the multiplier, as on the chip) | pitch within 1 Hz of ymfm (the old order is off by 4 Hz); level within 5% |
| FM sustain at SL 0 | fastest decay with sustain level 0: the slot must go straight to sustain | level within 2% (taking a decay step first was 0.75 dB quiet) |
| FM envelopes | attack, decay, sustain, release and key scaling | envelope difference: median < 1.5 dB, 90th percentile < 4 dB |
| FM in-game | the driver's FM jingle (command `$5F`) through the RSP model at 22050 and 55555 Hz, both channels | level within 5%, log-spectrum correlation > 0.95 (> 0.9 at 22050 Hz), band energy shares within 4 points |
| Timers | Timer A/B periods for several register values | identical to ymfm |

`rspfm_model.h` (used by `fmmodel.c`, `fmstim.c` and `egstim.c`) is the C model of the RSP routine. It mirrors
`rsp_fm.S` operation by operation and uses the exact tables the N64 build
uploads, including the 2× oversampling (two operator steps summed per output
sample), and it renders in place on the CPU's `fm_state` as the RSP's state
DMA does. `fmmodel.c` applies queued register writes at the same samples as
`sound_render`. `FMMASK=02|04|10|20 ./fmmodel ...` renders a single channel
(ymfm_play takes the same mask) for debugging. The ares suite checks that the
model and the real RSP agree.

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
dropouts, no clipping at the output. A dropout is a 20–100 ms hole that cuts
the sound off; a musical rest decays into silence first and is not counted.
Holes that end on ares's own 10 ms output chunks mean ares fell behind real
time (not the ROM) and are reported as INFO. The N64-side AI underrun count
is reported separately for boot and gameplay. It also prints FPS and the
sound CPU cost split.

ares must have the machine to itself: the playback is real time, so other
heavy jobs (e.g. the host suite) cause host-side underruns that the gap check
reports as dropouts.

Env: `BIOS` (NeoGeo BIOS; `sfix.sfix` beside it), `N64_INST`, `ARES`
(default `/opt/pak-ares/bin/ares`), `GAME_ZIP`, `AUTOPLAY_SECONDS`.

## ADPCM-A on the RSP

ADPCM-A is decoded on the RSP (`cmd_adpcma_render` in `rsp_fm.S`). The CPU
stages each chunk's bytes and end handling (`adpcma_stage`), and
`adpcma_model` in `ym2610_intf.c` is a C model of the RSP routine. Host builds
run the model, so the host suite checks it against ymfm and the old
decoder's output. `SND_CFLAGS=-DADPCMA_VERIFY` runs the model next to the RSP
on the N64 and logs `[AVERIFY] <chunks>, <n> mismatched` (outputs and state
must match bit for bit; 0 mismatches over 40,960 gameplay chunks in ares).

## Other switches

* `SND_CFLAGS=-DSND_DISABLE` turns the sound CPU off, for A/B performance runs.
* `SND_CFLAGS=-DSOUND_DEBUG_MODE=3` plays a hardcoded FM tone through the RSP.
* The `[SND]` debug lines (once per second) report command/NMI counts, register
  writes, per-source peaks and clipping, the AI underrun count, write-queue
  overflows (`ymq_full`, should stay 0), and the sound CPU cost: Z80, time
  still spent waiting for the RSP (FM renders while the CPU mixes), and mixing
  split into SSG, ADPCM-A, ADPCM-B, the FM envelope and the rest.
