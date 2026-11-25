# Cache-coloured code and data layout

Ported from picodrive64 (`platform/n64/tools/layout`, where it took a 68K
interpreter from 22.96 to 17.64 ms/frame). The VR4300 has a 16 KB
direct-mapped I-cache (32-byte lines) and an 8 KB direct-mapped D-cache
(16-byte lines): two pieces of code or data whose addresses are congruent
mod the cache size evict each other, however far apart they are. MVS64 was
dominated by such misses (handlers, AOT functions, memory helpers, the Z80,
sound and the renderer fighting over the same lines), and any change in code
size reshuffled them by up to +-3% FPS.

The method: record an exact trace of every instruction-fetch run and every
data access over real gameplay (a patched ares), then simulated-anneal the
position ("colour": address mod cache size) of every hot code section and
data object to minimise simulated misses, and pin each one there in the
linker script with a padding statement (`. += (colour - .) & mask;`), so a
placement does not depend on the sizes of the sections before it. Cold
sections fill the gaps.

Blazing Star, ares, mean of three code layouts (`-DCODE_PAD` 0/576/1600),
frames 2400-4000 / 4000-6000 / 6000-9000:

    default layout             52.8 / 50.0 / 46.6 FPS
    tools/layout/blazstar.ld   58.3 / 56.0 / 52.7 FPS   (+10% / +12% / +13%)

and the three layouts now agree within 0.1 FPS (they spread +-3% before).
Simulated on the traces: I-cache misses -69%, D-cache misses + writebacks -11%.

## Use

The build uses `tools/layout/<rom>.ld` when it exists (with `LTO_SECTIONS=1`,
one section per function and data object, also in LTO's link-time code
generation). `CACHE_LAYOUT=` (empty) builds the plain layout;
`CACHE_LAYOUT=<file>` picks another. `apply_layout.py` merges the placement
into `mvs64.ld` (binutils 2.42 has no `--section-ordering-file`).

## Regenerating

A layout degrades gracefully as code changes: sections keep their colour as
long as their names match, and renamed ones (LTO `.isra`/`.lto_priv`
suffixes can change) fall back to default placement. Regenerate after
changing hot code, or for another game:

    ARES=/opt/ares-trace/build/desktop-ui/ares BIOS=... tools/layout/regen.sh /tmp/layout-work [game.zip]

`ARES` must be built with `ares-cachetrace.patch` (against ares `af4cbb0`,
the Pak repo's pin; a separate build from the miss-profiling one in
tools/prof, whose patch conflicts with it). It adds emux XPROF codes 8/9,
which `-DCACHE_TRACE` issues around `TRACE_FRAMES` (30) frames from
`TRACE_START`, running the interpreter so every access goes through the
cache model. Traces are ~18 MB (.i) and ~28 MB (.d) per window.

Check a new link against the traces with `icolor SECS TRACE.i 0 0 - NEW.secs`
and `dcolor OBJS -1 0 NEW.objs TRACE.d...` (see regen.sh for the file
formats).

The heap and the stack are not movable; the D-cache model treats them as
fixed. `m68ki_hot` and `z80_blk` are 8 KB-aligned wrappers that pin their
hot fields by padding, so regen.sh keeps them out of the annealing.
