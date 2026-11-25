#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <inttypes.h>
#include "emu.h"
#include "sound/ym2610_intf.h"
#ifdef USE_FAME
#include "fame_adapter.h"
#else
#include "m68k.h"
#endif
#include "hw.h"
#include "video.h"
#include "roms.h"
#include "platform.h"

static int cpu_trace_count = 0;
void cpu_trace(unsigned int pc) {
	(void)cpu_trace_count;
	#ifndef N64
	if (cpu_trace_count == 0) {
		m68k_set_instr_hook_callback(NULL);
		return;
	}

	char inst[1024];
	m68k_disassemble(inst, pc, M68K_CPU_TYPE_68000);
	debugf("trace: %06x %-30s", pc, inst);

	if (strstr(inst, "A0")) debugf("A0=%08x ", m68k_get_reg(NULL, M68K_REG_A0));
	if (strstr(inst, "A1")) debugf("A1=%08x ", m68k_get_reg(NULL, M68K_REG_A1));
	if (strstr(inst, "A2")) debugf("A2=%08x ", m68k_get_reg(NULL, M68K_REG_A2));
	if (strstr(inst, "A3")) debugf("A3=%08x ", m68k_get_reg(NULL, M68K_REG_A3));
	if (strstr(inst, "A4")) debugf("A4=%08x ", m68k_get_reg(NULL, M68K_REG_A4));
	if (strstr(inst, "A5")) debugf("A5=%08x ", m68k_get_reg(NULL, M68K_REG_A5));
	if (strstr(inst, "A6")) debugf("A6=%08x ", m68k_get_reg(NULL, M68K_REG_A6));
	if (strstr(inst, "A7")) debugf("A7=%08x ", m68k_get_reg(NULL, M68K_REG_A7));
	if (strstr(inst, "D0")) debugf("D0=%08x ", m68k_get_reg(NULL, M68K_REG_D0));
	if (strstr(inst, "D1")) debugf("D1=%08x ", m68k_get_reg(NULL, M68K_REG_D1));
	if (strstr(inst, "D2")) debugf("D2=%08x ", m68k_get_reg(NULL, M68K_REG_D2));
	if (strstr(inst, "D3")) debugf("D3=%08x ", m68k_get_reg(NULL, M68K_REG_D3));
	if (strstr(inst, "D4")) debugf("D4=%08x ", m68k_get_reg(NULL, M68K_REG_D4));
	if (strstr(inst, "D5")) debugf("D5=%08x ", m68k_get_reg(NULL, M68K_REG_D5));
	if (strstr(inst, "D6")) debugf("D6=%08x ", m68k_get_reg(NULL, M68K_REG_D6));
	if (strstr(inst, "D7")) debugf("D7=%08x ", m68k_get_reg(NULL, M68K_REG_D7));

	debugf("\n");

	cpu_trace_count--;
	#endif
}

void cpu_start_trace(int cnt) {
	#ifndef N64
	m68k_set_instr_hook_callback(cpu_trace);
	#endif
	cpu_trace_count = cnt;
}

static int g_frame;
static uint64_t g_clock, g_clock_framebegin;
static uint64_t m68k_clock;
static EmuEvent events[MAX_EVENTS];
uint32_t profile_hw_io;
uint32_t profile_dma_load;

#ifdef PROFILER
/* ares "emux" counters (needs --setting Developer/HomebrewMode=true): global
 * CPU cycles and cache misses, read with the XPROFREAD pseudo-instruction. */
static inline uint64_t xprof_read(uint64_t code) {
	register int64_t slot asm("t0") = -1;
	register uint64_t v asm("t1") = code;
	asm volatile(".word 0x42000029 | (8 << 20) | (9 << 15)" : "+r"(v) : "r"(slot));
	return v;
}
typedef struct { uint64_t cyc, ic, dc; } xprof_t;
xprof_t xp_68k, xp_snd, xp_draw, xp_all;
static xprof_t xs_68k, xs_snd, xs_draw, xs_all;   /* sums over a fixed emulated-frame range */
static unsigned long xs_insn;
static inline void xp_begin(xprof_t *p) { p->cyc -= xprof_read(0); p->ic -= xprof_read(0x11); p->dc -= xprof_read(0x21); }
static inline void xp_end(xprof_t *p)   { p->cyc += xprof_read(0); p->ic += xprof_read(0x11); p->dc += xprof_read(0x21); }
#define XP_BEGIN(p) xp_begin(&(p))
#define XP_END(p)   xp_end(&(p))
#else
#define XP_BEGIN(p)
#define XP_END(p)
#endif

static uint64_t m68k_exec(uint64_t clock) {
	clock /= M68K_CLOCK_DIV;
	if (clock > m68k_clock) {
		int ncycles = (int)(clock - m68k_clock);
		#ifdef USE_FAME
		int executed = fame_adapter_execute(ncycles);
		#else
		XP_BEGIN(xp_68k);
		int executed = m68k_execute(ncycles);
		XP_END(xp_68k);
		#endif
		m68k_clock += executed;
	}
	return m68k_clock * M68K_CLOCK_DIV;
}


// Return the next event that must be executed
static EmuEvent* next_event() {
    EmuEvent *e = NULL;
    for (int i=0;i<MAX_EVENTS;i++) {
        if (!events[i].cb) continue;
        if (!e || events[i].clock < e->clock) e=&events[i];
    }
    return e;
}

int emu_add_event(int64_t clock, EmuEventCb cb, void *cbarg) {
    for (int i=0;i<MAX_EVENTS;i++) {
        if (events[i].cb) continue;
        events[i].clock = clock;
        events[i].cb = cb;
        events[i].cbarg = cbarg;
        events[i].current = false;
        return i;
    }
    assert(0);
}

void emu_change_event(int event_id, int64_t newclock) {
	events[event_id].clock = newclock;
	if (events[event_id].current) {
		#ifdef USE_FAME
		fame_adapter_end_timeslice();
		#else
		m68k_end_timeslice();
		#endif
	}
}

int64_t emu_clock(void) {
	#ifdef USE_FAME
	return g_clock + fame_adapter_cycles_run() * M68K_CLOCK_DIV;
	#else
	return g_clock + m68k_cycles_run() * M68K_CLOCK_DIV;
	#endif
}

int64_t emu_clock_frame(void) {
	return emu_clock() - g_clock_framebegin;
}

void emu_cpu_reset(void) {
	#ifdef USE_FAME
	fame_adapter_reset();
	#else
	m68k_pulse_reset();
	#endif
}

uint32_t emu_pc(void) {
	#ifdef USE_FAME
	return fame_adapter_get_pc();
	#else
	return m68k_get_reg(NULL, M68K_REG_PC) & 0xFFFFFF;
	#endif
}

void emu_cpu_irq(int irq, bool on) {
	#ifdef USE_FAME
	fame_adapter_set_virq(irq, on);
	#else
	m68k_set_virq(irq, on);
	#endif
}


uint32_t emu_vblank_start(void* arg) {
	emu_cpu_irq(1, true);
	hw_vblank();
	return FRAME_CLOCK;
}

uint32_t render_time;
static uint32_t g_drawn;   /* frames actually drawn (adaptive frameskip skips the rest) */

uint32_t emu_render(void *arg) {

	#ifdef FB_CRC
	/* -DFB_CRC: draw every 500th frame whatever the frameskip, and log a CRC
	 * of the finished framebuffer, to check that a renderer change leaves
	 * the picture identical (the game state does not depend on rendering) */
	const int fb_force = (g_frame % 500 == 0);
	#else
	const int fb_force = 0;
	#endif

	#ifdef N64
	if (CONFIG_FRAMESKIP_MODE == 2 && !fb_force) {
		extern volatile int N64_FRAME;

		/* Adaptive frameskip: baseline 1-in-3 (like fixed mode), but
		 * render extra frames when the CPU is keeping up with real-time.
		 * Never skip MORE than 2 consecutive frames — heavy scenes stay
		 * at a steady 20 FPS floor rather than dropping to 10-12 FPS. */
		static int skip_count = 0;

		if (skip_count < 2) {
			/* Baseline: always skip at least 2 frames (render 1 in 3).
			 * But if we're ahead of real-time, render early. */
			if (skip_count >= 1 && N64_FRAME <= g_frame) {
				/* Ahead of real-time: render this frame (1 in 2) */
				skip_count = 0;
			} else {
				skip_count++;
				return FRAME_CLOCK;
			}
		} else {
			skip_count = 0;
		}

		/* Resync if we've fallen behind */
		if (N64_FRAME > g_frame + 1) {
			disable_interrupts();
			N64_FRAME = g_frame;
			enable_interrupts();
		}
	}
	#endif

	if (CONFIG_FRAMESKIP_MODE == 1) {
		if (g_frame % 3 != 0) {
			;
			return FRAME_CLOCK;
		}
	}

	#ifdef N64
	uint32_t t0 = TICKS_READ();
	#endif
	g_drawn++;
	XP_BEGIN(xp_draw);
	plat_beginframe();
	video_render();
	plat_endframe();
	XP_END(xp_draw);
	#if defined(FB_CRC) && defined(N64)
	if (fb_force) {
		extern uint8_t *g_screen_ptr;
		rdpq_fence(); rspq_wait();
		const uint16_t *p = (const uint16_t *)UncachedAddr(g_screen_ptr);
		uint32_t crc = 2166136261u;
		for (int i = 0; i < 320 * 224; i++) crc = (crc ^ p[i]) * 16777619u;
		debugf("[FBCRC] frame=%d crc=%08lx\n", (int)g_frame, (unsigned long)crc);
	}
	#endif

	rom_next_frame();

	#ifdef N64
	render_time = TICKS_DISTANCE(t0, TICKS_READ());
	#endif

	return FRAME_CLOCK;
}

void emu_run_frame(void) {
    uint64_t vsync = g_clock_framebegin + FRAME_CLOCK;
    EmuEvent *e;

    // Run all events that are scheduled before next vsync
    while ((e = next_event()) && (e->clock < vsync)) {
    	e->current = true;
        g_clock = m68k_exec(e->clock);
        e->current = false;

        // Call the event callback, and check if it must be repeated.
        if (g_clock >= e->clock) {    	
	        uint32_t repeat = e->cb(e->cbarg);
	        if (repeat != 0) e->clock += repeat;
	        else e->cb = NULL;
        }
    }

    while (g_clock < vsync)
    	g_clock = m68k_exec(vsync);

    g_frame++;
	g_clock_framebegin += FRAME_CLOCK;
}

int main(int argc, char *argv[]) {
	#ifndef N64
	if (argc < 2) {
		fprintf(stderr, "Usage:\n    mvs64 <romdir>\n");
		return 1;
	}
	#else 
	argc = 0; argv = NULL;
	#endif

	plat_init(44100, FPS);
	plat_enable_video(true);

	#ifdef N64
	rom_load("rom:/");
	#else
	rom_load(argv[1]);
	#endif

	#ifdef USE_FAME
	fame_adapter_init();
	#else
	m68k_init();
	#endif

	#ifdef USE_DRC
	extern void drc68k_init(void);
	drc68k_init();
	#endif

	hw_init();

	/* Initialize Z80 + YM2610 sound subsystem with M1 ROM. */
	extern uint8_t *M1_ROM;
	extern int M1_ROM_SIZE;
	sound_init(M1_ROM, M1_ROM_SIZE);

	g_clock = 0;

	#ifdef USE_FAME
	fame_adapter_reset();
	#else
	m68k_set_cpu_type(M68K_CPU_TYPE_68000);
	m68k_pulse_reset();
	#endif
	m68k_clock = 0;

	emu_add_event(LINE_CLOCK*24,  emu_render, NULL);
	emu_add_event(LINE_CLOCK*248, emu_vblank_start, NULL);

	#ifdef N64
	uint32_t fps_frame = 0;
	uint32_t fps_time = TICKS_READ();
	#endif
	while (1) {
		render_time = 0;
		profile_hw_io = 0;
		profile_dma_load = 0;
		#ifdef N64
		uint32_t t0 = TICKS_READ();
		#endif
		#ifdef N64
		/* Sound runs on the audio clock (see sound_pump), so music keeps its
		 * tempo and the audio interface never starves when FPS < 60. */
		XP_BEGIN(xp_all);
		XP_BEGIN(xp_snd); sound_frame_begin(); XP_END(xp_snd);
		emu_run_frame();
		XP_BEGIN(xp_snd); sound_pump(); XP_END(xp_snd);
		XP_END(xp_all);
		#else
		emu_run_frame();
		sound_update(Z80_CLOCK / 60);
		#endif

		if (!plat_poll()) break;
		#ifdef PROFILER_MISS
		/* Patched ares only (tools/prof/ares-missprof.patch): histogram of the
		 * code lines and data that miss the caches over frames MISS_START to
		 * MISS_END (default 1500-3000), written to $MISSPROF_OUT.  Independent
		 * of -DPROFILER, so it can be used on a production layout. */
		#ifndef MISS_START
		#define MISS_START 1500
		#define MISS_END 3000
		#endif
		if (g_frame == MISS_START) { register int64_t s asm("t0") = 0; asm volatile(".word 0x42000028 | (8 << 20) | (5 << 6)" : : "r"(s)); }
		if (g_frame == MISS_END) { register int64_t s asm("t0") = 0; asm volatile(".word 0x42000028 | (8 << 20) | (6 << 6)" : : "r"(s)); }
		#endif
		#ifdef PROFILER
		#ifdef PROFILER_SAMPLE
		{ extern void prof_frame(int); prof_frame((int)g_frame); }
		#endif
		#ifndef XSUM_START
		#define XSUM_START 1500
		#define XSUM_END   3000
		#endif
		{
			/* Exact totals over emulated frames [XSUM_START, XSUM_END): comparable
			 * between builds because AUTOPLAY input is frame-based. */
			extern unsigned long prof_insn;
			#if defined(PROFILER_OPS) || defined(PROFILER_PCS) || defined(PROFILER_OPIDX)
			extern uint32_t prof_op_fn[2048], prof_op_cnt[2048]; extern int prof_op_on;
			#else
			int prof_op_on = 0; (void)prof_op_on;
			#endif
			static xprof_t l68k, lsnd, ldraw, lall; static unsigned long linsn_base;
			#define XACC(s, p, l) (s).cyc += (p).cyc - (l).cyc, (s).ic += (p).ic - (l).ic, (s).dc += (p).dc - (l).dc
			static xprof_t t68k, tsnd, tdraw, tall; static unsigned long tinsn;
			/* running totals (the per-second printer zeroes xp_*, so accumulate deltas here) */
			XACC(t68k, xp_68k, l68k); XACC(tsnd, xp_snd, lsnd); XACC(tdraw, xp_draw, ldraw); XACC(tall, xp_all, lall);
			tinsn += prof_insn - linsn_base;
			l68k = xp_68k; lsnd = xp_snd; ldraw = xp_draw; lall = xp_all; linsn_base = prof_insn;
			#ifdef PROFILER_STATE
			if (g_frame % 500 == 0) {
				/* state fingerprint: equal between two builds = same emulation result */
				extern uint8_t WORK_RAM[]; uint32_t h = 2166136261u;
				char sl[16 * 5 + 1]; uint32_t hs = 2166136261u;
				for (int i = 0; i < 0x10000; i++) {
					h = (h ^ WORK_RAM[i]) * 16777619u; hs = (hs ^ WORK_RAM[i]) * 16777619u;
					if ((i & 0xfff) == 0xfff) { sprintf(sl + 5 * (i >> 12), "%04x ", (unsigned)(hs & 0xffff)); hs = 2166136261u; }
				}
				/* per-4 KB slices locate a difference (e.g. only the stack page) */
				debugf("[STATE] frame=%d workram=%08lx %s\n", (int)g_frame, (unsigned long)h, sl);
			}
			#endif
			#ifdef PROFILER_NAT
			extern uint64_t xn_cyc, xn_ic, xn_dc, xn_calls; static uint64_t n0[4];
			if (g_frame == XSUM_START) { n0[0] = xn_cyc; n0[1] = xn_ic; n0[2] = xn_dc; n0[3] = xn_calls; }
			if (g_frame == XSUM_END) debugf("[XSUM] native cycles=%lu icache_miss=%lu dcache_miss=%lu calls=%lu\n",
				(unsigned long)((xn_cyc - n0[0]) / (XSUM_END - XSUM_START)), (unsigned long)((xn_ic - n0[1]) / (XSUM_END - XSUM_START)),
				(unsigned long)((xn_dc - n0[2]) / (XSUM_END - XSUM_START)), (unsigned long)((xn_calls - n0[3]) / (XSUM_END - XSUM_START)));
			#endif
			{ static uint64_t r0, h0, c0;
			  if (g_frame == XSUM_START) { r0 = xprof_read(0x200); h0 = xprof_read(0x201); c0 = xprof_read(0); }
			  if (g_frame == XSUM_END) {
				uint64_t r = xprof_read(0x200) - r0, h = xprof_read(0x201) - h0, cc = xprof_read(0) - c0;
				debugf("[XSUM] rsp_busy=%lu rsp_halted=%lu cpu_clocks=%lu per frame\n", (unsigned long)((r - h) / (XSUM_END - XSUM_START)),
					(unsigned long)(h / (XSUM_END - XSUM_START)), (unsigned long)(cc / (XSUM_END - XSUM_START))); } }
			{ extern unsigned long prof_exec_calls; static unsigned long e0;
			  if (g_frame == XSUM_START) e0 = prof_exec_calls;
			  if (g_frame == XSUM_END) debugf("[XSUM] exec_calls=%lu per frame\n", (prof_exec_calls - e0) / (XSUM_END - XSUM_START)); }
			if (g_frame == XSUM_START) { xs_68k = t68k; xs_snd = tsnd; xs_draw = tdraw; xs_all = tall; xs_insn = tinsn; prof_op_on = 1; }
			if (g_frame == XSUM_END) {
				int nf = XSUM_END - XSUM_START; prof_op_on = 0;
				#define XS(name, t, s) debugf("[XSUM] %s cycles=%lu icache_miss=%lu dcache_miss=%lu\n", name, (unsigned long)(((t).cyc - (s).cyc) / nf), (unsigned long)(((t).ic - (s).ic) / nf), (unsigned long)(((t).dc - (s).dc) / nf))
				XS("all", tall, xs_all); XS("68k", t68k, xs_68k); XS("sound", tsnd, xs_snd); XS("draw", tdraw, xs_draw);
				debugf("[XSUM] insn=%lu frames=%d\n", (tinsn - xs_insn) / nf, nf);
				#ifdef PROFILER_DISP
				{ extern xprof_t xp_disp, xp_hnd; static xprof_t z;
				  /* totals since boot, per instruction x1000 would be nicer but keep it simple: whole-run sums */
				  debugf("[XDISP] dispatch cycles=%llu icache_miss=%llu dcache_miss=%llu\n", xp_disp.cyc - z.cyc, xp_disp.ic, xp_disp.dc);
				  debugf("[XDISP] handlers cycles=%llu icache_miss=%llu dcache_miss=%llu\n", xp_hnd.cyc, xp_hnd.ic, xp_hnd.dc); }
				#endif
				#ifdef PROFILER_OPIDX
				for (int i = 0; i < 2048; i++) if (prof_op_cnt[i]) debugf("[PROFIDX] %x %lu\n", i, (unsigned long)prof_op_cnt[i]);
				#endif
				#ifdef PROFILER_OPS
				for (int i = 0; i < 2048; i++) if (prof_op_cnt[i]) debugf("[PROFOP] %lx %lu\n", (unsigned long)prof_op_fn[i], (unsigned long)prof_op_cnt[i]);
				#endif
				#ifdef PROFILER_CALLS
				{ extern uint32_t prof_call[1024]; for (int i = 0; i < 1024; i++) if (prof_call[i]) debugf("[PROFCALL] %lx\n", (unsigned long)prof_call[i]); }
				#endif
				#ifdef PROFILER_PCS
				for (int i = 0; i < 2048; i++) if (prof_op_cnt[i]) debugf("[PROFPC] %lx %lu\n", (unsigned long)((prof_op_fn[i] - 1) << 5), (unsigned long)prof_op_cnt[i]);
				#endif
			}
		}
		#endif

		#ifdef N64
		uint32_t emu_time = TICKS_DISTANCE(t0, TICKS_READ());

		/* Per-frame line for profiling builds only: formatting floats every
		 * frame costs I-cache and a few % of the frame. */
		#if defined(PROFILER) || defined(PROFILE_FRAMES)
		debugf("[PROFILE] cpu:%.2f%% io:%lu draw:%.2f%% dma:%.2f%% PC:%06lx\n",
			(float)emu_time * 100.f / (float)(TICKS_PER_SECOND / 60),
			(unsigned long)profile_hw_io,
			(float)render_time * 100.f / (float)(TICKS_PER_SECOND / 60),
			(float)profile_dma_load * 100.f / (float)(TICKS_PER_SECOND / 60),
			(uint32_t)emu_pc());
		#else
		(void)emu_time;
		#endif
		#endif

		rom_next_frame();
		#ifdef N64
		{
			/* Frame limiter: never run ahead of the video interrupt count.
			 * Without it light scenes run at 80+ FPS: the game is too fast and
			 * sends sound commands faster than the (real-time) Z80 can take
			 * them, which loses commands and leaves the driver's ADPCM state
			 * stuck.  When we are behind, emu_render pulls N64_FRAME back, so
			 * no debt builds up. */
			extern volatile int N64_FRAME;
			while (N64_FRAME < g_frame) { /* wait for the VI interrupt */ }
		}
		uint32_t curtime = TICKS_READ();
		if (TICKS_DISTANCE(fps_time, curtime) > TICKS_FROM_MS(1000)) {
			static uint32_t fps_drawn;
			debugf("FPS: %.1f drawn %.1f\n", (g_frame - fps_frame) * (float)TICKS_PER_SECOND / TICKS_DISTANCE(fps_time, curtime),
				(g_drawn - fps_drawn) * (float)TICKS_PER_SECOND / TICKS_DISTANCE(fps_time, curtime));
			fps_drawn = g_drawn;
			sound_debug_stats();
			fps_frame = g_frame;
			fps_time = curtime;
		}
		#endif
	}

	debugf("end\n");
	cpu_start_trace(1000);
	m68k_exec(g_clock+100);

	#ifndef N64
	FILE *f = fopen("vram.dump", "wb");
	fwrite(VIDEO_RAM, 1, sizeof(VIDEO_RAM), f);
	fclose(f);
	#endif

	plat_save_screenshot("screen.bmp");
}
