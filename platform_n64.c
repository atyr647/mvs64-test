#include "platform.h"
#include <rspq_constants.h>   /* SP_STATUS_SIG_MORE, SP_WSTATUS_SET_SIG_MORE */
#include <memory.h>

volatile int N64_FRAME = 0;
uint32_t RSP_OVL_ID = 0;

DEFINE_RSP_UCODE(rsp_video);

uint8_t keystate[256];

extern char end __attribute__((section (".data")));

/* Lost RSP wake-ups (seen as "RSP CRASH | display_get ... wait loop timed
 * out", PC:018 STATUS [halt broke sig5 sig6 sig7]): rspq wakes the RSP by
 * setting SIG_MORE and clearing HALT, and if that lands between the ucode's
 * SIG_MORE check and its `break` (RSPQCmd_WaitNewInput) the RSP sleeps with
 * work queued; libdragon's wait loops then only spin. Halted with SIG_MORE
 * set happens in no other case (the ucode clears SIG_MORE when it wakes), so
 * the VBlank interrupt re-issues the wake: a lost wake-up now costs at most a
 * frame instead of a hang. Same diagnosis as gpsp64's
 * tools/libdragon-patches/rspq-lost-wakeup.patch, without patching the SDK. */
volatile uint32_t rsp_lost_wakeups;

static void vblank_handler(void) {
    N64_FRAME++;
    uint32_t st = *SP_STATUS;
    if ((st & (SP_STATUS_HALTED | SP_STATUS_SIG_MORE)) == (SP_STATUS_HALTED | SP_STATUS_SIG_MORE)) {
        rsp_lost_wakeups++;
        *SP_STATUS = SP_WSTATUS_SET_SIG_MORE | SP_WSTATUS_CLEAR_HALT | SP_WSTATUS_CLEAR_BROKE;
    }
}

void plat_init(int audiofreq, int fps) {
#ifdef __LIBDRAGON_DEBUG_H
    debug_init_isviewer();
    debug_init_usblog();
#endif
    debugf("MVS64\n");
    register_VI_handler(vblank_handler);

    char *heap_top = (char*)0x80000000 + get_memory_size() - 0x10000;
    char *heap_end = &end;
    debugf("heap [%p - %p = %d]\n", heap_top, heap_end, heap_top-heap_end);

	joypad_init();
    // NOTE: there seems to be a bug in libdragon display library when ANTIALIAS_OFF
    // is used. Some RDP register is not configured correctly and the display is
    // corrupted on NTSC consoles.
	display_init(RESOLUTION_320x240, DEPTH_16_BPP, 2, GAMMA_NONE, ANTIALIAS_RESAMPLE);
    dfs_init(DFS_DEFAULT_LOCATION);
    rdpq_init();
    // rdpq_debug_start();  // Too slow for real-time; enable selectively

    // Register our custom RSP overlay into the RSP queue engine
    RSP_OVL_ID = rspq_overlay_register(&rsp_video);

    // Initialize audio output (22050 Hz stereo, 4 buffers)
    audio_init(22050, 4);
}

int plat_poll(void) {
	joypad_poll();
    joypad_inputs_t ckeys = joypad_get_inputs(JOYPAD_PORT_1);

    memset(keystate, 0, sizeof(keystate));

    if (ckeys.btn.d_up)     { keystate[PLAT_KEY_P1_UP] = 1; }
    if (ckeys.btn.d_down)   { keystate[PLAT_KEY_P1_DOWN] = 1; }
    if (ckeys.btn.d_left)   { keystate[PLAT_KEY_P1_LEFT] = 1; }
    if (ckeys.btn.d_right)  { keystate[PLAT_KEY_P1_RIGHT] = 1; }
    if (ckeys.btn.a)        { keystate[PLAT_KEY_P1_A] = 1; }
    if (ckeys.btn.b)        { keystate[PLAT_KEY_P1_B] = 1; }
    if (ckeys.btn.c_down)   { keystate[PLAT_KEY_P1_C] = 1; }
    if (ckeys.btn.c_right)  { keystate[PLAT_KEY_P1_D] = 1; }
    if (ckeys.btn.start)    { keystate[PLAT_KEY_P1_START] = 1; }

#ifdef AUTOPLAY
    /* Scripted input for unattended testing (make ... SND_CFLAGS=-DAUTOPLAY):
     * pulse COIN+START (and A for menus) every 2 s once booted, then play:
     * autofire on A, weave up/down/left/right, bomb with B now and then. */
    {
        static uint32_t f;
        f++;
        if (f > 600) {
            uint32_t t = f % 120;
            if (t < 6)              keystate[PLAT_KEY_COIN_1] = 1;
            if (t >= 20 && t < 26)  keystate[PLAT_KEY_P1_START] = 1;
            if (t >= 50 && t < 56)  keystate[PLAT_KEY_P1_A] = 1;
            if ((f & 7) < 4)        keystate[PLAT_KEY_P1_A] = 1;        /* autofire */
            uint32_t m = (f / 45) % 8;                                  /* weave */
            if (m == 1 || m == 2) keystate[PLAT_KEY_P1_UP] = 1;
            if (m == 5 || m == 6) keystate[PLAT_KEY_P1_DOWN] = 1;
            if (m == 3)           keystate[PLAT_KEY_P1_RIGHT] = 1;
            if (m == 7)           keystate[PLAT_KEY_P1_LEFT] = 1;
            if ((f % 900) < 4)    keystate[PLAT_KEY_P1_B] = 1;           /* bomb */
        }
    }
#endif

    return 1;
}

void plat_enable_video(int enable) {

}

void plat_save_screenshot(const char *fn) {

}

uint8_t *g_screen_ptr;
int g_screen_pitch;

void plat_beginframe(void) {
    surface_t *rdp_disp = display_get();

	g_screen_ptr = rdp_disp->buffer;
	g_screen_pitch = 320*2;

    rdpq_attach(rdp_disp, NULL);
	rdpq_set_scissor(0, 0, 320, 224);
}

void plat_endframe(void) {
	rdpq_detach_show();
}

#ifdef PROFILER_SAMPLE
/* Sampling profiler (make ... SND_CFLAGS="-DAUTOPLAY -DPROFILER -DPROFILER_SAMPLE"): a ~2.3 kHz
 * timer interrupt records the interrupted PC in 64-byte buckets, plus the 68K
 * PC, between two frame numbers; the histogram is dumped to the debug log and
 * symbolized off-target with tools/prof/symbolize.py. */
extern char __text_start[], __text_end[];
#define PROF_SHIFT 7
#define PROF_BUCKETS (0x180000 >> PROF_SHIFT)
static uint16_t prof_hist[PROF_BUCKETS];
static uint32_t prof_other, prof_total;
#define PROF_M68K_N 1024
static uint32_t prof_m68k_pc[PROF_M68K_N]; static uint16_t prof_m68k_cnt[PROF_M68K_N];
static volatile int prof_on;
extern uint32_t emu_pc(void);

static void prof_tick(int ovfl) {
    if (!prof_on) return;
    uint32_t pc = C0_EPC();
    uint32_t off = pc - (uint32_t)__text_start;
    prof_total++;
    if (off < (PROF_BUCKETS << PROF_SHIFT)) { if (prof_hist[off >> PROF_SHIFT] != 0xFFFF) prof_hist[off >> PROF_SHIFT]++; }
    else prof_other++;
    /* 68K PC (as last stored by the core), open-addressed */
    uint32_t m = emu_pc() & ~1u, h = (m * 2654435761u) >> 22;
    for (int i = 0; i < 16; i++, h = (h + 1) & (PROF_M68K_N - 1)) {
        if (prof_m68k_cnt[h] == 0 || prof_m68k_pc[h] == m) { prof_m68k_pc[h] = m; if (prof_m68k_cnt[h] != 0xFFFF) prof_m68k_cnt[h]++; break; }
    }
}

void prof_frame(int frame) {
    static int inited;
    if (!inited) { inited = 1; timer_init(); new_timer(20011, TF_CONTINUOUS, prof_tick); }
#ifndef PROF_START
#define PROF_START 1500
#define PROF_END   4500
#endif
    if (frame == PROF_START) { prof_on = 1; debugf("[PROF] start\n"); }
    if (frame == PROF_END) {
        prof_on = 0;
        debugf("[PROF] total=%lu other=%lu text=%p\n", (unsigned long)prof_total, (unsigned long)prof_other, __text_start);
        for (int i = 0; i < PROF_BUCKETS; i++) if (prof_hist[i]) debugf("[PROFN] %x %u\n", i << PROF_SHIFT, prof_hist[i]);
        for (int i = 0; i < PROF_M68K_N; i++) if (prof_m68k_cnt[i]) debugf("[PROFM] %lx %u\n", (unsigned long)prof_m68k_pc[i], prof_m68k_cnt[i]);
        debugf("[PROF] end\n");
    }
}
#endif
