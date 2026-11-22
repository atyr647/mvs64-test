#include "platform.h"
#include <memory.h>

volatile int N64_FRAME = 0;
uint32_t RSP_OVL_ID = 0;

DEFINE_RSP_UCODE(rsp_video);

uint8_t keystate[256];

extern char end __attribute__((section (".data")));

static void vblank_handler(void) {
    N64_FRAME++;
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
