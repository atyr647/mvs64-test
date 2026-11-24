#ifndef RSP_FM_H
#define RSP_FM_H

#include <stdint.h>

/*
 * RSP FM synthesis state — adapted from PicoDrive64 for NeoGeo YM2610.
 *
 * YM2610 has 4 FM channels (vs YM2612's 6), but the operator structure
 * is identical: 4 operators per channel, 8 algorithms, same feedback/
 * modulation routing.
 *
 * Algorithm routing parameters (computed by CPU, consumed by RSP):
 *   c1_mod:    modulation source for C1 (SLOT2): 0=zero, 1=op1_out>>16
 *   m2_mod:    modulation source for M2 (SLOT3): 0=zero, 1=prev_mem
 *   c2_mod:    modulation source for C2 (SLOT4):
 *                0=zero, 1=M2_out, 2=op1s+M2, 3=mem+M2, 4=op1s
 *   out_flags: which operators contribute to output:
 *                bit0=op1s, bit1=C1, bit2=M2, bit3=C2
 *   mem_src:   what writes to delay memory:
 *                0=none, 1=C1, 2=op1s+C1, 3=op1s
 */

#define YM2610_FM_CHANNELS  4

/* Per-channel state sent to RSP for FM synthesis */
struct rsp_fm_chan {
	uint32_t phase[4];     /* 0-15: operator phase counters */
	uint32_t incr[4];      /* 16-31: phase increments */
	uint16_t vol_out[4];   /* 32-39: envelope levels (attenuation) */
	uint8_t  c1_mod;       /* 40: C1 modulation source */
	uint8_t  fb_shift;     /* 41: feedback shift (0=off, 1-7) */
	uint8_t  m2_mod;       /* 42: M2 modulation source */
	uint8_t  enabled;      /* 43: channel active */
	int32_t  op1_out;      /* 44-47: operator 1 feedback state */
	int32_t  mem;          /* 48-51: algorithm delay memory */
	uint8_t  c2_mod;       /* 52: C2 modulation type */
	uint8_t  out_flags;    /* 53: output contribution flags */
	uint8_t  mem_src;      /* 54: delay memory source */
	uint8_t  pan;          /* 55: output enable, bit7 = left, bit6 = right */
}; /* 56 bytes */

/* Full state for YM2610 FM channels */
struct rsp_fm_state {
	struct rsp_fm_chan ch[YM2610_FM_CHANNELS];  /* 224 bytes */
	uint16_t num_samples;
	uint16_t pad;
	uint32_t pad2;          /* RSP DMA moves multiples of 8 bytes: 228 -> 232 */
}; /* 232 bytes */

void rsp_fm_init(void);
/* ---- ADPCM-A on the RSP ----
 * The CPU keeps the register state, addresses and end-of-sample handling;
 * per render chunk it stages each channel's compressed bytes and says how
 * many samples to produce.  The RSP decodes, resamples (18.5 kHz -> 22.05
 * kHz, linear) and applies the volume, writing the per-sample L and R sums
 * (int32, before the x0.5 gain).  Layout is shared with rsp_fm.S. */
#define RSP_ADPCMA_BYTES 48           /* staged bytes per channel per chunk (<= 92 samples) */
struct rsp_adpcma_chan {
	int16_t  acc;      /* 12-bit decoder accumulator, 0..4095 */
	int16_t  step;     /* step index 0..48 */
	int16_t  s0, s1;   /* last two decoded samples (sign-extended) */
	uint16_t frac;     /* resampling phase (Q16) */
	uint16_t nsamp;    /* samples to produce this chunk (0 = channel idle) */
	uint8_t  k0;       /* first nibble: 0 = high nibble of bytes[0], 1 = low */
	uint8_t  out;      /* bit0 add to L, bit1 add to R (0 = decode only) */
	uint8_t  mul;      /* volume multiplier 8..15 */
	uint8_t  sh;       /* volume shift 5..12 */
} __attribute__((packed));            /* 16 bytes */
struct rsp_adpcma_state {
	struct rsp_adpcma_chan ch[6];                 /*  96 bytes */
	uint8_t bytes[6][RSP_ADPCMA_BYTES];           /* 288 bytes */
};                                                /* 384 bytes */

/* The RSP runs FM_OVERSAMPLE operator steps per output sample (incr[] is
 * per step) and sums them into the output, which removes most of the
 * aliasing of bright FM patches at 22050 Hz. rsp_fm.S hardcodes 2. */
#define FM_OVERSAMPLE 2

/* Renders state->num_samples (<= 128) stereo samples, int32 L,R interleaved. */
void rsp_fm_render(struct rsp_fm_state *state, int32_t *out_buf);   /* asynchronous */
/* queue ADPCM-A for the same chunk (call right after rsp_fm_render) */
void rsp_adpcma_render(struct rsp_adpcma_state *state, int n, int32_t *out_buf);
void rsp_fm_wait(void);                                              /* until that render is done */

#endif
