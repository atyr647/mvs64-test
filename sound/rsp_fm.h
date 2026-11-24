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
	uint16_t reset;         /* bit 4*ch+op: restart that operator's phase (key-on) */
	uint32_t pad2;          /* RSP DMA moves multiples of 8 bytes: 228 -> 232 */
}; /* 232 bytes */

/* State the RSP carries from chunk to chunk (persistent in RDRAM; the CPU
 * only initialises it).  Each render command gets a snapshot of the
 * parameters above and merges this in: phase / op1_out / mem per channel. */
struct rsp_fm_dyn {
	struct { uint32_t phase[4]; int32_t op1_out, mem; } ch[4];   /* 96 bytes */
};

void rsp_fm_init(void);
/* ---- ADPCM-A on the RSP ----
 * The CPU keeps the register state, addresses and end-of-sample handling;
 * per render chunk it stages each channel's compressed bytes and says how
 * many samples to produce.  The RSP decodes, resamples (18.5 kHz -> 22.05
 * kHz, linear) and applies the volume, writing the per-sample L and R sums
 * (int32, before the x0.5 gain).  The SSG is added to both, x2 (the sums
 * are multiples of 4, so (a + 2*ssg) >> 1 == a/2 + ssg exactly).
 * Layout is shared with rsp_fm.S. */
#define RSP_ADPCMA_BYTES 48           /* staged bytes per channel per chunk (<= 92 samples) */
struct rsp_adpcma_chan {
	int16_t  acc;      /* 12-bit decoder accumulator, 0..4095 */
	int16_t  step;     /* step index 0..48 */
	int16_t  s0, s1;   /* last two decoded samples (sign-extended) */
	uint16_t frac;     /* resampling phase (Q16) */
	uint16_t nsamp;    /* samples to produce this chunk (0 = channel idle) */
	uint8_t  k0;       /* first nibble: 0 = high nibble of bytes[0], 1 = low */
	uint8_t  out;      /* bit0 add to L, bit1 add to R (0 = decode only); bit7 key-on: reset the decoder */
	uint8_t  mul;      /* volume multiplier 8..15 */
	uint8_t  sh;       /* volume shift 5..12 */
} __attribute__((packed));            /* 16 bytes */
/* SSG generator state (the RSP runs it per sample unless the CPU found the
 * chunk silent and advanced it in bulk).  The first 32 bytes are its state,
 * the rest is staged from the registers each chunk. */
struct rsp_ssg_state {
	uint32_t cnt[3];       /*  0 tone counters (Q16) */
	uint32_t noise_cnt;    /* 12 */
	uint32_t env_cnt;      /* 16 */
	uint32_t lfsr;         /* 20 17-bit noise LFSR */
	uint32_t env_state;    /* 24 0..31 ramp, 32 held */
	int32_t  dc;           /* 28 DC blocker (Q8) */
	uint32_t thr[3];       /* 32 tone period << 16 */
	uint32_t nthr, ethr;   /* 44, 48 noise / envelope period << 16 */
	uint8_t  tone[3];      /* 52 */
	uint8_t  noise_bit;    /* 55 */
	uint8_t  vol[3];       /* 56 registers 8..10 */
	uint8_t  mixer;        /* 59 register 7 */
	uint8_t  shape;        /* 60 register 0xD */
	uint8_t  active;       /* 61 (unused: always runs) */
	uint8_t  restart;      /* 62 shape register written: restart the envelope */
	uint8_t  pad;
};                         /* 64 bytes */

/* ADPCM-A decoder and SSG state carried from chunk to chunk by the RSP
 * (merged into the struct below: acc/step/s0/s1 per channel, SSG bytes
 * 0..31 and 52..55). */
struct rsp_mix_dyn {
	struct { int16_t acc, step, s0, s1; } a[6];   /*  48 bytes */
	uint8_t ssg[36];                               /*  36 bytes: rsp_ssg_state 0..31, 52..55 */
	uint8_t pad[12];
};                                                 /*  96 bytes */

struct rsp_adpcma_state {
	struct rsp_adpcma_chan ch[6];                 /*  96 bytes */
	struct rsp_ssg_state ssg;                     /*  64 bytes (also written back) */
	uint8_t bytes[6][RSP_ADPCMA_BYTES];           /* 288 bytes */
};                                                /* 448 bytes */

/* The RSP runs FM_OVERSAMPLE operator steps per output sample (incr[] is
 * per step) and sums them into the output, which removes most of the
 * aliasing of bright FM patches at 22050 Hz. rsp_fm.S hardcodes 2. */
#define FM_OVERSAMPLE 2

/* Renders state->num_samples (<= 128) stereo samples, int32 L,R interleaved. */
/* Queue one chunk (asynchronous): FM, then ADPCM-A + SSG.  The parameter
 * snapshots must stay untouched until rsp_audio_wait(). */
void rsp_fm_render(struct rsp_fm_state *params, struct rsp_fm_dyn *dyn, int32_t *out_buf);
void rsp_adpcma_render(struct rsp_adpcma_state *params, struct rsp_mix_dyn *dyn, int n, int32_t *out_buf);
void rsp_audio_kick(void);    /* after queueing a batch: syncpoint + start the RSP */
void rsp_audio_wait(void);    /* until everything queued before the kick is done */
void rsp_audio_init_dyn(void *dyn, int bytes);   /* after the CPU initialised a dynamic block */

#endif
