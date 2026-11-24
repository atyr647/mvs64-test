/*
 * RSP FM Synthesis - CPU side (MVS64 / YM2610)
 *
 * Adapted from PicoDrive64's YM2612 RSP FM overlay.
 * CPU handles envelope state machine + register writes,
 * RSP handles the per-sample synthesis hot loop.
 *
 * YM2610 has 4 FM channels (vs YM2612's 6); operator structure
 * and algorithm routing are identical.
 */
#include <stdio.h>
#include <string.h>
#include <math.h>

#ifdef N64
#include <libdragon.h>
#endif

#include "rsp_fm.h"

#ifdef N64

DEFINE_RSP_UCODE(rsp_fm);

static uint32_t fm_ovl_id;
static int tables_uploaded = 0;

static uint16_t rsp_sin_tab[256] __attribute__((aligned(16)));
static uint16_t rsp_exp_tab[256] __attribute__((aligned(16)));

enum {
	CMD_FM_RENDER = 0,
	CMD_FM_INIT_TABLES = 1,
	CMD_ADPCMA_RENDER = 2,
};

void rsp_fm_init(void)
{
	fm_ovl_id = rspq_overlay_register(&rsp_fm);
}

static void upload_tables(void)
{
	if (tables_uploaded) return;

	/* Build sin_tab from the standard FM formula.
	 * This is the same table used by YM2612/YM2610. */
	for (int i = 0; i < 256; i++) {
		double m = sin(((i*2)+1) * M_PI / 1024.0);
		double o;
		int n;
		if (m > 0.0)
			o = 8*log(1.0/m)/log(2.0);
		else
			o = 8*log(-1.0/m)/log(2.0);
		o = o / (128.0/1024.0 / 4);
		n = (int)(2.0*o);
		if (n&1) n = (n>>1)+1;
		else     n = n>>1;
		rsp_sin_tab[i] = n;
	}

	/* exp_tab: the OPN's attenuation->volume table, extracted from the die
	 * (ymfm's s_power_table): 10-bit mantissa with the implied 0x400 bit,
	 * pre-shifted left by 2 so outputs are 13-bit (max 8168) as on the chip.
	 * Operator modulation (>>1) and feedback (>>(10-FB)) in rsp_fm.S assume
	 * this scale; the earlier 2048-max table made every FM tone 4x too
	 * weakly modulated (nearly pure sines). */
	static const uint16_t opn_power_table[256] = {
		8168, 8148, 8124, 8104, 8080, 8060, 8040, 8016, 7996, 7972, 7952, 7932, 7908, 7888, 7864, 7844,
		7824, 7804, 7780, 7760, 7740, 7720, 7696, 7676, 7656, 7636, 7616, 7592, 7572, 7552, 7532, 7512,
		7492, 7472, 7452, 7432, 7412, 7392, 7372, 7352, 7332, 7312, 7292, 7272, 7252, 7232, 7212, 7192,
		7176, 7156, 7136, 7116, 7096, 7076, 7060, 7040, 7020, 7000, 6984, 6964, 6944, 6928, 6908, 6888,
		6868, 6852, 6832, 6816, 6796, 6776, 6760, 6740, 6724, 6704, 6688, 6668, 6652, 6632, 6616, 6596,
		6580, 6560, 6544, 6524, 6508, 6492, 6472, 6456, 6436, 6420, 6404, 6384, 6368, 6352, 6336, 6316,
		6300, 6284, 6264, 6248, 6232, 6216, 6200, 6180, 6164, 6148, 6132, 6116, 6100, 6080, 6064, 6048,
		6032, 6016, 6000, 5984, 5968, 5952, 5936, 5920, 5904, 5888, 5872, 5856, 5840, 5824, 5808, 5792,
		5776, 5760, 5744, 5732, 5716, 5700, 5684, 5668, 5652, 5636, 5624, 5608, 5592, 5576, 5564, 5548,
		5532, 5516, 5504, 5488, 5472, 5456, 5444, 5428, 5412, 5400, 5384, 5368, 5356, 5340, 5328, 5312,
		5296, 5284, 5268, 5256, 5240, 5228, 5212, 5200, 5184, 5168, 5156, 5144, 5128, 5116, 5100, 5088,
		5072, 5060, 5044, 5032, 5020, 5004, 4992, 4976, 4964, 4952, 4936, 4924, 4912, 4896, 4884, 4872,
		4856, 4844, 4832, 4820, 4804, 4792, 4780, 4768, 4752, 4740, 4728, 4716, 4704, 4688, 4676, 4664,
		4652, 4640, 4628, 4616, 4600, 4588, 4576, 4564, 4552, 4540, 4528, 4516, 4504, 4492, 4480, 4468,
		4456, 4444, 4432, 4420, 4408, 4396, 4384, 4372, 4360, 4348, 4336, 4324, 4312, 4300, 4288, 4276,
		4264, 4256, 4244, 4232, 4220, 4208, 4196, 4184, 4176, 4164, 4152, 4140, 4128, 4120, 4108, 4096
	};
	for (int i = 0; i < 256; i++)
		rsp_exp_tab[i] = opn_power_table[i];

	data_cache_hit_writeback(rsp_sin_tab, sizeof(rsp_sin_tab));
	data_cache_hit_writeback(rsp_exp_tab, sizeof(rsp_exp_tab));

	rspq_write(fm_ovl_id, CMD_FM_INIT_TABLES,
		   0,
		   PhysicalAddr(rsp_sin_tab),
		   PhysicalAddr(rsp_exp_tab));
	/* The syncpoint is queued in the same batch: rspq_wait() here (a later
	 * syncpoint) hung the RSP queue in ares about one boot in two. */
	rspq_syncpoint_t s = rspq_syncpoint_new();
	rspq_flush();
	rspq_syncpoint_wait(s);

	tables_uploaded = 1;
}

/* Not the high-priority queue: audio jumping ahead of the video's RSP work
 * lowered the CPU's wait here but delayed frames (ares: 43.7 -> 29.4 FPS). */
static rspq_syncpoint_t fm_sync;

/* Queue one FM render and return at once (rsp_adpcma_render queues the
 * ADPCM-A part and the syncpoint); rsp_fm_wait() blocks until the RSP
 * has written the state and the samples back.
 *
 * The syncpoint is queued in the same batch as the render.  (Appending it
 * later, as rspq_wait() does, means waking an RSP that has just gone to sleep
 * after the render: that wake-up can be lost, and rspq_wait() then times out
 * with an RSP crash screen.  rspq_wait() would also wait for the RDP to finish
 * drawing, which the audio does not need.)
 *
 * The RSP writes both buffers with DMA, so the CPU must hold no cached copy
 * of them: the state is flushed and dropped here, the output lines are
 * dropped here too, and the caller does not touch either until rsp_fm_wait(). */
void rsp_fm_render(struct rsp_fm_state *state, int32_t *out_buf)
{
	upload_tables();

	data_cache_hit_writeback_invalidate(state, (sizeof(*state) + 15) & ~15);
	{
		uintptr_t a = (uintptr_t)out_buf & ~(uintptr_t)15;
		uintptr_t b = ((uintptr_t)(out_buf + 2 * state->num_samples) + 15) & ~(uintptr_t)15;
		data_cache_hit_invalidate((void *)a, b - a);
	}

	rspq_write(fm_ovl_id, CMD_FM_RENDER,
		   state->num_samples,
		   PhysicalAddr(state),
		   PhysicalAddr(out_buf));
}

/* Queue ADPCM-A for the chunk just queued by rsp_fm_render, then the
 * syncpoint that rsp_fm_wait() waits for (one wait covers both). */
void rsp_adpcma_render(struct rsp_adpcma_state *state, int n, int32_t *out_buf)
{
	data_cache_hit_writeback_invalidate(state, (sizeof(*state) + 15) & ~15);
	{
		uintptr_t a = (uintptr_t)out_buf & ~(uintptr_t)15;
		uintptr_t b = ((uintptr_t)(out_buf + 2 * n) + 15) & ~(uintptr_t)15;
		data_cache_hit_invalidate((void *)a, b - a);
	}
	rspq_write(fm_ovl_id, CMD_ADPCMA_RENDER, n, PhysicalAddr(state), PhysicalAddr(out_buf));
	fm_sync = rspq_syncpoint_new();
	rspq_flush();
}

void rsp_fm_wait(void)
{
	rspq_syncpoint_wait(fm_sync);
}
#else
void rsp_fm_render(struct rsp_fm_state *state, int32_t *out_buf) {}
void rsp_fm_wait(void) {}
void rsp_adpcma_render(struct rsp_adpcma_state *state, int n, int32_t *out_buf) {}
#endif
