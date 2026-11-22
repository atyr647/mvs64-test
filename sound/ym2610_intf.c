/*
 * YM2610 interface for MVS64
 *
 * Z80 sound CPU + YM2610 register interface.
 * Uses CZ80 for Z80 emulation, RSP overlay for FM synthesis.
 *
 * TODO: This is a scaffold. The YM2610 register decoder and
 * envelope state machine need to be implemented to actually
 * produce sound. Currently sets up Z80 execution and I/O
 * port routing.
 */

#include <stdio.h>
#include <string.h>
#ifdef N64
#include <libdragon.h>
#endif
#include "ym2610_intf.h"
#include "rsp_fm.h"

/* CZ80 Z80 emulator */
#include "../cpu/cz80/cz80.h"

/* ---- Z80 Memory Map (NeoGeo) ---- */

#define Z80_ROM_SIZE    (128 * 1024)  /* M1 ROM, up to 128 KB */
#define Z80_RAM_SIZE    (2 * 1024)    /* 2 KB work RAM */

static uint8_t z80_rom[Z80_ROM_SIZE];
static uint8_t z80_ram[Z80_RAM_SIZE];
static cz80_struc z80_cpu;

/* 68K↔Z80 command/response latches */
static uint8_t cmd_latch;
static uint8_t result_latch = 0x01; /* "ready" initial value */
static int nmi_pending;
static int nmi_enabled;      /* port $08 write enables, $18 write disables */

/* Z80 bank windows (selected by IN A,($08..$0B); bank number = high byte of port) */
static uint32_t z80_bank_off[4];  /* ROM offsets: [0]=$F000 2K [1]=$E000 4K [2]=$C000 8K [3]=$8000 16K */

/* YM2610 register state */
static uint8_t ym_addr1;  /* address port 1 latch */
static uint8_t ym_addr2;  /* address port 2 latch */

/* YM2610 Timer state.
 * Timer A: 10-bit, period = 72 * (1024 - TA) / 8MHz → in Z80 cycles (4 MHz): 36 * (1024 - TA)
 * Timer B:  8-bit, period = 1152 * (256 - TB) / 8MHz → in Z80 cycles: 576 * (256 - TB) */
static uint16_t timer_a_val;      /* 10-bit value from regs $24/$25 */
static uint8_t  timer_b_val;      /* 8-bit value from reg $26 */
static uint8_t  timer_ctrl;       /* reg $27: enable/load/reset */
static int32_t  timer_a_counter;  /* cycles until next Timer A overflow */
static int32_t  timer_b_counter;  /* cycles until next Timer B overflow */
static uint8_t  ym_status;        /* status register 0 (bit0=TimerA, bit1=TimerB) */
static uint8_t  ym_adpcm_status;  /* status register 1 (bit7=ADPCM-B EOS, bits0-5=ADPCM-A end) */
static uint8_t  timer_irq_ena;    /* bits: 0=Timer A IRQ, 1=Timer B IRQ */

#define TIMER_A_PERIOD(v)  (36 * (1024 - (v)))
#define TIMER_B_PERIOD(v)  (576 * (256 - (v)))

/* YM2610 IRQ output is level-sensitive: asserted while any enabled flag is set. */
static void ym_update_irq(void)
{
	int on = (ym_status & timer_irq_ena & 0x03) != 0;
	Cz80_Set_IRQ(&z80_cpu, 0, on ? ASSERT_LINE : CLEAR_LINE);
}

static void timers_advance(int cycles)
{
	if (timer_ctrl & 0x01) { /* Timer A running */
		timer_a_counter -= cycles;
		if (timer_a_counter <= 0) {
			ym_status |= 0x01;
			int period = TIMER_A_PERIOD(timer_a_val);
			if (period <= 0) period = 1;
			while (timer_a_counter <= 0)
				timer_a_counter += period;
			ym_update_irq();
		}
	}
	if (timer_ctrl & 0x02) { /* Timer B running */
		timer_b_counter -= cycles;
		if (timer_b_counter <= 0) {
			ym_status |= 0x02;
			int period = TIMER_B_PERIOD(timer_b_val);
			if (period <= 0) period = 1;
			while (timer_b_counter <= 0)
				timer_b_counter += period;
			ym_update_irq();
		}
	}
}

/* RSP FM state (filled by CPU-side envelope updates) */
/* The RSP DMAs channel phases back into this block, so the CPU must never hold
 * a stale cached copy: keep it on its own 16-byte-aligned cache lines (256 bytes)
 * so it can be invalidated after each render without touching neighbours. */
static struct {
	struct rsp_fm_state st;
	uint8_t pad[256 - sizeof(struct rsp_fm_state)];
} __attribute__((aligned(16))) fm_wrap;
#define fm_state (fm_wrap.st)
static int32_t __attribute__((aligned(16))) fm_output[512]; /* max samples per frame (16-byte aligned for cache ops) */

/* Diagnostic counters */
static uint32_t stat_cmd_count;      /* 68K commands sent to Z80 */
static uint32_t stat_reg1_writes;    /* YM2610 port 1 register writes */
static uint32_t stat_reg2_writes;    /* YM2610 port 2 register writes */
static uint32_t stat_keyon_writes;   /* Register $28 writes */
static uint32_t stat_frames;         /* sound_render calls */
static uint32_t stat_update_calls;   /* sound_update invocations */
static uint32_t stat_port_writes;    /* any Z80 OUT instruction */
static uint32_t stat_port_reads;     /* any Z80 IN instruction */
static uint32_t stat_ram_writes;     /* Z80 writes to RAM */
static uint32_t stat_nmis_fired;     /* NMIs actually raised to CZ80 */
static uint8_t  cmd_log[256]; static uint32_t cmd_log_printed;
static int32_t  stat_peak;           /* peak |sample| since last stats dump */
static uint32_t stat_nonzero;
static uint32_t stat_ssg_nonzero;        /* non-zero output samples since last dump */
static uint32_t stat_port_hist[256]; /* per-port OUT histogram */
static uint32_t stat_port_rhist[256]; /* per-port IN histogram */
static uint8_t stat_p04_vals[16];   /* first unique values written to $04 */
static int stat_p04_nvals;
/* Trace buffer: first 32 port write/read events with PC */
struct port_event { uint16_t port; uint8_t val; uint8_t is_read; uint16_t pc; };
static struct port_event port_trace[32];
static int port_trace_n;
static int z80_has_rom = 0;

void sound_debug_stats(void)
{
#ifdef N64
	uint32_t z80_pc = Cz80_Get_Reg(&z80_cpu, CZ80_PC);
	uint32_t z80_sp = Cz80_Get_Reg(&z80_cpu, CZ80_SP);
	debugf("[SND] rom=%d cmd=%lu nmi=%lu upd=%lu portW=%lu portR=%lu ramW=%lu reg1=%lu reg2=%lu keyon=%lu pc=%04lx sp=%04lx\n",
		z80_has_rom,
		(unsigned long)stat_cmd_count,
		(unsigned long)stat_nmis_fired,
		(unsigned long)stat_update_calls,
		(unsigned long)stat_port_writes,
		(unsigned long)stat_port_reads,
		(unsigned long)stat_ram_writes,
		(unsigned long)stat_reg1_writes,
		(unsigned long)stat_reg2_writes,
		(unsigned long)stat_keyon_writes,
		(unsigned long)z80_pc,
		(unsigned long)z80_sp);
	debugf("[SND] audio: peak=%ld nonzero=%lu ssg=%lu fm_ch_en=%d%d%d%d\n",
		(long)stat_peak, (unsigned long)stat_nonzero, (unsigned long)stat_ssg_nonzero,
		fm_state.ch[0].enabled, fm_state.ch[1].enabled,
		fm_state.ch[2].enabled, fm_state.ch[3].enabled);
	if (cmd_log_printed < stat_cmd_count) {
		debugf("[SND] cmds:");
		for (; cmd_log_printed < stat_cmd_count && cmd_log_printed < sizeof(cmd_log); cmd_log_printed++)
			debugf(" %02x", cmd_log[cmd_log_printed]);
		debugf("\n");
		cmd_log_printed = stat_cmd_count;
	}
	stat_peak = 0; stat_nonzero = 0; stat_ssg_nonzero = 0;

	/* Find top 6 ports by write count */
	int top[6] = {-1,-1,-1,-1,-1,-1};
	for (int i = 0; i < 256; i++) {
		if (stat_port_hist[i] == 0) continue;
		for (int s = 0; s < 6; s++) {
			if (top[s] < 0 || stat_port_hist[i] > stat_port_hist[top[s]]) {
				for (int k = 5; k > s; k--) top[k] = top[k-1];
				top[s] = i;
				break;
			}
		}
	}
	debugf("[SND] outW:");
	for (int s = 0; s < 6; s++) {
		if (top[s] < 0) break;
		debugf(" $%02x=%lu", top[s], (unsigned long)stat_port_hist[top[s]]);
	}

	/* Find top 6 ports by read count */
	int rtop[6] = {-1,-1,-1,-1,-1,-1};
	for (int i = 0; i < 256; i++) {
		if (stat_port_rhist[i] == 0) continue;
		for (int s = 0; s < 6; s++) {
			if (rtop[s] < 0 || stat_port_rhist[i] > stat_port_rhist[rtop[s]]) {
				for (int k = 5; k > s; k--) rtop[k] = rtop[k-1];
				rtop[s] = i;
				break;
			}
		}
	}
	debugf(" | inR:");
	for (int s = 0; s < 6; s++) {
		if (rtop[s] < 0) break;
		debugf(" $%02x=%lu", rtop[s], (unsigned long)stat_port_rhist[rtop[s]]);
	}
	debugf("\n");

	/* Dump first unique values written to $04 */
	debugf("[SND] $04 vals:");
	for (int i = 0; i < stat_p04_nvals && i < 16; i++)
		debugf(" %02x", stat_p04_vals[i]);
	debugf(" | tmr_ctrl=%02x tmr_a=%d tmr_b=%d status=%02x adpcm=%02x\n",
		timer_ctrl, timer_a_val, timer_b_val, ym_status, ym_adpcm_status);

	/* Dump first 32 port events */
	if (port_trace_n > 0) {
		debugf("[SND] trace:");
		for (int i = 0; i < port_trace_n; i++)
			debugf(" %c%04x=%02x@%04x",
				port_trace[i].is_read ? 'R' : 'W',
				port_trace[i].port, port_trace[i].val,
				port_trace[i].pc);
		debugf("\n");
	}

#endif
}

/* Forward declarations for YM2610 register handlers */
static void ym2610_write_reg1(uint8_t addr, uint8_t val);
static void ym2610_write_reg2(uint8_t addr, uint8_t val);

/* ---- Z80 memory read/write callbacks ---- */
/* CZ80 signatures: read(u32) → u8, write(u32, u8) */

static void z80_map_banks(void)
{
	/* Keep CZ80's opcode-fetch pages in sync with the bank windows. */
	Cz80_Set_Fetch(&z80_cpu, 0x0000, 0x7FFF, (uintptr_t)z80_rom);
	Cz80_Set_Fetch(&z80_cpu, 0x8000, 0xBFFF, (uintptr_t)(z80_rom + z80_bank_off[3]) - 0x8000);
	Cz80_Set_Fetch(&z80_cpu, 0xC000, 0xDFFF, (uintptr_t)(z80_rom + z80_bank_off[2]) - 0xC000);
	Cz80_Set_Fetch(&z80_cpu, 0xE000, 0xEFFF, (uintptr_t)(z80_rom + z80_bank_off[1]) - 0xE000);
	Cz80_Set_Fetch(&z80_cpu, 0xF000, 0xF7FF, (uintptr_t)(z80_rom + z80_bank_off[0]) - 0xF000);
	Cz80_Set_Fetch(&z80_cpu, 0xF800, 0xFFFF, (uintptr_t)z80_ram - 0xF800);
}

static UINT8 z80_read(UINT32 addr)
{
	addr &= 0xFFFF;
	if (addr < 0x8000)  return z80_rom[addr];
	if (addr >= 0xF800) return z80_ram[addr - 0xF800];
	if (addr >= 0xF000) return z80_rom[(z80_bank_off[0] + (addr - 0xF000)) & (Z80_ROM_SIZE - 1)];
	if (addr >= 0xE000) return z80_rom[(z80_bank_off[1] + (addr - 0xE000)) & (Z80_ROM_SIZE - 1)];
	if (addr >= 0xC000) return z80_rom[(z80_bank_off[2] + (addr - 0xC000)) & (Z80_ROM_SIZE - 1)];
	return z80_rom[(z80_bank_off[3] + (addr - 0x8000)) & (Z80_ROM_SIZE - 1)];
}

static void z80_write(UINT32 addr, UINT8 val)
{
	addr &= 0xFFFF;
	if (addr >= 0xF800) {
		z80_ram[addr - 0xF800] = val;
		stat_ram_writes++;
	}
}

/* ---- Z80 I/O port callbacks ---- */

static UINT8 z80_port_read(UINT16 fullport)
{
	int port = fullport & 0xFF;
	int hi = fullport >> 8;
	stat_port_reads++;
	stat_port_rhist[port]++;
	switch (port) {
	case 0x00: /* Command from 68K (clears pending NMI) */
		nmi_pending = 0;
		return cmd_latch;
	case 0x04: /* YM2610 status port 1 (timer flags) */
		return ym_status;
	case 0x06: /* YM2610 status port 2 (ADPCM flags) */
		return ym_adpcm_status;
	case 0x08: z80_bank_off[0] = ((uint32_t)hi * 0x0800) & (Z80_ROM_SIZE - 1); z80_map_banks(); return 0;
	case 0x09: z80_bank_off[1] = ((uint32_t)hi * 0x1000) & (Z80_ROM_SIZE - 1); z80_map_banks(); return 0;
	case 0x0A: z80_bank_off[2] = ((uint32_t)hi * 0x2000) & (Z80_ROM_SIZE - 1); z80_map_banks(); return 0;
	case 0x0B: z80_bank_off[3] = ((uint32_t)hi * 0x4000) & (Z80_ROM_SIZE - 1); z80_map_banks(); return 0;
	default:
		return 0xFF;
	}
}

static void z80_port_write(UINT16 fullport, UINT8 val)
{
	int port = fullport & 0xFF;
	stat_port_writes++;
	stat_port_hist[port]++;
	switch (port) {
	case 0x04: ym_addr1 = val; break;           /* YM2610 address port 1 */
	case 0x05: ym2610_write_reg1(ym_addr1, val); break;
	case 0x06: ym_addr2 = val; break;           /* YM2610 address port 2 */
	case 0x07: ym2610_write_reg2(ym_addr2, val); break;
	case 0x08: nmi_enabled = 1; break;          /* enable NMI from 68K */
	case 0x18: nmi_enabled = 0; break;          /* disable NMI from 68K */
	case 0x0C: result_latch = val; break;       /* response to 68K */
	default: break;                             /* $00 (clear latch), $C0 etc: ignore */
	}
}

/* ---- YM2610 register write handlers ---- */

/*
 * YM2610 register map (simplified):
 *
 * Port 1 ($04/$05):
 *   $00-$0F: SSG (AY-3-8910 compatible)
 *   $10-$1F: ADPCM-B
 *   $20:     LFO frequency
 *   $24-$27: Timer A/B
 *   $28:     Key on/off
 *   $30-$9F: FM ch1/ch3 operator params (DT/MUL, TL, KS/AR, DR, SR, SL/RR)
 *   $A0-$A7: FM ch1/ch3 frequency
 *   $B0-$B7: FM ch1/ch3 feedback/algorithm
 *
 * Port 2 ($06/$07):
 *   $00-$0F: ADPCM-A
 *   $10-$1F: ADPCM-A per-channel volume
 *   $30-$9F: FM ch2/ch4 operator params
 *   $A0-$A7: FM ch2/ch4 frequency
 *   $B0-$B7: FM ch2/ch4 feedback/algorithm
 */

/* ---- FM Operator / Channel state (CPU-side) ---- */

/* Detune table (same as YM2612) */
static const int32_t dt_tab[4 * 32] = {
/* DT0 */ 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
/* DT1 */ 0,0,0,0,1,1,1,1,1,1,1,1,2,2,2,2,2,3,3,3,4,4,4,5,5,6,6,7,8,8,8,8,
/* DT2 */ 1,1,1,1,2,2,2,2,2,3,3,3,4,4,4,5,5,6,6,7,8,8,9,10,11,12,13,14,16,16,16,16,
/* DT3 */ 2,2,2,2,2,3,3,3,4,4,4,5,5,6,6,7,8,8,9,10,11,12,13,14,16,17,19,20,22,22,22,22
};

/* Algorithm routing decomposition table.
 * For each of 8 algorithms, gives: c1_mod, m2_mod, c2_mod, out_flags, mem_src */
static const uint8_t algo_table[8][5] = {
	/* algo 0: M1→C1→M2→C2 */       {1, 1, 1, 8,  1},
	/* algo 1: (M1+C1)→M2→C2 */     {0, 1, 1, 8,  2},
	/* algo 2: (M1+(C1→M2))→C2 */   {0, 0, 3, 8,  3},
	/* algo 3: ((M1→C1)+M2)→C2 */   {1, 0, 1, 8,  1},
	/* algo 4: (M1→C1)+(M2→C2) */   {1, 0, 1, 12, 0},
	/* algo 5: M1→(C1+M2+C2) */     {1, 1, 1, 14, 0},
	/* algo 6: (M1→C1)+M2+C2 */     {1, 0, 0, 14, 0},
	/* algo 7: M1+C1+M2+C2 */       {0, 0, 0, 15, 0},
};

/* Per-operator state */
typedef struct {
	uint8_t  dt;       /* detune (0-7) */
	uint8_t  mul;      /* multiply (0-15) */
	uint8_t  tl;       /* total level (0-127, attenuation in 0.75dB steps) */
	uint8_t  ks;       /* key scale (0-3) */
	uint8_t  ar;       /* attack rate (0-31) */
	uint8_t  d1r;      /* decay 1 rate (0-31) */
	uint8_t  d2r;      /* decay 2 rate (0-31) */
	uint8_t  rr;       /* release rate (0-15) */
	uint8_t  sl;       /* sustain level (0-15) */
	uint8_t  key_on;   /* key state */
	/* Envelope generator */
	uint8_t  eg_state; /* EG_OFF / EG_ATT / EG_DEC / EG_SUS / EG_REL */
	uint16_t env;      /* current attenuation 0 (loud) .. 0x3FF (silent) */
	uint8_t  r_ar, r_d1r, r_d2r, r_rr;  /* effective rates 0..63 incl. key scaling (0 = never) */
	uint16_t sl_lvl;   /* decay->sustain switch level in env units */
} ym_slot_t;

/* Per-channel state */
typedef struct {
	ym_slot_t slot[4];   /* 4 operators: M1(0), C1(1), M2(2), C2(3) */
	uint16_t  fnum;      /* frequency number (11 bits) */
	uint8_t   block;     /* block/octave (3 bits) */
	uint8_t   fb;        /* feedback (0-7) */
	uint8_t   algo;      /* algorithm (0-7) */
	uint8_t   active;    /* any key on? */
} ym_chan_t;

static ym_chan_t ym_ch[4]; /* 4 FM channels for YM2610 */

/* Map operator register offset to slot index.
 * YM2610 slot ordering: 0→M1, 1→M2, 2→C1, 3→C2
 * (different from logical order — inherited from OPN family) */
static const int slot_map[4] = { 0, 2, 1, 3 };


/* ---- Envelope generator (OPN: 10-bit attenuation, 0.09375 dB/unit) ---- */

enum { EG_OFF = 0, EG_ATT, EG_DEC, EG_SUS, EG_REL };
#define EG_MAX 0x3FF

/* Increment patterns, indexed by (eg_cnt >> shift) & 7 */
static const uint8_t eg_pat_lo[4][8] = {   /* rates 0..47 (groups 0..11) */
	{0,1,0,1,0,1,0,1}, {0,1,0,1,1,1,0,1}, {0,1,1,1,0,1,1,1}, {0,1,1,1,1,1,1,1},
};
static const uint8_t eg_pat_hi[4][8] = {   /* rates 48..51 (group 12), x2 per group above */
	{1,1,1,1,1,1,1,1}, {1,1,1,2,1,1,1,2}, {1,2,1,2,1,2,1,2}, {1,2,2,2,1,2,2,2},
};

/* One EG tick = 3 native samples = 3 * 144 / 8 MHz; per 22050 Hz output sample
 * that is (8e6/144/3)/22050 = 0.83987 ticks (Q16). */
#define EG_TICKS_PER_SAMPLE_Q16  55042u
static uint32_t eg_acc;
static uint32_t eg_cnt = 1;

static inline int eg_rate_eff(int base, int ks, int kc)
{
	if (base == 0) return 0;                 /* R=0: envelope never moves */
	int r = base + (kc >> (3 - ks));
	return r > 63 ? 63 : r;
}

/* Increment for one EG tick at effective rate r (0 = none) */
static inline int eg_step(int r)
{
	if (r < 2) return 0;
	int g = r >> 2, sub = r & 3;
	if (g < 12) {
		int sh = 11 - g;
		if (eg_cnt & ((1u << sh) - 1)) return 0;
		return eg_pat_lo[sub][(eg_cnt >> sh) & 7];
	}
	if (g == 15) return (r >= 60) ? 8 : eg_pat_hi[sub][eg_cnt & 7] * 8;
	return eg_pat_hi[sub][eg_cnt & 7] << (g - 12);
}

static void eg_tick_slot(ym_slot_t *s)
{
	switch (s->eg_state) {
	case EG_ATT: {
		int inc = eg_step(s->r_ar);
		if (inc) {
			s->env += (int)(((~(int)s->env) * inc) >> 4);
			if ((int16_t)s->env <= 0) { s->env = 0; s->eg_state = EG_DEC; }
		}
		break; }
	case EG_DEC: {
		s->env += eg_step(s->r_d1r);
		if (s->env >= s->sl_lvl) s->eg_state = EG_SUS;
		break; }
	case EG_SUS:
		s->env += eg_step(s->r_d2r);
		if (s->env >= EG_MAX) { s->env = EG_MAX; s->eg_state = EG_OFF; }
		break;
	case EG_REL:
		s->env += eg_step(s->r_rr);
		if (s->env >= EG_MAX) { s->env = EG_MAX; s->eg_state = EG_OFF; }
		break;
	default: break;
	}
}

/* Advance every slot's envelope by the time of nsamples output samples */
static void eg_run(int nsamples)
{
	eg_acc += (uint32_t)nsamples * EG_TICKS_PER_SAMPLE_Q16;
	uint32_t ticks = eg_acc >> 16;
	eg_acc &= 0xFFFF;
	while (ticks--) {
		for (int c = 0; c < 4; c++)
			for (int o = 0; o < 4; o++) {
				ym_slot_t *s = &ym_ch[c].slot[o];
				if (s->eg_state != EG_OFF) eg_tick_slot(s);
			}
		eg_cnt++;
	}
}

static inline uint16_t slot_vol(const ym_slot_t *s)
{
	if (s->eg_state == EG_OFF) return EG_MAX;
	int v = s->env + (s->tl << 3);
	return v > EG_MAX ? EG_MAX : v;
}

/* Push envelope-derived volumes + enable flags into the RSP state */
static void eg_push(void)
{
	for (int c = 0; c < 4; c++) {
		ym_chan_t *ch = &ym_ch[c];
		struct rsp_fm_chan *rch = &fm_state.ch[c];
		int on = 0;
		for (int o = 0; o < 4; o++) {
			rch->vol_out[o] = slot_vol(&ch->slot[o]);
			if (ch->slot[o].eg_state != EG_OFF) on = 1;
		}
		rch->enabled = on;
	}
}

/* Key on/off for one slot */
static void slot_key(int ch_idx, int op, int on)
{
	ym_slot_t *s = &ym_ch[ch_idx].slot[op];
	if (on && !s->key_on) {
		s->key_on = 1;
		fm_state.ch[ch_idx].phase[op] = 0;          /* phase restarts on key-on */
		if (s->r_ar >= 62) { s->env = 0; s->eg_state = EG_DEC; }   /* instant attack */
		else               { s->eg_state = EG_ATT; }
	} else if (!on && s->key_on) {
		s->key_on = 0;
		if (s->eg_state != EG_OFF) s->eg_state = EG_REL;
	}
}

/* Update the rsp_fm_state for one channel from CPU-side state */
static void update_fm_channel(int ch_idx)
{
	ym_chan_t *ch = &ym_ch[ch_idx];
	struct rsp_fm_chan *rch = &fm_state.ch[ch_idx];

	/* Frequency -> phase increment per OUTPUT sample.
	 *
	 * Native OPN: f = fnum * 2^block * fs / 2^21 with fs = 8 MHz / 144.
	 * The RSP keeps a 32-bit phase whose bits 16..25 index a 1024-step sine,
	 * so one cycle is 2^26 units, and it advances once per 22050 Hz output
	 * sample:  inc = f * 2^26 / 22050 = (fnum << block) * PHASE_K  with
	 * PHASE_K = (8e6/144) * 32 / 22050 = 80.625 (Q16 below). */
	#define PHASE_K_Q16   5283830u    /* 80.6248 * 65536 */
	#define DETUNE_SCALE  161         /* a detune unit is 1/2^20 of a native step = 2*PHASE_K here */
	uint32_t base_inc = (uint32_t)(((uint64_t)((uint32_t)(ch->fnum & 0x7FF) << ch->block) * PHASE_K_Q16) >> 16);

	/* Key code for detune: block + two note bits derived from fnum's top bits */
	int f11 = (ch->fnum >> 10) & 1, f10 = (ch->fnum >> 9) & 1;
	int f9  = (ch->fnum >> 8) & 1,  f8  = (ch->fnum >> 7) & 1;
	int n3  = (f11 & (f10 | f9 | f8)) | ((f11 ^ 1) & f10 & f9 & f8);
	int kc  = ((ch->block & 7) << 2) | (f11 << 1) | n3;

	for (int op = 0; op < 4; op++) {
		ym_slot_t *sl = &ch->slot[op];

		/* Apply detune */
		int dt_idx = (sl->dt & 3) * 32 + kc;
		int32_t dt_val = dt_tab[dt_idx] * DETUNE_SCALE;
		if (sl->dt & 4) dt_val = -dt_val;

		/* Apply multiply (MUL=0 means x0.5) */
		uint32_t inc = base_inc;
		if (sl->mul) inc = inc * sl->mul;
		else         inc = inc >> 1;
		inc += dt_val;

		rch->phase[op] = rch->phase[op]; /* preserve running phase */
		rch->incr[op] = inc;

		/* Envelope rates (with key scaling) and sustain level */
		sl->r_ar  = eg_rate_eff(2 * sl->ar,  sl->ks, kc);
		sl->r_d1r = eg_rate_eff(2 * sl->d1r, sl->ks, kc);
		sl->r_d2r = eg_rate_eff(2 * sl->d2r, sl->ks, kc);
		sl->r_rr  = eg_rate_eff(4 * sl->rr + 2, sl->ks, kc);
		sl->sl_lvl = (sl->sl == 15 ? 31 : sl->sl) << 5;

		rch->vol_out[op] = slot_vol(sl);
	}

	/* Algorithm routing */
	const uint8_t *at = algo_table[ch->algo & 7];
	rch->c1_mod    = at[0];
	rch->m2_mod    = at[1];
	rch->c2_mod    = at[2];
	rch->out_flags = at[3];
	rch->mem_src   = at[4];

	/* Feedback */
	rch->fb_shift = ch->fb ? (ch->fb + 6) : 0;

	/* Channel active while any operator's envelope is still running */
	{
		int on = 0;
		for (int o = 0; o < 4; o++) if (ch->slot[o].eg_state != EG_OFF) on = 1;
		rch->enabled = on;
	}
}

/* Write to a FM operator register.
 * ch_pair: 0 for channels 0/1 (port 1), 1 for channels 2/3 (port 2) */
static void ym_write_fm_reg(int ch_pair, uint8_t addr, uint8_t val)
{
	if ((addr & 3) == 0 || (addr & 3) == 3) return; /* only ch1/ch2 of each port exist */
	int ch_offset = (addr & 3) - 1;
	int ch_idx = ch_pair * 2 + ch_offset;
	if (ch_idx >= 4) return;

	ym_chan_t *ch = &ym_ch[ch_idx];

	if (addr >= 0x30 && addr < 0xA0) {
		/* Operator parameter registers */
		int op_idx = slot_map[(addr >> 2) & 3];
		ym_slot_t *sl = &ch->slot[op_idx];
		int reg = addr & 0xF0;

		switch (reg) {
		case 0x30: /* DT / MUL */
			sl->dt  = (val >> 4) & 7;
			sl->mul = val & 0xF;
			break;
		case 0x40: /* TL */
			sl->tl = val & 0x7F;
			break;
		case 0x50: /* KS / AR */
			sl->ks = (val >> 6) & 3;
			sl->ar = val & 0x1F;
			break;
		case 0x60: /* D1R (+ AM on bit 7) */
			sl->d1r = val & 0x1F;
			break;
		case 0x70: /* D2R */
			sl->d2r = val & 0x1F;
			break;
		case 0x80: /* SL / RR */
			sl->sl = (val >> 4) & 0xF;
			sl->rr = val & 0xF;
			break;
		}
		update_fm_channel(ch_idx);

	} else if (addr >= 0xA0 && addr < 0xB0) {
		/* Frequency registers */
		int freg = addr & 0x0F;
		if (freg < 4) {
			/* A0-A3: frequency number low 8 bits */
			ch->fnum = (ch->fnum & 0x700) | val;
			update_fm_channel(ch_idx);
		} else if (freg >= 4 && freg < 8) {
			/* A4-A7: block + frequency number high 3 bits */
			ch->fnum = (ch->fnum & 0xFF) | ((val & 0x07) << 8);
			ch->block = (val >> 3) & 7;
			update_fm_channel(ch_idx);
		}

	} else if (addr >= 0xB0 && addr < 0xB8) {
		/* B0-B3: feedback / algorithm */
		ch->fb   = (val >> 3) & 7;
		ch->algo = val & 7;
		update_fm_channel(ch_idx);
	}
}


/* ---- SSG (AY-3-8910 compatible): 3 square channels + noise + envelope ----
 *
 * Runs on the CPU at the 22050 Hz output rate.  The chip's SSG clock is
 * 8 MHz / 4 = 2 MHz; tone half-periods are counted in units of clock/8
 * (250 kHz), so f_tone = 2 MHz / (16 * period).  Noise steps every
 * 2*NP units, the envelope every 2*EP units (16 levels per ramp). */

static uint8_t  ssg_reg[16];
static uint32_t ssg_cnt[3], ssg_noise_cnt, ssg_env_cnt;
static uint8_t  ssg_tone[3];
static uint32_t ssg_lfsr = 1;
static uint8_t  ssg_noise_bit;
static int      ssg_env_level, ssg_env_dir, ssg_env_stopped;
static int32_t  ssg_dc;

#define SSG_TICKS_Q16   743040u   /* (250000 / 22050) * 65536 */

/* 2^((n-15)/2) * 3000: AY-style 3 dB-per-step volume table */
static const int16_t ssg_vol_tab[16] = {
	0, 23, 33, 47, 66, 94, 133, 188, 265, 375, 530, 750, 1061, 1500, 2121, 3000
};

static void ssg_env_restart(void)
{
	int att = (ssg_reg[0xD] >> 2) & 1;
	ssg_env_level   = att ? 0 : 15;
	ssg_env_dir     = att ? 1 : -1;
	ssg_env_stopped = 0;
	ssg_env_cnt     = 0;
}

static void ssg_env_step(void)
{
	if (ssg_env_stopped) return;
	int shape = ssg_reg[0xD] & 15;
	int cont = (shape >> 3) & 1, att = (shape >> 2) & 1, alt = (shape >> 1) & 1, hold = shape & 1;
	ssg_env_level += ssg_env_dir;
	if (ssg_env_level < 0 || ssg_env_level > 15) {      /* end of one 16-step ramp */
		if (!cont) {
			ssg_env_level = 0; ssg_env_stopped = 1;
		} else if (hold) {
			ssg_env_level = (alt ^ att) ? 15 : 0;       /* final level after hold */
			ssg_env_stopped = 1;
		} else if (alt) {
			ssg_env_dir = -ssg_env_dir;
			ssg_env_level = ssg_env_dir > 0 ? 0 : 15;
		} else {
			ssg_env_level = ssg_env_dir > 0 ? 0 : 15;   /* restart the ramp */
		}
	}
}

static void ssg_write(uint8_t addr, uint8_t val)
{
	static const uint8_t mask[16] = { 0xFF,0x0F,0xFF,0x0F,0xFF,0x0F,0x1F,0xFF,
	                                  0x1F,0x1F,0x1F,0xFF,0xFF,0x0F,0xFF,0xFF };
	ssg_reg[addr & 15] = val & mask[addr & 15];
	if ((addr & 15) == 0x0D) ssg_env_restart();
}

/* One 22050 Hz output sample of SSG output (unipolar levels, DC-blocked) */
static int32_t ssg_sample(void)
{
	/* tone */
	for (int c = 0; c < 3; c++) {
		uint32_t tp = ((uint32_t)(ssg_reg[c * 2 + 1] & 15) << 8) | ssg_reg[c * 2];
		if (!tp) tp = 1;
		ssg_cnt[c] += SSG_TICKS_Q16;
		uint32_t thr = tp << 16;
		while (ssg_cnt[c] >= thr) { ssg_cnt[c] -= thr; ssg_tone[c] ^= 1; }
	}
	/* noise: 17-bit LFSR, taps 0 and 3 */
	{
		uint32_t np = ssg_reg[6] & 31;
		if (!np) np = 1;
		ssg_noise_cnt += SSG_TICKS_Q16;
		uint32_t thr = (np * 2) << 16;
		while (ssg_noise_cnt >= thr) {
			ssg_noise_cnt -= thr;
			uint32_t b = (ssg_lfsr ^ (ssg_lfsr >> 3)) & 1;
			ssg_lfsr = (ssg_lfsr >> 1) | (b << 16);
			ssg_noise_bit = ssg_lfsr & 1;
		}
	}
	/* envelope */
	{
		uint32_t ep = ssg_reg[0xB] | ((uint32_t)ssg_reg[0xC] << 8);
		if (!ep) ep = 1;
		ssg_env_cnt += SSG_TICKS_Q16;
		uint32_t thr = (ep * 2) << 16;
		while (ssg_env_cnt >= thr) { ssg_env_cnt -= thr; ssg_env_step(); }
	}
	/* mix */
	int32_t mix = 0;
	for (int c = 0; c < 3; c++) {
		int tone_dis  = (ssg_reg[7] >> c) & 1;
		int noise_dis = (ssg_reg[7] >> (c + 3)) & 1;
		if ((ssg_tone[c] | tone_dis) & (ssg_noise_bit | noise_dis)) {
			int a = ssg_reg[8 + c];
			int v = (a & 0x10) ? ssg_env_level : (a & 15);
			mix += ssg_vol_tab[v];
		}
	}
	/* remove DC (constant-high channels, volume steps) */
	ssg_dc += ((mix << 8) - ssg_dc) >> 9;      /* Q8 running mean, no integer stall */
	return mix - ((ssg_dc + 128) >> 8);
}

static void ym2610_write_reg1(uint8_t addr, uint8_t val)
{
	stat_reg1_writes++;
#ifdef HOST_REG
	HOST_REG(1, addr, val);
#endif
	if (addr == 0x28) {
		stat_keyon_writes++;
#ifdef HOST_TRACE
		HOST_TRACE("keyon val=%02x\n", val);
#endif
		/* Key on/off. YM2610 FM channels are selected by 1,2 (port 1) and
		 * 5,6 (port 2); 0,3,4,7 are unused. */
		int c = val & 7;
		if (c == 0 || c == 3 || c == 4 || c == 7) return;
		int ch_idx = ((c & 3) - 1) + ((c & 4) ? 2 : 0);
		ym_chan_t *ch = &ym_ch[ch_idx];
		update_fm_channel(ch_idx);                 /* make sure rates are current */
		slot_key(ch_idx, 0, (val >> 4) & 1);
		slot_key(ch_idx, 1, (val >> 5) & 1);
		slot_key(ch_idx, 2, (val >> 6) & 1);
		slot_key(ch_idx, 3, (val >> 7) & 1);
		ch->active = (val >> 4) != 0;
		update_fm_channel(ch_idx);
		return;
	}

	if (addr == 0x1C) {
		/* ADPCM flag control: writing resets corresponding end flags
		 * in status register 1 (port $06 read). */
		if (val & 0x80) ym_adpcm_status &= ~0x80; /* reset ADPCM-B EOS */
		if (val & 0x3F) ym_adpcm_status &= ~(val & 0x3F); /* reset ADPCM-A flags */
		return;
	}

	if (addr == 0x10) {
		/* ADPCM-B control: bit 7 = start, bit 0 = reset.
		 * Reset clears EOS flag; EOS is only set when playback reaches end. */
		if (val & 0x01) ym_adpcm_status &= ~0x80; /* reset clears EOS */
		return;
	}

	if (addr == 0x24) {
		timer_a_val = (timer_a_val & 0x03) | ((uint16_t)val << 2);
		return;
	}
	if (addr == 0x25) {
		timer_a_val = (timer_a_val & 0x3FC) | (val & 0x03);
		return;
	}
	if (addr == 0x26) {
		timer_b_val = val;
		return;
	}
	if (addr == 0x27) {
		/* Register $27 — Timer/mode control (from MAME OPN):
		 * bit 0: Load/run Timer A
		 * bit 1: Load/run Timer B
		 * bit 2: IRQ enable Timer A
		 * bit 3: IRQ enable Timer B
		 * bit 4: Reset Timer A overflow flag
		 * bit 5: Reset Timer B overflow flag */
		if (val & 0x10) ym_status &= ~0x01;
		if (val & 0x20) ym_status &= ~0x02;
		/* Load/run timers */
		if ((val & 0x01) && !(timer_ctrl & 0x01)) {
			int period = TIMER_A_PERIOD(timer_a_val);
			if (period <= 0) period = 1;
			timer_a_counter = period;
		}
		if ((val & 0x02) && !(timer_ctrl & 0x02)) {
			int period = TIMER_B_PERIOD(timer_b_val);
			if (period <= 0) period = 1;
			timer_b_counter = period;
		}
		timer_ctrl = val & 0x03;
		timer_irq_ena = (val >> 2) & 0x03;
		ym_update_irq();
		return;
	}

	if (addr < 0x10) {
		ssg_write(addr, val);
		return;
	}

	if (addr >= 0x30)
		ym_write_fm_reg(0, addr, val); /* channels 0 & 1 */

	/* TODO: $00-$0F SSG, $10-$1F ADPCM-B, $20 LFO */
}

static void ym2610_write_reg2(uint8_t addr, uint8_t val)
{
	stat_reg2_writes++;
#ifdef HOST_REG
	HOST_REG(2, addr, val);
#endif

	if (addr == 0x00) {
		/* ADPCM-A dump/keyon: bit 7 = dump all, bits 0-5 = keyon mask.
		 * Set end flags for any channels that are "started" (since we
		 * don't actually play ADPCM, they instantly end). */
		if (val & 0x80) {
			ym_adpcm_status |= 0x3F; /* all ADPCM-A channels ended */
		} else {
			ym_adpcm_status |= (val & 0x3F); /* mark started channels as ended */
		}
		return;
	}

	if (addr >= 0x30)
		ym_write_fm_reg(1, addr, val); /* channels 2 & 3 */

	/* TODO: $08-$0F ADPCM-A per-channel params, $10-$1F volumes */
}

/* Z80 IRQ acknowledge callback — clear the IRQ line */
static INT32 z80_irq_callback(INT32 irqline)
{
	(void)irqline;
	return 0xFF; /* IM 1 vector (RST $38) */
}

/* ---- Public API ---- */

static void snd_cmd(uint8_t cmd);

void sound_init(const uint8_t *rom, int rom_size)
{
	memset(&z80_cpu, 0, sizeof(z80_cpu));
	memset(z80_rom, 0, sizeof(z80_rom));
	memset(z80_ram, 0, sizeof(z80_ram));
	memset(&fm_state, 0, sizeof(fm_state));
	memset(ym_ch, 0, sizeof(ym_ch));
	for (int c = 0; c < 4; c++)
		for (int o = 0; o < 4; o++) { ym_ch[c].slot[o].env = EG_MAX; ym_ch[c].slot[o].eg_state = EG_OFF; }
	eg_acc = 0; eg_cnt = 1;
	memset(ssg_reg, 0, sizeof(ssg_reg)); memset(ssg_cnt, 0, sizeof(ssg_cnt)); memset(ssg_tone, 0, sizeof(ssg_tone));
	ssg_noise_cnt = 0; ssg_lfsr = 1; ssg_noise_bit = 0; ssg_dc = 0; ssg_env_restart();

	timer_a_val = 0;
	timer_b_val = 0;
	timer_ctrl = 0;
	timer_a_counter = 0;
	timer_b_counter = 0;
	ym_status = 0;
	ym_adpcm_status = 0x00; /* no ADPCM flags on power-up */
	timer_irq_ena = 0;
	stat_p04_nvals = 0;
	port_trace_n = 0;
	memset(stat_port_rhist, 0, sizeof(stat_port_rhist));

	z80_has_rom = 0;
	if (rom && rom_size > 0) {
		int copy_size = rom_size < Z80_ROM_SIZE ? rom_size : Z80_ROM_SIZE;
		memcpy(z80_rom, rom, copy_size);
		z80_has_rom = 1;
#ifdef N64
		debugf("[SND] sound_init: M1 ROM loaded, %d bytes, first8: %02x %02x %02x %02x %02x %02x %02x %02x\n",
			copy_size,
			z80_rom[0], z80_rom[1], z80_rom[2], z80_rom[3],
			z80_rom[4], z80_rom[5], z80_rom[6], z80_rom[7]);
		/* Dump ROM around the polling loop at PC $01C0-$01EF */
		debugf("[SND] ROM@01c0:");
		for (int i = 0x01C0; i < 0x01F0 && i < copy_size; i++)
			debugf(" %02x", z80_rom[i]);
		debugf("\n");
#endif
	} else {
#ifdef N64
		debugf("[SND] sound_init: NO M1 ROM — Z80 disabled\n");
#endif
	}

	/* Initialize CZ80 */
	Cz80_Init(&z80_cpu);

	/* Set up memory map: Z80 address space is 64 KB.
	 * Fetch table must be set up correctly even if ROM is empty,
	 * because Cz80_Exec() dereferences Fetch[pc >> FETCH_SFT] + pc. */
	memset(z80_bank_off, 0, sizeof(z80_bank_off));
	nmi_enabled = 0;
	z80_map_banks();

	Cz80_Set_ReadB(&z80_cpu, z80_read);
	Cz80_Set_WriteB(&z80_cpu, z80_write);
	Cz80_Set_INPort(&z80_cpu, z80_port_read);
	Cz80_Set_OUTPort(&z80_cpu, z80_port_write);
	Cz80_Set_IRQ_Callback(&z80_cpu, z80_irq_callback);

	Cz80_Reset(&z80_cpu);

	result_latch = 0x01;
	nmi_pending = 0;
	nmi_enabled = 0;

#ifdef N64
	rsp_fm_init();
#endif
}

void sound_reset(void)
{
	Cz80_Reset(&z80_cpu);
	result_latch = 0x01;
	nmi_pending = 0;
	memset(&fm_state, 0, sizeof(fm_state));
}

void sound_update(int cycles)
{
	if (!z80_has_rom) return;  /* no sound driver loaded — skip */

	stat_update_calls++;
#ifdef SND_SELFTEST
	{
		/* Real Blazing Star startup sequence (as sent by BIOS/game), then the
		 * music command $5F. Frame numbers are sound_update() calls. */
		static const struct { uint32_t fr; uint8_t cmd; } tl[] = {
			{60,0x01},{64,0x03},{94,0x03},{98,0x03},{102,0x1b},{106,0x03},
			{136,0x07},{140,0x7f},{160,0x5f},
		};
		for (unsigned i = 0; i < sizeof(tl)/sizeof(tl[0]); i++)
			if (stat_update_calls == tl[i].fr) snd_cmd(tl[i].cmd);
	}
#endif

	/* Run Z80 in chunks so timers fire mid-frame.
	 * 4000 cycles ≈ 1 ms — enough granularity for Timer A/B. */
	while (cycles > 0) {
		int chunk = (cycles < 4000) ? cycles : 4000;
		/* The 68K command NMI is gated by the Z80's NMI-enable port. */
		if (nmi_pending && nmi_enabled) {
			Cz80_Set_IRQ(&z80_cpu, IRQ_LINE_NMI, HOLD_LINE);
			nmi_pending = 0;
			stat_nmis_fired++;
		}
		timers_advance(chunk);
		Cz80_Exec(&z80_cpu, chunk);
		cycles -= chunk;
	}
}

static void snd_cmd(uint8_t cmd)
{
	if (stat_cmd_count < sizeof(cmd_log)) cmd_log[stat_cmd_count] = cmd;
	stat_cmd_count++;
	cmd_latch = cmd;
	nmi_pending = 1;
}

void sound_command(uint8_t cmd)
{
#ifndef SND_SELFTEST
	snd_cmd(cmd);
#else
	(void)cmd;   /* self-test build: the 68K is ignored, commands are injected */
#endif
}

uint8_t sound_result(void)
{
	return result_latch;
}

/* Audio pipeline test modes for debugging.
 *   0 = normal: FM synthesis from YM2610 state via RSP
 *   1 = test tone: 440Hz sine wave directly (bypasses RSP + FM)
 *   2 = test buzz: alternating +/- 8000 (unmistakable, no table needed)
 *   3 = FM test: hardcoded channel setup, RSP renders (tests RSP path)
 */
#ifndef SOUND_DEBUG_MODE
#define SOUND_DEBUG_MODE 0
#endif

static void setup_test_fm_channel(void)
{
	/* Force channel 0 into a known-playing state bypassing Z80/reg decoder.
	 * Algorithm 7 (all 4 ops → output), all ops at low TL (loud), fixed
	 * phase increments for a steady tone. */
	struct rsp_fm_chan *rch = &fm_state.ch[0];
	memset(rch, 0, sizeof(*rch));
	for (int op = 0; op < 4; op++) {
		rch->incr[op]    = 0x8000; /* steady phase advance ~ middle pitch */
		rch->vol_out[op] = 0;      /* zero attenuation = full volume */
	}
	rch->fb_shift  = 0;
	rch->c1_mod    = 0;  /* C1 unmodulated */
	rch->m2_mod    = 0;  /* M2 unmodulated */
	rch->c2_mod    = 0;  /* C2 unmodulated */
	rch->out_flags = 0xF; /* all 4 operators → output (algo 7) */
	rch->mem_src   = 0;
	rch->enabled   = 1;
}

/* FM output gain: 4 channels of 14-bit-ish operator output are far below full
 * scale on their own; this is the (provisional) mixer level. */
#ifndef FM_GAIN
#define FM_GAIN 6
#endif

int sound_render(int16_t *buf, int max_samples)
{
	stat_frames++;
	int nsamples = max_samples;
	if (nsamples > 512) nsamples = 512;
#if SOUND_DEBUG_MODE == 3
	setup_test_fm_channel();   /* hardcoded FM channel through the real RSP path */
#endif

#if SOUND_DEBUG_MODE == 1
	/* 440Hz sine wave at 22050Hz: period = 50 samples */
	static int phase = 0;
	static const int16_t sine_lut[50] = {
		0,     2048,  4063,  6020,  7889,  9647,  11272, 12744, 14044,
		15157, 16068, 16766, 17241, 17488, 17500, 17277, 16820, 16134,
		15226, 14105, 12781, 11270, 9589,  7755,  5790,  3716,  1556,
		-667,  -2926, -5193, -7442, -9644, -11770,-13793,-15685,-17420,
		-18974,-20323,-21447,-22327,-22948,-23298,-23370,-23160,-22668,
		-21898,-20858,-19561,-18022,-16261
	};
	for (int i = 0; i < nsamples; i++) {
		int16_t s = sine_lut[phase++ % 50];
		buf[i*2+0] = s;
		buf[i*2+1] = s;
	}
	return nsamples;

#elif SOUND_DEBUG_MODE == 2
	/* Audible square wave: flip every 25 samples → 441Hz */
	static int phase = 0;
	static int16_t val = 8000;
	for (int i = 0; i < nsamples; i++) {
		if (++phase >= 25) { phase = 0; val = -val; }
		buf[i*2+0] = val;
		buf[i*2+1] = val;
	}
	return nsamples;

#else
#ifdef N64
	/* The RSP overlay holds at most 256 output samples per render, so split the
	 * frame into equal chunks (each a multiple of 2 samples = 8-byte DMA aligned).
	 * Channel phases carry over between chunks through fm_state. */
	{
		/* Chunk = envelope update interval (also bounded by the RSP's 256-sample
		 * work buffer).  Must be even so each chunk's output stays 8-byte aligned. */
		#ifndef EG_CHUNK
		#define EG_CHUNK 92
		#endif
		int per = EG_CHUNK;
		int off = 0;
		while (off < nsamples) {
			int n = nsamples - off;
			if (n > per) n = per;
			fm_state.num_samples = n;
#if SOUND_DEBUG_MODE != 3
			eg_run(n);
			eg_push();
#endif
			rsp_fm_render(&fm_state, fm_output + off);
			rspq_wait();
			/* RSP wrote the phases back to RDRAM: drop our stale cached copy. */
			data_cache_hit_invalidate(&fm_wrap, sizeof(fm_wrap));
			off += n;
		}
	}
	/* Invalidate cache so CPU sees RSP's DMA writes */
	data_cache_hit_invalidate(fm_output, (nsamples * sizeof(int32_t) + 15) & ~15);
#else
	memset(fm_output, 0, nsamples * sizeof(int32_t));
	eg_run(nsamples);      /* keep envelope state advancing on host builds */
	eg_push();
#endif

	/* Convert 32-bit mono → 16-bit stereo */
	for (int i = 0; i < nsamples; i++) {
		int32_t ssg = ssg_sample();
		if (ssg) stat_ssg_nonzero++;
		int32_t s = fm_output[i] * FM_GAIN + ssg;
		if (s > 32767) s = 32767;
		if (s < -32768) s = -32768;
		if (s) { stat_nonzero++; int32_t a = s < 0 ? -s : s; if (a > stat_peak) stat_peak = a; }
		buf[i * 2 + 0] = (int16_t)s; /* L */
		buf[i * 2 + 1] = (int16_t)s; /* R */
	}

#if defined(SND_SELFTEST) && defined(N64)
	/* Stream 150 frames of mono output (from frame 170) to the debug log as
	 * hex, so the waveform can be rebuilt and analysed off-target.  Streamed,
	 * not buffered: the PB-ROM allocation leaves very little heap to spare. */
	{
		enum { CAP_FRAMES = 150, CAP_START = 170 };
		static int cap_frames = 0;
		if (stat_update_calls >= CAP_START && cap_frames < CAP_FRAMES) {
			static const char hx[] = "0123456789abcdef";
			if (cap_frames == 0) debugf("[WAV] begin rate=22050\n");
			for (int i = 0; i < nsamples; i += 46) {
				char line[46 * 4 + 1]; int k = 0;
				for (int j = 0; j < 46 && i + j < nsamples; j++) {
					uint16_t v = (uint16_t)buf[(i + j) * 2];
					line[k++] = hx[(v >> 12) & 15]; line[k++] = hx[(v >> 8) & 15];
					line[k++] = hx[(v >> 4) & 15];  line[k++] = hx[v & 15];
				}
				line[k] = 0;
				debugf("[WAV] %s\n", line);
			}
			if (++cap_frames == CAP_FRAMES) debugf("[WAV] end\n");
		}
	}
#endif

	return nsamples;
#endif
}
