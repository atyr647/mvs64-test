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
static uint8_t  ym_flag_mask = 0xBF;  /* reg $1C: masks EOS flags from status reads */
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
static int32_t __attribute__((aligned(16))) fm_output[1024]; /* max samples per frame (16-byte aligned for cache ops) */

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
static uint32_t stat_ssg_nonzero;
static uint32_t prof_z80, prof_rsp, prof_mix, prof_ssg, prof_a, prof_b;
static uint32_t stat_underruns, stat_pumped;
static int32_t  pk_fm, pk_ssg, pk_a, pk_b;      /* per-source peak |sample| per stats period */
static uint32_t stat_clipped;                    /* output samples clamped */   /* AI found fully drained / samples pushed */   /* N64 ticks spent per stats period */
static uint32_t stat_adpcma_keyons, stat_adpcma_ends, stat_adpcma_nonzero;
static uint32_t stat_adpcmb_keyons, stat_adpcmb_nonzero;        /* non-zero output samples since last dump */
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
	debugf("[SND] audio: peak=%ld nonzero=%lu ssg=%lu adpcmA(keyon=%lu end=%lu nz=%lu) adpcmB(keyon=%lu nz=%lu) fm_ch_en=%d%d%d%d\n",
		(long)stat_peak, (unsigned long)stat_nonzero, (unsigned long)stat_ssg_nonzero,
		(unsigned long)stat_adpcma_keyons, (unsigned long)stat_adpcma_ends, (unsigned long)stat_adpcma_nonzero,
		(unsigned long)stat_adpcmb_keyons, (unsigned long)stat_adpcmb_nonzero,
		fm_state.ch[0].enabled, fm_state.ch[1].enabled,
		fm_state.ch[2].enabled, fm_state.ch[3].enabled);
	if (cmd_log_printed < stat_cmd_count) {
		debugf("[SND] cmds:");
		for (; cmd_log_printed < stat_cmd_count && cmd_log_printed < sizeof(cmd_log); cmd_log_printed++)
			debugf(" %02x", cmd_log[cmd_log_printed]);
		debugf("\n");
		cmd_log_printed = stat_cmd_count;
	}
	{
		static uint32_t last; uint32_t now = TICKS_READ();
		uint32_t span = last ? TICKS_DISTANCE(last, now) : TICKS_PER_SECOND;
		last = now;
		debugf("[SND] cost: z80=%.1f%% rsp_fm=%.1f%% mix(ssg+adpcm)=%.1f%% of real time | pushed=%lu underruns=%lu\n",
			prof_z80 * 100.f / span, prof_rsp * 100.f / span, prof_mix * 100.f / span,
			(unsigned long)stat_pumped, (unsigned long)stat_underruns);
		stat_pumped = 0;
		debugf("[SND] peaks: fm=%ld ssg=%ld adpcmA=%ld adpcmB=%ld clipped=%lu\n",
			(long)pk_fm, (long)pk_ssg, (long)pk_a, (long)pk_b, (unsigned long)stat_clipped);
		pk_fm = pk_ssg = pk_a = pk_b = 0; stat_clipped = 0;
		debugf("[SND] mix split: ssg=%.1f%% adpcmA=%.1f%% adpcmB=%.1f%%\n",
			prof_ssg * 100.f / span, prof_a * 100.f / span, prof_b * 100.f / span);
		prof_z80 = prof_rsp = prof_mix = prof_ssg = prof_a = prof_b = 0;
	}
	stat_peak = 0; stat_nonzero = 0; stat_ssg_nonzero = 0;
	stat_adpcma_nonzero = 0; stat_adpcmb_nonzero = 0;

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
		return ym_adpcm_status & ym_flag_mask;
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

/* Algorithm routing for the RSP: {c1_mod, m2_mod, c2_mod, out_flags, mem_src}
 *   c1_mod 1: C1 <- op1          m2_mod 1: M2 <- mem (previous sample)
 *   c2_mod 1: M2   2: op1+M2   3: mem+M2   4: op1
 *   out_flags: bit0 op1, bit1 C1, bit2 M2, bit3 C2
 *   mem_src 1: C1   2: op1+C1   3: op1   (stored for the next sample)
 * Checked against ymfm's s_algorithm_ops (O1=M1 O2=C1 O3=M2 O4=C2). */
static const uint8_t algo_table[8][5] = {
	/* 0: O1->O2->O3->O4           */ {1, 1, 1,  8, 1},
	/* 1: (O1+O2)->O3->O4          */ {0, 1, 1,  8, 2},
	/* 2: (O1+(O2->O3))->O4        */ {0, 1, 2,  8, 1},
	/* 3: ((O1->O2)+O3)->O4        */ {1, 0, 3,  8, 1},
	/* 4: (O1->O2)+(O3->O4)        */ {1, 0, 1, 10, 0},
	/* 5: O1->O2, O1->O3, O1->O4   */ {1, 1, 4, 14, 3},
	/* 6: (O1->O2)+O3+O4           */ {1, 0, 0, 14, 0},
	/* 7: O1+O2+O3+O4              */ {0, 0, 0, 15, 0},
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

	} else if (addr >= 0xB0 && addr < 0xB4) {
		/* B0-B3: feedback / algorithm */
		ch->fb   = (val >> 3) & 7;
		ch->algo = val & 7;
		update_fm_channel(ch_idx);
	}
	/* B4-B7: L/R output enable + LFO AMS/PMS (mono output, no LFO yet).
	 * These used to fall into the FB/ALGO case above, so every pan write
	 * reset the channel to algorithm 0 with no feedback. */
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
static uint32_t ssg_env_state;      /* ymfm: 0..31 ramp position, 32 = finished/held */
static int32_t  ssg_dc;

#define SSG_TICKS_Q16   743040u   /* (250000 / 22050) * 65536 */

/* Volume -> amplitude, 32 levels (ymfm, from MAME's measured table).
 * Fixed volumes v map to level v*2|1; the envelope uses all 32. */
static const int16_t ssg_amp[32] = {
	    0,   32,   78,  141,  178,  222,  262,  306,
	  369,  441,  509,  585,  701,  836,  965, 1112,
	 1334, 1595, 1853, 2146, 2576, 3081, 3576, 4135,
	 5000, 6006, 7023, 8155, 9963,11976,14132,16382
};
/* Mix level: ymfm amplitude x 0.84/0.98 (MAME's Neo Geo SSG vs FM/ADPCM routing)
 * x 0.5 (the same /2 the ADPCM paths get) = 0.429 of ymfm's SSG output.
 * Calibrated against ymfm with a register stimulus (110 measured 0.649). */
#define SSG_MIX_Q8  73

static void ssg_env_restart(void)
{
	ssg_env_state = 0;          /* writing the shape register restarts the envelope */
}

static inline int ssg_env_held(void)
{
	int shape = ssg_reg[0xD];
	return ((shape & 1) | (((shape >> 3) & 1) ^ 1));   /* hold, or no continue */
}

static void ssg_env_step(void)
{
	if (ssg_env_held() && ssg_env_state >= 32) { ssg_env_state = 32; return; }
	ssg_env_state = (ssg_env_state + 1) & 63;
}

static inline int ssg_env_volume(void)
{
	int shape = ssg_reg[0xD];
	int cont = (shape >> 3) & 1, att = (shape >> 2) & 1, alt = (shape >> 1) & 1;
	if (ssg_env_held() && ssg_env_state >= 32) {
		ssg_env_state = 32;
		return ((att ^ alt) & cont) ? 31 : 0;
	}
	if (alt) att ^= (ssg_env_state >> 5) & 1;
	return (int)(ssg_env_state & 31) ^ (att ? 0 : 31);
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
		uint32_t thr = ep << 16;
		while (ssg_env_cnt >= thr) { ssg_env_cnt -= thr; ssg_env_step(); }
	}
	/* mix */
	int32_t mix = 0;
	int envv = -1;
	for (int c = 0; c < 3; c++) {
		int tone_dis  = (ssg_reg[7] >> c) & 1;
		int noise_dis = (ssg_reg[7] >> (c + 3)) & 1;
		if ((ssg_tone[c] | tone_dis) & (ssg_noise_bit | noise_dis)) {
			int a = ssg_reg[8 + c], v;
			if (a & 0x10) { if (envv < 0) envv = ssg_env_volume(); v = envv; }
			else          { v = (a & 15) * 2; if (v) v |= 1; }
			mix += ssg_amp[v];
		}
	}
	mix = (mix * SSG_MIX_Q8) >> 8;
	/* remove DC (constant-high channels, volume steps) */
	ssg_dc += ((mix << 8) - ssg_dc) >> 9;      /* Q8 running mean, no integer stall */
	return mix - ((ssg_dc + 128) >> 8);
}

#ifndef ADPCMA_GAIN_SHIFT
#define ADPCMA_GAIN_SHIFT 1
#endif

/* ---- ADPCM-A: 6 channels of 4-bit ADPCM from the V ROM ----
 *
 * 12-bit accumulator, 49-entry step table, 18518.5 Hz native rate
 * (8 MHz / 432), resampled (linear) to the 22050 Hz output.  Samples are
 * streamed from the cart in 256-byte chunks per channel. */

extern void vrom_read(uint32_t offset, uint8_t *dst, int len);

#define ADPCMA_CHUNK       256
#define ADPCMA_RATE_Q16    55042u        /* 18518.5 / 22050 * 65536 */

static const int16_t adpcma_steps[49] = {
	16, 17, 19, 21, 23, 25, 28, 31, 34, 37, 41, 45, 50, 55, 60, 66, 73, 80, 88, 97, 107, 118, 130, 143, 157, 173, 190, 209, 230, 253, 279, 307, 337, 371, 408, 449, 494, 544, 598, 658, 724, 796, 876, 963, 1060, 1166, 1282, 1411, 1552
};
static const int8_t adpcma_step_inc[8] = { -1, -1, -1, -1, 2, 5, 7, 9 };

typedef struct {
	uint8_t  on;
	uint8_t  hi;          /* next nibble is the high one */
	uint8_t  level;       /* 0..31 */
	uint8_t  pan;         /* bits 7,6 = L,R */
	uint16_t start_reg, end_reg;   /* address registers in 256-byte units */
	uint32_t pos;         /* byte address of the nibble pair being decoded */
	uint32_t end;         /* first byte address AFTER the sample (end register + 1) << 8 */
	uint8_t  byte;        /* byte currently being decoded */
	uint32_t chunk_base;  /* ROM offset of the chunk in adpcma_buf, or ~0 */
	int32_t  acc, step;   /* acc: 12-bit unsigned, wraps (&0xFFF); sign-extended on output */
	int32_t  s0, s1;      /* last two decoded samples, for interpolation */
	uint32_t frac;
} adpcma_t;

static adpcma_t adpcma[6];
static uint8_t  adpcma_total = 0x3F;      /* total level register, 0..63 */
static uint8_t  adpcma_buf[6][ADPCMA_CHUNK] __attribute__((aligned(16)));

static inline int adpcma_fetch(int ch, uint32_t addr)
{
	adpcma_t *a = &adpcma[ch];
	uint32_t base = addr & ~(uint32_t)(ADPCMA_CHUNK - 1);
	if (a->chunk_base != base) {
		vrom_read(base, adpcma_buf[ch], ADPCMA_CHUNK);
		a->chunk_base = base;
	}
	return adpcma_buf[ch][addr & (ADPCMA_CHUNK - 1)];
}

/* Decode the next native-rate sample of channel ch; returns 0 when finished.
 * Matches ymfm: the end address is inclusive and only its low 20 bits are
 * compared, checked just before fetching each new byte. */
static int adpcma_decode(int ch)
{
	adpcma_t *a = &adpcma[ch];
	int nib;
	if (a->hi) {
		if (((a->pos ^ a->end) & 0xFFFFF) == 0) return 0;
		a->byte = (uint8_t)adpcma_fetch(ch, a->pos++);
		nib = a->byte >> 4;
		a->hi = 0;
	} else {
		nib = a->byte & 15;
		a->hi = 1;
	}

	int d = ((2 * (nib & 7) + 1) * adpcma_steps[a->step]) / 8;
	if (nib & 8) d = -d;
	a->acc = (a->acc + d) & 0xFFF;                    /* 12-bit wrap */
	a->step += adpcma_step_inc[nib & 7];
	if (a->step < 0) a->step = 0; else if (a->step > 48) a->step = 48;
	return 1;
}

static void adpcma_keyon(int ch)
{
	adpcma_t *a = &adpcma[ch];
	a->pos  = (uint32_t)a->start_reg << 8;
	a->end  = ((uint32_t)a->end_reg + 1) << 8;
	a->hi = 1; a->acc = 0; a->step = 0;
	a->s0 = a->s1 = 0; a->frac = 0;
	a->chunk_base = ~(uint32_t)0;
	a->on = 1;
	stat_adpcma_keyons++;
#ifdef N64
	if (stat_adpcma_keyons <= 40)
		debugf("[ADPCMA] keyon ch%d start=%04x end=%04x (%lu bytes) lvl=%02x pan=%02x tl=%02x\n",
			ch, a->start_reg, a->end_reg, (unsigned long)(a->end - a->pos), a->level, a->pan, adpcma_total);
#endif
}

static void adpcma_write(uint8_t addr, uint8_t val)
{
	if (addr == 0x00) {                               /* key on / dump */
		for (int c = 0; c < 6; c++)
			if (val & (1 << c)) {
				if (val & 0x80) adpcma[c].on = 0;
				else            adpcma_keyon(c);
			}
	} else if (addr == 0x01) {
		adpcma_total = val & 0x3F;
	} else if (addr >= 0x08 && addr <= 0x0D) {
		adpcma[addr - 0x08].level = val & 0x1F;
		adpcma[addr - 0x08].pan   = val & 0xC0;
	} else if (addr >= 0x10 && addr <= 0x2D) {
		int c = addr & 7;
		if (c >= 6) return;
		switch (addr >> 3) {                          /* 2:$10 3:$18 4:$20 5:$28 */
		case 2: adpcma[c].start_reg = (adpcma[c].start_reg & 0xFF00) | val; break;
		case 3: adpcma[c].start_reg = (adpcma[c].start_reg & 0x00FF) | ((uint16_t)val << 8); break;
		case 4: adpcma[c].end_reg   = (adpcma[c].end_reg   & 0xFF00) | val; break;
		case 5: adpcma[c].end_reg   = (adpcma[c].end_reg   & 0x00FF) | ((uint16_t)val << 8); break;
		}
	}
}

/* Add n output samples of all ADPCM-A channels into acc */
static void adpcma_render(int32_t *out, int n)
{
	static int32_t acc[1024];
	memset(acc, 0, n * sizeof(int32_t));
	int any = 0;
	for (int c = 0; c < 6; c++) {
		adpcma_t *a = &adpcma[c];
		if (!a->on) continue;
		int vol = ((a->level ^ 0x1F) & 0x1F) + ((adpcma_total ^ 0x3F) & 0x3F);
		int audible = vol < 63 && (a->pan & 0xC0);
		int mul = 15 - (vol & 7), sh = 5 + (vol >> 3);
		for (int i = 0; i < n; i++) {
			a->frac += ADPCMA_RATE_Q16;
			while (a->frac >= 65536) {
				a->frac -= 65536;
				a->s0 = a->s1;
				if (!adpcma_decode(c)) {                    /* reached the end address */
					a->on = 0;
					ym_adpcm_status |= (uint8_t)(1 << c);  /* end-of-sample flag */
					stat_adpcma_ends++;
					break;
				}
				a->s1 = (a->acc ^ 0x800) - 0x800;           /* sign-extend 12 bits */
			}
			if (!a->on) break;
			if (!audible) continue;
			/* |s1-s0| < 4096 and frac < 65536: the product fits in 32 bits */
			int32_t v = a->s0 + (((a->s1 - a->s0) * (int32_t)a->frac) >> 16);
			int32_t smp = (((v << 4) * mul) >> sh) & ~3;
			acc[i] += smp;
			any |= smp;
		}
	}
	if (any) {
		stat_adpcma_nonzero += n;
		for (int i = 0; i < n; i++) { int32_t v = acc[i] >> ADPCMA_GAIN_SHIFT; int32_t m = v < 0 ? -v : v; if (m > pk_a) pk_a = m; out[i] += v; }   /* gain on the channel sum */
	}
}


/* ---- ADPCM-B: single delta-T ADPCM channel (ported from ymfm) ----
 *
 * 16-bit accumulator, adaptive step, playback rate = delta_n/65536 * 55.5 kHz
 * (8 MHz / 144).  Reads from the same V ROM as ADPCM-A. */

#define ADPCMB_STEP_MIN   127
#define ADPCMB_STEP_MAX   24576
#define ADPCMB_ST_EOS     0x01
#define ADPCMB_ST_PLAYING 0x04
#define ADPCMB_CLK_Q16    165119u         /* 55555.6 / 22050 * 65536 B-clocks per output sample */

static uint8_t  adpcmb_reg[16];           /* regs $10..$1B, index = addr - $10 */
static uint8_t  adpcmb_status;
static uint8_t  adpcmb_nibble, adpcmb_byte;
static uint32_t adpcmb_pos16, adpcmb_addr, adpcmb_clk_acc;
static int32_t  adpcmb_acc, adpcmb_prev, adpcmb_step;
static uint32_t adpcmb_chunk_base = ~0u;
static uint8_t  adpcmb_buf[ADPCMA_CHUNK] __attribute__((aligned(16)));

static inline int adpcmb_fetch(uint32_t addr)
{
	uint32_t base = addr & ~(uint32_t)(ADPCMA_CHUNK - 1);
	if (adpcmb_chunk_base != base) {
		vrom_read(base, adpcmb_buf, ADPCMA_CHUNK);
		adpcmb_chunk_base = base;
	}
	return adpcmb_buf[addr & (ADPCMA_CHUNK - 1)];
}

static inline uint32_t adpcmb_start(void)   { return adpcmb_reg[2] | ((uint32_t)adpcmb_reg[3] << 8); }
static inline uint32_t adpcmb_end(void)     { return adpcmb_reg[4] | ((uint32_t)adpcmb_reg[5] << 8); }
static inline uint32_t adpcmb_delta_n(void) { return adpcmb_reg[9] | ((uint32_t)adpcmb_reg[10] << 8); }

static void adpcmb_load_start(void)
{
	adpcmb_status = (adpcmb_status & ~ADPCMB_ST_EOS) | ADPCMB_ST_PLAYING;
	adpcmb_addr = adpcmb_start() << 8;
	adpcmb_nibble = 0; adpcmb_byte = 0; adpcmb_pos16 = 0;
	adpcmb_acc = adpcmb_prev = 0; adpcmb_step = ADPCMB_STEP_MIN;
	adpcmb_chunk_base = ~0u;
}

static void adpcmb_reset(void)
{
	adpcmb_status = 0;
	adpcmb_nibble = 0; adpcmb_byte = 0; adpcmb_pos16 = 0; adpcmb_addr = 0;
	adpcmb_acc = adpcmb_prev = 0; adpcmb_step = ADPCMB_STEP_MIN;
}

/* Decode the next ADPCM-B nibble (what one clock does when the 16-bit position
 * counter overflows).  Returns 0 when the sample has ended (EOS raised). */
static int adpcmb_nibble_step(void)
{
	if (adpcmb_nibble == 0)
		adpcmb_byte = (uint8_t)adpcmb_fetch(adpcmb_addr);
	int data = (uint8_t)(adpcmb_byte << (4 * adpcmb_nibble)) >> 4;
	adpcmb_nibble ^= 1;

	if (adpcmb_nibble == 0) {                          /* finished a byte */
		if (adpcmb_addr == (((adpcmb_end() + 1) << 8) - 1)) {
			if (adpcmb_reg[0] & 0x10) {                /* repeat */
				adpcmb_load_start();
			} else {                                   /* done: raise EOS */
				adpcmb_acc = adpcmb_prev = 0;
				adpcmb_status = (adpcmb_status & ~ADPCMB_ST_PLAYING) | ADPCMB_ST_EOS;
				return 0;
			}
		} else {
			adpcmb_addr = (adpcmb_addr + 1) & 0xFFFFFF;
		}
	}

	adpcmb_prev = adpcmb_acc;
	int d = (2 * (data & 7) + 1) * adpcmb_step / 8;
	if (data & 8) d = -d;
	adpcmb_acc += d;
	if (adpcmb_acc > 32767) adpcmb_acc = 32767; else if (adpcmb_acc < -32768) adpcmb_acc = -32768;
	static const uint8_t scale[8] = { 57, 57, 57, 57, 77, 102, 128, 153 };
	adpcmb_step = (adpcmb_step * scale[data & 7]) / 64;
	if (adpcmb_step < ADPCMB_STEP_MIN) adpcmb_step = ADPCMB_STEP_MIN;
	else if (adpcmb_step > ADPCMB_STEP_MAX) adpcmb_step = ADPCMB_STEP_MAX;
	return 1;
}

static void adpcmb_write(uint8_t regnum, uint8_t val)
{
	if (regnum == 0) val = (val | 0x20) & ~0x40;     /* YM2610: external mode forced on, no record */
	adpcmb_reg[regnum & 15] = val;
	if (regnum == 0) {
		if (val & 0x80) {
			adpcmb_load_start();
			stat_adpcmb_keyons++;
#ifdef N64
			if (stat_adpcmb_keyons <= 20)
				debugf("[ADPCMB] start=%04x end=%04x dn=%04x lvl=%02x pan=%02x rep=%d\n",
					(unsigned)adpcmb_start(), (unsigned)adpcmb_end(), (unsigned)adpcmb_delta_n(),
					adpcmb_reg[0xB], adpcmb_reg[1] & 0xC0, (val >> 4) & 1);
#endif
		} else {
			adpcmb_status &= ~ADPCMB_ST_EOS;
		}
		if (val & 0x01) adpcmb_reset();
	}
}

/* Add n output samples of ADPCM-B into acc */
static void adpcmb_render(int32_t *acc, int n)
{
	if (adpcmb_status & ADPCMB_ST_PLAYING) {
		int audible = (adpcmb_reg[1] & 0xC0) != 0;
		int32_t lvl = adpcmb_reg[0xB];
		uint32_t dn = adpcmb_delta_n();
		for (int i = 0; i < n; i++) {
			/* the 2-3 chip clocks of this output sample, batched: each adds
			 * delta_n to the 16-bit position, and each overflow decodes one nibble */
			adpcmb_clk_acc += ADPCMB_CLK_Q16;
			uint32_t k = adpcmb_clk_acc >> 16;
			adpcmb_clk_acc &= 0xFFFF;
			if (!(adpcmb_reg[0] & 0x80)) { adpcmb_status &= ~ADPCMB_ST_PLAYING; break; }
			uint32_t total = adpcmb_pos16 + k * dn;
			adpcmb_pos16 = total & 0xFFFF;
			int ended = 0;
			for (uint32_t m = total >> 16; m; m--)
				if (!adpcmb_nibble_step()) { ended = 1; break; }
			if (ended) break;
			if (!audible) continue;
			/* linear interpolation; (acc-prev) < 2^17 and pos>>1 < 2^15 fit in 32 bits */
			int32_t r = adpcmb_prev + (((adpcmb_acc - adpcmb_prev) * (int32_t)(adpcmb_pos16 >> 1)) >> 15);
			r = ((r * lvl) >> 9) >> ADPCMA_GAIN_SHIFT;
			{ int32_t m2 = r < 0 ? -r : r; if (m2 > pk_b) pk_b = m2; }
			acc[i] += r;
		}
		if (audible) stat_adpcmb_nonzero += n;
	} else {
		/* idle: keep the 55.5 kHz clock phase running so key-on timing is exact */
		adpcmb_clk_acc = (uint32_t)(((uint64_t)adpcmb_clk_acc + (uint64_t)n * ADPCMB_CLK_Q16) & 0xFFFF);
	}
	/* mirror the live EOS state into the visible flags (bit 6 tracks, bit 7 is seen) */
	uint8_t live = (adpcmb_status & ADPCMB_ST_EOS) ? 0x40 : 0x00;
	if ((live ^ ym_adpcm_status) & 0x40)
		ym_adpcm_status = (ym_adpcm_status & ~0xC0) | live | (live << 1);
}

static void ssg_render(int32_t *acc, int n)
{
	int silent = 1;
	for (int c = 0; c < 3; c++) if (ssg_reg[8 + c] & 0x1F) silent = 0;
	if (silent && ssg_dc == 0) {
		/* Nothing audible and DC settled: just advance the tone, noise and
		 * envelope counters by n samples so they stay exactly in phase. */
		uint64_t adv = (uint64_t)n * SSG_TICKS_Q16;
		for (int c = 0; c < 3; c++) {
			uint32_t tp = ((uint32_t)(ssg_reg[c * 2 + 1] & 15) << 8) | ssg_reg[c * 2];
			if (!tp) tp = 1;
			uint64_t thr = (uint64_t)tp << 16, t = ssg_cnt[c] + adv;
			ssg_tone[c] ^= (uint8_t)((t / thr) & 1);
			ssg_cnt[c] = (uint32_t)(t % thr);
		}
		{
			uint32_t np = ssg_reg[6] & 31; if (!np) np = 1;
			uint64_t thr = (uint64_t)(np * 2) << 16, t = ssg_noise_cnt + adv;
			for (uint64_t k = t / thr; k; k--) {
				uint32_t b = (ssg_lfsr ^ (ssg_lfsr >> 3)) & 1;
				ssg_lfsr = (ssg_lfsr >> 1) | (b << 16);
			}
			ssg_noise_bit = ssg_lfsr & 1;
			ssg_noise_cnt = (uint32_t)(t % thr);
		}
		{
			uint32_t ep = ssg_reg[0xB] | ((uint32_t)ssg_reg[0xC] << 8); if (!ep) ep = 1;
			uint64_t thr = (uint64_t)ep << 16, t = ssg_env_cnt + adv;
			uint64_t k = t / thr;
			if (ssg_env_held()) ssg_env_state = (ssg_env_state + k >= 32) ? 32 : ssg_env_state + (uint32_t)k;
			else                ssg_env_state = (uint32_t)((ssg_env_state + k) & 63);
			ssg_env_cnt = (uint32_t)(t % thr);
		}
		return;
	}
	for (int i = 0; i < n; i++) { int32_t v = ssg_sample(); if (v) stat_ssg_nonzero++; int32_t m = v < 0 ? -v : v; if (m > pk_ssg) pk_ssg = m; acc[i] += v; }
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
		/* EOS flag control: 1 bits reset the flag and mask it from reads,
		 * 0 bits unmask (ymfm/YM2610 behaviour) */
		ym_flag_mask = (uint8_t)(~val & 0xBF);
		ym_adpcm_status &= (uint8_t)~(val & 0xBF);
		return;
	}

	if (addr >= 0x10 && addr <= 0x1B) {
		adpcmb_write(addr - 0x10, val);
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

	if (addr < 0x30) {
		adpcma_write(addr, val);
		return;
	}

	if (addr >= 0x30)
		ym_write_fm_reg(1, addr, val); /* channels 2 & 3 */

}

/* Z80 IRQ acknowledge callback — clear the IRQ line */
static INT32 z80_irq_callback(INT32 irqline)
{
	(void)irqline;
	return 0xFF; /* IM 1 vector (RST $38) */
}

/* ---- Public API ---- */

static void snd_cmd(uint8_t cmd);
#ifdef SND_SELFTEST
static int selftest_cap_armed;
#endif

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
	memset(adpcma, 0, sizeof(adpcma)); adpcma_total = 0x3F;
	memset(adpcmb_reg, 0, sizeof(adpcmb_reg)); adpcmb_reset(); adpcmb_clk_acc = 0; ym_flag_mask = 0xBF;
	for (int c = 0; c < 6; c++) { adpcma[c].level = 0x1F; adpcma[c].pan = 0xC0; }
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
#ifdef SND_DISABLE
	rom_size = 0;   /* A/B performance testing: run with the sound CPU off */
#endif
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

/* Z80 cycles already executed in the current frame (see sound_sync) */
static int z80_frame_done;
/* Z80 cycles planned for the current emulated frame.  On N64 this follows the
 * real-time audio demand (see sound_frame_begin/sound_pump), not 1/60 s. */
static int z80_frame_target = Z80_CLOCK / 60;
static int32_t z80_carry;        /* cycles run beyond what rendered audio paid for */
static int snd_plan = 368;       /* expected samples per emulated frame (smoothed) */

static inline int samples_to_z80(int n)
{
	return (int)(((int64_t)n * Z80_CLOCK) / SND_OUTPUT_RATE);
}

/* Run the Z80 (+ YM2610 timers) for the given number of Z80 cycles, in chunks
 * so timers fire mid-frame and NMIs are delivered promptly.
 * 4000 cycles ~ 1 ms is enough granularity for Timer A/B. */
/* Idle detection: single-step a few instructions; if the CPU comes back to an
 * identical register state (same PC, same registers, ignoring R) without any
 * RAM write or port access in between, it is in a loop whose outcome cannot
 * change until an interrupt arrives.  Interrupts are only raised at chunk
 * boundaries, so the rest of the chunk can be skipped.  Delay loops (DJNZ...)
 * change a register every pass, so they are never mistaken for idle. */
typedef struct { uint16_t bc, de, hl, fa, ix, iy, sp, bc2, de2, hl2, fa2, iff; uint32_t pc; } z80_snap_t;

static inline void z80_snap(z80_snap_t *s)
{
	s->bc = z80_cpu.BC.W; s->de = z80_cpu.DE.W; s->hl = z80_cpu.HL.W; s->fa = z80_cpu.FA.W;
	s->ix = z80_cpu.IX.W; s->iy = z80_cpu.IY.W; s->sp = z80_cpu.SP.W;
	s->bc2 = z80_cpu.BC2.W; s->de2 = z80_cpu.DE2.W; s->hl2 = z80_cpu.HL2.W; s->fa2 = z80_cpu.FA2.W;
	s->iff = z80_cpu.IFF.W;
	s->pc = (uint32_t)(z80_cpu.PC - z80_cpu.BasePC);
}

static uint32_t stat_z80_idle_skipped;   /* cycles fast-forwarded */

/* Returns cycles consumed (executed + skipped); never more than budget+one instruction. */
static int z80_exec_idle_aware(int budget)
{
#ifndef Z80_NO_IDLE_SKIP
	z80_snap_t s0, s1;
	z80_snap(&s0);
	uint32_t fx0 = stat_ram_writes + stat_port_writes + stat_port_reads;
	int used = 0;
	for (int i = 0; i < 24 && used < budget; i++) {
		used += Cz80_Exec(&z80_cpu, 1);
		if (stat_ram_writes + stat_port_writes + stat_port_reads != fx0) break;   /* side effects: busy */
		z80_snap(&s1);
		if (memcmp(&s0, &s1, sizeof(s0)) == 0) {
			int rest = budget - used;
			if (rest > 0) { stat_z80_idle_skipped += rest; used += rest; }
			return used;
		}
	}
	if (budget - used > 0) used += Cz80_Exec(&z80_cpu, budget - used);
	return used;
#else
	return Cz80_Exec(&z80_cpu, budget);
#endif
}

static void z80_run(int cycles)
{
#ifdef N64
	uint32_t t0 = TICKS_READ();
#endif
	while (cycles > 0) {
		int chunk = (cycles < 4000) ? cycles : 4000;
		/* The 68K command NMI is gated by the Z80's NMI-enable port. */
		if (nmi_pending && nmi_enabled) {
			Cz80_Set_IRQ(&z80_cpu, IRQ_LINE_NMI, HOLD_LINE);
			nmi_pending = 0;
			stat_nmis_fired++;
		}
		timers_advance(chunk);
		z80_exec_idle_aware(chunk);
		cycles -= chunk;
	}
#ifdef N64
	prof_z80 += TICKS_DISTANCE(t0, TICKS_READ());
#endif
}

/* Bring the Z80 up to the 68K's position in the frame.  Called by the 68K
 * sound port accessors so that commands sent back-to-back within one frame
 * are each seen by the Z80 (as on hardware) instead of overwriting the
 * single command latch, and so replies are fresh when the 68K polls them.
 * master_clock_in_frame is in 24 MHz units; the Z80 runs at master / 6. */
void sound_sync(int64_t master_clock_in_frame)
{
	if (!z80_has_rom) return;
	/* FRAME_CLOCK = 24 MHz / 60 = 400000 master clocks per emulated frame */
	int target = (int)((master_clock_in_frame * z80_frame_target) / 400000);
	if (target > z80_frame_target) target = z80_frame_target;
	int delta = target - z80_frame_done;
	if (delta > 0) {
		z80_run(delta);
		z80_frame_done += delta;
	}
}

/* Once per emulated frame: frame counter + (self-test) command injection */
static void snd_frame_tick(void)
{
	stat_update_calls++;
#ifdef SND_SELFTEST
	{
		/* Real Blazing Star startup sequence (as sent by BIOS/game), then the
		 * music command $5F etc. Frame numbers are emulated frames. */
		static const struct { uint32_t fr; uint8_t cmd; } tl[] = {
			{60,0x01},{64,0x03},{94,0x03},{98,0x03},{102,0x1b},{106,0x03},
			{136,0x07},{140,0x7f},{160,0x5f},{400,0x03},{410,0x07},{440,0xc0},{500,0xdc},{560,0xcf},{620,0x80},
		};
		for (unsigned i = 0; i < sizeof(tl)/sizeof(tl[0]); i++)
			if (stat_update_calls == tl[i].fr) {
				snd_cmd(tl[i].cmd);
				if (tl[i].cmd == 0x5f) selftest_cap_armed = 1;   /* capture from the next render */
			}
	}
#endif
}

/* Fixed-rate path (host builds / test harnesses): one frame = `cycles`. */
void sound_update(int cycles)
{
	if (!z80_has_rom) return;  /* no sound driver loaded — skip */
	snd_frame_tick();
	/* Finish the frame: run whatever the 68K-side syncs did not already cover */
	int remaining = cycles - z80_frame_done;
	if (remaining > 0) z80_run(remaining);
	z80_frame_done = 0;
}

/* ---- Real-time path (N64) ----
 *
 * Sound runs on the audio clock, not on emulated frames: when the emulator is
 * slower than 60 FPS the 68K (and so the game) runs slow, but the Z80, the
 * YM2610 timers and the sample generators still advance exactly as much as the
 * audio interface plays.  Music keeps its tempo and pitch, and the AI never
 * starves; only the arrival of 68K commands follows the game's speed.
 *
 *   sound_frame_begin()  before emu_run_frame(): plan this frame's Z80 budget
 *   sound_sync()         from the 68K sound port, inside the frame
 *   sound_pump()         after the frame: fill every free AI buffer       */

void sound_frame_begin(void)
{
	if (!z80_has_rom) return;
	snd_frame_tick();
	int t = samples_to_z80(snd_plan) - z80_carry;
	z80_frame_target = t > 0 ? t : 0;
	z80_frame_done = 0;
}

#ifdef N64
/* Render one block of n samples into dst, first advancing the Z80 to the
 * point in time the block ends at.  `rendered` = samples already rendered
 * this frame. */
static void pump_block(int16_t *dst, int n, int rendered)
{
	/* z80_carry = cycles the Z80 had already run ahead of rendered audio when
	 * this frame began, so this frame only needs to reach (time - carry). */
	int need = samples_to_z80(rendered + n) - z80_carry;
	if (need > z80_frame_done) {
		z80_run(need - z80_frame_done);
		z80_frame_done = need;
	}
	sound_render(dst, n);
}

int sound_pump(void)
{
	if (!z80_has_rom) return 0;
	int rendered = 0;
	const int limit = 3 * audio_get_buffer_length();   /* bound the catch-up after a stall */
	/* libdragon keeps 4 buffers, 2 of them queued in the AI.  Being able to
	 * fill 3 or more at once means the AI ran out of data: an underrun. */

	while (rendered < limit && audio_can_write()) {
		int n = audio_get_buffer_length();
		int16_t *dst = (int16_t *)audio_write_begin();
		pump_block(dst, n, rendered);
		audio_write_end();
		rendered += n;
	}
	if (rendered >= 3 * audio_get_buffer_length()) stat_underruns++;
	stat_pumped += rendered;
	/* Whatever the Z80 ran beyond the audio rendered so far carries forward
	 * (it ran ahead under sound_sync while the AI was full). */
	z80_carry += z80_frame_done - samples_to_z80(rendered);
	/* Smoothed expectation of samples per frame, for next frame's Z80 budget */
	snd_plan = (snd_plan * 7 + rendered + 4) / 8;
	if (snd_plan < 64) snd_plan = 64;
	return rendered;
}
#endif

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

/* FM mix level.  The RSP sums carrier outputs at the chip's 13-bit operator
 * scale; the YM2610 outputs each channel >>1 (ymfm, "OPNB is 13-bit"), and
 * every source here gets the same x0.5 as the ADPCM paths: x0.25 overall. */
#ifndef FM_MIX_SHIFT
#define FM_MIX_SHIFT 2
#endif

int sound_render(int16_t *buf, int max_samples)
{
	stat_frames++;
	int nsamples = max_samples;
	if (nsamples > 1024) nsamples = 1024;
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
	uint32_t t_rsp = TICKS_READ();
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
	prof_rsp += TICKS_DISTANCE(t_rsp, TICKS_READ());
	uint32_t t_mix = TICKS_READ();
#else
	memset(fm_output, 0, nsamples * sizeof(int32_t));
	eg_run(nsamples);      /* keep envelope state advancing on host builds */
	eg_push();
#endif

	/* Mix: FM (from the RSP) + SSG + ADPCM-A + ADPCM-B, each rendered as a block */
	{
		static int32_t mix[1024];
		for (int i = 0; i < nsamples; i++) { mix[i] = fm_output[i] >> FM_MIX_SHIFT; int32_t m = mix[i] < 0 ? -mix[i] : mix[i]; if (m > pk_fm) pk_fm = m; }
#ifdef N64
		uint32_t q0 = TICKS_READ();
		ssg_render(mix, nsamples);
		uint32_t q1 = TICKS_READ();
		adpcma_render(mix, nsamples);
		uint32_t q2 = TICKS_READ();
		adpcmb_render(mix, nsamples);
		uint32_t q3 = TICKS_READ();
		prof_ssg += TICKS_DISTANCE(q0, q1); prof_a += TICKS_DISTANCE(q1, q2); prof_b += TICKS_DISTANCE(q2, q3);
#else
		ssg_render(mix, nsamples);
		adpcma_render(mix, nsamples);
		adpcmb_render(mix, nsamples);
#endif
		for (int i = 0; i < nsamples; i++) {
			int32_t s = mix[i];
			if (s > 32767) { s = 32767; stat_clipped++; }
			if (s < -32768) { s = -32768; stat_clipped++; }
			if (s) { stat_nonzero++; int32_t a = s < 0 ? -s : s; if (a > stat_peak) stat_peak = a; }
			buf[i * 2 + 0] = (int16_t)s; /* L */
			buf[i * 2 + 1] = (int16_t)s; /* R */
		}
	}
#ifdef N64
	prof_mix += TICKS_DISTANCE(t_mix, TICKS_READ());
#endif

#if defined(SND_SELFTEST) && defined(N64)
	/* Stream 150 frames of mono output (from frame 170) to the debug log as
	 * hex, so the waveform can be rebuilt and analysed off-target.  Streamed,
	 * not buffered: the PB-ROM allocation leaves very little heap to spare. */
	{
		enum { CAP_SAMPLES = 88200 };   /* 4 s, starting with the first render after cmd $5F */
		static int cap_n = 0;
		if (selftest_cap_armed && cap_n < CAP_SAMPLES) {
			static const char hx[] = "0123456789abcdef";
			if (cap_n == 0) debugf("[WAV] begin rate=22050\n");
			for (int i = 0; i < nsamples && cap_n < CAP_SAMPLES; i += 46) {
				char line[46 * 4 + 1]; int k = 0;
				for (int j = 0; j < 46 && i + j < nsamples && cap_n < CAP_SAMPLES; j++, cap_n++) {
					uint16_t v = (uint16_t)buf[(i + j) * 2];
					line[k++] = hx[(v >> 12) & 15]; line[k++] = hx[(v >> 8) & 15];
					line[k++] = hx[(v >> 4) & 15];  line[k++] = hx[v & 15];
				}
				line[k] = 0;
				debugf("[WAV] %s\n", line);
			}
			if (cap_n >= CAP_SAMPLES) debugf("[WAV] end\n");
		}
	}
#endif

	return nsamples;
#endif
}
