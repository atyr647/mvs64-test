#ifndef ROMS_H
#define ROMS_H

extern uint8_t *P_ROM;
extern unsigned int rom_pc_idle_skip;

/* Z80 sound driver ROM (M1) */
extern uint8_t *M1_ROM;
extern int M1_ROM_SIZE;

/* ADPCM sample ROM (V1+V2+... concatenated, as packed by mvsmakerom).
 * vrom_read() copies len bytes at offset; anything outside the ROM reads as 0.
 * dst must be 16-byte aligned and len a multiple of 16 on N64 (DMA + cache). */
extern int VROM_SIZE;
void vrom_read(uint32_t offset, uint8_t *dst, int len);

void rom_load(const char *dir);
void rom_load_prom(const char *dir);

uint8_t* crom_get_sprite(int spritenum);
uint8_t* srom_get_sprite(int spritenum);

void srom_set_bank(int bank);  // 0 = fixed (BIOS), 1 = game

void pbrom_cache_init(void);
uint8_t* pbrom_linear(void);
uint8_t* pbrom_cache_lookup(uint32_t addr);

void rom_next_frame(void);

#endif
