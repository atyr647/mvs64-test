/* Disassemble a range of a (big-endian) 68K P-ROM image: dasm68k <p.rom> <start hex> <end hex>
 * build: gcc -I. tools/prof/dasm68k.c m68kdasm.c -o dasm68k */
#include <stdio.h>
#include <stdlib.h>
#include "m68k.h"
static unsigned char rom[0x800000]; static unsigned size;
unsigned int m68k_read_disassembler_8(unsigned int a){ return a<size?rom[a]:0; }
unsigned int m68k_read_disassembler_16(unsigned int a){ return (m68k_read_disassembler_8(a)<<8)|m68k_read_disassembler_8(a+1); }
unsigned int m68k_read_disassembler_32(unsigned int a){ return (m68k_read_disassembler_16(a)<<16)|m68k_read_disassembler_16(a+2); }
int main(int c,char**v){ FILE*f=fopen(v[1],"rb"); size=fread(rom,1,sizeof rom,f);
  unsigned pc=strtoul(v[2],0,16), end=strtoul(v[3],0,16); char buf[256];
  while(pc<end){ unsigned n=m68k_disassemble(buf,pc,M68K_CPU_TYPE_68000)&0xffff; if(!n) n=2; printf("%06x: %s\n",pc,buf); pc+=n; } }
