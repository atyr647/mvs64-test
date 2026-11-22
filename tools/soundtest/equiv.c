#include <stdio.h>
#include <stdint.h>
#include <string.h>
static FILE *vf;
void vrom_read(uint32_t off,uint8_t*dst,int len){ fseek(vf,off,SEEK_SET); if(fread(dst,1,len,vf)!=(size_t)len) memset(dst,0,len); }
static FILE *rf;                       /* optional: register writes with Z80 time */
static unsigned long long eq_now(void);
#define HOST_REG(p,a,v) do{ if(rf) fprintf(rf,"%llu %d %02x %02x\n",eq_now(),p,a,v); }while(0)
#include YM_SRC
static unsigned long long eq_now(void){ return z80_now(); }
static int16_t buf[1024*2];
int main(int argc,char**argv){
  FILE*f=fopen(argv[1],"rb"); static uint8_t rom[131072]; int n=fread(rom,1,sizeof rom,f); fclose(f); vf=fopen(argv[2],"rb");
  FILE*o=fopen(argv[3],"wb"); rf=argc>4?fopen(argv[4],"w"):NULL;
  sound_init(rom,n);
  static const struct{int fr;int cmd;} tl[]={{60,1},{64,3},{94,3},{98,3},{102,0x1b},{106,3},{136,7},{140,0x7f},{160,0x5f},{400,0x03},{410,0x07},{440,0xc0},{450,0xc0},{470,0xdc},{500,0xcf},{520,0x80},{900,0xb6},{1200,0x21},{1500,0x03},{1510,0x07},{1530,0x5f}};
  for(int u=1;u<=2400;u++){
    for(unsigned i=0;i<sizeof tl/sizeof tl[0];i++) if(u==tl[i].fr) snd_cmd(tl[i].cmd);
    sound_update(Z80_CLOCK/60); sound_render(buf,368); fwrite(buf,4,368,o);
  }
  fclose(o); if(rf) fclose(rf);
}
