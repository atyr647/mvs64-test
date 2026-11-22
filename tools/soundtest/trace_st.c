/* Run the real M1 driver; record YM2610 writes (sample time, port, addr, val)
 * and render our output with either SSG muted (mode 0: ADPCM A+B only)
 * or ADPCM muted (mode 1: SSG only). */
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
static FILE *vf, *tf; static long cur_sample; static unsigned long long frame_z0;
static unsigned long long host_now(void);
/* writes are stamped at their real Z80 time within the frame (368 samples per
 * frame), so ymfm sees e.g. the driver's key-off/key-on pairs ~100 us apart */
void vrom_read(uint32_t off,uint8_t*dst,int len){ fseek(vf,off,SEEK_SET); if(fread(dst,1,len,vf)!=(size_t)len) memset(dst,0,len); }
#define HOST_REG(p,a,v) do{ if(tf){ fprintf(tf,"%.3f %d %02x %02x\n",cur_sample+(host_now()-frame_z0)*368.0/(Z80_CLOCK/60),p,a,v);} }while(0)
#include YM_SRC
static unsigned long long host_now(void){ return z80_now(); }
static int16_t buf[1024*2];
int main(int argc,char**argv){
  FILE*f=fopen(argv[1],"rb"); static uint8_t rom[131072]; int n=fread(rom,1,sizeof rom,f); fclose(f);
  vf=fopen(argv[2],"rb"); int mode=atoi(argv[3]); tf=fopen(argv[4],"w"); FILE*o=fopen(argv[5],"wb");
  sound_init(rom,n);
  /* command timeline (frame, cmd): real startup, then gameplay commands seen in the autoplay log */
  static const struct{int fr;int cmd;} tl[]={ {60,1},{64,3},{94,3},{98,3},{102,0x1b},{106,3},{136,7},{140,0x7f},{160,0x5f} };

  for(int u=1;u<=500;u++){
    for(unsigned i=0;i<sizeof tl/sizeof tl[0];i++) if(u==tl[i].fr) snd_cmd(tl[i].cmd);
    cur_sample=(long)(u-1)*368; frame_z0=z80_now();
    sound_update(Z80_CLOCK/60);
    snd_host_mute = mode==0 ? 1 : 2;   /* 0: ADPCM only, 1: SSG only */
    sound_render(buf,368);
    for(int i=0;i<368;i++) fwrite(&buf[i*2],2,1,o);
  }
  fclose(o); fclose(tf);
  fprintf(stderr,"A keyons %u, B keyons %u, SSG nz %u\n",stat_adpcma_keyons,stat_adpcmb_keyons,stat_ssg_nonzero);
}
