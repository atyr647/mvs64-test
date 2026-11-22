/* Host C model of rsp_fm.S (instruction-for-instruction semantics), driven by
 * the real CPU-side FM state (update_fm_channel + envelope) for the M1 driver.
 * Renders at 22050 Hz (what the RSP does) and at the native 55555 Hz. */
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
static FILE *vf;
void vrom_read(uint32_t off,uint8_t*dst,int len){ fseek(vf,off,SEEK_SET); if(fread(dst,1,len,vf)!=(size_t)len) memset(dst,0,len); }
#include YM_SRC
#include "rspfm_model.h"

int main(int argc,char**argv){
  FILE*f=fopen(argv[1],"rb"); static uint8_t rom[131072]; int n=fread(rom,1,sizeof rom,f); fclose(f); vf=fopen(argv[2],"rb");
  double rate=atof(argv[3]); FILE*o=fopen(argv[4],"wb"); FILE*oR=argc>5?fopen(argv[5],"wb"):NULL;
  tables(); sound_init(rom,n);
  static const struct{int fr;int cmd;} tl[]={ {60,1},{64,3},{94,3},{98,3},{102,0x1b},{106,3},{136,7},{140,0x7f},{160,0x5f} };
  /* optional FMMASK: ymfm-style channel mask (0x02,0x04,0x10,0x20 = our ch0..3) */
  int mask=getenv("FMMASK")?(int)strtol(getenv("FMMASK"),0,16):0x36; static const int bit[4]={0x02,0x04,0x10,0x20};
  double frac=0;
  for(int u=1;u<=500;u++){
    for(unsigned i=0;i<sizeof tl/sizeof tl[0];i++) if(u==tl[i].fr) snd_cmd(tl[i].cmd);
    sound_update(Z80_CLOCK/60);
    /* as sound_render: split at queued register writes, EG chunks <= 92 */
    for(int done=0,seg;done<368;done+=seg,rclk_pos+=seg){
      seg=ymq_step(368-done<92?368-done:92);
      eg_run(seg); eg_push();
      frac += seg*rate/22050.0; int m=(int)frac; frac-=m;
      /* render in place on fm_state, like the RSP's state DMA round trip
       * (so key-on phase resets made by the CPU take effect) */
      int en[4]; for(int c=0;c<4;c++){ en[c]=fm_state.ch[c].enabled; if(!(mask&bit[c])) fm_state.ch[c].enabled=0; }
      static int32_t buf[2048]; model(&fm_state,buf,m,22050.0/rate);
      for(int c=0;c<4;c++) fm_state.ch[c].enabled=en[c];
      for(int i=0;i<m;i++){ int32_t v=buf[2*i]>>FM_MIX_SHIFT; if(v>32767)v=32767; if(v<-32768)v=-32768; int16_t s=v; fwrite(&s,2,1,o); if(oR){ int32_t w=buf[2*i+1]>>FM_MIX_SHIFT; if(w>32767)w=32767; if(w<-32768)w=-32768; int16_t q=w; fwrite(&q,2,1,oR);} } 
    }
  }
  fclose(o);
}
