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
  static struct rsp_fm_state st; int first=1; double frac=0;
  for(int u=1;u<=500;u++){
    for(unsigned i=0;i<sizeof tl/sizeof tl[0];i++) if(u==tl[i].fr) snd_cmd(tl[i].cmd);
    sound_update(Z80_CLOCK/60);
    for(int off=0;off<368;off+=92){
      eg_run(92); eg_push();
      /* carry phases/op1/mem in our own copy; take params from fm_state */
      for(int c=0;c<4;c++){ struct rsp_fm_chan *d=&st.ch[c], *s=&fm_state.ch[c];
        uint32_t ph[4]; memcpy(ph,d->phase,16); int32_t o1=d->op1_out, mem=d->mem;
        *d=*s; if(!first){ memcpy(d->phase,ph,16); d->op1_out=o1; d->mem=mem; }
        /* key-on resets phase in fm_state: honour it */
        for(int k=0;k<4;k++) if(s->phase[k]==0 && ph[k]!=0 && 0) d->phase[k]=0;
      }
      first=0;
      frac += 92.0*rate/22050.0; int m=(int)frac; frac-=m;
      static int32_t buf[2048]; model(&st,buf,m,22050.0/rate);
      for(int i=0;i<m;i++){ int32_t v=buf[2*i]>>2; if(v>32767)v=32767; if(v<-32768)v=-32768; int16_t s=v; fwrite(&s,2,1,o); if(oR){ int32_t w=buf[2*i+1]>>2; if(w>32767)w=32767; if(w<-32768)w=-32768; int16_t q=w; fwrite(&q,2,1,oR);} } 
    }
  }
  fclose(o);
}
