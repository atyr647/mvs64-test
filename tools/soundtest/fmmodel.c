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
#include "exp_tab.h"
static uint16_t m_sin[256];
static void tables(void){
  for (int i = 0; i < 256; i++) {
    double m = sin(((i*2)+1) * M_PI / 1024.0), o; int n;
    o = (m > 0.0) ? 8*log(1.0/m)/log(2.0) : 8*log(-1.0/m)/log(2.0);
    o = o / (128.0/1024.0 / 4); n = (int)(2.0*o); n = (n&1) ? (n>>1)+1 : n>>1; m_sin[i] = n;
  }
}
static int32_t opcalc(uint32_t t5, uint16_t vol){
  int neg = t5 & 0x200;
  t5 = (t5 & 0x100) ? ((t5 & 0xFF) ^ 0xFF) : (t5 & 0xFF);
  uint32_t p = ((uint32_t)(vol & 0xFFFE) << 2) + m_sin[t5];
  if (p >= 3328) return 0;
  int32_t o = m_exp[p & 0xFF] >> (p >> 8);
  return neg ? -o : o;
}
/* render n samples; incr_num/incr_den rescales phase increments (for 55555 Hz) */
static void model(struct rsp_fm_state *st, int32_t *out, int n, double scale){
  memset(out,0,n*4);
  for (int c=0;c<4;c++){
    struct rsp_fm_chan *ch=&st->ch[c];
    if(!ch->enabled) continue;
    uint32_t inc[4]; for(int k=0;k<4;k++) inc[k]=(uint32_t)llround(ch->incr[k]*scale);
    for(int i=0;i<n;i++){
      int32_t pm=0;
      if(ch->fb_shift){ uint32_t t5=(uint32_t)ch->op1_out+((uint32_t)ch->op1_out<<16); pm=((int32_t)t5>>16)<<ch->fb_shift; }
      int32_t o1=opcalc((ch->phase[0]+pm)>>16, ch->vol_out[0]);
      uint32_t t2=((uint32_t)ch->op1_out<<16)|((uint32_t)o1&0xFFFF); ch->op1_out=(int32_t)t2;
      int32_t s1=(int32_t)t2>>16;
      int32_t t4= ch->c1_mod ? s1 : 0;
      int32_t s2=opcalc((ch->phase[1]>>16)+(t4>>1), ch->vol_out[1]);
      t4= ch->m2_mod ? ch->mem : 0;
      int32_t s3=opcalc((ch->phase[2]>>16)+(t4>>1), ch->vol_out[2]);
      switch(ch->c2_mod){case 1:t4=s3;break;case 2:t4=s1+s3;break;case 3:t4=ch->mem+s3;break;case 4:t4=s1;break;default:t4=0;}
      int32_t s5=opcalc((ch->phase[3]>>16)+(t4>>1), ch->vol_out[3]);
      switch(ch->mem_src){case 1:ch->mem=s2;break;case 2:ch->mem=s1+s2;break;case 3:ch->mem=s1;break;}
      int32_t acc=0; if(ch->out_flags&1)acc+=s1; if(ch->out_flags&2)acc+=s2; if(ch->out_flags&4)acc+=s3; if(ch->out_flags&8)acc+=s5;
      out[i]+=acc;
      for(int k=0;k<4;k++) ch->phase[k]+=inc[k];
    }
  }
}
int main(int argc,char**argv){
  FILE*f=fopen(argv[1],"rb"); static uint8_t rom[131072]; int n=fread(rom,1,sizeof rom,f); fclose(f); vf=fopen(argv[2],"rb");
  double rate=atof(argv[3]); FILE*o=fopen(argv[4],"wb");
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
      static int32_t buf[1024]; model(&st,buf,m,22050.0/rate);
      for(int i=0;i<m;i++){ int32_t v=buf[i]>>2; if(v>32767)v=32767; if(v<-32768)v=-32768; int16_t s=v; fwrite(&s,2,1,o);} 
    }
  }
  fclose(o);
}
