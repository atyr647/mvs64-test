/* C model of rsp_fm.S, operation for operation, for host-side tests.
 * Include after ym2610_intf.c (needs struct rsp_fm_state) and exp_tab.h
 * (the exp table rsp_fm.c uploads, extracted by run_host_tests.sh). */
#include <math.h>
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
/* Render n stereo samples (int32 L,R interleaved) exactly as rsp_fm.S does.
 * scale rescales the phase increments (22050/rate) for rendering at `rate`. */
static void model(struct rsp_fm_state *st, int32_t *out, int n, double scale){
  memset(out,0,n*8);
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
      if(ch->pan&0x80) out[2*i]+=acc;
      if(ch->pan&0x40) out[2*i+1]+=acc;
      for(int k=0;k<4;k++) ch->phase[k]+=inc[k];
    }
  }
}
