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

static FILE *tf; static long t22;
static void W(int port,int a,int v){ fprintf(tf,"%ld %d %02x %02x\n",t22,port,a,v); if(port==1) ym2610_write_reg1(a,v); else ym2610_write_reg2(a,v); }
static struct rsp_fm_state st; static int first=1; static double frac=0; static FILE *o;
static void run(int frames,double rate){
  for(int f=0;f<frames;f++){
    for(int off=0;off<368;off+=92){
      eg_run(92); eg_push();
      for(int c=0;c<4;c++){ struct rsp_fm_chan *d=&st.ch[c], *s=&fm_state.ch[c];
        uint32_t ph[4]; memcpy(ph,d->phase,16); int32_t o1=d->op1_out, mem=d->mem;
        *d=*s; if(!first){ memcpy(d->phase,ph,16); d->op1_out=o1; d->mem=mem; } }
      first=0; frac+=92.0*rate/22050.0; int m=(int)frac; frac-=m;
      static int32_t buf[1024]; model(&st,buf,m,22050.0/rate);
      for(int i=0;i<m;i++){ int32_t v=buf[i]>>2; int16_t q=v>32767?32767:v<-32768?-32768:v; fwrite(&q,2,1,o);}
    }
    t22+=368;
  }
}
static FILE *lab;
static void note(const char*name,int algo,int fb,int tl1,int tl2,int tl3,int tl4,int mul1,int mul2,int mul3,int mul4,double rate){
  /* channel 1 (port A, offset 1) = our ch0. Slot register offsets: O1 +0, O3 +4, O2 +8, O4 +C */
  int off[4]={0,8,4,12}, tl[4]={tl1,tl2,tl3,tl4}, mul[4]={mul1,mul2,mul3,mul4};
  fprintf(lab,"%ld %s\n",t22,name);
  W(1,0x28,0x01);                      /* key off */
  run(10,rate);
  for(int k=0;k<4;k++){ int b=off[k]+1;
    W(1,0x30+b,mul[k]); W(1,0x40+b,tl[k]); W(1,0x50+b,0x1f); W(1,0x60+b,0x00); W(1,0x70+b,0x00); W(1,0x80+b,0x0f); W(1,0x90+b,0); }
  W(1,0xB1,(fb<<3)|algo); W(1,0xB5,0xC0);
  W(1,0xA5,(4<<3)|(1084>>8)); W(1,0xA1,1084&255);   /* ~440 Hz: block 4, fnum 1084 */
  W(1,0x28,0xF1);                      /* all 4 operators on, channel 1 */
  run(30,rate);
}
int main(int argc,char**argv){
  double rate=atof(argv[1]); tables(); sound_init(NULL,0);
  tf=fopen(argv[2],"w"); o=fopen(argv[3],"wb"); lab=fopen(argv[4],"w");
  run(5,rate);
  note("A_sine_algo7",       7,0, 127,127,127,0,   1,1,1,1, rate);
  note("B_mod_O3toO4_tl20",  0,0, 127,127,20,0,    1,1,1,1, rate);
  note("C_cascade_17_20_20", 0,0, 17,20,20,20,     1,3,2,1, rate);
  note("D_fb6_algo7_O1only", 7,6, 0,127,127,127,   1,1,1,1, rate);
  note("E_algo4",            4,0, 20,0,20,0,       1,1,2,1, rate);
  fclose(o); fclose(tf); fclose(lab); return 0;
}
