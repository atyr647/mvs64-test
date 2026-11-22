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
      static int32_t buf[2048]; model(&st,buf,m,22050.0/rate);
      for(int i=0;i<m;i++){ int32_t v=buf[2*i]>>2; int16_t q=v>32767?32767:v<-32768?-32768:v; fwrite(&q,2,1,o);}
    }
    t22+=368;
  }
}
static FILE *lab;
/* one carrier (algo 7, only O4 audible), given EG settings; key on for `on`
 * frames, then key off for `off` frames */
static void egnote(const char*name,int ar,int d1r,int sl,int d2r,int rr,int ks,int blk,int on,int off,double rate){
  fprintf(lab,"%ld %s\n",t22,name);
  W(1,0x28,0x01); run(20,rate);                           /* let any previous note die */
  int off4[4]={0,8,4,12};
  for(int k=0;k<4;k++){ int b=off4[k]+1;
    W(1,0x30+b,1); W(1,0x40+b,k==3?0:127); W(1,0x50+b,(ks<<6)|ar); W(1,0x60+b,d1r); W(1,0x70+b,d2r); W(1,0x80+b,(sl<<4)|rr); W(1,0x90+b,0); }
  W(1,0xB1,7); W(1,0xB5,0xC0);
  W(1,0xA5,(blk<<3)|(1084>>8)); W(1,0xA1,1084&255);
  W(1,0x28,0xF1); run(on,rate);
  W(1,0x28,0x01); run(off,rate);
}
int main(int argc,char**argv){
  double rate=atof(argv[1]); tables(); sound_init(NULL,0);
  tf=fopen(argv[2],"w"); o=fopen(argv[3],"wb"); lab=fopen(argv[4],"w");
  run(5,rate);
  /*      name               AR D1R SL D2R RR KS blk on off */
  egnote("slow_attack",       12, 0, 0,  0, 8, 0, 4, 60, 30, rate);
  egnote("med_attack",        18, 0, 0,  0, 8, 0, 4, 40, 30, rate);
  egnote("decay_to_sl",       31,10, 6,  0, 6, 0, 4, 60, 40, rate);
  egnote("decay_sustain",     31,14, 3,  6, 7, 0, 4, 90, 40, rate);
  egnote("slow_release",      31, 0, 0,  0, 3, 0, 4, 20, 90, rate);
  egnote("fast_release",      31, 0, 0,  0,12, 0, 4, 20, 30, rate);
  egnote("keyscale_hi_oct",   12, 8, 4,  0, 6, 3, 7, 50, 30, rate);
  fclose(o); fclose(tf); fclose(lab); return 0;
}
