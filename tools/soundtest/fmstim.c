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
static struct rsp_fm_state st; static int first=1; static double frac=0; static FILE *o, *oR;
static void run(int frames,double rate){
  for(int f=0;f<frames;f++){
    for(int off=0;off<368;off+=92){
      eg_run(92); eg_push();
      for(int c=0;c<4;c++){ struct rsp_fm_chan *d=&st.ch[c], *s=&fm_state.ch[c];
        uint32_t ph[4]; memcpy(ph,d->phase,16); int32_t o1=d->op1_out, mem=d->mem;
        *d=*s; if(!first){ memcpy(d->phase,ph,16); d->op1_out=o1; d->mem=mem; } }
      first=0; frac+=92.0*rate/22050.0; int m=(int)frac; frac-=m;
      static int32_t buf[2048]; model(&st,buf,m,22050.0/rate);
      for(int i=0;i<m;i++){ int32_t v=buf[2*i]>>2; int16_t q=v>32767?32767:v<-32768?-32768:v; fwrite(&q,2,1,o); if(oR){ int32_t w=buf[2*i+1]>>2; int16_t z=w>32767?32767:w<-32768?-32768:w; fwrite(&z,2,1,oR);} }
    }
    t22+=368;
  }
}
static FILE *lab;
static int note_pan=0xC0, note_block=4, note_dt[4];
static void note(const char*name,int algo,int fb,int tl1,int tl2,int tl3,int tl4,int mul1,int mul2,int mul3,int mul4,double rate){
  /* channel 1 (port A, offset 1) = our ch0. Slot register offsets: O1 +0, O3 +4, O2 +8, O4 +C */
  int off[4]={0,8,4,12}, tl[4]={tl1,tl2,tl3,tl4}, mul[4]={mul1,mul2,mul3,mul4};
  fprintf(lab,"%ld %s\n",t22,name);
  W(1,0x28,0x01);                      /* key off */
  run(10,rate);
  for(int k=0;k<4;k++){ int b=off[k]+1;
    W(1,0x30+b,(note_dt[k]<<4)|mul[k]); W(1,0x40+b,tl[k]); W(1,0x50+b,0x1f); W(1,0x60+b,0x00); W(1,0x70+b,0x00); W(1,0x80+b,0x0f); W(1,0x90+b,0); }
  W(1,0xB1,(fb<<3)|algo); W(1,0xB5,note_pan);
  W(1,0xA5,(note_block<<3)|(1084>>8)); W(1,0xA1,1084&255);   /* block 4, fnum 1084 = ~440 Hz */
  W(1,0x28,0xF1);                      /* all 4 operators on, channel 1 */
  run(30,rate);
}
int main(int argc,char**argv){
  double rate=atof(argv[1]); tables(); sound_init(NULL,0);
  tf=fopen(argv[2],"w"); o=fopen(argv[3],"wb"); lab=fopen(argv[4],"w"); oR=argc>5?fopen(argv[5],"wb"):NULL;
  run(5,rate);
  note("A_sine_algo7",       7,0, 127,127,127,0,   1,1,1,1, rate);
  note("B_mod_O3toO4_tl20",  0,0, 127,127,20,0,    1,1,1,1, rate);
  note("C_cascade_17_20_20", 0,0, 17,20,20,20,     1,3,2,1, rate);
  note("D_fb6_algo7_O1only", 7,6, 0,127,127,127,   1,1,1,1, rate);
  note("E_algo4",            4,0, 20,0,20,0,       1,1,2,1, rate);
  note_pan=0x80;
  note("F_pan_left_only",    4,0, 20,0,20,0,       1,1,2,1, rate);
  note_pan=0xC0;
  /* Detune: added before MUL on the chip, so it scales with MUL. 1760 Hz x5. */
  note_block=6; note_dt[0]=3;
  note("G_detune+3_mul5",    7,0, 0,127,127,127,   5,1,1,1, rate);
  note_dt[0]=7;
  note("H_detune-3_mul5",    7,0, 0,127,127,127,   5,1,1,1, rate);
  note_block=4; note_dt[0]=0;
  fclose(o); fclose(tf); fclose(lab); return 0;
}
