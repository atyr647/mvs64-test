/* Drive our SSG with a register stimulus at frame boundaries; emit the same
 * writes as a trace for ymfm.  Segments: (label, regs...) each 0.5 s. */
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
static FILE *vf, *tf; static long cur_sample;
void vrom_read(uint32_t off,uint8_t*dst,int len){ memset(dst,0,len); }
#include YM_SRC
static int16_t buf[1024*2];
static void w(int a,int v){ fprintf(tf,"%ld 1 %02x %02x\n",cur_sample,a,v); ym2610_write_reg1(a,v); }
static FILE *o, *lab;
static void seg(const char*name,int frames){ fprintf(lab,"%ld %s\n",cur_sample,name); for(int f=0;f<frames;f++){ sound_render(buf,368); for(int i=0;i<368;i++) fwrite(&buf[i*2],2,1,o); cur_sample+=368; } }
int main(int argc,char**argv){
  tf=fopen("ssg_regs.txt","w"); o=fopen("ours_ssgstim.pcm","wb"); lab=fopen("ssg_labels.txt","w");
  sound_init(NULL,0);
  w(7,0x3F); seg("silence",15);
  w(0,284&255); w(1,284>>8); w(7,0x3E);
  for(int v=15; v>=1; v--){ char n[32]; sprintf(n,"toneA_vol%d",v); w(8,v); seg(n,30); }
  w(8,0); w(7,0x37); w(6,8); w(8,15); seg("noiseA_np8",30);
  w(8,0); w(7,0x38); w(0,200); w(1,0); w(2,150); w(3,0); w(4,113); w(5,0); w(8,12); w(9,12); w(10,12); seg("3tones_vol12",30);
  w(8,0); w(9,0); w(10,0); w(7,0x3E); w(0,284&255); w(1,284>>8); w(0xB,0x00); w(0xC,0x02); w(8,0x10); w(0xD,0x0E); seg("envA_shape0E",60);
  w(0xD,0x08); seg("envA_shape08",60);
  w(0xD,0x00); seg("envA_shape00",30);
  fclose(o); fclose(tf); fclose(lab); return 0;
}
