/* Run the real M1 driver; record YM2610 writes (sample time, port, addr, val)
 * and render our output with either SSG muted (mode 0: ADPCM A+B only)
 * or ADPCM muted (mode 1: SSG only). */
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
static FILE *vf, *tf; static long cur_sample;
void vrom_read(uint32_t off,uint8_t*dst,int len){ fseek(vf,off,SEEK_SET); if(fread(dst,1,len,vf)!=(size_t)len) memset(dst,0,len); }
#define HOST_REG(p,a,v) do{ if(tf){ fprintf(tf,"%ld %d %02x %02x\n",cur_sample,p,a,v);} }while(0)
#include YM_SRC
static int16_t buf[1024*2];
int main(int argc,char**argv){
  FILE*f=fopen(argv[1],"rb"); static uint8_t rom[131072]; int n=fread(rom,1,sizeof rom,f); fclose(f);
  vf=fopen(argv[2],"rb"); int mode=atoi(argv[3]); tf=fopen(argv[4],"w"); FILE*o=fopen(argv[5],"wb");
  sound_init(rom,n);
  /* command timeline (frame, cmd): real startup, then gameplay commands seen in the autoplay log */
  static const struct{int fr;int cmd;} tl[]={
    {60,1},{64,3},{94,3},{98,3},{102,0x1b},{106,3},{136,7},{140,0x7f},
    {200,0x18},{201,0x58},{300,0xf2},{330,0xc0},{334,0xc0},{338,0xc0},{360,0xf5},{380,0xc0},{390,0xde},
    {420,0x18},{421,0x5d},{500,0xc0},{505,0xc0},{510,0xdc},{520,0xcf},{600,0x80},{900,0xb6},
    {1200,0xc0},{1210,0xf5},{1250,0x18},{1251,0x40},{1600,0xc5},{1700,0xd0},{1800,0x90},{2200,0xa0},
  };
  for(int u=1;u<=3000;u++){
    for(unsigned i=0;i<sizeof tl/sizeof tl[0];i++) if(u==tl[i].fr) snd_cmd(tl[i].cmd);
    cur_sample=(long)(u-1)*368;
    sound_update(Z80_CLOCK/60);
    /* mute one side for the render only, then restore the real register values */
    uint8_t sv_ssg[3]={ssg_reg[8],ssg_reg[9],ssg_reg[10]}, sv_tl=adpcma_total, sv_bpan=adpcmb_reg[1];
    if(mode==0){ ssg_reg[8]=ssg_reg[9]=ssg_reg[10]=0; }
    else { adpcma_total=0; adpcmb_reg[1]&=~0xC0; }
    sound_render(buf,368);
    memcpy(&ssg_reg[8],sv_ssg,3); adpcma_total=sv_tl; adpcmb_reg[1]=sv_bpan;
    for(int i=0;i<368;i++) fwrite(&buf[i*2],2,1,o);
  }
  fclose(o); fclose(tf);
  fprintf(stderr,"A keyons %u, B keyons %u, SSG nz %u\n",stat_adpcma_keyons,stat_adpcmb_keyons,stat_ssg_nonzero);
}
