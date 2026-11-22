// Replay a YM2610 register trace through ymfm at 55.5 kHz.
// usage: ymfm_play <vrom> <trace> <out_fm_adpcm_L.pcm> <out_ssg.pcm> <n22050samples> <fm_channel_mask_hex> [out_fm_adpcm_R.pcm]
// (mask 0 = FM silent; 36 = the YM2610's four FM channels)
#include "ymfm_opn.h"
#include <cstdio>
#include <vector>
#include <cstdlib>
struct intf : ymfm::ymfm_interface {
  std::vector<uint8_t> v;
  uint8_t ymfm_external_read(ymfm::access_class t, uint32_t a) override {
    if (t == ymfm::ACCESS_ADPCM_A || t == ymfm::ACCESS_ADPCM_B) return a < v.size() ? v[a] : 0;
    return 0;
  }
};
int main(int argc, char** argv) {
  intf I; FILE* vf = fopen(argv[1], "rb"); fseek(vf,0,SEEK_END); I.v.resize(ftell(vf)); fseek(vf,0,SEEK_SET); fread(I.v.data(),1,I.v.size(),vf);
  ymfm::ym2610 chip(I, (uint8_t)strtol(argv[6],0,16));
  chip.set_fidelity(ymfm::OPN_FIDELITY_MED);
  chip.reset();
  FILE* tf = fopen(argv[2], "r"); FILE* oa = fopen(argv[3], "wb"); FILE* os = fopen(argv[4], "wb"); FILE* orr = argc > 7 ? fopen(argv[7], "wb") : nullptr;
  long total22 = atol(argv[5]);
  const double rate = 8000000.0 / 144;   // 55555.6 Hz
  double t22; int p; unsigned a, v; int have = fscanf(tf, "%lf %d %x %x", &t22, &p, &a, &v) == 4;
  long n55 = (long)(total22 * rate / 22050.0);
  ymfm::ym2610::output_data out;
  for (long i = 0; i < n55; i++) {
    double tnow22 = i * 22050.0 / rate;
    while (have && t22 <= tnow22) {
      if (p == 1) { chip.write_address(a); chip.write_data(v); } else { chip.write_address_hi(a); chip.write_data_hi(v); }
      have = fscanf(tf, "%lf %d %x %x", &t22, &p, &a, &v) == 4;
    }
    chip.generate(&out, 1);
    int16_t l = (int16_t)out.data[0], s = (int16_t)out.data[2];
    fwrite(&l, 2, 1, oa); fwrite(&s, 2, 1, os);
    if (orr) { int16_t rr = (int16_t)out.data[1]; fwrite(&rr, 2, 1, orr); }
  }
}
