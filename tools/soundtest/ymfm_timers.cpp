// Print the timer durations (in 8 MHz master clocks) ymfm schedules for a set of TA/TB values.
#include "ymfm_opn.h"
#include <cstdio>
struct intf : ymfm::ymfm_interface {
  int last[2] = {0, 0};
  void ymfm_set_timer(uint32_t t, int32_t d) override { if (d >= 0) last[t] = d; }
};
int main() {
  const int ta[] = {0, 1, 512, 900, 1023}, tb[] = {0, 128, 192, 255};
  for (int v : ta) { intf I; ymfm::ym2610 c(I); c.reset();
    c.write_address(0x24); c.write_data(v >> 2); c.write_address(0x25); c.write_data(v & 3);
    c.write_address(0x27); c.write_data(0x01); printf("A %d %d\n", v, I.last[0]); }
  for (int v : tb) { intf I; ymfm::ym2610 c(I); c.reset();
    c.write_address(0x26); c.write_data(v); c.write_address(0x27); c.write_data(0x02); printf("B %d %d\n", v, I.last[1]); }
}
