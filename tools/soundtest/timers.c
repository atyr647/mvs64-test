/* Print our timer periods in 8 MHz master clocks (Z80 cycles x 2) for the same values. */
#include <stdio.h>
#include <stdint.h>
#include <string.h>
void vrom_read(uint32_t off, uint8_t *dst, int len) { memset(dst, 0, len); }
#include YM_SRC
int main(void) {
	const int ta[] = {0, 1, 512, 900, 1023}, tb[] = {0, 128, 192, 255};
	for (unsigned i = 0; i < 5; i++) printf("A %d %d\n", ta[i], TIMER_A_PERIOD(ta[i]) * 2);
	for (unsigned i = 0; i < 4; i++) printf("B %d %d\n", tb[i], TIMER_B_PERIOD(tb[i]) * 2);
	return 0;
}
