#!/bin/sh
# Print the [XSUM] summary of a -DPROFILER run (per emulated frame, fixed frame range)
grep "^\[XSUM\]" "$1" | awk '
/insn=/ { split($2,a,"="); insn=a[2]; next }
/exec_calls=/ { next }
/rsp_busy=/ { split($2,a,"="); split($3,b,"="); split($4,d,"="); printf "RSP busy %d of %d RSP cycles per frame (%.0f%%); CPU clocks %d\n", a[2], a[2]+b[2], 100*a[2]/(a[2]+b[2]), d[2]; next }
{ split($3,c,"="); split($4,i,"="); split($5,d,"=");
  printf "%-6s %8d clocks/frame (%5.1f%% of 60Hz)  icache misses %6d (%4.1f%%)  dcache misses %6d (%4.1f%%)\n", $2, c[2], c[2]/15625, i[2], 9600*i[2]/c[2], d[2], 8000*d[2]/c[2] }
END { if (insn) printf "68K instructions interpreted per frame: %d\n", insn }'
