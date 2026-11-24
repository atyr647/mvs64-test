#include <rsp_queue.inc>

    .data

    RSPQ_BeginOverlayHeader
        RSPQ_DefineCommand cmd_fm_render, 12
        RSPQ_DefineCommand cmd_fm_init_tables, 12
        RSPQ_DefineCommand cmd_adpcma_render, 12
    RSPQ_EndOverlayHeader

    RSPQ_BeginSavedState
    .align 3
sin_tab:    .space 512       /* 256 x uint16_t */
exp_tab:    .space 512       /* 256 x uint16_t */
    RSPQ_EndSavedState

    /* ADPCM-A step sizes and step index changes (ymfm / YM2610) */
    .align 1
adpcma_steps:
    .half 16, 17, 19, 21, 23, 25, 28, 31, 34, 37, 41, 45, 50, 55, 60, 66, 73, 80, 88, 97, 107, 118, 130, 143, 157
    .half 173, 190, 209, 230, 253, 279, 307, 337, 371, 408, 449, 494, 544, 598, 658, 724, 796, 876, 963, 1060, 1166, 1282, 1411, 1552
adpcma_inc:
    .byte -1, -1, -1, -1, 2, 5, 7, 9

    .bss
    .align 3
adpcma_state: .space 384      /* struct rsp_adpcma_state: 6 x 16 bytes of state, 6 x 48 staged bytes */
rdram_astate: .space 4
    .align 3
chan_state:  .space 232       /* YM2610: 4 channels * 56 + 8 header; multiple of 8 so work_out stays DMA-aligned */
work_out:   .space 1024      /* up to 128 stereo samples: int32 L,R interleaved */
rdram_state: .space 4        /* saved RDRAM addr for state DMA back */
rdram_out:   .space 4        /* saved RDRAM addr for output DMA */

    .text

cmd_fm_init_tables:
    /* a1 = PhysicalAddr(sin_tab), a2 = PhysicalAddr(exp_tab) */
    move    v0, ra

    move    s0, a1
    li      s4, %lo(sin_tab)
    li      t0, DMA_SIZE(512, 1)
    jal     DMAIn
    nop

    move    s0, a2
    li      s4, %lo(exp_tab)
    li      t0, DMA_SIZE(512, 1)
    jal     DMAIn
    nop

    jr      v0
    nop

    ##############################################
    # cmd_fm_render
    #
    # a0[23:0]  = num_samples
    # a1        = PhysicalAddr(state)  [232 bytes]
    # a2        = PhysicalAddr(out_buf)
    #
    # Register allocation:
    #   v0 = saved ra (return to rspq)
    #   v1 = channel pointer in DMEM
    #   fp = channel counter (6..0)        ** NOT gp - gp is rspq_dmem_buf_ptr! **
    #   k1 = num_samples (preserved)
    #   k0 = output pointer (per sample)
    #   t9 = operator step counter (2 per output sample)
    #   s1 = routing value for op1
    #   s2 = C1 result
    #   s3 = M2 result
    #   s5 = C2 result
    ##############################################

cmd_fm_render:
    move    v0, ra

    /* Save RDRAM addresses */
    sw      a1, %lo(rdram_state)
    sw      a2, %lo(rdram_out)
    andi    k1, a0, 0xFFFF          /* k1 = num_samples */

    /* DMA channel state from RDRAM to DMEM (YM2610: 232 bytes) */
    move    s0, a1
    li      s4, %lo(chan_state)
    li      t0, DMA_SIZE(232, 1)
    jal     DMAIn
    nop

    /* Clear output buffer (stereo: 8 bytes per sample) */
    li      t3, %lo(work_out)
    sll     t4, k1, 3
    add     t5, t3, t4
clear_loop:
    sw      zero, 0(t3)
    addiu   t3, t3, 4
    bne     t3, t5, clear_loop
    nop

    /* ---- Process 4 FM channels (YM2610) ---- */
    li      v1, %lo(chan_state)
    li      fp, 4

chan_loop:
    lbu     t0, 43(v1)              /* enabled */
    beqz    t0, chan_next
    nop

    sll     t9, k1, 1               /* step counter: 2 operator steps per output sample */
    li      k0, %lo(work_out)      /* output pointer */

sample_loop:
    /* ============ Operator M1 (SLOT1) with feedback ============ */
    lw      t0, 0(v1)              /* phase[0] */
    lhu     t1, 32(v1)             /* vol_out[0] */
    lw      t2, 44(v1)             /* op1_out (prev<<16 | curr) */
    lbu     t3, 41(v1)             /* fb_shift */

    /* Feedback: pm = ((op1_out + (op1_out<<16)) >> 16) << fb_shift */
    beqz    t3, op1_no_fb
    move    t4, zero               /* pm = 0 if no feedback */
    sll     t5, t2, 16
    addu    t5, t2, t5
    sra     t4, t5, 16             /* prev + curr */
    sllv    t4, t4, t3             /* << fb_shift */
op1_no_fb:

    /* op_calc1: sin = (phase + pm) >> 16 */
    addu    t5, t0, t4
    srl     t5, t5, 16
    andi    t6, t5, 0x200          /* neg flag */
    andi    t7, t5, 0x100
    beqz    t7, op1_no_flip
    andi    t5, t5, 0xFF
    xori    t5, t5, 0xFF
    andi    t5, t5, 0xFF
op1_no_flip:
    andi    t1, t1, 0xFFFE         /* env &= ~1 */
    sll     t7, t1, 2              /* env << 2 */
    sll     t8, t5, 1
    lhu     t8, %lo(sin_tab)(t8)
    addu    t7, t7, t8             /* p = (env<<2) + sin_tab[sin] */
    li      t8, 3328
    slt     t8, t7, t8
    beqz    t8, op1_silent
    nop

    andi    t8, t7, 0xFF
    sll     t8, t8, 1
    lhu     t5, %lo(exp_tab)(t8)
    srl     t7, t7, 8
    srlv    t5, t5, t7

    bnez    t6, op1_neg
    nop
    b       op1_update
    nop
op1_neg:
    subu    t5, zero, t5
    b       op1_update
    nop

op1_silent:
    move    t5, zero

op1_update:
    /* Update op1_out: (old << 16) | (new & 0xFFFF) */
    sll     t2, t2, 16
    andi    t8, t5, 0xFFFF
    or      t2, t2, t8
    sw      t2, 44(v1)

    /* s1 = op1_out>>16 for algorithm routing */
    sra     s1, t2, 16

    /* ============ Operator C1 (SLOT2) ============ */
    lbu     t3, 40(v1)             /* c1_mod */
    move    t4, zero
    beqz    t3, c1_calc
    nop
    move    t4, s1                 /* pm = op1 routing value */

c1_calc:
    lw      t0, 4(v1)              /* phase[1] */
    lhu     t1, 34(v1)             /* vol_out[1] */
    srl     t5, t0, 16
    sra     t8, t4, 1
    addu    t5, t5, t8
    andi    t6, t5, 0x200
    andi    t7, t5, 0x100
    beqz    t7, c1_no_flip
    andi    t5, t5, 0xFF
    xori    t5, t5, 0xFF
    andi    t5, t5, 0xFF
c1_no_flip:
    andi    t1, t1, 0xFFFE
    sll     t7, t1, 2
    sll     t8, t5, 1
    lhu     t8, %lo(sin_tab)(t8)
    addu    t7, t7, t8
    li      t8, 3328
    slt     t8, t7, t8
    beqz    t8, c1_silent
    nop
    andi    t8, t7, 0xFF
    sll     t8, t8, 1
    lhu     s2, %lo(exp_tab)(t8)
    srl     t7, t7, 8
    srlv    s2, s2, t7
    beqz    t6, c1_done
    nop
    subu    s2, zero, s2
    b       c1_done
    nop
c1_silent:
    move    s2, zero
c1_done:

    /* ============ Operator M2 (SLOT3) ============ */
    lbu     t3, 42(v1)             /* m2_mod */
    move    t4, zero
    beqz    t3, m2_calc
    nop
    lw      t4, 48(v1)             /* pm = mem */

m2_calc:
    lw      t0, 8(v1)              /* phase[2] */
    lhu     t1, 36(v1)             /* vol_out[2] */
    srl     t5, t0, 16
    sra     t8, t4, 1
    addu    t5, t5, t8
    andi    t6, t5, 0x200
    andi    t7, t5, 0x100
    beqz    t7, m2_no_flip
    andi    t5, t5, 0xFF
    xori    t5, t5, 0xFF
    andi    t5, t5, 0xFF
m2_no_flip:
    andi    t1, t1, 0xFFFE
    sll     t7, t1, 2
    sll     t8, t5, 1
    lhu     t8, %lo(sin_tab)(t8)
    addu    t7, t7, t8
    li      t8, 3328
    slt     t8, t7, t8
    beqz    t8, m2_silent
    nop
    andi    t8, t7, 0xFF
    sll     t8, t8, 1
    lhu     s3, %lo(exp_tab)(t8)
    srl     t7, t7, 8
    srlv    s3, s3, t7
    beqz    t6, m2_done
    nop
    subu    s3, zero, s3
    b       m2_done
    nop
m2_silent:
    move    s3, zero
m2_done:

    /* ============ Operator C2 (SLOT4) ============ */
    /* c2_mod: 0=zero, 1=M2, 2=op1+M2, 3=mem+M2, 4=op1 */
    lbu     t3, 52(v1)
    move    t4, zero
    beqz    t3, c2_calc
    li      t8, 1
    beq     t3, t8, c2_m2
    li      t8, 2
    beq     t3, t8, c2_op1m2
    li      t8, 3
    beq     t3, t8, c2_memm2
    li      t8, 4
    beq     t3, t8, c2_op1
    nop
    b       c2_calc
    nop
c2_m2:
    b       c2_calc
    move    t4, s3
c2_op1m2:
    b       c2_calc
    addu    t4, s1, s3
c2_memm2:
    lw      t4, 48(v1)
    b       c2_calc
    addu    t4, t4, s3
c2_op1:
    move    t4, s1

c2_calc:
    lw      t0, 12(v1)             /* phase[3] */
    lhu     t1, 38(v1)             /* vol_out[3] */
    srl     t5, t0, 16
    sra     t8, t4, 1
    addu    t5, t5, t8
    andi    t6, t5, 0x200
    andi    t7, t5, 0x100
    beqz    t7, c2_no_flip
    andi    t5, t5, 0xFF
    xori    t5, t5, 0xFF
    andi    t5, t5, 0xFF
c2_no_flip:
    andi    t1, t1, 0xFFFE
    sll     t7, t1, 2
    sll     t8, t5, 1
    lhu     t8, %lo(sin_tab)(t8)
    addu    t7, t7, t8
    li      t8, 3328
    slt     t8, t7, t8
    beqz    t8, c2_silent
    nop
    andi    t8, t7, 0xFF
    sll     t8, t8, 1
    lhu     s5, %lo(exp_tab)(t8)
    srl     t7, t7, 8
    srlv    s5, s5, t7
    beqz    t6, c2_done
    nop
    subu    s5, zero, s5
    b       c2_done
    nop
c2_silent:
    move    s5, zero
c2_done:

    /* ============ Update delay memory ============ */
    /* mem_src: 0=none, 1=C1, 2=op1+C1, 3=op1 */
    lbu     t3, 54(v1)
    beqz    t3, mem_done
    li      t8, 1
    beq     t3, t8, mem_c1
    li      t8, 2
    beq     t3, t8, mem_op1c1
    li      t8, 3
    beq     t3, t8, mem_op1
    nop
    b       mem_done
    nop
mem_c1:
    b       mem_done
    sw      s2, 48(v1)
mem_op1c1:
    addu    t0, s1, s2
    b       mem_done
    sw      t0, 48(v1)
mem_op1:
    sw      s1, 48(v1)
mem_done:

    /* ============ Accumulate output ============ */
    /* channel sample = sum of the carriers selected by out_flags */
    lbu     t3, 53(v1)
    move    t0, zero

    andi    t8, t3, 1
    beqz    t8, no_op1
    nop
    addu    t0, t0, s1
no_op1:
    andi    t8, t3, 2
    beqz    t8, no_c1
    nop
    addu    t0, t0, s2
no_c1:
    andi    t8, t3, 4
    beqz    t8, no_m2
    nop
    addu    t0, t0, s3
no_m2:
    andi    t8, t3, 8
    beqz    t8, no_c2
    nop
    addu    t0, t0, s5
no_c2:
    /* pan (byte 55): bit7 = left, bit6 = right */
    lbu     t3, 55(v1)
    andi    t8, t3, 0x80
    beqz    t8, no_left
    nop
    lw      t1, 0(k0)
    addu    t1, t1, t0
    sw      t1, 0(k0)
no_left:
    andi    t8, t3, 0x40
    beqz    t8, no_right
    nop
    lw      t1, 4(k0)
    addu    t1, t1, t0
    sw      t1, 4(k0)
no_right:
    /* both steps of a sample accumulate into the same slot (2x oversampling,
     * box decimation): move on after the second (odd counter) step */
    andi    t8, t9, 1
    beqz    t8, same_slot
    nop
    addiu   k0, k0, 8
same_slot:

    /* ============ Advance phases (AFTER all operators) ============ */
    lw      t0, 0(v1)
    lw      t1, 16(v1)
    addu    t0, t0, t1
    sw      t0, 0(v1)

    lw      t0, 4(v1)
    lw      t1, 20(v1)
    addu    t0, t0, t1
    sw      t0, 4(v1)

    lw      t0, 8(v1)
    lw      t1, 24(v1)
    addu    t0, t0, t1
    sw      t0, 8(v1)

    lw      t0, 12(v1)
    lw      t1, 28(v1)
    addu    t0, t0, t1
    sw      t0, 12(v1)

    addiu   t9, t9, -1
    bnez    t9, sample_loop
    nop

chan_next:
    addiu   v1, v1, 56
    addiu   fp, fp, -1
    bnez    fp, chan_loop
    nop

    /* ---- DMA state back to RDRAM (YM2610: 232 bytes) ---- */
    lw      s0, %lo(rdram_state)
    li      s4, %lo(chan_state)
    li      t0, DMA_SIZE(232, 1)
    jal     DMAOut
    nop

    /* ---- DMA output to RDRAM ---- */
    lw      s0, %lo(rdram_out)
    li      s4, %lo(work_out)
    sll     t0, k1, 3
    addiu   t0, t0, 7
    andi    t0, t0, 0xFFF8
    addiu   t0, t0, -1
    jal     DMAOut
    nop

    jr      v0
    nop

    ##############################################
    # cmd_adpcma_render: ADPCM-A decode + resample + volume, 6 channels
    #
    # a0[15:0] = num_samples (<= 128), a1 = PhysicalAddr(struct rsp_adpcma_state),
    # a2 = PhysicalAddr(out): int32 L,R sums per sample.
    # Mirrors adpcma_model() in sound/ym2610_intf.c operation for operation.
    #
    #   v0 = saved ra      v1 = channel state     fp = channel's staged bytes
    #   s7 = channels left k1 = num_samples       k0 = output pointer
    #   t9 = samples left  s1 = acc  s2 = step  s3 = s0  s5 = s1  s6 = frac
    #   a3 = nibble index  a0 = out flags  a1 = mul  a2 = sh  s4 = rate
    ##############################################
cmd_adpcma_render:
    move    v0, ra
    sw      a1, %lo(rdram_astate)
    sw      a2, %lo(rdram_out)
    andi    k1, a0, 0xFFFF

    move    s0, a1
    li      s4, %lo(adpcma_state)
    li      t0, DMA_SIZE(384, 1)
    jal     DMAIn
    nop

    li      t3, %lo(work_out)
    sll     t4, k1, 3
    add     t5, t3, t4
a_clear:
    sw      zero, 0(t3)
    addiu   t3, t3, 4
    bne     t3, t5, a_clear
    nop

    li      v1, %lo(adpcma_state)
    li      fp, %lo(adpcma_state) + 96
    li      s7, 6
a_chan:
    lhu     t9, 10(v1)              /* nsamp */
    beqz    t9, a_chan_next
    nop
    lh      s1, 0(v1)               /* acc */
    lh      s2, 2(v1)               /* step */
    lh      s3, 4(v1)               /* s0 */
    lh      s5, 6(v1)               /* s1 */
    lhu     s6, 8(v1)               /* frac */
    lbu     a3, 12(v1)              /* first nibble index */
    lbu     a0, 13(v1)              /* out flags */
    lbu     a1, 14(v1)              /* mul */
    lbu     a2, 15(v1)              /* sh */
    li      k0, %lo(work_out)
    li      s4, 55042               /* ADPCMA_RATE_Q16 */

a_samp:
    addu    s6, s6, s4
    srl     t0, s6, 16
    beqz    t0, a_nodec
    nop
    /* ---- decode one nibble ---- */
    andi    s6, s6, 0xFFFF
    move    s3, s5                  /* s0 = s1 */
    srl     t0, a3, 1
    addu    t0, fp, t0
    lbu     t1, 0(t0)
    andi    t2, a3, 1
    bnez    t2, a_lownib
    andi    t3, t1, 15              /* (delay slot) low nibble */
    srl     t3, t1, 4               /* high nibble */
a_lownib:
    addiu   a3, a3, 1
    sll     t0, s2, 1
    lhu     t4, %lo(adpcma_steps)(t0)   /* d */
    andi    t5, t3, 7               /* m */
    move    t6, t4                  /* p = d */
    andi    t7, t5, 1
    beqz    t7, a_m1
    sll     t8, t4, 1
    addu    t6, t6, t8
a_m1:
    andi    t7, t5, 2
    beqz    t7, a_m2
    sll     t8, t4, 2
    addu    t6, t6, t8
a_m2:
    andi    t7, t5, 4
    beqz    t7, a_m4
    sll     t8, t4, 3
    addu    t6, t6, t8
a_m4:
    srl     t6, t6, 3
    andi    t7, t3, 8
    beqz    t7, a_pos
    nop
    subu    t6, zero, t6
a_pos:
    addu    s1, s1, t6
    andi    s1, s1, 0xFFF
    lb      t7, %lo(adpcma_inc)(t5)
    addu    s2, s2, t7
    bgez    s2, a_s0
    nop
    move    s2, zero
a_s0:
    slti    t7, s2, 49
    bnez    t7, a_s48
    nop
    li      s2, 48
a_s48:
    xori    t7, s1, 0x800
    addiu   s5, t7, -0x800          /* s1 = sign-extended acc */
a_nodec:
    beqz    a0, a_samp_next
    nop
    /* ---- v = s0 + ((s1 - s0) * frac) >> 16 (signed x unsigned, high half) ---- */
    subu    t0, s5, s3
    mtc2    t0, $v04.e0
    mtc2    s6, $v05.e0
    vmudm   $v06, $v04, $v05.e0
    mfc2    t1, $v06.e0
    addu    t1, s3, t1
    sll     t1, t1, 4               /* x = v << 4 */
    /* ---- p = x * mul (mul = 8..15) >> sh, & ~3 ---- */
    addiu   t2, a1, -8
    sll     t3, t1, 3
    andi    t4, t2, 1
    beqz    t4, a_v1
    nop
    addu    t3, t3, t1
a_v1:
    andi    t4, t2, 2
    beqz    t4, a_v2
    sll     t5, t1, 1
    addu    t3, t3, t5
a_v2:
    andi    t4, t2, 4
    beqz    t4, a_v4
    sll     t5, t1, 2
    addu    t3, t3, t5
a_v4:
    srav    t3, t3, a2
    li      t4, -4
    and     t3, t3, t4
    andi    t4, a0, 1
    beqz    t4, a_noL
    nop
    lw      t5, 0(k0)
    addu    t5, t5, t3
    sw      t5, 0(k0)
a_noL:
    andi    t4, a0, 2
    beqz    t4, a_samp_next
    nop
    lw      t5, 4(k0)
    addu    t5, t5, t3
    sw      t5, 4(k0)
a_samp_next:
    addiu   t9, t9, -1
    bnez    t9, a_samp
    addiu   k0, k0, 8               /* (delay slot) */

    sh      s1, 0(v1)
    sh      s2, 2(v1)
    sh      s3, 4(v1)
    sh      s5, 6(v1)
    sh      s6, 8(v1)
a_chan_next:
    addiu   v1, v1, 16
    addiu   s7, s7, -1
    bnez    s7, a_chan
    addiu   fp, fp, 48              /* (delay slot) */

    /* state back (the staged bytes are not changed) */
    lw      s0, %lo(rdram_astate)
    li      s4, %lo(adpcma_state)
    li      t0, DMA_SIZE(96, 1)
    jal     DMAOut
    nop
    lw      s0, %lo(rdram_out)
    li      s4, %lo(work_out)
    sll     t0, k1, 3
    addiu   t0, t0, -1
    jal     DMAOut
    nop
    jr      v0
    nop
