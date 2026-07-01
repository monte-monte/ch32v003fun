#
# Erase a block of various length in flash.
# Based on original code from pgm-b003fun.c by cnlohr
# Copyright 2026 monte-monte
#

addi a4, a0, %lo(args+4);    // Start reading properties after this binary in scratchpad
c.lw a1, 0(a4);     // a1 = Address to erase from.
c.lw a5, 4(a4);     // a5 = Get flash address (0x4002200C)
lh t0, 8(a4);       // t0 = page size (64 or 256)
lh a2, 10(a4);      // t1 = total amount of data to erase
add a2, a1, a2;     // t1 = end address of the section we are writing

lui a3, %hi(0x00020000);
c.sw a3, 4(a5);     // FLASH->CTLR = CR_PAGE_ER
addi a3, a3, 0x40;

1:
c.sw a1, 8(a5);     // FLASH->ADDR = writing location.
c.sw a3, 4(a5);     // FLASH->CTRL = CR_PAGE_ER|CR_STRT_Set

2:
c.lw a4, 0(a5);     // read FLASH->STATR
c.andi a4, 1;       //  Mask off BUSY bit.
c.bnez a4, 2b;

add a1, a1, t0;     // Advance the erase address

blt a1, a2, 1b;     // Loop until the end of block.

c.li a3, -1;
c.sw a3, 0(a0);     // Write -1 into 0x00 indicating all done.

ret;

.balign 4
args:
