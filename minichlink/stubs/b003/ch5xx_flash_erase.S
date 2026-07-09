// Note: I tried to split this erase procedure into multiple smaller binary stubs, and then running them one by one, but that appears impossible.
// I think the issue is that when stub returns to the bootloader firmware and the flash is still open, it then goes to USB irq and something breaks.
// My guess is that's some stray function (most probably memcpy) being called from flash, that messes flash controller operation, either resetting it's stat,
// or simply crashing the program (depending on which operation got interrupted). Putting whole erase procedure into ram and running it all at once works.

#define C_SB(rs2, rs1, uimm5) .insn cs 0, 0b101, rs2, ((uimm5&0xf)>>1)|((uimm5&1)<<4)(rs1)
#define C_LBU(rd, rs1, uimm5) .insn cl 0, 0b001, rd, ((uimm5&0xf)>>1)|((uimm5&1)<<4)(rs1)
#define C_SH(rs2, rs1, uimm5) .insn cs 0b10, 0b101, rs2, (uimm5>>1)(rs1)
#define C_LHU(rd, rs1, uimm5) .insn cl 0b10, 0b001, rd, (uimm5>>1)(rs1)

// addi sp, sp, -8;
// sw s0, 0(sp);
// sw s1, 4(sp);
// la a3, 0xe000e000;
// lw s0, 0(a3); // save imterrupt masks
// lw s1, 4(a3);
// c.li a4, -1;
// sw a4, 0x180(a3); // disable interrupts
// sw a4, 0x184(a3);
csrr a5, mstatus;
andi a5, a5, -137;
csrw mstatus, a5;   /* Disabling interrupts globally */
// CH5xx_FLASH_OPEN(0xe0)
// la a3, 0x40001040; // R8_SAFE_ACCESS_SIG
// li a4, 0x57;
// li a5, 0xa8;
// // sb a4, 0(a3);
// C_SB(a4, a3, 0);
// // sb a5, 0(a3);
// C_SB(a5, a3, 0);
// li a4, 0xe0; // Enable erase/write of whole flash
// // sb a4, 4(a3); // R8_FLASH_DATA
// C_SB(a4, a3, 4);

la a3, 0x40001800; // R32_FLASH_DATA

c.li a4, 4;
// sb a4, 6(a3); // R8_FLASH_CTRL = 4
C_SB(a4, a3, 6);
// ch5xx_flash_begin(0xff)
c.li a4, 5;
sb zero, 6(a3); // R8_FLASH_CTRL = 0
// sb a4, 6(a3); // R8_FLASH_CTRL = 5
C_SB(a4, a3, 6);
li a5, 0xff;
c.nop;
c.nop;
// sb a5, 4(a3); // R8_FLASH_DATA = 0xff
C_SB(a5, a3, 4);
// ch5xx_flash_out(0xff)
1:
lb a4, 6(a3); // R8_FLASH_CTRL
blt a4, zero, 1b;
// sb a5, 4(a3); // R8_FLASH_DATA = 0xff
C_SB(a5, a3, 4);
// ch5xx_flash_end
2:
lb a4, 6(a3); // R8_FLASH_CTRL
blt a4, zero, 2b;
sb zero, 6(a3); // R8_FLASH_CTRL = 0
// END OF FLASH_OPEN

addi a2, a0, %lo(args+4);
c.lw a4, 0(a2); // length
c.lw a5, 4(a2); // address
sw zero, 4(a0);

// ERASE LOOP
li t0, 0x10000; // block = 0x10000;
li a1, 0xd8; // cmd = 0xd8 (64K)
erase_loop:
c.lw a2, 4(a0);
c.addi a2, 1;
c.sw a2, 4(a0);
addi a2, t0, -1; // if ((block - 1 & address) == 0)
and a2, a2, a5;
c.beqz a2, first_begin;

// t0 - block
// t1 - spare
// t2 - spare
// a0 - scratchpad offset
// a1 - cmd
// a2 - out counter, wait counter
// a3 - R32_FLASH_DATA
// a4 - length
// a5 - address

block_shift:
srli t0, t0, 4;
addi a2, t0, -0x10;
c.beqz a2, end4; // end erase, block is less than 256
li a1, 0x20; // cmd = 0x20 (4096)
li a2, 0x1000;
beq a2, t0, erase_loop;
li a1, 0x81; // cmd = 0x81 (256)
j erase_loop;

first_begin:
bltu a4, t0, block_shift; // block <= length
sub a4, a4, t0; // length -= block
// ch5xx_flash_begin(dev, 6)
sb zero, 6(a3); // R8_FLASH_CTRL
c.li a2, 5;
// sb a2, 6(a3); // R8_FLASH_CTRL
C_SB(a2, a3, 6);
c.li a2, 6; 
c.nop;
c.nop;
// sb a2, 4(a3); // R8_FLASH_DATA
C_SB(a2, a3, 4);
end:
lb a2, 6(a3); // R8_FLASH_CTRL
blt a2, zero, end;
sb zero, 6(a3); // R8_FLASH_CTRL

// ch5xx_flash_begin(dev, cmd)
second_begin:
sb zero, 6(a3); // R8_FLASH_CTRL
c.li a2, 5; 
// sb a2, 6(a3); // R8_FLASH_CTRL
C_SB(a2, a3, 6);
c.nop;
c.nop;
// sb a1, 4(a3); // R8_FLASH_DATA = cmd
C_SB(a1, a3, 4);

c.li a2, 3; // only 3 outs
// ch5xx_flash_out
c.mv t2, a5;
OUT:
OUT_LOOP:
lb t1, 6(a3); // R8_FLASH_CTRL
blt t1, zero, OUT_LOOP;

srli t1, t2, 16; // a5 contains address
andi t1, t1, 0xff;

sb t1, 4(a3); // R8_FLASH_DATA

c.slli t2, 8;
c.addi a2, -1;
c.bnez a2, OUT;

add a5, a5, t0; // address += block;

end2:
lb a2, 6(a3); // R8_FLASH_CTRL
blt a2, zero, end2;
sb zero, 6(a3); // R8_FLASH_CTRL

// ch5xx_flash_wait
li t1, 0x80000;
loop:
beq t1, zero, return;
c.addi t1, -1;

// ch5xx_flash_begin(5)
c.li a2, 5;
sb zero, 6(a3); // R8_FLASH_CTRL
// sb a2, 6(a3); // R8_FLASH_CTRL
C_SB(a2, a3, 6);
// sb a2, 4(a3); // R8_FLASH_DATA
C_SB(a2, a3, 4);

// ch5xx_flash_in
in:
lb a2, 6(a3); // R8_FLASH_CTRL
blt a2, zero, in;
lb t2, 4(a3); // R8_FLASH_DATA
// again
in2:
lb a2, 6(a3); // R8_FLASH_CTRL
blt a2, zero, in2;
lb t2, 4(a3); // R8_FLASH_DATA

// ch5xx_flash_end
end3:
lb a2, 6(a3); // R8_FLASH_CTRL
blt a2, zero, end3;
sb zero, 6(a3); // R8_FLASH_CTRL

andi t2, t2, 1;
bne t2, zero, loop;
j erase_loop;

end4:
lb t1, 6(a3); // R8_FLASH_CTRL
blt t1, zero, end4;
sb zero, 6(a3); // R8_FLASH_CTRL

return:
// c.sw a1, 4(a0);
sw a5, 4(a0);
c.li a3, -1;
c.sw a3, 0(a0);

// la a3, 0xe000e000;
// sw s0, 0x100(a3);
// sw s1, 0x104(a3);
// lw s0, 0(sp);
// lw s1, 4(sp);
// addi sp, sp, 8;
csrr a5, mstatus;   /* Enabling interrupts globally */
ori a5, a5, 0x88;
csrw mstatus, a5;
ret;

.balign 4
args:
