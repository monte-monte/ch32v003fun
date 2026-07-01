#define C_SB(rs2, rs1, uimm5) .insn cs 0, 0b101, rs2, ((uimm5&0xf)>>1)|((uimm5&1)<<4)(rs1)
#define C_LBU(rd, rs1, uimm5) .insn cl 0, 0b001, rd, ((uimm5&0xf)>>1)|((uimm5&1)<<4)(rs1)
// Read data, arbitrarily from memory. (byte-wise)
csrr a5, mstatus;
andi a5, a5, -137;
csrw mstatus, a5;   /* Disabling interrupts globally */

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

addi a4, a0, %lo(args+4);    // Start reading properties, starting from scratchpad + 52.
c.lw a1, 0(a4);     // Get starting address to read

// ch5xx_flash_addr(0xb)
c.li a2, 5;
flash_begin:
sb zero, 6(a3);
C_SB(a2, a3, 6);
c.li a5, 0xb;
C_SB(a5, a3, 4);
// ch5xx_flash_out
OUT:
lb a5, 6(a3);
blt a5, zero, OUT;
srli a5, a1, 16; // a1 contains address
andi a5, a5, 0xff;
C_SB(a5, a3, 4);

c.slli a1, 8;
c.addi a2, -1;
c.bnez a2, OUT

c.lw a2, 4(a4);     // Get length to read.;

flash_in:
lb a5, 6(a3);
blt a5, zero, flash_in;
lbu a1, 4(a3);
lbu a1, 4(a3);
// C_LBU(a5, a3, 4);
C_SB(a1, a4, 0);
c.addi a4, 1;
c.addi a2, -1;
c.bnez a2, flash_in;

// ch5xx_flash_end
3:
lb a4, 6(a3); // R8_FLASH_CTRL
blt a4, zero, 3b;
sb zero, 6(a3); // R8_FLASH_CTRL = 0

c.li a3, -1;
c.sw a3, 0(a0)		// Write -1 into 0x00 indicating all done.
csrr a5, mstatus;   /* Enabling interrupts globally */
ori a5, a5, 0x88;
csrw mstatus, a5;
ret

.balign 4
args:
