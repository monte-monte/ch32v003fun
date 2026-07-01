#define C_SB(rs2, rs1, uimm5) .insn cs 0, 0b101, rs2, ((uimm5&0xf)>>1)|((uimm5&1)<<4)(rs1)
#define C_LBU(rd, rs1, uimm5) .insn cl 0, 0b001, rd, ((uimm5&0xf)>>1)|((uimm5&1)<<4)(rs1)
#define C_SH(rs2, rs1, uimm5) .insn cs 0b10, 0b101, rs2, (uimm5>>1)(rs1)
#define C_LHU(rd, rs1, uimm5) .insn cl 0b10, 0b001, rd, (uimm5>>1)(rs1)

addi a3, a0, %lo(args+4);
c.lw a4, 0(a3); // cmd
c.lw a5, 4(a3); // addr

la a3, 0x40001800; // R32_FLASH_DATA

c.li a2, 0xb;
andi a1, a4, 0xbf;
bne a1, a2, first_begin; // if ((cmd & 0xbf) != 0xb)
c.li a2, 5;     // out counter
j second_begin;

first_begin:
// ch5xx_flash_begin(dev, 6)
sb zero, 6(a3);
c.li a2, 5;
// sb a2, 6(a3);
C_SB(a2, a3, 6);
c.li a2, 6; 
// sb a2, 4(a3);
C_SB(a2, a3, 4);
end:
lb a2, 6(a3);
blt a2, zero, end;
sb zero, 6(a3);
c.li a2, 3; // only 3 outs

// ch5xx_flash_begin(dev, cmd)
second_begin:
sb zero, 6(a3);
c.li a1, 5; 
// sb a1, 6(a3);
C_SB(a1, a3, 6);
// sb a4, 4(a3);
C_SB(a4, a3, 4);

// ch5xx_flash_out
OUT:
lb a4, 6(a3);
blt a4, zero, OUT;

srli a1, a5, 16; // a5 contains address
andi a1, a1, 0xff;
// sb a1, 4(a3);
C_SB(a1, a3, 4);

c.slli a5, 8;
c.addi a2, -1;
c.bnez a2, OUT;

c.li a3, -1;
c.sw a3, 0(a0);
ret;

.balign 4
args:
