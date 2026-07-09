// ch5xx_flash_end
la a3, 0x40001800; // R32_FLASH_DATA
addi a4, a0, %lo(args+4);
c.lw a4, 0(a4);
c.lw a2, 0(a4); // load timer value for wait loop

1:
lb a4, 6(a3);
blt a4, zero, 1b;
sb zero, 6(a3);

beq a2, zero, return;
c.addi a2, -1;

loop:
// ch5xx_flash_begin(5)
c.li a5, 5;
sb zero, 6(a3);
c.li a4, 5;
c.nop;
c.nop;
sb a4, 6(a3);
sb a5, 4(a3);

// ch5xx_flash_in
2:
lb a4, 6(a3);
blt a4, zero, 2b;
lb a5, 4(a3);
// again
2:
lb a4, 6(a3);
blt a4, zero, 2b;
lb a5, 4(a3);

// ch5xx_flash_end
lb a4, 6(a3);
blt a4, zero, 1b;
sb zero, 6(a3);

andi a4, a5, 1;
c.bnez a4, loop;

c.addi a4, 1;
c.sw a4, 4(a0);

return:
c.li a3, -1;
c.sw a3, 0(a0);
ret;

.balign 4
args:
