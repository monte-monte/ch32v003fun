la a3, 0x40001800; // R32_FLASH_DATA
addi a4, a0, %lo(args+4);
c.lw a5, 0(a4);

1:
lb a4, 6(a3);
blt a4, zero, 1b;
sb a5, 4(a3);

c.li a3, -1;
c.sw a3, 0(a0);
ret;

.balign 4
args:
