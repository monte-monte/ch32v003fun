// ch5xx_flash_begin
la a3, 0x40001800; // R32_FLASH_DATA
addi a4, a0, %lo(args+4);
c.lw a5, 0(a4); // load cmd

sb zero, 6(a3);
c.li a4, 5;
sb a4, 6(a3);
c.nop;
c.nop;
sb a5, 4(a3);

c.li a3, -1;
c.sw a3, 0(a0);
ret;

.balign 4
args: