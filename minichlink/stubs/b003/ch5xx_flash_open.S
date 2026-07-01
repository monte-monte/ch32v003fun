
la a3, 0x40001800; // R32_FLASH_DATA
c.li a4, 4;
sb a4, 6(a3);
// ch5xx_flash_begin(0xff)
c.li a4, 5;
sb zero, 6(a3);
sb a4, 6(a3);
li a5, 0xff;
c.nop;
c.nop;
sb a5, 4(a3);
// ch5xx_flash_out(0xff)
1:
lb a4, 6(a3);
blt a4, zero, 1b;
sb a5, 4(a3);
// ch5xx_flash_end
2:
lb a4, 6(a3);
blt a4, zero, 2b;
sb zero, 6(a3);

c.li a3, -1;
c.sw a3, 0(a0);
ret;
