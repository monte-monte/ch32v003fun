la a3, 0x40001800; // R32_FLASH_DATA
1:
lb a4, 6(a3);
blt a4, zero, 1b;
lbu a5, 4(a3);

c.sw a5, 4(a0);

c.li a3, -1;
c.sw a3, 0(a0);
ret;
