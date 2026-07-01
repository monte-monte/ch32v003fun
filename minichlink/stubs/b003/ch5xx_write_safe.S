// ch5xx_write_safe
addi a3, a0, %lo(args+4);
c.lw a4, 0(a3); // value
c.lw a5, 4(a3); // addr
c.lw a2, 8(a3); // mode

la a3, 0x40001040; // R8_SAFE_ACCESS_SIG
li t0, 0x57;
li t1, 0xa8;
sb t0, 0(a3);
sb t1, 0(a3);

c.beqz a2, 1f;
bge a2, zero, 2f;
sb a4, 4(a3);
j return;
1:
sh a4, 4(a3);
j return;
2:
sw a4, 4(a3);

return:
sb zero, 0(a3);
c.li a3, -1;
c.sw a3, 0(a0);
ret;

.balign 4
args: