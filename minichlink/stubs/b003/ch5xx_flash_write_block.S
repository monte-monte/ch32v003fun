// 
//  A procedure to write data to flash on CH5xx series of chips by WCH.
//  This is designed to be used as a binary stub within a bootloader loaded by minichlink
//  Copyright 2026 monte-monte
// 

#define C_SB(rs2, rs1, uimm5) .insn cs 0, 0b101, rs2, ((uimm5&0xf)>>1)|((uimm5&1)<<4)(rs1)
#define C_LBU(rd, rs1, uimm5) .insn cl 0, 0b001, rd, ((uimm5&0xf)>>1)|((uimm5&1)<<4)(rs1)
// t0 = copy of a1 - runwordpad address
// a0 = scratchpad offset
// a3 = R32_FLASH_DATA
// a4 = address in scratchpad
// a5 = target address

// Copy write function to beyond scratchpad
addi a3, a1, 4;
c.mv t0, a1; 
addi a4, a0, %lo(main_loop+4);
addi a5, a0, %lo(args+4);
1:
bge a4, a5, 2f;
c.lw a2, 0(a4);
c.sw a2, 0(a3); // Write function binary to after runwordpad in RAM
c.addi a4, 4;
c.addi a3, 4;
c.j 1b;
2:
c.lw a3, 0(a5); // load R32_FLASH_DATA address from scratch
c.lw a5, 4(a5); // load address_to_write_to from scratch
addi a4, a1, 4;
c.jr a4;

// data packet format:
// 4 bytes - HID ID + 3 zeros
// 4 bytes - end address to check
// 4 length of data in current packet
// data
// ...
// 4 bytes - data ready word - 0x12345678

// at runwordpad(a1)+4
.balign 4
main_loop:
c.li a1, -1;
c.sw a1, 0(a0); // indicate to the programmer that we are ready for the data
sw zero, 8(a0); // Clear old length
data_check:
c.lw a4, 8(a0); // If there is a data length in scratchpad, it means we've got data;
c.beqz a4, data_check;
c.add a4, a0; // calculate the offset of the end of data
data_ready:
c.lw a1, 12(a4); // check if there is a valid key after the data
li a2, 0x12345678;
bne a1, a2, data_ready; // wait until there is one
addi a4, a0, 12; // starting offset to read data from

// wait till USB IN will set runwordpad
sw zero, 0(t0);
usb_acked:
lw a1, 0(t0)
c.beqz a1, usb_acked;
// disable interrupts global
csrr a1, mstatus;
andi a1, a1, -137;
csrw mstatus, a1;

// ch5xx_flash_open
c.li a1, 4;
C_SB(a1, a3, 6); // R8_FLASH_CTRL = 4
// ch5xx_flash_begin(0xff)
c.li a1, 5;
sb zero, 6(a3); // R8_FLASH_CTRL = 0
C_SB(a1, a3, 6); // R8_FLASH_CTRL = 5
li a2, 0xff;
c.nop;
c.nop;
C_SB(a2, a3, 4); // R8_FLASH_DATA = 0xff
// ch5xx_flash_out(0xff)
open_out:
lb a1, 6(a3); // R8_FLASH_CTRL
blt a1, zero, open_out;
C_SB(a2, a3, 4); // R8_FLASH_DATA = 0xff
// ch5xx_flash_end
open_end:
lb a1, 6(a3); // R8_FLASH_CTRL
blt a1, zero, open_end;
sb zero, 6(a3); // R8_FLASH_CTRL = 0

// ch5xx_flash_addr
START:
// ch5xx_flash_begin(6)
sb zero, 6(a3);
c.li a1, 5;
sb a1, 6(a3);
c.nop;
c.nop;
c.li a1, 6;
sb a1, 4(a3);
END:
lb a1, 6(a3);
blt a1, zero, END;
sb zero, 6(a3);

// ch5xx_flash_begin(2)
sb zero, 6(a3);
c.li a1, 5;
sb a1, 6(a3);
c.nop;
c.nop;
c.li a1, 2;
sb a1, 4(a3);
// Actually set address
// ch5xx_flash_out(addr >> 16 & 0xff)
c.li a2, 3;
c.mv t1, a5;
OUT:
lb a1, 6(a3);
blt a1, zero, OUT;
srli a1, t1, 16; // t1 contains current address
andi a1, a1, 0xff;
sb a1, 4(a3);
c.slli t1, 8;
c.addi a2, -1;
c.bnez a2, OUT;

ADDRES_IS_SET:

li a2, 64; // Load up the word counter
WRITE:
c.lw a1, 0(a4);
// li a1, 0x12345678;
c.addi a4, 4;
c.sw a1, 0(a3); 
c.li a1, 0x4;
WRITE_LOOP1:
lb t1, 6(a3);
blt t1, zero, WRITE_LOOP1;
li t1, 21;
sb t1, 6(a3);
c.addi a1, -1; 
c.bnez a1, WRITE_LOOP1;
c.addi a2, -1;
c.bnez a2, WRITE;
// end of a block
addi a5, a5, 256; // increment the address
// ch5xx_flash_wait
end_write:
lb a2, 6(a3); // R8_FLASH_CTRL
blt a2, zero, end_write;
sb zero, 6(a3); // R8_FLASH_CTRL

li t1, 0x80000;
wait_loop:
beq t1, zero, return;
c.addi t1, -1;

// ch5xx_flash_begin(5)
c.li a2, 5;
sb zero, 6(a3); // R8_FLASH_CTRL
C_SB(a2, a3, 6);
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
bne t2, zero, wait_loop;

c.lw a1, 8(a0);
c.add a1, a0;
c.addi a1, 12;
bne a1, a4, START; // there is more data in the scratchpad

c.lw a1, 4(a0); // load end address
beq a5, a1, return; // check if we are done writing all data
// enable interrupts global
csrr a1, mstatus;
ori a1, a1, 0x88;
csrw mstatus, a1;
j main_loop;

return:
// enable interrupts global
csrr a1, mstatus;
ori a1, a1, 0x88;
csrw mstatus, a1;

c.li a3, -1;
c.sw a3, 0(a0);
sw zero, 0(t0); // clear runwordpad so we don't immediately jump back to scratchpad full of random data;
ret;

.balign 4
args:
