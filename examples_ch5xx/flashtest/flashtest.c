/*
  Flash read and write demo for ch5xx

  The flash on ch5xx is accessed through some funny flash controller,
  a library to talk to that is implemented in extralibs/ch5xx_flash.h
  Some functions in that library, and the user function must be run
  from RAM otherwise the chip locks up.
  By default these are automatically and always loaded into RAM which
  takes about 480 bytes of it, but it's possible to load them on demand
  into a special RAM section by using FUNCONF_CH5XXFLASHLIB_SECTION.
  The top comment in extralibs/ch5xx_flash.h explains how to do that.

  IMPORTANT NOTE ON ERASE:
  Flash erase is done in Sectors of 4kB, when you want to write something
  to flash you first need to erase the sector it is on.
  The ch5xx_flash_cmd_erase(addr, len) function takes care of aligning
  the start and end addresses to sector boundaries, so it will erase
  more than just [addr, addr+len]!
*/

#include <stdio.h>
#include "ch32fun.h"
#include "ch5xx_flash.h"

#define SECTOR_SIZE     4096 // 4kB
#define SOMETHING_NICE "ch32fun"

uint8_t buf_compare[] = SOMETHING_NICE;
uint8_t buf[] = SOMETHING_NICE;
int check_buffers() {
	int result = 1;
	for(int i = 0; i < sizeof(buf); i++) {
		printf("%02x ", buf[i]);
		if(buf[i] != buf_compare[i]) {
			result = 0;
		}
	}
	printf("\n");
	return result;
}

__HIGH_CODE
void flashtest() {
	// init and empty buffer
	for(int i = 0; i < sizeof(buf); i++) {
		buf[i] = 0;
	}

	// read start of third sector
	uint32_t addr = 2*SECTOR_SIZE;
	uint32_t len = sizeof(buf);
	ch5xx_flash_cmd_read(addr, buf, len);
	int check1 = check_buffers();

	// erase third sector
	ch5xx_flash_cmd_erase(addr, len);
	ch5xx_flash_cmd_read(addr, buf, len);
	int check2 = check_buffers();

	// write something nice to the start of the third sector and verify the write
	ch5xx_flash_cmd_write(addr, buf_compare, sizeof(buf));
	int verify = ch5xx_flash_cmd_verify(addr, buf_compare, sizeof(buf));

	// read it back
	ch5xx_flash_cmd_read(addr, buf, len);
	int check3 = check_buffers();

	printf("flashtest: the first check will be false only on first run, second is always false and third is always true\n");
	printf("checks: chk1:%d chk2:%d verify(%d):%d chk3:%d\n", check1, check2, sizeof(buf), verify, check3);
}

int main()
{
	SystemInit();

	flashtest(); // must run from RAM

	while(1);
}
