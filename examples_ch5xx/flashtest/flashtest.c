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
