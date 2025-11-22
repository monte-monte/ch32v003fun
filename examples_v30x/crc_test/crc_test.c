// Simple example that shows how to use the CRC functions

#include "ch32fun.h"
#include <stdio.h>

static void CRC_init(void) {
	// Enable CRC clock
	RCC->AHBPCENR |= RCC_AHBPeriph_CRC;
}

// Calculate CRC for a single 32-bit value
static u32 CRC_calculate32(u32 data) {
	// Reset CRC unit
	CRC->CTLR = CRC_CTLR_RESET;
	
	// Write data to trigger CRC calculation
	CRC->DATAR = data;
	
	// Wait for CRC calculation
	u32 timeout = 10000;
	while (CRC->DATAR == 0 && --timeout);

	// read the CRC result
	return CRC->DATAR;
}

// Calculate CRC for an array of 32-bit values
static u32 CRC_calculateArray32(u32 *data, u32 length) {
	CRC->CTLR = CRC_CTLR_RESET;
	
	// Process each 32-bit word
	for(u32 i = 0; i < length; i++) {
		CRC->DATAR = data[i];
	}
	
	// Return final CRC
	return CRC->DATAR;
}


int main() {
	SystemInit();
	Delay_Ms(100);

	printf("\n~ CRC Test ~\n");
	CRC_init();
	
	// Single 32-bit value
	u32 single_data1 = 0x12345678;
	u32 crc = CRC_calculate32(single_data1);
	printf("CRC of 0x%08X: 0x%08X\n", single_data1, crc);	// expected 0xDF8A8A2B

	// Single 32-bit value
	u32 single_data2 = 0x12345677;
	crc = CRC_calculate32(single_data2);
	printf("CRC of 0x%08X: 0x%08X\n", single_data2, crc);	// expected 0xE7C53796

	// Array of 32-bit values
	u32 data_array[] = {0x11223344, 0x55667788, 0x99AABBCC, 0xDDEEFF00};
	u32 array_size = sizeof(data_array) / sizeof(data_array[0]);
	crc = CRC_calculateArray32(data_array, array_size);
	printf("CRC of array: 0x%08X\n", crc);

	while(1) {}
}