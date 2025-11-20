#include "ch32fun.h"

void CRC_init(void) {
	// Enable CRC clock
	RCC->AHBPCENR |= RCC_AHBPeriph_CRC;
}

// Calculate CRC for a single 32-bit value
u32 CRC_calculate32(u32 data) {
	// Reset CRC unit
	CRC->CTLR = CRC_CTLR_RESET;
	
	// Write data to trigger CRC calculation
	CRC->DATAR = data;
	
	// Wait for CRC calculation
	Delay_Ms(1);

	// read the CRC result
	return CRC->DATAR;
}

// Calculate CRC for an array of 32-bit values
u32 CRC_calculateArray32(u32 *data, u32 length) {
	CRC->CTLR = CRC_CTLR_RESET;
	
	// Process each 32-bit word
	for(u32 i = 0; i < length; i++) {
		CRC->DATAR = data[i];
	}
	
	// Return final CRC
	return CRC->DATAR;
}
