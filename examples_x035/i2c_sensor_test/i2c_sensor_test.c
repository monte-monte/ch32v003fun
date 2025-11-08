// Simple example demonstrating I2C communication with a BH1750 light sensor

#include "ch32fun.h"
#include <stdio.h>
#include "i2c_lib.h"

#define SYSTEM_CLOCK_MHZ 48

u16 get_sensor_reading(u8 i2cAddress) {
	u8 data[2];
	i2c_readReg(i2cAddress, 0x13, data, 2);	// get Reading
	u16 raw = (data[0] << 8) | data[1];
	return raw * 12 / 10;  // Convert to lux
}

int main() {
	SystemInit();
	funGpioInitAll(); // Enable GPIOs

	printf("\n~ I2C sensors Example ~\n");
	printf("Chip ID: %08lX\n", ESIG->UID0);
	printf("Chip Capacity: %d KB\n", ESIG->CAP);

	funPinMode(PA10, GPIO_CFGLR_OUT_50Mhz_AF_PP);	// I2C1 SCL
	funPinMode(PA11, GPIO_CFGLR_OUT_50Mhz_AF_PP);  // I2C1 SDA

	i2c_init(SYSTEM_CLOCK_MHZ, 100);
	printf("CTLR1: 0x%04x\r\n", I2C1->CTLR1);
	printf("CTLR2: 0x%04x\r\n", I2C1->CTLR2);
	printf("CKCFGR: 0x%04x\r\n", I2C1->CKCFGR);

	u8 bh1750_address = 0x23;
	i2c_sendByte(bh1750_address, 0x01); // Power on
	i2c_sendByte(bh1750_address, 0x23); // resolution

	while(1) {
		u16 lux = get_sensor_reading(bh1750_address);
		printf("BH1750 Reading: %d lx\n", lux);
		Delay_Ms(1000);
	}
}
