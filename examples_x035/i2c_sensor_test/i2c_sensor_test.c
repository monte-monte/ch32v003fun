// Simple example demonstrating I2C communication with a BH1750 light sensor

#include "ch32fun.h"
#include <stdio.h>
#include "i2c_lib.h"

#define SYSTEM_CLOCK_HZ 48000000

u16 get_sensor_reading(u8 i2cAddress) {
	u8 data[2];
	i2c_readReg_buffer(i2cAddress, 0x13, data, 2);	// get Reading
	u16 raw = (data[0] << 8) | data[1];
	return raw * 12 / 10;  // Convert to lux
}


u8 TxData[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06 };

u8 target_i2cAddr = 0x02;

u8 my_val = 0;

void print_ch32_readings() {
	u8 rx_buf[8];
	u8 read, err;

	if (my_val == 0) {
		// read command 0x01
		u8 err = i2c_sendByte(target_i2cAddr, 0x01);
		if (!err) {
			read = i2c_readByte(target_i2cAddr);
			printf("\nRead cmd 0x%02X: 0x%02X\n", 0x01, read);
		}
	} else {
		// read command 0x23
		err = i2c_sendByte(target_i2cAddr, 0x23);
		if (!err) {
			read = i2c_readByte(target_i2cAddr);
			printf("Read cmd 0x%02X: 0x%02X\n", 0x23, read);
		}
	}

	my_val = !my_val;

	// // read command 0x10
	// err = i2c_sendByte(target_i2cAddr, 0x10);
	// if (!err) {
	// 	i2c_readBytes(target_i2cAddr, rx_buf, 2);
	
	// 	printf("Read cmd 0x%02X: ", 0x10);
	// 	for (int i = 0; i < 2; i++) {
	// 		printf("0x%02X ", 0x10 + i, rx_buf[i]);
	// 	}
	// 	printf("\n");	
	// }

	// // read command 0x11
	// i2c_sendByte(target_i2cAddr, 0x11);
	// i2c_readBytes(target_i2cAddr, rx_buf, 4);
	
	// printf("Read cmd 0x%02X: \n", 0x11);
	// for (int i = 0; i < 4; i++) {
	// 	printf("0x%02X ", 0x11 + i, rx_buf[i]);
	// }
	// printf("\n");

	

	// u8 err = i2c_sendBytes(0x02, &TxData, 6);

	// printf("\nMaster Sent packet: ");
	// for(int i = 0; i < 6; i++) {
	// 	printf("%02X ", TxData[i]);
	// }
	// printf("\r\n");


	// // Read back
	// u8 read[1] = { 0x01 };
	// err = i2c_readBytes(0x02, &read, 1);

	// printf("Master: Received packet: ");
	// for(int i = 0; i < sizeof(read); i++) {
	// 	printf("%02X ", read[i]);
	// }
	// printf("\r\n");
}

int main() {
	SystemInit();
	funGpioInitAll(); // Enable GPIOs

	printf("\n~ I2C sensors Example ~\n");
	printf("Chip ID: %08lX\n", ESIG->UID0);
	printf("Chip Capacity: %d KB\n", ESIG->CAP);

	funPinMode(PA10, GPIO_CFGLR_OUT_50Mhz_AF_PP);	// I2C1 SCL
	funPinMode(PA11, GPIO_CFGLR_OUT_50Mhz_AF_PP);  // I2C1 SDA

	i2c_init(SYSTEM_CLOCK_HZ, 100000);

	printf("\nCTLR1: 0x%04X\n", I2C1->CTLR1);	
	printf("CTLR2: 0x%04X\n", I2C1->CTLR2);	
	printf("CKCFGR: 0x%04X\n", I2C1->CKCFGR);

	// u8 bh1750_address = 0x23;
	// i2c_sendByte(bh1750_address, 0x01); // Power on
	// i2c_sendByte(bh1750_address, 0x23); // resolution

	while(1) {
		print_ch32_readings();
		// u16 lux = get_sensor_reading(bh1750_address);
		// printf("BH1750 Reading: %d lx\n", lux);
		Delay_Ms(1000);
	}
}
