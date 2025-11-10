/**
 * I2C Master Communication Example
 * 
 * This example demonstrates I2C communication with an I2C slave device.
 * You can use this with:
 * - BH1750 light sensor (actual hardware)
 * - Another CH32X035 running i2c_slave_test firmware
 * 
 * Target I2C address: 0x66 for the i2c_slave_test example
 * 
 * 0x3x Slave Command Set:
 * 0x01 - Minick BH1750 Power on command (returns 1 byte)
 * 0x23 - Minmick BH1750 Resolution command (returns 1 byte) 
 * 0x13 - Read 2 bytes from slave
 * 0x14 - Read 4 bytes from slave
 * 0x30 - Read from slave's writable buffer (32 bytes)
 * 0x31 - Write to slave's writable buffer (32 bytes)
 * 
 * Command 0x30: Read from writable buffer
 * Format: { 0x30, start_index }
 * - start_index: Position to start reading from (0-31)
 * Since buffer size is 32 bytes (0-31), reading from index 29:
 * - Returns bytes 29, 30, 31 (3 valid bytes)
 * - Remaining requested bytes return 0xFF (buffer boundary exceeded)
 * 
 * Command 0x31: Write to writable buffer  
 * Format: { 0x31, start_index, data0, data1, ... }
 * - start_index: Position to start writing to (0-31)
 * - dataX: Bytes to write (up to buffer boundary)
 * Since buffer ends at index 31:
 * - Writes 0xAA to index 29, 0xBB to index 30, 0xCC to index 31
 * - 0xDD is not written (buffer full)
 * - Returns success for written bytes only
 */


#include "ch32fun.h"
#include <stdio.h>
#include "i2c_lib.h"

#define SYSTEM_CLOCK_HZ 48000000

// #define I2C_ADDRESS 0x23		// use this for BH1750
#define I2C_ADDRESS 0x66		// use this for i2c_slave_test

u8 read_state = 0;

void print_ch32_readings(u8 i2_addr) {
	u8 rx_buf[8];
	u8 read, err;

	switch (read_state) {
		case 0:
			// read command 0x01: return 1 byte
			err = i2c_readReg_buffer(i2_addr, 0x01, rx_buf, 1);
			if (!err) {
				printf("\nRead 1 byte (cmd 0x01): 0x%02X", rx_buf[0]);
			} else {
				printf("\nError 0x%02X", err);
			}
			break;

		case 1:
			// read command 0x10: return 2 bytes
			err = i2c_readReg_buffer(i2_addr, 0x13, rx_buf, 2);
			if (!err) {
				printf("\nRead 2 bytes (cmd 0x13): ");
				for (int i = 0; i < 2; i++) {
					printf("0x%02X ", rx_buf[i]);
				}
			} else {
				printf("\nError 0x%02X", err);
			}

			break;

		case 2:
			// read command 0x11: return 4 bytes
			err = i2c_readReg_buffer(i2_addr, 0x14, rx_buf, 4);

			if (!err) {
				printf("\nRead 4 bytes (cmd 0x14): ");
				for (int i = 0; i < 4; i++) {
					printf("0x%02X ", rx_buf[i]);
				}
			} else {
				printf("\nError 0x%02X", err);
			}
			break;

		case 3:
			{
				// write command 0x31. write buffer
				u8 write_request[] = { 0x31, 29, 0xAA, 0xBB, 0xCC, 0xDD };
				err = i2c_sendBytes(i2_addr, &write_request, sizeof(write_request));

				if (!err) {
					printf("\nwrite buffer (cmd 0x%02X): successful", 0x31);
				}
				if (err) {
					printf("\nError 0x%02X", err);
				}
				break;
			}

		case 4:
			{
				// read command 0x30: read buffer
				u8 read_request[] = { 0x30, 29 };
				err = i2c_readRegTx_buffer(i2_addr, &read_request, sizeof(read_request), &rx_buf, 5);

				if (!err) {
					printf("\nRead buffer (cmd 0x30): ");
					for (int i = 0; i < 5; i++) {
						printf("0x%02X ", rx_buf[i]);
					}
				} else {
					printf("\nError 0x%02X", err);
				}
				printf("\n");
				break;
			}

		default:
			break;
	}

	read_state++;
	if (read_state > 4) read_state = 0;
}

u16 get_bh1750_readings(u8 i2cAddress) {
	u8 data[2];
	i2c_readReg_buffer(i2cAddress, 0x13, data, 2);	// get Reading
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

	i2c_init(SYSTEM_CLOCK_HZ, 100000);
	printf("\nCTLR1: 0x%04X\n", I2C1->CTLR1);	
	printf("CTLR2: 0x%04X\n", I2C1->CTLR2);	
	printf("CKCFGR: 0x%04X\n", I2C1->CKCFGR);

	i2c_sendByte(I2C_ADDRESS, 0x01); // Power on
	i2c_sendByte(I2C_ADDRESS, 0x23); // resolution

	while(1) {
		#if I2C_ADDRESS == 0x23
			u16 lux = get_bh1750_readings(I2C_ADDRESS);
			printf("BH1750 Reading: %d lx\n", lux);
		#else
			print_ch32_readings(I2C_ADDRESS);
		#endif

		Delay_Ms(1000);
	}
}
