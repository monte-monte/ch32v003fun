// Simple example demonstrating I2C communication with a BH1750 light sensor

#include "ch32fun.h"
#include <stdio.h>
#include "../i2c_sensor_test/i2c_lib.h"

#define SYSTEM_CLOCK_HZ 48000000
#define LED_PIN PA5

// Response data
u8 slave_temperature = 25;

// Mock sensor functions
u8 ReadTemperatureSensor(void) {
    slave_temperature++;
	return slave_temperature;
}

int main() {
    SystemInit();
	funGpioInitAll(); // Enable GPIOs
    
	printf("\n~ I2C slave Example ~\n");
	printf("Chip ID: %08lX\n", ESIG->UID0);
	printf("Chip Capacity: %d KB\n", ESIG->CAP);

	funPinMode(PA10, GPIO_CFGLR_OUT_50Mhz_AF_PP);	// I2C1 SCL
	funPinMode(PA11, GPIO_CFGLR_OUT_50Mhz_AF_PP);  // I2C1 SDA

	u8 led_state = 0;
	funPinMode(LED_PIN, GPIO_CFGLR_OUT_10Mhz_PP);	// LED

    // Initialize I2C as slave
    i2c_slave_init(0x02, SYSTEM_CLOCK_HZ, 100000);
	printf("CTLR1: 0x%04X\n", I2C1->CTLR1);	
	printf("CTLR2: 0x%04X\n", I2C1->CTLR2);	
	printf("OADDR1: 0x%04X\n", I2C1->OADDR1);
	printf("CKCFGR: 0x%04X\n", I2C1->CKCFGR);

	Delay_Ms(2000);

	uint8_t packet_count = 0;
	uint8_t rx_buffer[6];

	u32 timeout = I2C_TIMEOUT;

    // Update sensor data periodically
    while(1) {
		funDigitalWrite(LED_PIN, led_state);
		led_state = !led_state;

        // Update your sensor data here
        slave_temperature = ReadTemperatureSensor();   
        Delay_Ms(1000);
    }
}

volatile u8 slave_tx_len = 0;
volatile u8 slave_tx_index = 0;
volatile u8 slave_rx_index = 0;
volatile u8 slave_buffer[] = { 0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA };
volatile u8 is_transmitter = 0;


void I2C1_EV_IRQHandler(void) __attribute__((interrupt));
void I2C1_EV_IRQHandler(void) {
    // printf("\nSLAVE ISR: STAR1=0x%04X\n", I2C1->STAR1);
    
	//# Master request address, slave address match
    if(I2C1->STAR1 & I2C_STAR1_ADDR) {
		printf("\n****SLAVE ADDR MATCH\n");
		(void)I2C1->STAR1;
		(void)I2C1->STAR2;

		//# check if master request a read or a write
		is_transmitter = (I2C1->STAR2 & I2C_STAR2_TRA) ? 1 : 0;

		if (is_transmitter) {
			// Master request read - prepare to send data
			slave_tx_index = 0;
		} else {
			// Master request write - reset receive buffer
            slave_rx_index = 0;
		}
    }
    
	//# Master sends data, slave is ready to receive
    if(I2C1->STAR1 & I2C_STAR1_RXNE) {
        u8 received_byte = I2C1->DATAR;
		printf("****SLAVE RXNE: 0x%02X, index: %d\n", received_byte, slave_rx_index);

		if (slave_rx_index == 0) {
			switch (received_byte) {
				case 0x10:
					slave_tx_len = 2;
					break;
				case 0x11:
					slave_tx_len = 4;
					break;
			}
		}

		slave_buffer[slave_rx_index++] = received_byte;
    }
    
	//# Master requests data, slave is ready to send
	if(I2C1->STAR1 & I2C_STAR1_TXE && is_transmitter) {
		printf("****SLAVE TXE: 0x%02X, tx_idx: %d\n", slave_buffer[0], slave_tx_index);

		switch(slave_buffer[0]) {
			case 0x01: case 0x23:
				I2C1->DATAR = slave_buffer[1];
				break;
			case 0x10:
				if (slave_tx_index < slave_tx_len) {
					I2C1->DATAR = slave_buffer[5 + slave_tx_index++];
				} else {
					I2C1->DATAR = 0xFF;
				}
				break;
			case 0x11:
				if (slave_tx_index < slave_tx_len) {
					I2C1->DATAR = slave_buffer[7 + slave_tx_index++];
				} else {
					I2C1->DATAR = 0xFF;
				}
				break;
			default:
				I2C1->DATAR = 0xFF;
				break;
		}
	}

	//# Master sends stop
    if(I2C1->STAR1 & I2C_STAR1_STOPF) {
		// Clear STOPF flag by reading SR1 and writing CR1
        (void)I2C1->STAR1;
        I2C1->CTLR1 |= 0;

		// printf("\nStop Condition. received data: ");
		// for (int i = 0; i < slave_rx_index; i++) {
		// 	printf("0x%02X ", slave_buffer[i]);
		// }
		// printf("\n");

        // Reset state
        slave_rx_index = 0;
        slave_tx_index = 0;
        is_transmitter = 0;
    }
}
