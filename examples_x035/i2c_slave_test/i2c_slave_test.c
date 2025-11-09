// Simple example demonstrating I2C communication with a BH1750 light sensor

#include "ch32fun.h"
#include <stdio.h>
#include "../i2c_sensor_test/i2c_lib.h"

#define SYSTEM_CLOCK_HZ 48000000

#define LED_PIN PA5

// Response data
u8 slave_temperature = 25;
u8 slave_status = 0xAA;

// Mock sensor functions
u8 ReadTemperatureSensor(void) {
    slave_temperature++;
	return slave_temperature;
}

u8 ReadStatusRegister(void) {
    return 0xAA;
}


#define  I2C_EVENT_SLAVE_RECEIVER_ADDRESS_MATCHED          ((uint32_t)0x00020002) /* BUSY and ADDR flags */
#define I2C_FLAG_RXNE                                        ((uint32_t)0x10000040)
#define  I2C_EVENT_SLAVE_STOP_DETECTED                     ((uint32_t)0x00000010)  /* STOPF flag */

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
    
	// CTLR1: 0x0401
	// CTLR2: 0x0030
	// OADDR1: 0x0004
	// CKCFGR: 0x012C

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
        slave_status = ReadStatusRegister();        
        Delay_Ms(1000);
    }
}

// Command definitions
#define CMD_READ_TEMP   0x01
#define CMD_READ_STATUS 0x02
#define CMD_READ_DATA   0x03

// Global variables for slave state
volatile u8 slave_command = 0;
volatile u8 slave_tx_len = 0;
volatile u8 slave_tx_index = 0;
volatile u8 slave_rx_index = 0;
volatile u8 slave_buffer[] = { 0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA };
volatile u8 is_transmitter = 0;


// void I2C_PrepareResponse(u8 command) {
//     switch(command) {
//         case CMD_READ_TEMP:
//             printf("CMD_READ_TEMP\r\n");
//             // Prepare to send temperature (1 byte)
//             slave_buffer[0] = slave_temperature;
//             slave_tx_index = 0;
//             break;

//         case CMD_READ_STATUS:
//             printf("CMD_READ_STATUS\r\n");
//             // Prepare to send status (1 byte)
//             slave_buffer[0] = slave_status;
//             slave_tx_index = 0;
//             break;

//         case CMD_READ_DATA:
//             printf("CMD_READ_DATA\r\n");
//             // Prepare to send data array (4 bytes)
//             for(int i = 0; i < 4; i++) {
//                 slave_buffer[i] = slave_data[i];
//             }
//             slave_tx_index = 0;
//             break;

//         default:
//             printf("Unknown Command: 0x%02X\r\n", command);
//             // Unknown command - send error code
//             slave_buffer[0] = 0xFF;  // Error code
//             slave_tx_index = 0;
//             break;
//     }

//     // Send first byte
//     I2C1->DATAR = slave_buffer[slave_tx_index++];
// }


void I2C_SendNextByte(void) {
    if(slave_tx_index < 4) {
        // Send next byte
        I2C1->DATAR = slave_buffer[slave_tx_index++];
        printf("SLAVE: Sent byte %d: 0x%02X\n", slave_tx_index, slave_buffer[slave_tx_index-1]);
    } else {
        // No more data - send 0xFF
        I2C1->DATAR = 0xFF;
        printf("SLAVE: No more data, sending 0xFF\n");
    }
}

void I2C1_EV_IRQHandler(void) __attribute__((interrupt));
void I2C1_EV_IRQHandler(void) {
    // printf("\nSLAVE ISR: STAR1=0x%04X\n", I2C1->STAR1);
    
	//# Master request address, slave address match
    if(I2C1->STAR1 & I2C_STAR1_ADDR) {
		// printf("\n****SLAVE ADDR MATCH\n");
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
		printf("****SLAVE: TXE\n");

		switch(slave_buffer[0]) {
			case 0x01:
				printf("IM HERE 0x01\n");
				I2C1->DATAR = slave_buffer[1];
				break;
			case 0x23:
				printf("IM HERE 0x23\n");
				I2C1->DATAR = slave_buffer[2];
				break;
			case 0x10:
				I2C1->DATAR = slave_buffer[1];
				break;
			case 0x11:
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
