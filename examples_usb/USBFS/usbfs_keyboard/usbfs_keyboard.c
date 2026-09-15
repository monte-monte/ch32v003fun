#include "ch32fun.h"
#include <stdio.h>
#include <string.h>
#include "fsusb.h"
#include "usb_hid_keys.h"

#if defined(CH32V30x)
#define LED PA15
#define LED_ON 1
#elif defined(CH570_CH572)
#define LED PA9
#define LED_ON 0
#elif defined(CH5xx)
#define LED PA8
#define LED_ON 0
#elif defined(CH32V10x)
#define LED PC8
#define LED_ON 0
#elif defined(CH32L103)
#define LED PC13
#define LED_ON 0
#elif defined(CH32X03x)
#define LED PB12
#define LED_ON 1
#else
#define LED PB2
#define LED_ON 1
#endif

#define KEYBOARD_EP 1

#if defined(CH570_CH572)
#define COL1 PA10
#define COL2 PA11
#define COL3 PA2
#define ROW1 PA4
#define ROW2 PA5
#define ROW3 PA6
#define ROW4 PA7 // Make sure that Reset pin is disabled or it's configured to be on PA8
#elif defined(CH591_CH592)
#define COL1 PA14
#define COL2 PA15
#define COL3 PA5
#define ROW1 PA10
#define ROW2 PA11
#define ROW3 PA12
#define ROW4 PA13
#elif defined(CH32X03x)
#define COL1 PA2
#define COL2 PA1
#define COL3 PA0
#define ROW1 PA6
#define ROW2 PA5
#define ROW3 PA4
#define ROW4 PA3
#endif

uint32_t col_pins[3] = {COL1, COL2, COL3};
uint32_t row_pins[4] = {ROW1, ROW2, ROW3, ROW4};

uint8_t key_matrix[4][3];
uint8_t key_map[4][3] = {
	{KEY_1, KEY_2, KEY_3},
	{KEY_4, KEY_5, KEY_6},
	{KEY_7, KEY_8, KEY_9},
	{KEY_KPASTERISK, KEY_0, KEY_ENTER},
};
uint8_t key_packet[8];
uint8_t last_n_key = 0;

uint8_t scan_keys() {
	uint8_t ret = 0;
	uint8_t key_pos = 0;
	Delay_Ms(1); // To be able to register multiple presses
	for (int x = 0; x < 3; x++) {
		funPinMode(col_pins[x], GPIO_CFGLR_OUT_10Mhz_PP);
		funDigitalWrite(col_pins[x], 1);
		for (int y = 0; y < 4; y++) {
			if (funDigitalRead(row_pins[y])) {
				if (!key_matrix[y][x]) {
					key_matrix[y][x] = 1;
					if (key_pos < 6) key_packet[(key_pos++)+2] = key_map[y][x];
					ret = 1;
				} else if (key_matrix[y][x]) {
					if (!key_pos && key_matrix[y][x] == 1) {
						key_packet[key_pos+2] = key_map[y][x];
					}
					key_matrix[y][x] = 2;
					if (!ret) ret = 2;
				}
			} else {
				key_matrix[y][x] = 0;
			}
		}
		
		funDigitalWrite(col_pins[x], 0);
		funPinMode(col_pins[x], GPIO_CFGLR_IN_FLOAT);
	}
	last_n_key = key_pos;
	return ret;
}

int HandleHidUserSetReportSetup( struct _USBState * ctx, tusb_control_request_t * req )
{
	return 0;
}

int HandleHidUserGetReportSetup( struct _USBState * ctx, tusb_control_request_t * req )
{
	return 0;
}

void HandleHidUserReportDataOut( struct _USBState * ctx, uint8_t * data, int len )
{
	if (len == 1) {
		funDigitalWrite(LED, ((!!(data[0]&KEYBOARD_LED_CAPSLOCK))==LED_ON)); // Toggle LED based on CAPS LOCK state
	}

	printf("HID Report Out data:\n");
	for ( int i = 0; i < len; i++ )
	{
		printf( "0x%02x ", data[i] );
	}
	printf( "\n" );
}

int HandleHidUserReportDataIn( struct _USBState * ctx, uint8_t * data, int len )
{
	return len;
}

void HandleHidUserReportOutComplete( struct _USBState * ctx )
{
	return;
}

int HandleInRequest( struct _USBState * ctx, int endp, uint8_t * data, int len )
{
	return 0;
}

int HandleSetupCustom( struct _USBState * ctx, int setup_code)
{
	return 0;
}

void HandleDataOut( struct _USBState * ctx, int endp, uint8_t * data, int len )
{
	// Process incoming data from the host
	if ( !endp ) return;
	printf( "EP%d OUT request data:\n", endp );
	for ( int i = 0; i < len; i++ )
	{
		printf( "0x%02x ", data[i] );
	}
	printf( "\n" );
}

__USBFS_FUN_ATTRIBUTE
static __attribute__((noreturn)) void processLoop()
{
	while(1) {
		uint8_t scan = scan_keys();
		if (key_packet[2]) {
			USBFS_SendEndpointNEW(KEYBOARD_EP, key_packet, 8, 1);
			memset(key_packet, 0, 8);
		} else if (!scan) {
			uint32_t * buffer = (uint32_t*)USBFS_GetEPBufferIfAvailable(KEYBOARD_EP);
			if (buffer) {
				buffer[0] = 0;
				buffer[1] = 0;
				USBFS_SendEndpoint(KEYBOARD_EP, 8);
			}
		}
	}
}

int main()
{
	SystemInit();

	funGpioInitAll();

	for (int i = 0; i < 4; i++) {
#ifdef CH5xx
		funPinMode(row_pins[i], GPIO_ModeIN_PD);
#else
		funPinMode(row_pins[i], GPIO_CFGLR_IN_PUPD);
		funDigitalWrite(row_pins[i], 0);
#endif
	}

	funPinMode(LED, GPIO_CFGLR_OUT_10Mhz_PP);
	funDigitalWrite(LED, !LED_ON);

	printf("USBFS starting...");

	USBFSSetup();

	printf("ok\n");

	processLoop();
}
