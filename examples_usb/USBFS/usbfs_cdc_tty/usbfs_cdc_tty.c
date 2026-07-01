#include "ch32fun.h"
#include <stdio.h>
#include <string.h>
#include "fsusb.h"

#if defined(CH32V30x)
#define PIN_LED PA15
#define LED_ON 1
#elif defined(CH570_CH572)
#define PIN_LED PA9
#define LED_ON 0
#elif defined(CH57x)
#define PIN_LED PA7
#define LED_ON 0
#elif defined(CH5xx)
#define PIN_LED PA8
#define LED_ON 0
#elif defined(CH32V10x)
#define PIN_LED PC8
#define LED_ON 0
#elif defined(CH32X03x)
#define PIN_LED PB12
#define LED_ON 1
#else
#define PIN_LED PB2
#define LED_ON 1
#endif

#define BLINK_DELAY 100

#ifdef CH5xx
#ifdef CH570_CH572
#define BUTTON_PRESSED 0
#else
#define PIN_BUTTON     PB22
#define BUTTON_PRESSED !funDigitalRead( PIN_BUTTON )
#endif
#endif

uint8_t run = 0; // print stuff or not

void blink(int n) {
	for(int i = n-1; i >= 0; i--) {
		funDigitalWrite( PIN_LED, LED_ON ); // Turn on LED
		Delay_Ms(BLINK_DELAY);
		funDigitalWrite( PIN_LED, !LED_ON ); // Turn off LED
		if(i) Delay_Ms(BLINK_DELAY);
	}
}

// this callback is mandatory when FUNCONF_USE_USBPRINTF is defined,
// can be empty though
void HandleUSBInput(int numbytes, uint8_t *data) {
	if(numbytes == 1) {
		switch(data[0]) {
		case '1':
			blink(1);
			break;
		case '2':
			blink(2);
			break;
		case '3':
			blink(3);
			break;
#ifdef CH5xx
		case 'b':
			blink(5);
			USBFSReset();
			jump_isprom();
			break;
#endif
		case 'c':
			run = 1;
			break;
		case 'p':
			run = 0;
			break;
		}
	}
	else {
		_write(0, (const char*)data, numbytes);
	}
}


int main()
{
	SystemInit();

	funGpioInitAll();

	funPinMode( PIN_LED,    GPIO_CFGLR_OUT_10Mhz_PP ); // Set PIN_LED to output
#if defined(CH5xx) && !defined(CH570_CH572)
	funPinMode( PIN_BUTTON, GPIO_CFGLR_IN_PUPD ); // Set PIN_BUTTON to input
#endif

	USBFSSetup();
	blink(1);

	int i = 0;
	while(1) {
		poll_input(); // check if there is input from the tty

#ifdef CH5xx
		if( BUTTON_PRESSED ) {
			blink(5);
			USBFSReset();
			jump_isprom();
		}
#endif

		if(run) {
			printf("Counting: %d\r", i++);
		}

		Delay_Ms(100);
	}
}
