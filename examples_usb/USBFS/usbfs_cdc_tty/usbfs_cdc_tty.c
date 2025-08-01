#include "ch32fun.h"
#include <stdio.h>
#include <string.h>
#include "fsusb.h"

#if defined(CH57x) && (MCU_PACKAGE == 0 || MCU_PACKAGE == 2) // ch570/2
#define PIN_LED    PA9
#define PIN_BUTTON PA1
#define BUTTON_PRESSED funDigitalRead( PIN_BUTTON )
#else
#define PIN_LED    PA8
#define PIN_BUTTON PB22
#define BUTTON_PRESSED !funDigitalRead( PIN_BUTTON )
#endif

void blink(int n) {
	for(int i = n-1; i >= 0; i--) {
		funDigitalWrite( PIN_LED, FUN_LOW ); // Turn on LED
		Delay_Ms(33);
		funDigitalWrite( PIN_LED, FUN_HIGH ); // Turn off LED
		if(i) Delay_Ms(33);
	}
}

int HandleInRequest( struct _USBState * ctx, int endp, uint8_t * data, int len )
{
	int ret = 0;  // Just NAK
	blink(2);
	switch (endp)
	{
		case 1:
			// ret = -1; // Just ACK
			break;
		case 3:
			// ret = -1; // ACK, without it RX was stuck in some cases, leaving for now as a reminder
			break;
	}
	return ret;
}

void HandleDataOut( struct _USBState * ctx, int endp, uint8_t * data, int len )
{
	blink(2);
	if( endp == 0 )
	{
		ctx->USBFS_SetupReqLen = 0; // To ACK
		if( ctx->USBFS_SetupReqCode == CDC_SET_LINE_CODING )
		{
		}
	}
	if( endp == 2 )
	{
	}
}

int HandleSetupCustom( struct _USBState * ctx, int setup_code)
{
	int ret = -1;
	blink(2);

	if( ctx->USBFS_SetupReqType & USB_REQ_TYP_CLASS )
	{
		switch( setup_code )
		{
			case CDC_SET_LINE_CODING:
			case CDC_SET_LINE_CTLSTE:
			case CDC_SEND_BREAK:
				ret = (ctx->USBFS_SetupReqLen) ? ctx->USBFS_SetupReqLen : -1;
				break;
			case CDC_GET_LINE_CODING:
				ret = ctx->USBFS_SetupReqLen;
				break;
			default:
				ret = 0;
				break;
		}
	}
	else if( ctx->USBFS_SetupReqType & USB_REQ_TYP_VENDOR )
	{
		/* Manufacturer request */
	}
	else
	{
		ret = 0; // Go to STALL
	}
	return ret;
}


int main()
{
	SystemInit();

	funGpioInitAll();

	funPinMode( PIN_LED,    GPIO_CFGLR_OUT_10Mhz_PP ); // Set PIN_LED to output
	funPinMode( PIN_BUTTON, GPIO_CFGLR_IN_PUPD ); // Set PIN_BUTTON to input

	if( BUTTON_PRESSED ) {
		blink(5);
		jump_isprom();
	}

	USBFSSetup();
	blink(1);

	while(1);
}
