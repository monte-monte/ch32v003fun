#include "ch32fun.h"
#include "hsusb.h"

#if defined(CH58x)
#define LED               PA8
#define LED_ON 0
#elif defined(CH32V30x)
#define LED               PA15
#define LED_ON 1
#endif

#ifndef __HIGH_CODE
#define __HIGH_CODE
#endif

#define USB_DATA_BUF_SIZE 512

volatile uint32_t data_ready = 0;
__attribute__((aligned(4))) static uint8_t gs_usb_data_buf[USB_DATA_BUF_SIZE*2];

__HIGH_CODE
void blink(int n) {
	for(int i = n-1; i >= 0; i--) {
		funDigitalWrite( LED, LED_ON ); // Turn on LED
		Delay_Ms(100);
		funDigitalWrite( LED, !LED_ON ); // Turn off LED
		if(i) Delay_Ms(100);
	}
}

#ifdef CH5xx
__INTERRUPT
void GPIOB_IRQHandler() {
	int status = R16_PB_INT_IF;
	R16_PB_INT_IF = status; // acknowledge

	if(status & PB8) { // PB22 interrupt comes in on PB8 if RB_PIN_INTX is set
		USBHSReset();
		blink(2);
		jump_isprom();
	}
}
#endif

void GPIOSetup() {
	funPinMode( LED, GPIO_CFGLR_OUT_2Mhz_PP );

#ifdef CH5xx
	funPinMode( PB22, GPIO_CFGLR_IN_PU ); // Set PB22 to input, pullup as keypress is to gnd

	R16_PIN_ALTERNATE |= RB_PIN_INTX; // set PB8 interrupt to PB22
	R16_PB_INT_MODE |= (PB8 & ~PB); // edge mode, should go to ch32fun.h
	funDigitalWrite(PB22, FUN_LOW); // falling edge

	NVIC_EnableIRQ(GPIOB_IRQn);
	R16_PB_INT_IF = (PB8 & ~PB); // reset PB8/PB22 flag
	R16_PB_INT_EN |= (PB8 & ~PB); // enable PB8/PB22 interrupt
#endif
}

int HandleSetupCustom( struct _USBState * ctx, int setup_code) {
	return 0;
}

int HandleInRequest( struct _USBState * ctx, int endp, uint8_t * data, int len ) {
	return 0;
}

__HIGH_CODE
void HandleDataOut( struct _USBState * ctx, int endp, uint8_t * data, int len ) {
	// this is actually the data rx handler
	if( endp == USB_EP_RX ) {
#ifdef CH5xx
		if(len == 4 && ((uint32_t*)data)[0] == 0x010001a2) {
			USBHSReset();
			blink(2);
			jump_isprom();
		}
		else 
#endif
		if( data_ready == 0 ) {
			if (len > USB_DATA_BUF_SIZE) len = USB_DATA_BUF_SIZE;
			data_ready = len;
#ifdef CH58x
			mcpy_raw(gs_usb_data_buf, data, data +len);
#else
			memcpy(gs_usb_data_buf, data, len);
#endif
		}
		else {
			// previous usb buffer not consumed yet, what should we do?
		}

		ctx->endpoints[USB_EP_RX].busy = 0;
	}
}


int main() {
	SystemInit();

	funGpioInitAll(); // no-op on ch5xx
	GPIOSetup();

	USBHSSetup();

	blink(5);

	while(1) {
		if( data_ready ) {
			// Send data back to PC.
			while( USBHSCTX.endpoints[USB_EP_TX].busy & 1 );
			USBHS_SendEndpointNEW( USB_EP_TX, gs_usb_data_buf, data_ready, 0 );
			data_ready = 0;
		}
	}
}
