#include "ch32fun.h"
#include "fsusb.h"

#if defined(CH32V30x)
#define LED PA15
#define LED_ON 1
#elif defined(CH570_CH572)
#define LED PA9
#define LED_ON 0
#elif defined(CH57x)
#define LED PA7
#define LED_ON 0
#elif defined(CH5xx)
#define LED PA8
#define LED_ON 0
#elif defined(CH32V10x)
#define LED PC8
#define LED_ON 0
#elif defined(CH32X03x)
#define LED PB12
#define LED_ON 1
#else
#define LED PB2
#define LED_ON 1
#endif

#define BLINK_DELAY 100

#ifndef __HIGH_CODE
#define __HIGH_CODE
#endif

__attribute__((aligned(4))) static volatile uint8_t gs_usb_data_buf[USBFS_PACKET_SIZE];

__HIGH_CODE
void blink(int n) {
	for(int i = n-1; i >= 0; i--) {
		funDigitalWrite( LED, FUN_LOW ); // Turn on LED
		Delay_Ms(BLINK_DELAY);
		funDigitalWrite( LED, FUN_HIGH ); // Turn off LED
		if(i) Delay_Ms(BLINK_DELAY);
	}
}

#ifdef CH5xx
__INTERRUPT
void GPIOB_IRQHandler() {
	int status = R16_PB_INT_IF;
	R16_PB_INT_IF = status; // acknowledge

	if(status & PB8) { // PB22 interrupt comes in on PB8 if RB_PIN_INTX is set
		USBFSReset();
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
	if ( endp == 0 ) {
		// this is in the hsusb.c default handler
		ctx->USBFS_SetupReqLen = 0; // To ACK
	}
	else if( endp == USB_EP_RX ) {
#ifdef CH5xx
		if(len == 4 && ((uint32_t*)data)[0] == 0x010001a2) {
			USBFSReset();
			blink(2);
			jump_isprom();
		}
		else 
#endif
		if( gs_usb_data_buf[0] == 0 ) {
			gs_usb_data_buf[0] = len;
			for(int i = 0; i < len; i++) {
				gs_usb_data_buf[i +1] = data[i];
			}
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

	USBFSSetup();

	blink(5);
	

	while(1) {
		if( gs_usb_data_buf[0] ) {
			// Send data back to PC.
			while( USBFSCTX.endpoints[USB_EP_TX].busy & 1 );
			USBFS_SendEndpointNEW( USB_EP_TX, &((uint8_t *)gs_usb_data_buf)[1], gs_usb_data_buf[0], /*copy=*/1 ); // USBFS needs a copy here
			gs_usb_data_buf[0] = 0;
		}
	}
}
