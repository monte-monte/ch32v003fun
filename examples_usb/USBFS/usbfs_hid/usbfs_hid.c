#include "ch32fun.h"
#include <stdio.h>
#include <string.h>
#include "fsusb.h"

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

uint32_t count;
int doreboot = 0;
int last = 0;

void handle_debug_input( int numbytes, uint8_t * data )
{
	last = data[0];
	count += numbytes;
}

int lrx = 0;

uint8_t scratchpad[256];

int HandleHidUserSetReportSetup( struct _USBState * ctx, tusb_control_request_t * req )
{
	int id = req->wValue & 0xff;
	if( id == 0xaa && req->wLength <= sizeof(scratchpad) )
	{
		ctx->pCtrlPayloadPtr = scratchpad;
		lrx = req->wLength;
		return req->wLength;
	}
	return 0;
}

int HandleHidUserGetReportSetup( struct _USBState * ctx, tusb_control_request_t * req )
{
	int id = req->wValue & 0xff;
	switch( id )
	{
		case 0xaa: // Handle data for testtop
			ctx->pCtrlPayloadPtr = scratchpad;
			if( sizeof(scratchpad) - 1 < lrx )
				return sizeof(scratchpad) - 1;
			else
				return lrx;
		
		case 0xe2: // Copy the printf debug buffer out of DMDATA0.
			memcpy( scratchpad, (char*)DMDATA0, 8 );
			ctx->pCtrlPayloadPtr = scratchpad;
			*DMDATA0 = 0;
			return 8;
	}
	return 0;
}

void HandleHidUserReportDataOut( struct _USBState * ctx, uint8_t * data, int len )
{
	switch( data[0] )
	{
		// Handle reboot into bootloader request
		case 0xe1:
			if( len > 7 )
			{
				if( strncmp( (char*)(data+1), "\xbe\xef\x00\xc0\x01\xd0\x0d", 7 ) == 0 )
				{
					doreboot = 1000;
				}
			}
		break;
	}
}

int HandleHidUserReportDataIn( struct _USBState * ctx, uint8_t * data, int len )
{
	// You almost will never need this, in general, you will use HandleHidUserGetReportSetup.
//	printf( "IN %d %d %08x %08x\n", len, ctx->USBFS_SetupReqLen, data, FSUSBCTX.ENDPOINTS[0] );
//	memset( data, 0xcc, len );
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
	printf( "\n\n" );
}

__USBFS_FUN_ATTRIBUTE
static __attribute__((noreturn)) void processLoop()
{
	while(1)
	{
		// If EP is not busy send data to the host
		uint8_t * buffer = USBFS_GetEPBufferIfAvailable( 1 );
		if( buffer )
		{
			buffer[0] = 'C';
			buffer[1] = 'H';
			buffer[2] = '3';
			buffer[3] = '2';
			buffer[4] = 'F';
			buffer[5] = 'U';
			buffer[6] = 'N';
			buffer[7] = '\n';
			USBFS_SendEndpoint( 1, 8 /* data length */ );
		}

		Delay_Ms(100);
		
		if( doreboot )
		{
			if( --doreboot == 0 )
			{
#if defined(CH5xx)
				USBFSReset();
				jump_isprom();
#else
				// There aren't any other chips that can reboot into USB bootloader, are there?
#endif
			}
		}
	}
}

int main()
{
	SystemInit();

	funGpioInitAll();

	funPinMode( LED, GPIO_CFGLR_OUT_10Mhz_PP );
	funDigitalWrite( LED, !LED_ON );

	printf("USBFS starting...");

	USBFSSetup();

	printf("ok\n");

	funDigitalWrite( LED, LED_ON );

	processLoop();
}
