
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "libusb.h"
#include "minichlink.h"

/*************************
 * All commands here are "inspired" by https://github.com/wagiminator/MCU-Flash-Tools/blob/main/chprog.py
 * but Wagiminator is awesome so it's ok. (To the best of my knowledge that's the license)
 *************************/
#define WCH_XOR_KEY_LEN     8
#define WCH_UID_LEN         8
#define WCH_USB_EP_OUT      0x02
#define WCH_USB_EP_IN       0x82
#define WCH_USB_TIMEOUT     5000
#define WCHCHECK(x) if( (status = x) ) { fprintf( stderr, "Bad USB Operation on " __FILE__ ":%d (%d)\n", __LINE__, status ); exit( status ); }

struct ISPProgrammerStruct
{
	void * internal;
	libusb_device_handle * devh;
	int lasthaltmode; // For non-003 chips
};

static inline libusb_device_handle * isp_base_setup( int inhibit_startup )
{
	libusb_context * ctx = 0;
	int status;
	status = libusb_init(&ctx);
	if (status < 0) {
		fprintf( stderr, "Error: libusb_init_context() returned %d\n", status );
		exit( status );
	}
	
	libusb_device **list;
	ssize_t cnt = libusb_get_device_list(ctx, &list);
	ssize_t i = 0;

	libusb_device *found = NULL;

	for (i = 0; i < cnt; i++) {
		libusb_device *device = list[i];
		struct libusb_device_descriptor desc;
		int r = libusb_get_device_descriptor(device,&desc);
		if( r == 0 && desc.idVendor == 0x4348 && desc.idProduct == 0x55e0) { found = device; }
	}

	if( !found )
	{
		return 0;
	}

	libusb_device_handle * devh;
	status = libusb_open( found, &devh );
	if( status )
	{
		fprintf( stderr, "Error: couldn't access bootloader (libusb_open() = %d)\n", status );
		return 0;
	}
		
	WCHCHECK( libusb_claim_interface(devh, 0) );

	return devh;
}

void wch_isp_command( libusb_device_handle * devh, const void * command_v, int commandlen, int * transferred, uint8_t * reply, int replymax )
{
	uint8_t * command = (uint8_t*)command_v;
	uint8_t buffer[1024];
	int got_to_recv = 0;
	int status;
	int transferred_local;
	if( !transferred ) transferred = &transferred_local;
	status = libusb_bulk_transfer( devh, WCH_USB_EP_OUT, command, commandlen, transferred, WCH_USB_TIMEOUT );
	if( status ) goto sendfail;
	got_to_recv = 1;
	if( !reply )
	{
		reply = buffer; replymax = sizeof( buffer );
	}

//	printf("wch_isp_command send (%d)", commandlen); for(int i = 0; i< commandlen; printf(" %02x",command[i++])); printf("\n");

	status = libusb_bulk_transfer( devh, WCH_USB_EP_IN, reply, replymax, transferred, WCH_USB_TIMEOUT );

//	printf("wch_isp_command reply (%d)", *transferred); for(int i = 0; i< *transferred; printf(" %02x",reply[i++])); printf("\n"); 

	if( status ) goto sendfail;
	return;
sendfail:
	fprintf( stderr, "Error sending WCH command (%s): ", got_to_recv?"on recv":"on send" );
	int i;
	for( i = 0; i < commandlen; i++ )
	{
		printf( "%02x ", command[i] );
	}
	printf( "\n" );
	exit( status );
}

int ISPWriteReg32( void * dev, uint8_t reg_7_bit, uint32_t command ) { fprintf( stderr, "ISPWriteReg32\n" ); return 0; }
int ISPReadReg32( void * dev, uint8_t reg_7_bit, uint32_t * commandresp ) { fprintf( stderr, "ISPReadReg32\n" ); return 0; }
int ISPWriteWord( void * dev, uint32_t address_to_write, uint32_t data ) { fprintf( stderr, "ISPWriteWord\n" ); return 0; }

int ISPSetupInterface( void * d ) {
	libusb_device_handle * dev = ((struct ISPProgrammerStruct*)d)->devh;
	struct InternalState * iss = (struct InternalState*)(((struct ProgrammerStructBase*)d)->internal);
	uint8_t rbuff[1024];
	uint8_t chip_type = 0;
	uint8_t uid[WCH_UID_LEN];
	uint8_t xor_key[WCH_XOR_KEY_LEN];
	uint32_t transferred = 0;

	// request chip id
	wch_isp_command( dev, "\xa1\x12\x00\x52\x11MCU ISP & WCH.CN", 21, (int*)&transferred, rbuff, 1024 );
	if(transferred == 6) {
		// printf("id response: %02x %02x %02x %02x %02x %02x\n", rbuff[0], rbuff[1], rbuff[2], rbuff[3], rbuff[4], rbuff[5]);
		chip_type = rbuff[4];
		printf("chip type: ch5%02x\n", chip_type);
	}
	else {
		printf("ERROR: Request Chip ID failed.\n");
		return -1;
	}

	// read config
	wch_isp_command( dev, "\xa7\x02\x00\x1f\x00", 5, (int*)&transferred, rbuff, 1024 );
	if(transferred == 30) {
		printf("config: %02x%02x%02x%02x%02x%02x %02x%02x%02x%02x%02x%02x\n", rbuff[6], rbuff[7], rbuff[8], rbuff[9], rbuff[10], rbuff[11], rbuff[12], rbuff[13], rbuff[14], rbuff[15], rbuff[16], rbuff[17]);
		printf("bootloader: v%d.%d.%d\n", rbuff[19], rbuff[20], rbuff[21]);
		printf("chip uid: %02x %02x %02x %02x %02x %02x %02x %02x\n", rbuff[22], rbuff[23], rbuff[24], rbuff[25], rbuff[26], rbuff[27], rbuff[28], rbuff[29]);
		memcpy(uid, &rbuff[22], WCH_UID_LEN);
	}
	else {
		printf("ERROR: Read config failed.\n");
		return -2;
	}

	// create local encryption key
	uint8_t sum = 0;
	for(int i = 0; i < WCH_UID_LEN; i++) {
		sum += uid[i];
	}
	for(int i = 0; i < WCH_XOR_KEY_LEN -1; i++) {
		xor_key[i] = sum;
	}
	xor_key[WCH_XOR_KEY_LEN -1] = sum + chip_type;
	// printf("encryption key: %02x %02x %02x %02x %02x %02x %02x %02x\n", xor_key[0], xor_key[1], xor_key[2], xor_key[3], xor_key[4], xor_key[5], xor_key[6], xor_key[7]);

	// send encryption key
	sum = 0;
	for(int i = 0; i < WCH_XOR_KEY_LEN; i++) {
		sum += xor_key[i];
	}
	wch_isp_command( dev, "\xa3\x1e\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00", 33, (int*)&transferred, rbuff, 1024 );
	if(rbuff[4] != sum) {
		printf("ERROR: Failed to set encryption key.\n");
		return -3;
	}
	memcpy(iss->isp_xor_key, xor_key, WCH_XOR_KEY_LEN);

	return 0;
}

int ISPWriteBinaryBlob( void * dev, uint32_t address_to_write, uint32_t blob_size, const uint8_t * blob ) {
	fprintf( stderr, "ISPWriteBinaryBlob\n" );
	return 0;
}

int ISPHaltMode( void * dev, int mode ) {
	fprintf( stderr, "ISPHaltMode %d\n", mode );
	return 0;
}

void * TryInit_WCHISP()
{
	libusb_device_handle * wch_isp_devh;
	wch_isp_devh = isp_base_setup(0);
	if( !wch_isp_devh ) return 0;

	struct ISPProgrammerStruct * ret = malloc( sizeof( struct ISPProgrammerStruct ) );
	memset( ret, 0, sizeof( *ret ) );
	ret->devh = wch_isp_devh;
	ret->lasthaltmode = 0;

	MCF.WriteReg32 = ISPWriteReg32;
	MCF.ReadReg32 = ISPReadReg32;
	MCF.WriteWord = ISPWriteWord;

	MCF.SetupInterface = ISPSetupInterface; // no need
	MCF.WriteBinaryBlob = ISPWriteBinaryBlob;
	MCF.HaltMode = ISPHaltMode;

	return ret;
};
