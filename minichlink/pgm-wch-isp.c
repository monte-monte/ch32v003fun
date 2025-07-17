
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "libusb.h"
#include "minichlink.h"

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

int ISPWriteReg32( void * dev, uint8_t reg_7_bit, uint32_t command ) { fprintf( stderr, "ISPWriteReg32\n" ); return 0; }
int ISPReadReg32( void * dev, uint8_t reg_7_bit, uint32_t * commandresp ) { fprintf( stderr, "ISPReadReg32\n" ); return 0; }
int ISPWriteWord( void * dev, uint32_t address_to_write, uint32_t data ) { fprintf( stderr, "ISPWriteWord\n" ); return 0; }

int ISPSetupInterface( void * dev ) { fprintf( stderr, "ISP bootloader interface was born ready\n" ); return 0; }
int ISPWriteBinaryBlob( void * dev, uint32_t address_to_write, uint32_t blob_size, const uint8_t * blob ) { fprintf( stderr, "ISPWriteBinaryBlob\n" ); return 0; }
int ISPHaltMode( void * dev, int mode ) { fprintf( stderr, "ISPHaltMode\n" ); return 0; }

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
