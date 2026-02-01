#ifndef _USBD_H
#define _USBD_H

#include <stdint.h>
#include "ch32fun.h"
#include "usb_defines.h"
#include "usb_config.h"


// Register Bit Definitions
#define USBD_CNTR_CTRM          0x8000
#define USBD_CNTR_PMAOVRM       0x4000
#define USBD_CNTR_ERRM          0x2000
#define USBD_CNTR_WKUPM         0x1000
#define USBD_CNTR_SUSPM         0x0800
#define USBD_CNTR_RESETM        0x0400
#define USBD_CNTR_SOFM          0x0200
#define USBD_CNTR_ESOFM         0x0100
#define USBD_CNTR_RESUME        0x0010
#define USBD_CNTR_FSUSP         0x0008
#define USBD_CNTR_LP_MODE       0x0004
#define USBD_CNTR_PDWN          0x0002
#define USBD_CNTR_FRES          0x0001

#define USBD_ISTR_CTR           0x8000
#define USBD_ISTR_PMAOVR        0x4000
#define USBD_ISTR_ERR           0x2000
#define USBD_ISTR_WKUP          0x1000
#define USBD_ISTR_SUSP          0x0800
#define USBD_ISTR_RESET         0x0400
#define USBD_ISTR_SOF           0x0200
#define USBD_ISTR_ESOF          0x0100
#define USBD_ISTR_DIR           0x0010
#define USBD_ISTR_EP_ID         0x000F

#define USBD_EPR_CTR_RX         0x8000
#define USBD_EPR_DTOG_RX        0x4000
#define USBD_EPR_STAT_RX_MASK   0x3000
#define USBD_EPR_STAT_RX_DIS    0x0000
#define USBD_EPR_STAT_RX_STALL  0x1000
#define USBD_EPR_STAT_RX_NAK    0x2000
#define USBD_EPR_STAT_RX_VALID  0x3000
#define USBD_EPR_SETUP          0x0800
#define USBD_EPR_EP_TYPE_MASK   0x0600
#define USBD_EPR_EP_TYPE_BULK   0x0000
#define USBD_EPR_EP_TYPE_CTRL   0x0200
#define USBD_EPR_EP_TYPE_ISO    0x0400
#define USBD_EPR_EP_TYPE_INT    0x0600
#define USBD_EPR_EP_KIND        0x0100
#define USBD_EPR_CTR_TX         0x0080
#define USBD_EPR_DTOG_TX        0x0040
#define USBD_EPR_STAT_TX_MASK   0x0030
#define USBD_EPR_STAT_TX_DIS    0x0000
#define USBD_EPR_STAT_TX_STALL  0x0010
#define USBD_EPR_STAT_TX_NAK    0x0020
#define USBD_EPR_STAT_TX_VALID  0x0030
#define USBD_EPR_EA             0x000F

// Packet Memory Area (PMA)
// On CH32V20x/F103, PMA is 512 bytes, accessed as 16-bit words spaced by 32-bits.
// Base 0x40006000.
#define USBD_PMA_BASE           0x40006000
#define USBD_PMA_SIZE           512 

// Helper for requests
#define USB_REQ_TYP_MASK        0x60
#define USB_REQ_TYP_STANDARD    0x00
#define USB_REQ_TYP_CLASS       0x20
#define USB_REQ_TYP_VENDOR      0x40

#define USB_REQ_RECIP_MASK      0x1F
#define USB_REQ_RECIP_DEVICE    0x00
#define USB_REQ_RECIP_IFACE     0x01
#define USB_REQ_RECIP_ENDP      0x02

#define DEF_USBD_UEP0_SIZE      64
#define DEF_UEP_IN              0x80
#define DEF_UEP_OUT             0x00

// Context Structure
struct _USBState;

// API Functions
int USBDSetup();
int USBD_SendEndpoint(int endp, uint8_t *data, int len);

// Internal/Callback Functions (User must implement some if FUSB_USER_HANDLERS is set)
#if FUSB_HID_USER_REPORTS
int HandleHidUserGetReportSetup(struct _USBState *ctx, tusb_control_request_t *req);
int HandleHidUserSetReportSetup(struct _USBState *ctx, tusb_control_request_t *req);
void HandleHidUserReportDataOut(struct _USBState *ctx, uint8_t *data, int len);
int HandleHidUserReportDataIn(struct _USBState *ctx, uint8_t *data, int len);
void HandleHidUserReportOutComplete(struct _USBState *ctx);
#endif

#if FUSB_USER_HANDLERS
int HandleInRequest(struct _USBState *ctx, int endp, uint8_t *data, int len);
void HandleDataOut(struct _USBState *ctx, int endp, uint8_t *data, int len);
int HandleSetupCustom(struct _USBState *ctx, int setup_code);
#endif

typedef enum
{
	USBD_EP_OFF = 0,
	USBD_EP_RX  = -1,
	USBD_EP_TX  = 1,
	USBD_EP_RTX = 2,
} USBD_EP_mode;

// Configuration defaults
#ifndef FUSB_EP1_MODE
#define FUSB_EP1_MODE 0
#endif
#ifndef FUSB_EP2_MODE
#define FUSB_EP2_MODE 0
#endif
#ifndef FUSB_EP3_MODE
#define FUSB_EP3_MODE 0
#endif
#ifndef FUSB_EP4_MODE
#define FUSB_EP4_MODE 0
#endif
#ifndef FUSB_EP5_MODE
#define FUSB_EP5_MODE 0
#endif
#ifndef FUSB_EP6_MODE
#define FUSB_EP6_MODE 0
#endif
#ifndef FUSB_EP7_MODE
#define FUSB_EP7_MODE 0
#endif
#ifndef FUSB_CONFIG_EPS
#define FUSB_CONFIG_EPS 5
#endif
#ifndef FUSB_HID_INTERFACES
#define FUSB_HID_INTERFACES 0
#endif

struct _USBState
{
	// Setup Request
	uint8_t  USBD_SetupReqCode;
	uint8_t  USBD_SetupReqType;
	uint16_t USBD_SetupReqLen;
	uint32_t USBD_IndexValue;

	// Device Status
	uint8_t  USBD_DevConfig;
	uint8_t  USBD_DevAddr;
	uint8_t  USBD_DevSleepStatus;
	uint8_t  USBD_DevEnumStatus;

	uint8_t* pCtrlPayloadPtr;

	// Buffers in main RAM (synced to PMA by library)
	uint8_t ENDPOINTS[FUSB_CONFIG_EPS][DEF_USBD_UEP0_SIZE];

	USBD_EP_mode endpoint_mode[FUSB_CONFIG_EPS+1];

	#define CTRL0BUFF               (USBDCTX.ENDPOINTS[0])
	#define pUSBD_SetupReqPak       ((tusb_control_request_t*)CTRL0BUFF)

#if FUSB_HID_INTERFACES > 0
	uint8_t USBD_HidIdle[FUSB_HID_INTERFACES];
	uint8_t USBD_HidProtocol[FUSB_HID_INTERFACES];
#endif
	volatile uint8_t USBD_Endp_Busy[FUSB_CONFIG_EPS];
	volatile uint8_t USBD_errata_dont_send_endpoint_in_window;

	// Internal: Map logical endpoints to PMA addresses
	uint16_t pma_offset[FUSB_CONFIG_EPS][2]; // [ep][0=TX, 1=RX]
};

extern struct _USBState USBDCTX;

#include "usbd.c"

#endif
