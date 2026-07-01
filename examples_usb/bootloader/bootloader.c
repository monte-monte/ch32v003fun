#include "ch32fun.h"
#include <stdio.h>
#include <string.h>
#include "fsusb.h"

#if defined(CH32V30x)
#define BOOTLOADER_LED_PIN PA15
#define BOOTLOADER_LED_POLARITY 1
#elif defined(CH57x)
#define SYSTICK_CNT (SysTick->CNT)
#define BOOTLOADER_LED_PIN PA7
#define BOOTLOADER_LED_POLARITY 0
#define BOOTLOADER_BTN_PIN 0
#elif defined(CH5xx)
#define SYSTICK_CNT (SysTick->CNTL)
#define BOOTLOADER_LED_PIN PA8
#define BOOTLOADER_LED_POLARITY 0
#elif defined(CH32V10x)
#define BOOTLOADER_LED_PIN PC8
#define BOOTLOADER_LED_POLARITY 0
#elif defined(CH32X03x)
#define SYSTICK_CNT (SysTick->CNTL)
#define BOOTLOADER_LED_PIN PB12
#define BOOTLOADER_LED_POLARITY 1
#define BOOTLOADER_BTN_PIN PC17
#else
#define BOOTLOADER_LED_PIN PB2
#define BOOTLOADER_LED_POLARITY 1
#define BOOTLOADER_BTN_PIN 0
#endif

// If you don't want to automatically boot into the application ever,
// comment out BOOTLOADER_TIMEOUT_USB and set this flag:
#define DISABLE_BOOTLOAD 1

#ifdef BOOTLOADER_LED_PIN
#if BOOTLOADER_LED_POLARITY
#define BOOT_LED(n) funDigitalWrite(BOOTLOADER_LED_PIN, n)
#else
#define BOOT_LED(n) funDigitalWrite(BOOTLOADER_LED_PIN, !n)
#endif
#else
#define BOOT_LED(n)
#endif

// Bootloader Button Config
// If you want to use a Button during boot to enter bootloader, use these defines 
// to setup the Button. If you do, it makes sense to also set DISABLE_BOOTLOAD above, 
// set BOOTLOADER_TIMEOUT_PWR_MS to 0 and disable BOOTLOADER_TIMEOUT_USB

#define BOOTLOADER_BTN_TRIG_LEVEL 0 // 1 = HIGH; 0 = LOW
#define BOOTLOADER_BTN_PULL 1 // 1 = Pull-Up; 0 = Pull-Down; Optional, comment out for floating input

// Timeout for bootloader after power-up, set to 0 to stay in bootloader forever
#define BOOTLOADER_TIMEOUT_PWR_MS 5000

#define BOOTLOADER_TIMEOUT_BASE 4294967295 - Ticks_from_Ms(BOOTLOADER_TIMEOUT_PWR_MS)

// To be able to reboot into bootloader from firmware without pressing a button
// For this to work add a following reboot function to your firmware code:
// ----------------------------------
// FLASH->BOOT_MODEKEYR = 0x45670123;
// FLASH->BOOT_MODEKEYR = 0xCDEF89AB;
// FLASH->STATR = 0x4000;
// RCC->RSTSCKR |= 0x1000000;
// PFIC->CFGR = 0xBEEF0080;
// ----------------------------------
#define SOFT_REBOOT_TO_BOOTLOADER

#define DATA_SIZE 6144

#define SCRATCHPAD_SIZE DATA_SIZE+128

__attribute__((section(".scratchpad"))) uint8_t scratchpad[SCRATCHPAD_SIZE];
__attribute__((section(".runwordpad"))) volatile int32_t runwordpad;
__attribute__((section(".boot_address"))) volatile uint32_t boot_usercode_address;

uint32_t runwordpadready = 0;
uint32_t current_scratchpad_size = 128;
volatile uint32_t cmd_len = 0;

volatile uint8_t reset_timeout = 0;

static inline void asmDelay(int delay) {
	asm volatile(
"1:	c.addi %[delay], -1\n"
"bne %[delay], x0, 1b\n" :[delay]"+r"(delay)  );
}

void boot_usercode()
{
	Delay_Ms(100);
#if !defined(CH5xx)
	FLASH->BOOT_MODEKEYR = FLASH_KEY1;
	FLASH->BOOT_MODEKEYR = FLASH_KEY2;
	FLASH->STATR = 0; // 1<<14 is zero, so, boot user code.
	PFIC->SCTLR = 1<<31;
#else
	USBFSReset();
	SYS_SAFE_ACCESS(R8_RST_WDOG_CTRL |= 1;);
#endif
}

__USBFS_FUN_ATTRIBUTE
void process_loop()
{
	int32_t localpad = (int32_t)SYSTICK_CNT;
	while(1)
	{
#if defined(BOOTLOADER_TIMEOUT_PWR_MS) && BOOTLOADER_TIMEOUT_PWR_MS
		if( localpad < 0 )
		{
			localpad = (int32_t)SYSTICK_CNT;
			if( localpad >= 0 )
			{
#if !defined(DISABLE_BOOTLOAD) || !DISABLE_BOOTLOAD
				BOOT_LED(0);
				// Boot to user program.
				boot_usercode();
#else
				localpad = 0;
#endif
			}
			if(reset_timeout) localpad = 0;
		}
#endif
		if( localpad > 0 )
		{
			if( --localpad == 0 )
			{
				/* Scratchpad strucure:
					4-bytes:		LONG( 0x000000aa )
						... code (this is executed) (120 bytes)
					4-bytes:        LONG( 0x1234abcd )

					After the scratchpad is the runpad, its structure is:
					4-bytes:   int32_t  if negative, how long to go before bootloading.  If 0, do nothing.  If positive, execute..
				*/
				typedef void (*setype)( uint32_t *, volatile int32_t * );
				setype scratchexec = (setype)(scratchpad+4);
				scratchexec( (uint32_t*)&scratchpad[0], &runwordpad );
				BOOT_LED(1);
			}
		}

		volatile uint32_t commandpad = runwordpad;
		if( commandpad )
		{
			BOOT_LED(0);
			localpad = commandpad-1;
			runwordpad = 0;
		}
	}
}

int main()
{
	SystemInit();
	runwordpad = 0;
	boot_usercode_address = (uint32_t)boot_usercode;
	SYSTICK_CNT = BOOTLOADER_TIMEOUT_BASE;
	funGpioInitAll();
#if FUNCONF_USE_UARTPRINTF
	SetupUART(FUNCONF_UART_PRINTF_BAUD);
#endif

#if defined(BOOTLOADER_LED_PIN)
	funPinMode(BOOTLOADER_LED_PIN, GPIO_CFGLR_OUT_10Mhz_PP);
#endif

#if BOOTLOADER_BTN_PIN && defined(BOOTLOADER_BTN_TRIG_LEVEL)

	// Configure the Bootloader Pin
	#if defined(BOOTLOADER_BTN_PULL)
		funPinMode(BOOTLOADER_BTN_PIN, GPIO_Speed_In | GPIO_CNF_IN_PUPD );
	#else
		funPinMode(BOOTLOADER_BTN_PIN, GPIO_Speed_In | GPIO_CNF_IN_FLOATING);
	#endif
	
	#if defined(BOOTLOADER_BTN_PULL)
	#if BOOTLOADER_BTN_PULL == 1
		funDigitalWrite(BOOTLOADER_BTN_PIN, 1);
	#else
		funDigitalWrite(BOOTLOADER_BTN_PIN, 0);
	#endif
	#endif

	asmDelay(1000000);

	#if !defined(DISABLE_BOOTLOAD) || !DISABLE_BOOTLOAD
	#if BOOTLOADER_BTN_TRIG_LEVEL == 0
		#if defined(SOFT_REBOOT_TO_BOOTLOADER) && !defined(CH5xx)
			if(funDigitalRead(BOOTLOADER_BTN_PIN) && !(RCC->RSTSCKR == 0x10000000)) boot_usercode();
		#elif defined(SOFT_REBOOT_TO_BOOTLOADER)
			if(funDigitalRead(BOOTLOADER_BTN_PIN) && (R8_RESET_STATUS & 0x7)) boot_usercode();
		#else
			if(funDigitalRead(BOOTLOADER_BTN_PIN)) boot_usercode();
		#endif
	#else
		#if defined(SOFT_REBOOT_TO_BOOTLOADER) && !defined(CH5xx)
			if(!funDigitalRead(BOOTLOADER_BTN_PIN) && !(RCC->RSTSCKR == 0x10000000)) boot_usercode();
		#elif defined(SOFT_REBOOT_TO_BOOTLOADER)
			if(!funDigitalRead(BOOTLOADER_BTN_PIN) && (R8_RESET_STATUS & 0x7)) boot_usercode();
		#else
			if(!funDigitalRead(BOOTLOADER_BTN_PIN)) boot_usercode();
		#endif
	#endif
	#endif
#endif
	USBFSSetup();
	// Bootloader timeout / localpad: 
	// localpad counting up to 0 is used for timeout
	// localpad transitioning from -1 to 0 boots user code
	// localpad set to 0 disables timeout
	// localpad counting down to 0 is used for executing code from scratchpad
	BOOT_LED(1);
	process_loop();
}

void HandleHidUserReportDataOut( struct _USBState * ctx, uint8_t * data, int len )
{
	return;
}

int HandleHidUserGetReportSetup( struct _USBState * ctx, tusb_control_request_t * req )
{
	if( req->wLength > SCRATCHPAD_SIZE ) req->wLength = SCRATCHPAD_SIZE;
	// The host wants to read back from us.
	
	ctx->pCtrlPayloadPtr = scratchpad;
	return req->wLength;
}

int HandleHidUserSetReportSetup( struct _USBState * ctx, tusb_control_request_t * req )
{
	if( req->wLength > SCRATCHPAD_SIZE ) req->wLength = SCRATCHPAD_SIZE;

	runwordpad = 1; //request stoppage.
	ctx->pCtrlPayloadPtr = scratchpad;
	cmd_len = req->wLength;
	return req->wLength;
}

int HandleHidUserReportDataIn( struct _USBState * ctx, uint8_t * data, int len )
{
	// You almost will never need this, in general, you will use HandleHidUserGetReportSetup.
	// printf( "IN %d %d %08x %08x\n", len, ctx->USBFS_SetupReqLen, data, USBFSCTX.endpoints[0].in );
//	memset( data, 0xcc, len );
	return len;
}

void HandleHidUserReportOutComplete( struct _USBState * ctx )
{
	uint32_t * last4 = (uint32_t*)&scratchpad[cmd_len-4];
	if( *last4 == 0x1234abcd )
	{
		*last4 = 0;
		runwordpad = 100;
		ctx->pCtrlPayloadPtr = 0;
	}

	return;
}
