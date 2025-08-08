#include "ch32fun.h"
#include "iSLER.h"
#include <stdio.h>

#ifdef CH570_CH572 // this comes from iSLER.h
#define LED PA9
#else
#define LED PA8
#endif

#define SLEEPTIME_MS 300

uint8_t adv[] = {0x66, 0x55, 0x44, 0x33, 0x22, 0x11, // MAC (reversed)
				 0x03, 0x19, 0x00, 0x00, // 0x19: "Appearance", 0x00, 0x00: "Unknown"
				 0x08, 0x09, 'c', 'h', '3', '2', 'f', 'u', 'n'}; // 0x09: "Complete Local Name"
uint8_t adv_channels[] = {37,38,39};

void blink(int n) {
	for(int i = n-1; i >= 0; i--) {
		funDigitalWrite( LED, FUN_LOW ); // Turn on LED
		Delay_Ms(33);
		funDigitalWrite( LED, FUN_HIGH ); // Turn off LED
		if(i) Delay_Ms(33);
	}
}

int main()
{
	SystemInit();

	funGpioInitAll();
	funPinMode( LED, GPIO_CFGLR_OUT_2Mhz_PP );

	RFCoreInit(LL_TX_POWER_0_DBM);

	blink(5);
	printf(".~ ch32fun BLE beacon ~.\n");

	for(int c = 0; c < sizeof(adv_channels); c++) {
		Frame_TX(adv, sizeof(adv), adv_channels[c], PHY_1M);
	}

	while(1) {
		blink(1);
		for(int c = 0; c < sizeof(adv_channels); c++) {
			Frame_TX(adv, sizeof(adv), adv_channels[c], PHY_1M);
		}
		LowPower( MS_TO_RTC(SLEEPTIME_MS), (RB_PWR_RAM12K | RB_PWR_EXTEND | RB_XT_PRE_EN) );
		RFWakeup();
		DCDCEnable();
	}
}
