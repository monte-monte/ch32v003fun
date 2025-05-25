#include "ch32fun.h"
#include <stdio.h>

#define LED PA8
#define LED_ON FUN_LOW
#define LED_OFF FUN_HIGH
#define SLEEPTIME_MS 3000

__INTERRUPT
void RTC_IRQHandler(void)
{
	// clear timer and trigger flags
	R8_RTC_FLAG_CTRL = (RB_RTC_TMR_CLR | RB_RTC_TRIG_CLR);
}

void blink_led(int n) {
	for(int i = n-1; i >= 0; i--) {
		funDigitalWrite( LED, LED_ON );
		LowPowerIdle( MS_TO_RTC(33) );
		funDigitalWrite( LED, LED_OFF );
		if(i) LowPowerIdle( MS_TO_RTC(33) );
	}
}

int main()
{
	SystemInit();

	DCDCEnable(); // Enable the internal DCDC
	LSIEnable(); // Disable LSE, enable LSI
	RTCInit(); // Set the RTC counter to 0
	SleepInit(); // Enable wakeup from sleep by RTC, and enable RTC IRQ

	funPinMode( LED, GPIO_CFGLR_OUT_2Mhz_PP );
	funDigitalWrite( LED, LED_OFF );
	blink_led(1);

	while(1)
	{
		// sleep according to the power plan in the second parameter, and wait for interrupt (WFI)
		LowPower( MS_TO_RTC(SLEEPTIME_MS), (RB_PWR_RAM2K | RB_PWR_RAM24K | RB_PWR_EXTEND | RB_XT_PRE_EN) );
		DCDCEnable(); // Sleep disables DCDC
		blink_led(2);

		// sleep with minimal power plan, and wait for interrupt (WFI)
		LowPower( MS_TO_RTC(SLEEPTIME_MS), (RB_PWR_RAM2K) );
		DCDCEnable(); // Sleep disables DCDC
		blink_led(3);
	}
}
