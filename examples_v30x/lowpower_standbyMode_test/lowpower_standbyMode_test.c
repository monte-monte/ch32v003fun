// Simple example showing how to use the low power standby mode
// The current consumption while sleep is around 2uA

// Setup:
// PB10 - LED indicator

// Program behavior:
// 1. 5 second delay to allow reprogramming
// 2. Set all GPIOs to input pull up
// 3. Enter standby
// 4. Wake up using the IWDG timer (after 8 seconds), the indicator Led will blink 5 times upon wakeup
// 5. Go back to sleep and repeat


#include "ch32fun.h"
#include <stdio.h>

#define LED_PIN PB10

void blink_led(int time) {
	for (int i = 0; i < time; i++) {
		funDigitalWrite(LED_PIN, 1);
		Delay_Ms( 50 );
		funDigitalWrite(LED_PIN, 0);
		Delay_Ms( 50 );
	}
}

void IWDG_init(u16 prescaler, u16 reload) {
	// 1. Enable access to IWDG registers
	IWDG->CTLR = IWDG_WriteAccess_Enable;
	
	// 2. Set prescaler and reload value
	IWDG->PSCR = prescaler;
	IWDG->RLDR = reload;

	// 3. Enable IWDG (cannot be disabled after this!)
	IWDG->CTLR = CTLR_KEY_Reload;
	IWDG->CTLR = CTLR_KEY_Enable;

	// 4. wait for LSI ready
	while(!(RCC->RSTSCKR & RCC_LSIRDY));
}

int main() {
	SystemInit();
	Delay_Ms(100);
	funGpioInitA();
	funGpioInitB();

	printf("\n~ Low Power Stop Mode Test ~\n");
	//! WARNING: need 5 seconds to allow reprogramming on power up
	Delay_Ms(5000);

	// Set all GPIOs to input pull up to reduce power consumption
	GPIOA->CFGLR = 0x88888888;
	GPIOB->CFGLR = 0x88888888;
	GPIOC->CFGLR = 0x88888888;
	GPIOD->CFGLR = 0x88888888;
	GPIOE->CFGLR = 0x88888888;

	GPIOA->CFGHR = 0x88888888;
	GPIOB->CFGHR = 0x88888888;
	GPIOC->CFGHR = 0x88888888;
	GPIOD->CFGHR = 0x88888888;
	GPIOE->CFGHR = 0x88888888;

	funPinMode(LED_PIN, GPIO_CFGLR_OUT_10Mhz_PP);
	blink_led(5);

	// Enable PWR clock
	RCC->APB1PCENR |= RCC_APB1Periph_PWR;

	// Enter standby mode	
	PWR->CTLR |= PWR_CTLR_PDDS;
	NVIC->SCTLR |= (1 << 2);

	// reload value for 8 second timeout = (8 * 40KHz / 128) - 1 = 2499
	IWDG_init(IWDG_Prescaler_128, 2499);
	__WFI();

	while(1);
}