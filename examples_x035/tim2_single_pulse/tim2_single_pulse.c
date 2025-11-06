#include "ch32fun.h"
#include <stdio.h>

// T1C1: PB9
// T1C2: PB10
// T1C3: PB11
// T1C4: PC16/PC11

// T2C1: PA0
// T2C2: PA1
// T2C3: PA2
// T2C4: PA3

#define INPUT_PIN PA1		// CH2
#define PWM_PIN PA0			// CH1

u16 period = 1000;

void TIM2_single_pulse_init() {
	// Enable TIM2 clock
	RCC->APB1PCENR |= RCC_APB1Periph_TIM2;
	
	//# Timer base configuration
	// TIM2->CTLR1 &= ~(TIM_DIR | TIM_CMS | TIM_CTLR1_CKD);
	// TIM2->CTLR1 |= TIM_CounterMode_Up | TIM_CKD_DIV1;
	// TIM2->PSC = 48000-1;			// Prescaler
	// TIM2->ATRLR = 200;			// PWM period

	TIM2->CHCTLR1 &= ~(TIM_CC1S | TIM_OC1M);
	TIM2->CHCTLR1 |= TIM_OCMode_PWM2;
    TIM2->PSC = 48000-1;
	TIM2->ATRLR = 200;

	TIM2->SWEVGR = TIM_UG;

	//# TIM2 OC1 configuration
	TIM2->CHCTLR1 &= ~(TIM_CC1S | TIM_OC1M);
	TIM2->CHCTLR1 |= TIM_OCMode_PWM2;
	TIM2->CH1CVR = 100;
	TIM2->CCER &= ~(TIM_CC1P | TIM_CC1E | TIM_CC1NP | TIM_CC1NE);
	TIM2->CCER |= TIM_OCPolarity_High | TIM_OutputState_Enable;

	TIM2->BDTR |= TIM_MOE;

	//# TIM2 config
	TIM2->CHCTLR1 &= ~(TIM_CC2S | TIM_IC2F);
	TIM2->CHCTLR1 |= 0x00 << 12;
	TIM2->CHCTLR1 |= TIM_ICSelection_DirectTI << 8;
    TIM2->CCER &= ~(TIM_CC2P | TIM_CC2E);
	TIM2->CCER |= TIM_CC2E | (TIM_ICPolarity_Falling << 4);

	//# TIM2 prescaler
	TIM2->CHCTLR1 &= ~(TIM_IC2PSC);
	TIM2->CHCTLR1 |= TIM_ICPSC_DIV1 << 8;

	//# Configure TIM2 for One Pulse Mode
	TIM2->CTLR1 &= ~TIM_OPM;
	TIM2->CTLR1 |= TIM_OPMode_Single;

	//# Configure TIM2 to be triggered by TI2 input
	TIM2->SMCFGR &= ~(TIM_TS);
	TIM2->SMCFGR |= TIM_TS_TI2FP2;

	//# Select Slave Mode: Trigger Mode
	TIM2->SMCFGR &= ~(TIM_SMS);
	TIM2->SMCFGR |= TIM_SlaveMode_Trigger;

	//# Main output enable and update
	TIM2->CTLR1 |= TIM_CEN;			// Start timer
}


int main() {
	SystemInit();
	funGpioInitAll(); // Enable GPIOs

	printf("\n~ ADC Single Pulse Example ~\n");
	printf("Chip ID: %08lX\n", ESIG->UID0);
	printf("Chip Capacity: %d KB\n", ESIG->CAP);

	// TIM2 CH4 output
	funPinMode(PWM_PIN, GPIO_CFGLR_OUT_50Mhz_AF_PP);
	funPinMode(INPUT_PIN, GPIO_CFGLR_IN_PUPD);
	TIM2_single_pulse_init();


	printf("\nCFGLR: 0x%08X\n", GPIOA->CFGLR);
	printf("CFGHR: 0x%08X\n", GPIOA->CFGHR);
	printf("CFGXR: 0x%08X\n", GPIOA->CFGXR);

	// TIM_TimeBaseInit
	// printf("CTLR1: 0x%04X\n", TIM2->CTLR1);
	printf("\nATRLR: 0x%04X\n", TIM2->ATRLR);
	printf("PSC: 0x%04X\n", TIM2->PSC);
	printf("SWEVGR: 0x%04X\n", TIM2->SWEVGR);

	// TIM_OC1Init
	printf("\nCTLR2: 0x%08X\n", TIM2->CTLR2);
	// printf("CHCTLR1: 0x%04X\n", TIM2->CHCTLR1);
	printf("CH1CVR: 0x%04X\n", TIM2->CH1CVR);
	// printf("CCER: 0x%04X\n", TIM2->CCER);

	printf("BDTR: 0x%04X\n", TIM2->BDTR);

	// TIM_ICInit
	printf("\nCHCTLR1: 0x%04X\n", TIM2->CHCTLR1);
	printf("CCER: 0x%04X\n", TIM2->CCER);

	printf("CTLR1: 0x%08X\n", TIM2->CTLR1);
	printf("SMCFGR: 0x%04X\n", TIM2->SMCFGR);


	// fade blocking loop
	while(1) {
		// printf("read: %u\n", funDigitalRead(INPUT_PIN));
		// Delay_Ms(1000);
	}
}
