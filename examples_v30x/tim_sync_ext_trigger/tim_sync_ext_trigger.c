// Simple example that shows how to use the TIM1 for external trigger
// which is used to start both TIM1 and TIM2.
// Here the TIM1 is set as a slave to the TI1FP1 filtered TI1 signal source,
// It is also configure as a master that triggers TIM2 via the TRGO trigger output of the ITR0 internal trigger.

// program behavior:
// when the TIM1 input trigger PA8 goes low, TIM1 and TIM2 starts counting.

#include "ch32fun.h"
#include <stdio.h>

void TIM_init(TIM_TypeDef * TIMx, u16 prescaler, u16 period) {
	// Enable TIMER clock
	if (TIMx == TIM1) {
		RCC->APB2PCENR |= RCC_APB2Periph_TIM1;
	}
	#ifdef TIM2
	else if (TIMx == TIM2) {
		RCC->APB1PCENR |= RCC_APB1Periph_TIM2;
	}
	#endif

	TIMx->CTLR1 = (TIMx->CTLR1 & ~TIM_DIR) | TIM_CounterMode_Up;
	TIMx->PSC = prescaler-1;
	TIMx->ATRLR = period;
	TIMx->SWEVGR = TIM_UG;
}

void timers_external_trigger_setup() {
	TIM_init(TIM1, 48000, 0xFFFF);
	TIM_init(TIM2, 48000, 0xFFFF);

	// CompareCaptureControlReg1: select CompareCapture Channel 1 as input
	TIM1->CHCTLR1 &= ~(TIM_CC1S);
	TIM1->CHCTLR1 |= TIM_ICSelection_DirectTI;
	// CompareCaptureEnableReg: polarity use CompareCapture Channel 1 for capture on falling edge
	TIM1->CCER &= ~(TIM_CC1P);
	TIM1->CCER |= (TIM_ICPolarity_Falling) | TIM_CC1E;

	// TIM1 slave mode configuration: use TI1FP1 (filtered TI1 signal) as trigger source
	// Sets trigger slave mode - TIM1 starts/resets on external trigger
	TIM1->SMCFGR = (TIM1->SMCFGR & ~TIM_TS) | TIM_TS_TI1FP1;
	TIM1->SMCFGR |= (TIM1->SMCFGR & ~TIM_SMS) | TIM_SlaveMode_Trigger;

	// TIM1 master mode configuration: TIM1 outputs TRGO (trigger output) signal when it starts
	TIM1->SMCFGR |= (TIM1->SMCFGR & ~TIM_MSM) | TIM_MasterSlaveMode_Enable;
    TIM1->CTLR2 |= (TIM1->CTLR2 & ~TIM_MMS) | TIM_TRGOSource_Enable;

	// TIM2 slave mode configuration: use ITR0 (internal trigger 0) as trigger source
	// See Figure 15-2 GTPM internal trigger connection in the Reference Manual
	TIM2->SMCFGR = (TIM2->SMCFGR & ~TIM_TS) | TIM_TS_ITR0;
	TIM2->SMCFGR |= (TIM2->SMCFGR & ~TIM_SMS) | TIM_SlaveMode_Trigger;
}

int main() {
	SystemInit();
	funGpioInitAll(); // Enable GPIOs

	printf("\n~ ADC Group Injection Example ~\n");
	printf("Chip ID: %08lX\n", ESIG->UID0);
	printf("Chip Capacity: %d KB\n", ESIG->CAP);

	// TIM2 CH2 input
	funPinMode(PA8, GPIO_CFGLR_IN_PUPD);
	funDigitalWrite(PA8, 1); // Pull-up
	timers_external_trigger_setup();

	while(1) {
		printf("TIM1: %d\n", TIM1->CNT);
		printf("TIM2: %d\n", TIM2->CNT);
		Delay_Ms(1000);
	}
}
