/*
	DMA-scheduled IO example

	This uses the DMA and Timer 2 to 
*/

#include "ch32fun.h"
#include <stdio.h>

void DMA1_Channel2_IRQHandler(void) __attribute__((interrupt));
void DMA1_Channel2_IRQHandler(void) { TIM2->CTLR1 = 0; DMA1->INTFCR = DMA_CTCIF2; }

void RunScheduledDMA( uint32_t * values, volatile uint32_t ** addresses, uint16_t * times, int count )
{
	// Channel 5 @2 (T2C1) load new times to TIM2->ATLAR
	// Channel 7 @4 load new address into Channel 2
	// Channel 2 @UP Write DMA

	// Will write output, but configured by other DMAs.
	DMA1_Channel2->CNTR = count;
	DMA1_Channel2->MADDR = (uint32_t)&values[0];
	DMA1_Channel2->CFGR = DMA_DIR_PeripheralDST | DMA_CFGR1_PL | DMA_CFGR1_MINC | DMA_CFGR1_TCIE | DMA_CFGR1_EN | DMA_CFGR1_MSIZE_1 | DMA_CFGR1_PSIZE_1;

	DMA1_Channel7->CNTR = count;
	DMA1_Channel7->MADDR = (uint32_t)&addresses[0];
	DMA1_Channel7->PADDR = (uint32_t)&DMA1_Channel2->PADDR;
	DMA1_Channel7->CFGR = DMA_DIR_PeripheralDST | DMA_CFGR1_PL | DMA_CFGR1_MINC | DMA_CFGR1_EN | DMA_CFGR1_MSIZE_1 | DMA_CFGR1_PSIZE_1 | DMA_CFGR1_CIRC;

	DMA1_Channel5->CNTR = count;
	DMA1_Channel5->MADDR = (uint32_t)&times[0];
	DMA1_Channel5->PADDR = (uint32_t)&TIM2->ATRLR;
	DMA1_Channel5->CFGR = DMA_DIR_PeripheralDST | DMA_CFGR1_PL | DMA_CFGR1_MINC | DMA_CFGR1_EN | DMA_CFGR1_MSIZE_0 | DMA_CFGR1_PSIZE_0 | DMA_CFGR1_CIRC;

	TIM2->PSC = 0;

	// Reload immediately (This does not seem to be working?)  see the M2 we need to do in CLONESETM2?  But not having it is worse.
	TIM2->SWEVGR = 0;//TIM2_SWEVGR_UG;
	TIM2->DMAINTENR = TIM2_DMAINTENR_COMDE | TIM2_DMAINTENR_TDE |
		TIM2_DMAINTENR_UDE | TIM2_DMAINTENR_CC2DE | TIM2_DMAINTENR_CC1DE;
	TIM2->CH1CVR = 2;
	TIM2->CH2CVR = 3;
	TIM2->ATRLR = 16;       // Auto Reload - sets period  (This is how fast each pixel works per set)
	TIM2->CNT = 1;
	TIM2->CTLR1 = TIM2_CTLR1_CEN; // Enable
	__WFI();
}

int main()
{
	SystemInit();
	
	// Enable GPIOD, C and ADC
	RCC->APB2PCENR |= RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOD |
		RCC_APB2Periph_GPIOC | RCC_APB2Periph_ADC1;

	funPinMode( PC0, GPIO_CFGLR_OUT_10Mhz_PP );
	funPinMode( PC1, GPIO_CFGLR_OUT_10Mhz_PP );

	uint32_t values   [] = { 0x00, 0x01, 0x02, 0x03, 0, 1, 2, 3, 0, 1, 2, 3 };
	volatile uint32_t * addresses[] = { 
		&GPIOC->OUTDR, &GPIOC->OUTDR, &GPIOC->OUTDR, &GPIOC->OUTDR,
		&GPIOC->OUTDR, &GPIOC->OUTDR, &GPIOC->OUTDR, &GPIOC->OUTDR,
		&GPIOC->OUTDR, &GPIOC->OUTDR, &GPIOC->OUTDR, &GPIOC->OUTDR
	};
	uint16_t times    [] = { 15, 15, 15, 15,15, 15, 15, 15, 15, 15, 15 }; // Minimum value 11, it gets stable around 16.


	RCC->APB1PCENR |= RCC_APB1Periph_TIM2;
	RCC->AHBPCENR |= RCC_AHBPeriph_DMA1;
	NVIC_EnableIRQ(DMA1_Channel2_IRQn);

	int count = sizeof(times)/sizeof(times[0]);

	while(1)
	{
		RunScheduledDMA( values, addresses, times, count );
	}

}

