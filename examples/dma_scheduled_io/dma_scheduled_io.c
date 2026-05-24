/*
	DMA-scheduled IO example

	This uses the DMA and Timer 2 to schedule a series of IO operations.
	Tested on the ch32v006, but should also work on the ch32v003.

	In general, it lets you set up a series of operations where you set
	some register values to other register values. 

	It shows multiple possible avenues of use, both for a sort of write-only
	system, as well as a system that lets you arbitrarily read/write from/to
	anywhre at specific times.
*/

#include "ch32fun.h"
#include <stdio.h>

#ifdef CH32V003
#define TIM1_SWEVGR_UG TIM_UG
#define TIM1_DMAINTENR_COMDE TIM_COMDE
#define TIM1_DMAINTENR_TDE TIM_TDE
#define TIM1_DMAINTENR_UDE TIM_UDE
#define TIM1_DMAINTENR_CC1DE TIM_CC1DE
#define TIM1_DMAINTENR_CC2DE TIM_CC2DE
#define TIM1_DMAINTENR_CC3DE TIM_CC3DE
#define TIM1_DMAINTENR_CC4DE TIM_CC4DE
#define TIM1_CTLR1_CEN TIM_CEN

#define RESETISRDONEFLAG asm volatile( "li x4, 0" );
#define WAITFORISRDONE asm volatile( "1: beqz x4, 1b" );
#define ISRCONT asm volatile( "addi x4, x4, 1" );
#else
#define ISRCONT
#endif

void DMA1_Channel3_IRQHandler(void) __attribute__((interrupt));
void DMA1_Channel3_IRQHandler(void) { TIM1->CTLR1 = 0; DMA1->INTFCR = DMA_CTCIF3; ISRCONT }

// Max rate, ~4MSPS
// For reading from fixed values and writing them to hardware.
void RunScheduledDMABasic( const uint32_t * values, volatile uint32_t ** addresses, const int time, int count )
{
	// Will write output, but configured by other DMAs.
	DMA1_Channel2->CNTR = count;
	DMA1_Channel2->MADDR = (uint32_t)&values[0];
	DMA1_Channel2->CFGR = DMA_DIR_PeripheralDST | DMA_CFGR1_PL | DMA_CFGR1_MINC | DMA_CFGR1_EN | DMA_CFGR1_MSIZE_1 | DMA_CFGR1_PSIZE_1;

	DMA1_Channel3->CNTR = count;
	DMA1_Channel3->MADDR = (uint32_t)&addresses[0];
	DMA1_Channel3->PADDR = (uint32_t)&DMA1_Channel2->PADDR;
	DMA1_Channel3->CFGR = DMA_DIR_PeripheralDST | DMA_CFGR1_PL_0 | DMA_CFGR1_MINC | DMA_CFGR1_TCIE | DMA_CFGR1_EN | DMA_CFGR1_MSIZE_1 | DMA_CFGR1_PSIZE_1 | DMA_CFGR1_CIRC;

	TIM1->PSC = 1;

	TIM1->SWEVGR = TIM1_SWEVGR_UG;
	TIM1->DMAINTENR = TIM1_DMAINTENR_COMDE | TIM1_DMAINTENR_TDE |
		TIM1_DMAINTENR_CC1DE | TIM1_DMAINTENR_CC2DE;
	TIM1->CH1CVR = 5; // T1C1 -> DMACH2
	TIM1->CH2CVR = 1; // T1C2 -> DMACH3
	TIM1->ATRLR = time;       // Auto Reload - sets period  (This is how fast each pixel works per set)
	TIM1->CNT = time-2;
	TIM1->CTLR1 = TIM1_CTLR1_CEN; // Enable

#ifdef CH32V003
	RESETISRDONEFLAG
	WAITFORISRDONE
#else
	__WFI(); // Optional, but this quiets down the bus because the CPU isn't constantly hitting it.
#endif
}

// Max rate, ~4MSPS
// For reading from fixed values and writing them to hardware (variable rate)
void RunScheduledDMA( uint32_t * values, volatile uint32_t ** addresses, const uint32_t * times, int count )
{
	// Will write output, but configured by other DMAs.
	DMA1_Channel2->CNTR = count;
	DMA1_Channel2->MADDR = (uint32_t)&values[0];
	DMA1_Channel2->CFGR = DMA_DIR_PeripheralDST | DMA_CFGR1_PL | DMA_CFGR1_MINC | DMA_CFGR1_EN | DMA_CFGR1_MSIZE_1 | DMA_CFGR1_PSIZE_1;

	DMA1_Channel3->CNTR = count;
	DMA1_Channel3->MADDR = (uint32_t)&addresses[0];
	DMA1_Channel3->PADDR = (uint32_t)&DMA1_Channel2->PADDR;
	DMA1_Channel3->CFGR = DMA_DIR_PeripheralDST | DMA_CFGR1_PL_0 | DMA_CFGR1_MINC | DMA_CFGR1_TCIE | DMA_CFGR1_EN | DMA_CFGR1_MSIZE_1 | DMA_CFGR1_PSIZE_1 | DMA_CFGR1_CIRC;

	DMA1_Channel6->CNTR = count;
	DMA1_Channel6->MADDR = (uint32_t)&times[0];
	DMA1_Channel6->PADDR = (uint32_t)&TIM1->ATRLR;
	DMA1_Channel6->CFGR = DMA_DIR_PeripheralDST | DMA_CFGR1_PL_1 | DMA_CFGR1_MINC | DMA_CFGR1_EN | DMA_CFGR1_MSIZE_1 | DMA_CFGR1_PSIZE_0 | DMA_CFGR1_CIRC;

	TIM1->PSC = 1;

	// Reload immediately (This does not seem to be working?)  see the M2 we need to do in CLONESETM2?  But not having it is worse.
	TIM1->SWEVGR = TIM1_SWEVGR_UG;
	TIM1->DMAINTENR = TIM1_DMAINTENR_COMDE | TIM1_DMAINTENR_TDE |
		TIM1_DMAINTENR_CC3DE | TIM1_DMAINTENR_CC2DE | TIM1_DMAINTENR_CC1DE;
	TIM1->CH1CVR = 5; // T1C1 -> DMACH2
	TIM1->CH2CVR = 1; // T1C2 -> DMACH3
	TIM1->CH3CVR = 3; // T1C3 -> DMACH6
	TIM1->ATRLR = 32;       // Auto Reload - sets period  (This is how fast each pixel works per set)
	TIM1->CNT = 32-2;
	TIM1->CTLR1 = TIM1_CTLR1_CEN; // Enable

#ifdef CH32V003
	RESETISRDONEFLAG
	WAITFORISRDONE
#else
	__WFI(); // Optional, but this quiets down the bus because the CPU isn't constantly hitting it.
#endif
}

// For arbitrary IO.
// Practically 2MSPS max limit.
void RunScheduledDMAFullControl( volatile uint32_t ** fromaddresses, volatile uint32_t ** addresses, const uint32_t * times, int count )
{
	DMA1_Channel3->CNTR = count;
	DMA1_Channel3->MADDR = (uint32_t)&addresses[0];
	DMA1_Channel3->PADDR = (uint32_t)&DMA1_Channel2->PADDR;
	DMA1_Channel3->CFGR = DMA_DIR_PeripheralDST | DMA_CFGR1_PL_0 | DMA_CFGR1_MINC | DMA_CFGR1_TCIE | DMA_CFGR1_EN | DMA_CFGR1_MSIZE_1 | DMA_CFGR1_PSIZE_1 | DMA_CFGR1_CIRC;

	DMA1_Channel6->CNTR = count;
	DMA1_Channel6->MADDR = (uint32_t)&times[0];
	DMA1_Channel6->PADDR = (uint32_t)&TIM1->ATRLR;
	DMA1_Channel6->CFGR = DMA_DIR_PeripheralDST | DMA_CFGR1_PL_1 | DMA_CFGR1_MINC | DMA_CFGR1_EN | DMA_CFGR1_MSIZE_1 | DMA_CFGR1_PSIZE_0 | DMA_CFGR1_CIRC;

	DMA1_Channel4->CNTR = count;
	DMA1_Channel4->MADDR = (uint32_t)&fromaddresses[0];
	DMA1_Channel4->PADDR = (uint32_t)&DMA1_Channel2->MADDR;
	DMA1_Channel4->CFGR = DMA_DIR_PeripheralDST | 0 | DMA_CFGR1_MINC | DMA_CFGR1_EN | DMA_CFGR1_MSIZE_1 | DMA_CFGR1_PSIZE_1 | DMA_CFGR1_CIRC;

	// Will write output, but configured by other DMAs.
	DMA1_Channel2->CNTR = count;
	DMA1_Channel2->CFGR = DMA_DIR_PeripheralDST | DMA_CFGR1_PL | DMA_CFGR1_EN | DMA_CFGR1_MSIZE_1 | DMA_CFGR1_PSIZE_1;

	// Reload immediately (This does not seem to be working?)  see the M2 we need to do in CLONESETM2?  But not having it is worse.
	TIM1->SWEVGR = TIM1_SWEVGR_UG;
	TIM1->DMAINTENR = TIM1_DMAINTENR_COMDE | TIM1_DMAINTENR_TDE |
		TIM1_DMAINTENR_CC4DE | TIM1_DMAINTENR_CC3DE | TIM1_DMAINTENR_CC2DE | TIM1_DMAINTENR_CC1DE;
	TIM1->CH1CVR = 8; // T1C1 -> DMACH2
	TIM1->CH2CVR = 3; // T1C2 -> DMACH3
	TIM1->CH3CVR = 1; // T1C3 -> DMACH6
	TIM1->CH4CVR = 5; // T1C4 -> DMACH4
	TIM1->ATRLR = times[0];       // Auto Reload - sets period  (This is how fast each pixel works per set)
	TIM1->CNT = times[0]-2;
	TIM1->CTLR1 = TIM1_CTLR1_CEN; // Enable
#ifdef CH32V003
	RESETISRDONEFLAG
	WAITFORISRDONE
#else
	__WFI(); // Optional, but this quiets down the bus because the CPU isn't constantly hitting it.
#endif
}



int main()
{
	SystemInit();
	
	// Enable GPIOD, C and ADC
	RCC->APB2PCENR |= RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOC | RCC_APB2Periph_ADC1 | RCC_APB2Periph_TIM1;

	funPinMode( PC0, GPIO_CFGLR_OUT_10Mhz_PP );
	funPinMode( PC1, GPIO_CFGLR_OUT_10Mhz_PP );

	RCC->AHBPCENR  |= RCC_AHBPeriph_DMA1;

	NVIC_EnableIRQ(DMA1_Channel3_IRQn);

	// Only put these in flash if you're going at < 1MSPS.

	// Create a pattern on PC0, PC1.
	uint32_t values   [] = { 1, 3, 0, 3, 1, 2, 1, 3, 0, 3, 1, 2 };

	// Other values
	uint32_t valuesb  [] = { 1, 2, 1, 2, 1, 2, 0, 1, 2, 3, 2, 3 };

	volatile uint32_t * fromaddresses[] = {
		&valuesb[0], &valuesb[1], &valuesb[2], &valuesb[3],
		&valuesb[4], &valuesb[5], &valuesb[6], &valuesb[7],
		&valuesb[8], &valuesb[9], &valuesb[10], &valuesb[11] };

	volatile uint32_t * addresses[] = { 
		&GPIOC->OUTDR, &GPIOC->OUTDR, &GPIOC->OUTDR, &GPIOC->OUTDR,
		&GPIOC->OUTDR, &GPIOC->OUTDR, &GPIOC->OUTDR, &GPIOC->OUTDR,
		&GPIOC->OUTDR, &GPIOC->OUTDR, &GPIOC->OUTDR, &GPIOC->OUTDR,
	};

	// Not tested for reliability, while some could go faster,
	//  I did want to say what would be stable from my testing here.
	//
	// Lowest times:        003    006
	// RunScheduledDMABasic  5     5
	// RunScheduledDMA       8     8
	// RunScheduledDMAFull   11    11
	//
	//
	//  Speed = 48 / (Time + 1)
	//
	//
	// These are the lengths of time to dwell at each state.
	// for the ch32v003/ch32v006, this is at 48MHz increments.
	//
	// But, for full control, you have to slow dow nto about 23.
	// This "time" is actually+1 for timer ATLAR.
	// First time is not precise.
	// You have to test these on your device for stability
	// and precision to avoid clock jitter.
	uint32_t times[] = {
		8, 8, 8, 8,
		8, 8, 8, 8,
		8, 8, 8, 8 }; 


	int count = sizeof(times)/sizeof(times[0]);

	while(1)
	{
		RunScheduledDMABasic( values, addresses, 4, count );
		//RunScheduledDMA( values, addresses, times, count );
		//RunScheduledDMAFullControl( fromaddresses, addresses, times, count );

		// Do a little dance to let us see where the pattern starts.
		GPIOC->OUTDR = 0x00;
		GPIOC->OUTDR = 0xff;
		GPIOC->OUTDR = 0x00;
		GPIOC->OUTDR = 0xff;
		GPIOC->OUTDR = 0x00;
		GPIOC->OUTDR = 0xff;
		GPIOC->OUTDR = 0x00;
		GPIOC->OUTDR = 0xff;
	}

}

