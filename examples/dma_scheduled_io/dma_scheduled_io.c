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

void DMA1_Channel7_IRQHandler(void) __attribute__((interrupt));
void DMA1_Channel7_IRQHandler(void) { TIM2->CTLR1 = 0; DMA1->INTFCR = DMA_CTCIF7; }

// Max rate, ~4MSPS
// For reading from fixed values and writing them to hardware.
void RunScheduledDMABasic( const uint32_t * values, const volatile uint32_t ** addresses, const int time, int count )
{
	// Channel 7 @ (T2C2) load new address into Channel 2
	// Channel 2 @UP Write DMA

	// Will write output, but configured by other DMAs.
	DMA1_Channel2->CNTR = count;
	DMA1_Channel2->MADDR = (uint32_t)&values[0];
	DMA1_Channel2->CFGR = DMA_DIR_PeripheralDST | DMA_CFGR1_PL | DMA_CFGR1_MINC | DMA_CFGR1_EN | DMA_CFGR1_MSIZE_1 | DMA_CFGR1_PSIZE_1;

	DMA1_Channel7->CNTR = count;
	DMA1_Channel7->MADDR = (uint32_t)&addresses[0];
	DMA1_Channel7->PADDR = (uint32_t)&DMA1_Channel2->PADDR;
	DMA1_Channel7->CFGR = DMA_DIR_PeripheralDST | DMA_CFGR1_PL_0 | DMA_CFGR1_MINC | DMA_CFGR1_TCIE | DMA_CFGR1_EN | DMA_CFGR1_MSIZE_1 | DMA_CFGR1_PSIZE_1 | DMA_CFGR1_CIRC;

	TIM2->PSC = 1;

	// Reload immediately (This does not seem to be working?)  see the M2 we need to do in CLONESETM2?  But not having it is worse.
	TIM2->SWEVGR = TIM2_SWEVGR_UG;
	TIM2->DMAINTENR = TIM2_DMAINTENR_COMDE | TIM2_DMAINTENR_TDE |
		TIM2_DMAINTENR_UDE | TIM2_DMAINTENR_CC2DE;
	TIM2->CH2CVR = 1;		// Need to get loaded in on the first go.
	TIM2->ATRLR = time;       // Auto Reload - sets period  (This is how fast each pixel works per set)
	TIM2->CNT = 0;
	TIM2->CTLR1 = TIM2_CTLR1_CEN; // Enable

	__WFI(); // Optional, but this quiets down the bus because the CPU isn't constantly hitting it.
}

// Max rate, ~4MSPS
// For reading from fixed values and writing them to hardware (variable rate)
void RunScheduledDMA( const uint32_t * values, const volatile uint32_t ** addresses, const uint16_t * times, int count )
{
	// Channel 5 @ (T2C1) load new times to TIM2->ATLAR
	// Channel 7 @ (T2C2) load new address into Channel 2
	// Channel 2 @UP Write DMA

	// Will write output, but configured by other DMAs.
	DMA1_Channel2->CNTR = count;
	DMA1_Channel2->MADDR = (uint32_t)&values[0];
	DMA1_Channel2->CFGR = DMA_DIR_PeripheralDST | DMA_CFGR1_PL | DMA_CFGR1_MINC | DMA_CFGR1_EN | DMA_CFGR1_MSIZE_1 | DMA_CFGR1_PSIZE_1;

	DMA1_Channel7->CNTR = count;
	DMA1_Channel7->MADDR = (uint32_t)&addresses[0];
	DMA1_Channel7->PADDR = (uint32_t)&DMA1_Channel2->PADDR;
	DMA1_Channel7->CFGR = DMA_DIR_PeripheralDST | DMA_CFGR1_PL_0 | DMA_CFGR1_MINC | DMA_CFGR1_TCIE | DMA_CFGR1_EN | DMA_CFGR1_MSIZE_1 | DMA_CFGR1_PSIZE_1 | DMA_CFGR1_CIRC;

	DMA1_Channel5->CNTR = count;
	DMA1_Channel5->MADDR = (uint32_t)&times[0];
	DMA1_Channel5->PADDR = (uint32_t)&TIM2->ATRLR;
	DMA1_Channel5->CFGR = DMA_DIR_PeripheralDST | DMA_CFGR1_PL_1 | DMA_CFGR1_MINC | DMA_CFGR1_EN | DMA_CFGR1_MSIZE_1 | DMA_CFGR1_PSIZE_1 | DMA_CFGR1_CIRC;

	TIM2->PSC = 1;

	// Reload immediately (This does not seem to be working?)  see the M2 we need to do in CLONESETM2?  But not having it is worse.
	TIM2->SWEVGR = TIM2_SWEVGR_UG;
	TIM2->DMAINTENR = TIM2_DMAINTENR_COMDE | TIM2_DMAINTENR_TDE |
		TIM2_DMAINTENR_UDE | TIM2_DMAINTENR_CC2DE | TIM2_DMAINTENR_CC1DE;
	TIM2->CH1CVR = 1;
	TIM2->CH2CVR = 2;		// Need to get loaded in on the first go.
	TIM2->ATRLR = 32;       // Auto Reload - sets period  (This is how fast each pixel works per set)
	TIM2->CNT = 0;
	TIM2->CTLR1 = TIM2_CTLR1_CEN; // Enable

	__WFI(); // Optional, but this quiets down the bus because the CPU isn't constantly hitting it.
}

// For arbitrary IO.
// Practically 2MSPS max limit.
void RunScheduledDMAFullControl( const volatile uint32_t ** fromaddresses, const volatile uint32_t ** addresses, const uint16_t * times, int count )
{
	// Channel 5 @ (T2C1) load new times to TIM2->ATLAR
	// Channel 7 @ (T2C2) load new address into Channel 2
	// Channel 1 @ (T2C3) load new other address into Channel 2.
	// Channel 2 @UP Write DMA

	DMA1_Channel7->CNTR = count;
	DMA1_Channel7->MADDR = (uint32_t)&addresses[0];
	DMA1_Channel7->PADDR = (uint32_t)&DMA1_Channel2->PADDR;
	DMA1_Channel7->CFGR = DMA_DIR_PeripheralDST | DMA_CFGR1_PL_0 | DMA_CFGR1_MINC | DMA_CFGR1_TCIE | DMA_CFGR1_EN | DMA_CFGR1_MSIZE_1 | DMA_CFGR1_PSIZE_1 | DMA_CFGR1_CIRC;

	DMA1_Channel5->CNTR = count;
	DMA1_Channel5->MADDR = (uint32_t)&times[0];
	DMA1_Channel5->PADDR = (uint32_t)&TIM2->ATRLR;
	DMA1_Channel5->CFGR = DMA_DIR_PeripheralDST | DMA_CFGR1_PL_1 | DMA_CFGR1_MINC | DMA_CFGR1_EN | DMA_CFGR1_MSIZE_1 | DMA_CFGR1_PSIZE_1 | DMA_CFGR1_CIRC;

	DMA1_Channel1->CNTR = count;
	DMA1_Channel1->MADDR = (uint32_t)&fromaddresses[0];
	DMA1_Channel1->PADDR = (uint32_t)&DMA1_Channel2->MADDR;
	DMA1_Channel1->CFGR = DMA_DIR_PeripheralDST | 0 | DMA_CFGR1_MINC | DMA_CFGR1_EN | DMA_CFGR1_MSIZE_1 | DMA_CFGR1_PSIZE_1 | DMA_CFGR1_CIRC;

	// Will write output, but configured by other DMAs.
	DMA1_Channel2->CNTR = 1;
	DMA1_Channel2->CFGR = DMA_DIR_PeripheralDST | DMA_CFGR1_PL | DMA_CFGR1_EN | DMA_CFGR1_MSIZE_1 | DMA_CFGR1_PSIZE_1 | DMA_CFGR1_CIRC;


	TIM2->PSC = 1;  // For whatever reason this seems to be more reliable and go faster if this is set to 1.

	// Reload immediately (This does not seem to be working?)  see the M2 we need to do in CLONESETM2?  But not having it is worse.
	TIM2->SWEVGR = TIM2_SWEVGR_UG;
	TIM2->DMAINTENR = TIM2_DMAINTENR_COMDE | TIM2_DMAINTENR_TDE |
		TIM2_DMAINTENR_UDE | TIM2_DMAINTENR_CC3DE | TIM2_DMAINTENR_CC2DE | TIM2_DMAINTENR_CC1DE;
	TIM2->CH1CVR = 1; // This is the trigger
	TIM2->CH2CVR = 2;
	TIM2->CH3CVR = 3;		// Need to get loaded in on the first go.
	TIM2->ATRLR = times[0];       // Auto Reload - sets period  (This is how fast each pixel works per set)
	TIM2->CNT = 0;
	TIM2->CTLR1 = TIM2_CTLR1_CEN; // Enable

	__WFI(); // Optional, but this quiets down the bus because the CPU isn't constantly hitting it.
}



int main()
{
	SystemInit();
	
	// Enable GPIOD, C and ADC
	RCC->APB2PCENR |= RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOD |
		RCC_APB2Periph_GPIOC | RCC_APB2Periph_ADC1;

	funPinMode( PC0, GPIO_CFGLR_OUT_10Mhz_PP );
	funPinMode( PC1, GPIO_CFGLR_OUT_10Mhz_PP );

	RCC->APB1PCENR |= RCC_APB1Periph_TIM2;
	RCC->AHBPCENR |= RCC_AHBPeriph_DMA1;

	NVIC_EnableIRQ(DMA1_Channel7_IRQn);

	// Only put these in flash if you're going at < 1MSPS.

	// Create a pattern on PC0, PC1.
	uint32_t values   [] = { 3, 2, 1, 0, 3, 2, 2, 3, 0, 1, 2, 3 };

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

	// Minimum value 5 for Basic
	// Minimum value 5 for Regular
	// Minimum value 11 for full DMA
	// But, for full control, you have to slow dow nto about 23.
	// This "time" is actually+1 for timer ATLAR.
	// First time is not precise.
	// You have to test these on your device for stability
	// and precision to avoid clock jitter.
	uint32_t times[] = {
		5, 5, 5, 5,
		5, 5, 5, 5,
		5, 5, 5, 5 }; 


	int count = sizeof(times)/sizeof(times[0]);

	while(1)
	{
		//RunScheduledDMABasic( values, addresses, 5, count );
		RunScheduledDMA( values, addresses, times, count );
		//RunScheduledDMAFullControl( fromaddresses, addresses, times, count );

		// Do a little dance to let us see where the pattern starts.
		GPIOC->OUTDR = 0x00;
		GPIOC->OUTDR = 0xff;
		GPIOC->OUTDR = 0x00;
		GPIOC->OUTDR = 0xff;
	}

}

