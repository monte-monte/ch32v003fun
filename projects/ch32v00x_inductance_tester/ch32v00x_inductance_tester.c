/*
	DMA-scheduled IO example

	This uses the DMA and Timer 2 to schedule a series of IO operations.
	Tested on the ch32v006, but should also work on the ch32v003.

	In general, it lets you set up a series of operations where you set
	some register values to other register values. 

	It shows multiple possible avenues of use, both for a sort of write-only
	system, as well as a system that lets you arbitrarily read/write from/to
	anywhre at specific times.

	TODO: On ch32v003, I could not find a way to `wfi`
*/

#include "ch32fun.h"
#include <stdio.h>

#define COIL_ON_ADC_NUMBER 3
#define COIL_ON_PORT       GPIOD
#define COIL_ON_PIN        2
#define EXTENDED_MODE 1

#ifdef CH32V003
#define TIM1_SWEVGR_UG TIM_UG
#define TIM1_DMAINTENR_COMDE TIM_COMDE
#define TIM1_DMAINTENR_TDE TIM_TDE
#define TIM1_DMAINTENR_UDE TIM_UDE
#define TIM1_DMAINTENR_CC3DE TIM_CC3DE
#define TIM1_DMAINTENR_CC2DE TIM_CC2DE
#define TIM1_DMAINTENR_CC1DE TIM_CC1DE
#define TIM1_CTLR1_CEN TIM_CEN
#define ADC_EXTSEL_SWSTART CTLR2_EXTTRIG_SWSTART_Set
#define ADC_BUFEN 0

// Macro used for force-alingining ADC timing, from
// touch code.  This is needed, because on the 003
// even if we compensate for the cycle offset, there
// is a far sub-clock phase offset of about 2 degrees
// that causes some noise on the output.
//
// This doesn't work on the 006, but it doesn't need
// to.  On the 006, it seems to be precisely aligned
// anyway.
#define FORCEALIGNADC \
	asm volatile( \
		"\n\
		.balign 4\n\
		andi a2, %[cyccnt], 3\n\
		c.slli a2, 1\n\
		c.addi a2, 12\n\
		auipc a1, 0\n\
		c.add  a2, a1\n\
		jalr a2, 1\n\
		.long 0x00010001\n\
		.long 0x00010001\n\
		"\
		:: [cyccnt]"r"(SysTick->CNT) : "a1", "a2"\
	);

register volatile unsigned continueflag asm ("x4");

#define RESETISRDONEFLAG asm volatile( "li x4, 0" );
#define WAITFORISRDONE asm volatile( "1: beqz x4, 1b" );
#define ISRCONT asm volatile( "addi x4, x4, 1" );
#else
#define ISRCONT
#endif


void DMA1_Channel2_IRQHandler(void) __attribute__((interrupt));
void DMA1_Channel2_IRQHandler(void) { TIM1->CTLR1 = 0; DMA1->INTFCR = DMA_CTCIF2; ISRCONT; }

// Max rate, ~4MSPS
// For reading from fixed values and writing them to hardware (variable rate)
void RunScheduledDMA( const uint32_t * values, volatile uint32_t * const * addresses, uint32_t * times, int count )
{
	// Will write output, but configured by other DMAs.
	DMA1_Channel6->CNTR = count;
	DMA1_Channel6->MADDR = (uint32_t)&values[0];
	DMA1_Channel6->PADDR = (uint32_t)&addresses[0];
	DMA1_Channel6->CFGR = DMA_DIR_PeripheralDST | DMA_CFGR1_PL | DMA_CFGR1_MINC | DMA_CFGR1_EN | DMA_CFGR1_MSIZE_1 | DMA_CFGR1_PSIZE_1;

	// ISR on this DMA
	DMA1_Channel2->CNTR = count;
	DMA1_Channel2->MADDR = (uint32_t)&addresses[0];
	DMA1_Channel2->PADDR = (uint32_t)&DMA1_Channel6->PADDR;
	DMA1_Channel2->CFGR = DMA_DIR_PeripheralDST | DMA_CFGR1_PL_0 | DMA_CFGR1_MINC | DMA_CFGR1_TCIE | DMA_CFGR1_EN | DMA_CFGR1_MSIZE_1 | DMA_CFGR1_PSIZE_1 | DMA_CFGR1_CIRC;

	DMA1_Channel3->CNTR = count;
	DMA1_Channel3->MADDR = (uint32_t)&times[0];
	DMA1_Channel3->PADDR = (uint32_t)&TIM1->ATRLR;
	DMA1_Channel3->CFGR = DMA_DIR_PeripheralDST | DMA_CFGR1_PL_1 | DMA_CFGR1_MINC | DMA_CFGR1_EN | DMA_CFGR1_MSIZE_1 | DMA_CFGR1_PSIZE_0 | DMA_CFGR1_CIRC;

	TIM1->PSC = 0; // so we don't have to worry about ADC/timer synchronization.

	// Reload immediately (This does not seem to be working?)  see the M2 we need to do in CLONESETM2?  But not having it is worse.
	TIM1->SWEVGR = TIM1_SWEVGR_UG;
	TIM1->DMAINTENR = TIM1_DMAINTENR_COMDE | TIM1_DMAINTENR_TDE |
		TIM1_DMAINTENR_CC3DE | TIM1_DMAINTENR_CC2DE | TIM1_DMAINTENR_CC1DE;
	TIM1->CH1CVR = 1;
	TIM1->CH2CVR = 3;
	TIM1->CH3CVR = 7;
#ifdef CH32V003
	RESETISRDONEFLAG
//	FORCEALIGNADC
#endif
	TIM1->ATRLR = 32;       // Auto Reload - sets period  (This is how fast each pixel works per set)
//	TIM1->CNT = 26 + ((((SysTick->CNT) & 1))); // Synchronize the timer to even cycles so it syncs with the ADC.
	TIM1->CNT = 26;
	TIM1->CTLR1 = TIM1_CTLR1_CEN;
#ifdef CH32V003
	WAITFORISRDONE
#else
	__WFI();
#endif
}

void RepeatScheduledDMA()
{
	DMA1_Channel6->CNTR = DMA1_Channel3->CNTR;  // re-set Channel 6's DMA data.
#ifdef CH32V003
	RESETISRDONEFLAG
	FORCEALIGNADC
#endif

	TIM1->ATRLR = 32;       // Auto Reload - sets period  (This is how fast each pixel works per set)
	TIM1->CNT = 26 + ((((SysTick->CNT) & 1))); // Synchronize the timer to even cycles so it syncs with the ADC.
	TIM1->CTLR1 = TIM1_CTLR1_CEN; // Enable

#ifdef CH32V003
	WAITFORISRDONE
#else
	__WFI(); // Optional, but this quiets down the bus because the CPU isn't constantly hitting it.
#endif
}


int main()
{
	SystemInit();
	
	// Enable GPIOD, C and ADC
	RCC->APB2PCENR |= RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOD |
		RCC_APB2Periph_GPIOC | RCC_APB2Periph_ADC1  | RCC_APB2Periph_TIM1;

	funPinMode( PD2, GPIO_CFGLR_OUT_10Mhz_PP );

	RCC->APB1PCENR |= RCC_APB1Periph_TIM2;
	RCC->AHBPCENR  |= RCC_AHBPeriph_DMA1 | RCC_AHBPeriph_SRAM;

	NVIC_EnableIRQ(DMA1_Channel2_IRQn);

	////////////////////////////////////////////////////////////////////

#ifdef CH32V003
	// ADCCLK = 24 MHz => RCC_ADCPRE = 0: divide sys clock by 2
	RCC->CFGR0 &= ~(0x1F<<11);
#else
	// ADCCLK = 48 MHz => RCC_ADCPRE divide by 2
	RCC->CFGR0 = (RCC->CFGR0 & ~(RCC_ADCPRE | RCC_CFGR0_ADC_CLK_MODE | RCC_HPRE));
	// You could use RCC_CFGR0_ADC_CLK_MODE and not worry about time sync with
	// the ADC, since its timing granularity is 1/48MHz. But, the 006 has
	// serious INL issues (most of which can be dithered away) when using
	// 48MHz mode on the ADC.  I recommend leaving the ADC in 24MHz mode.
#endif

	// Set up single conversion on chl 3 (PD2)
	ADC1->RSQR1 = 0;
	ADC1->RSQR2 = 0;
	ADC1->RSQR3 = (COIL_ON_ADC_NUMBER<<0);

#if EXTENDED_MODE
	#define SAMPTIME 0b101
#else
// Good for shorter terms
	#define SAMPTIME 0b100
#endif

	ADC1->SAMPTR1 = SAMPTIME;
	ADC1->SAMPTR2 = SAMPTIME | (SAMPTIME<<3) | (SAMPTIME<<6) | (SAMPTIME<<9) | (SAMPTIME<<12) | (SAMPTIME<<15) | (SAMPTIME<<18) | (SAMPTIME<<21) | (SAMPTIME<<24) | (SAMPTIME<<27);
	// Turn on ADC module  Not sure why it needs to be done twice.
	ADC1->CTLR2 = ADC_ADON | ADC_EXTSEL_SWSTART;

	// enable scanning
	ADC1->CTLR1 = ADC_SCAN;// | ADC_BUFEN;

	// Reset calibration
	ADC1->CTLR2 |= ADC_RSTCAL;
	while(ADC1->CTLR2 & ADC_RSTCAL);
	
	// Calibrate
	ADC1->CTLR2 |= ADC_CAL;
	while(ADC1->CTLR2 & ADC_CAL);

	// Create a pattern on PC0, PC1.
	uint32_t values[] = {
		ADC_ADON | ADC_EXTSEL,
		ADC_ADON | ADC_EXTSEL | ADC_SWSTART,
		1<<(COIL_ON_PIN), // Write 1 on port.
		1<<((COIL_ON_PIN)+16), // Write 0 on port.
	};

	volatile uint32_t * addresses[] = { 
		&ADC1->CTLR2,
		&ADC1->CTLR2,
		&COIL_ON_PORT->BSHR,
		&COIL_ON_PORT->BSHR,
	};

#ifdef CH32V003
	uint32_t times[] = {
		34, // Manually tuned.
		27, // Manually tuned.
		27, // Then set port 1 to 1, and wait a few cycles
		20,
		 }; 
#else
	uint32_t times[] = {
		31, // Manually tuned.
		27, // Manually tuned.
		27, // Then set port 1 to 1, and wait a few cycles
		10,
		 }; 
#endif

	int count = sizeof(times)/sizeof(times[0]);

	RunScheduledDMA( values, addresses, times, count );

	while(1)
	{
		int i;
#if EXTENDED_MODE
		for( int n = 85; n > 27; n-- )
		{
			times[1] = n;
			times[2] = 58;
#else
		for( int n = 27; n < 38; n++ )
		{
			times[1] = n;
#endif
			uint32_t sum = 0;
			for( i = 0; i < 16; i++ )
			{
				RepeatScheduledDMA();
				sum += ADC1->RDATAR;
			}

			// Faster to print into fixed buffer.
			char cts[10];
			int n = sprintf( cts, "%d ", (int)sum );
			_write( 0, cts, n );
		}

		printf( "\n" );
	}

}

