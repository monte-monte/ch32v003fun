/*
	ADC-aided capacitive touch sensing.

	This demonstrates use of the ADC with cap sense pads. By using the ADC, you
	can get much higher resolution than if you use the exti.  But, you are
	limited by the number of ADC lines, i.e. you can only support a maximum of
	eight (8) touch inputs with this method.

	In exchange for that, typically you can get > 1,000 LSBs per 1 CM^2 and it
	can convert faster.  Carefully written code can be as little as 2us per
	conversion!

	The mechanism of operation for the touch sensing on the CH32V003 is to:
		* Hold an IO low.
		* Start the ADC
		* Use the internal pull-up to pull the line high.
		* The ADC will sample the voltage on the slope.
		* Lower voltage = longer RC respone, so higher capacitance. 
*/

#include "ch32fun.h"
#include <stdio.h>

#include "ch32v00x_touch.h"

#define CAPSENSES 8

void Read8( uint32_t * sum )
{
	memset( sum, 0, sizeof(uint32_t)*CAPSENSES) ;
	// Sampling all touch pads, 3x should take 6030 cycles, and runs at about 8kHz
	// 3x is 7928 cycles on the 006.
	// But we can go many more.
	int iterations = 100;
	#if CAPSENSES > 0
	sum[0] += ReadTouchPin( GPIOA, 2, 0, iterations );
	#endif
	#if CAPSENSES > 1
	sum[1] += ReadTouchPin( GPIOA, 1, 1, iterations );
	#endif
	#if CAPSENSES > 2
	sum[2] += ReadTouchPin( GPIOC, 4, 2, iterations );
	#endif
	#if CAPSENSES > 3
	sum[3] += ReadTouchPin( GPIOD, 2, 3, iterations );
	#endif
	#if CAPSENSES > 4
	sum[4] += ReadTouchPin( GPIOD, 3, 4, iterations );
	#endif
	#if CAPSENSES > 5
	sum[5] += ReadTouchPin( GPIOD, 5, 5, iterations );
	#endif
	#if CAPSENSES > 6
	sum[6] += ReadTouchPin( GPIOD, 6, 6, iterations );
	#endif
	#if CAPSENSES > 7
	sum[7] += ReadTouchPin( GPIOD, 4, 7, iterations );
	#endif
}

int main()
{
	SystemInit();

	printf("Capacitive Touch ADC example\n");
	
	// Enable GPIOD, C and ADC
	RCC->APB2PCENR |= RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOC | RCC_APB2Periph_ADC1;

	InitTouchADC();
	printf( "Initialized\n" );

	// should be ready for SW conversion now
	uint32_t base[CAPSENSES] = { 0 };

	uint32_t start = SysTick->CNT;
	Read8( base );
	uint32_t end = SysTick->CNT;
	printf( "Time: %d\n", (int)(end-start) );

	Read8( base );

	while(1)
	{
		uint32_t sum[CAPSENSES] = { 0 };

		Read8( sum );

		int n;
		for( n = 0; n < CAPSENSES; n++ )
		{
			int diff = sum[n] - base[n];
			if( diff < 0 ) diff = 0;
			sum[n] = diff;
			printf( "%d ", (int)sum[n] );
		}
		printf( "\n" );
	}
}

