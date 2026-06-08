#include "ch32fun.h"
#include <stdio.h>

void EXTI7_0_IRQHandler( void ) INTERRUPT_DECORATOR;
void EXTI7_0_IRQHandler( void ) 
{
	// Flash just a little blip.
	funDigitalWrite( PC1, FUN_HIGH );
	funDigitalWrite( PC1, FUN_LOW );

	// Acknowledge the interrupt
	EXTI->INTFR = EXTI_Line3;
}

int main()
{
	SystemInit();

	// Enable GPIOs
	RCC->APB2PCENR = RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO;

	// GPIO D3 for input pin change.
	funPinMode( PD3, GPIO_CFGLR_IN_FLOAT );
	// funPinMode( PD4, GPIO_CFGLR_IN_PUPD ); // Keep SWIO enabled / seems to be unnecessary

	// GPIO C0 Push-Pull (our output)
	funPinMode( PC0,  GPIO_CFGLR_OUT_10Mhz_PP );
	funPinMode( PC1,  GPIO_CFGLR_OUT_10Mhz_PP );

	// Configure the IO as an interrupt.
	AFIO->EXTICR = AFIO_EXTICR_EXTI3_PD; // Defines PORT and PIN number.
	EXTI->INTENR = EXTI_INTENR_MR3; // Enable EXT3

	// You can set `R`ising edge here or `F`alling edge here.
	EXTI->RTENR = EXTI_RTENR_TR3;  // Rising edge trigger

	// enable interrupt
	NVIC_EnableIRQ( EXTI7_0_IRQn );

	while(1)
	{
		asm volatile( "nop" );
	}
}
