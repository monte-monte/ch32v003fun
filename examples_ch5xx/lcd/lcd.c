#include "ch32fun.h"
#include <stdio.h>

#define ENABLE_COM0 1
#define ENABLE_COM1 1
#define ENABLE_COM2 1
#define ENABLE_COM3 1

#define ENABLE_SEG0 1
#define ENABLE_SEG1 1
#define ENABLE_SEG2 1
#define ENABLE_SEG3 1
#define ENABLE_SEG4 1
#define ENABLE_SEG5 1
#define ENABLE_SEG6 1
#define ENABLE_SEG7 1
#define ENABLE_SEG8 1
#define ENABLE_SEG9 1
#define ENABLE_SEG10 1
#define ENABLE_SEG11 1
#define ENABLE_SEG12 1
#define ENABLE_SEG13 1
#define ENABLE_SEG14 1
#define ENABLE_SEG15 1
#define ENABLE_SEG16 1
#define ENABLE_SEG17 1
#define ENABLE_SEG18 1
#define ENABLE_SEG19 1

#if defined(CH584_CH585)
#define ENABLE_SEG20 1
#define ENABLE_SEG21 1
#define ENABLE_SEG22 1
#define ENABLE_SEG23 1
#define ENABLE_SEG24 1
#define ENABLE_SEG25 1
#define ENABLE_SEG26 1
#define ENABLE_SEG27 1
#define NUMBER_OF_SEGMENTS 28
#define NUMBER_OF_COMMONS 4
#else
#define NUMBER_OF_SEGMENTS 20
#define NUMBER_OF_COMMONS 4
#endif

#include "./ch5xx_lcd.h"

#define ANIM_DELAY 150

int main()
{
	SystemInit();
	Delay_Ms(500); // delay before potentially disabling Debug

	LCDInit(LCD_DUTY_1_4, LCD_BIAS_1_3);

	// Gradually fill the screen by turning all segments on
	uint32_t segment = 0;
	uint32_t common = 0;

	while (1) {
		if (common > NUMBER_OF_COMMONS-1) {
			common = 0;
			LCDClear();
		}
		LCDSegSet(common, segment++);
		if (segment > NUMBER_OF_SEGMENTS-1) {
			segment = 0;
			common++;
		}
		Delay_Ms(ANIM_DELAY);
	}
}
