#ifndef CH5xx_LCD
#define CH5xx_LCD

#include <ch32fun.h>
#include <ch5xxhw.h>

#define LCD_VOLTAGE_3V3     0
#define LCD_VOLTAGE_2V5     1

#define LCD_BIAS_1_2        0
#define LCD_BIAS_1_3        1

#define LCD_DUTY_1_2        0 // 2 common pins
#define LCD_DUTY_1_3        1 // 3 common pins
#define LCD_DUTY_1_4        2 // 4 common pins

#define LCD_CLOCK_256       0 // 256Hz
#define LCD_CLOCK_512       1 // 512Hz
#define LCD_CLOCK_1000      2 // 1KHz
#define LCD_CLOCK_128       3 // 128Hz

#if defined(CH584_CH585)
#define LCD_CMD R8_LCD_CMD
#else
#define LCD_CMD R32_LCD_CMD
#endif

#ifndef ENABLE_COM0
#define ENABLE_COM0 0
#endif
#ifndef ENABLE_COM1
#define ENABLE_COM1 0
#endif
#ifndef ENABLE_COM2
#define ENABLE_COM2 0
#endif
#ifndef ENABLE_COM3
#define ENABLE_COM3 0
#endif

#ifndef ENABLE_SEG0
#define ENABLE_SEG0 0
#endif
#ifndef ENABLE_SEG1
#define ENABLE_SEG1 0
#endif
#ifndef ENABLE_SEG2
#define ENABLE_SEG2 0
#endif
#ifndef ENABLE_SEG3
#define ENABLE_SEG3 0
#endif
#ifndef ENABLE_SEG4
#define ENABLE_SEG4 0
#endif
#ifndef ENABLE_SEG5
#define ENABLE_SEG5 0
#endif
#ifndef ENABLE_SEG6
#define ENABLE_SEG6 0
#endif
#ifndef ENABLE_SEG7
#define ENABLE_SEG7 0
#endif
#ifndef ENABLE_SEG8
#define ENABLE_SEG8 0
#endif
#ifndef ENABLE_SEG9
#define ENABLE_SEG9 0
#endif
#ifndef ENABLE_SEG10
#define ENABLE_SEG10 0
#endif
#ifndef ENABLE_SEG11
#define ENABLE_SEG11 0
#endif
#ifndef ENABLE_SEG12
#define ENABLE_SEG12 0
#endif
#ifndef ENABLE_SEG13
#define ENABLE_SEG13 0
#endif
#ifndef ENABLE_SEG14
#define ENABLE_SEG14 0
#endif
#ifndef ENABLE_SEG15
#define ENABLE_SEG15 0
#endif
#ifndef ENABLE_SEG16
#define ENABLE_SEG16 0
#endif
#ifndef ENABLE_SEG17
#define ENABLE_SEG17 0
#endif
#ifndef ENABLE_SEG18
#define ENABLE_SEG18 0
#endif
#ifndef ENABLE_SEG19
#define ENABLE_SEG19 0
#endif
#if defined(CH584_CH585)
#ifndef ENABLE_SEG20
#define ENABLE_SEG20 0
#endif
#ifndef ENABLE_SEG21
#define ENABLE_SEG21 0
#endif
#ifndef ENABLE_SEG22
#define ENABLE_SEG22 0
#endif
#ifndef ENABLE_SEG23
#define ENABLE_SEG23 0
#endif
#ifndef ENABLE_SEG24
#define ENABLE_SEG24 0
#endif
#ifndef ENABLE_SEG25
#define ENABLE_SEG25 0
#endif
#ifndef ENABLE_SEG26
#define ENABLE_SEG26 0
#endif
#ifndef ENABLE_SEG27
#define ENABLE_SEG27 0
#endif
#endif

#ifndef LCD_FREQ
#define LCD_FREQ LCD_CLOCK_128
#endif

static void LCDInit(uint8_t duty, uint8_t bias) {
#if defined(CH584_CH585)
#if ENABLE_SEG14 || ENABLE_SEG15
	SYS_SAFE_ACCESS(
		R8_SAFE_DEBUG_CTRL |= RB_DEBUG_DIS;
	);
#endif
	// Disable digital input for LCD pins
	R32_PIN_IN_DIS |= (ENABLE_SEG0<<16)  | // PB0- PB15
	                  (ENABLE_SEG1<<17)  |
	                  (ENABLE_SEG2<<18)  |
	                  (ENABLE_SEG3<<19)  |
	                  (ENABLE_COM0<<20)  |
	                  (ENABLE_COM1<<21)  |
	                  (ENABLE_COM2<<22)  |
	                  (ENABLE_COM3<<23)  |
	                  (ENABLE_SEG8<<24)  |
	                  (ENABLE_SEG9<<25)  |
	                  (ENABLE_SEG10<<26) |
	                  (ENABLE_SEG11<<27) |
	                  (ENABLE_SEG12<<28) |
	                  (ENABLE_SEG13<<29) |
	                  (ENABLE_SEG14<<30) |
	                  (ENABLE_SEG15<<31) |
	                  (ENABLE_SEG4<<0)   | //PA0-PA9, PA13
	                  (ENABLE_SEG5<<1)   |
	                  (ENABLE_SEG6<<2)   |
	                  (ENABLE_SEG7<<3)   |
	                  (ENABLE_SEG24<<7)  |
	                  (ENABLE_SEG25<<8)  |
	                  (ENABLE_SEG26<<9)  |
	                  (ENABLE_SEG27<<13);

	R16_PIN_CONFIG |= (ENABLE_SEG16<<8)  |
	                  (ENABLE_SEG17<<9)  |
	                  (ENABLE_SEG18<<10) |
	                  (ENABLE_SEG19<<11) |
	                  (ENABLE_SEG20<<12) |
	                  (ENABLE_SEG21<<13) |
	                  (ENABLE_SEG22<<14) |
	                  (ENABLE_SEG23<<15);

	R32_LCD_SEG_EN = (ENABLE_SEG0<<0)   |
	                 (ENABLE_SEG1<<1)   |
	                 (ENABLE_SEG2<<2)   |
	                 (ENABLE_SEG3<<3)   |
	                 (ENABLE_SEG4<<4)   |
	                 (ENABLE_SEG5<<5)   |
	                 (ENABLE_SEG6<<6)   |
	                 (ENABLE_SEG7<<7)   |
	                 (ENABLE_SEG8<<8)   |
	                 (ENABLE_SEG9<<9)   |
	                 (ENABLE_SEG10<<10) |
	                 (ENABLE_SEG11<<11) |
	                 (ENABLE_SEG12<<12) |
	                 (ENABLE_SEG13<<13) |
	                 (ENABLE_SEG14<<14) |
	                 (ENABLE_SEG15<<15) |
	                 (ENABLE_SEG16<<16) |
	                 (ENABLE_SEG17<<17) |
	                 (ENABLE_SEG18<<18) |
	                 (ENABLE_SEG19<<19) |
	                 (ENABLE_SEG20<<20) |
	                 (ENABLE_SEG21<<21) |
	                 (ENABLE_SEG22<<22) |
	                 (ENABLE_SEG23<<23) |
	                 (ENABLE_SEG24<<24) |
	                 (ENABLE_SEG25<<25) |
	                 (ENABLE_SEG26<<26) |
	                 (ENABLE_SEG27<<27);
#elif defined(CH591_CH592)
	R32_PIN_CONFIG2 = (ENABLE_COM0<<28)  |
	                  (ENABLE_COM1<<29)  |
	                  (ENABLE_COM2<<30)  |
	                  (ENABLE_COM3<<31)  |
	                  (ENABLE_SEG0<<23)  |
	                  (ENABLE_SEG1<<20)  |
	                  (ENABLE_SEG2<<25)  |
	                  (ENABLE_SEG3<<24)  |
	                  (ENABLE_SEG4<<4)   |
	                  (ENABLE_SEG5<<5)   |
	                  (ENABLE_SEG6<<15)  |
	                  (ENABLE_SEG7<<14)  |
	                  (ENABLE_SEG8<<13)  |
	                  (ENABLE_SEG9<<12)  |
	                  (ENABLE_SEG10<<11) |
	                  (ENABLE_SEG11<<10) |
	                  (ENABLE_SEG12<<8)  |
	                  (ENABLE_SEG13<<9)  |
	                  (ENABLE_SEG14<<27) |
	                  (ENABLE_SEG15<<26) |
	                  (ENABLE_SEG16<<22) |
	                  (ENABLE_SEG17<<16) |
	                  (ENABLE_SEG18<<6)  |
	                  (ENABLE_SEG19<<7);
#if ENABLE_COM2 || ENABLE_COM3
	R16_PIN_ALTERNATE |= (1<<13); // If you need more that 2 common pins, you need to disable debug module 
#endif
	LCD_CMD = (ENABLE_SEG0<<8)   |
	          (ENABLE_SEG1<<9)   |
	          (ENABLE_SEG2<<10)  |
	          (ENABLE_SEG3<<11)  |
	          (ENABLE_SEG4<<12)  |
	          (ENABLE_SEG5<<13)  |
	          (ENABLE_SEG6<<14)  |
	          (ENABLE_SEG7<<15)  |
	          (ENABLE_SEG8<<16)  |
	          (ENABLE_SEG9<<17)  |
	          (ENABLE_SEG10<<18) |
	          (ENABLE_SEG11<<19) |
	          (ENABLE_SEG12<<20) |
	          (ENABLE_SEG13<<21) |
	          (ENABLE_SEG14<<22) |
	          (ENABLE_SEG15<<23) |
	          (ENABLE_SEG16<<24) |
	          (ENABLE_SEG17<<25) |
	          (ENABLE_SEG18<<26) |
	          (ENABLE_SEG19<<27);
#endif
	LCD_CMD |= RB_LCD_SYS_EN | RB_LCD_ON | (LCD_FREQ << 5) | (duty << 3) | (bias << 2);
}

static void LCDSegSet(uint32_t com, uint32_t seg) {
	// uint32_t bank = (seg/8);
	uint32_t bank = (seg>>3);
	// *(&R32_LCD_RAM0+(bank)) |= (1<<com)<<((seg-(bank*8)-1)*4);
	*((uint32_t*)(&R32_LCD_RAM0)+bank) |= (1<<com)<<((seg-(bank<<3))<<2);
}

static void LCDSegClear(uint32_t com, uint32_t seg) {
	uint32_t bank = (seg>>3);
	*((uint32_t*)(&R32_LCD_RAM0)+bank) &= ~((1<<com)<<((seg-(bank<<3))<<2));
}

static void LCDClear() {
	R32_LCD_RAM0 = 0;
	R32_LCD_RAM1 = 0;
	R32_LCD_RAM2 = 0;
#if defined(CH584_CH585)
	R32_LCD_RAM3 = 0;
#endif
}

#define LCDSetVoltage(v) LCD_CMD = (LCD_CMD&0x7f) | (v<<7)
#define LCDPower(p) LCD_CMD = (LCD_CMD&0xfe) | p
#define LCDSetFreq(f) LCD_CMD = (LCD_CMD&0x9f) | (f<<5)

#endif

// CH592 LCD mapping:
// COMMON0 - PB12
// COMMON1 - PB13
// COMMON2 - PB14
// COMMON3 - PB15
// SEGMENT0 - PB7
// SEGMENT1 - PB4
// SEGMENT2 - PB23
// SEGMENT3 - PB22
// SEGMENT4 - PA4
// SEGMENT5 - PA5
// SEGMENT6 - PA15
// SEGMENT7 - PA14
// SEGMENT8 - PA13
// SEGMENT9 - PA12
// SEGMENT10 - PA11
// SEGMENT11 - PA10
// SEGMENT12 - PA8
// SEGMENT13 - PA9
// SEGMENT14 - PB11
// SEGMENT15 - PB10
// SEGMENT16 - PB6
// SEGMENT17 - PB0
// SEGMENT18 - PA6
// SEGMENT19 - PA7

// CH585 LCD mapping:
// COMMON0 - PB4
// COMMON1 - PB5
// COMMON2 - PB6
// COMMON3 - PB7
// SEGMENT0 - PB0
// SEGMENT1 - PB1
// SEGMENT2 - PB2
// SEGMENT3 - PB3
// SEGMENT4 - PA0
// SEGMENT5 - PA1
// SEGMENT6 - PA2
// SEGMENT7 - PA3
// SEGMENT8 - PB8
// SEGMENT9 - PB9
// SEGMENT10 - PB10
// SEGMENT11 - PB11
// SEGMENT12 - PB12
// SEGMENT13 - PB13
// SEGMENT14 - PB14
// SEGMENT15 - PB15
// SEGMENT16 - PB16
// SEGMENT17 - PB17
// SEGMENT18 - PB18
// SEGMENT19 - PB19
// SEGMENT20 - PB20
// SEGMENT21 - PB21
// SEGMENT22 - PB22
// SEGMENT23 - PB23
// SEGMENT24 - PA7
// SEGMENT25 - PA8
// SEGMENT26 - PA9
// SEGMENT27 - PA13
