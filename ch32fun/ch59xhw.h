#ifndef TODO_HARDWARE_H
#define TODO_HARDWARE_H

#include "ch32fun.h"

#ifndef __ASSEMBLER__  // Things before this can be used in assembly.

#ifdef __cplusplus
extern "C" {
#endif

/* Interrupt Number Definition, according to the selected device */
typedef enum IRQn
{
	/******  RISC-V Processor Exceptions Numbers *******************************************************/
	NonMaskableInt_IRQn = 2, /* 2 Non Maskable Interrupt                             */
	EXC_IRQn = 3,            /* 3 Exception Interrupt                                */
	Ecall_M_Mode_IRQn = 5,   /* 5 Ecall M Mode Interrupt                             */
	Ecall_U_Mode_IRQn = 8,   /* 8 Ecall U Mode Interrupt                             */
	Break_Point_IRQn = 9,    /* 9 Break Point Interrupt                              */
	SysTicK_IRQn = 12,       /* 12 System timer Interrupt                            */
	Software_IRQn = 14,      /* 14 software Interrupt                                */

	/******  RISC-V specific Interrupt Numbers *********************************************************/
	TMR0_IRQn = 16,          /* 0:  TMR0 */
	GPIOA_IRQn = 17,         /* GPIOA */
	GPIOB_IRQn = 18,         /* GPIOB */
	SPI0_IRQn = 19,          /* SPI0 */
	BB_IRQn = 20,            /* BLEB */
	LLE_IRQn = 21,           /* BLEL */
	USB_IRQn = 22,           /* USB */
	TMR1_IRQn = 24,          /* TMR1 */
	TMR2_IRQn = 25,          /* TMR2 */
	UART0_IRQn = 26,         /* UART0 */
	UART1_IRQn = 27,         /* UART1 */
	RTC_IRQn = 28,           /* RTC */
	ADC_IRQn = 29,           /* ADC */
	I2C_IRQn = 30,           /* I2C */
	PWMX_IRQn = 31,          /* PWMX */
	TMR3_IRQn = 32,          /* TMR3 */
	UART2_IRQn = 33,         /* UART2 */
	UART3_IRQn = 34,         /* UART3 */
	WDOG_BAT_IRQn = 35,      /* WDOG_BAT */
} IRQn_Type;

#define BASE_VECTOR "\n\
	.balign  2\n\
	.option   push;\n\
	.option   norvc;\n\
	j handle_reset\n\
	.word   0\n\
	.word   NMI_Handler                 /* NMI Handler */\n\
	.word   HardFault_Handler           /* Hard Fault Handler */\n\
	.word   0xF5F9BDA9\n\
	.word   Ecall_M_Mode_Handler        /* 5 */\n\
	.word   0\n\
	.word   0\n\
	.word   Ecall_U_Mode_Handler		/* 8 */\n\
	.word   Break_Point_Handler			/* 9 */\n\
	.word   0\n\
	.word   0\n\
	.word   SysTick_Handler            /* SysTick Handler */\n\
	.word   0\n\
	.word   SW_Handler                 /* SW Handler */\n\
	.word   0\n\
	/* External Interrupts */\n\
	.word   TMR0_IRQHandler            /* 0:  TMR0 */\n\
	.word   GPIOA_IRQHandler           /* GPIOA */\n\
	.word   GPIOB_IRQHandler           /* GPIOB */\n\
	.word   SPI0_IRQHandler            /* SPI0 */\n\
	.word   BB_IRQHandler              /* BLEB */\n\
	.word   LLE_IRQHandler             /* BLEL */\n\
	.word   USB_IRQHandler             /* USB */\n\
	.word   0\n\
	.word   TMR1_IRQHandler            /* TMR1 */\n\
	.word   TMR2_IRQHandler            /* TMR2 */\n\
	.word   UART0_IRQHandler           /* UART0 */\n\
	.word   UART1_IRQHandler           /* UART1 */\n\
	.word   RTC_IRQHandler             /* RTC */\n\
	.word   ADC_IRQHandler             /* ADC */\n\
	.word   I2C_IRQHandler             /* I2C */\n\
	.word   PWMX_IRQHandler            /* PWMX */\n\
	.word   TMR3_IRQHandler            /* TMR3 */\n\
	.word   UART2_IRQHandler           /* UART2 */\n\
	.word   UART3_IRQHandler           /* UART3 */\n\
	.word   WDOG_BAT_IRQHandler        /* WDOG_BAT */\n"

#define DEFAULT_INTERRUPT_VECTOR_CONTENTS BASE_VECTOR "\n.option pop;\n"

#define __HIGH_CODE __attribute__((section(".highcode"), used))
#define __INTERRUPT __attribute__((interrupt))

/* memory mapped structure for SysTick */
typedef struct
{
	__IO uint32_t CTLR;
	__IO uint32_t SR;
	__IO uint64_t CNT;
	__IO uint64_t CMP;
} SysTick_Type;


/* memory mapped structure for Program Fast Interrupt Controller (PFIC) */
typedef struct
{
	__I uint32_t  ISR[8];           // 0
	__I uint32_t  IPR[8];           // 20H
	__IO uint32_t ITHRESDR;         // 40H
	uint8_t       RESERVED[4];      // 44H
	__O uint32_t  CFGR;             // 48H
	__I uint32_t  GISR;             // 4CH
	__IO uint8_t  VTFIDR[4];        // 50H
	uint8_t       RESERVED0[0x0C];  // 54H
	__IO uint32_t VTFADDR[4];       // 60H
	uint8_t       RESERVED1[0x90];  // 70H
	__O uint32_t  IENR[8];          // 100H
	uint8_t       RESERVED2[0x60];  // 120H
	__O uint32_t  IRER[8];          // 180H
	uint8_t       RESERVED3[0x60];  // 1A0H
	__O uint32_t  IPSR[8];          // 200H
	uint8_t       RESERVED4[0x60];  // 220H
	__O uint32_t  IPRR[8];          // 280H
	uint8_t       RESERVED5[0x60];  // 2A0H
	__IO uint32_t IACTR[8];         // 300H
	uint8_t       RESERVED6[0xE0];  // 320H
	__IO uint8_t  IPRIOR[256];      // 400H
	uint8_t       RESERVED7[0x810]; // 500H
	__IO uint32_t SCTLR;            // D10H
} PFIC_Type;
#endif /* __ASSEMBLER__*/

#ifdef __ASSEMBLER__
#define CORE_PERIPH_BASE           (0xE0000000) /* System peripherals base address in the alias region */
#else
#define CORE_PERIPH_BASE           ((uint32_t)(0xE0000000))
#endif /* __ASSEMBLER__*/

#define PFIC_BASE                  (CORE_PERIPH_BASE + 0xE000)
#define SysTick_BASE               (CORE_PERIPH_BASE + 0xF000)

#define PFIC                       ((PFIC_Type *) PFIC_BASE)
#define NVIC                       PFIC
#define NVIC_KEY1                  ((uint32_t)0xFA050000)
#define	NVIC_KEY2	               ((uint32_t)0xBCAF0000)
#define	NVIC_KEY3		           ((uint32_t)0xBEEF0000)

#define SysTick                    ((SysTick_Type *) SysTick_BASE)
#define SYSTICK_LOAD_RELOAD_MSK    (0xFFFFFFFFFFFFFFFF)
#define SYSTICK_CTLR_SWIE          (1 << 31)
#define SYSTICK_CTLR_INIT          (1 << 5)
#define SYSTICK_CTLR_MODE          (1 << 4)
#define SYSTICK_CTLR_STRE          (1 << 3)
#define SYSTICK_CTLR_STCLK         (1 << 2)
#define SYSTICK_CTLR_STIE          (1 << 1)
#define SYSTICK_CTLR_STE           (1 << 0)
#define SYSTICK_SR_CNTIF           (1 << 0)

typedef enum
{
	CLK_SOURCE_LSI = 0x00,
	CLK_SOURCE_LSE,

	CLK_SOURCE_HSE_16MHz =  (0x20 |  2),
	CLK_SOURCE_HSE_8MHz =   (0x20 |  4),
	CLK_SOURCE_HSE_6_4MHz = (0x20 |  5),
	CLK_SOURCE_HSE_4MHz =   (0x20 |  8),

	CLK_SOURCE_PLL_80MHz =  (0x40 |  6),
	CLK_SOURCE_PLL_60MHz =  (0x40 |  8),
	CLK_SOURCE_PLL_48MHz =  (0x40 | 10),
	CLK_SOURCE_PLL_32MHz =  (0x40 | 15),
	CLK_SOURCE_PLL_24MHz =  (0x40 | 20),
} SYS_CLKTypeDef;

typedef enum
{
	LSE_RCur_70 = 0,
	LSE_RCur_100,
	LSE_RCur_140,
	LSE_RCur_200
} LSECurrentTypeDef;

typedef enum
{
	HSE_RCur_75 = 0,
	HSE_RCur_100,
	HSE_RCur_125,
	HSE_RCur_150
} HSECurrentTypeDef;

// for ISP programming
#define ISPROM_ADDRESS                  0x00078000
#define ISPROM_IN_RAM_ADDRESS           0x20003800
#define ISPROM_START_OFFSET             0x94
#define ISPROM_SIZE                     0x2500
#define ISPROM_BOOTBUTTON_CHECK_ADDRESS 0x20003998
#define ISPROM_BSS_ADDRESS              0x20005cb8
#define ISPROM_BSS_SIZE                 0x0564
#define ISPROM_IN_RAM_GLOBALPOINTER     "0x200064b0" // string because it goes into asm()
#define ISPROM_IN_RAM_ENTRYPOINT        "0x20004e8a" // string because it goes into asm()

// For debug writing to the debug interface.
#define DMDATA0 			((vu32*)0xe0000380)
#define DMDATA1 			((vu32*)0xe0000384)
#define DMSTATUS_SENTINEL	((vu32*)0xe0000388)// Reads as 0x00000000 if debugger is attached.

/* Independent watch-dog register */
#define R32_IWDG_KR         (*((vu32*)0x40001000)) // WO, watch-dog key register
#define R32_IWDG_CFG        (*((vu32*)0x40001004)) // RW, watch-dog configuration
#define  RB_RLR             0x0FFF                    // RW, watch-dog counter reload (write protect)
#define  RB_PR              0x7000                    // PR, prescale (write protect)
#define  RB_PVU             0x8000                    // RO, register update flag (write protect)
#define  RB_COUNT           0xFF0000                  // RO, watch-dog down counter
#define  RB_STOP_EN         0x20000000                // RW, watch-dog stop enable (write protect)
#define  RB_WR_PROTECT      0x40000000                // RO, write protect
#define  RB_IWDG_EN         0x80000000                // RO, watch-dog enable

/* System: clock configuration register */
#define R32_CLK_SYS_CFG     (*((vu32*)0x40001008)) // RWA, system clock configuration, SAM
#define  RB_CLK_PLL_DIV     0x1F                      // RWA, output clock divider from PLL or CK32M
#define  RB_CLK_SYS_MOD     0xC0                      // RWA, system clock source mode: 00=divided from 32MHz, 01=divided from PLL-480MHz, 10=directly from 32MHz, 11=directly from 32KHz
#define  RB_TX_32M_PWR_EN   0x40000                   // RWA, extern 32MHz HSE power contorl
#define  RB_XT_FORCE_EN     0x80000                   // RWA, system clock control in Halt mode
#define  RB_PLL_PWR_EN      0x100000                  // RWA, PLL power control

/* System: sleep control register */
#define R32_SLEEP_CONTROL   (*((vu32*)0x4000100C)) // RWA, sleep control, SAM
#define R8_SLP_CLK_OFF0     (*((vu8*)0x4000100C))  // RWA, sleep clock off control byte 0, SAM
#define  RB_SLP_CLK_TMR0    0x01                      // RWA, close TMR0 clock
#define  RB_SLP_CLK_TMR1    0x02                      // RWA, close TMR1 clock
#define  RB_SLP_CLK_TMR2    0x04                      // RWA, close TMR2 clock
#define  RB_SLP_CLK_TMR3    0x08                      // RWA, close TMR3 clock
#define  RB_SLP_CLK_UART0   0x10                      // RWA, close UART0 clock
#define  RB_SLP_CLK_UART1   0x20                      // RWA, close UART1 clock
#define  RB_SLP_CLK_UART2   0x40                      // RWA, close UART2 clock
#define  RB_SLP_CLK_UART3   0x80                      // RWA, close UART3 clock
#define R8_SLP_CLK_OFF1     (*((vu8*)0x4000100D))  // RWA, sleep clock off control byte 1, SAM
#define  RB_SLP_CLK_SPI0    0x01                      // RWA, close SPI0 clock
#define  RB_SLP_CLK_PWMX    0x04                      // RWA, close PWMx clock
#define  RB_SLP_CLK_I2C     0x08                      // RWA, close I2C clock
#define  RB_SLP_CLK_USB     0x10                      // RWA, close USB clock
#define  RB_SLP_CLK_LCD     0x40                      // RWA, close LCD clock
#define  RB_SLP_CLK_BLE     0x80                      // RWA, close BLE clock
#define R8_SLP_WAKE_CTRL    (*((vu8*)0x4000100E))  // RWA, wake control, SAM
#define  RB_SLP_USB_WAKE    0x01                      // RWA, enable USB waking
//#define  RB_SLP_BLE_WAKE    0x04                      // RWA, enable BLE waking
#define  RB_SLP_RTC_WAKE    0x08                      // RWA, enable RTC waking
#define  RB_SLP_GPIO_WAKE   0x10                      // RWA, enable GPIO waking
#define  RB_SLP_BAT_WAKE    0x20                      // RWA, enable BAT waking
#define  RB_WAKE_EV_MODE    0x40                      // RWA, event wakeup mode: 0=event keep valid for long time, 1=short pulse event
#define  RB_GPIO_WAKE_MODE  0x80                      // RWA,
#define R8_SLP_POWER_CTRL   (*((vu8*)0x4000100F))  // RWA, peripherals power down control, SAM
#define  RB_WAKE_DLY_MOD    0x03                      // RWA, wakeup delay time selection
// RB_WAKE_DLY_MOD select wakeup delay
//   00: long time, 3590 cycles+TSUHSE
//   01: short time, 520 cycles+TSUHSE
//   10: shorter time, 70 cycles+TSUHSE
//   11: no delay, 8 cycles+TSUHSE
//#define  RB_SLP_USB_PWR_DN  0x01                      // RWA, enable USB power down
//#define  RB_SLP_BLE_PWR_DN  0x04                      // RWA, enable BLE power down
#define  RB_SLP_CLK_RAMX    0x10                      // RWA, close main SRAM clock
#define  RB_SLP_CLK_RAM2K   0x20                      // RWA, close retention 2KB SRAM clock
#define  RB_RAM_RET_LV      0x40                      // RWA, SRAM retention voltage selection: 0=normal, 1=low voltage for low power
#define R32_SLEEP_CTRL2     (*((vu32*)0x40001010)) // RWA, touchkey wake up enable
#define  RB_TKEY0_5_WAKE_EN    0x003F                 // RWA, touchkey wake up enable channel 0-5
#define  RB_TKEY8_13_WAKE_EN   0x3F00                 // RWA, touchkey wake up enable channel 8-13

/* System: I/O pin configuration register */
#define R32_PIN_CONFIG      (*((vu32*)0x40001018)) // RW, I/O pin configuration
#define R16_PIN_ALTERNATE   (*((vu16*)0x40001018)) // RW, function pin alternate configuration
#define  RB_PIN_TMR0        0x01                      // RW, TMR0 alternate pin enable: 0=TMR0/PWM0/CAP0 on PA[9], 1=TMR0_/PWM0_/CAP0_ on PB[23]
#define  RB_PIN_TMR1        0x02                      // RW, TMR1 alternate pin enable: 0=TMR1/PWM1/CAP1 on PA[10], 1=TMR1_/PWM1_/CAP1_ on PB[10]
#define  RB_PIN_TMR2        0x04                      // RW, TMR2 alternate pin enable: 0=TMR2/PWM2/CAP2 on PA[11], 1=TMR2_/PWM2_/CAP2_ on PB[11]
#define  RB_PIN_TMR3        0x08                      // RW, TMR3 alternate pin enable: 0=TMR3/PWM3/CAP3 on PA[2], 1=TMR3_/PWM3_/CAP3_ on PB[22]
#define  RB_PIN_UART0       0x10                      // RW, RXD0/TXD0 alternate pin enable: 0=RXD0/TXD0 on PB[4]/PB[7], 1=RXD0_/TXD0_ on PA[15]/PA[14]
#define  RB_PIN_UART1       0x20                      // RW, RXD1/TXD1 alternate pin enable: 0=RXD1/TXD1 on PA[8]/PA[9], 1=RXD1_/TXD1_ on PB[12]/PB[13]
#define  RB_PIN_UART2       0x40                      // RW, RXD2/TXD2 alternate pin enable: 0=RXD2/TXD2 on PA[6]/PA[7], 1=RXD2_/TXD2_ on PB[22]/PB[23]
#define  RB_PIN_UART3       0x80                      // RW, RXD3/TXD3 alternate pin enable: 0=RXD3/TXD3 on PA[4]/PA[5], 1=RXD3_/TXD3_ on PB[20]/PB[21]
#define  RB_PIN_SPI0        0x100                     // RW, SCS/SCK0/MOSI/MISO alternate pin enable: 0=SCS/SCK0/MOSI/MISO on PA[12]/PA[13]/PA[14]/PA[15], 1=SCS_/SCK0_/MOSI_/MISO_ on PB[12]/PB[13]/PB[14]/PB[15]
#define  RB_PIN_PWMX        0x400                     // RW, PWM4/PWM5/PWM7/PWM8/PWM9 alternate pin enable: 0=PWM4/5/7/8/9 on PA[12]/PA[13]/PB[4]/PB[6]/PB[7], 1=PWM4/5/7/8/9 on PA[6]/PA[7]/PB[1]/PB[2]/P[3]
#define  RB_PIN_I2C         0x800                     // RW, SCL/SDA alternate pin enable: 0=SCL/SDA on PB[13]/PB[12], 1=SCL_/SDA_ on PB[21]/PB[20]
#define  RB_PIN_MODEM       0x1000                    // RW, DSR/DTR alternate pin enable: 0=DSR/DTR on PB[1]/PB[5], 1=DSR_/DTR_ on PB[14]/PB[15]
#define  RB_RF_ANT_SW_EN    0x8000                    // RW, RF antenna switch control output enable: 0=disable output, 1=output on PB[16]/PB[17]/PB[18]/PB[19]/PB[20]/PB[21]
#define R16_PIN_ANALOG_IE   (*((vu16*)0x4000101A)) // RW, analog pin enable and digital input disable
#define  RB_PIN_USB_DP_PU   0x40                      // RW, USB UDP internal pullup resistance enable: 0=enable/disable by RB_UC_DEV_PU_EN, 1=enable pullup, replace RB_UC_DEV_PU_EN under sleep mode
#define  RB_PIN_USB_IE      0x80                      // RW, USB analog I/O enable: 0=analog I/O disable, 1=analog I/O enable
#define R32_PIN_CONFIG2     (*((vu32*)0x4000101C)) // RW, I/O pin configuration
#define  RB_PIN_PA4_15_DIS  0xFFF0                    // RW, PA4-PA15 digital input disable
#define  RB_PIN_PB0_DIS     0x10000                   // RW, PB0 digital input disable
#define  RB_PIN_PB4_DIS     0x100000                  // RW, PB4 digital input disable
#define  RB_PIN_PB6_7_DIS   0xC00000                  // RW, PB6-PB7 digital input disable
#define  RB_PIN_PB22_23_DIS 0x3000000                 // RW, PB22-PB23 digital input disable
#define  RB_PIN_PB10_15_DIS 0xFC000000                // RW, PB10-PB15 digital input disable

/* System: power management register */
#define R32_POWER_MANAG     (*((vu32*)0x40001020)) // RWA, power management register, SAM
#define R16_POWER_PLAN      (*((vu16*)0x40001020)) // RWA, power plan before sleep instruction, SAM
#define  RB_PWR_XROM        0x01                      // RWA, power for flash ROM
#define  RB_PWR_RAM2K       0x02                      // RWA, power for retention 2KB SRAM
#define  RB_PWR_CORE        0x04                      // RWA, power retention for core and base peripherals
#define  RB_PWR_EXTEND      0x08                      // RWA, power retention for USB and BLE
#define  RB_PWR_RAM24K      0x10                      // RWA, power for main SRAM
#define  RB_MAIN_ACT        0x40                      // RWA, main power chose
#define  RB_PWR_SYS_EN      0x80                      // RWA, power for system
#define  RB_PWR_LDO_EN      0x0100                    // RWA, LDO enable
#define  RB_PWR_DCDC_EN     0x0200                    // RWA, DC/DC converter enable: 0=DC/DC disable and bypass, 1=DC/DC enable
#define  RB_PWR_DCDC_PRE    0x0400                    // RWA, DC/DC converter pre-enable
#define  RB_XT_PRE_CFG      0x1800                    // RWA, extern 32MHz HSE early wake up time configuration
#define  RB_PWR_MUST_0      0x2000                    // RWA, must write 0
#define  RB_XT_PRE_EN       0x4000                    // RWA, extern 32MHz HSE early wake up enable, must be used with LSI/LSE
#define  RB_PWR_PLAN_EN     0x8000                    // RWA/WZ, power plan enable, auto clear after sleep executed
#define R16_AUX_POWER_ADJ   (*((vu16*)0x40001022))  // RWA, aux power adjust control, SAM
#define  RB_ULPLDO_ADJ      0x0007                    // RWA, Ultra-Low-Power LDO voltage adjust
#define  RB_DCDC_CHARGE     0x0080                    // RWA, DC/DC aux charge enable
#define  RB_IPU_TKEY_SEL    0xC000                    // RWA, TouchKey wakeup

/* System: battery detector register */
#define R32_BATTERY_CTRL    (*((vu32*)0x40001024)) // RWA, battery voltage detector, SAM
#define R8_BAT_DET_CTRL     (*((vu8*)0x40001024))  // RWA, battery voltage detector control, SAM
#define  RB_BAT_DET_EN      0x01                      // RWA, battery voltage detector enable if RB_BAT_MON_EN=0
#define  RB_BAT_LOW_VTHX    0x01                      // RWA, select monitor threshold voltage if RB_BAT_MON_EN=1
#define  RB_BAT_MON_EN      0x02                      // RWA, battery voltage monitor enable under sleep mode
#define  RB_BAT_LOWER_IE    0x04                      // RWA, interrupt enable for battery lower voltage
#define  RB_BAT_LOW_IE      0x08                      // RWA, interrupt enable for battery low voltage
// request NMI interrupt if both RB_BAT_LOWER_IE and RB_BAT_LOW_IE enabled
#define R8_BAT_DET_CFG      (*((vu8*)0x40001025))  // RWA, battery voltage detector configuration, SAM
#define  RB_BAT_LOW_VTH     0x03                      // RWA, select detector/monitor threshold voltage of battery voltage low
#define R8_BAT_STATUS       (*((vu8*)0x40001026))  // RO, battery status
#define  RB_BAT_STAT_LOWER  0x01                      // RO, battery lower voltage status for detector, high action
#define  RB_BAT_STAT_LOW    0x02                      // RO, battery low voltage status for detector/monitor, high action

/* System: 32KHz oscillator control register */
#define R32_OSC32K_CTRL     (*((vu32*)0x4000102C)) // RWA, 32KHz oscillator control, SAM
#define R16_INT32K_TUNE     (*((vu16*)0x4000102C)) // RWA, internal 32KHz oscillator tune control, SAM
#define  RB_INT32K_TUNE     0x1FFF                    // RWA, internal 32KHz oscillator frequency tune
#define R8_XT32K_TUNE       (*((vu8*)0x4000102E))  // RWA, external 32KHz oscillator tune control, SAM
#define  RB_XT32K_I_TUNE    0x03                      // RWA, external 32KHz oscillator current tune: 00=75% current, 01=standard current, 10=150% current, 11=200% current for startup
#define  RB_XT32K_C_LOAD    0xF0                      // RWA, external 32KHz oscillator load capacitor tune: Cap = RB_XT32K_C_LOAD + 12pF
#define R8_CK32K_CONFIG     (*((vu8*)0x4000102F))  // RWA, 32KHz oscillator configure
#define  RB_CLK_XT32K_PON   0x01                      // RWA, external 32KHz oscillator power on
#define  RB_CLK_INT32K_PON  0x02                      // RWA, internal 32KHz oscillator power on
#define  RB_CLK_OSC32K_XT   0x04                      // RWA, 32KHz oscillator source selection: 0=RC, 1=XT
#define  RB_CLK_OSC32K_FILT 0x08                      // RWA, internal 32KHz oscillator low noise mode disable: 1=enable, 0=disable
#define  RB_32K_CLK_PIN     0x80                      // RO, 32KHz oscillator clock pin status

/* System: real-time clock register */
#define R32_RTC_CTRL        (*((vu32*)0x40001030)) // RWA, RTC control, SAM
#define R8_RTC_FLAG_CTRL    (*((vu8*)0x40001030))  // RW, RTC flag and clear control
#define  RB_RTC_TMR_CLR     0x10                      // RW, set 1 to clear RTC timer action flag, auto clear
#define  RB_RTC_TRIG_CLR    0x20                      // RW, set 1 to clear RTC trigger action flag, auto clear
#define  RB_RTC_TMR_FLAG    0x40                      // RO, RTC timer action flag
#define  RB_RTC_TRIG_FLAG   0x80                      // RO, RTC trigger action flag
#define R8_RTC_MODE_CTRL    (*((vu8*)0x40001031))  // RWA, RTC mode control, SAM
#define  RB_RTC_TMR_MODE    0x07                      // RWA, RTC timer mode: 000=0.125S, 001=0.25S, 010=0.5S, 011=1S, 100=2S, 101=4S, 110=8S, 111=16S
#define  RB_RTC_IGNORE_B0   0x08                      // RWA, force ignore bit0 for trigger mode: 0=compare bit0, 1=ignore bit0
#define  RB_RTC_TMR_EN      0x10                      // RWA, RTC timer mode enable
#define  RB_RTC_TRIG_EN     0x20                      // RWA, RTC trigger mode enable
#define  RB_RTC_LOAD_LO     0x40                      // RWA, set 1 to load RTC count low word R32_RTC_CNT_32K, auto clear after loaded
#define  RB_RTC_LOAD_HI     0x80                      // RWA, set 1 to load RTC count high word R32_RTC_CNT_DAY, auto clear after loaded
#define R32_RTC_TRIG        (*((vu32*)0x40001034)) // RWA, RTC trigger value, SAM
#define R32_RTC_CNT_32K     (*((vu32*)0x40001038)) // RO, RTC count based 32KHz
#define R16_RTC_CNT_32K     (*((vu16*)0x40001038)) // RO, RTC count based 32KHz
#define R16_RTC_CNT_2S      (*((vu16*)0x4000103A)) // RO, RTC count based 2 second
#define R32_RTC_CNT_DAY     (*((vu32*)0x4000103C)) // RO, RTC count based one day, only low 14 bit

#define RTC_MAX_COUNT       0xA8C00000
#define RTC_FREQ            32000 // LSI
// #define RTC_FREQ            32768 // LSE
#define CLK_PER_US          (1.0 / ((1.0 / RTC_FREQ) * 1000 * 1000))
#define CLK_PER_MS          (CLK_PER_US * 1000)
#define US_TO_RTC(us)       ((uint32_t)((us) * CLK_PER_US + 0.5))
#define MS_TO_RTC(ms)       ((uint32_t)((ms) * CLK_PER_MS + 0.5))
#define RTC_WAIT_TICKS(t)   uint32_t rtcset = R32_RTC_CNT_32K +(t); while(R32_RTC_CNT_32K <= rtcset)
#define SLEEP_RTC_MIN_TIME  US_TO_RTC(1000)
#define SLEEP_RTC_MAX_TIME  (RTC_MAX_COUNT - 1000 * 1000 * 30)
#define WAKE_UP_RTC_MAX_TIME US_TO_RTC(1600)

/* System: safe accessing register */
#define R32_SAFE_ACCESS     (*((vu32*)0x40001040)) // RW, safe accessing
#define R8_SAFE_ACCESS_SIG  (*((vu8*)0x40001040))  // WO, safe accessing sign register, must write SAFE_ACCESS_SIG1 then SAFE_ACCESS_SIG2 to enter safe accessing mode
#define  RB_SAFE_ACC_MODE   0x03                      // RO, current safe accessing mode: 11=safe/unlocked (SAM), other=locked (00..01..10..11)
#define  RB_SAFE_ACC_ACT    0x08                      // RO, indicate safe accessing status now: 0=locked, read only, 1=safe/unlocked (SAM), write enabled
#define  RB_SAFE_ACC_TIMER  0x70                      // RO, safe accessing timer bit mask (16*clock number)
#define SAFE_ACCESS_SIG1    0x57                      // WO: safe accessing sign value step 1
#define SAFE_ACCESS_SIG2    0xA8                      // WO: safe accessing sign value step 2
#define SAFE_ACCESS_SIG0    0x00                      // WO: safe accessing sign value for disable
#define R8_CHIP_ID          (*((vu8*)0x40001041))  // RF, chip ID register, always is ID_CH59*
#define R8_SAFE_ACCESS_ID   (*((vu8*)0x40001042))  // RF, safe accessing ID register, always 0x0C
#define R8_WDOG_COUNT       (*((vu8*)0x40001043))  // RW, watch-dog count, count by clock frequency Fsys/131072
#define SYS_SAFE_ACCESS(a)  do { R8_SAFE_ACCESS_SIG = SAFE_ACCESS_SIG1; \
								 R8_SAFE_ACCESS_SIG = SAFE_ACCESS_SIG2; \
								 asm volatile ("nop\nnop"); \
								 {a} \
								 R8_SAFE_ACCESS_SIG = SAFE_ACCESS_SIG0; \
								 asm volatile ("nop\nnop"); } while(0)

/* System: global configuration register */
#define R32_GLOBAL_CONFIG   (*((vu32*)0x40001044)) // RW, global configuration
#define R8_RESET_STATUS     (*((vu8*)0x40001044))  // RO, reset status
#define  RB_RESET_FLAG      0x07                      // RO: recent reset flag
#define  RST_FLAG_SW        0x00
#define  RST_FLAG_RPOR      0x01
#define  RST_FLAG_WTR       0x02
#define  RST_FLAG_MR        0x03
//#define  RST_FLAG_GPWSM     0x04                      // RO, power on reset flag during sleep/shutdown: 0=no power on reset during sleep/shutdown, 1=power on reset occurred during sleep/shutdown
#define  RST_FLAG_GPWSM     0x05
// RB_RESET_FLAG: recent reset flag
//   000 - SR, software reset, by RB_SOFTWARE_RESET=1 @RB_WDOG_RST_EN=0
//   001 - RPOR, real power on reset
//   010 - WTR, watch-dog timer-out reset
//   011 - MR, external manual reset by RST pin input low
//   101 - GRWSM, global reset by waking under shutdown mode
//   1?? - LRW, power on reset occurred during sleep
#define R8_GLOB_ROM_CFG     R8_RESET_STATUS           // RWA, flash ROM configuration, SAM
#define  RB_ROM_CODE_OFS    0x10                      // RWA, code offset address selection in Flash ROM: 0=start address 0x000000, 1=start address 0x040000
#define  RB_ROM_CTRL_EN     0x20                      // RWA, enable flash ROM control interface enable: 0=disable access, 1=enable access control register
#define  RB_ROM_DATA_WE     0x40                      // RWA, enable flash ROM data & code area being erase/write: 0=all writing protect, 1=enable data area program and erase
#define  RB_ROM_CODE_WE     0x80                      // RWA, enable flash ROM code area being erase/write: 0=code writing protect, 1=enable code area program and erase
#define R8_GLOB_CFG_INFO    (*((vu8*)0x40001045))  // RO, global configuration information and status
#define  RB_CFG_ROM_READ    0x01                      // RO, indicate protected status of Flash ROM code and data: 0=reading protect, 1=enable read by external programmer
#define  RB_CFG_RESET_EN    0x04                      // RO, manual reset input enable status
#define  RB_CFG_BOOT_EN     0x08                      // RO, boot-loader enable status
#define  RB_CFG_DEBUG_EN    0x10                      // RO, debug enable status
#define  RB_BOOT_LOADER     0x20                      // RO, indicate boot loader status: 0=application status (by software reset), 1=boot loader status
#define R8_RST_WDOG_CTRL    (*((vu8*)0x40001046))  // RWA, reset and watch-dog control, SAM
#define  RB_SOFTWARE_RESET  0x01                      // WA/WZ, global software reset, high action, auto clear
#define  RB_WDOG_RST_EN     0x02                      // RWA, enable watch-dog reset if watch-dog timer overflow: 0=as timer only, 1=enable reset if timer overflow
#define  RB_WDOG_INT_EN     0x04                      // RWA, watch-dog timer overflow interrupt enable: 0=disable, 1=enable
#define  RB_WDOG_INT_FLAG   0x10                      // RW1, watch-dog timer overflow interrupt flag, cleared by RW1 or reload watch-dog count or __SEV(Send-Event)
#define R8_GLOB_RESET_KEEP  (*((vu8*)0x40001047))  // RW, value keeper during global reset

/*System: Miscellaneous Control register */
#define R32_MISC_CTRL       (*((vu32*)0x40001048)) // RWA, miscellaneous control register
#define R8_PLL_CONFIG       (*((vu8*)0x4000104B))  // RWA, PLL configuration control, SAM
#define  RB_PLL_CFG_DAT     0x7F                      // RWA, PLL configuration control, SAM

/* System: 32MHz oscillator control register */
#define R8_XT32M_TUNE       (*((vu8*)0x4000104E))  // RWA, external 32MHz oscillator tune control, SAM
#define  RB_XT32M_I_BIAS    0x03                      // RWA, external 32MHz oscillator bias current tune: 00=75% current, 01=standard current, 10=125% current, 11=150% current
#define  RB_XT32M_C_LOAD    0x70                      // RWA, external 32MHz oscillator load capacitor tune: Cap = RB_XT32M_C_LOAD * 2 + 10pF

/* System: oscillator frequency calibration register */
#define R32_OSC_CALIB       (*((vu32*)0x40001050)) // RWA, oscillator frequency calibration, SAM
#define R16_OSC_CAL_CNT     (*((vu16*)0x40001050)) // RO, system clock count value for 32KHz multi-cycles
#define  RB_OSC_CAL_CNT     0x3FFF                    // RO, system clock count value for 32KHz multi-cycles
#define  RB_OSC_CAL_OV_CLR  0x4000                    // RW1, indicate R8_OSC_CAL_OV_CNT not zero, set 1 to clear R8_OSC_CAL_OV_CNT
#define  RB_OSC_CAL_IF      0x8000                    // RW1, interrupt flag for oscillator capture end, set 1 to clear
#define R8_OSC_CAL_OV_CNT   (*((vu8*)0x40001052))  // RO, oscillator frequency calibration overflow times
#define R8_OSC_CAL_CTRL     (*((vu8*)0x40001053))  // RWA, oscillator frequency calibration control, SAM
#define  RB_OSC_CNT_TOTAL   0x07                      // RWA, total cycles mode for oscillator capture
// RB_OSC_CNT_TOTAL: select total cycles for oscillator capture
//    000: 1
//    001: 2
//    010: 4
//    011: 32
//    100: 64
//    101: 128
//    110: 1024
//    111: 2047
#define  RB_OSC_CNT_HALT    0x08                      // RO, calibration counter halt status: 0=counting, 1=halt for reading count value
#define  RB_OSC_CAL_IE      0x10                      // RWA, interrupt enable for oscillator capture end
#define  RB_OSC_CNT_EN      0x20                      // RWA, calibration counter enable
#define  RB_OSC_CNT_END     0x40                      // RWA, select oscillator capture end mode: 0=normal, 1=append 2 cycles

/* System: ADC and Touch-key register */
#define R32_ADC_CTRL        (*((vu32*)0x40001058)) // RW, ADC control
#define R8_ADC_CHANNEL      (*((vu8*)0x40001058))  // RW, ADC input channel selection
#define  RB_ADC_CH_INX      0x0F                      // RW, ADC input channel index
#define R8_ADC_CFG          (*((vu8*)0x40001059))  // RW, ADC configure
#define  RB_ADC_POWER_ON    0x01                      // RW, ADC power control: 0=power down, 1=power on
#define  RB_ADC_BUF_EN      0x02                      // RW, ADC input buffer enable 
#define  RB_ADC_DIFF_EN     0x04                      // RW, ADC input channel mode: 0=single-end, 1=differnetial
#define  RB_ADC_OFS_TEST    0x08                      // RW, enable ADC offset test mode: 0=normal mode, 1=short to test offset
#define  RB_ADC_PGA_GAIN    0x30                      // RW, set ADC input PGA gain: 00=-12dB, 01=-6dB, 10=0dB, 11=6dB
#define  RB_ADC_CLK_DIV     0xC0                      // RW, select ADC clock frequency: 00=3.2MHz, 01=8MHz, 10=5.33MHz, 11=4MHz
#define R8_ADC_CONVERT      (*((vu8*)0x4000105A))  // RW, ADC convert control
#define  RB_ADC_START       0x01                      // RW, ADC convert start control: 0=stop ADC convert, 1=start an ADC convert, auto clear
#define  RB_ADC_PGA_GAIN2   0x02                      // RW, ADC gain direction, must be 0 when using TS
#define  RB_ADC_EOC_X       0x80                      // RO, end of ADC conversion flag
#define R8_TEM_SENSOR       (*((vu8*)0x4000105B))  // RW, temperature sensor control
#define  RB_TEM_SEN_PWR_ON  0x80                      // RW, temperature sensor power control: 0=power down, 1=power on
#define R32_ADC_DATA        (*((vu32*)0x4000105C)) // RO, ADC data and status
#define R16_ADC_DATA        (*((vu16*)0x4000105C)) // RO, ADC data
#define  RB_ADC_DATA        0x0FFF                    // RO, ADC conversion data
#define R8_ADC_INT_FLAG     (*((vu8*)0x4000105E))  // RO, ADC interrupt flag register
#define  RB_ADC_IF_EOC      0x80                      // RO, ADC conversion interrupt flag: 0=free or converting, 1=end of conversion, interrupt action, auto ADC or write R8_ADC_CONVERT or write R8_TKEY_CONVERT to clear flag
#define R32_TKEY_CTRL       (*((vu8*)0x40001054))  // RW, Touchkey control
#define R8_TKEY_COUNT       (*((vu8*)0x40001054))  // RW, Touchkey charge and discharge count
#define  RB_TKEY_CHARG_CNT  0x1F                      // RW, Touchkey charge count
#define  RB_TKEY_DISCH_CNT  0xE0                      // RW, Touchkey discharge count
#define R8_TKEY_CONVERT     (*((vu8*)0x40001056))  // RW, Touchkey convert control
#define  RB_TKEY_START      0x01                      // RW, Touchkey convert start control: 0=stop Touchkey convert, 1=start a Touchkey convert, auto clear
#define R8_TKEY_CFG         (*((vu8*)0x40001057))  // RW, Touchkey configure
#define  RB_TKEY_PWR_ON     0x01                      // RW, Touchkey power on: 0=power down, 1=power on
#define  RB_TKEY_CURRENT    0x02                      // RW, Touchkey charge current selection: 0=35uA, 1=70uA
#define  RB_TKEY_DRV_EN     0x04                      // RW, Touchkey drive shield enable
#define  RB_TKEY_PGA_ADJ    0x08                      // RW, ADC input PGA speed selection: 0=slow, 1=fast
#define  RB_TKEY_DMA_EN     0x10                      // RW, Touchkey DMA enable
#define  RB_TKEY_AUTO_EN    0x20                      // RW, Touchkey auto-trigger enable
#define  RB_TKEY_RAND_EN    0x40                      // RW, Touchkey random trigger enable
#define R32_ADC_DMA_CTRL    (*((vu32*)0x40001060)) // RW, ADC DMA control
#define R8_ADC_CTRL_DMA     (*((vu8*)0x40001061))  // RW, ADC DMA control
#define  RB_ADC_DMA_ENABLE  0x01                      // RW, ADC DMA enable
#define  RB_ADC_DMA_LOOP    0x04                      // RW, ADC DMA address loop enable
#define  RB_ADC_IE_DMA_END  0x08                      // RW, enable interrupt for ADC DMA completion
#define  RB_ADC_IE_EOC      0x10                      // RW, enable interrupt for end of ADC conversion
#define  RB_ADC_CONT_EN     0x40                      // RW, enable contineous conversion ADC
#define  RB_ADC_AUTO_EN     0x80                      // RW, enable auto continuing ADC for DMA
#define R8_ADC_DMA_IF       (*((vu8*)0x40001062))  // RW1, ADC interrupt flag
#define  RB_ADC_IF_DMA_END  0x08                      // RW1, interrupt flag for ADC DMA completion
#define  RB_ADC_IF_END_ADC  0x10                      // RW1, interrupt flag for end of ADC conversion, DMA for auto ADC or write R8_ADC_CONVERT to clear flag
#define R8_ADC_AUTO_CYCLE   (*((vu8*)0x40001063))  // RW, auto ADC cycle value, unit is 16 Fsys
#define R32_ADC_DMA_NOW     (*((vu32*)0x40001064)) // RW, ADC DMA current address
#define R16_ADC_DMA_NOW     (*((vu16*)0x40001064)) // RW, ADC DMA current address
#define R32_ADC_DMA_BEG     (*((vu32*)0x40001068)) // RW, ADC DMA begin address
#define R16_ADC_DMA_BEG     (*((vu16*)0x40001068)) // RW, ADC DMA begin address
#define R32_ADC_DMA_END     (*((vu32*)0x4000106C)) // RW, ADC DMA end address
#define R16_ADC_DMA_END     (*((vu16*)0x4000106C)) // RW, ADC DMA end address

/* System: GPIO interrupt control register */
#define R32_GPIO_INT_EN     (*((vu32*)0x40001090)) // RW, GPIO interrupt enable
#define R16_PA_INT_EN       (*((vu16*)0x40001090)) // RW, GPIO PA interrupt enable
#define R16_PB_INT_EN       (*((vu16*)0x40001092)) // RW, GPIO PB interrupt enable
#define R32_GPIO_INT_MODE   (*((vu32*)0x40001094)) // RW, GPIO interrupt mode: 0=level action, 1=edge action
#define R16_PA_INT_MODE     (*((vu16*)0x40001094)) // RW, GPIO PA interrupt mode: 0=level action, 1=edge action
#define R16_PB_INT_MODE     (*((vu16*)0x40001096)) // RW, GPIO PB interrupt mode: 0=level action, 1=edge action
#define R32_GPIO_INT_IF     (*((vu32*)0x4000109C)) // RW1, GPIO interrupt flag
#define R16_PA_INT_IF       (*((vu16*)0x4000109C)) // RW1, GPIO PA interrupt flag
#define R16_PB_INT_IF       (*((vu16*)0x4000109E)) // RW1, GPIO PB interrupt flag

/* GPIO PA register */
#define R32_PA_DIR          (*((vu32*)0x400010A0)) // RW, GPIO PA I/O direction: 0=in, 1=out
#define R32_PA_PIN          (*((vu32*)0x400010A4)) // RO, GPIO PA input
#define R32_PA_OUT          (*((vu32*)0x400010A8)) // RW, GPIO PA output
#define R32_PA_CLR          (*((vu32*)0x400010AC)) // WZ, GPIO PA clear output: 0=keep, 1=clear
#define R32_PA_PU           (*((vu32*)0x400010B0)) // RW, GPIO PA pullup resistance enable
#define R32_PA_PD_DRV       (*((vu32*)0x400010B4)) // RW, PA pulldown for input or PA driving capability for output

/* GPIO PB register */
#define R32_PB_DIR          (*((vu32*)0x400010C0)) // RW, GPIO PB I/O direction: 0=in, 1=out
#define R32_PB_PIN          (*((vu32*)0x400010C4)) // RO, GPIO PB input
#define R32_PB_OUT          (*((vu32*)0x400010C8)) // RW, GPIO PB output
#define R32_PB_CLR          (*((vu32*)0x400010CC)) // WZ, GPIO PB clear output: 0=keep, 1=clear
#define R32_PB_PU           (*((vu32*)0x400010D0)) // RW, GPIO PB pullup resistance enable
#define R32_PB_PD_DRV       (*((vu32*)0x400010D4)) // RW, PB pulldown for input or PB driving capability for output

#define PA0      (0x00000001) /*!< Pin 0 selected */
#define PA1      (0x00000002) /*!< Pin 1 selected */
#define PA2      (0x00000004) /*!< Pin 2 selected */
#define PA3      (0x00000008) /*!< Pin 3 selected */
#define PA4      (0x00000010) /*!< Pin 4 selected */
#define PA5      (0x00000020) /*!< Pin 5 selected */
#define PA6      (0x00000040) /*!< Pin 6 selected */
#define PA7      (0x00000080) /*!< Pin 7 selected */
#define PA8      (0x00000100) /*!< Pin 8 selected */
#define PA9      (0x00000200) /*!< Pin 9 selected */
#define PA10     (0x00000400) /*!< Pin 10 selected */
#define PA11     (0x00000800) /*!< Pin 11 selected */
#define PA12     (0x00001000) /*!< Pin 12 selected */
#define PA13     (0x00002000) /*!< Pin 13 selected */
#define PA14     (0x00004000) /*!< Pin 14 selected */
#define PA15     (0x00008000) /*!< Pin 15 selected */

#define PB       (0x80000000) /* Bit mask to indicate bank B */
#define PB0      (0x80000001) /*!< Pin 0 selected */
#define PB1      (0x80000002) /*!< Pin 1 selected */
#define PB2      (0x80000004) /*!< Pin 2 selected */
#define PB3      (0x80000008) /*!< Pin 3 selected */
#define PB4      (0x80000010) /*!< Pin 4 selected */
#define PB5      (0x80000020) /*!< Pin 5 selected */
#define PB6      (0x80000040) /*!< Pin 6 selected */
#define PB7      (0x80000080) /*!< Pin 7 selected */
#define PB8      (0x80000100) /*!< Pin 8 selected */
#define PB9      (0x80000200) /*!< Pin 9 selected */
#define PB10     (0x80000400) /*!< Pin 10 selected */
#define PB11     (0x80000800) /*!< Pin 11 selected */
#define PB12     (0x80001000) /*!< Pin 12 selected */
#define PB13     (0x80002000) /*!< Pin 13 selected */
#define PB14     (0x80004000) /*!< Pin 14 selected */
#define PB15     (0x80008000) /*!< Pin 15 selected */
#define PB16     (0x80010000) /*!< Pin 16 selected */
#define PB17     (0x80020000) /*!< Pin 17 selected */
#define PB18     (0x80040000) /*!< Pin 18 selected */
#define PB19     (0x80080000) /*!< Pin 19 selected */
#define PB20     (0x80100000) /*!< Pin 20 selected */
#define PB21     (0x80200000) /*!< Pin 21 selected */
#define PB22     (0x80400000) /*!< Pin 22 selected */
#define PB23     (0x80800000) /*!< Pin 23 selected */
#define P_All    (0xFFFFFFFF) /*!< All pins selected */

/* GPIO alias name */
#define  bAIN9   (1<<0)    // PA0
#define  bSCK1   (1<<0)    // PA0
#define  bAIN8   (1<<1)    // PA1
#define  bSDO    (1<<1)    // PA1
#define  bMOSI1  bSDO
#define  bAIN7   (1<<2)    // PA2
#define  bSDI    (1<<2)    // PA2
#define  bMISO1  bSDI
#define  bAIN6   (1<<3)    // PA3
#define  bAIN0   (1<<4)    // PA4
#define  bRXD3   (1<<4)    // PA4
#define  bRXD3_  (1<<4)    // PA4
#define  bAIN1   (1<<5)    // PA5
#define  bTXD3   (1<<5)    // PA5
#define  bTXD3_  (1<<5)    // PA5
#define  bAIN10  (1<<6)    // PA6
#define  bRXD2_  (1<<6)    // PA6
#define  bPWM4_  (1<<6)    // PA6
#define  bAIN11  (1<<7)    // PA7
#define  bTXD2_  (1<<7)    // PA7
#define  bPWM5_  (1<<7)    // PA7
#define  bAIN12  (1<<8)    // PA8
#define  bRXD1   (1<<8)    // PA8
#define  bAIN13  (1<<9)    // PA9
#define  bTMR0   (1<<9)    // PA9
#define  bCAP0   bTMR0
#define  bPWM0   bTMR0
#define  bTXD1   (1<<9)    // PA9
#define  bX32KI  (1<<10)   // PA10
#define  bTMR1   (1<<10)   // PA10
#define  bCAP1   bTMR1
#define  bPWM1   bTMR1
#define  bX32KO  (1<<11)   // PA11
#define  bTMR2   (1<<11)   // PA11
#define  bCAP2   bTMR2
#define  bPWM2   bTMR2
#define  bAIN2   (1<<12)   // PA12
#define  bPWM4   (1<<12)   // PA12
#define  bSCS    (1<<12)   // PA12
#define  bAIN3   (1<<13)   // PA13
#define  bSCK0   (1<<13)   // PA13
#define  bPWM5   (1<<13)   // PA13
#define  bAIN4   (1<<14)   // PA14
#define  bMOSI   (1<<14)   // PA14
#define  bTXD0_  (1<<14)   // PA14
#define  bAIN5   (1<<15)   // PA15
#define  bMISO   (1<<15)   // PA15
#define  bRXD0_  (1<<15)   // PA15
#define  bPWM6   (1<<0)    // PB0
#define  bCTS    (1<<0)    // PB0
#define  bDSR    (1<<1)    // PB1
#define  bPWM7_  (1<<1)    // PB1
#define  bRI     (1<<2)    // PB2
#define  bPWM8_  (1<<2)    // PB2
#define  bDCD    (1<<3)    // PB3
#define  bPWM9_  (1<<3)    // PB3
#define  bPWM7   (1<<4)    // PB4
#define  bRXD0   (1<<4)    // PB4
#define  bDTR    (1<<5)    // PB5
#define  bRTS    (1<<6)    // PB6
#define  bPWM8   (1<<6)    // PB6
#define  bTXD0   (1<<7)    // PB7
#define  bPWM9   (1<<7)    // PB7
#define  bUDM    (1<<10)   // PB10
#define  bTMR1_  (1<<10)   // PB10
#define  bCAP1_  bTMR1_
#define  bPWM1_  bTMR1_
#define  bUDP    (1<<11)   // PB11
#define  bTMR2_  (1<<11)   // PB11
#define  bCAP2_  bTMR2_
#define  bPWM2_  bTMR2_
#define  bU2DM   (1<<12)   // PB12
#define  bSCS_   (1<<12)   // PB12
#define  bRXD1_  (1<<12)   // PB12
#define  bU2DP   (1<<13)   // PB13
#define  bSCK0_  (1<<13)   // PB13
#define  bTXD1_  (1<<13)   // PB13
#define  bTIO    (1<<14)   // PB14
#define  bDSR_   (1<<14)   // PB14
#define  bMOSI_  (1<<14)   // PB14
#define  bSDA    (1<<14)   // PB14
#define  bPWM10  (1<<14)   // PB14
#define  bTCK    (1<<15)   // PB15
#define  bMISO_  (1<<15)   // PB15
#define  bSCL    (1<<15)   // PB15
#define  bDTR_   (1<<15)   // PB15
#define  bRXD2   (1<<22)   // PB22
#define  bTMR3   (1<<22)   // PB22
#define  bCAP3   bTMR3
#define  bPWM3   bTMR3
#define  bTMR3_  (1<<22)   // PB22
#define  bCAP3_  bTMR3_
#define  bPWM3_  bTMR3_
#define  bRST    (1<<23)   // PB23
#define  bTMR0_  (1<<23)   // PB23
#define  bCAP0_  bTMR0_
#define  bPWM0_  bTMR0_
#define  bTXD2   (1<<23)   // PB23
#define  bPWM11  (1<<23)   // PB23

typedef enum
{
	GPIO_ModeIN_Floating,
	GPIO_ModeIN_PU,
	GPIO_ModeIN_PD,
	GPIO_ModeOut_PP_5mA,
	GPIO_ModeOut_PP_20mA,
} GPIOModeTypeDef;

/* General Purpose I/O */
typedef enum
{
	GPIO_CFGLR_IN_FLOAT = GPIO_ModeIN_Floating,
	GPIO_CFGLR_IN_PUPD = GPIO_ModeIN_PU, // is most common
	GPIO_CFGLR_IN_PU = GPIO_ModeIN_PU,
	GPIO_CFGLR_IN_PD = GPIO_ModeIN_PD, // to suppress the -Wswitch warning
	GPIO_CFGLR_OUT_10Mhz_PP = GPIO_ModeOut_PP_20mA,
	GPIO_CFGLR_OUT_2Mhz_PP = GPIO_ModeOut_PP_5mA,
	GPIO_CFGLR_OUT_50Mhz_PP = GPIO_ModeOut_PP_20mA,
} GPIO_CFGLR_PIN_MODE_Typedef;

/* System: Flash ROM control register */
#define R32_FLASH_DATA      (*((vu32*)0x40001800)) // RO/WO, flash ROM data
#define R32_FLASH_CONTROL   (*((vu32*)0x40001804)) // RW, flash ROM control
#define R8_FLASH_DATA       (*((vu8*)0x40001804))  // RO/WO, flash ROM data buffer
#define R8_FLASH_CTRL       (*((vu8*)0x40001806))  // RW, flash ROM access control
#define R8_FLASH_CFG        (*((vu8*)0x40001807))  // RW, flash ROM access config, SAM
/* Timer0 register */
#define R32_TMR0_CONTROL    (*((vu32*)0x40002000)) // RW, TMR0 control
#define R8_TMR0_CTRL_MOD    (*((vu8*)0x40002000))  // RW, TMR0 mode control
#define R8_TMR0_INTER_EN    (*((vu8*)0x40002002))  // RW, TMR0 interrupt enable
// #define R32_TMR0_STATUS     (*((vu32*)0x40002004)) // RW, TMR0 status
#define R8_TMR0_INT_FLAG    (*((vu8*)0x40002006))  // RW1, TMR0 interrupt flag
#define R8_TMR0_FIFO_COUNT  (*((vu8*)0x40002007))  // RO, TMR0 FIFO count status
#define R32_TMR0_COUNT      (*((vu32*)0x40002008)) // RO, TMR0 current count
#define R16_TMR0_COUNT      (*((vu16*)0x40002008)) // RO, TMR0 current count
#define R8_TMR0_COUNT       (*((vu8*)0x40002008))  // RO, TMR0 current count
#define R32_TMR0_CNT_END    (*((vu32*)0x4000200C)) // RW, TMR0 end count value, only low 26 bit
#define R32_TMR0_FIFO       (*((vu32*)0x40002010)) // RO/WO, TMR0 FIFO register, only low 26 bit
#define R16_TMR0_FIFO       (*((vu16*)0x40002010)) // RO/WO, TMR0 FIFO register
#define R8_TMR0_FIFO        (*((vu8*)0x40002010))  // RO/WO, TMR0 FIFO register

/* Timer1 register */
#define R32_TMR1_CONTROL    (*((vu32*)0x40002400)) // RW, TMR1 control
#define R8_TMR1_CTRL_MOD    (*((vu8*)0x40002400))  // RW, TMR1 mode control
#define R8_TMR1_CTRL_DMA    (*((vu8*)0x40002401))  // RW, TMR1 DMA control
#define R8_TMR1_INTER_EN    (*((vu8*)0x40002402))  // RW, TMR1 interrupt enable
// #define R32_TMR1_STATUS     (*((vu32*)0x40002404)) // RW, TMR1 status
#define R8_TMR1_INT_FLAG    (*((vu8*)0x40002406))  // RW1, TMR1 interrupt flag
#define R8_TMR1_FIFO_COUNT  (*((vu8*)0x40002407))  // RO, TMR1 FIFO count status
#define R32_TMR1_COUNT      (*((vu32*)0x40002408)) // RO, TMR1 current count
#define R16_TMR1_COUNT      (*((vu16*)0x40002408)) // RO, TMR1 current count
#define R8_TMR1_COUNT       (*((vu8*)0x40002408))  // RO, TMR1 current count
#define R32_TMR1_CNT_END    (*((vu32*)0x4000240C)) // RW, TMR1 end count value, only low 26 bit
#define R32_TMR1_FIFO       (*((vu32*)0x40002410)) // RO/WO, TMR1 FIFO register, only low 26 bit
#define R16_TMR1_FIFO       (*((vu16*)0x40002410)) // RO/WO, TMR1 FIFO register
#define R8_TMR1_FIFO        (*((vu8*)0x40002410))  // RO/WO, TMR1 FIFO register
#define R32_TMR1_DMA_NOW    (*((vu32*)0x40002414)) // RW, TMR1 DMA current address
#define R16_TMR1_DMA_NOW    (*((vu16*)0x40002414)) // RW, TMR1 DMA current address
#define R32_TMR1_DMA_BEG    (*((vu32*)0x40002418)) // RW, TMR1 DMA begin address
#define R16_TMR1_DMA_BEG    (*((vu16*)0x40002418)) // RW, TMR1 DMA begin address
#define R32_TMR1_DMA_END    (*((vu32*)0x4000241C)) // RW, TMR1 DMA end address
#define R16_TMR1_DMA_END    (*((vu16*)0x4000241C)) // RW, TMR1 DMA end address

/* Timer2 register */
#define R32_TMR2_CONTROL    (*((vu32*)0x40002800)) // RW, TMR2 control
#define R8_TMR2_CTRL_MOD    (*((vu8*)0x40002800))  // RW, TMR2 mode control
#define R8_TMR2_CTRL_DMA    (*((vu8*)0x40002801))  // RW, TMR2 DMA control
#define R8_TMR2_INTER_EN    (*((vu8*)0x40002802))  // RW, TMR2 interrupt enable
// #define R32_TMR2_STATUS     (*((vu32*)0x40002804)) // RW, TMR2 status
#define R8_TMR2_INT_FLAG    (*((vu8*)0x40002806))  // RW1, TMR2 interrupt flag
#define R8_TMR2_FIFO_COUNT  (*((vu8*)0x40002807))  // RO, TMR2 FIFO count status
#define R32_TMR2_COUNT      (*((vu32*)0x40002808)) // RO, TMR2 current count
#define R16_TMR2_COUNT      (*((vu16*)0x40002808)) // RO, TMR2 current count
#define R8_TMR2_COUNT       (*((vu8*)0x40002808))  // RO, TMR2 current count
#define R32_TMR2_CNT_END    (*((vu32*)0x4000280C)) // RW, TMR2 end count value, only low 26 bit
#define R32_TMR2_FIFO       (*((vu32*)0x40002810)) // RO/WO, TMR2 FIFO register, only low 26 bit
#define R16_TMR2_FIFO       (*((vu16*)0x40002810)) // RO/WO, TMR2 FIFO register
#define R8_TMR2_FIFO        (*((vu8*)0x40002810))  // RO/WO, TMR2 FIFO register
#define R32_TMR2_DMA_NOW    (*((vu32*)0x40002814)) // RW, TMR2 DMA current address
#define R16_TMR2_DMA_NOW    (*((vu16*)0x40002814)) // RW, TMR2 DMA current address
#define R32_TMR2_DMA_BEG    (*((vu32*)0x40002818)) // RW, TMR2 DMA begin address
#define R16_TMR2_DMA_BEG    (*((vu16*)0x40002818)) // RW, TMR2 DMA begin address
#define R32_TMR2_DMA_END    (*((vu32*)0x4000281C)) // RW, TMR2 DMA end address
#define R16_TMR2_DMA_END    (*((vu16*)0x4000281C)) // RW, TMR2 DMA end address

/* Timer3 register */
#define R32_TMR3_CONTROL    (*((vu32*)0x40002C00)) // RW, TMR3 control
#define R8_TMR3_CTRL_MOD    (*((vu8*)0x40002C00))  // RW, TMR3 mode control
#define R8_TMR3_INTER_EN    (*((vu8*)0x40002C02))  // RW, TMR3 interrupt enable
// #define R32_TMR3_STATUS     (*((vu32*)0x40002C04)) // RW, TMR3 status
#define R8_TMR3_INT_FLAG    (*((vu8*)0x40002C06))  // RW1, TMR3 interrupt flag
#define R8_TMR3_FIFO_COUNT  (*((vu8*)0x40002C07))  // RO, TMR3 FIFO count status
#define R32_TMR3_COUNT      (*((vu32*)0x40002C08)) // RO, TMR3 current count
#define R16_TMR3_COUNT      (*((vu16*)0x40002C08)) // RO, TMR3 current count
#define R8_TMR3_COUNT       (*((vu8*)0x40002C08))  // RO, TMR3 current count
#define R32_TMR3_CNT_END    (*((vu32*)0x40002C0C)) // RW, TMR3 end count value, only low 26 bit
#define R32_TMR3_FIFO       (*((vu32*)0x40002C10)) // RO/WO, TMR3 FIFO register, only low 26 bit
#define R16_TMR3_FIFO       (*((vu16*)0x40002C10)) // RO/WO, TMR3 FIFO register
#define R8_TMR3_FIFO        (*((vu8*)0x40002C10))  // RO/WO, TMR3 FIFO register

/* Timer register address offset and bit define */
#define TMR_FIFO_SIZE       8                         // timer FIFO size (depth)
#define BA_TMR0             ((vu8*)0x40002000)     // point TMR0 base address
#define BA_TMR1             ((vu8*)0x40002400)     // point TMR1 base address
#define BA_TMR2             ((vu8*)0x40002800)     // point TMR2 base address
#define BA_TMR3             ((vu8*)0x40002C00)     // point TMR3 base address
#define TMR_CTRL_MOD        0
#define  RB_TMR_MODE_IN     0x01                      // RW, timer in mode: 0=timer/PWM, 1=capture/count
#define  RB_TMR_ALL_CLEAR   0x02                      // RW, force clear timer FIFO and count
#define  RB_TMR_COUNT_EN    0x04                      // RW, timer count enable
#define  RB_TMR_OUT_EN      0x08                      // RW, timer output enable
#define  RB_TMR_OUT_POLAR   0x10                      // RW, timer PWM output polarity: 0=default low and high action, 1=default high and low action
#define  RB_TMR_CAP_COUNT   0x10                      // RW, count sub-mode if RB_TMR_MODE_IN=1: 0=capture, 1=count
#define  RB_TMR_PWM_REPEAT  0xC0                      // RW, timer PWM repeat mode: 00=1, 01=4, 10=8, 11-16
#define  RB_TMR_CAP_EDGE    0xC0                      // RW, timer capture edge mode: 00=disable, 01=edge change, 10=fall to fall, 11-rise to rise
#define TMR_CTRL_DMA        1
#define  RB_TMR_DMA_ENABLE  0x01                      // RW, timer1/2 DMA enable
#define  RB_TMR_DMA_LOOP    0x04                      // RW, timer1/2 DMA address loop enable
#define TMR_INTER_EN        2
#define  RB_TMR_IE_CYC_END  0x01                      // RW, enable interrupt for timer capture count timeout or PWM cycle end
#define  RB_TMR_IE_DATA_ACT 0x02                      // RW, enable interrupt for timer capture input action or PWM trigger
#define  RB_TMR_IE_FIFO_HF  0x04                      // RW, enable interrupt for timer FIFO half (capture fifo >=4 or PWM fifo <=3)
#define  RB_TMR_IE_DMA_END  0x08                      // RW, enable interrupt for timer1/2 DMA completion
#define  RB_TMR_IE_FIFO_OV  0x10                      // RW, enable interrupt for timer FIFO overflow
#define TMR_INT_FLAG        6
#define  RB_TMR_IF_CYC_END  0x01                      // RW1, interrupt flag for timer capture count timeout or PWM cycle end
#define  RB_TMR_IF_DATA_ACT 0x02                      // RW1, interrupt flag for timer capture input action or PWM trigger
#define  RB_TMR_IF_FIFO_HF  0x04                      // RW1, interrupt flag for timer FIFO half (capture fifo >=4 or PWM fifo <=3)
#define  RB_TMR_IF_DMA_END  0x08                      // RW1, interrupt flag for timer1/2 DMA completion
#define  RB_TMR_IF_FIFO_OV  0x10                      // RW1, interrupt flag for timer FIFO overflow
#define TMR_FIFO_COUNT      7
#define TMR_COUNT           0x08
#define TMR_CNT_END         0x0C
#define TMR_FIFO            0x10
#define TMR_DMA_NOW         0x14
#define TMR_DMA_BEG         0x18
#define TMR_DMA_END         0x1C

/* UART0 register */
#define R32_UART0_CTRL      (*((vu32*)0x40003000)) // RW, UART0 control
#define R8_UART0_MCR        (*((vu8*)0x40003000))  // RW, UART0 modem control
#define R8_UART0_IER        (*((vu8*)0x40003001))  // RW, UART0 interrupt enable
#define R8_UART0_FCR        (*((vu8*)0x40003002))  // RW, UART0 FIFO control
#define R8_UART0_LCR        (*((vu8*)0x40003003))  // RW, UART0 line control
#define R32_UART0_STAT      (*((vu32*)0x40003004)) // RO, UART0 status
#define R8_UART0_IIR        (*((vu8*)0x40003004))  // RO, UART0 interrupt identification
#define R8_UART0_LSR        (*((vu8*)0x40003005))  // RO, UART0 line status
#define R8_UART0_MSR        (*((vu8*)0x40003006))  // RO, UART0 modem status
#define R32_UART0_FIFO      (*((vu32*)0x40003008)) // RW, UART0 data or FIFO port
#define R8_UART0_RBR        (*((vu8*)0x40003008))  // RO, UART0 receiver buffer, receiving byte
#define R8_UART0_THR        (*((vu8*)0x40003008))  // WO, UART0 transmitter holding, transmittal byte
#define R8_UART0_RFC        (*((vu8*)0x4000300A))  // RO, UART0 receiver FIFO count
#define R8_UART0_TFC        (*((vu8*)0x4000300B))  // RO, UART0 transmitter FIFO count
#define R32_UART0_SETUP     (*((vu32*)0x4000300C)) // RW, UART0 setup
#define R16_UART0_DL        (*((vu16*)0x4000300C)) // RW, UART0 divisor latch
#define R8_UART0_DLL        (*((vu8*)0x4000300C))  // RW, UART0 divisor latch LSB byte
// #define R8_UART0_DLM        (*((vu8*)0x4000300D))  // RW, UART0 divisor latch MSB byte
#define R8_UART0_DIV        (*((vu8*)0x4000300E))  // RW, UART0 pre-divisor latch byte, only low 7 bit, from 1 to 0/128
#define R8_UART0_ADR        (*((vu8*)0x4000300F))  // RW, UART0 slave address: 0xFF=disable, other=enable

/* UART1 register */
#define R32_UART1_CTRL      (*((vu32*)0x40003400)) // RW, UART1 control
#define R8_UART1_MCR        (*((vu8*)0x40003400))  // RW, UART1 modem control
#define R8_UART1_IER        (*((vu8*)0x40003401))  // RW, UART1 interrupt enable
#define R8_UART1_FCR        (*((vu8*)0x40003402))  // RW, UART1 FIFO control
#define R8_UART1_LCR        (*((vu8*)0x40003403))  // RW, UART1 line control
#define R32_UART1_STAT      (*((vu32*)0x40003404)) // RO, UART1 status
#define R8_UART1_IIR        (*((vu8*)0x40003404))  // RO, UART1 interrupt identification
#define R8_UART1_LSR        (*((vu8*)0x40003405))  // RO, UART1 line status
#define R32_UART1_FIFO      (*((vu32*)0x40003408)) // RW, UART1 data or FIFO port
#define R8_UART1_RBR        (*((vu8*)0x40003408))  // RO, UART1 receiver buffer, receiving byte
#define R8_UART1_THR        (*((vu8*)0x40003408))  // WO, UART1 transmitter holding, transmittal byte
#define R8_UART1_RFC        (*((vu8*)0x4000340A))  // RO, UART1 receiver FIFO count
#define R8_UART1_TFC        (*((vu8*)0x4000340B))  // RO, UART1 transmitter FIFO count
#define R32_UART1_SETUP     (*((vu32*)0x4000340C)) // RW, UART1 setup
#define R16_UART1_DL        (*((vu16*)0x4000340C)) // RW, UART1 divisor latch
#define R8_UART1_DLL        (*((vu8*)0x4000340C))  // RW, UART1 divisor latch LSB byte
// #define R8_UART1_DLM        (*((vu8*)0x4000340D))  // RW, UART1 divisor latch MSB byte
#define R8_UART1_DIV        (*((vu8*)0x4000340E))  // RW, UART1 pre-divisor latch byte, only low 7 bit, from 1 to 0/128

/* UART2 register */
#define R32_UART2_CTRL      (*((vu32*)0x40003800)) // RW, UART2 control
#define R8_UART2_MCR        (*((vu8*)0x40003800))  // RW, UART2 modem control
#define R8_UART2_IER        (*((vu8*)0x40003801))  // RW, UART2 interrupt enable
#define R8_UART2_FCR        (*((vu8*)0x40003802))  // RW, UART2 FIFO control
#define R8_UART2_LCR        (*((vu8*)0x40003803))  // RW, UART2 line control
#define R32_UART2_STAT      (*((vu32*)0x40003804)) // RO, UART2 status
#define R8_UART2_IIR        (*((vu8*)0x40003804))  // RO, UART2 interrupt identification
#define R8_UART2_LSR        (*((vu8*)0x40003805))  // RO, UART2 line status
#define R32_UART2_FIFO      (*((vu32*)0x40003808)) // RW, UART2 data or FIFO port
#define R8_UART2_RBR        (*((vu8*)0x40003808))  // RO, UART2 receiver buffer, receiving byte
#define R8_UART2_THR        (*((vu8*)0x40003808))  // WO, UART2 transmitter holding, transmittal byte
#define R8_UART2_RFC        (*((vu8*)0x4000380A))  // RO, UART2 receiver FIFO count
#define R8_UART2_TFC        (*((vu8*)0x4000380B))  // RO, UART2 transmitter FIFO count
#define R32_UART2_SETUP     (*((vu32*)0x4000380C)) // RW, UART2 setup
#define R16_UART2_DL        (*((vu16*)0x4000380C)) // RW, UART2 divisor latch
#define R8_UART2_DLL        (*((vu8*)0x4000380C))  // RW, UART2 divisor latch LSB byte
// #define R8_UART2_DLM        (*((vu8*)0x4000380D))  // RW, UART2 divisor latch MSB byte
#define R8_UART2_DIV        (*((vu8*)0x4000380E))  // RW, UART2 pre-divisor latch byte, only low 7 bit, from 1 to 0/128

/* UART3 register */
#define R32_UART3_CTRL      (*((vu32*)0x40003C00)) // RW, UART3 control
#define R8_UART3_MCR        (*((vu8*)0x40003C00))  // RW, UART3 modem control
#define R8_UART3_IER        (*((vu8*)0x40003C01))  // RW, UART3 interrupt enable
#define R8_UART3_FCR        (*((vu8*)0x40003C02))  // RW, UART3 FIFO control
#define R8_UART3_LCR        (*((vu8*)0x40003C03))  // RW, UART3 line control
#define R32_UART3_STAT      (*((vu32*)0x40003C04)) // RO, UART3 status
#define R8_UART3_IIR        (*((vu8*)0x40003C04))  // RO, UART3 interrupt identification
#define R8_UART3_LSR        (*((vu8*)0x40003C05))  // RO, UART3 line status
#define R32_UART3_FIFO      (*((vu32*)0x40003C08)) // RW, UART3 data or FIFO port
#define R8_UART3_RBR        (*((vu8*)0x40003C08))  // RO, UART3 receiver buffer, receiving byte
#define R8_UART3_THR        (*((vu8*)0x40003C08))  // WO, UART3 transmitter holding, transmittal byte
#define R8_UART3_RFC        (*((vu8*)0x40003C0A))  // RO, UART3 receiver FIFO count
#define R8_UART3_TFC        (*((vu8*)0x40003C0B))  // RO, UART3 transmitter FIFO count
#define R32_UART3_SETUP     (*((vu32*)0x40003C0C)) // RW, UART3 setup
#define R16_UART3_DL        (*((vu16*)0x40003C0C)) // RW, UART3 divisor latch
#define R8_UART3_DLL        (*((vu8*)0x40003C0C))  // RW, UART3 divisor latch LSB byte
// #define R8_UART3_DLM        (*((vu8*)0x40003C0D))  // RW, UART3 divisor latch MSB byte
#define R8_UART3_DIV        (*((vu8*)0x40003C0E))  // RW, UART3 pre-divisor latch byte, only low 7 bit, from 1 to 0/128

/* UART register address offset and bit define */
#define UART_FIFO_SIZE      8                         // UART FIFO size (depth)
#define UART_RECV_RDY_SZ    7                         // the max FIFO trigger level for UART receiver data available
#define BA_UART0            ((vu8*)0x40003000)     // point UART0 base address
#define BA_UART1            ((vu8*)0x40003400)     // point UART1 base address
#define BA_UART2            ((vu8*)0x40003800)     // point UART2 base address
#define BA_UART3            ((vu8*)0x40003C00)     // point UART3 base address
#define UART_MCR            0
#define  RB_MCR_DTR         0x01                      // RW, UART0 control DTR
#define  RB_MCR_RTS         0x02                      // RW, UART0 control RTS
#define  RB_MCR_OUT1        0x04                      // RW, UART0 control OUT1
#define  RB_MCR_OUT2        0x08                      // RW, UART control OUT2
#define  RB_MCR_INT_OE      0x08                      // RW, UART interrupt output enable
#define  RB_MCR_LOOP        0x10                      // RW, UART0 enable local loop back
#define  RB_MCR_AU_FLOW_EN  0x20                      // RW, UART0 enable autoflow control
#define  RB_MCR_TNOW        0x40                      // RW, UART0 enable TNOW output on DTR pin
#define  RB_MCR_HALF        0x80                      // RW, UART0 enable half-duplex
#define UART_IER            1
#define  RB_IER_RECV_RDY    0x01                      // RW, UART interrupt enable for receiver data ready
#define  RB_IER_THR_EMPTY   0x02                      // RW, UART interrupt enable for THR empty
#define  RB_IER_LINE_STAT   0x04                      // RW, UART interrupt enable for receiver line status
#define  RB_IER_MODEM_CHG   0x08                      // RW, UART0 interrupt enable for modem status change
#define  RB_IER_DTR_EN      0x10                      // RW, UART0 DTR/TNOW output pin enable
#define  RB_IER_RTS_EN      0x20                      // RW, UART0 RTS output pin enable
#define  RB_IER_TXD_EN      0x40                      // RW, UART TXD pin enable
#define  RB_IER_RESET       0x80                      // WZ, UART software reset control, high action, auto clear
#define UART_FCR            2
#define  RB_FCR_FIFO_EN     0x01                      // RW, UART FIFO enable
#define  RB_FCR_RX_FIFO_CLR 0x02                      // WZ, clear UART receiver FIFO, high action, auto clear
#define  RB_FCR_TX_FIFO_CLR 0x04                      // WZ, clear UART transmitter FIFO, high action, auto clear
#define  RB_FCR_FIFO_TRIG   0xC0                      // RW, UART receiver FIFO trigger level: 00-1byte, 01-2bytes, 10-4bytes, 11-7bytes
#define UART_LCR            3
#define  RB_LCR_WORD_SZ     0x03                      // RW, UART word bit length: 00-5bit, 01-6bit, 10-7bit, 11-8bit
#define  RB_LCR_STOP_BIT    0x04                      // RW, UART stop bit length: 0-1bit, 1-2bit
#define  RB_LCR_PAR_EN      0x08                      // RW, UART parity enable
#define  RB_LCR_PAR_MOD     0x30                      // RW, UART parity mode: 00-odd, 01-even, 10-mark, 11-space
#define  RB_LCR_BREAK_EN    0x40                      // RW, UART break control enable
#define  RB_LCR_DLAB        0x80                      // RW, UART reserved bit
#define  RB_LCR_GP_BIT      0x80                      // RW, UART general purpose bit
#define UART_IIR            4
#define  RB_IIR_NO_INT      0x01                      // RO, UART no interrupt flag: 0=interrupt action, 1=no interrupt
#define  RB_IIR_INT_MASK    0x0F                      // RO, UART interrupt flag bit mask
#define  RB_IIR_FIFO_ID     0xC0                      // RO, UART FIFO enabled flag
#define UART_LSR            5
#define  RB_LSR_DATA_RDY    0x01                      // RO, UART receiver fifo data ready status
#define  RB_LSR_OVER_ERR    0x02                      // RZ, UART receiver overrun error
#define  RB_LSR_PAR_ERR     0x04                      // RZ, UART receiver parity error
#define  RB_LSR_FRAME_ERR   0x08                      // RZ, UART receiver frame error
#define  RB_LSR_BREAK_ERR   0x10                      // RZ, UART receiver break error
#define  RB_LSR_TX_FIFO_EMP 0x20                      // RO, UART transmitter fifo empty status
#define  RB_LSR_TX_ALL_EMP  0x40                      // RO, UART transmitter all empty status
#define  RB_LSR_ERR_RX_FIFO 0x80                      // RO, indicate error in UART receiver fifo
#define UART_MSR            6
#define  RB_MSR_CTS_CHG     0x01                      // RZ, UART0 CTS changed status, high action
#define  RB_MSR_DSR_CHG     0x02                      // RZ, UART0 DSR changed status, high action
//#define  RB_MSR_RI_CHG      0x04                      // RZ, UART0 RI changed status, high action
//#define  RB_MSR_DCD_CHG     0x08                      // RZ, UART0 DCD changed status, high action
#define  RB_MSR_CTS         0x10                      // RO, UART0 CTS action status
#define  RB_MSR_DSR         0x20                      // RO, UART0 DSR action status
//#define  RB_MSR_RI          0x40                      // RO, UART0 RI action status
//#define  RB_MSR_DCD         0x80                      // RO, UART0 DCD action status
#define UART_RBR            8
#define UART_THR            8
#define UART_RFC            0x0A
#define UART_TFC            0x0B
#define UART_DLL            0x0C
// #define UART_DLM            0x0D
#define UART_DIV            0x0E
#define UART_ADR            0x0F

/* UART interrupt identification values for IIR bits 3:0 */
#define UART_II_SLV_ADDR    0x0E                      // RO, UART0 slave address match
#define UART_II_LINE_STAT   0x06                      // RO, UART interrupt by receiver line status
#define UART_II_RECV_RDY    0x04                      // RO, UART interrupt by receiver data available
#define UART_II_RECV_TOUT   0x0C                      // RO, UART interrupt by receiver fifo timeout
#define UART_II_THR_EMPTY   0x02                      // RO, UART interrupt by THR empty
#define UART_II_MODEM_CHG   0x00                      // RO, UART0 interrupt by modem status change
#define UART_II_NO_INTER    0x01                      // RO, no UART interrupt is pending

/* SPI0 register */
#define R32_SPI0_CONTROL    (*((vu32*)0x40004000)) // RW, SPI0 control
#define R8_SPI0_CTRL_MOD    (*((vu8*)0x40004000))  // RW, SPI0 mode control
#define R8_SPI0_CTRL_CFG    (*((vu8*)0x40004001))  // RW, SPI0 configuration control
#define R8_SPI0_INTER_EN    (*((vu8*)0x40004002))  // RW, SPI0 interrupt enable
#define R8_SPI0_CLOCK_DIV   (*((vu8*)0x40004003))  // RW, SPI0 master clock divisor
#define R8_SPI0_SLAVE_PRE   (*((vu8*)0x40004003))  // RW, SPI0 slave preset value
#define R32_SPI0_STATUS     (*((vu32*)0x40004004)) // RW, SPI0 status
#define R8_SPI0_BUFFER      (*((vu8*)0x40004004))  // RO, SPI0 data buffer
#define R8_SPI0_RUN_FLAG    (*((vu8*)0x40004005))  // RO, SPI0 work flag
#define R8_SPI0_INT_FLAG    (*((vu8*)0x40004006))  // RW1, SPI0 interrupt flag
#define R8_SPI0_FIFO_COUNT  (*((vu8*)0x40004007))  // RO, SPI0 FIFO count status
#define R32_SPI0_TOTAL_CNT  (*((vu32*)0x4000400C)) // RW, SPI0 total byte count, only low 12 bit
#define R16_SPI0_TOTAL_CNT  (*((vu16*)0x4000400C)) // RW, SPI0 total byte count, only low 12 bit
#define R32_SPI0_FIFO       (*((vu32*)0x40004010)) // RW, SPI0 FIFO register
#define R8_SPI0_FIFO        (*((vu8*)0x40004010))  // RO/WO, SPI0 FIFO register
#define R8_SPI0_FIFO_COUNT1 (*((vu8*)0x40004013))  // RO, SPI0 FIFO count status
#define R32_SPI0_DMA_NOW    (*((vu32*)0x40004014)) // RW, SPI0 DMA current address
#define R16_SPI0_DMA_NOW    (*((vu16*)0x40004014)) // RW, SPI0 DMA current address
#define R32_SPI0_DMA_BEG    (*((vu32*)0x40004018)) // RW, SPI0 DMA begin address
#define R16_SPI0_DMA_BEG    (*((vu16*)0x40004018)) // RW, SPI0 DMA begin address
#define R32_SPI0_DMA_END    (*((vu32*)0x4000401C)) // RW, SPI0 DMA end address
#define R16_SPI0_DMA_END    (*((vu16*)0x4000401C)) // RW, SPI0 DMA end address

/* SPI register address offset and bit define */
#define SPI_FIFO_SIZE       8                         // SPI FIFO size (depth)
#define BA_SPI0             ((vu8*)0x40004000)     // point SPI0 base address
#define SPI_CTRL_MOD        0
#define  RB_SPI_MODE_SLAVE  0x01                      // RW, SPI0 slave mode: 0=master/host, 1=slave/device
#define  RB_SPI_ALL_CLEAR   0x02                      // RW, force clear SPI FIFO and count
#define  RB_SPI_2WIRE_MOD   0x04                      // RW, SPI0 enable 2 wire mode for slave: 0=3wire(SCK0,MOSI,MISO), 1=2wire(SCK0,MISO=MXSX)
#define  RB_SPI_MST_SCK_MOD 0x08                      // RW, SPI master clock mode: 0=mode 0, 1=mode 3
#define  RB_SPI_SLV_CMD_MOD 0x08                      // RW, SPI0 slave command mode: 0=byte stream, 1=first byte command
#define  RB_SPI_FIFO_DIR    0x10                      // RW, SPI FIFO direction: 0=out(write @master mode), 1=in(read @master mode)
#define  RB_SPI_SCK_OE      0x20                      // RW, SPI SCK output enable
#define  RB_SPI_MOSI_OE     0x40                      // RW, SPI MOSI output enable
#define  RB_SPI_MISO_OE     0x80                      // RW, SPI MISO output enable
#define SPI_CTRL_CFG        1
#define  RB_SPI_DMA_ENABLE  0x01                      // RW, SPI0 DMA enable
#define  RB_SPI_DMA_LOOP    0x04                      // RW, SPI0 DMA address loop enable
#define  RB_SPI_AUTO_IF     0x10                      // RW, enable buffer/FIFO accessing to auto clear RB_SPI_IF_BYTE_END interrupt flag
#define  RB_SPI_BIT_ORDER   0x20                      // RW, SPI bit data order: 0=MSB first, 1=LSB first
#define  RB_SPI_MST_DLY_EN  0x40                      // RW, SPI master input delay enable
#define SPI_INTER_EN        2
#define  RB_SPI_IE_CNT_END  0x01                      // RW, enable interrupt for SPI total byte count end
#define  RB_SPI_IE_BYTE_END 0x02                      // RW, enable interrupt for SPI byte exchanged
#define  RB_SPI_IE_FIFO_HF  0x04                      // RW, enable interrupt for SPI FIFO half
#define  RB_SPI_IE_DMA_END  0x08                      // RW, enable interrupt for SPI0 DMA completion
#define  RB_SPI_IE_FIFO_OV  0x10                      // RW, enable interrupt for SPI0 FIFO overflow
#define  RB_SPI_IE_FST_BYTE 0x80                      // RW, enable interrupt for SPI0 slave mode first byte received
#define SPI_CLOCK_DIV       3
#define SPI_SLAVE_PRESET    3
#define SPI_BUFFER          4
#define SPI_RUN_FLAG        5
#define  RB_SPI_SLV_CMD_ACT 0x10                      // RO, SPI0 slave first byte / command flag
#define  RB_SPI_FIFO_READY  0x20                      // RO, SPI FIFO ready status
#define  RB_SPI_SLV_CS_LOAD 0x40                      // RO, SPI0 slave chip-select loading status
#define  RB_SPI_SLV_SELECT  0x80                      // RO, SPI0 slave selection status
#define SPI_INT_FLAG        6
#define  RB_SPI_IF_CNT_END  0x01                      // RW1, interrupt flag for SPI total byte count end
#define  RB_SPI_IF_BYTE_END 0x02                      // RW1, interrupt flag for SPI byte exchanged
#define  RB_SPI_IF_FIFO_HF  0x04                      // RW1, interrupt flag for SPI FIFO half (RB_SPI_FIFO_DIR ? >=4bytes : <4bytes)
#define  RB_SPI_IF_DMA_END  0x08                      // RW1, interrupt flag for SPI0 DMA completion
#define  RB_SPI_IF_FIFO_OV  0x10                      // RW1, interrupt flag for SPI0 FIFO overflow
#define  RB_SPI_FREE        0x40                      // RO, current SPI free status
#define  RB_SPI_IF_FST_BYTE 0x80                      // RW1, interrupt flag for SPI0 slave mode first byte received
#define SPI_FIFO_COUNT      7
#define SPI_TOTAL_CNT       0x0C
#define SPI_FIFO            0x10
#define SPI_DMA_NOW         0x14
#define SPI_DMA_BEG         0x18
#define SPI_DMA_END         0x1C

/* I2C register */
#define R16_I2C_CTRL1       (*((vu16*)0x40004800)) // RW, I2C control 1
#define R16_I2C_CTRL2       (*((vu16*)0x40004804)) // RW, I2C control 2
#define R16_I2C_OADDR1      (*((vu16*)0x40004808)) // RW, I2C own address register 1
#define R16_I2C_OADDR2      (*((vu16*)0x4000480C)) // RW, I2C own address register 2
#define R16_I2C_DATAR       (*((vu16*)0x40004810)) // RW, I2C data register
#define R16_I2C_STAR1       (*((vu16*)0x40004814)) // R0, I2C stauts register 1
#define R16_I2C_STAR2       (*((vu16*)0x40004818)) // R0, I2C status register 2
// #define R8_I2C_PEC          (*((vu8*) 0x40004819)) // R0, I2C Packet error checking register 
#define R16_I2C_CKCFGR      (*((vu16*)0x4000481C)) // RW, I2C clock control register
#define R16_I2C_RTR         (*((vu16*)0x40004820)) // RW, I2C trise register

/* I2C register address offset and bit define */
#define BA_I2C              ((vu8*)0x40004800)     // point I2C base address
#define I2C_CTRL1           0
#define  RB_I2C_PE          0x0001                    // RW, Peripheral enable
#define  RB_I2C_SMBUS       0x0002                    // RW, SMBUS mode: 0=I2C mode, 1=SMBUS mode
#define  RB_I2C_SMBTYPE     0x0008                    // RW, SMBus type: 0=Device, 1=Host
#define  RB_I2C_EBARP       0x0010                    // RW, ARP enable
#define  RB_I2C_ENPEC       0x0020                    // RW, PEC ebable
#define  RB_I2C_ENGC        0x0040                    // RW, General call enable
#define  RB_I2C_NOSTRETCH   0x0080                    // RW, Clock stretching disable (Slave mode)
#define  RB_I2C_START       0x0100                    // RW, Start generation: master mode: 0=no start, 1=repeated start; slave mode: 0=no start, 1=start at bus free
#define  RB_I2C_STOP        0x0200                    // RW, Stop generation: master mode: 0=no stop, 1=stop after the current byte transfer or after the current Start condition is sent; slave mode: 0=no stop, 1=Release the SCL and SDA lines after the current byte transfer
#define  RB_I2C_ACK         0x0400                    // RW, Acknowledge enable
#define  RB_I2C_POS         0x0800                    // RW, Acknowledge/PEC Position (for data reception)
#define  RB_I2C_PEC         0x1000                    // RW, Packet error checking: 0=No PEC transfer, 1=PEC transfer (in Tx or Rx mode)
#define  RB_I2C_ALERT       0x2000                    // RW, SMBus alert: 0=Releases SMBA pin high, 1=Drives SMBA pin low.
#define  RB_I2C_SWRST       0x8000                    // RW, Software reset
#define I2C_CTRL2           4
#define  RB_I2C_FREQ        0x003F                    // RW, Peripheral clock frequency, The minimum allowed frequency is 2 MHz,the maximum frequency is 36 MHz
#define  RB_I2C_ITERREN     0x0100                    // RW, Error interrupt enable
#define  RB_I2C_ITEVTEN     0x0200                    // RW, Event interrupt enable
#define  RB_I2C_ITBUFEN     0x0400                    // RW, Buffer interrupt enable
#define I2C_OADDR1          8
#define  RB_I2C_ADD0        0x0001                    // RW, bit0 of address in 10-bit addressing mode
#define  RB_I2C_ADD7_1      0x00FE                    // RW, bit[7:1] of address
#define  RB_I2C_ADD9_8      0x0300                    // RW, bit[9:8] of address in 10-bit addressing mode
#define  RB_I2C_ADDMODE     0x8000                    // RW, Addressing mode (slave mode): 0=7-bit slave address, 1=10-bit slave address
#define I2C_OADDR2          12
#define  RB_I2C_ENDUAL      0x0001                    // RW, Dual addressing mode enable
#define  RB_I2C_ADD2        0x00FE                    // RW, bit[7:1] of address2
#define I2C_DATAR           16              
#define I2C_STAR1           20
#define  RB_I2C_SB          0x0001                    // RW0, Start bit flag (Master mode)
#define  RB_I2C_ADDR        0x0002                    // RW0, Address sent (master mode)/matched (slave mode) flag
#define  RB_I2C_BTF         0x0004                    // RO, Byte transfer finished flag
#define  RB_I2C_ADD10       0x0008                    // RO, 10-bit header sent flag (Master mode)
#define  RB_I2C_STOPF       0x0010                    // RO, Stop detection flag (slave mode)
#define  RB_I2C_RxNE        0x0040                    // RO, Data register not empty flag (receivers)
#define  RB_I2C_TxE         0x0080                    // RO, Data register empty flag (transmitters)
#define  RB_I2C_BERR        0x0100                    // RW0, Bus error flag
#define  RB_I2C_ARLO        0x0200                    // RW0, Arbitration lost flag (master mode)
#define  RB_I2C_AF          0x0400                    // RW0, Acknowledge failure flag
#define  RB_I2C_OVR         0x0800                    // RW0, Overrun/Underrun flag
#define  RB_I2C_PECERR      0x1000                    // RW0, PEC Error flag in reception
#define  RB_I2C_TIMEOUT     0x4000                    // RW0, Timeout or Tlow error flag
#define  RB_I2C_SMBALERT    0x8000                    // RW0, SMBus alert flag
#define I2C_STAR2           24
#define  RB_I2C_MSL         0x0001                    // RO, Mode statu: 0=Slave mode, 1=Master mode
#define  RB_I2C_BUSY        0x0002                    // RO, Bus busy flag
#define  RB_I2C_TRA         0x0004                    // RO, Trans flag: 0=data bytes received, 1=data bytes transmitted
#define  RB_I2C_GENCALL     0x0010                    // RO, General call address (Slave mode) received flag
#define  RB_I2C_SMBDEFAULT  0x0020                    // RO, SMBus device default address (Slave mode) received flag
#define  RB_I2C_SMBHOST     0x0040                    // RO, SMBus host header (Slave mode) received flag
#define  RB_I2C_DUALF       0x0080                    // RO, Dual flag (Slave mode): 0=Received address matched with OAR1, 1=Received address matched with OAR2
#define  RB_I2C_PECX        0xFF00                    // RO, Packet error checking register
#define I2C_CKCFGR          28
#define  RB_I2C_CCR         0x0FFF                    // RW, Controls the SCL clock in Fm/Sm mode (Master mode)
#define  RB_I2C_DUTY        0x4000                    // RW, Fm mode duty cycle: 0=L/H=2, 1=L/H=16/9
#define  RB_I2C_F_S         0x8000                    // RW, I2C master mode selection: 0=standard mode, 1=fast mode
#define I2C_RTR             32
#define  RB_I2C_TRISE       0x003F                    // RW, Maximum rise time in Fm/Sm mode (Master mode)

/* PWM4/5/6/7/8/9/10/11 register */
#define R32_PWM_CONTROL     (*((vu32*)0x40005000)) // RW, PWM control
#define R8_PWM_OUT_EN       (*((vu8*)0x40005000))  // RW, PWM output enable control
#define R8_PWM_POLAR        (*((vu8*)0x40005001))  // RW, PWM output polarity control
#define R8_PWM_CONFIG       (*((vu8*)0x40005002))  // RW, PWM configuration
#define R8_PWM_CLOCK_DIV    (*((vu8*)0x40005003))  // RW, PWM clock divisor
#define R32_PWM4_7_DATA     (*((vu32*)0x40005004)) // RW, PWM4-7 data holding
#define R8_PWM4_DATA        (*((vu8*)0x40005004))  // RW, PWM4 data holding
#define R8_PWM5_DATA        (*((vu8*)0x40005005))  // RW, PWM5 data holding
#define R8_PWM6_DATA        (*((vu8*)0x40005006))  // RW, PWM6 data holding
#define R8_PWM7_DATA        (*((vu8*)0x40005007))  // RW, PWM7 data holding
#define R32_PWM8_11_DATA    (*((vu32*)0x40005008)) // RW, PWM8-11 data holding
#define R8_PWM8_DATA        (*((vu8*)0x40005008))  // RW, PWM8 data holding
#define R8_PWM9_DATA        (*((vu8*)0x40005009))  // RW, PWM9 data holding
#define R8_PWM10_DATA       (*((vu8*)0x4000500A))  // RW, PWM10 data holding
#define R8_PWM11_DATA       (*((vu8*)0x4000500B))  // RW, PWM11 data holding
#define R8_PWM_INT_CTRL     (*((vu32*)0x4000500C)) // RW, PWM interrupt control
#define  RB_PWM_IE_CYC      0x01                      // RW, enable interrupt for PWM cycle end
#define  RB_PWM_CYC_PRE     0x02                      // RW, select PWM cycle interrupt point: 0=after count 0xFE (0x7E for 7 bits mode...), 1=after count 0xF0 (0x70 for 7 bits mode...)
#define  RB_PWM_IF_CYC      0x80                      // RW1, interrupt flag for PWM cycle end
#define R32_PWM_REG_DATA8   (*((vu32*)0x40005010)) // RW, PWM8-9 data register
#define R32_PWM_REG_CYCLE   (*((vu32*)0x40005014)) // RW, PWM cycle value

/* PWM4/5/6/7/8/9/10/11 register address offset and bit define */
#define BA_PWMX             ((vu8*)0x40005000)     // point PWM4/5/6/7/8/9/10/11 base address
#define PWM_OUT_EN          0
#define  RB_PWM4_OUT_EN     0x01                      // RW, PWM4 output enable
#define  RB_PWM5_OUT_EN     0x02                      // RW, PWM5 output enable
#define  RB_PWM6_OUT_EN     0x04                      // RW, PWM6 output enable
#define  RB_PWM7_OUT_EN     0x08                      // RW, PWM7 output enable
#define  RB_PWM8_OUT_EN     0x10                      // RW, PWM8 output enable
#define  RB_PWM9_OUT_EN     0x20                      // RW, PWM9 output enable
#define  RB_PWM10_OUT_EN    0x40                      // RW, PWM10 output enable
#define  RB_PWM11_OUT_EN    0x80                      // RW, PWM11 output enable
#define PWM_POLAR           1
#define  RB_PWM4_POLAR      0x01                      // RW, PWM4 output polarity: 0=default low and high action, 1=default high and low action
#define  RB_PWM5_POLAR      0x02                      // RW, PWM5 output polarity: 0=default low and high action, 1=default high and low action
#define  RB_PWM6_POLAR      0x04                      // RW, PWM6 output polarity: 0=default low and high action, 1=default high and low action
#define  RB_PWM7_POLAR      0x08                      // RW, PWM7 output polarity: 0=default low and high action, 1=default high and low action
#define  RB_PWM8_POLAR      0x10                      // RW, PWM8 output polarity: 0=default low and high action, 1=default high and low action
#define  RB_PWM9_POLAR      0x20                      // RW, PWM9 output polarity: 0=default low and high action, 1=default high and low action
#define  RB_PWM10_POLAR     0x40                      // RW, PWM10 output polarity: 0=default low and high action, 1=default high and low action
#define  RB_PWM11_POLAR     0x80                      // RW, PWM11 output polarity: 0=default low and high action, 1=default high and low action
#define PWM_CONFIG          2
#define  RB_PWM_CYCLE_SEL   0x01                      // RW, PWM cycle selection: 0=256/128/64/32 clocks, 1=255/127/63/31 clocks
#define  RB_PWM_STAG_ST     0x02                      // RO, PWM stagger cycle status
#define  RB_PWM_CYC_MOD     0x0C                      // RW, PWM data width mode: 00=8 bits data, 01=7 bits data, 10=6 bits data, 11=16 bits data
#define  RB_PWM4_5_STAG_EN  0x10                      // RW, PWM4/5 stagger output enable: 0=independent output, 1=stagger output
#define  RB_PWM6_7_STAG_EN  0x20                      // RW, PWM6/7 stagger output enable: 0=independent output, 1=stagger output
#define  RB_PWM8_9_STAG_EN  0x40                      // RW, PWM8/9 stagger output enable: 0=independent output, 1=stagger output
#define  RB_PWM10_11_STAG_EN 0x80                     // RW, PWM10/11 stagger output enable: 0=independent output, 1=stagger output
#define PWM_CLOCK_DIV       3
#define PWM4_DATA_HOLD      4
#define PWM5_DATA_HOLD      5
#define PWM6_DATA_HOLD      6
#define PWM7_DATA_HOLD      7
#define PWM8_DATA_HOLD      8
#define PWM9_DATA_HOLD      9
#define PWM10_DATA_HOLD     10
#define PWM11_DATA_HOLD     11

/* LCD register */
#define R32_LCD_CMD         (*((vu32*)(0x40006000)))
#define  RB_LCD_SYS_EN      0x01                      // RW, LCD digital system enable
#define  RB_LCD_ON          0x02                      // RW, LCD analog system enable
#define  RB_LCD_BIAS        0x04                      // RW, LCD bias select:  0=1/2 bias,  1=1/3 bias
#define  RB_LCD_DUTY        0x18                      // RW, LCD duty select:  00=1/2 duty,  01=1/3 duty,  10=1/4 duty
#define  RB_LCD_SCAN_CLK    0x60                      // RW, LCD scan clock select: 00=256Hz, 01=512Hz, 10=1KHz, 11=128Hz
#define  RB_LCD_VLCD_SEL    0x80                      // RW, LCD drive voltage：0=VIO33*100%(3.3V),1=VIO33*76%(2.5V)
#define  RB_LCD_SEG0_7_EN   0xFF00                    // RW, SEG0-SEG7 enable
#define  RB_LCD_SEG8_15_EN  0xFF0000                  // RW, SEG8-SEG15 enable
#define  RB_LCD_SEG16_19_EN 0xF000000                 // RW, SEG16-SEG19 enable

#define R32_LCD_RAM0        (*((vu32*)(0x40006004))) // RW, LCD driver data0, address 0-3
#define R32_LCD_RAM1        (*((vu32*)(0x40006008))) // RW, LCD driver data1, address 4-7
#define R32_LCD_RAM2        (*((vu32*)(0x4000600C))) // RW, LCD driver data2, address 8-12
#define USB_BASE_ADDR              (0x40008000)
#define BA_USB              ((vu8*)0x40008000)     // point USB base address

/*  USB  */
#define R32_USB_CONTROL     (*((vu32*)0x40008000)) // USB control & interrupt enable & device address
#define R8_USB_CTRL         (*((vu8*)0x40008000))  // USB base control
#define  RB_UC_HOST_MODE    0x80      // enable USB host mode: 0=device mode, 1=host mode
#define  RB_UC_LOW_SPEED    0x40      // enable USB low speed: 0=12Mbps, 1=1.5Mbps
#define  RB_UC_DEV_PU_EN    0x20      // USB device enable and internal pullup resistance enable
#define  RB_UC_SYS_CTRL1    0x20      // USB system control high bit
#define  RB_UC_SYS_CTRL0    0x10      // USB system control low bit
#define  MASK_UC_SYS_CTRL   0x30      // bit mask of USB system control
// bUC_HOST_MODE & bUC_SYS_CTRL1 & bUC_SYS_CTRL0: USB system control
//   0 00: disable USB device and disable internal pullup resistance
//   0 01: enable USB device and disable internal pullup resistance, need RB_PIN_USB_DP_PU=1 or need external pullup resistance
//   0 1x: enable USB device and enable internal pullup resistance
//   1 00: enable USB host and normal status
//   1 01: enable USB host and force UDP/UDM output SE0 state
//   1 10: enable USB host and force UDP/UDM output J state
//   1 11: enable USB host and force UDP/UDM output resume or K state
#define  RB_UC_INT_BUSY     0x08      // enable automatic responding busy for device mode or automatic pause for host mode during interrupt flag UIF_TRANSFER valid
#define  RB_UC_RESET_SIE    0x04      // force reset USB SIE, need software clear
#define  RB_UC_CLR_ALL      0x02      // force clear FIFO and count of USB
#define  RB_UC_DMA_EN       0x01      // DMA enable and DMA interrupt enable for USB

#define R8_UDEV_CTRL        (*((vu8*)0x40008001))  // USB device physical prot control
#define  RB_UD_PD_DIS       0x80      // disable USB UDP/UDM pulldown resistance: 0=enable pulldown, 1=disable
#define  RB_UD_DP_PIN       0x20      // ReadOnly: indicate current UDP pin level
#define  RB_UD_DM_PIN       0x10      // ReadOnly: indicate current UDM pin level
#define  RB_UD_LOW_SPEED    0x04      // enable USB physical port low speed: 0=full speed, 1=low speed
#define  RB_UD_GP_BIT       0x02      // general purpose bit
#define  RB_UD_PORT_EN      0x01      // enable USB physical port I/O: 0=disable, 1=enable

#define R8_UHOST_CTRL       R8_UDEV_CTRL              // USB host physical prot control
#define  RB_UH_PD_DIS       0x80      // disable USB UDP/UDM pulldown resistance: 0=enable pulldown, 1=disable
#define  RB_UH_DP_PIN       0x20      // ReadOnly: indicate current UDP pin level
#define  RB_UH_DM_PIN       0x10      // ReadOnly: indicate current UDM pin level
#define  RB_UH_LOW_SPEED    0x04      // enable USB port low speed: 0=full speed, 1=low speed
#define  RB_UH_BUS_RESET    0x02      // control USB bus reset: 0=normal, 1=force bus reset
#define  RB_UH_PORT_EN      0x01      // enable USB port: 0=disable, 1=enable port, automatic disabled if USB device detached

#define R8_USB_INT_EN       (*((vu8*)0x40008002))  // USB interrupt enable
#define  RB_UIE_DEV_SOF     0x80      // enable interrupt for SOF received for USB device mode
#define  RB_UIE_DEV_NAK     0x40      // enable interrupt for NAK responded for USB device mode
#define  RB_MOD_1_WIRE      0x20      // enable single wire mode
#define  RB_UIE_FIFO_OV     0x10      // enable interrupt for FIFO overflow
#define  RB_UIE_HST_SOF     0x08      // enable interrupt for host SOF timer action for USB host mode
#define  RB_UIE_SUSPEND     0x04      // enable interrupt for USB suspend or resume event
#define  RB_UIE_TRANSFER    0x02      // enable interrupt for USB transfer completion
#define  RB_UIE_DETECT      0x01      // enable interrupt for USB device detected event for USB host mode
#define  RB_UIE_BUS_RST     0x01      // enable interrupt for USB bus reset event for USB device mode

#define R8_USB_DEV_AD       (*((vu8*)0x40008003))  // USB device address
#define  RB_UDA_GP_BIT      0x80      // general purpose bit
#define  MASK_USB_ADDR      0x7F      // bit mask for USB device address

#define R32_USB_STATUS      (*((vu32*)0x40008004)) // USB miscellaneous status & interrupt flag & interrupt status
#define R8_USB_MIS_ST       (*((vu8*)0x40008005))  // USB miscellaneous status
#define  RB_UMS_SOF_PRES    0x80      // RO, indicate host SOF timer presage status
#define  RB_UMS_SOF_ACT     0x40      // RO, indicate host SOF timer action status for USB host
#define  RB_UMS_SIE_FREE    0x20      // RO, indicate USB SIE free status
#define  RB_UMS_R_FIFO_RDY  0x10      // RO, indicate USB receiving FIFO ready status (not empty)
#define  RB_UMS_BUS_RESET   0x08      // RO, indicate USB bus reset status
#define  RB_UMS_SUSPEND     0x04      // RO, indicate USB suspend status
#define  RB_UMS_DM_LEVEL    0x02      // RO, indicate UDM level saved at device attached to USB host
#define  RB_UMS_DEV_ATTACH  0x01      // RO, indicate device attached status on USB host

#define R8_USB_INT_FG       (*((vu8*)0x40008006))  // USB interrupt flag
#define  RB_U_IS_NAK        0x80    // RO, indicate current USB transfer is NAK received
#define  RB_U_TOG_OK        0x40    // RO, indicate current USB transfer toggle is OK
#define  RB_U_SIE_FREE      0x20    // RO, indicate USB SIE free status
#define  RB_UIF_FIFO_OV     0x10    // FIFO overflow interrupt flag for USB, direct bit address clear or write 1 to clear
#define  RB_UIF_HST_SOF     0x08    // host SOF timer interrupt flag for USB host, direct bit address clear or write 1 to clear
#define  RB_UIF_SUSPEND     0x04    // USB suspend or resume event interrupt flag, direct bit address clear or write 1 to clear
#define  RB_UIF_TRANSFER    0x02    // USB transfer completion interrupt flag, direct bit address clear or write 1 to clear
#define  RB_UIF_DETECT      0x01    // device detected event interrupt flag for USB host mode, direct bit address clear or write 1 to clear
#define  RB_UIF_BUS_RST     0x01    // bus reset event interrupt flag for USB device mode, direct bit address clear or write 1 to clear

#define R8_USB_INT_ST       (*((vu8*)0x40008007))  // USB interrupt status
#define  RB_UIS_SETUP_ACT   0x80      // RO, indicate SETUP token & 8 bytes setup request received for USB device mode
#define  RB_UIS_TOG_OK      0x40      // RO, indicate current USB transfer toggle is OK
#define  RB_UIS_TOKEN1      0x20      // RO, current token PID code bit 1 received for USB device mode
#define  RB_UIS_TOKEN0      0x10      // RO, current token PID code bit 0 received for USB device mode
#define  MASK_UIS_TOKEN     0x30      // RO, bit mask of current token PID code received for USB device mode
#define  UIS_TOKEN_OUT      0x00
#define  UIS_TOKEN_SOF      0x10
#define  UIS_TOKEN_IN       0x20
#define  UIS_TOKEN_SETUP    0x30
// bUIS_TOKEN1 & bUIS_TOKEN0: current token PID code received for USB device mode, keep last status during SETUP token, clear RB_UIF_TRANSFER ( RB_UIF_TRANSFER from 1 to 0 ) to set free
//   00: OUT token PID received
//   01: SOF token PID received
//   10: IN token PID received
//   11: free
#define  MASK_UIS_ENDP      0x0F      // RO, bit mask of current transfer endpoint number for USB device mode
#define  MASK_UIS_H_RES     0x0F      // RO, bit mask of current transfer handshake response for USB host mode: 0000=no response, time out from device, others=handshake response PID received

#define R8_USB_RX_LEN       (*((vu8*)0x40008008))  // USB receiving length
#define R32_USB_BUF_MODE    (*((vu32*)0x4000800C)) // USB endpoint buffer mode
#define R8_UEP4_1_MOD       (*((vu8*)0x4000800C))  // endpoint 4/1 mode
#define  RB_UEP1_RX_EN      0x80      // enable USB endpoint 1 receiving (OUT)
#define  RB_UEP1_TX_EN      0x40      // enable USB endpoint 1 transmittal (IN)
#define  RB_UEP1_BUF_MOD    0x10      // buffer mode of USB endpoint 1
// bUEPn_RX_EN & bUEPn_TX_EN & bUEPn_BUF_MOD: USB endpoint 1/2/3 buffer mode, buffer start address is UEPn_DMA
//   0 0 x:  disable endpoint and disable buffer
//   1 0 0:  64 bytes buffer for receiving (OUT endpoint)
//   1 0 1:  dual 64 bytes buffer by toggle bit bUEP_R_TOG selection for receiving (OUT endpoint), total=128bytes
//   0 1 0:  64 bytes buffer for transmittal (IN endpoint)
//   0 1 1:  dual 64 bytes buffer by toggle bit bUEP_T_TOG selection for transmittal (IN endpoint), total=128bytes
//   1 1 0:  64 bytes buffer for receiving (OUT endpoint) + 64 bytes buffer for transmittal (IN endpoint), total=128bytes
//   1 1 1:  dual 64 bytes buffer by bUEP_R_TOG selection for receiving (OUT endpoint) + dual 64 bytes buffer by bUEP_T_TOG selection for transmittal (IN endpoint), total=256bytes
#define  RB_UEP4_RX_EN      0x08      // enable USB endpoint 4 receiving (OUT)
#define  RB_UEP4_TX_EN      0x04      // enable USB endpoint 4 transmittal (IN)
// bUEP4_RX_EN & bUEP4_TX_EN: USB endpoint 4 buffer mode, buffer start address is UEP0_DMA
//   0 0:  single 64 bytes buffer for endpoint 0 receiving & transmittal (OUT & IN endpoint)
//   1 0:  single 64 bytes buffer for endpoint 0 receiving & transmittal (OUT & IN endpoint) + 64 bytes buffer for endpoint 4 receiving (OUT endpoint), total=128bytes
//   0 1:  single 64 bytes buffer for endpoint 0 receiving & transmittal (OUT & IN endpoint) + 64 bytes buffer for endpoint 4 transmittal (IN endpoint), total=128bytes
//   1 1:  single 64 bytes buffer for endpoint 0 receiving & transmittal (OUT & IN endpoint)
//           + 64 bytes buffer for endpoint 4 receiving (OUT endpoint) + 64 bytes buffer for endpoint 4 transmittal (IN endpoint), total=192bytes

#define R8_UEP2_3_MOD       (*((vu8*)0x4000800D))   // endpoint 2/3 mode
#define  RB_UEP3_RX_EN      0x80      // enable USB endpoint 3 receiving (OUT)
#define  RB_UEP3_TX_EN      0x40      // enable USB endpoint 3 transmittal (IN)
#define  RB_UEP3_BUF_MOD    0x10      // buffer mode of USB endpoint 3
#define  RB_UEP2_RX_EN      0x08      // enable USB endpoint 2 receiving (OUT)
#define  RB_UEP2_TX_EN      0x04      // enable USB endpoint 2 transmittal (IN)
#define  RB_UEP2_BUF_MOD    0x01      // buffer mode of USB endpoint 2

#define R8_UEP567_MOD       (*((vu8*)0x4000800E))   // endpoint 5/6/7 mode
#define  RB_UEP7_RX_EN      0x20      // enable USB endpoint 7 receiving (OUT)
#define  RB_UEP7_TX_EN      0x10      // enable USB endpoint 7 transmittal (IN)
#define  RB_UEP6_RX_EN      0x08      // enable USB endpoint 6 receiving (OUT)
#define  RB_UEP6_TX_EN      0x04      // enable USB endpoint 6 transmittal (IN)
#define  RB_UEP5_RX_EN      0x02      // enable USB endpoint 5 receiving (OUT)
#define  RB_UEP5_TX_EN      0x01      // enable USB endpoint 5 transmittal (IN)
// bUEPn_RX_EN & bUEPn_TX_EN: USB endpoint 5/6/7 buffer mode, buffer start address is UEPn_DMA
//   0 0:  disable endpoint and disable buffer
//   1 0:  64 bytes buffer for receiving (OUT endpoint)
//   0 1:  64 bytes buffer for transmittal (IN endpoint)
//   1 1:  64 bytes buffer for receiving (OUT endpoint) + 64 bytes buffer for transmittal (IN endpoint), total=128bytes

#define R8_UH_EP_MOD        R8_UEP2_3_MOD             //host endpoint mode
#define  RB_UH_EP_TX_EN     0x40      // enable USB host OUT endpoint transmittal
#define  RB_UH_EP_TBUF_MOD  0x10      // buffer mode of USB host OUT endpoint
// bUH_EP_TX_EN & bUH_EP_TBUF_MOD: USB host OUT endpoint buffer mode, buffer start address is UH_TX_DMA
//   0 x:  disable endpoint and disable buffer
//   1 0:  64 bytes buffer for transmittal (OUT endpoint)
//   1 1:  dual 64 bytes buffer by toggle bit bUH_T_TOG selection for transmittal (OUT endpoint), total=128bytes
#define  RB_UH_EP_RX_EN     0x08      // enable USB host IN endpoint receiving
#define  RB_UH_EP_RBUF_MOD  0x01      // buffer mode of USB host IN endpoint
// bUH_EP_RX_EN & bUH_EP_RBUF_MOD: USB host IN endpoint buffer mode, buffer start address is UH_RX_DMA
//   0 x:  disable endpoint and disable buffer
//   1 0:  64 bytes buffer for receiving (IN endpoint)
//   1 1:  dual 64 bytes buffer by toggle bit bUH_R_TOG selection for receiving (IN endpoint), total=128bytes

#define R16_UEP0_DMA        (*((vu16*)0x40008010)) // endpoint 0 DMA buffer address
#define R16_UEP1_DMA        (*((vu16*)0x40008014)) // endpoint 1 DMA buffer address
#define R16_UEP2_DMA        (*((vu16*)0x40008018)) // endpoint 2 DMA buffer address
#define R16_UH_RX_DMA       R16_UEP2_DMA              // host rx endpoint buffer address
#define R16_UEP3_DMA        (*((vu16*)0x4000801C)) // endpoint 3 DMA buffer address
#define R16_UH_TX_DMA       R16_UEP3_DMA              // host tx endpoint buffer address
#define R16_UEP5_DMA        (*((vu16*)0x40008054)) // endpoint 5 DMA buffer address
#define R16_UEP6_DMA        (*((vu16*)0x40008058)) // endpoint 6 DMA buffer address
#define R16_UEP7_DMA        (*((vu16*)0x4000805C)) // endpoint 7 DMA buffer address
#define R32_USB_EP0_CTRL    (*((vu32*)0x40008020)) // endpoint 0 control & transmittal length
#define R8_UEP0_T_LEN       (*((vu8*)0x40008020))  // endpoint 0 transmittal length
#define R8_UEP0_CTRL        (*((vu8*)0x40008022))  // endpoint 0 control
#define R32_USB_EP1_CTRL    (*((vu32*)0x40008024)) // endpoint 1 control & transmittal length
#define R8_UEP1_T_LEN       (*((vu8*)0x40008024))  // endpoint 1 transmittal length
#define R8_UEP1_CTRL        (*((vu8*)0x40008026))  // endpoint 1 control
#define  RB_UEP_R_TOG       0x80      // expected data toggle flag of USB endpoint X receiving (OUT): 0=DATA0, 1=DATA1
#define  RB_UEP_T_TOG       0x40      // prepared data toggle flag of USB endpoint X transmittal (IN): 0=DATA0, 1=DATA1
#define  RB_UEP_AUTO_TOG    0x10      // enable automatic toggle after successful transfer completion on endpoint 1/2/3: 0=manual toggle, 1=automatic toggle
#define  RB_UEP_R_RES1      0x08      // handshake response type high bit for USB endpoint X receiving (OUT)
#define  RB_UEP_R_RES0      0x04      // handshake response type low bit for USB endpoint X receiving (OUT)
#define  MASK_UEP_R_RES     0x0C      // bit mask of handshake response type for USB endpoint X receiving (OUT)
#define  UEP_R_RES_ACK      0x00
#define  UEP_R_RES_TOUT     0x04
#define  UEP_R_RES_NAK      0x08
#define  UEP_R_RES_STALL    0x0C
// RB_UEP_R_RES1 & RB_UEP_R_RES0: handshake response type for USB endpoint X receiving (OUT)
//   00: ACK (ready)
//   01: no response, time out to host, for non-zero endpoint isochronous transactions
//   10: NAK (busy)
//   11: STALL (error)
#define  RB_UEP_T_RES1      0x02      // handshake response type high bit for USB endpoint X transmittal (IN)
#define  RB_UEP_T_RES0      0x01      // handshake response type low bit for USB endpoint X transmittal (IN)
#define  MASK_UEP_T_RES     0x03      // bit mask of handshake response type for USB endpoint X transmittal (IN)
#define  UEP_T_RES_ACK      0x00
#define  UEP_T_RES_TOUT     0x01
#define  UEP_T_RES_NAK      0x02
#define  UEP_T_RES_STALL    0x03
// bUEP_T_RES1 & bUEP_T_RES0: handshake response type for USB endpoint X transmittal (IN)
//   00: DATA0 or DATA1 then expecting ACK (ready)
//   01: DATA0 or DATA1 then expecting no response, time out from host, for non-zero endpoint isochronous transactions
//   10: NAK (busy)
//   11: STALL (error)

#define R8_UH_SETUP         R8_UEP1_CTRL              // host aux setup
#define  RB_UH_PRE_PID_EN   0x80      // USB host PRE PID enable for low speed device via hub
#define  RB_UH_SOF_EN       0x40      // USB host automatic SOF enable

#define R32_USB_EP2_CTRL    (*((vu32*)0x40008028)) // endpoint 2 control & transmittal length
#define R8_UEP2_T_LEN       (*((vu8*)0x40008028))  // endpoint 2 transmittal length
#define R8_UEP2_CTRL        (*((vu8*)0x4000802A))  // endpoint 2 control

#define R8_UH_EP_PID        R8_UEP2_T_LEN             // host endpoint and PID
#define  MASK_UH_TOKEN      0xF0      // bit mask of token PID for USB host transfer
#define  MASK_UH_ENDP       0x0F      // bit mask of endpoint number for USB host transfer

#define R8_UH_RX_CTRL       R8_UEP2_CTRL              // host receiver endpoint control
#define  RB_UH_R_TOG        0x80      // expected data toggle flag of host receiving (IN): 0=DATA0, 1=DATA1
#define  RB_UH_R_AUTO_TOG   0x10      // enable automatic toggle after successful transfer completion: 0=manual toggle, 1=automatic toggle
#define  RB_UH_R_RES        0x04      // prepared handshake response type for host receiving (IN): 0=ACK (ready), 1=no response, time out to device, for isochronous transactions

#define R32_USB_EP3_CTRL    (*((vu32*)0x4000802c)) // endpoint 3 control & transmittal length
#define R8_UEP3_T_LEN       (*((vu8*)0x4000802c))  // endpoint 3 transmittal length
#define R8_UEP3_CTRL        (*((vu8*)0x4000802e))  // endpoint 3 control
#define R8_UH_TX_LEN        R8_UEP3_T_LEN             // host transmittal endpoint transmittal length

#define R8_UH_TX_CTRL       R8_UEP3_CTRL              // host transmittal endpoint control
#define  RB_UH_T_TOG        0x40      // prepared data toggle flag of host transmittal (SETUP/OUT): 0=DATA0, 1=DATA1
#define  RB_UH_T_AUTO_TOG   0x10      // enable automatic toggle after successful transfer completion: 0=manual toggle, 1=automatic toggle
#define  RB_UH_T_RES        0x01      // expected handshake response type for host transmittal (SETUP/OUT): 0=ACK (ready), 1=no response, time out from device, for isochronous transactions

#define R32_USB_EP4_CTRL    (*((vu32*)0x40008030)) // endpoint 4 control & transmittal length
#define R8_UEP4_T_LEN       (*((vu8*)0x40008030))  // endpoint 4 transmittal length
#define R8_UEP4_CTRL        (*((vu8*)0x40008032))  // endpoint 4 control

#define R32_USB_EP5_CTRL    (*((vu32*)0x40008064)) // endpoint 5 control & transmittal length
#define R8_UEP5_T_LEN       (*((vu8*)0x40008064))  // endpoint 5 transmittal length
#define R8_UEP5_CTRL        (*((vu8*)0x40008066))  // endpoint 5 control

#define R32_USB_EP6_CTRL    (*((vu32*)0x40008068)) // endpoint 6 control & transmittal length
#define R8_UEP6_T_LEN       (*((vu8*)0x40008068))  // endpoint 6 transmittal length
#define R8_UEP6_CTRL        (*((vu8*)0x4000806A))  // endpoint 6 control

#define R32_USB_EP7_CTRL    (*((vu32*)0x4000806C)) // endpoint 7 control & transmittal length
#define R8_UEP7_T_LEN       (*((vu8*)0x4000806C))  // endpoint 7 transmittal length
#define R8_UEP7_CTRL        (*((vu8*)0x4000806E))  // endpoint 7 control

RV_STATIC_INLINE void LSIEnable() {
	SYS_SAFE_ACCESS(
		R8_CK32K_CONFIG &= ~(RB_CLK_OSC32K_XT | RB_CLK_XT32K_PON); // turn off LSE
		R8_CK32K_CONFIG |= RB_CLK_INT32K_PON; // turn on LSI
	);
}

RV_STATIC_INLINE void DCDCEnable()
{
	SYS_SAFE_ACCESS(
		R16_AUX_POWER_ADJ |= RB_DCDC_CHARGE;
		R16_POWER_PLAN |= RB_PWR_DCDC_PRE;
	);

	RTC_WAIT_TICKS(2);

	SYS_SAFE_ACCESS(
		R16_POWER_PLAN |= RB_PWR_DCDC_EN;
	);
}

RV_STATIC_INLINE void SleepInit() {
	SYS_SAFE_ACCESS(
		R8_SLP_WAKE_CTRL |= RB_SLP_RTC_WAKE;
		R8_RTC_MODE_CTRL |= RB_RTC_TRIG_EN;
	);
	// NVIC_EnableIRQ(RTC_IRQn) is not available here yet
	NVIC->IENR[((uint32_t)(RTC_IRQn) >> 5)] = (1 << ((uint32_t)(RTC_IRQn) & 0x1F));
}

RV_STATIC_INLINE void RTCInit() {
	SYS_SAFE_ACCESS(
		R32_RTC_TRIG = 0;
		R8_RTC_MODE_CTRL |= RB_RTC_LOAD_HI;
	);
	while((R32_RTC_TRIG & 0x3FFF) != (R32_RTC_CNT_DAY & 0x3FFF));
	SYS_SAFE_ACCESS(
		R32_RTC_TRIG = 0;
		R8_RTC_MODE_CTRL |= RB_RTC_LOAD_LO;
	);
}

RV_STATIC_INLINE void RTCTrigger(uint32_t cyc) {
	uint32_t t = R32_RTC_CNT_32K + cyc;
	if(t > RTC_MAX_COUNT) {
		t -= RTC_MAX_COUNT;
	}

	SYS_SAFE_ACCESS(
		R32_RTC_TRIG = t;
	);
}

RV_STATIC_INLINE void LowPowerIdle(uint32_t cyc)
{
	RTCTrigger(cyc);

	NVIC->SCTLR &= ~(1 << 2); // sleep
	NVIC->SCTLR &= ~(1 << 3); // wfi
	asm volatile ("wfi\nnop\nnop" );
}

RV_STATIC_INLINE void LowPowerSleep(uint32_t cyc, uint16_t power_plan) {
	RTCTrigger(cyc);

	SYS_SAFE_ACCESS(
		R8_BAT_DET_CTRL = 0;
		R8_XT32K_TUNE = (R16_RTC_CNT_32K > 0x3fff) ? (R8_XT32K_TUNE & 0xfc) | 0x01 : R8_XT32K_TUNE;
		R8_XT32M_TUNE = (R8_XT32M_TUNE & 0xfc) | 0x03;
	);

	NVIC->SCTLR |= (1 << 2); //deep sleep

	SYS_SAFE_ACCESS(
		R8_SLP_POWER_CTRL |= RB_RAM_RET_LV;
		R16_POWER_PLAN = RB_PWR_PLAN_EN | RB_PWR_CORE | power_plan;
		R8_PLL_CONFIG |= (1 << 5);
	);

	NVIC->SCTLR &= ~(1 << 3); // wfi
	asm volatile ("wfi\nnop\nnop" );

	SYS_SAFE_ACCESS(
		R16_POWER_PLAN &= ~RB_XT_PRE_EN;
		R8_PLL_CONFIG &= ~(1 << 5);
		R8_XT32M_TUNE = (R8_XT32M_TUNE & 0xfc) | 0x01;
	);
}

RV_STATIC_INLINE void LowPower(uint32_t time, uint16_t power_plan) {
	uint32_t time_sleep, time_curr;
	
	if (time <= WAKE_UP_RTC_MAX_TIME) {
		time = time + (RTC_MAX_COUNT - WAKE_UP_RTC_MAX_TIME);
	}
	else {
		time = time - WAKE_UP_RTC_MAX_TIME;
	}

	time_curr = R32_RTC_CNT_32K;
	if (time < time_curr) {
		time_sleep = time + (RTC_MAX_COUNT - time_curr);
	}
	else {
		time_sleep = time - time_curr;
	}
	
	if ((SLEEP_RTC_MIN_TIME < time_sleep) && (time_sleep < SLEEP_RTC_MAX_TIME)) {
		LowPowerSleep( time, power_plan );
	}
	else {
		LowPowerIdle( time );
	}
	
	RTCInit(); // after sleeping this is necessary, but resets the RTC to 0 so don't use it to keep wall time
}

RV_STATIC_INLINE void jump_isprom() {
	memcpy((void*)ISPROM_IN_RAM_ADDRESS, (void*)(ISPROM_ADDRESS + ISPROM_START_OFFSET), ISPROM_SIZE); // copy bootloader to ram
	*(int16_t*)(ISPROM_BOOTBUTTON_CHECK_ADDRESS + 0xa) = 0x4505; // li a0,1, patch PB11 (option byte is not read correctly) detection
	memset((void*)ISPROM_BSS_ADDRESS, 0, ISPROM_BSS_SIZE); // clear .bss

	asm( "la gp, " ISPROM_IN_RAM_GLOBALPOINTER "\n"
		 ".option arch, +zicsr\n"
		 "li t0, " ISPROM_IN_RAM_ENTRYPOINT "\n"
		 "csrw mepc, t0\n" // __set_MEPC is not available here
		 "mret\n");
}

#define HardFault_IRQn        EXC_IRQn

/* Standard Peripheral Library old definitions (maintained for legacy purpose) */
#define HSI_Value             HSI_VALUE
#define HSE_Value             HSE_VALUE
#define HSEStartUp_TimeOut    HSE_STARTUP_TIMEOUT

#ifdef __cplusplus
}
#endif

#endif // Header guard
