// Time_per_tick = (4096 × 2^WDGTB) ÷ PCLK1_frequency
// Total_time = Time_per_tick × (T[6:0] - 0x3F)
// Window_start_time = Time_per_tick × (W[6:0] - 0x3F)

#include "ch32fun.h"
#include <stdio.h>

#define WWDG_CNT 0X7F

void WWDG_init(u16 counter, u8 window_value, u8 prescaler) {
	RCC->APB1PCENR |= RCC_APB1Periph_WWDG;

	WWDG->CTLR = counter & WWDG_CTLR_T;

	WWDG->CFGR = (WWDG->CFGR & CFGR_WDGTB_Mask) | prescaler;
	WWDG->CFGR |= (WWDG->CFGR & CFGR_W_Mask) | window_value;

	WWDG->CTLR = CTLR_WDGA_Set | WWDG_CNT;

	// Reset WWDG
	WWDG->STATR = 0;
}

int main() {
    SystemInit();
    funGpioInitAll(); // Enable GPIOs

    printf("\n~ WWDG Test ~\n");

    // set PA3 to input pullup
    funPinMode(PA3, GPIO_CFGLR_IN_PUPD);

	WWDG_init(WWDG_CNT, 0x5f, WWDG_Prescaler_8);

	while(1) {
        u8 read = funDigitalRead( PA3 );
        printf("PA3: %d\n", read);
        if (read) {
            // Reload watchdog
            printf("reload watchdog\n");
            WWDG->CTLR = WWDG_CNT & BIT_Mask;
        }
        Delay_Ms( 1000 );
	}
}
