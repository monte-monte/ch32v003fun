# Demonstration of using Timer 1 capture and DMA

This example uses Timer 1 capture to look for down or up-going events on PD2/PA1/PC3/PC4. Then using DMA to capture that event and to write it to a circular queue that can be read out later.

## For it to produce interesting output, you can wire PD2/PA1/PC3/PC4 to PD3 which the demo outputs pulses on.

* TIM1CH1 is PD2
* TIM1CH2 is PA1
* TIM1CH3 is PC3
* TIM1CH4 is PC4
