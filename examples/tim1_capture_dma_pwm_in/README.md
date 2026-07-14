# Demonstration of using Timer 1 capture and DMA

This example uses Timer 1 capture to look for down and up-going events on PD2. Then using DMA to capture those events and to write it to a circular queue that can be read out later.

## For it to produce interesting output, you can wire PD2 to PD3 which the demo outputs pulses on.
