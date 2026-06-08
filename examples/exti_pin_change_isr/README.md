# Basic pin change example

The basic example just shows how to handle a pin change interrupt.

For other low-latency operations, see the `vector_free_table_irq` demo.

If you decide to descend the icerber that is HPE, please read below.

## Notes on using HPE with this example

Place configuration items here, you can see a full list in ch32fun/ch32fun.h
To reconfigure to a different processor, update TARGET_MCU in the  Makefile

Ugh this is tricky.
This is how we set (INTSYSCR) to enable hardware interrupt nesting
and hardware stack. BUT this means we can only stack intterrupts 2 deep.

This feature is called "HPE"

Note: If you don't do this, you will need to set your functions to be
__attribute__((interrupt)) instead of  __attribute__((naked))  with an
mret.

PLEASE BE CAREFUL WHEN DOING THIS THOUGH.  There are a number of things
you should know with HPE.  The issue is that HPE doesn't preserve s0,
and s1. You should review the following material before using HPE.
  https://github.com/cnlohr/ch32v003fun/issues/90
  https://www.reddit.com/r/RISCV/comments/126262j/notes_on_wch_fast_interrupts/
  https://www.eevblog.com/forum/microcontrollers/bizarre-problem-on-ch32v003-with-systick-isr-corrupting-uart-tx-data


When you execute code here, from RAM, the latency is between 310 and 480ns.
From RAM is using  `__attribute__((section(".srodata")));`

Chart is in Cycles Spent @ 48MHz

|   __attribute__ |  HPE ON  |  HPE OFF |
| ((interrupt))   |   29  |   28  |
| ((section(".srodata"))) and ((interrupt)) | 28 | 23 |
| ((naked)) | 23 | 21 |
| ((section(".srodata"))) and ((naked)) | 15 | 16| 

These were done with an empty (nop) loop in main.

HPE ON  = 0x804 = 3
HPE OFF = 0x804 = 0

Bog-standard interrupt test with variance. I.e.
`__attribute__((interrupt))` with cursed code in main loop.

Variance tests: 27-30 cycles

Which will manifest as interurpt jitter.


