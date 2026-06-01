# ch32v006 inductance tester

### Tested:

☑ CH32V006

☑ CH32V003

### Video:

[![The Worlds Cheapest Time Domain Reflectometer](https://img.youtube.com/vi/1vs_WxC2Odo/mqdefault.jpg)](https://youtu.be/1vs_WxC2Odo)

### Introduction

Simply connect a coil of wire (about 20 turns, about 1.5cm wide) between GND and PD2. Then put pieces of metal against it and see how the inductance changes.

This works by turning on PD2, while simultaneously measuring the ADC value of PD2 with slight time variations, measuring with the precision of 1/48Mth of a second.

```
 +----------+
 | CH32V006 |
 |          |
 |      PD2 | ----\
 |          |     c
 |          |     c
 |          |     c
 |      GND | ----/
 +----------+
```

### Methodology

This uses timed IO using DMA and a timer to specifically perform IO operations while the main CPU is asleep.  Unlike the CPU which is very difficult to precisely time things with, the DMA subsystem can do lots of cycle-accurate operations.  This is based off of 3 tables.

 * Table 1, `values` - This is what values the various peripherals will be set to.
 * Table 2, `addresses` - This is what the addresses of the peripherals that the values will be set to.
 * Table 3, `times` or how many cycles to wait between each operation.

The DMA can perform one operation at a time, and there is a minimum time between operations.  That minimum time is about 15 clock cycles.

### Difference

There is some departure in the code between the 006 and 003.

For the 003, it waits on the input to become free. This is OKAY because the loop is very tight, so it fits in cache.

For the 006, it performs a `wfi` where the CPU goes to sleep. This is great because the CPU isn't going to be using the bus at all while asleep.


