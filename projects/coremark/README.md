# Coremark build script for ch32fun

This allows you to easily build and run [Coremark](https://www.eembc.org/coremark/index.php) benchmark on any MCU that is supported in ch32fun. Coremark is industry adopted benchmark for roughly comparing core performance of different MCUs and other embedded processors. Same as all other benchmarks it can't be used to absolutely measure the performance of the device, but nevertheless it can be a useful tool when comparing and choosing among many different chip models available on the market. Also running this test with different compilers and/or compiler settings can show what is influencing the performance of your chip and how, and maybe it can help you to optimize your code if needed.

If you want so compare your results with other known runs you can visit this page: https://www.eembc.org/coremark/scores.php

Also there is an official result list for various WCH MCUs: https://special.wch.cn/zh_cn/coremark_scores/#/
There were some doubts in legitimacy of those numbers, but after my own tests they seem pretty close to what I was getting on these chips, albeit the benchmark was compiled with pretty aggressive gcc flags, that you would rarely see in everyday life.

## How to use

- You will need a copy of [Coremark](https://github.com/eembc/coremark) in the ``coremark`` subfolder. Alternatively, you can point to the Coremark in some other location on your PC by editing ``COREMARK`` variable in the ``Makefile``.
- Select your MCU model in the ``Makefile``
- Connect your board to the PC and do ``make`` in this folder.
- Open the terminal in minichlink, or do ``make terminal``, wait until the test is done running and results are printed.

## Quirks and optimizations

As stated in Coremark description for a result to be valid the benchmark needs run for at least 10 seconds. The duration of the test depends on the number of iterations. By default it is set to 4000, but it is too much for slower processors. You may want to decrease this number to 1500 or even 500. Why does it matter? This *port* uses a raw SysTick value to measure the time spent and if you are running SysTick at HCLK and the processor has a 32 bit counter, it will likely overflow and wrap around for longer durations of the test, this will inevitably produce a wrong result. So you would want a number of iterations be such that it takes 10-20 seconds to complete. You can easily change the number of iterations by editing ``CORE_ITERATIONS`` variable in the ``Makefile``.

Another setting you can change (but shouldn't if you want a comparable result) is ``CORE_DATA_SIZE``. By default it is 2000 and it will use this much bytes in the RAM for a data buffer. This is why you can't run this benchmark on chips like ch32v003 or ch32v002 to get a number to compare with other known results.

``CORE_MEM_METHOD`` is a setting that allows you to choose where the data is stored. The options are: Static - 0, Heap - 1 and Stack - 2. Most results online I've seen were using the Stack option.

``OPT_FLAGS`` - gcc flags for optimizing the compilation of Coremark files. Usually they are compiled with ``-O3``. WCH used ``-Ofast  -funroll-all-loops  -finline-limit=600  -ftree-dominator-opts  -fno-if-conversion2  -fselective-scheduling  -fno-code-hoisting`` for their official table. Some of these flags may even decrease the performance on some chips, you'll have to experiment with them. ``-funroll-all-loops`` - doubles the resulting binary and the size will depend on a number of iterations. The default compiler settings used in ch32fun are ``-Os -flto -ffunction-sections -fdata-sections -fmessage-length=0 -msmall-data-limit=8`` you can use them to see how much performance is traded for space optimization.

The ``printf`` function that is used in ch32fun lacks float support, but to see the Coremark results you need it. This is why there is a third-party printf library in this folder.

## Running from RAM

Many WCH's chips have a rather slow flash memory. This impacts the performance drastically in some cases. The most advanced chips like ch32v20x and ch32v30x use a SRAM cache for the *zero-wait flash* to increase their performance, but chips from ch5xx family doesn't have that cache even though they have pretty fast cores. To measure the core performance on such chips you may want to runt the benchmark in the RAM. You can do it by using the ``make runfromram`` command. But be aware that you would need the resulting firmware size + data buffer be lower than the RAM size of your MCU. You can achieve this by changing compiler flags and disabling third-party printf library. If you disable printf lib, the results will print total number of ticks and you will have to calculate the result by yourself, but this is a worthy tradeoff. 

## Some results

|        |Marks   |Flash/RAM|Frequency|Iterations|Compiler version|Build flags|
|:-      |:-:     |:-:      |:-:      |:-:       |:-:             |:-:        |
|CH570   |27      |Flash    |50MHz    |500       |gcc 13.2.0      |-O3        |
|CH570   |187     |RAM      |100MHz   |2500      |gcc 13.2.0      |-Os -flto -ffunction-sections -fdata-sections -fmessage-length=0 -msmall-data-limit=8   |
|CH32V006|57      |Flash    |48MHz    |1500      |gcc 13.2.0      |-O3        |
|CH32V006|40      |Flash    |48MHz    |1500      |gcc 13.2.0      |-Os -flto -ffunction-sections -fdata-sections -fmessage-length=0 -msmall-data-limit=8   |
|CH32V006|62      |Flash    |48MHz    |1500      |gcc 13.2.0      |-O3 -funroll-all-loops -finline-limit=600 -ftree-dominator-opts -fno-if-conversion2 -fselective-scheduling -fno-code-hoisting     |
|CH32X035|29      |Flash    |48MHz    |500       |gcc 13.2.0      |-Os -flto -ffunction-sections -fdata-sections -fmessage-length=0 -msmall-data-limit=8   |
|CH32X035|71      |RAM      |48MHz    |1500      |gcc 13.2.0      |-Os -flto -ffunction-sections -fdata-sections -fmessage-length=0 -msmall-data-limit=8   |
|CH32X035|106     |RAM      |48MHz    |1500      |gcc 13.2.0      |-O3        |
|CH32X035|115     |RAM      |48MHz    |1500      |gcc 13.2.0      |-O3 -finline-limit=600 -ftree-dominator-opts -fno-if-conversion2 -fselective-scheduling -fno-code-hoisting        |
|CH585   |242     |RAM      |78MHz    |4000      |gcc 13.2.0      |-O3 -funroll-all-loops -finline-limit=600 -ftree-dominator-opts -fno-if-conversion2 -fselective-scheduling -fno-code-hoisting     |
|CH32V307|266     |Flash (zero-wait)  |144MHz    |4000      |gcc 13.2.0      |-Os -flto -ffunction-sections -fdata-sections -fmessage-length=0 -msmall-data-limit=8   |
|CH32V307|384     |Flash (zero-wait)  |144MHz    |4000      |gcc 13.2.0      |-O3  |
|CH32V307|459     |Flash (zero-wait)  |144MHz    |5000      |gcc 13.2.0      |-O3 -funroll-all-loops -finline-limit=600 -ftree-dominator-opts -fno-if-conversion2 -fselective-scheduling -fno-code-hoisting|
|CH32V307|463     |Flash (zero-wait)  |144MHz    |5000      |gcc 13.2.0      |-O3 -funroll-all-loops -finline-limit=600 -ftree-dominator-opts -fno-if-conversion2 -fselective-scheduling |
|CH32V307|469     |Flash (zero-wait)  |144MHz    |5000      |gcc 13.2.0      |-Ofast -funroll-all-loops -finline-limit=600 -ftree-dominator-opts -fno-if-conversion2 -fselective-scheduling -fno-code-hoisting |

I've included multiple ch32v307 results to demonstrate how unintuitively some compiler flags can influence the performance. Adding ``-fno-code-hoisting`` produced worse results with ``-O3`` than on all other chips I've tried, but with ``-Ofast`` it make result better.
