all : flash

TARGET:=spi_dma_chain
TARGET_MCU:=CH32V203

include ../../ch32fun/ch32fun.mk

flash : cv_flash
clean : cv_clean
macro : $(FILES_TO_COMPILE) $(LINKER_SCRIPT) $(EXTRA_ELF_DEPENDENCIES)
	$(PREFIX)-gcc -E spi_dma_chain.c -o spi_dma_chain.i $(CFLAGS) $(LDFLAGS)