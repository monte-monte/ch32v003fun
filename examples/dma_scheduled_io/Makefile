all : flash

TARGET:=dma_scheduled_io

EXTRA_CFLAGS+=-Wno-volatile-register-var -ffixed-x4 

TARGET_MCU?=CH32V006
include ../../ch32fun/ch32fun.mk

flash : cv_flash
clean : cv_clean


