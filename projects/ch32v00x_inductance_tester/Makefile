all : flash

TARGET:=ch32v00x_inductance_tester

TARGET_MCU?=CH32V006

EXTRA_CFLAGS+=-Wno-volatile-register-var -ffixed-x4 

include ../../ch32fun/ch32fun.mk

flash : cv_flash
clean : cv_clean


