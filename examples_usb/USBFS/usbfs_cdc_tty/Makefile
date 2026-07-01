all : flash

TARGET:=usbfs_cdc_tty
TARGET_MCU:=CH570
# TARGET_MCU:=CH32V307
# TARGET_MCU_PACKAGE:=CH32V30x_D8C

ADDITIONAL_C_FILES:=../../../extralibs/fsusb.c

include ../../../ch32fun/ch32fun.mk

flash : cv_flash
clean : cv_clean

