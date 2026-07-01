all : flash

TARGET:=usbfs_msc
TARGET_MCU:=CH570
# Target package is only needed for CH32V20x/30x
# TARGET_MCU_PACKAGE:=CH32V30x_D8C

ADDITIONAL_C_FILES:=../../../extralibs/fsusb.c
include ../../../ch32fun/ch32fun.mk

flash : cv_flash
clean : cv_clean

