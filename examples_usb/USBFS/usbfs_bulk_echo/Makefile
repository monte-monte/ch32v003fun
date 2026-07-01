all : flash

TARGET:=usbfs_bulk_echo
TARGET_MCU:=CH585
# TARGET_MCU:=CH32V307
# TARGET_MCU_PACKAGE:=CH32V30x_D8C

ADDITIONAL_C_FILES:=../../../extralibs/fsusb.c

include ../../../ch32fun/ch32fun.mk

bootloader :
	./$(TARGET).py -b

flash : cv_flash
clean : cv_clean
