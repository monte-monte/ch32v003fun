all : flash

TARGET:=usbfs_keyboard
TARGET_MCU:=CH32X035
# TARGET_MCU:=CH32L103
# TARGET_MCU:=CH32V208
# TARGET_MCU_PACKAGE:=CH32V208WBU6
# TARGET_MCU:=CH32V203
# TARGET_MCU:=CH32V307
# TARGET_MCU:=CH592
# TARGET_MCU:=CH570

ADDITIONAL_C_FILES:=../../../extralibs/fsusb.c

include ../../../ch32fun/ch32fun.mk

install_udev_rules : 
	sudo cp ./99-v203demo.rules /etc/udev/rules.d/
	sudo udevadm control --reload
	sudo udevadm trigger

flash : cv_flash
clean : cv_clean

