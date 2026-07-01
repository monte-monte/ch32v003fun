all : flash

TARGET:=bootloader
# TARGET_MCU:=CH32X035
TARGET_MCU:=CH570

ADDITIONAL_C_FILES:=../../../extralibs/fsusb.c

ifeq ($(findstring CH32X03,$(TARGET_MCU)),CH32X03)
LINKER_SCRIPT:=./ch32x035-usb-bootloader.ld
BOOT_FLASH_CMD?=$(MINICHLINK)/minichlink -w $< bootloader
else ifeq ($(findstring CH570,$(TARGET_MCU)),CH570)
LINKER_SCRIPT:=./ch570-usb-bootloader.ld
BOOT_FLASH_CMD?=$(MINICHLINK)/minichlink -Bw $< bootloader
# else ifeq ($(findstring CH572,$(TARGET_MCU)),CH572)
else
BOOT_FLASH_CMD?=$(MINICHLINK)/minichlink -Bw $< bootloader
LINKER_SCRIPT:=./ch570-usb-bootloader.ld
endif

include ../../../ch32fun/ch32fun.mk

install_udev_rules : 
	sudo cp ../../minichlink/99-minichlink.rules /etc/udev/rules.d/
	sudo udevadm control --reload
	sudo udevadm trigger

flash_boot : $(TARGET).bin
	$(BOOT_FLASH_CMD)

flash : cv_flash
clean : cv_clean
