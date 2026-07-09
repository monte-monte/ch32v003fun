# USB Bootloader

Open source alternative to factory bootloaders found on WCH chips. Compared to official (factory) ISP (bootloader) this provides numerous missing features:

- Allows all minichlink operations except gdb interface (for now). It can read back any address accessible to the chip.
- Non-destructive writing. It preserves the flash contents when writing, only erasing the parts that are being overwritten. No total erasure like ISP does.
- For CH5xx you don't need to change any config options use this bootloader while flashing using SWD is enabled. (ISP requires enabling read protections first on CH570/2 and disabling Debug Module completely on all other CH5xx chips)
- Configurable - pick and choose which GPIO to use for the BOOT button, whether to use a button at all, how big of a delay to use, etc.
- Easy to reboot into bootloader from your firmware using built-in ``funRebootToBootloader`` function.
- Hackable - this bootloader uses a sketchpad buffer to run binary stubs from, this is how it achieves all of its functions. You can write and run any binary you want if you can fit it in the available RAM.
- Make it your own. You can add any functionality you want, change USB descriptor, add protocol or transport (I2C, UART or even iSLER). Everything is open.
- You can always go back to the factory ISP, just don't forget to backup it, or find a dump of it somewhere on github.

## What

This is a port or adaptation of a bootloader first written for CH32V003 in rv003usb project. It uses hardware USB peripheral that is found on various CH32 and CH5xx chips. It's designed to accept binary stubs that are then run from RAM, this allows to implement wide variety of functions for different chips only implementing changes on a host/programmer side.

WIP! Currently is only tested on CH32X035 and CH570. But probably will work already on other CH5xx. To make it work on CH32V20x and CH32V30x it will require additional stubs for minichlink, and also more testing. CH32V103 will probably not work, because it has weirdly divided in two boot partition and I'm not sure how to correctly flash it (if it's even possible).

## How

If you want, configure options to your needs:

- ``BOOTLOADER_LED_PIN`` - LED to use as operational indicator. Disable if not needed
- ``BOOTLOADER_LED_POLARITY`` - 1 - lights when GPIO is HIGH, 0 - when LOW
- ``BOOTLOADER_BTN_PIN`` - GPIO pin for the BOOT button. Disable if not needed
- ``BOOTLOADER_BTN_TRIG_LEVEL`` - 1 - GPIO is HIGH when the button is pressed, 0 - GPIO is LOW
- ``BOOTLOADER_BTN_PULL`` - 1 - endable PULL-UP, 0 - enable PULL-DOWN for button GPIO. Disable if not needed
- ``BOOTLOADER_TIMEOUT_PWR_MS`` - timeout in milliseconds from entering bootloader until it boots the firmware if no activity was detected
- ``SOFT_REBOOT_TO_BOOTLOADER`` - whether to enable entering bootloader from the firmware using ``funRebootToBootloader`` function.

Then set the target MCU in the Makefile and do ``make flash_boot``. This will owerwrite factory bootloader in the BOOT area of the flash.

Alternatively, ``make`` will flash the bootloader to the main program memory, it will still work, but if you try to write flash with it it will overwrite itself. But this can be useful for testing, while preserving factory ISP in the BOOT area.

To be able to flash this into BOOT area you will need a proper programmer, ISP won't work for this.
