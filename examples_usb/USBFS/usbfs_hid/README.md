# USBFS HID device

This example shows two methods of transfering data using HID. First one is using HID feature reports, that sends data IN and OUT via control requests to EP0. Second method allows write to and read from hidraw device on linux using IN and OUT interrupt transfers to EP1.

To test feature report transfers you can use a ``testtop`` program that can be found in a corresponding subdirectory.

To test hidraw, you need to find which one is assigned to your device (the simplest way is to look into dmesg) and then do ``cat /dev/hidraw`` to read from and ``echo -ne '\x01\x02\x03\x04\x05\x06\x07\x08' > /dev/hidraw``.

## Useful links about USB HID

- https://eleccelerator.com/usbdescreqparser/
- https://www.usb.org/sites/default/files/documents/hid1_11.pdf
- https://www.usb.org/sites/default/files/hut1_4.pdf
