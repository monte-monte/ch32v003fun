# USBFS keyboard

A minimal USB keyboard implementation. It creates HID keyboard device on EP1. Standard keyboard USB packet is 8 bytes, fist byte is a modifier key (Alt, Ctrl, Shift), second is *reserved* and then go 6 bytes of keys in a format that you can find in ``usb_hid_keys.h``. After sending a packet with a keystrokes you have to follow with an empty packet, or the last keystroke from the previous packet will be repeated until canceled.

I was using a generic 4x3 keypad matrix. You can use something else, or make a real keyboard, but then you will need to implement your own scan function. 

This example has predefined GPIO setting for use with EVT board of CH570/2, WeAct CH32X035 and WeAct CH592 boards. Make changes accordingly if you want to use another board.
