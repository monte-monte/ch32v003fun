# USBFS test device

This is a simple example of basic USBFS functionality. It creates two HID devices - a mouse and a keyboard. *Keyboard* send 3 presses of ``8`` and *mouse* slowly crawls the cursor diagonally to the bottom right corner of a screen.

To test HIDAPI transfers you can use a ``testtop`` program that can be found in a corresponding subdirectory.

## Notes about performance on CH5xx

CH5xx don't have a cached flash like CH32. To achieve comparable performance EVT puts all critical functions into RAM. This increases performance significantly, but it's still worse than on CH32 models. To enable this you can use ``FUSB_FROM_RAM`` option that will put all relevant functions to RAM.
