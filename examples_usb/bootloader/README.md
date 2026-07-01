# USB Bootloader

This is a port or adaptation of a bootloader first written for CH32V003 in rv003usb project. This one uses hardware USB peripheral that is found on various CH32 and CH5xx chips. It designed to accept binary stubs that are then run from RAM, this allows to implement wide variety of functions for different chips only implementing chages on a host/programmer side.

WIP! Currently is only tested on CH32X035 and CH570. But probably will work already on other CH5xx. To make it work on CH32V20x and CH32V30x it will require additional stubs for minichlink, and also more testing. CH32V103 will probably not work, because it has weirdly divided in two boot partition and I'm not sure how to correctly flash it (if it's even possible).
