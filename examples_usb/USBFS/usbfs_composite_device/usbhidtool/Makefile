all : usbhidtool

VID?=1209
PID?=d035

# to make CI happy.
usbhidtool.hidapi.elf : usbhidtool.c
	gcc -o $@ $^ -I../../../../minichlink -ludev

usbhidtool : usbhidtool.c
	gcc -o $@ $^ -I../../../../minichlink -ludev

go_into_bootloader : usbhidtool
	./usbhidtool -v $(VID) -p $(PID) -r

usermonitor : usbhidtool
	./usbhidtool -v $(VID) -p $(PID) -t

clean :
	rm -rf *.o *~ usbhidtool

