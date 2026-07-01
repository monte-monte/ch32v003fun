ifeq ($(OS),Windows_NT)
	WHICH:=where
	NULLDEV:=nul
else
	WHICH:=which
	NULLDEV:=/dev/null
	OS_NAME := $(shell uname -s | tr A-Z a-z)
endif

PREFIX_DEFAULT:=riscv64-elf

ifneq ($(shell $(WHICH) riscv64-unknown-elf-gcc 2>$(NULLDEV)),)
	PREFIX_DEFAULT:=riscv64-unknown-elf
else ifneq ($(shell $(WHICH) riscv-none-elf-gcc 2>$(NULLDEV)),)
	PREFIX_DEFAULT:=riscv-none-elf
else ifneq ($(shell $(WHICH) riscv64-linux-gnu-gcc 2>$(NULLDEV)),)
	PREFIX_DEFAULT:=riscv64-linux-gnu
else ifeq ($(OS_NAME),netbsd)
	# assume using pkgsrc/cross/riscv64-none-elf-gcc and
	# pkgsrc/cross/riscv64-none-elf-binutils
	PKGSRC_LOCALBASE ?= /usr/pkg
	ifneq ($(wildcard $(PKGSRC_LOCALBASE)/cross-riscv64-none-elf/bin/riscv64-none-elf-gcc),)
		PREFIX_DEFAULT := $(PKGSRC_LOCALBASE)/cross-riscv64-none-elf/bin/riscv64-none-elf
	endif
endif

PREFIX?=$(PREFIX_DEFAULT)

FLAGS:=-march=rv32imac_zicsr -mabi=ilp32 -nostartfiles -nostdlib -Ttext 0x0 -mno-relax -e 0x0

ASM:=$(wildcard *.S)
OBJ:=$(ASM:.S=.o)

%.o: %.S
	$(PREFIX)-gcc $(FLAGS) $< -o $@
	$(PREFIX)-objcopy -O binary $@ $(@:.o=.bin)
	xxd -i $(@:.o=.bin) > $(@:.o=.h)

%.h: %.o
	$(@.h=.o)

%: %.o
	$(@.o)

all: $(OBJ)

clean:
	rm -f $(OBJ) $(OBJ:.o=.bin) $(OBJ:.o=.h)