all : flash

TARGET:=coremark
# TARGET_MCU:=CH32V006
# TARGET_MCU:=CH32X035
# TARGET_MCU:=CH32V103
# TARGET_MCU:=CH32V203
# TARGET_MCU_PACKAGE:=CH32V203F8 #64K version
# TARGET_MCU:=CH32V208
# TARGET_MCU_PACKAGE:=CH32V208WBU6
# TARGET_MCU:=CH32V307
TARGET_MCU:=CH570
TARGET_MCU_PACKAGE:=CH570D
# TARGET_MCU:=CH585
# TARGET_MCU_PACKAGE:=CH585F

COREMARK:=coremark
ENABLE_FPU:=0
CORE_ITERATIONS=4000 #Standard is 4K if you get runs less than 10s increase this number
CORE_DATA_SIZE=2000 #Will result in this much RAM use
CORE_MEM_METHOD=0 #0 - STATIC, 1 - HEAP, 2 - STACK
CORE_MEM_STR="STATIC"

LDFLAGS+=
# OPT_FLAGS:=-O3 -funroll-all-loops -finline-limit=600 -ftree-dominator-opts -fno-if-conversion2 -fselective-scheduling -fno-code-hoisting
# OPT_FLAGS:=-Os -flto -ffunction-sections -fdata-sections -fmessage-length=0 -msmall-data-limit=8
OPT_FLAGS:=-O3

COREMARK_CFLAGS=$(CFLAGS_ARCH) -DTARGET_MCU=$(TARGET_MCU) -DMCU_PACKAGE=$(MCU_PACKAGE) -static-libgcc -I$(NEWLIB) -I$(CH32FUN) -nostdlib -I. -Wall $(EXTRA_CFLAGS) -I$(COREMARK) -DCOMPILER_FLAGS=\""$(OPT_FLAGS)"\" -DITERATIONS=$(CORE_ITERATIONS) -DTOTAL_DATA_SIZE=$(CORE_DATA_SIZE) -DMEM_METHOD=$(CORE_MEM_METHOD) -DMEM_LOCATION=\"$(CORE_MEM_STR)\" -DMAIN_HAS_NOARGC -ffreestanding $(OPT_FLAGS)
include ../../ch32fun/ch32fun.mk
CFLAGS+=-ffreestanding
COREMARK_FILES=core_portme.c $(COREMARK)/core_list_join.c $(COREMARK)/core_main.c $(COREMARK)/core_matrix.c $(COREMARK)/core_state.c $(COREMARK)/core_util.c

RAM_LD_FILE:=generated_$(TARGET_MCU_PACKAGE)_RAM.ld

$(RAM_LD_FILE):
	$(PREFIX)-gcc -E -P -x c -DTARGET_MCU=$(TARGET_MCU) -DMCU_PACKAGE=$(MCU_PACKAGE) -DTARGET_MCU_LD=$(TARGET_MCU_LD) -DTARGET_MCU_MEMORY_SPLIT=$(TARGET_MCU_MEMORY_SPLIT) app_in_ram.ld > $(RAM_LD_FILE)

coremark.elf:
	$(PREFIX)-gcc -c -o ch32fun.o $(SYSTEM_C) $(CFLAGS)
	$(PREFIX)-gcc -c -o printf.o printf.c $(CFLAGS)
	$(PREFIX)-gcc -o $@ ch32fun.o printf.o $(COREMARK_FILES) $(COREMARK_CFLAGS) $(LDFLAGS)
# 	$(PREFIX)-gcc -o $@ ch32fun.o $(COREMARK_FILES) $(COREMARK_CFLAGS) $(LDFLAGS)

runfromram: LINKER_SCRIPT:=$(RAM_LD_FILE)

runfromram: $(RAM_LD_FILE) $(TARGET).bin
	$(MINICHLINK)/minichlink -ks 0x10 0x80000001 -s 0x10 0x80000003 -s 0x10 0x80000001 -s 0x18 0 -w $(TARGET).bin 0x20000000 -s 0x04 0x20000000 -s 0x17 0x002307b1 -s 0x10 0x40000001 -T
	
flash: cv_flash	
clean:
	rm -rf ch32fun.o printf.o $(TARGET).elf $(TARGET).bin $(TARGET)_ext.bin $(TARGET).hex $(TARGET).lst $(TARGET).map $(TARGET).hex $(GENERATED_LD_FILE) $(RAM_LD_FILE) || true

