TARGET=flash
CC=arm-none-eabi-gcc
MACH=cortex-m4
FLOAT=soft
LINKER_SCRIPT=Linkers/STM32F401xC.ld

#DEFINES+= -DNDEBUG

CFLAGS= -c -mcpu=$(MACH) -mthumb -mfloat-abi=$(FLOAT) -std=gnu11 -Wall -O0 -ggdb $(DEFINES)
LDFLAGS =  -mcpu=$(MACH) -mthumb -mfloat-abi=$(FLOAT) --specs=nano.specs -T $(LINKER_SCRIPT) -Wl,-Map=$(TARGET).map -ggdb -u _printf_float
OBJCOPY=arm-none-eabi-objcopy

###########################################
#				 INCLUDES

INCLUDES+= -I App/Inc/
INCLUDES+= -I Drivers/Inc/ 


############################################
# 				OUTPUT FILES

OBJS		+= App/Build/main.o
OBJS		+= App/Build/syscalls.o
OBJS		+= App/Build/startup.o

OBJS		+= App/Build/driver_systick.o
OBJS		+= App/Build/driver_interrupt.o
OBJS		+= App/Build/driver_gpio.o
OBJS		+= App/Build/driver_uart.o
OBJS		+= App/Build/system.o


all: App/Build/$(TARGET).elf

App/Build/%.o: App/Src/%.c
	$(CC) $(CFLAGS) $(INCLUDES) -o App/Build/$(*).o App/Src/$(*).c

App/Build/%.o: Src/%.S
	$(CC) $(CFLAGS) $(INCLUDES) -o App/Build/$(*).o App/Src/$(*).S

App/Build/%.o: Linkers/%.c
	$(CC) $(CFLAGS) $(INCLUDES) -o App/Build/$(*).o Linkers/$(*).c

App/Build/%.o: Drivers/Src/%.c
	$(CC) $(CFLAGS) $(INCLUDES) -o App/Build/$(*).o Drivers/Src/$(*).c

App/Build/$(TARGET).elf: $(OBJS)
	$(CC) $(LDFLAGS) -o $@ $^
	$(OBJCOPY) -O binary App/Build/$(TARGET).elf App/Build/$(TARGET).bin
	arm-none-eabi-size App/Build/$(TARGET).elf

load:
	openocd -f interface/jlink.cfg -c "transport select swd" -f target/stm32f4x.cfg -c init -c "reset init" \
	-c "flash write_image erase App/Build/$(TARGET).elf" -c "reset run" -c shutdown

debug:
	openocd -f interface/jlink.cfg -c "transport select swd" -f target/stm32f4x.cfg -c init -c "reset init"

reset:
	openocd -f interface/jlink.cfg -c "transport select swd" -f target/stm32f4x.cfg -c init -c "reset run" -c shutdown

clean:
	rm -rf *.o *.map App/Build/*.o