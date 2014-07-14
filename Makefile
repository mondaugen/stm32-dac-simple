# put your *.o targets here, make should handle the rest!

SRCS = main.c stm32f4xx_it.c system_stm32f4xx.c syscalls.c synths.c interpolation.c
INCS = inc/synths.h inc/interpolation.h

# all the files will be generated with this name (main.elf, main.bin, main.hex, etc)

PROJ_NAME=main

# DON'T: optimize, this makes setting the clock not work (why?)

CC=arm-none-eabi-gcc
OBJCOPY=arm-none-eabi-objcopy

CFLAGS  = -g -Wall -Tstm32_flash.ld -O0 
CFLAGS += -mlittle-endian -mthumb -mcpu=cortex-m4 #-mthumb-interwork
CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16 -march=armv7e-m

vpath %.c src
vpath %.a lib

ROOT=$(shell pwd)

CFLAGS += -Iinc -Ilib -Ilib/inc 
CFLAGS += -Ilib/inc/core -Ilib/inc/peripherals

SRCS += lib/startup_stm32f4xx.s # add startup file to build

OBJS = $(SRCS:.c=.o)

###################################################

.PHONY: lib proj

all: lib proj

lib:
	$(MAKE) -C lib

proj: 	$(PROJ_NAME).elf

$(PROJ_NAME).elf: $(SRCS) $(INCS)
	$(CC) $(CFLAGS) $(filter-out %.h,$^) -o $@ -Llib -lstm32f4 -lm
	$(OBJCOPY) -O ihex $(PROJ_NAME).elf $(PROJ_NAME).hex
	$(OBJCOPY) -O binary $(PROJ_NAME).elf $(PROJ_NAME).bin

clean:
	$(MAKE) -C lib clean
	rm -f $(PROJ_NAME).elf
	rm -f $(PROJ_NAME).hex
	rm -f $(PROJ_NAME).bin
