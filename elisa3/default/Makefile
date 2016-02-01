###############################################################################
# Makefile for the project elisa3-aseba
###############################################################################

## General Flags
PROJECT = elisa3-aseba
MCU = atmega2560
TARGET = elisa3-aseba.elf
CC = avr-gcc

CPP = avr-g++

## Options common to compile, link and assembly rules
COMMON = -mmcu=$(MCU)

## Compile options common for all C compilation units.
CFLAGS = $(COMMON)
CFLAGS += -Wall -gdwarf-2 -Os -std=gnu99 -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums
CFLAGS += -MD -MP -MT $(*F).o -MF dep/$(@F).d 

## Assembly specific flags
ASMFLAGS = $(COMMON)
ASMFLAGS += $(CFLAGS)
ASMFLAGS += -x assembler-with-cpp -Wa,-gdwarf2

## Linker flags
LDFLAGS = $(COMMON)
LDFLAGS +=  -Wl,-Map=elisa3-aseba.map


## Intel Hex file production flags
HEX_FLASH_FLAGS = -R .eeprom -R .fuse -R .lock -R .signature

HEX_EEPROM_FLAGS = -j .eeprom
HEX_EEPROM_FLAGS += --set-section-flags=.eeprom="alloc,load"
HEX_EEPROM_FLAGS += --change-section-lma .eeprom=0 --no-change-warnings


## Include Directories
INCLUDES = -I"G:\lavoro\aseba\aseba-dev\targets\elisa3-aseba\..\.." 

## Objects that must be built in order to link
OBJECTS = elisa3aseba.o leds.o motors.o ports_io.o usart.o variables.o behaviors.o speed_control.o adc.o sensors.o twimaster.o vm-buffer.o vm.o natives.o utility.o ir_remote_control.o mirf.o spi.o 

## Objects explicitly added by the user
LINKONLYOBJECTS = 

## Build
all: $(TARGET) elisa3-aseba.hex elisa3-aseba.eep elisa3-aseba.lss size

## Compile
elisa3aseba.o: ../elisa3aseba.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

leds.o: ../leds.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

motors.o: ../motors.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

ports_io.o: ../ports_io.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

usart.o: ../usart.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

variables.o: ../variables.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

behaviors.o: ../behaviors.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

speed_control.o: ../speed_control.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

adc.o: ../adc.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

sensors.o: ../sensors.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

twimaster.o: ../twimaster.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

vm-buffer.o: ../../../transport/buffer/vm-buffer.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

vm.o: ../../../vm/vm.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

natives.o: ../../../vm/natives.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

utility.o: ../utility.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

ir_remote_control.o: ../ir_remote_control.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

mirf.o: ../mirf.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

spi.o: ../spi.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

##Link
$(TARGET): $(OBJECTS)
	 $(CC) $(LDFLAGS) $(OBJECTS) $(LINKONLYOBJECTS) $(LIBDIRS) $(LIBS) -o $(TARGET)

%.hex: $(TARGET)
	avr-objcopy -O ihex $(HEX_FLASH_FLAGS)  $< $@

%.eep: $(TARGET)
	-avr-objcopy $(HEX_EEPROM_FLAGS) -O ihex $< $@ || exit 0

%.lss: $(TARGET)
	avr-objdump -h -S $< > $@

size: ${TARGET}
	@echo
	@avr-size -C --mcu=${MCU} ${TARGET}

## Clean target
.PHONY: clean
clean:
	-rm -rf $(OBJECTS) elisa3-aseba.elf dep/* elisa3-aseba.hex elisa3-aseba.eep elisa3-aseba.lss elisa3-aseba.map


## Other dependencies
-include $(shell mkdir dep 2>NUL) $(wildcard dep/*)

