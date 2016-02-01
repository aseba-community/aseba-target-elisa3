################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL := cmd.exe
RM := rm -rf

USER_OBJS :=

LIBS := 
PROJ := 

O_SRCS := 
C_SRCS := 
S_SRCS := 
S_UPPER_SRCS := 
OBJ_SRCS := 
ASM_SRCS := 
PREPROCESSING_SRCS := 
OBJS := 
OBJS_AS_ARGS := 
C_DEPS := 
C_DEPS_AS_ARGS := 
EXECUTABLES := 
OUTPUT_FILE_PATH :=
OUTPUT_FILE_PATH_AS_ARGS :=
AVR_APP_PATH :=$$$AVR_APP_PATH$$$
QUOTE := "
ADDITIONAL_DEPENDENCIES:=
OUTPUT_FILE_DEP:=
LIB_DEP:=
LINKER_SCRIPT_DEP:=

# Every subdirectory with source files must be described here
SUBDIRS := 


# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS +=  \
../eepromIO.c \
../elisa_natives.c \
../irCommunication.c \
../natives.c \
../elisa3aseba.c \
../leds.c \
../motors.c \
../ports_io.c \
../usart.c \
../variables.c \
../behaviors.c \
../speed_control.c \
../adc.c \
../sensors.c \
../twimaster.c \
../utility.c \
../ir_remote_control.c \
../mirf.c \
../spi.c \
../vm-buffer.c \
../vm.c


PREPROCESSING_SRCS += 


ASM_SRCS += 


OBJS +=  \
eepromIO.o \
elisa_natives.o \
irCommunication.o \
natives.o \
elisa3aseba.o \
leds.o \
motors.o \
ports_io.o \
usart.o \
variables.o \
behaviors.o \
speed_control.o \
adc.o \
sensors.o \
twimaster.o \
utility.o \
ir_remote_control.o \
mirf.o \
spi.o \
vm-buffer.o \
vm.o

OBJS_AS_ARGS +=  \
eepromIO.o \
elisa_natives.o \
irCommunication.o \
natives.o \
elisa3aseba.o \
leds.o \
motors.o \
ports_io.o \
usart.o \
variables.o \
behaviors.o \
speed_control.o \
adc.o \
sensors.o \
twimaster.o \
utility.o \
ir_remote_control.o \
mirf.o \
spi.o \
vm-buffer.o \
vm.o

C_DEPS +=  \
eepromIO.d \
elisa_natives.d \
irCommunication.d \
natives.d \
elisa3aseba.d \
leds.d \
motors.d \
ports_io.d \
usart.d \
variables.d \
behaviors.d \
speed_control.d \
adc.d \
sensors.d \
twimaster.d \
utility.d \
ir_remote_control.d \
mirf.d \
spi.d \
vm-buffer.d \
vm.d

C_DEPS_AS_ARGS +=  \
eepromIO.d \
elisa_natives.d \
irCommunication.d \
natives.d \
elisa3aseba.d \
leds.d \
motors.d \
ports_io.d \
usart.d \
variables.d \
behaviors.d \
speed_control.d \
adc.d \
sensors.d \
twimaster.d \
utility.d \
ir_remote_control.d \
mirf.d \
spi.d \
vm-buffer.d \
vm.d

OUTPUT_FILE_PATH +=elisa3-aseba.elf

OUTPUT_FILE_PATH_AS_ARGS +=elisa3-aseba.elf

ADDITIONAL_DEPENDENCIES:=

OUTPUT_FILE_DEP:= ./makedep.mk

LIB_DEP+= 

LINKER_SCRIPT_DEP+= 


# AVR32/GNU C Compiler











































./%.o: .././%.c
	@echo Building file: $<
	@echo Invoking: AVR/GNU C Compiler : 4.9.2
	$(QUOTE)C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-gcc.exe$(QUOTE)  -x c -funsigned-char -funsigned-bitfields -I"..\..\.." -I"C:\Program Files (x86)\Atmel\Studio\7.0\Packs\atmel\ATmega_DFP\1.0.90\include" -I"../../../../aseba/trunk" -I"../../../../aseba/trunk/vm" -I"../../../../aseba/trunk/transport/buffer"  -O3 -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -Wall -mmcu=atmega2560 -B "C:\Program Files (x86)\Atmel\Studio\7.0\Packs\atmel\ATmega_DFP\1.0.90\gcc\dev\atmega2560" -c -gdwarf-2 -std=gnu99 -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"   -o "$@" "$<" 
	@echo Finished building: $<
	



# AVR32/GNU Preprocessing Assembler



# AVR32/GNU Assembler




ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: $(OUTPUT_FILE_PATH) $(ADDITIONAL_DEPENDENCIES)

$(OUTPUT_FILE_PATH): $(OBJS) $(USER_OBJS) $(OUTPUT_FILE_DEP) $(LIB_DEP) $(LINKER_SCRIPT_DEP)
	@echo Building target: $@
	@echo Invoking: AVR/GNU Linker : 4.9.2
	$(QUOTE)C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-gcc.exe$(QUOTE) -o$(OUTPUT_FILE_PATH_AS_ARGS) $(OBJS_AS_ARGS) $(USER_OBJS) $(LIBS) -Wl,-Map="elisa3-aseba.map" -Wl,--start-group  -Wl,--end-group -Wl,-L"H:\lavoro\_elisa-3\aseba\aseba\trunk\vm"  -Wl,--gc-sections -mmcu=atmega2560 -B "C:\Program Files (x86)\Atmel\Studio\7.0\Packs\atmel\ATmega_DFP\1.0.90\gcc\dev\atmega2560"  
	@echo Finished building target: $@
	"C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-objcopy.exe" -O ihex -R .eeprom -R .fuse -R .lock -R .signature -R .user_signatures  "elisa3-aseba.elf" "elisa3-aseba.hex"
	"C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-objcopy.exe" -j .eeprom  --set-section-flags=.eeprom=alloc,load --change-section-lma .eeprom=0  --no-change-warnings -O ihex "elisa3-aseba.elf" "elisa3-aseba.eep" || exit 0
	"C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-objdump.exe" -h -S "elisa3-aseba.elf" > "elisa3-aseba.lss"
	"C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-objcopy.exe" -O srec -R .eeprom -R .fuse -R .lock -R .signature -R .user_signatures "elisa3-aseba.elf" "elisa3-aseba.srec"
	"C:\Program Files (x86)\Atmel\Studio\7.0\toolchain\avr8\avr8-gnu-toolchain\bin\avr-size.exe" "elisa3-aseba.elf"
	
	





# Other Targets
clean:
	-$(RM) $(OBJS_AS_ARGS) $(EXECUTABLES)  
	-$(RM) $(C_DEPS_AS_ARGS)   
	rm -rf "elisa3-aseba.elf" "elisa3-aseba.a" "elisa3-aseba.hex" "elisa3-aseba.lss" "elisa3-aseba.eep" "elisa3-aseba.map" "elisa3-aseba.srec" "elisa3-aseba.usersignatures"
	