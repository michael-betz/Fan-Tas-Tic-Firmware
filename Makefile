
#--------------------------------
# Misc constants
#--------------------------------
TARGET      = fantastic
USB_SERIAL  = /dev/ttyUSB0
PART        = TM4C123GH6PM
TIVAWARE    = /home/michael/ti/tivaware
GIT_VERSION = $(shell git describe --abbrev=4 --dirty --always --tags)
PREFIX      = arm-none-eabi
CC			= ${PREFIX}-gcc

#--------------------------------
# Source files (.c)
#--------------------------------
SRCS        = $(wildcard *.c)
SRCS       += $(wildcard utils/*.c)
SRCS       += $(wildcard drivers/*.c)
SRCS       += $(wildcard FreeRTOS/*.c)
SRCS       += $(wildcard $(TIVAWARE)/third_party/FreeRTOS/Source/*.c)
SRCS       += $(wildcard $(TIVAWARE)/third_party/FreeRTOS/Source/portable/GCC/ARM_CM4F/*.c)
SRCS       += $(TIVAWARE)/third_party/FreeRTOS/Source/portable/MemMang/heap_2.c
OBJS        = $(subst .c,.o,$(SRCS))

#--------------------------------
# Header files (.h)
#--------------------------------
INC         = -I. -Iutils -Idrivers
INC        += -I$(TIVAWARE)
INC        += -I$(TIVAWARE)/third_party/FreeRTOS/Source/include
INC        += -I$(TIVAWARE)/third_party/FreeRTOS/Source/portable/GCC/ARM_CM4F

#--------------------------------
# Compiler flags
#--------------------------------
## Warnings, standards
CFLAGS      = -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
CFLAGS     += -c -O0 -Wall -Werror -std=c99
CFLAGS     += -Dgcc -DPART_${PART} -DTARGET_IS_TM4C123_RB1
# CFLAGS     += -ffunction-sections -fdata-sections
CFLAGS     += -DUART_BUFFERED
CFLAGS     += -DGIT_VERSION=\"$(GIT_VERSION)\"

#--------------------------------
# Linker flags
#--------------------------------
LDFLAGS     = -Wl,-T,project.ld
LDFLAGS    += -Wl,-Map=$(TARGET).map,--cref
# Optional, but often ends up with smaller code
# LDFLAGS    += -Wl,--gc-sections
LDFLAGS    += -Wl,--relax,--stats,--verbose
LDFLAGS    += -Wl,-L$(TIVAWARE)/driverlib/gcc
LDFLAGS    += -Wl,-L$(TIVAWARE)/usblib/gcc

all: $(TARGET).axf

bootload: clean $(TARGET).hex
	-pkill miniterm*
	-pkill xterm*
	avrdude -P $(USB_SERIAL) -b 57600 -c arduino -p atmega328p -U flash:w:$(TARGET).hex
	xterm -fa monaco -fs 15 -bg black -fg green -hold -e "miniterm.py $(USB_SERIAL) 115200"&

%.o: %.c
	@echo -----------------------------------------------
	@echo  Compiling $@
	@echo -----------------------------------------------
	$(CC) $(INC) $(CFLAGS) -o $@ -c $<

%.hex: %.axf
	@echo -----------------------------------------------
	@echo  Generating intel hex file
	@echo -----------------------------------------------
	$(TOOLS_PREFIX)objcopy -O ihex -R .eeprom $< $@

$(TARGET).axf: $(OBJS)
	@echo -----------------------------------------------
	@echo  Linking together an application .axf file
	@echo -----------------------------------------------
	$(CC) $(LDFLAGS) -o $@ $^ -Wl,-lusb,-ldriver
	chmod -x $@
	$(TOOLS_PREFIX)size $@

clean:
	rm -f $(TARGET).hex $(TARGET).lst $(TARGET).axf $(TARGET).map $(OBJS)

.PHONY: clean all bootload
