#--------------------------------
# Misc constants
#--------------------------------
TARGET      = fantastic
USB_SERIAL  = /dev/ttyACM0
PART        = TM4C123GH6PM
TIVAWARE    = /home/michael/ti/tivaware
TOOLS_PREFIX= arm-none-eabi-
CC			= ${TOOLS_PREFIX}gcc

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
CFLAGS      = -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=softfp
CFLAGS     += -c -O3 -Wall -Werror -std=c99
CFLAGS     += -Dgcc -DPART_${PART} -DTARGET_IS_TM4C123_RB1
CFLAGS     += -DUART_BUFFERED
# CFLAGS     += -ffunction-sections -fdata-sections

#--------------------------------
# Linker flags
#--------------------------------
LDFLAGS     = -Wl,-T,tm4c123gh6pm.lds
LDFLAGS    += -Wl,--relax,--stats
LDFLAGS    += -Wl,-L$(TIVAWARE)/driverlib/gcc
LDFLAGS    += -Wl,-L$(TIVAWARE)/usblib/gcc
# LDFLAGS    += -Wl,--gc-sections

all: $(TARGET).axf

%.o: %.c
	@echo -----------------------------------------------
	@echo  Compiling $@
	@echo -----------------------------------------------
	$(CC) $(INC) $(CFLAGS) -o $@ -c $<

$(TARGET).axf: $(OBJS)
	@echo -----------------------------------------------
	@echo  Linking together an application .axf file
	@echo -----------------------------------------------
	$(CC) $(LDFLAGS) -o $@ $^ -Wl,-lusb,-ldriver
	chmod -x $@
	$(TOOLS_PREFIX)size $@

%.bin: %.axf
	$(TOOLS_PREFIX)objcopy -O binary $< $@

flash: $(TARGET).bin
	openocd --file board/ek-tm4c123gxl.cfg -c "program $< verify reset exit"

terminal: clean $(TARGET).hex
	-pkill miniterm*
	-pkill xterm*
	xterm -fa monaco -fs 15 -bg black -fg green -hold -e "miniterm.py $(USB_SERIAL) 115200"&

clean:
	rm -f $(TARGET).hex $(TARGET).lst $(TARGET).axf $(TARGET).map $(OBJS)
	rm -f $(TARGET).bin

.PHONY: clean all flash terminal
