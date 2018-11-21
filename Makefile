# VERBOSE = 1
# DEBUG = 1
TARGET = fantastic
PART = TM4C123GH6PM
ROOT = /home/michael/ti/tivaware
GIT_VERSION = $(shell git describe --abbrev=4 --dirty --always --tags)
include ${ROOT}/makedefs

LIBC = /usr/lib/arm-none-eabi/newlib/armv7e-m/fpu/fpv5-d16/libc.a

#--------------------------------
# Source files (.c)
#--------------------------------
# Folders to search
VPATH       = drivers
VPATH      += $(ROOT)/utils
VPATH      += $(ROOT)/third_party/FreeRTOS/Source
VPATH      += $(ROOT)/third_party/FreeRTOS/Source/portable/GCC/ARM_CM4F
VPATH      += $(ROOT)/third_party/FreeRTOS/Source/portable/MemMang
# Files to compile
SRCS        = i2cHandlerTask.c  main.c  mySpi.c  myTasks.c  startup_gcc.c
SRCS       += my_uartstdio.c usbCallbacks.c  usb_serial_structs.c
SRCS       += cmdline.c ustdlib.c
# FreeRTOS stuff from tivaware folder
SRCS       += croutine.c  event_groups.c  list.c  queue.c  tasks.c  timers.c
SRCS       += port.c heap_2.c
# Turn into gcc/*.o files, add libs and linker scripts
OBJS		= $(addprefix ${COMPILER}/, $(subst .c,.o,$(SRCS)))
OBJS       += ${ROOT}/usblib/${COMPILER}/libusb.a
OBJS       += ${ROOT}/sensorlib/${COMPILER}/libsensor.a
OBJS       += ${ROOT}/driverlib/${COMPILER}/libdriver.a
OBJS       += $(TARGET).ld

#--------------------------------
# Header files (.h)
#--------------------------------
IPATH 	    = . drivers
IPATH      += $(ROOT)
IPATH      += $(ROOT)/third_party/FreeRTOS/Source/include
IPATH      += $(ROOT)/third_party/FreeRTOS/Source/portable/GCC/ARM_CM4F

#--------------------------------
# Flags
#--------------------------------
CFLAGS 	   += -DGIT_VERSION=\"$(GIT_VERSION)\"
CFLAGS 	   += -DTARGET_IS_TM4C123_RB1 -DUART_BUFFERED
SCATTERgcc_$(TARGET) = $(TARGET).ld
ENTRY_$(TARGET) = ResetISR

# The default rule, which causes the $(TARGET) example to be built.
all: ${COMPILER}
all: ${COMPILER}/$(TARGET).axf

# The rule to create the target directory.
${COMPILER}:
	mkdir -p ${COMPILER}

${COMPILER}/$(TARGET).axf: $(OBJS)

flash: ${COMPILER}/$(TARGET).axf
	openocd --file board/ek-tm4c123gxl.cfg -c "program $< verify reset exit"

# The rule to clean out all the build products.
clean:
	rm -rf ${COMPILER} ${wildcard *~}

# Include the automatically generated dependency files.
ifneq (${MAKECMDGOALS},clean)
-include ${wildcard ${COMPILER}/*.d} __dummy__
endif

.PHONY: all flash clean
