# Location of the stm32f10x periph library
STD_PERIPH_LIB = StdPeriph_lib
CMSIS = $(STD_PERIPH_LIB)/CMSIS/CM3
#
FREERTOS = src/freertos

# Please add all the appropriate src files, (*.o) targets
C_SRCS = delay.c main.c system_stm32f10x.c
# FreeRTOS
# C_SRCS += list.c port.c queue.c croutine.c event_groups.c heap_1.c tasks.c timers.c
C_SRCS += $(wildcard $(FREERTOS)/*.c)
# 
C_SRCS += suart.c
#
C_SRCS += stm32f10x_it.c 
 
CPP_SRCS =

ASM_SRCS += startup_stm32f10x_md.s

# all the files will be generated with this name (main.elf, main.bin, main.hex, etc)
PROJ_NAME = main

# Location of the linker scripts
LDSCRIPT_INC = Device/ldscripts

CC = arm-none-eabi-gcc
CXX = arm-none-eabi-g++
LD = arm-none-eabi-gcc
AR = arm-none-eabi-ar
AS =arm-none-eabi-as
CP = arm-none-eabi-objcopy
OD = arm-none-eabi-objdump

OBJCOPY = arm-none-eabi-objcopy
OBJDUMP = arm-none-eabi-objdump
SIZE = arm-none-eabi-size

# Select the appropriate option for your device, the available options are listed below
# with a description copied from stm32f10x.h
# Make sure to set the startup code file to the right device family, too!
#
# STM32F10X_LD 		STM32F10X_LD: STM32 Low density devices
# STM32F10X_LD_VL	STM32F10X_LD_VL: STM32 Low density Value Line devices
# STM32F10X_MD		STM32F10X_MD: STM32 Medium density devices
# STM32F10X_MD_VL	STM32F10X_MD_VL: STM32 Medium density Value Line devices 
# STM32F10X_HD		STM32F10X_HD: STM32 High density devices
# STM32F10X_HD_VL	STM32F10X_HD_VL: STM32 High density value line devices
# STM32F10X_XL		STM32F10X_XL: STM32 XL-density devices
# STM32F10X_CL		STM32F10X_CL: STM32 Connectivity line devices 
#
# - Low-density devices are STM32F101xx, STM32F102xx and STM32F103xx microcontrollers
#   where the Flash memory density ranges between 16 and 32 Kbytes.
# 
# - Low-density value line devices are STM32F100xx microcontrollers where the Flash
#   memory density ranges between 16 and 32 Kbytes.
# 
# - Medium-density devices are STM32F101xx, STM32F102xx and STM32F103xx microcontrollers
#   where the Flash memory density ranges between 64 and 128 Kbytes.
# 
# - Medium-density value line devices are STM32F100xx microcontrollers where the 
#   Flash memory density ranges between 64 and 128 Kbytes.   
# 
# - High-density devices are STM32F101xx and STM32F103xx microcontrollers where
#   the Flash memory density ranges between 256 and 512 Kbytes.
# 
# - High-density value line devices are STM32F100xx microcontrollers where the 
#   Flash memory density ranges between 256 and 512 Kbytes.   
# 
# - XL-density devices are STM32F101xx and STM32F103xx microcontrollers where
#   the Flash memory density ranges between 512 and 1024 Kbytes.
# 
# - Connectivity line devices are STM32F105xx and STM32F107xx microcontrollers.
#
# HSE_VALUE sets the value of the HSE clock, 8MHz in this case 

# PLEASE CHOOSE THE CORRECT MCU FOR YOUR APPLICATION
MCU_FAMILY = STM32F10X_MD
HSE_VALUE = 8000000

# Default to selecting the STD_PERIPH_DRIVER
DEFS = -DUSE_STDPERIPH_DRIVER -D$(MCU_FAMILY) -DHSE_VALUE=$(HSE_VALUE)

MCU = cortex-m3
MCFLAGS = -mcpu=$(MCU) -mthumb -mlittle-endian -mthumb-interwork

STM32_INCLUDES = -I$(CMSIS)/DeviceSupport/ST/STM32F10x/ \
	-I$(CMSIS)/CoreSupport/ \
	-Iinc -I$(STD_PERIPH_LIB) -I$(STD_PERIPH_LIB)/inc

OPTIMIZE       = -Os
#OPTIMIZE       = -g

CFLAGS = $(MCFLAGS)  $(OPTIMIZE)  $(DEFS) -I. -I./ $(STM32_INCLUDES) -I$(FREERTOS)/include
CFLAGS += -lc -lnosys -specs=nosys.specs -Wl,-Map=$(PROJ_NAME).map -Wl,-T,stm32_flash.ld
CFLAGS += -Wl,--gc-sections  
CFLAGS += -fno-common -mfloat-abi=soft -fsingle-precision-constant -fomit-frame-pointer -ffunction-sections -fdata-sections

AFLAGS = $(MCFLAGS) 

BUILD_DIR = ./build

OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SRCS:.c=.o)))
vpath %.c src
vpath %.c $(FREERTOS)

OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(CPP_SRCS:.cpp=.o)))
vpath %.cpp src

OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SRCS:.s=.o)))
vpath %.s Device

.PHONY: lib

all: lib $(PROJ_NAME).elf 

lib:
	$(warning Please check that the MCU in the makefile matches what you expect.)
	$(MAKE) -C $(STD_PERIPH_LIB)

$(BUILD_DIR)/%.o: %.cpp Makefile | $(BUILD_DIR) 
	@echo "C++. Compiling $@..."
	@$(CXX) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.cpp=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR) 
	@echo "C. Compiling $@..."
	@$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	@echo "S. Compiling $@..."
	@$(AS) -c $(AFLAGS) $< -o $@

$(PROJ_NAME).elf: $(OBJECTS) Makefile
	@echo Please check $(LDSCRIPT_INC) to ensure that the linker script is correct.
	@echo
	@echo $(value STARTUP)
	#$(CC) $(CFLAGS) $^ -o $@ -L$(STD_PERIPH_LIB) -lstm32f1 -lm -L$(LDSCRIPT_INC)
	@$(CXX) $(OBJECTS) $(CFLAGS) $(LDFLAGS) -o $@ -L$(STD_PERIPH_LIB) -lstm32f1 -lm -L$(LDSCRIPT_INC)
	$(OBJCOPY) -O ihex $(PROJ_NAME).elf $(PROJ_NAME).hex
	$(OBJCOPY) -O binary $(PROJ_NAME).elf $(PROJ_NAME).bin
	$(OBJDUMP) -St $(PROJ_NAME).elf >$(PROJ_NAME).lst
	$(SIZE) $(PROJ_NAME).elf

clean:
	find ./ -name '*~' | xargs rm -f
	rm -f $(BUILD_DIR)/*
	rm -f *.o
	rm -f $(PROJ_NAME).elf
	rm -f $(PROJ_NAME).hex
	rm -f $(PROJ_NAME).bin
	rm -f $(PROJ_NAME).map
	rm -f $(PROJ_NAME).lst

distclean: clean
	$(MAKE) -C $(STD_PERIPH_LIB) clean

flash:
	@st-flash write $(PROJ_NAME).bin 0x8000000
