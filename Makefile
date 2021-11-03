PROJ_NAME=omnia-gd32f150

################################################################################
#                   SETUP TOOLS                                                #
################################################################################

CC      = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
AS      = arm-none-eabi-as
SIZE	= arm-none-eabi-size

##### Preprocessor options
#FW_VERSION = "{ $(shell git rev-parse HEAD | sed 's/\(..\)/0x\1, /g' | sed -r 's/,\s+$$//') }"

#defines needed when working with the STM peripherals library
DEFS 	= -DGD32F1x0
DEFS   += -DGD32F130_150
#DEFS   += -DSTM32F40_41xxx

DEFS   += -DUSE_STDPERIPH_DRIVER
#DEFS   += -D__ASSEMBLY_
DEFS   += -DIRC8M_VALUE=8000000
#DEFS   += -DVERSION=$(FW_VERSION)

##### Assembler options

AFLAGS  = -mcpu=cortex-m3 
AFLAGS += -mthumb
AFLAGS += -mthumb-interwork
AFLAGS += -mlittle-endian
AFLAGS += -mfloat-abi=soft
#AFLAGS += -mfpu=fpv4-sp-d1

##### Compiler options

CFLAGS  = -ggdb
CFLAGS += -O0
CFLAGS += -Wall -Wextra -Warray-bounds #-pedantic
CFLAGS += $(AFLAGS)
CFLAGS += -nostdlib
CFLAGS += -ffunction-sections
CFLAGS += -fdata-sections

##### Linker options

# tell ld which linker file to use
# (this file is in the current directory)
#  -Wl,...:     tell GCC to pass this to linker.
#    -Map:      create map file
#    --cref:    add cross reference to  map file
LFLAGS  = -T$(LINKER_DIR)/GD32F150_FLASH.ld
LFLAGS +="-Wl,-Map=$(PROJ_NAME).map",--cref
LFLAGS += -nostartfiles
LFLAGS += -Xlinker --gc-sections

# directories to be searched for header files
INCLUDE = $(addprefix -I,$(INC_DIRS))

################################################################################
#                   SOURCE FILES DIRECTORIES                                   #
################################################################################
PROJ_ROOT_DIR	= src

#APPLICATION
APP_ROOT_DIR	= $(PROJ_ROOT_DIR)/application


STM_ROOT_DIR = $(APP_ROOT_DIR)/GD32F1x0_Firmware_Library
APP_SRC_DIR		= $(APP_ROOT_DIR)/app
STM_LIB_DIR = $(STM_ROOT_DIR)/GD32F1x0_standard_peripheral/Source
STM_DEVICE = $(STM_ROOT_DIR)/CMSIS/GD/GD32F1x0/Source
STM_STARTUP_DIR = $(STM_ROOT_DIR)/CMSIS/GD/GD32F1x0/Source/GCC

LINKER_DIR = $(APP_ROOT_DIR)/linker

vpath %.c $(APP_ROOT_DIR)
vpath %.c $(APP_SRC_DIR)
vpath %.c $(STM_LIB_DIR)
vpath %.c $(STM_DEVICE)
vpath %.s $(STM_STARTUP_DIR)

################################################################################
#                   HEADER FILES DIRECTORIES                                   #
################################################################################

# The header files we use are located here
INC_DIRS += $(STM_ROOT_DIR)/GD32F1x0_standard_peripheral/Include
INC_DIRS += $(STM_ROOT_DIR)/CMSIS
INC_DIRS += $(STM_ROOT_DIR)/CMSIS/GD/GD32F1x0/Include
INC_DIRS += $(APP_SRC_DIR)

################################################################################
#                   SOURCE FILES TO COMPILE                                    #
################################################################################
SRCS  += main.c
SRCS  += gd32f1x0_it.c



################# STM LIB ##########################
SRCS  += system_gd32f1x0.c

SRCS  += gd32f1x0_misc.c
SRCS  += gd32f1x0_rcu.c
SRCS  += gd32f1x0_gpio.c
SRCS  += gd32f1x0_usart.c
SRCS  += gd32f1x0_timer.c
SRCS  += gd32f1x0_spi.c
SRCS  += gd32f1x0_i2c.c
SRCS  += gd32f1x0_fmc.c

SRCS  += wan_lan_pci_status.c
SRCS  += slave_i2c_device.c
SRCS  += power_control.c
SRCS  += msata_pci.c
SRCS  += led_driver.c
SRCS  += delay.c
SRCS  += debug_serial.c
SRCS  += debounce.c
SRCS  += app.c
SRCS  += eeprom.c

# startup file, calls main
ASRC  = startup_gd32f1x0.s

OBJS  = $(SRCS:.c=.o)
OBJS += $(ASRC:.s=.o)

################################################################################
#                         SIZE OF OUTPUT                                       #
################################################################################
ELFSIZE = $(SIZE) -d $(PROJ_NAME).elf

buildsize: $(PROJ_NAME).elf
	@echo Program Size: 
	$(ELFSIZE)

################################################################################
#                         SETUP TARGETS                                        #
################################################################################

.PHONY: all

all: $(PROJ_NAME).elf buildsize
						
$(PROJ_NAME).elf: $(OBJS)
	@echo "[Linking    ]  $@"	
	@$(CC) $(CFLAGS) $(LFLAGS) $(INCLUDE) $^ -o $@
	@$(OBJCOPY) -O ihex $(PROJ_NAME).elf   $(PROJ_NAME).hex
	@$(OBJCOPY) -O binary $(PROJ_NAME).elf $(PROJ_NAME).bin

%.o : %.c
	@echo "[Compiling  ]  $^"
	$(CC) -c $(DEFS) $(CFLAGS) $(INCLUDE) $< -o $@

%.o : %.s 
	@echo "[Assembling ]" $^
	@$(AS) $(AFLAGS) $< -o $@

clean:
	rm -r -f *.o $(PROJ_NAME).elf $(PROJ_NAME).hex $(PROJ_NAME).bin $(PROJ_NAME).map

#********************************
# generating of the dependencies
dep:	
	$(CC) $(DEFS) $(CFLAGS) $(INCLUDE) -MM $(PROJ_ROOT_DIR)/*.c $(APP_SRC_DIR)/*.c > dep.list

## insert generated dependencies
-include dep.list 
#*******************************

