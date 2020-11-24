APP_NAME=omnia_hw_ctrl
BOOT_NAME=bootloader_mcu

################################################################################
#                   SETUP TOOLS                                                #
################################################################################

CROSS_COMPILE ?= arm-none-eabi-

CC      = $(CROSS_COMPILE)gcc
OBJCOPY = $(CROSS_COMPILE)objcopy
AS      = $(CROSS_COMPILE)as
SIZE	= $(CROSS_COMPILE)size

##### Preprocessor options
FW_VERSION = "{ $(shell git rev-parse HEAD | sed 's/\(..\)/0x\1, /g' | sed -r 's/,\s+$$//') }"

#defines needed when working with the STM peripherals library
DEFS 	= -DGD32F1x0
DEFS   += -DGD32F130_150
#DEFS   += -DSTM32F40_41xxx

DEFS   += -DUSE_STDPERIPH_DRIVER
#DEFS   += -D__ASSEMBLY_
DEFS   += -DIRC8M_VALUE=8000000
DEFS   += -DVERSION=$(FW_VERSION)

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
LFLAGS +="-Wl,-Map=$(APP_NAME).map",--cref
LFLAGS += -nostartfiles
LFLAGS += -Xlinker --gc-sections

BOOT_LFLAGS  = -T$(BOOT_LINKER_DIR)/GD32F150_FLASH.ld
BOOT_LFLAGS +="-Wl,-Map=$(BOOT_NAME).map",--cref
BOOT_LFLAGS += -nostartfiles
BOOT_LFLAGS += -Xlinker --gc-sections

# directories to be searched for header files
INCLUDE = $(addprefix -I,$(INC_DIRS))

BOOT_INCLUDE = $(addprefix -I,$(BOOT_INC_DIRS))

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



# startup file, calls main
ASRC  = startup_gd32f1x0.s

OBJS  = $(SRCS:.c=.o)
OBJS += $(ASRC:.s=.o)

#BOOTLOADER -------------------------------------------------------------------
BOOT_ROOT_DIR		= $(PROJ_ROOT_DIR)/bootloader
BOOT_LINKER_DIR		= $(BOOT_ROOT_DIR)/linker

BOOT_STM_ROOT_LIB  	= $(APP_ROOT_DIR)/GD32F1x0_Firmware_Library
BOOT_STM_SRC_DIR    = $(BOOT_STM_ROOT_LIB)/GD32F1x0_standard_peripheral/Source

BOOT_STM_CMSIS_DIR 	= $(APP_ROOT_DIR)/GD32F1x0_Firmware_Library/CMSIS/GD/GD32F1x0/Source

BOOT_STARTUP_DIR = $(BOOT_ROOT_DIR)/startup

vpath %.c $(BOOT_ROOT_DIR)
vpath %.c $(BOOT_STM_SRC_DIR)
vpath %.c $(BOOT_STM_CMSIS_DIR)

BOOT_INC_DIRS += $(BOOT_STM_ROOT_LIB)/GD32F1x0_standard_peripheral/Include
BOOT_INC_DIRS += $(BOOT_STM_ROOT_LIB)//CMSIS/GD/GD32F1x0/Include
BOOT_INC_DIRS += $(BOOT_ROOT_DIR)
BOOT_INC_DIRS += $(BOOT_STM_ROOT_LIB)/CMSIS

INC_DIRS += $(STM_ROOT_DIR)/GD32F1x0_standard_peripheral/Include
INC_DIRS += $(STM_ROOT_DIR)/CMSIS
INC_DIRS += $(STM_ROOT_DIR)/CMSIS/GD/GD32F1x0/Include
INC_DIRS += $(APP_SRC_DIR)

BOOTSRCS  += boot_main.c
BOOTSRCS  += boot_i2c.c
BOOTSRCS  += flash.c
BOOTSRCS  += boot_gd32f1x0_it.c
BOOTSRCS  += boot_system_gd32f1x0.c
BOOTSRCS  += boot_led_driver.c
BOOTSRCS  += delay.c
BOOTSRCS  += power_control.c
BOOTSRCS  += debug_serial.c
BOOTSRCS  += eeprom.c
BOOTSRCS  += bootloader.c

BOOTSRCS  += gd32f1x0_misc.c
BOOTSRCS  += gd32f1x0_rcu.c
BOOTSRCS  += gd32f1x0_gpio.c
BOOTSRCS  += gd32f1x0_usart.c
BOOTSRCS  += gd32f1x0_timer.c
BOOTSRCS  += gd32f1x0_spi.c
BOOTSRCS  += gd32f1x0_i2c.c
BOOTSRCS  += gd32f1x0_fmc.c

BOOTASRC  = $(BOOT_STARTUP_DIR)/boot_startup_gd32f1x0.s

BOOT_OBJS  = $(BOOTSRCS:.c=.o)
BOOT_OBJS += $(BOOTASRC:.s=.o)

################################################################################
#                         SIZE OF OUTPUT                                       #
################################################################################
APP_ELFSIZE = $(SIZE) -d $(APP_NAME).elf

app_buildsize: $(APP_NAME).elf
	@echo Program Size: 
	$(APP_ELFSIZE)

BOOT_ELFSIZE = $(SIZE) -d $(BOOT_NAME).elf

boot_buildsize: $(BOOT_NAME).elf
	@echo Program Size: 
	$(BOOT_ELFSIZE)

################################################################################
#                         SETUP TARGETS                                        #
################################################################################

.PHONY: app

all: app boot

app: $(APP_NAME).elf app_buildsize

boot: $(BOOT_NAME).elf boot_buildsize
						
$(APP_NAME).elf: $(OBJS)
	@echo "[Linking    ]  $@"	
	@$(CC) $(CFLAGS) $(LFLAGS) $(INCLUDE) $^ -o $@
	@$(OBJCOPY) -O ihex $(APP_NAME).elf   $(APP_NAME).hex
	@$(OBJCOPY) -O binary $(APP_NAME).elf $(APP_NAME).bin

$(BOOT_NAME).elf: $(BOOT_OBJS)
	@echo "[Linking    ]  $@"	
	@$(CC) $(CFLAGS) $(BOOT_LFLAGS) $(BOOT_INCLUDE) $^ -o $@
	@$(OBJCOPY) -O ihex $(BOOT_NAME).elf   $(BOOT_NAME).hex
	@$(OBJCOPY) -O binary $(BOOT_NAME).elf $(BOOT_NAME).bin

%.o : %.c
	@echo "[Compiling  ]  $^"
	$(CC) -c $(DEFS) $(CFLAGS) $(INCLUDE) $(BOOT_INCLUDE) $< -o $@

%.o : %.s 
	@echo "[Assembling ]" $^
	@$(AS) $(AFLAGS) $< -o $@

clean:
	rm -r -f *.o $(APP_NAME).elf $(APP_NAME).hex $(APP_NAME).bin $(APP_NAME).map $(BOOT_NAME).elf $(BOOT_NAME).hex $(BOOT_NAME).bin $(BOOT_NAME).map

cleanapp:
	rm -r -f *.o $(APP_NAME).elf $(APP_NAME).hex $(APP_NAME).bin $(APP_NAME).map
cleanboot:
	rm -r -f *.o $(BOOT_NAME).elf $(BOOT_NAME).hex $(BOOT_NAME).bin $(BOOT_NAME).map

#********************************
# generating of the dependencies
dep:	
	$(CC) $(DEFS) $(CFLAGS) $(INCLUDE) -MM $(PROJ_ROOT_DIR)/*.c $(APP_SRC_DIR)/*.c > dep.list

## insert generated dependencies
-include dep.list 
#*******************************

