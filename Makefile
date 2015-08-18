PROJ_NAME=turris_lite_control

################################################################################
#                   SETUP TOOLS                                                #
################################################################################

CC      = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
AS      = arm-none-eabi-as
SIZE	= arm-none-eabi-size

##### Preprocessor options

#defines needed when working with the STM peripherals library
DEFS 	= -DSTM32F030R8T6
DEFS   += -DSTM32F030X8
DEFS   += -DUSE_STDPERIPH_DRIVER
#DEFS   += -DUSE_FULL_ASSERT
DEFS   += -D__ASSEMBLY_

##### Assembler options

AFLAGS  = -mcpu=cortex-m0 
AFLAGS += -mthumb
AFLAGS += -mlittle-endian

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
LFLAGS  = -T$(LINKER_DIR)/STM32F0308_FLASH.ld
LFLAGS +="-Wl,-Map=$(PROJ_NAME).map",--cref
LFLAGS += -nostartfiles
LFLAGS += -Xlinker --gc-sections

# directories to be searched for header files
INCLUDE = $(addprefix -I,$(INC_DIRS))

################################################################################
#                   SOURCE FILES DIRECTORIES                                   #
################################################################################
PROJ_ROOT_DIR	= src
STM_ROOT_LIB    = $(PROJ_ROOT_DIR)/stm_lib/stm32f0xx_stdperiph_driver
APP_SRC_DIR		= $(PROJ_ROOT_DIR)/app
STM_SRC_DIR     = $(STM_ROOT_LIB)/src
STM_CMSIS_DIR 	= $(PROJ_ROOT_DIR)/stm_lib/cmsis_boot
STM_STARTUP_DIR = $(STM_CMSIS_DIR)/startup

LINKER_DIR = $(PROJ_ROOT_DIR)/linker

vpath %.c $(PROJ_ROOT_DIR)
vpath %.c $(APP_SRC_DIR)
vpath %.c $(STM_SRC_DIR)
vpath %.c $(STM_CMSIS_DIR)
vpath %.s $(STM_STARTUP_DIR)

################################################################################
#                   HEADER FILES DIRECTORIES                                   #
################################################################################

# The header files we use are located here
INC_DIRS += $(STM_ROOT_LIB)/inc
INC_DIRS += $(STM_CMSIS_DIR)
INC_DIRS += $(APP_SRC_DIR)
INC_DIRS += $(PROJ_ROOT_DIR)/stm_lib/cmsis_core

################################################################################
#                   SOURCE FILES TO COMPILE                                    #
################################################################################
SRCS  += main.c
SRCS  += system_stm32f0xx.c
SRCS  += stm32f0xx_it.c
SRCS  += debounce.c
SRCS  += led_driver.c
SRCS  += delay.c
SRCS  += power_control.c
SRCS  += msata_pci.c
SRCS  += wan_lan_pci_status.c
SRCS  += slave_i2c_device.c

################# STM LIB ##########################
SRCS  += stm32f0xx_rcc.c
SRCS  += stm32f0xx_gpio.c
SRCS  += stm32f0xx_tim.c
SRCS  += stm32f0xx_dma.c
SRCS  += stm32f0xx_dbgmcu.c
SRCS  += stm32f0xx_exti.c
SRCS  += stm32f0xx_i2c.c
SRCS  += stm32f0xx_syscfg.c
SRCS  += stm32f0xx_misc.c
SRCS  += stm32f0xx_spi.c
SRCS  += stm32f0xx_flash.c
SRCS  += stm32f0xx_usart.c

# startup file, calls main
ASRC  = startup_stm32f030x8.s

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

