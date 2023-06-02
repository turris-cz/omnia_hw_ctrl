SRCS_PLAT_stm32	= $(wildcard src/platform/stm32/*.c)
SRCS_PLAT_stm32	+= src/platform/stm32/stm_lib/cmsis_boot/system_stm32f0xx.c

ISR_VECTOR_LENGTH_stm32	= 0xC0
APP_POS_stm32		= 0x5000
CSUM_POS_stm32		= $(ISR_VECTOR_LENGTH_stm32)
FEAT_POS_stm32		= 0xC8
BOOT_FEAT_POS_stm32	= 0xD4

CPPFLAGS_stm32	= -DSTM32F030X8 -DUSE_STDPERIPH_DRIVER -DMCU_TYPE=STM32
CPPFLAGS_stm32	+= -DSYS_CORE_FREQ=48000000U
CPPFLAGS_stm32	+= -DAPPLICATION_OFFSET=$(APP_POS_stm32) -DISR_VECTOR_LENGTH=$(ISR_VECTOR_LENGTH_stm32)
CPPFLAGS_stm32	+= -Isrc/platform/stm32
CPPFLAGS_stm32	+= -Isrc/platform/stm32/stm_lib/cmsis_boot
CPPFLAGS_stm32	+= -Isrc/platform/stm32/stm_lib/cmsis_core
CPPFLAGS_stm32	+= -Isrc/platform/stm32/stm_lib/stm32f0xx_stdperiph_driver/inc
CPPFLAGS_stm32	+= -DPOWEROFF_WAKEUP_ENABLED=1

CFLAGS_stm32	= -mcpu=cortex-m0 -mthumb -mlittle-endian -masm-syntax-unified

VARIANTS_stm32 = rev23 rev23-user-regulator rev32
CPPFLAGS_stm32-rev23			= -DOMNIA_BOARD_REVISION=23 -DUSER_REGULATOR_ENABLED=0
CPPFLAGS_stm32-rev23-user-regulator	= -DOMNIA_BOARD_REVISION=23 -DUSER_REGULATOR_ENABLED=1
CPPFLAGS_stm32-rev32			= -DOMNIA_BOARD_REVISION=32 -DUSER_REGULATOR_ENABLED=0

$(eval $(call PlatBuild,stm32))
