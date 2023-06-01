SRCS_PLAT_mkl	= $(wildcard src/platform/mkl/*.c)

ISR_VECTOR_LENGTH_mkl	= 0xC0
APP_POS_mkl		= 0x5000
CSUM_POS_mkl		= $(ISR_VECTOR_LENGTH_mkl)
FEAT_POS_mkl		= 0xC8
BOOT_FEAT_POS_mkl	= 0xD4

CPPFLAGS_mkl	= -DMKL81 -DMCU_TYPE=MKL
CPPFLAGS_mkl	+= -DSYS_CORE_FREQ=96000000U
CPPFLAGS_mkl	+= -DAPPLICATION_OFFSET=$(APP_POS_mkl) -DISR_VECTOR_LENGTH=$(ISR_VECTOR_LENGTH_mkl)
CPPFLAGS_mkl	+= -Isrc/platform/mkl
CPPFLAGS_mkl	+= -DPOWEROFF_WAKEUP_ENABLED=1

CFLAGS_mkl	= -mcpu=cortex-m0plus -mthumb -mlittle-endian -masm-syntax-unified

ifeq ($(RAM_BUILD), 1)
	CPPFLAGS_mkl += -DRAM_BUILD=1
else
	CPPFLAGS_mkl += -URAM_BUILD

	# since 0 is a valid address, we need this to avoid array-bounds warnings and null pointer dereference traps
	CFLAGS_mkl += --param=min-pagesize=0 -fno-delete-null-pointer-checks
endif

VARIANTS_mkl = rev32
CPPFLAGS_mkl-rev32 = -DOMNIA_BOARD_REVISION=32 -DUSER_REGULATOR_ENABLED=0

$(eval $(call PlatBuild,mkl))
