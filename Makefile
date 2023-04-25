CROSS_COMPILE	?= arm-none-eabi-

CC		= $(CROSS_COMPILE)gcc
CPP		= $(CROSS_COMPILE)gcc -E
OBJCOPY		= $(CROSS_COMPILE)objcopy
OBJDUMP		= $(CROSS_COMPILE)objdump

HOSTCC		= gcc
HOSTCFLAGS	= -O2

CFLAGS		= -ggdb -Os -fno-pie -ffreestanding -Wall -Wextra -Warray-bounds -nostdlib -ffunction-sections -fdata-sections
CPPFLAGS	= -DVERSION="{ $(shell git rev-parse HEAD | sed 's/\(..\)/0x\1, /g' | sed -r 's/,\s+$$//') }"
CPPFLAGS	+= -Isrc/include -Isrc/lib -Isrc/drivers -Isrc/application -Isrc/platform/common
CPPFLAGS_app	= -DBOOTLOADER_BUILD=0
CPPFLAGS_boot	= -DBOOTLOADER_BUILD=1

LDSCRIPT_app	= src/application/application.lds
LDSCRIPT_boot	= src/bootloader/bootloader.lds
LDFLAGS		= -T$(LDSCRIPT) -Wl,-Map=$(LDMAP),--cref -nostartfiles -no-pie -Xlinker --gc-sections

SRCS_DEBUG	= src/drivers/debug.c
SRCS_COMMON	= $(filter-out $(SRCS_DEBUG),$(wildcard src/drivers/*.c))
SRCS_COMMON	+= $(wildcard src/lib/*.c)
SRCS_COMMON	+= $(wildcard src/platform/common/*.c)
SRCS_APP	= $(wildcard src/application/*.c)
SRCS_BOOT	= $(wildcard src/bootloader/*.c)

HOSTTOOLS = crc32tool genflashimg stm32tool

ifeq ($(DBG_ENABLE), 1)
	SRCS_COMMON += $(SRCS_DEBUG)
	CPPFLAGS += -DDBG_ENABLE=1
	DEBUG_INFO_PRINT = \
		@echo -e "\n======================================================="; \
		echo "Built with debug output enabled on MCU's UART pins!"; \
		echo "MiniPCIe/mSATA card detection and PCIe1 PLED won't work"; \
		echo -e "=======================================================\n"
	ifdef DBG_BAUDRATE
		ifeq ($(shell echo "$(DBG_BAUDRATE)" | grep -qe "[^0-9]" && echo err),err)
			CPPFLAGS += $(error Wrong value for DBG_BAUDRATE: $(DBG_BAUDRATE))
		else
			CPPFLAGS += -DDBG_BAUDRATE=$(DBG_BAUDRATE)U
		endif
	else
		CPPFLAGS += -DDBG_BAUDRATE=115200U
	endif
else
	CPPFLAGS += -DDBG_ENABLE=0
	DEBUG_INFO_PRINT =
endif

ifeq ($(V), 1)
	Q =
	echo =
else
	Q = @
	echo = @printf "%26s %-4s [%-8s]  %s\n" "$(1)" "$(2)" "$(3)" "$(4)"
endif

.PHONY: all app boot clean

all: app boot tools/stm32tool

app:
	$(DEBUG_INFO_PRINT)
boot:
	$(DEBUG_INFO_PRINT)
clean:
	rm -rf $(addprefix tools/,$(HOSTTOOLS))

define HostToolRule
  tools/$(1): tools/$(1).c
	$(call echo,,,HOSTCC,$$<)
	$(Q)$(HOSTCC) $(HOSTCFLAGS) -o $$@ $$<
endef

$(foreach tool,$(HOSTTOOLS),$(eval $(call HostToolRule,$(tool))))

define CompileRule
  build.$(1)/$(2)/$(3)%.o: $(3)%.c
	$(call echo,$(1),$(2),CC,$$^)
	$(Q)mkdir -p $$(@D) && $$(CC) -c $$(CPPFLAGS) $$(CPPFLAGS_$(1)) $$(CFLAGS) $$(CFLAGS_$(1)) $$< -o $$@
endef

define LinkScriptRule
  build.$(1)/$(2)/$(3): $(3)
	$(call echo,$(1),$(2),GENLDS,$$^)
	$(Q)mkdir -p $$(@D) && $$(CPP) $$(CPPFLAGS) $$(CPPFLAGS_$(1)) -D__ASSEMBLY__ -x assembler-with-cpp -std=c99 -P -o $$@ $$<
endef

define PlatBuildVariant
  SRCS_APP_$(1) = $$(SRCS_COMMON) $$(SRCS_APP) $$(SRCS_PLAT_$(1))
  SRCS_BOOT_$(1) = $$(SRCS_COMMON) $$(SRCS_BOOT) $$(SRCS_PLAT_$(1))
  OBJS_APP_$(1) = $$(addprefix build.$(1)/app/,$$(SRCS_APP_$(1):.c=.o))
  OBJS_BOOT_$(1) = $$(addprefix build.$(1)/boot/,$$(SRCS_BOOT_$(1):.c=.o))
  LDSCRIPT_app_$(1) = build.$(1)/app/$$(LDSCRIPT_app)
  LDSCRIPT_boot_$(1) = build.$(1)/boot/$$(LDSCRIPT_boot)

  .PHONY: $(1) app_$(1) boot_$(1) clean_$(1)

  $(1): app_$(1) boot_$(1) $(1).flash.bin
  app_$(1): $(1).app.bin build.$(1)/app.hex build.$(1)/app.dis
  boot_$(1): $(1).boot.bin build.$(1)/boot.hex build.$(1)/boot.dis

  $(1).flash.bin: $(1).app.bin $(1).boot.bin tools/genflashimg
	$(call echo,$(1),,GENFLASH,$$@)
	$(Q)tools/genflashimg $(1).boot.bin $(1).app.bin $$(APP_POS_$(1)) >$$@
	$(Q)chmod +x $$@

  clean_$(1):
	rm -rf build.$(1) $(1).flash.bin $(1).app.bin $(1).boot.bin

  build.$(1)/%.hex: $(1).%.bin
	$(call echo,$(1),$$*,HEX,$$@)
	$(Q)$$(OBJCOPY) -I binary -O ihex $$< $$@

  build.$(1)/%.dis: build.$(1)/%.elf
	$(call echo,$(1),$$*,DISASM,$$@)
	$(Q)$$(OBJDUMP) -D -S $$< >$$@

  $(1).app.bin: build.$(1)/app.bin.nocrc tools/crc32tool
	$(call echo,$(1),app,CRC32,$$@)
	$(Q)tools/crc32tool $$(CSUM_POS_$(1)) $$(FEAT_POS_$(1)) $$< >$$@
	$(Q)chmod +x $$@

  build.$(1)/app.bin.nocrc: build.$(1)/app.elf
	$(call echo,$(1),app,BIN,$$@)
	$(Q)$$(OBJCOPY) -O binary $$< $$@

  $$(eval $$(call LinkScriptRule,$(1),app,$$(LDSCRIPT_app)))

  build.$(1)/app.elf: CPPFLAGS += $$(CPPFLAGS_app)
  build.$(1)/app.elf: LDSCRIPT = $$(LDSCRIPT_app_$(1))
  build.$(1)/app.elf: LDMAP = build.$(1)/app.map
  build.$(1)/app.elf: $$(OBJS_APP_$(1)) $$(LDSCRIPT_app_$(1))
	$(call echo,$(1),app,LD,$$@)
	$(Q)$$(CC) $$(CFLAGS) $$(CFLAGS_$(1)) $$(LDFLAGS) $$(OBJS_APP_$(1)) -o $$@

  $(1).boot.bin: build.$(1)/boot.bin.nocrc tools/crc32tool
	$(call echo,$(1),boot,CRC32,$$@)
	$(Q)tools/crc32tool ignore $$(BOOT_FEAT_POS_$(1)) $$< >$$@
	$(Q)chmod +x $$@

  build.$(1)/boot.bin.nocrc: build.$(1)/boot.elf
	$(call echo,$(1),boot,BIN,$$@)
	$(Q)$$(OBJCOPY) -O binary $$< $$@

  $$(eval $$(call LinkScriptRule,$(1),boot,$$(LDSCRIPT_boot)))

  build.$(1)/boot.elf: CPPFLAGS += $$(CPPFLAGS_boot)
  build.$(1)/boot.elf: LDSCRIPT = $$(LDSCRIPT_boot_$(1))
  build.$(1)/boot.elf: LDMAP = build.$(1)/boot.map
  build.$(1)/boot.elf: $$(OBJS_BOOT_$(1)) $$(LDSCRIPT_boot_$(1))
	$(call echo,$(1),boot,LD,$$@)
	$(Q)$$(CC) $$(CFLAGS_$(1)) $$(LDFLAGS) $$(OBJS_BOOT_$(1)) -o $$@

  $$(foreach dir,$$(sort $$(dir $$(SRCS_APP_$(1)))),$$(eval $$(call CompileRule,$(1),app,$$(dir))))
  $$(foreach dir,$$(sort $$(dir $$(SRCS_BOOT_$(1)))),$$(eval $$(call CompileRule,$(1),boot,$$(dir))))

  all: $(1)
  app: app_$(1)
  boot: boot_$(1)
  clean: clean_$(1)
endef

define PlatDefVariant
  APP_POS_$(1)-$(2) = $$(APP_POS_$(1))
  CSUM_POS_$(1)-$(2) = $$(CSUM_POS_$(1))
  FEAT_POS_$(1)-$(2) = $$(FEAT_POS_$(1))
  BOOT_FEAT_POS_$(1)-$(2) = $$(BOOT_FEAT_POS_$(1))
  CFLAGS_$(1)-$(2) := $$(CFLAGS_$(1)) $$(CFLAGS_$(1)-$(2))
  CPPFLAGS_$(1)-$(2) := $$(CPPFLAGS_$(1)) $$(CPPFLAGS_$(1)-$(2))
  SRCS_PLAT_$(1)-$(2) := $$(SRCS_PLAT_$(1)) $$(SRCS_PLAT_$(1)-$(2))

  .PHONY: $(1)
  $(1): $(1)-$(2)
endef

define PlatBuild
  $$(foreach variant,$$(VARIANTS_$(1)), \
     $$(eval $$(call PlatDefVariant,$(1),$$(variant))) \
     $$(eval $$(call PlatBuildVariant,$(1)-$$(variant))) \
    )
endef

include Makedefs.*.mk
