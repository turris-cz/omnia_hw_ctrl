#ifndef FIRMWARE_FLASH_COMMON_H
#define FIRMWARE_FLASH_COMMON_H

#include "compiler.h"
#include "memory_layout.h"

#define FIRMWARE_BEGIN \
	(BOOTLOADER_BUILD ? APPLICATION_BEGIN : BOOTLOADER_BEGIN)
#define FIRMWARE_MAX_SIZE \
	(BOOTLOADER_BUILD ? APPLICATION_MAX_SIZE : BOOTLOADER_MAX_SIZE)
#define FIRMWARE_FEATURES \
	(BOOTLOADER_BUILD ? APPLICATION_FEATURES : BOOTLOADER_FEATURES)
#define FIRMWARE_FEATURES_OFFSET (FIRMWARE_FEATURES - FIRMWARE_BEGIN)

#define FEATURES_MAGIC	0xfea70235

typedef struct {
	uint32_t magic;
	uint16_t features;
	uint8_t status_features;
	uint8_t reserved;
	uint32_t csum;
} features_t;

static inline bool firmware_is_good_stack_addr(uint32_t addr)
{
	if (addr & 3)
		return false;

	return addr > RAM_BEGIN + 0x1000 && addr <= RAM_END;
}

static inline bool firmware_is_good_reset_addr(uint32_t addr)
{
	if (!(addr & 1))
		return false;

	return addr >= FIRMWARE_BEGIN + ISR_VECTOR_LENGTH &&
	       addr < FIRMWARE_BEGIN + FIRMWARE_MAX_SIZE;
}

#endif /* FIRMWARE_FLASH_COMMON_H */
