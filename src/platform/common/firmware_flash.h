#ifndef FIRMWARE_FLASH_H
#define FIRMWARE_FLASH_H

#include "compiler.h"
#include "memory_layout.h"
#include "flash.h"
#include "debug.h"

#define FIRMWARE_BEGIN \
	(BOOTLOADER_BUILD ? APPLICATION_BEGIN : BOOTLOADER_BEGIN)
#define FIRMWARE_MAX_SIZE \
	(BOOTLOADER_BUILD ? APPLICATION_MAX_SIZE : BOOTLOADER_MAX_SIZE)

static __maybe_unused const void *firmware_buffer = (const void *)FIRMWARE_BEGIN;

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

static inline void
firmware_flash_start(uint16_t len, flash_callback_t callback, void *priv)
{
	debug("erasing %#010x - %#010x\n", FIRMWARE_BEGIN,
	      FIRMWARE_BEGIN + len - 1);

	flash_async_erase(FIRMWARE_BEGIN, len, callback, priv);
}

static inline void
firmware_flash_continue(uint16_t pos, const uint8_t *src, uint16_t len,
			flash_callback_t callback, void *priv)
{
	debug("programming %#010x - %#010x\n", FIRMWARE_BEGIN + pos,
	      FIRMWARE_BEGIN + pos + len - 1);

	flash_async_write(FIRMWARE_BEGIN + pos, src, len, callback, priv);
}

static inline void
firmware_flash_finish(flash_callback_t callback, void *priv)
{
	/* trivial finish operation for now */
	callback(true, priv);
}

#endif /* FIRMWARE_FLASH_H */
