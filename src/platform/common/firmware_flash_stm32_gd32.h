#ifndef FIRMWARE_FLASH_STM32_GD32_H
#define FIRMWARE_FLASH_STM32_GD32_H

#include "firmware_flash_common.h"
#include "flash.h"
#include "debug.h"

static __maybe_unused const void *firmware_buffer = (const void *)FIRMWARE_BEGIN;

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

#endif /* FIRMWARE_FLASH_STM32_GD32_H */
