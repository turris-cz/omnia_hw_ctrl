#ifndef FIRMWARE_FLASH_H
#define FIRMWARE_FLASH_H

#include "firmware_flash_common.h"
#include "flash.h"
#include "svc.h"
#include "string.h"

extern uint16_t firmware_flash_size;
extern uint8_t firmware_buffer[FIRMWARE_MAX_SIZE];

static inline void
firmware_flash_start(uint16_t size, flash_callback_t callback, void *priv)
{
	firmware_flash_size = size;
	callback(true, priv);
}

static inline void
firmware_flash_continue(uint16_t pos, const uint8_t *src, uint16_t len,
			flash_callback_t callback, void *priv)
{
	memcpy(&firmware_buffer[pos], src, len);
	callback(true, priv);
}

void plat_firmware_flash_finish(flash_callback_t callback, void *priv);
SYSCALL(plat_firmware_flash_finish, flash_callback_t, void *)

static inline void firmware_flash_finish(flash_callback_t callback, void *priv)
{
	sys_plat_firmware_flash_finish(callback, priv);
}

#endif /* FIRMWARE_FLASH_H */
