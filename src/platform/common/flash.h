#ifndef __FLASH_H
#define __FLASH_H

#include "svc.h"

typedef enum {
	FLASH_OP_NONE = 0,
	FLASH_OP_ERASE,
	FLASH_OP_WRITE,
} flash_op_type_t;

typedef void (*flash_callback_t)(bool success, void *priv);

void flash_init(void);
SYSCALL(flash_init)

void flash_async_erase(uint32_t start, uint16_t len, flash_callback_t callback,
		       void *priv);
void flash_async_write(uint32_t dst, const uint8_t *src, uint16_t len,
		       flash_callback_t callback, void *priv);

#endif /* __FLASH_H */
