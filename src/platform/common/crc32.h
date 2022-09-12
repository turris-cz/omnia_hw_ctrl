#ifndef CRC32_H
#define CRC32_H

#include "crc32_plat.h"
#include "compiler.h"
#include "cpu.h"

static inline bool crc32(uint32_t *res, uint32_t init,
			 const void *src, uint16_t len)
{
	/* we don't support unaligned length yet */
	if (len % 4)
		return false;

	*res = init;

	while (len > 0) {
		uint16_t now = MIN(len, 128);

		disable_irq();
		*res = crc32_plat(*res, src, now);
		enable_irq();
		src += now;
		len -= now;
	}

	return true;
}

#endif /* CRC32_H */
