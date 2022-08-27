#ifndef CRC32_H
#define CRC32_H

#include "gd32f1x0_rcu.h"
#include "gd32f1x0_crc.h"
#include "compiler.h"

static inline bool crc32(uint32_t *res, uint32_t init,
			 const uint32_t *addr, uint16_t len)
{
	/* we don't support unaligned length yet */
	if (len % 4)
		return 0;

	RCU_AHBEN |= RCU_AHBEN_CRCEN;

	CRC_IDATA = init;
	CRC_CTL = CRC_CTL_RST;
	while (len > 0) {
		CRC_DATA = *addr++;
		len -= 4;
	}

	*res = CRC_DATA;

	RCU_AHBEN &= ~RCU_AHBEN_CRCEN;

	return 1;
}

#endif /* CRC32_H */