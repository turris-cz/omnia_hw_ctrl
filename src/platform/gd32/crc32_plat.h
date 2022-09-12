#ifndef CRC32_PLAT_H
#define CRC32_PLAT_H

#include "gd32f1x0_rcu.h"
#include "gd32f1x0_crc.h"
#include "compiler.h"
#include "cpu.h"

static inline uint32_t crc32_plat(uint32_t init, const void *src, uint16_t len)
{
	CRC_IDATA = init;
	CRC_CTL = CRC_CTL_RST;

	while (len > 0) {
		CRC_DATA = get_unaligned32(src);
		src += 4;
		len -= 4;
	}

	return CRC_DATA;
}

static inline void crc32_enable(void)
{
	RCU_AHBEN |= RCU_AHBEN_CRCEN;
}

#endif /* CRC32_PLAT_H */
