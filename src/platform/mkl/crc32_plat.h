#ifndef CRC32_PLAT_H
#define CRC32_PLAT_H

#include "cpu.h"
#include "clock.h"

static inline uint32_t crc32_plat(uint32_t init, const void *src, uint32_t len)
{
	CRC_CTRL = CRC_CTRL_TCRC | CRC_CTRL_WAS;
	CRC_GPOLY = 0x04c11db7;
	CRC_DATA = init;
	CRC_CTRL = CRC_CTRL_TCRC;

	while (len > 0) {
		CRC_DATA = get_unaligned32(src);
		src += 4;
		len -= 4;
	}

	return CRC_DATA;
}

static inline void crc32_enable(void)
{
	sys_clk_config(CRC_Slot, true);
}

#endif /* CRC32_PLAT_H */
