#ifndef CRC32_PLAT_H
#define CRC32_PLAT_H

#include "stm32f0xx.h"
#include "compiler.h"
#include "cpu.h"

#define CRC_FREE_DATA_REG	(CRC->IDR)

static inline uint32_t crc32_plat(uint32_t init, const void *src, uint16_t len)
{
	CRC->INIT = init;
	CRC->CR = CRC_CR_RESET;

	while (len > 0) {
		CRC->DR = get_unaligned32(src);
		src += 4;
		len -= 4;
	}

	return CRC->DR;
}

static inline void crc32_enable(void)
{
	RCC->AHBENR |= RCC_AHBENR_CRCEN;
}

#endif /* CRC32_PLAT_H */
