#ifndef CRC32_H
#define CRC32_H

#include "stm32f0xx.h"
#include "compiler.h"

static inline bool crc32(uint32_t *res, uint32_t init,
			 const uint32_t *addr, uint16_t len)
{
	/* we don't support unaligned length yet */
	if (len % 4)
		return false;

	RCC->AHBENR |= RCC_AHBENR_CRCEN;

	CRC->INIT = init;
	CRC->CR = CRC_CR_RESET;
	while (len > 0) {
		CRC->DR = *addr++;
		len -= 4;
	}

	*res = CRC->DR;

	RCC->AHBENR &= ~RCC_AHBENR_CRCEN;

	return true;
}

#endif /* CRC32_H */
