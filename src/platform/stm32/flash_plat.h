#ifndef __FLASH_PLAT_H
#define __FLASH_PLAT_H

#include "stm32f0xx.h"
#include "flash.h"
#include "cpu.h"

static inline void flash_plat_init(void)
{
	/* Clear all FLASH flags */
	FLASH->SR = FLASH_SR_EOP | FLASH_SR_WRPERR | FLASH_SR_PGERR |
		    FLASH_SR_BSY;

	nvic_enable_irq_with_prio(FLASH_IRQn, 3);
}

static inline uint32_t flash_plat_get_and_clear_status(void)
{
	uint32_t stat = FLASH->SR;

	FLASH->SR = stat;

	return stat;
}

static inline bool flash_plat_status_okay(uint32_t stat)
{
	return stat & FLASH_SR_EOP;
}

static inline bool flash_plat_status_error(uint32_t stat)
{
	return stat & (FLASH_SR_PGERR | FLASH_SR_WRPERR);
}

static inline uint32_t flash_plat_op_bits(flash_op_type_t type)
{
	switch (type) {
	case FLASH_OP_ERASE:
		return FLASH_CR_PER | FLASH_CR_EOPIE | FLASH_CR_ERRIE;
	case FLASH_OP_WRITE:
		return FLASH_CR_PG | FLASH_CR_EOPIE | FLASH_CR_ERRIE;
	default:
		unreachable();
	}
}

static inline void flash_plat_op_begin(flash_op_type_t type)
{
	/* unlock flash if locked */
	if (FLASH->CR & FLASH_CR_LOCK) {
		FLASH->KEYR = FLASH_FKEY1;
		FLASH->KEYR = FLASH_FKEY2;
	}

	/* begin operation */
	FLASH->CR |= flash_plat_op_bits(type);
}

static inline void flash_plat_op_end(flash_op_type_t type)
{
	/* end operation */
	FLASH->CR &= ~flash_plat_op_bits(type);

	/* lock flash */
	FLASH->CR |= FLASH_CR_LOCK;
}

static inline void flash_plat_erase_next(uint32_t addr)
{
	FLASH->AR = addr;
	FLASH->CR |= FLASH_CR_STRT;
}

static inline void flash_plat_write_next(uint32_t *addr, const uint8_t **src)
{
	*(volatile uint16_t *)(*addr) = get_unaligned16(*src);
	*addr += 2;
	*src += 2;
}

#endif /* __FLASH_PLAT_H */
