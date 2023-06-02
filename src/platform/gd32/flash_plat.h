#ifndef __FLASH_PLAT_H
#define __FLASH_PLAT_H

#include "gd32f1x0.h"
#include "gd32f1x0_fmc.h"
#include "flash.h"
#include "cpu.h"

static inline void flash_plat_init(void)
{
	/* Clear all FLASH flags */
	FMC_STAT = FMC_STAT_ENDF | FMC_STAT_WPERR | FMC_STAT_PGERR |
		   FMC_STAT_BUSY;

	nvic_enable_irq_with_prio(FMC_IRQn, 3);
}

static inline uint32_t flash_plat_get_and_clear_status(void)
{
	uint32_t stat = FMC_STAT;

	FMC_STAT = stat;

	return stat;
}

static inline bool flash_plat_status_okay(uint32_t stat)
{
	return stat & FMC_STAT_ENDF;
}

static inline bool flash_plat_status_error(uint32_t stat)
{
	return stat & (FMC_STAT_WPERR | FMC_STAT_PGERR);
}

static inline uint32_t flash_plat_op_bits(flash_op_type_t type)
{
	switch (type) {
	case FLASH_OP_ERASE:
		return FMC_CTL_PER | FMC_CTL_ENDIE | FMC_CTL_ERRIE;
	case FLASH_OP_WRITE:
		return FMC_CTL_PG | FMC_CTL_ENDIE | FMC_CTL_ERRIE;
	default:
		unreachable();
	}
}

static inline void flash_plat_op_begin(flash_op_type_t type)
{
	/* unlock flash if locked */
	if (FMC_CTL & FMC_CTL_LK) {
		FMC_KEY = UNLOCK_KEY0;
		FMC_KEY = UNLOCK_KEY1;
	}

	/* begin operation */
	FMC_CTL |= flash_plat_op_bits(type);
}

static inline void flash_plat_op_end(flash_op_type_t type)
{
	/* end operation */
	FMC_CTL &= ~flash_plat_op_bits(type);

	/* lock flash */
	FMC_CTL |= FMC_CTL_LK;
}

static inline void flash_plat_erase_next(uint32_t addr)
{
	FMC_ADDR = addr;
	FMC_CTL |= FMC_CTL_START;
}

static inline void flash_plat_write_next(uint32_t *addr, const uint8_t **src)
{
	*(volatile uint32_t *)(*addr) = ((*src)[3] << 24) | ((*src)[2] << 16) |
					((*src)[1] << 8) | (*src)[0];
	*addr += 4;
	*src += 4;
}

#endif /* __FLASH_PLAT_H */
