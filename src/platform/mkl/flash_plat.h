#ifndef __FLASH_PLAT_H
#define __FLASH_PLAT_H

#include "flash.h"
#include "cpu.h"
#include "memory_layout.h"
#include "debug.h"

#define FLASH_PAGE_SIZE		0x800

static __privileged inline void flash_plat_init(void)
{
	/* Clear all FTFA flags */
	FTFA_FSTAT = FTFA_FSTAT_FPVIOL | FTFA_FSTAT_ACCERR |
		     FTFA_FSTAT_RDCOLERR;

	/* enable INTMUX clock */
	BME_OR(SIM_SCGC6) = SIM_SCGC6_INTMUX0;

	/* reset INTMUX channel 0 and mux FTFA IRQ */
	INTMUX_CHn_CSR(0) = INTMUX_CHn_CSR_RST;
	INTMUX_CHn_IER(0) = BIT(FTFA_IRQn);

	/* enable INTMUX channel 0 interrupt; flash irq must preempt SVC */
	nvic_enable_irq_with_prio(INTMUX0_0_IRQn, 2);
}

static __privileged inline uint32_t flash_plat_get_and_clear_status(void)
{
	uint32_t st = FTFA_FSTAT;

	FTFA_FSTAT = st & (FTFA_FSTAT_FPVIOL | FTFA_FSTAT_ACCERR);

	return st;
}

static __privileged inline bool flash_plat_status_okay(uint32_t stat)
{
	return (stat & (FTFA_FSTAT_CCIF | FTFA_FSTAT_MGSTAT0)) ==
	       FTFA_FSTAT_CCIF;
}

static __privileged inline bool flash_plat_status_error(uint32_t stat)
{
	return stat & FTFA_FSTAT_MGSTAT0;
}

static __privileged inline void flash_plat_op_begin(flash_op_type_t)
{
	/* Clear all FTFA flags */
	FTFA_FSTAT = FTFA_FSTAT_FPVIOL | FTFA_FSTAT_ACCERR |
		     FTFA_FSTAT_RDCOLERR;
}

static __privileged inline void flash_plat_op_end(flash_op_type_t)
{
	/* disable command completed interrupt */
	BME_AND(FTFA_FCNFG) = (uint8_t)~FTFA_FCNFG_CCIE;

	/*
	 * clear flash controller cache
	 * (don't use BME_OR, since we can't use BME for MCM access)
	 */
	MCM_PLACR |= MCM_PLACR_CFCC;
}

static __privileged inline void flash_plat_set_cmd_addr(uint8_t cmd,
							uint32_t addr)
{
	FTFA_FCCOB_LWORD(0) = (cmd << 24) | (addr & 0xffffff);
}

static __privileged inline void flash_plat_cmd_exec(void)
{
	/* start operation */
	FTFA_FSTAT = FTFA_FSTAT_CCIF;

	/* enable command completed interrupt */
	BME_OR(FTFA_FCNFG) = FTFA_FCNFG_CCIE;
}

static __privileged inline void flash_plat_erase_next(uint32_t addr)
{
	/* set erase command and address */
	flash_plat_set_cmd_addr(FTFA_FCCOB_FCMD_ERS_SECT, addr);

	/* execute the command */
	flash_plat_cmd_exec();
}

static __privileged inline void flash_plat_write_next(uint32_t *addr,
						      const uint8_t **src)
{
	/* set address */
	flash_plat_set_cmd_addr(FTFA_FCCOB_FCMD_PRG_LWRD, *addr);
	*addr += 4;

	/* set data */
	FTFA_FCCOB_LWORD(1) = get_unaligned32(*src);
	*src += 4;

	/* execute the command */
	flash_plat_cmd_exec();
}

#endif /* __FLASH_PLAT_H */
