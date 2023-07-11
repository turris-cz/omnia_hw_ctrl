#ifndef CLOCK_H
#define CLOCK_H

#include "cpu.h"
#include "svc.h"
#include "debug.h"

static __force_inline void clk_config(AIPS_Slot_Type periph, bool on)
{
	switch (periph) {
#define _CLK_CFG(_p, _r)						\
	case _p##_Slot:							\
		BME_BITFIELD(SIM_SCGC##_r, SIM_SCGC##_r##_##_p) =	\
			on ? SIM_SCGC##_r##_##_p : 0;			\
		break;

	_CLK_CFG(I2C0, 4)
	_CLK_CFG(LPUART0, 5)
	_CLK_CFG(TPM0, 6)
	_CLK_CFG(TPM1, 6)
	_CLK_CFG(TPM2, 6)
	_CLK_CFG(SPI1, 6)
	_CLK_CFG(CRC, 6)
#undef _CLK_CFG
	default:
		debug("Invalid peripheral %u for clock config!\n", periph);
	}
}
SYSCALL(clk_config, AIPS_Slot_Type, bool)

#endif /* CLOCK_H */
