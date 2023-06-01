#ifndef SPI_H
#define SPI_H

#include "cpu.h"

typedef uint8_t spi_nr_t;

#define LED_SPI		1

static __force_inline void spi_clk_config(spi_nr_t spi_nr, bool on)
{
	switch (spi_nr) {
#define _SPI_CLK_CFG(_n)					\
	case _n:						\
		BME_BITFIELD(SIM_SCGC6, SIM_SCGC6_SPI ## _n) =	\
			on ? SIM_SCGC6_SPI ## _n : 0;		\
		break;
	_SPI_CLK_CFG(0)
	_SPI_CLK_CFG(1)
#undef _SPI_CLK_CFG
	default:
		unreachable();
	}
}

static inline void spi_init(spi_nr_t nr)
{
	spi_clk_config(nr, false);
	spi_clk_config(nr, true);

	BME_BITFIELD(SPI_MCR(nr), SPI_MCR_HALT) = SPI_MCR_HALT;
	SPI_RSER(nr) = 0;
	SPI_MCR(nr) = SPI_MCR_MSTR | SPI_MCR_PCSIS_CS(0) | SPI_MCR_HALT |
		      SPI_MCR_DIS_RXF;
	SPI_SR(nr) = SPI_SR_TFFF | SPI_SR_TCF;
	SPI_CTAR(nr, 0) = SPI_CTAR_FMSZ(0xf);
	BME_BITFIELD(SPI_MCR(nr), SPI_MCR_HALT) = 0;
}

static __force_inline void spi_send16(spi_nr_t nr, uint16_t data)
{
	SPI_PUSHR(nr) = SPI_PUSHR_PCS_CS(0) | data;
}

#endif /* SPI_H */
