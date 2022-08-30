#ifndef SPI_H
#define SPI_H

#include "gd32f1x0_rcu.h"
#include "gd32f1x0_spi.h"
#include "compiler.h"
#include "debug.h"

typedef uint8_t spi_nr_t;

#define LED_SPI		0

static __force_inline uint32_t spi_to_plat(spi_nr_t spi_nr)
{
	switch (spi_nr) {
	case 0: return SPI0;
	case 1: return SPI1;
	default: unreachable();
	}
}

static __force_inline void spi_clk_config(spi_nr_t spi_nr, bool on)
{
	switch (spi_nr) {
#define _SPI_CLK_CFG(_bus, _n)								\
	case _n:									\
		if (on)									\
			RCU_ ## _bus ## EN |= RCU_ ## _bus ## EN_SPI ## _n ## EN;	\
		else									\
			RCU_ ## _bus ## EN &= ~RCU_ ## _bus ## EN_SPI ## _n ## EN;	\
		break;
	_SPI_CLK_CFG(APB2, 0)
	_SPI_CLK_CFG(APB1, 1)
#undef _SPI_CLK_CFG
	default:
		unreachable();
	}
}

static __force_inline void spi_reset(spi_nr_t spi_nr)
{
	switch (spi_nr) {
#define _SPI_RESET(_bus, _n)								\
	case _n:									\
		RCU_ ## _bus ## RST |= RCU_ ## _bus ## RST_SPI ## _n ## RST;		\
		RCU_ ## _bus ## RST &= ~RCU_ ## _bus ## RST_SPI ## _n ## RST;		\
		break;
	_SPI_RESET(APB2, 0)
	_SPI_RESET(APB1, 1)
#undef _SPI_RESET
	default:
		unreachable();
	}
}

static inline void spi_init(spi_nr_t spi_nr)
{
	uint32_t spi = spi_to_plat(spi_nr);

	spi_clk_config(spi_nr, false);
	spi_reset(spi_nr);
	spi_clk_config(spi_nr, true);

	SPI_CTL0(spi) = SPI_CTL0_MSTMOD | SPI_CK_PL_LOW_PH_1EDGE |
			SPI_CTL0_SWNSS | SPI_CTL0_SWNSSEN | SPI_PSC_4 |
			SPI_CTL0_BDEN | SPI_CTL0_BDOEN |
			SPI_ENDIAN_MSB | SPI_CTL0_FF16;

	SPI_CTL0(spi) |= SPI_CTL0_SPIEN;
}

static __force_inline void spi_send16(spi_nr_t spi_nr, uint16_t data)
{
	SPI_DATA(spi_to_plat(spi_nr)) = data;
}

static __force_inline bool spi_is_busy(spi_nr_t spi_nr)
{
	return !!(SPI_STAT(spi_to_plat(spi_nr)) & SPI_STAT_TRANS);
}

#endif /* SPI_H */
