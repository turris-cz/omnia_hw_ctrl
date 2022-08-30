#ifndef SPI_H
#define SPI_H

#include "stm32f0xx_spi.h"
#include "compiler.h"

typedef uint8_t spi_nr_t;

#define LED_SPI		1

static __force_inline SPI_TypeDef *spi_to_plat(spi_nr_t spi_nr)
{
	switch (spi_nr) {
	case 1: return SPI1;
	case 2: return SPI2;
	default: unreachable();
	}
}

static __force_inline void spi_clk_config(spi_nr_t spi_nr, bool on)
{
	switch (spi_nr) {
#define _SPI_CLK_CFG(_bus, _n)								\
	case _n:									\
		if (on)									\
			RCC->_bus ## ENR |= RCC_ ## _bus ## ENR_SPI ## _n ## EN;	\
		else									\
			RCC->_bus ## ENR &= ~RCC_ ## _bus ## ENR_SPI ## _n ## EN;	\
		break;
	_SPI_CLK_CFG(APB2, 1)
	_SPI_CLK_CFG(APB1, 2)
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
		RCC->_bus ## RSTR |= RCC_ ## _bus ## RSTR_SPI ## _n ## RST;		\
		RCC->_bus ## RSTR &= ~RCC_ ## _bus ## RSTR_SPI ## _n ## RST;		\
		break;
	_SPI_RESET(APB2, 1)
	_SPI_RESET(APB1, 2)
#undef _SPI_RESET
	default:
		unreachable();
	}
}

static inline void spi_init(spi_nr_t spi_nr)
{
	SPI_TypeDef *spi = spi_to_plat(spi_nr);

	spi_clk_config(spi_nr, false);
	spi_reset(spi_nr);
	spi_clk_config(spi_nr, true);

	spi->CR1 = SPI_Direction_1Line_Tx | SPI_FirstBit_MSB | SPI_CPOL_Low |
		   SPI_CPHA_1Edge | SPI_NSS_Soft | SPI_BaudRatePrescaler_2 |
		   SPI_Mode_Master;
	spi->CR2 = SPI_DataSize_16b;
	spi->CR1 |= SPI_CR1_SPE;
}

static __force_inline void spi_send16(spi_nr_t spi_nr, uint16_t data)
{
	spi_to_plat(spi_nr)->DR = data;
}

static __force_inline bool spi_is_busy(spi_nr_t spi_nr)
{
	return !!(spi_to_plat(spi_nr)->SR & SPI_SR_BSY);
}

#endif /* SPI_H */
