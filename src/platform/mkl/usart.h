#ifndef USART_H
#define USART_H

#include "cpu.h"
#include "clock.h"
#include "gpio.h"

typedef uint8_t usart_nr_t;

#define USART1_PINS_ALT_FN	2
#define USART1_PIN_TX		PIN(A, 2)
#define USART1_PIN_RX		PIN(A, 1)

#define DEBUG_USART		0

static __force_inline void usart_clk_config(usart_nr_t usart_nr, bool on)
{
	switch (usart_nr) {
	case 0: sys_clk_config(LPUART0_Slot, on); break;
	case 1: sys_clk_config(LPUART1_Slot, on); break;
	case 2: sys_clk_config(LPUART2_Slot, on); break;
	default: unreachable();
	}
}

static __force_inline void usart_init_pins(usart_nr_t usart_nr)
{
	compiletime_assert(usart_nr == DEBUG_USART,
			   "Invalid USART peripheral used");

	gpio_init_alts(USART1_PINS_ALT_FN, pin_pushpull, pin_spd_3, pin_pullup,
		       USART1_PIN_RX, USART1_PIN_TX);
}

static inline uint32_t usart_baud_reg(uint32_t baud_rate)
{
	uint32_t clk, best_diff, baud_reg;
	uint16_t best_sbr;
	uint8_t best_osr;

	/* optimize away the rest of the code if constant baud rate 115200 Bd */
	if (baud_rate == 115200)
		return LPUART_BAUD_OSR(13) | LPUART_BAUD_SBR(32);

	clk = 48000000U;
	best_diff = -1U;
	for (uint8_t osr = 32; osr >= 4; --osr) {
		uint32_t diff, actual_baud_rate;
		uint16_t sbr;

		sbr = clk / (baud_rate * osr) ? : 1;
		actual_baud_rate = clk / (sbr * osr);
		diff = ABSDIFF(baud_rate, actual_baud_rate);
		if (diff < best_diff) {
			best_diff = diff;
			best_osr = osr;
			best_sbr = sbr;
		}
	}

	baud_reg = LPUART_BAUD_OSR(best_osr) | LPUART_BAUD_SBR(best_sbr);
	if (best_osr >= 4 && best_osr <= 7)
		baud_reg |= LPUART_BAUD_BOTHEDGE;

	return baud_reg;
}

static inline void usart_init(usart_nr_t usart_nr, uint32_t baud_rate)
{
	usart_clk_config(usart_nr, false);
	usart_clk_config(usart_nr, true);

	usart_init_pins(usart_nr);

	LPUART_CTRL(usart_nr) &= ~(LPUART_CTRL_RE | LPUART_CTRL_TE);
	LPUART_CTRL(usart_nr) = 0;
	LPUART_BAUD(usart_nr) = usart_baud_reg(baud_rate);
	LPUART_FIFO(usart_nr) |= LPUART_FIFO_TXFLUSH | LPUART_FIFO_TXFE;
	LPUART_CTRL(usart_nr) = LPUART_CTRL_TE;
}

static __force_inline void usart_tx(usart_nr_t usart_nr, uint8_t data)
{
	while (!BME_BITFIELD(LPUART_STAT(usart_nr), LPUART_STAT_TDRE))
		nop();

	LPUART_DATA(usart_nr) = data;
}

static __force_inline bool usart_is_tx_complete(usart_nr_t usart_nr)
{
	return BME_BITFIELD(LPUART_STAT(usart_nr), LPUART_STAT_TC);
}

#endif /* USART_H */
