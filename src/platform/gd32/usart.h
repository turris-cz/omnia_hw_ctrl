#ifndef USART_H
#define USART_H

#include "gd32f1x0_usart.h"
#include "gd32f1x0_rcu.h"
#include "compiler.h"
#include "gpio.h"

typedef uint8_t usart_nr_t;

#define USART0_PINS_ALT_FN	1
#define USART0_PIN_TX		PIN(A, 9)
#define USART0_PIN_RX		PIN(A, 10)

#define DEBUG_USART		0

static __force_inline uint32_t usart_to_plat(usart_nr_t usart_nr)
{
	switch (usart_nr) {
	case 0: return USART0;
	case 1: return USART1;
	default: unreachable();
	}
}

static __force_inline void usart_clk_config(usart_nr_t usart_nr, bool on)
{
	switch (usart_nr) {
#define _USART_CLK_CFG(_bus, _n)							\
	case _n:									\
		if (on)									\
			RCU_ ## _bus ## EN |= RCU_ ## _bus ## EN_USART ## _n ## EN;	\
		else									\
			RCU_ ## _bus ## EN &= ~RCU_ ## _bus ## EN_USART ## _n ## EN;	\
		break;
	_USART_CLK_CFG(APB2, 0)
	_USART_CLK_CFG(APB1, 1)
#undef _USART_CLK_CFG
	default:
		unreachable();
	}
}

static __force_inline void usart_reset(usart_nr_t usart_nr)
{
	switch (usart_nr) {
#define _USART_RESET(_bus, _n)								\
	case _n:									\
		RCU_ ## _bus ## RST |= RCU_ ## _bus ## RST_USART ## _n ## RST;		\
		RCU_ ## _bus ## RST &= ~RCU_ ## _bus ## RST_USART ## _n ## RST;		\
		break;
	_USART_RESET(APB2, 0)
	_USART_RESET(APB1, 1)
#undef _USART_RESET
	default:
		unreachable();
	}
}

static __force_inline void usart_init_pins(usart_nr_t usart_nr)
{
	compiletime_assert(usart_nr == DEBUG_USART,
			   "Invalid USART peripheral used");

	gpio_init_alts(USART0_PINS_ALT_FN, pin_pushpull, pin_spd_3, pin_pullup,
		       USART0_PIN_RX, USART0_PIN_TX);
}

static inline void usart_init(usart_nr_t usart_nr, uint32_t baud_rate)
{
	uint32_t usart = usart_to_plat(usart_nr);
	uint32_t uclk, udiv;

	usart_clk_config(usart_nr, false);
	usart_reset(usart_nr);
	usart_clk_config(usart_nr, true);

	usart_init_pins(usart_nr);

	/* uclk = rcu_clock_freq_get(usart_nr == 0 ? CK_USART : CK_APB1); */
	uclk = 72000000;
	udiv = (uclk + baud_rate / 2) / baud_rate;
	USART_BAUD(usart) = (udiv & USART_BAUD_INTDIV) |
			    (udiv & USART_BAUD_FRADIV);
	USART_CTL0(usart) = USART_PM_NONE | USART_WL_8BIT | USART_CTL0_TEN;
	USART_CTL1(usart) = USART_STB_1BIT;
	USART_CTL0(usart) |= USART_CTL0_UEN;
}

static __force_inline void usart_tx(usart_nr_t usart_nr, uint8_t data)
{
	uint32_t usart = usart_to_plat(usart_nr);

	while (!(USART_STAT(usart) & USART_STAT_TBE))
		;

	USART_TDATA(usart) = data;
}

static __force_inline bool usart_is_tx_complete(usart_nr_t usart_nr)
{
	return !!(USART_STAT(usart_to_plat(usart_nr)) & USART_STAT_TC);
}

#endif /* USART_H */
