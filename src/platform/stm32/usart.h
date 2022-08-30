#ifndef USART_H
#define USART_H

#include "stm32f0xx_usart.h"
#include "compiler.h"
#include "gpio.h"

typedef uint8_t usart_nr_t;

#define USART1_PINS_ALT_FN	1
#define USART1_PIN_TX		PIN(A, 9)
#define USART1_PIN_RX		PIN(A, 10)

#define DEBUG_USART		1

static __force_inline USART_TypeDef *usart_to_plat(usart_nr_t usart_nr)
{
	switch (usart_nr) {
	case 1: return USART1;
	case 2: return USART2;
	default: unreachable();
	}
}

static __force_inline void usart_clk_config(usart_nr_t usart_nr, bool on)
{
	switch (usart_nr) {
#define _USART_CLK_CFG(_bus, _n)							\
	case _n:									\
		if (on)									\
			RCC->_bus ## ENR |= RCC_ ## _bus ## ENR_USART ## _n ## EN;	\
		else									\
			RCC->_bus ## ENR &= ~RCC_ ## _bus ## ENR_USART ## _n ## EN;	\
		break;
	_USART_CLK_CFG(APB2, 1)
	_USART_CLK_CFG(APB1, 2)
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
		RCC->_bus ## RSTR |= RCC_ ## _bus ## RSTR_USART ## _n ## RST;		\
		RCC->_bus ## RSTR &= ~RCC_ ## _bus ## RSTR_USART ## _n ## RST;;		\
		break;
	_USART_RESET(APB2, 1)
	_USART_RESET(APB1, 2)
#undef _USART_RESET
	default:
		unreachable();
	}
}

static __force_inline void usart_init_pins(usart_nr_t usart_nr)
{
	compiletime_assert(usart_nr == DEBUG_USART,
			   "Invalid USART peripheral used");

	gpio_init_alts(USART1_PINS_ALT_FN, pin_pushpull, pin_spd_3, pin_pullup,
		       USART1_PIN_RX, USART1_PIN_TX);
}

static inline void usart_init(usart_nr_t usart_nr, uint32_t baud_rate)
{
	USART_TypeDef *usart = usart_to_plat(usart_nr);
	USART_InitTypeDef init = {
		.USART_BaudRate = baud_rate,
		.USART_WordLength = USART_WordLength_8b,
		.USART_StopBits = USART_StopBits_1,
		.USART_Parity = USART_Parity_No,
		.USART_HardwareFlowControl = USART_HardwareFlowControl_None,
		.USART_Mode = USART_Mode_Rx | USART_Mode_Tx,
	};

	usart_clk_config(usart_nr, 0);
	usart_reset(usart_nr);
	usart_clk_config(usart_nr, 1);

	usart_init_pins(usart_nr);

	USART_Init(usart, &init);
	USART_Cmd(usart, ENABLE);
}

static __force_inline void usart_tx(usart_nr_t usart_nr, uint8_t data)
{
	USART_TypeDef *usart = usart_to_plat(usart_nr);

	while (!(usart->ISR & USART_ISR_TXE))
		;

	usart->TDR = data;
}

static __force_inline bool usart_is_tx_complete(usart_nr_t usart_nr)
{
	return !!(usart_to_plat(usart_nr)->ISR & USART_ISR_TC);
}

#endif /* USART_H */
