#ifndef TIMER_H
#define TIMER_H

#include "cpu.h"
#include "gd32f1x0_timer.h"
#include "gd32f1x0_rcu.h"
#include "compiler.h"
#include "debug.h"

typedef uint8_t timer_nr_t;

#define LED_TIMER		2
#define LED_EFFECT_TIMER	5
#define LED_PWM_TIMER		14
#define USB_TIMEOUT_TIMER	16

#define TIMER_PARENT_FREQ	SYS_CORE_FREQ

typedef enum {
	timer_interrupt,
	timer_pwm,
} timer_type_t;

static __force_inline void timer_clk_config(timer_nr_t tim_nr, bool on)
{
	switch (tim_nr) {
#define _TIMER_CLK_CFG(_bus, _n)							\
	case _n:									\
		if (on)									\
			RCU_ ## _bus ## EN |= RCU_ ## _bus ## EN_TIMER ## _n ## EN;	\
		else									\
			RCU_ ## _bus ## EN &= ~RCU_ ## _bus ## EN_TIMER ## _n ## EN;	\
		break;
	_TIMER_CLK_CFG(APB2, 14)
	_TIMER_CLK_CFG(APB2, 15)
	_TIMER_CLK_CFG(APB2, 16)
	_TIMER_CLK_CFG(APB1, 2)
	_TIMER_CLK_CFG(APB1, 5)
	_TIMER_CLK_CFG(APB1, 13)
#undef _TIMER_RCC_CFG
	default:
		unreachable();
	}
}

static __force_inline void timer_reset(timer_nr_t timer_nr)
{
	switch (timer_nr) {
#define _TIMER_RESET(_bus, _n)								\
	case _n:									\
		RCU_ ## _bus ## RST |= RCU_ ## _bus ## RST_TIMER ## _n ## RST;		\
		RCU_ ## _bus ## RST &= ~RCU_ ## _bus ## RST_TIMER ## _n ## RST;		\
		break;
	_TIMER_RESET(APB2, 14)
	_TIMER_RESET(APB2, 15)
	_TIMER_RESET(APB2, 16)
	_TIMER_RESET(APB1, 2)
	_TIMER_RESET(APB1, 5)
	_TIMER_RESET(APB1, 13)
#undef _TIMER_RESET
	default:
		unreachable();
	}
}

static __force_inline uint32_t timer_to_plat(timer_nr_t tim_nr)
{
	switch (tim_nr) {
	case 2: return TIMER2;
	case 5: return TIMER5;
	case 13: return TIMER13;
	case 14: return TIMER14;
	case 15: return TIMER15;
	case 16: return TIMER16;
	default: unreachable();
	}
}

static __force_inline uint8_t timer_irqn(timer_nr_t tim_nr)
{
	switch (tim_nr) {
	case 2: return TIMER2_IRQn;
	case 5: return TIMER5_DAC_IRQn;
	case 13: return TIMER13_IRQn;
	case 14: return TIMER14_IRQn;
	case 15: return TIMER15_IRQn;
	case 16: return TIMER16_IRQn;
	default: unreachable();
	}
}

static __force_inline void timer_init(timer_nr_t tim_nr, timer_type_t type,
				      uint16_t period, uint32_t freq,
				      uint8_t irq_prio)
{
	uint32_t tim = timer_to_plat(tim_nr);

	compiletime_assert(SYS_CORE_FREQ % freq == 0,
			   "Requested frequency unachievable");

	timer_clk_config(tim_nr, 0);
	timer_reset(tim_nr);
	timer_clk_config(tim_nr, 1);

	/* auto reload shadow */
	TIMER_CTL0(tim) = TIMER_CTL0_ARSE;

	TIMER_PSC(tim) = (SYS_CORE_FREQ / freq) - 1;
	TIMER_CAR(tim) = period - 1;

	/* generate an update event */
	TIMER_SWEVG(tim) = TIMER_SWEVG_UPG;

	switch (type) {
	case timer_pwm:
		/* channel 1 mode selection */
		TIMER_CHCTL0(tim) = (TIMER_OC_MODE_PWM0 |
				     TIMER_OC_SHADOW_DISABLE) << 8;
		/* enable channel 1 and set polarity low */
		TIMER_CHCTL2(tim) = TIMER_CHCTL2_CH1EN | TIMER_CHCTL2_CH1P;

		TIMER_CCHP(tim) = TIMER_CCHP_POEN;
		break;

	case timer_interrupt:
		/* enable up interrupt */
		TIMER_DMAINTEN(tim) = TIMER_DMAINTEN_UPIE;

		nvic_enable_irq(timer_irqn(tim_nr), irq_prio);
		break;
	}
}

static __force_inline void timer_deinit(timer_nr_t tim_nr)
{
	timer_reset(tim_nr);
}

static __force_inline void timer_enable(timer_nr_t tim_nr, bool on)
{
	if (on)
		TIMER_CTL0(timer_to_plat(tim_nr)) |= TIMER_CTL0_CEN;
	else
		TIMER_CTL0(timer_to_plat(tim_nr)) &= ~TIMER_CTL0_CEN;
}

static __force_inline bool timer_irq_clear_up(timer_nr_t tim_nr)
{
	uint32_t tim = timer_to_plat(tim_nr);
	bool up = TIMER_INTF(tim) & TIMER_INTF_UPIF;

	if (up)
		TIMER_INTF(tim) = ~TIMER_INTF_UPIF;

	return up;
}

static __force_inline void timer_set_period(timer_nr_t tim_nr, uint16_t period)
{
	TIMER_CAR(timer_to_plat(tim_nr)) = period - 1;
}

static __force_inline void timer_set_freq(timer_nr_t tim_nr, uint32_t freq)
{
	compiletime_assert(SYS_CORE_FREQ % freq == 0,
			   "Requested frequency unachievable");

	TIMER_PSC(timer_to_plat(tim_nr)) = (SYS_CORE_FREQ / freq) - 1;
}

static __force_inline void timer_set_counter(timer_nr_t tim_nr, uint32_t cnt)
{
	TIMER_CNT(timer_to_plat(tim_nr)) = cnt;
}

static __force_inline uint16_t timer_get_counter(timer_nr_t tim_nr)
{
	return TIMER_CNT(timer_to_plat(tim_nr));
}

static __force_inline void timer_set_pulse(timer_nr_t tim_nr, uint32_t pulse)
{
	TIMER_CH1CV(timer_to_plat(tim_nr)) = pulse;
}

#endif /* TIMER_H */
