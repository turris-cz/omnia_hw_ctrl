#ifndef TIMER_H
#define TIMER_H

#include "timer_common_stm32_gd32.h"

#define LED_TIMER		3
#define LED_PATTERN_TIMER	6
#define LED_PWM_TIMER		15
#define USB_TIMEOUT_TIMER	17

static __force_inline void timer_clk_config(timer_nr_t tim_nr, bool on)
{
	switch (tim_nr) {
#define _TIMER_CLK_CFG(_bus, _n)							\
	case _n:									\
		if (on)									\
			RCC->_bus ## ENR |= RCC_ ## _bus ## ENR_TIM ## _n ## EN;	\
		else									\
			RCC->_bus ## ENR &= ~RCC_ ## _bus ## ENR_TIM ## _n ## EN;	\
		break;
	_TIMER_CLK_CFG(APB2, 15)
	_TIMER_CLK_CFG(APB2, 16)
	_TIMER_CLK_CFG(APB2, 17)
	_TIMER_CLK_CFG(APB1, 3)
	_TIMER_CLK_CFG(APB1, 6)
	_TIMER_CLK_CFG(APB1, 14)
#undef _TIMER_CLK_CFG
	default:
		unreachable();
	}
}

static __force_inline void timer_reset(timer_nr_t timer_nr)
{
	switch (timer_nr) {
#define _TIMER_RESET(_bus, _n)								\
	case _n:									\
		RCC->_bus ## RSTR |= RCC_ ## _bus ## RSTR_TIM ## _n ## RST;		\
		RCC->_bus ## RSTR &= ~RCC_ ## _bus ## RSTR_TIM ## _n ## RST;		\
		break;
	_TIMER_RESET(APB2, 15)
	_TIMER_RESET(APB2, 16)
	_TIMER_RESET(APB2, 17)
	_TIMER_RESET(APB1, 3)
	_TIMER_RESET(APB1, 6)
	_TIMER_RESET(APB1, 14)
#undef _TIMER_RESET
	default:
		unreachable();
	}
}

static __force_inline TIM_TypeDef *timer_to_plat(timer_nr_t tim_nr)
{
	switch (tim_nr) {
	case 3: return TIM3;
	case 6: return TIM6;
	case 14: return TIM14;
	case 15: return TIM15;
	case 16: return TIM16;
	case 17: return TIM17;
	default: unreachable();
	}
}

static __force_inline uint8_t timer_irqn(timer_nr_t tim_nr)
{
	switch (tim_nr) {
	case 3: return TIM3_IRQn;
	case 6: return TIM6_DAC_IRQn;
	case 14: return TIM14_IRQn;
	case 15: return TIM15_IRQn;
	case 16: return TIM16_IRQn;
	case 17: return TIM17_IRQn;
	default: unreachable();
	}
}

static __force_inline void _timer_init(timer_nr_t tim_nr, uint32_t freq)
{
	TIM_TypeDef *tim = timer_to_plat(tim_nr);

	compiletime_assert(SYS_CORE_FREQ % freq == 0,
			   "Requested frequency unachievable");

	timer_clk_config(tim_nr, false);
	timer_reset(tim_nr);
	timer_clk_config(tim_nr, true);

	tim->CR1 = TIM_CR1_ARPE;
	tim->EGR = TIM_EGR_UG;
	tim->CNT = 0;
}

static __force_inline void timer_init(timer_nr_t tim_nr, uint32_t freq,
				      uint8_t irq_prio)
{
	TIM_TypeDef *tim = timer_to_plat(tim_nr);

	_timer_init(tim_nr, freq);

	/* prescaler and auto reload */
	tim->PSC = freq2psc(freq) - 1;
	tim->ARR = freq2car(freq) - 1;

	tim->DIER = TIM_DIER_UIE;

	nvic_enable_irq_with_prio(timer_irqn(tim_nr), irq_prio);
}

static __force_inline void timer_init_pwm(timer_nr_t tim_nr, uint32_t freq,
					  uint16_t period)
{
	TIM_TypeDef *tim = timer_to_plat(tim_nr);

	compiletime_assert(SYS_CORE_FREQ / freq < 65536,
			   "Requested frequency unachievable for PWM timer");

	_timer_init(tim_nr, freq);

	/* prescaler and auto reload */
	tim->PSC = (SYS_CORE_FREQ / freq) - 1;
	tim->ARR = period - 1;

	tim->CCMR1 = (TIM_OCMode_PWM1 << 8) | TIM_CCMR1_OC2PE;
	tim->CCER = TIM_CCER_CC2P | TIM_CCER_CC2E;
	tim->BDTR |= TIM_BDTR_MOE;
}

static __force_inline void timer_enable(timer_nr_t tim_nr, bool on)
{
	TIM_TypeDef *tim = timer_to_plat(tim_nr);

	if (on) {
		tim->CR1 |= TIM_CR1_CEN;
	} else {
		tim->CR1 &= ~TIM_CR1_CEN;

		/* clear counter on disable */
		tim->CNT = 0;
	}
}

static __force_inline void timer_irq_enable(timer_nr_t tim_nr, bool on)
{
	timer_to_plat(tim_nr)->DIER = on ? TIM_DIER_UIE : 0;
}

static __force_inline bool timer_irq_clear_up(timer_nr_t tim_nr)
{
	TIM_TypeDef *tim = timer_to_plat(tim_nr);
	bool up = tim->SR & TIM_SR_UIF;

	if (up)
		tim->SR = ~TIM_SR_UIF;

	return up;
}

static __force_inline void timer_set_freq(timer_nr_t tim_nr, uint32_t freq)
{
	compiletime_assert(SYS_CORE_FREQ % freq == 0,
			   "Requested frequency unachievable");

	timer_to_plat(tim_nr)->PSC = freq2psc(freq) - 1;
	timer_to_plat(tim_nr)->ARR = freq2car(freq) - 1;
}

static __force_inline uint16_t timer_get_counter(timer_nr_t tim_nr)
{
	return timer_to_plat(tim_nr)->CNT;
}

static __force_inline void timer_set_pwm_duty(timer_nr_t tim_nr, uint16_t duty)
{
	timer_to_plat(tim_nr)->CCR2 = duty;
}

#endif /* TIMER_H */
