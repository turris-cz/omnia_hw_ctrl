#ifndef TIMER_H
#define TIMER_H

#include "stm32f0xx_tim.h"
#include "compiler.h"
#include "cpu.h"

typedef uint8_t timer_nr_t;

#define LED_TIMER		3
#define LED_EFFECT_TIMER	6
#define LED_PWM_TIMER		15
#define USB_TIMEOUT_TIMER	17

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

static __force_inline void timer_init(timer_nr_t tim_nr, timer_type_t type,
				      uint16_t period, uint32_t freq,
				      uint8_t irq_prio)
{
	TIM_TypeDef *tim = timer_to_plat(tim_nr);
	TIM_TimeBaseInitTypeDef init = {
		.TIM_Period = period - 1,
		.TIM_Prescaler = (TIMER_PARENT_FREQ / freq) - 1,
		.TIM_ClockDivision = TIM_CKD_DIV1,
		.TIM_CounterMode = TIM_CounterMode_Up,
	};

	compiletime_assert(TIMER_PARENT_FREQ % freq == 0,
			   "Requested frequency unachievable");

	timer_clk_config(tim_nr, 0);
	timer_reset(tim_nr);
	timer_clk_config(tim_nr, 1);

	TIM_TimeBaseInit(tim, &init);

	if (type == timer_pwm) {
		TIM_OCInitTypeDef ocinit = {
			.TIM_OCMode = TIM_OCMode_PWM1,
			.TIM_OutputState = TIM_OutputState_Enable,
			.TIM_Pulse = 0,
			.TIM_OCPolarity = TIM_OCPolarity_Low,
		};

		TIM_OC2Init(tim, &ocinit);
		TIM_OC2PreloadConfig(tim, TIM_OCPreload_Enable);
	}

	TIM_ARRPreloadConfig(tim, ENABLE);

	if (type == timer_interrupt) {
		tim->DIER = TIM_IT_Update;

		nvic_enable_irq(timer_irqn(tim_nr), irq_prio);
	}

	if (type == timer_pwm)
		TIM_CtrlPWMOutputs(tim, ENABLE);
}

static __force_inline void timer_deinit(timer_nr_t tim_nr)
{
	timer_reset(tim_nr);
}

static __force_inline void timer_enable(timer_nr_t tim_nr, bool on)
{
	TIM_Cmd(timer_to_plat(tim_nr), on);
}

static __force_inline bool timer_irq_clear_up(timer_nr_t tim_nr)
{
	TIM_TypeDef *tim = timer_to_plat(tim_nr);
	bool up = tim->SR & TIM_IT_Update;

	if (up)
		tim->SR = ~(uint16_t)TIM_IT_Update;

	return up;
}

static __force_inline void timer_set_period(timer_nr_t tim_nr, uint16_t period)
{
	timer_to_plat(tim_nr)->ARR = period - 1;
}

static __force_inline void timer_set_freq(timer_nr_t tim_nr, uint32_t freq)
{
	compiletime_assert(TIMER_PARENT_FREQ % freq == 0,
			   "Requested frequency unachievable");

	timer_to_plat(tim_nr)->PSC = (TIMER_PARENT_FREQ / freq) - 1;
}

static __force_inline void timer_set_counter(timer_nr_t tim_nr, uint32_t cnt)
{
	timer_to_plat(tim_nr)->CNT = cnt;
}

static __force_inline uint16_t timer_get_counter(timer_nr_t tim_nr)
{
	return timer_to_plat(tim_nr)->CNT;
}

static __force_inline void timer_set_pulse(timer_nr_t tim_nr, uint32_t pulse)
{
	timer_to_plat(tim_nr)->CCR2 = pulse;
}

#endif /* TIMER_H */
