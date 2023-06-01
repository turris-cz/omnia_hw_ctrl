#ifndef TIMER_COMMON_STM32_GD32_H
#define TIMER_COMMON_STM32_GD32_H

#include "cpu.h"

typedef uint8_t timer_nr_t;

/*
 * These functions are used in STM32's and GD32's timer_init() and
 * timer_set_freq() to compute (at compile-time) the 16-bit timer prescaler
 * and 16-bit timer auto reload register values given the requested frequency.
 *
 * We want the values to be computed at compile-time so that the code is
 * optimized to simply put the constant values into the registers.
 */

static __force_inline uint32_t _freq2car(uint32_t freq)
{
	uint32_t p = SYS_CORE_FREQ / freq;

#define TRY_DIV(n)			\
	if (p > n && p % n == 0)	\
		return p / n;
	TRY_DIV(50000)
	TRY_DIV(30000)
	TRY_DIV(20000)
	TRY_DIV(10000)
	TRY_DIV(5000)
	TRY_DIV(3000)
	TRY_DIV(2000)
	TRY_DIV(1000)
	TRY_DIV(500)
	TRY_DIV(300)
	TRY_DIV(200)
	TRY_DIV(125)
	TRY_DIV(100)
	TRY_DIV(50)
	TRY_DIV(30)
	TRY_DIV(25)
	TRY_DIV(20)
	TRY_DIV(16)
	TRY_DIV(10)
	TRY_DIV(8)
	TRY_DIV(5)
	TRY_DIV(4)
	TRY_DIV(3)
	TRY_DIV(2)
#undef TRY_DIV
	return p;
}

static __force_inline uint32_t freq2car(uint32_t freq)
{
	compiletime_assert(_freq2car(freq) <= 65536 &&
			   _freq2car(freq) > 1,
			   "Requested frequency unachievable");

	return _freq2car(freq);
}

static __force_inline uint32_t _freq2psc(uint32_t freq)
{
	return (SYS_CORE_FREQ / freq) / freq2car(freq);
}

static __force_inline uint32_t freq2psc(uint32_t freq)
{
	compiletime_assert(_freq2psc(freq) <= 65536,
			   "Requested frequency unachievable");

	return _freq2psc(freq);
}

#endif /* TIMER_COMMON_STM32_GD32_H */
