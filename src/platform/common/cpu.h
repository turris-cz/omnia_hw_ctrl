#ifndef CPU_H
#define CPU_H

#include "compiler.h"

static __force_inline void enable_irq(void)
{
	asm volatile("cpsie i\n" : : : "memory");
}

static __force_inline void disable_irq(void)
{
	asm volatile("cpsid i\n" : : : "memory");
}

static __force_inline void set_msp(uint32_t msp)
{
	asm volatile("msr msp, %0\n" : : "r" (msp));
}

static __force_inline void isb(void)
{
	asm volatile("isb\n");
}

static __force_inline void nop(void)
{
	asm("nop\n");
}

static __force_inline void wait_for_interrupt(void)
{
	asm volatile("wfi\n");
}

#if defined(STM32F030X8)
# define LOOP_TICKS		4
#elif defined(GD32F1x0)
# define LOOP_TICKS		3
#else
# error "mdelay() parameters not defined for this platform"
#endif

#define MSEC_TO_TICKS		(SYS_CORE_FREQ / 1000)
#define MSEC_TO_LOOPS		(MSEC_TO_TICKS / LOOP_TICKS)

_Static_assert(SYS_CORE_FREQ % 1000 == 0,
	       "SYS_CORE_FREQ must be divisible by 1000");

_Static_assert(MSEC_TO_TICKS % LOOP_TICKS == 0,
	       "MSEC_TO_TICKS must be divisible by LOOP_TICKS");

static __force_inline void mdelay(uint32_t ms)
{
	uint32_t loops;

	compiletime_assert(ms < (UINT32_MAX / MSEC_TO_LOOPS),
			   "mdelay() maximum exceeded");

	if (!ms)
		return;

	loops = ms * MSEC_TO_LOOPS;
	asm volatile(
		"0:\n"
		"subs %0, #1\n"
		"bne 0b\n"
		: "+r" (loops)
	);
}

#undef MSEC_TO_LOOPS

static __force_inline __noreturn void reset_to_address(uint32_t isr_vec_addr)
{
	__noreturn void (*new_reset_handler)(void);
	uint32_t sp;

	disable_irq();

	/* get stack pointer from ISR vector */
	sp = *(volatile uint32_t *)isr_vec_addr;

	new_reset_handler = (void *)*(volatile uint32_t *)(isr_vec_addr + 4);

	/* set stack pointer */
	set_msp(sp);

	/* instruction synchronization barrier to flush pipeline */
	isb();

	/* jump to new app */
	new_reset_handler();
}

#endif /* CPU_H */
