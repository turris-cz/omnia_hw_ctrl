#ifndef SVC_H
#define SVC_H

#include "cpu.h"

#if SYSCALLS

typedef enum {
	SYS_clk_config,
	SYS_enable_irq_with_prio,
	SYS_time_config,
	SYS_msleep,
	SYS_flash_init,
	SYS_flash_async_op,
	SYS_watchdog_enable,
	SYS_watchdog_set_timeout,
	SYS_soft_reset_to_other_program,
	SYS_hard_reset,
} svc_t;

static __force_inline uint32_t sv_call_0(svc_t svc)
{
	register uint32_t r0 asm("r0");

	asm volatile(
		"svc %1\n\t"
		: "=r" (r0) : "I" (svc) : "memory"
	);

	return r0;
}

static __force_inline uint32_t sv_call_1(svc_t svc, uint32_t arg)
{
	register uint32_t r0 asm("r0") = arg;

	asm volatile(
		"svc %1\n\t"
		: "+r" (r0) : "I" (svc) : "memory"
	);

	return r0;
}

static __force_inline uint32_t sv_call_2(svc_t svc, uint32_t arg1,
					 uint32_t arg2)
{
	register uint32_t r0 asm("r0") = arg1;
	register uint32_t r1 asm("r1") = arg2;

	asm volatile(
		"svc %1\n\t"
		: "+r" (r0) : "I" (svc), "r" (r1) : "memory"
	);

	return r0;
}

# define SYSCALL(_n) __privileged _n ## _supervisor
# define DEF_SYSCALLER_1(_n)			\
	static __force_inline			\
	typeof(_n ## _supervisor()) _n(void)	\
	{					\
		sv_call_0(SYS_ ## _n);		\
	}
# define DEF_SYSCALLER_2(_n, _t)			\
	static __force_inline				\
	typeof(_n ## _supervisor((_t)0)) _n(_t arg)	\
	{						\
		sv_call_1(SYS_ ## _n, (uint32_t)arg);	\
	}
# define DEF_SYSCALLER_3(_n, _t1, _t2)					\
	static __force_inline						\
	typeof(_n ## _supervisor((_t1)0, (_t2)0))			\
	_n(_t1 arg1, _t2 arg2)						\
	{								\
		sv_call_2(SYS_ ## _n, (uint32_t)arg1, (uint32_t)arg2);	\
	}
#define DEF_SYSCALLER(...) VARIADIC(DEF_SYSCALLER_, __VA_ARGS__)

#else /* !SYSCALLS */
# define SYSCALL(_n) _n
# define DEF_SYSCALLER(...)
#endif /* !SYSCALLS */

#endif /* SVC_H */
