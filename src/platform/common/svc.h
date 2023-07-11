#ifndef SVC_H
#define SVC_H

#include "cpu.h"

#if PRIVILEGES

/*
 * If the PRIVILEGES macro is set to 1, we must implement supervisor calls, so
 * that the unprivileged mode is able to call privileged functions.
 *
 * A regular function can be turned into a syscall if:
 * - it accepts at most 4 arguments, each at most 32-bit wide (so that they can
 *   be passed in r0-r3 registers)
 * - it returns at most 32-bit wide result (so that it can be returned via r0;
 *   void is allowed)
 *
 * To turn a regular function into a syscall:
 * - add the __privileged property to the function so that it resides in the
 *   privileged text section
 * - after the declaration / inline definition of the function, use the
 *   SYSCALL(fname, arg1_type, ...) macro to define a static inline svc caller
 *   sys_fname
 * - add SYS_fname to the svc_t enumerator
 * - add a call to fname() under the SYS_fname case in the svc_handler function
 *   in svc.c
 * - call sys_fname() instead of fname() to call from unprivileged context
 *
 * Note:
 *   If you need to call the function from a privileged context (i.e. from an
 *   exception handler), just call the fname() function.
 *
 * Example:
 *   Let's say we have a function with declaration
 *
 *     bool do_something(void *ptr, uint16_t len);
 *
 *   We user the SYSCALL macro after the declaration like this:
 *
 *     bool do_something(void *ptr, uint16_t len);
 *     SYSCALL(do_something, void *, uint16_t)
 *
 *   We add SYS_do_something to the svc_t enumerator:
 *
 *     typedef enum {
 *         ...
 *         SYS_do_something,
 *     } svc_t;
 *
 *   Finally we add handling code to svc_handler:
 *
 *     void __irq svc_handler(void)
 *     {
 *         ...
 *         switch (svc) {
 *         ...
 *         case SYS_do_something:
 *             frame->r0 = do_something((void *)arg1, arg2);
 *             break;
 *         }
 *     }
 *
 *   From the unprivileged context we then call the system call as:
 *
 *     if (sys_do_something(&var, sizeof(var)))
 *         ...
 *
 * Implementation notes:
 *   The SYSCALL(fname, ...) macro defines a static inline function sys_fname,
 *   which passes the arguments in r0-r3, calls the svc instruction, and returns
 *   the value that is in r0 after the instruction
 *
 *     static inline bool sys_do_something(void *arg1, uint16_t arg2)
 *     {
 *         register uint32_t r0 asm("r0") = (uint32_t)arg1;
 *         register uint32_t r1 asm("r1") = (uint32_t)arg2;
 *
 *         asm volatile(
 *             "svc %1\n\t"
 *             : "+r" (r0)
 *             : "I" (svc), "r" (r1)
 *             : "memory"
 *         );
 *
 *         return (bool)r0;
 *     }
 */

typedef enum {
	SYS_input_signals_poll,
	SYS_sigreturn,
	SYS_button_counter_decrease,
	SYS_button_set_user_mode,
	SYS_input_signals_init,
	SYS_led_driver_set_brightness,
	SYS_led_driver_overwrite_brightness,
	SYS_led_set_user_mode,
	SYS_led_set_state_user,
	SYS_clk_config,
	SYS_enable_irq_with_prio,
	SYS_time_config,
	SYS_msleep,
	SYS_watchdog_enable,
	SYS_watchdog_set_timeout,
	SYS_flash_init,
	SYS_plat_firmware_flash_finish,
	SYS_plat_soft_reset_to_other_program,
	SYS_hard_reset,
	SYS_poweroff,
} svc_t;

static __force_inline uint32_t sv_call_0(svc_t svc)
{
	register uint32_t r0 asm("r0");

	asm volatile(
		"svc %1\n\t"
		: "=r" (r0)
		: "I" (svc)
		: "memory"
	);

	return r0;
}

static __force_inline uint32_t sv_call_1(svc_t svc, uint32_t arg)
{
	register uint32_t r0 asm("r0") = arg;

	asm volatile(
		"svc %1\n\t"
		: "+r" (r0)
		: "I" (svc)
		: "memory"
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
		: "+r" (r0)
		: "I" (svc), "r" (r1)
		: "memory"
	);

	return r0;
}

static __force_inline uint32_t sv_call_3(svc_t svc, uint32_t arg1,
					 uint32_t arg2, uint32_t arg3)
{
	register uint32_t r0 asm("r0") = arg1;
	register uint32_t r1 asm("r1") = arg2;
	register uint32_t r2 asm("r2") = arg3;

	asm volatile(
		"svc %1\n\t"
		: "+r" (r0)
		: "I" (svc), "r" (r1), "r" (r2)
		: "memory"
	);

	return r0;
}

static __force_inline uint32_t sv_call_4(svc_t svc, uint32_t arg1,
					 uint32_t arg2, uint32_t arg3,
					 uint32_t arg4)
{
	register uint32_t r0 asm("r0") = arg1;
	register uint32_t r1 asm("r1") = arg2;
	register uint32_t r2 asm("r2") = arg3;
	register uint32_t r3 asm("r3") = arg4;

	asm volatile(
		"svc %1\n\t"
		: "+r" (r0)
		: "I" (svc), "r" (r1), "r" (r2), "r" (r3)
		: "memory"
	);

	return r0;
}

#endif /* PRIVILEGES */

# define RTYPE_1(_n)			typeof(_n())
# define RTYPE_2(_n, _t)		typeof(_n((_t)0))
# define RTYPE_3(_n, _t1, _t2)		typeof(_n((_t1)0, (_t2)0))
# define RTYPE_4(_n, _t1, _t2, _t3)	typeof(_n((_t1)0, (_t2)0, (_t3)0))
# define RTYPE_5(_n, _t1, _t2, _t3, _t4) \
	typeof(_n((_t1)0, (_t2)0, (_t3)0, (_t4)0))

#if PRIVILEGES

# define SYSCALL_1(_n)						\
	static __force_inline RTYPE_1(_n)			\
	sys_ ## _n(void)					\
	{							\
		return (RTYPE_1(_n))sv_call_0(SYS_ ## _n);	\
	}
# define SYSCALL_2(_n, _t)						\
	static __force_inline RTYPE_2(_n, _t)				\
	sys_ ## _n(_t arg)						\
	{								\
		return (RTYPE_2(_n, _t))sv_call_1(SYS_ ## _n,		\
						  (uint32_t)arg);	\
	}
# define SYSCALL_3(_n, _t1, _t2)					\
	static __force_inline RTYPE_3(_n, _t1, _t2)			\
	sys_ ## _n(_t1 arg1, _t2 arg2)					\
	{								\
		return (RTYPE_3(_n, _t1, _t2))				\
			sv_call_2(SYS_ ## _n, (uint32_t)arg1,		\
				  (uint32_t)arg2);			\
	}
# define SYSCALL_4(_n, _t1, _t2, _t3)					\
	static __force_inline RTYPE_4(_n, _t1, _t2, _t3)		\
	sys_ ## _n(_t1 arg1, _t2 arg2, _t3 arg3)			\
	{								\
		return (RTYPE_4(_n, _t1, _t2, _t3))			\
			sv_call_3(SYS_ ## _n, (uint32_t)arg1,		\
				  (uint32_t)arg2, (uint32_t)arg3);	\
	}
# define SYSCALL_5(_n, _t1, _t2, _t3, _t4)				\
	static __force_inline RTYPE_5(_n, _t1, _t2, _t3, _t4)		\
	sys_ ## _n(_t1 arg1, _t2 arg2, _t3 arg3, _t4 arg4)		\
	{								\
		return (RTYPE_5(_n, _t1, _t2, _t3, _t4))		\
			sv_call_4(SYS_ ## _n, (uint32_t)arg1,		\
				  (uint32_t)arg2, (uint32_t)arg3,	\
				  (uint32_t)arg4);			\
	}

#else /* !PRIVILEGES */

# define SYSCALL_1(_n)				\
	static __force_inline RTYPE_1(_n)	\
	sys_ ## _n(void)			\
	{					\
		return _n();			\
	}
# define SYSCALL_2(_n, _t)			\
	static __force_inline RTYPE_2(_n, _t)	\
	sys_ ## _n(_t arg)			\
	{					\
		return _n(arg);			\
	}
# define SYSCALL_3(_n, _t1, _t2)			\
	static __force_inline RTYPE_3(_n, _t1, _t2)	\
	sys_ ## _n(_t1 arg1, _t2 arg2)			\
	{						\
		return _n(arg1, arg2);			\
	}
# define SYSCALL_4(_n, _t1, _t2, _t3)				\
	static __force_inline RTYPE_4(_n, _t1, _t2, _t3)	\
	sys_ ## _n(_t1 arg1, _t2 arg2, _t3 arg3)		\
	{							\
		return _n(arg1, arg2, arg3);			\
	}
# define SYSCALL_5(_n, _t1, _t2, _t3, _t4)			\
	static __force_inline RTYPE_5(_n, _t1, _t2, _t3, _t4)	\
	sys_ ## _n(_t1 arg1, _t2 arg2, _t3 arg3, _t4 arg4)	\
	{							\
		return _n(arg1, arg2, arg3, arg4);		\
	}

#endif /* !PRIVILEGES */

#define SYSCALL(...) VARIADIC(SYSCALL_, __VA_ARGS__)

#endif /* SVC_H */
