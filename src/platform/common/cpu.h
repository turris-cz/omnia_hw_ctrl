#ifndef CPU_H
#define CPU_H

#include "compiler.h"
#include "bits.h"

#if defined(STM32F030X8)
# include "stm32f0xx.h"
#elif defined(GD32F1x0)
# include "gd32f1x0.h"
#else
# error "unknown platform"
#endif

#define IRQ_PRIO_SHIFT	6

static inline uint32_t get_unaligned16(const void *ptr)
{
	const uint8_t *p = ptr;

	return (p[1] << 8) | p[0];
}

static inline uint32_t get_unaligned32(const void *ptr)
{
	const uint8_t *p = ptr;

	return (p[3] << 24) | (p[2] << 16) | (p[1] << 8) | p[0];
}

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

static __force_inline uint32_t get_ipsr(void)
{
	uint32_t ipsr;

	asm volatile("mrs %0, ipsr" : "=r" (ipsr));

	return ipsr;
}

static __force_inline void isb(void)
{
	asm volatile("isb\n");
}

static __force_inline void dsb(void)
{
	asm volatile("dsb\n");
}

static __force_inline void nop(void)
{
	asm("nop\n");
}

static __force_inline void wait_for_interrupt(void)
{
	asm volatile("wfi\n");
}

#ifdef __ARM_ARCH_6M__
static inline unsigned _irq_idx(IRQn_Type irq)
{
	if (irq < 0)
		return (irq + 8) >> 2;
	else
		return irq >> 2;
}

static inline unsigned _irq_shf(IRQn_Type irq)
{
	if (irq < 0)
		return ((irq + 8) & 3) << 3;
	else
		return (irq & 3) << 3;
}

static inline void nvic_set_priority(IRQn_Type irq, uint8_t prio)
{
	unsigned idx = _irq_idx(irq), shf = _irq_shf(irq);

	if (irq < 0)
		SCB->SHP[idx] = (SCB->SHP[idx] & ~(0xff << shf)) |
				((prio << IRQ_PRIO_SHIFT) << shf);
	else
		NVIC->IP[idx] = (NVIC->IP[idx] & ~(0xff << shf)) |
				((prio << IRQ_PRIO_SHIFT) << shf);
}

static inline uint8_t nvic_get_priority(IRQn_Type irq)
{
	unsigned idx = _irq_idx(irq), shf = _irq_shf(irq);

	if (irq < 0)
		return ((SCB->SHP[idx] >> shf) & 0xff) >> IRQ_PRIO_SHIFT;
	else
		return ((NVIC->IP[idx] >> shf) & 0xff) >> IRQ_PRIO_SHIFT;
}
#elifdef __ARM_ARCH_7M__
static inline void nvic_set_priority(IRQn_Type irq, uint8_t prio)
{
	if (irq < 0)
		SCB->SHP[(irq + 12) & 0xf] = prio << IRQ_PRIO_SHIFT;
	else
		NVIC->IP[irq & 0xff] = prio << IRQ_PRIO_SHIFT;
}

static inline uint8_t nvic_get_priority(IRQn_Type irq)
{
	if (irq < 0)
		return SCB->SHP[(irq + 12) & 0xf] >> IRQ_PRIO_SHIFT;
	else
		return NVIC->IP[irq & 0xff] >> IRQ_PRIO_SHIFT;
}

static inline void nvic_set_priority_grouping(uint8_t group)
{
	SCB->AIRCR = (SCB->AIRCR & 0xf8ff) | 0x5fa0000 | ((group & 7) << 8);
}
#else
# error "unknown platform"
#endif

static inline bool systick_config(uint32_t ticks)
{
	ticks -= 1;

	if (ticks > 0xffffff)
		return false;

	SysTick->LOAD = ticks;
	SysTick->VAL = 0;
	SysTick->CTRL = BIT(2) | BIT(1) | BIT(0);

	return true;
}

static inline void nvic_system_reset(void)
{
	dsb();
	SCB->AIRCR = (SCB->AIRCR & 0x8700) | 0x5fa0004;
	dsb();

	while (1)
		nop();
}

static inline void nvic_enable_irq(IRQn_Type irq)
{
	NVIC->ISER[irq >> 5] = BIT(irq & 0x1f);
}

static inline void nvic_enable_irq_with_prio(IRQn_Type irq, uint8_t prio)
{
	nvic_set_priority(irq, prio);
	nvic_enable_irq(irq);
}

static inline void nvic_disable_irq(IRQn_Type irq)
{
	NVIC->ICER[irq >> 5] = BIT(irq & 0x1f);
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
