#include "stm32f0xx.h"
#include "stm32f0xx_syscfg.h"
#include "compiler.h"
#include "memory_layout.h"
#include "message.h"

extern uint32_t _stack_top, _sfdata, _sdata, _edata, _sbss, _ebss;
extern void __noreturn main(void);

static void configure_isr_vector(void);
static void platform_init(void);

void __noreturn __naked __section(".startup")
reset_handler(void)
{
	asm volatile(
		"ldr	r0, =_stack_top\n"
		"mov	sp, r0\n"
	);

	platform_init();

	/* copy data */
	for (uint32_t *src = &_sfdata, *dst = &_sdata; dst < &_edata;)
		*dst++ = *src++;

	/* zero out bss */
	for (uint32_t *ptr = &_sbss; ptr < &_ebss; ++ptr)
		*ptr = 0;

	configure_isr_vector();
	main();
}

static void __irq __naked default_handler(void)
{
	disable_irq();

#if BOOTLOADER_BUILD
	while (1);
#else
	asm volatile(
		"mov	sp, %0\n"
		: : "lr" (RAM_END - SYS_RESET_MSG_LENGTH)
	);

	sys_reset_with_message(get_ipsr() & 0x3f);
#endif
}

void nmi_handler(void) __weak_alias(default_handler);
void hardfault_handler(void) __weak_alias(default_handler);
void svc_handler(void) __weak_alias(default_handler);
void pendsv_handler(void) __weak_alias(default_handler);
void external_irq(void) __weak_alias(default_handler);

void systick_irq_handler(void) __weak_alias(default_handler);
void flash_irq_handler(void) __weak_alias(default_handler);
void led_driver_irq_handler(void) __weak_alias(default_handler);
void led_driver_pattern_irq_handler(void) __weak_alias(default_handler);
void power_control_usb_timeout_irq_handler(void) __weak_alias(default_handler);
void i2c_slave_irq_handler(void) __weak_alias(default_handler);

static __used __section(".isr_vector") void * const isr_vector[] = {
	&_stack_top,
	reset_handler,
	nmi_handler,
	hardfault_handler,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	NULL,
	svc_handler,
	NULL,
	NULL,
	pendsv_handler,
	systick_irq_handler,
	NULL,					/* Window WatchDog */
	NULL,					/* Reserved */
	NULL,					/* RTC through the EXTI line */
	flash_irq_handler,			/* FLASH */
	NULL,					/* RCC */
	NULL,					/* EXTI Line 0 and 1 */
	NULL,					/* EXTI Line 2 and 3 */
	NULL,					/* EXTI Line 4 to 15 */
	NULL,					/* Reserved */
	NULL,					/* DMA1 Channel 1 */
	NULL,					/* DMA1 Channel 2 and Channel 3 */
	NULL,					/* DMA1 Channel 4 and Channel 5 */
	NULL,					/* ADC1 */
	NULL,					/* TIM1 Break, Update, Trigger and Commutation */
	NULL,					/* TIM1 Capture Compare */
	NULL,					/* Reserved */
	led_driver_irq_handler,			/* TIM3 */
	led_driver_pattern_irq_handler,		/* TIM6 */
	NULL,					/* Reserved */
	NULL,					/* TIM14 */
	NULL,					/* TIM15 */
	NULL,					/* TIM16 */
	power_control_usb_timeout_irq_handler,	/* TIM17 */
	NULL,					/* I2C1 */
	i2c_slave_irq_handler,			/* I2C2 */
	NULL,					/* SPI1 */
	NULL,					/* SPI2 */
	NULL,					/* USART1 */
	NULL,					/* USART2 */
	NULL,					/* Reserved */
	NULL,					/* Reserved */
	NULL,					/* Reserved */
};

static void platform_init(void)
{
	/* initialize system clocks */
	SystemInit();

	nvic_disable_all_and_clear_pending();
	systick_disable_clear();

	/* do not reset ports in bootloader */
	if (BOOTLOADER_BUILD)
		return;

	/* PORTs reset */
	RCC->AHBRSTR |= RCC_AHBRSTR_GPIOARST | RCC_AHBRSTR_GPIOBRST |
			RCC_AHBRSTR_GPIOCRST | RCC_AHBRSTR_GPIODRST |
			RCC_AHBRSTR_GPIOFRST;

	/* SYSCFG reset */
	RCC->APB2RSTR |= RCC_APB2RSTR_SYSCFGRST;

	/* disable PORTs reset */
	RCC->AHBRSTR &= ~(RCC_AHBRSTR_GPIOARST | RCC_AHBRSTR_GPIOBRST |
			  RCC_AHBRSTR_GPIOCRST | RCC_AHBRSTR_GPIODRST |
			  RCC_AHBRSTR_GPIOFRST);

	/* disable SYSCFG reset */
	RCC->APB2RSTR &= ~RCC_APB2RSTR_SYSCFGRST;
}

static void configure_isr_vector(void)
{
	if (BOOTLOADER_BUILD) {
		/* remap flash to 0x00000000 to map ISR vector there */
		SYSCFG->CFGR1 = (SYSCFG->CFGR1 & ~SYSCFG_CFGR1_MEM_MODE) |
				SYSCFG_MemoryRemap_Flash;
		return;
	}

	/* copy ISR vector to start of RAM */
	for (uint32_t *dst = (uint32_t *)RAM_BEGIN_RAW,
		      *src = (uint32_t *)&isr_vector[0];
	     src < (uint32_t *)&isr_vector[ARRAY_SIZE(isr_vector)];)
		*dst++ = *src++;

	/* remap RAM to 0x00000000 to map ISR vector there */
	SYSCFG->CFGR1 = (SYSCFG->CFGR1 & ~SYSCFG_CFGR1_MEM_MODE) |
			SYSCFG_MemoryRemap_SRAM;
}
