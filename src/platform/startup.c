#include <stdint.h>
#include "compiler.h"

extern uint32_t _stack_top, _sfdata, _sdata, _edata, _sbss, _ebss;

extern void __noreturn main(void);

void SystemInit(void);

void __noreturn __naked __section(".startup")
reset_handler(void)
{
	asm volatile(
		"ldr	r0, =_stack_top\n"
		"mov	sp, r0\n"
	);

	/* copy data */
	for (uint32_t *src = &_sfdata, *dst = &_sdata; dst < &_edata;)
		*dst++ = *src++;

	/* zero out bss */
	for (uint32_t *ptr = &_sbss; ptr < &_ebss; ++ptr)
		*(uint32_t *)ptr = 0;

	SystemInit();
	main();
}

static void __irq default_handler(void)
{
	while (1);
}

void nmi_handler(void) __weak_alias(default_handler);
void hardfault_handler(void) __weak_alias(default_handler);
void svc_handler(void) __weak_alias(default_handler);
void pendsv_handler(void) __weak_alias(default_handler);
void external_irq(void) __weak_alias(default_handler);

void systick_irq_handler(void) __weak_alias(default_handler);
void led_driver_irq_handler(void) __weak_alias(default_handler);
void led_driver_effect_irq_handler(void) __weak_alias(default_handler);
void debounce_timer_irq_handler(void) __weak_alias(default_handler);
void power_control_usb_timeout_irq_handler(void) __weak_alias(default_handler);
void i2c_slave_irq_handler(void) __weak_alias(default_handler);

static __used __section(".isr_vector") void *isr_vector[] = {
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
	NULL,					/* FLASH */
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
	led_driver_effect_irq_handler,		/* TIM6 */
	NULL,					/* Reserved */
	NULL,					/* TIM14 */
	NULL,					/* TIM15 */
	debounce_timer_irq_handler,		/* TIM16 */
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
