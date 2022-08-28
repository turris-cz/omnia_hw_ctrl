#include "gd32f1x0.h"
#include "gd32f1x0_misc.h"
#include "gd32f1x0_rcu.h"
#include "compiler.h"
#include "cpu.h"
#include "memory_layout.h"

extern uint32_t _stack_top, _sfdata, _sdata, _edata, _sbss, _ebss;
extern void __noreturn main(void);

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
		*(uint32_t *)ptr = 0;

	main();
}

static void __irq default_handler(void)
{
	while (1);
}

void nmi_handler(void) __weak_alias(default_handler);
void hardfault_handler(void) __weak_alias(default_handler);
void memmanage_handler(void) __weak_alias(default_handler);
void busfault_handler(void) __weak_alias(default_handler);
void usagefault_handler(void) __weak_alias(default_handler);
void svc_handler(void) __weak_alias(default_handler);
void debugmon_handler(void) __weak_alias(default_handler);
void pendsv_handler(void) __weak_alias(default_handler);
void external_irq(void) __weak_alias(default_handler);

void systick_irq_handler(void) __weak_alias(default_handler);
void led_driver_irq_handler(void) __weak_alias(default_handler);
void led_driver_effect_irq_handler(void) __weak_alias(default_handler);
void power_control_usb_timeout_irq_handler(void) __weak_alias(default_handler);
void i2c_slave_irq_handler(void) __weak_alias(default_handler);

static __used __section(".isr_vector") void *isr_vector[] = {
	&_stack_top,
	reset_handler,
	nmi_handler,
	hardfault_handler,
	memmanage_handler,
	busfault_handler,
	usagefault_handler,
	NULL,
	NULL,
	NULL,
	NULL,
	svc_handler,
	debugmon_handler,
	NULL,
	pendsv_handler,
	systick_irq_handler,
	NULL,					/* Window WatchDog */
	NULL,					/* LVD through EXTI Line detecion */
	NULL,					/* RTC */
	NULL,					/* FLASH */
	NULL,					/* RCU */
	NULL,					/* EXTI Line 0 and 1 */
	NULL,					/* EXTI Line 2 and 3 */
	NULL,					/* EXTI Line 4 to 15 */
	NULL,					/* TSI */
	NULL,					/* DMA1 Channel 0 */
	NULL,					/* DMA1 Channel 1 and Channel 2 */
	NULL,					/* DMA1 Channel 3 and Channel 4 */
	NULL,					/* ADC and CMP0-1 */
	NULL,					/* TIM0 Break, Update, Trigger and Commutation */
	NULL,					/* TIM0 Capture Compare */
	NULL,					/* TIM1 */
	led_driver_irq_handler,			/* TIM2 */
	led_driver_effect_irq_handler,		/* TIM5 and DAC */
	NULL,					/* Reserved */
	NULL,					/* TIM13 */
	NULL,					/* TIM14 */
	NULL,					/* TIM15 */
	power_control_usb_timeout_irq_handler,	/* TIM16 */
	NULL,					/* I2C0 */
	i2c_slave_irq_handler,			/* I2C1 */
	NULL,					/* SPI0 */
	NULL,					/* SPI1 */
	NULL,					/* USART0 */
	NULL,					/* USART1 */
	NULL,					/* Reserved */
	NULL,					/* CEC */
	NULL,					/* Reserved */
	NULL,					/* I2C0 error */
	NULL,					/* Reserved */
	i2c_slave_irq_handler,			/* I2C1 error */
	NULL,					/* I2C2 event */
	NULL,					/* I2C2 error */
	NULL,					/* USBD Low Priority */
	NULL,					/* USBD High Priority */
	NULL,					/* Reserved */
	NULL,					/* Reserved */
	NULL,					/* Reserved */
	NULL,					/* USBD Wake Up through EXTI Line18 */
	NULL,					/* Reserved */
	NULL,					/* Reserved */
	NULL,					/* Reserved */
	NULL,					/* Reserved */
	NULL,					/* Reserved */
	NULL,					/* DMA Channel 5-6 */
	NULL,					/* Reserved */
	NULL,					/* Reserved */
	NULL,					/* SPI2 */
};

static void system_init(void)
{
	/* don't reinitialize if already initialized correctly */
	if (((RCU_CFG0 & (RCU_AHB_CKSYS_DIV1 | RCU_APB2_CKAHB_DIV1 |
			  RCU_APB1_CKAHB_DIV1 | RCU_CFG0_PLLSEL |
			  RCU_CFG0_PLLMF | RCU_CFG0_SCS |
			  RCU_SCSS_PLL)) ==
	     (RCU_AHB_CKSYS_DIV1 | RCU_APB2_CKAHB_DIV1 | RCU_APB1_CKAHB_DIV1 |
	      RCU_PLLSRC_IRC8M_DIV2 | RCU_PLL_MUL18 | RCU_CKSYSSRC_PLL |
	      RCU_SCSS_PLL)) &&
	    (RCU_CTL0 & (RCU_CTL0_PLLEN | RCU_CTL0_PLLSTB)))
		return;

	/* enable IRC8M */
	RCU_CTL0 |= RCU_CTL0_IRC8MEN;
	while (!(RCU_CTL0 & RCU_CTL0_IRC8MSTB))
		nop();

	/* reset RCU */
	RCU_CFG0 &= ~(RCU_CFG0_SCS | RCU_CFG0_AHBPSC | RCU_CFG0_APB1PSC |
		      RCU_CFG0_APB2PSC | RCU_CFG0_ADCPSC | RCU_CFG0_CKOUTSEL |
		      RCU_CFG0_CKOUTDIV | RCU_CFG0_PLLDV);
	RCU_CFG0 &= ~(RCU_CFG0_PLLSEL | RCU_CFG0_PLLMF | RCU_CFG0_PLLPREDV);
	RCU_CFG0 &= ~(RCU_CFG0_USBDPSC);
	RCU_CTL0 &= ~(RCU_CTL0_HXTALEN | RCU_CTL0_CKMEN | RCU_CTL0_PLLEN | RCU_CTL0_HXTALBPS);
	RCU_CFG1 &= ~RCU_CFG1_HXTALPREDV;
	RCU_CFG2 &= ~(RCU_CFG2_USART0SEL | RCU_CFG2_CECSEL | RCU_CFG2_ADCSEL);
	RCU_CTL1 &= ~RCU_CTL1_IRC14MEN;
	RCU_INT = 0x00000000U;

	/* configure AHB, APB1, APB2 and PLL clocks*/
	RCU_CFG0 |= RCU_AHB_CKSYS_DIV1;
	RCU_CFG0 |= RCU_APB2_CKAHB_DIV1;
	RCU_CFG0 |= RCU_APB1_CKAHB_DIV1;
	RCU_CFG0 |= (RCU_PLLSRC_IRC8M_DIV2 | RCU_PLL_MUL18);

	/* enable PLL */
	RCU_CTL0 |= RCU_CTL0_PLLEN;
	while (!(RCU_CTL0 & RCU_CTL0_PLLSTB))
		nop();

	/* select PLL as system clock */
	RCU_CFG0 = (RCU_CFG0 & ~RCU_CFG0_SCS) | RCU_CKSYSSRC_PLL;
	while (!(RCU_CFG0 & RCU_SCSS_PLL))
		nop();
}

static void platform_init(void)
{
	/* initialize system clocks */
	system_init();

	/* set vector table offset */
	if (BOOTLOADER_BUILD)
		SCB->VTOR = BOOTLOADER_BEGIN;
	else
		SCB->VTOR = APPLICATION_BEGIN;

	/* disable all interrupts and clear all pending interrupts */
	for (unsigned int i = 0; i < ARRAY_SIZE(NVIC->ICER); ++i) {
		NVIC->ICER[i] = 0xffffffff;
		NVIC->ICPR[i] = 0xffffffff;
	}

	nvic_priority_group_set(NVIC_PRIGROUP_PRE2_SUB2);

	/* do not reset ports in bootloader */
	if (BOOTLOADER_BUILD)
		return;

	/* PORTs reset */
	RCU_AHBRST |= RCU_AHBRST_PARST | RCU_AHBRST_PBRST | RCU_AHBRST_PCRST |
		      RCU_AHBRST_PDRST | RCU_AHBRST_PFRST;

	/* disable PORTs reset */
	RCU_AHBRST &= ~(RCU_AHBRST_PARST | RCU_AHBRST_PBRST | RCU_AHBRST_PCRST |
			RCU_AHBRST_PDRST | RCU_AHBRST_PFRST);
}
