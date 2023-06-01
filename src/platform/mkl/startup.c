#include "cpu.h"
#include "memory_layout.h"
#include "reset.h"
#include "timer.h"

extern uint32_t _stack_top, _sfreloc, _sreloc, _ereloc, _sbss, _ebss;
extern void __noreturn main(void);

static void platform_init(void);

void __noreturn __naked __section(".startup")
reset_handler(void)
{
	disable_irq();

	set_msp((uint32_t)&_stack_top);

	platform_init();

	main();
}

static void __irq __naked default_handler(void)
{
	disable_irq();

#if BOOTLOADER_BUILD
	while (1);
#else
	set_msp((uint32_t)&_stack_top);

	set_reset_reason(APPLICATION_FAULT, get_ipsr() & 0x3f);
	nvic_system_reset();
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
void port_irq_handler(void) __weak_alias(default_handler);
void pit_wakeup_irq_handler(void) __weak_alias(default_handler);
void pit_irq_handler(void);

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
	NULL,					/* DMA0 channel 0 or 4 transfer complete */
	NULL,					/* DMA0 channel 1 or 5 transfer complete */
	NULL,					/* DMA0 channel 2 or 6 transfer complete */
	NULL,					/* DMA0 channel 3 or 7 transfer complete */
	NULL,					/* DMA0 error interrupt */
	NULL,					/* Flexible IO */
	NULL,					/* Timer/PWM module 0 */
	led_driver_irq_handler,			/* Timer/PWM module 1 */
	led_driver_pattern_irq_handler,		/* Timer/PWM module 2 */
	pit_irq_handler,			/* Periodic interrupt timer 0 */
	NULL,					/* Serial peripheral interface 0 */
	NULL,					/* EMVSIM0 */
	NULL,					/* Low power UART 0 */
	NULL,					/* Low power UART 1 */
	i2c_slave_irq_handler,			/* I2C module 0 */
	NULL,					/* QSPI0 */
	NULL,					/* DryIce tamper */
	NULL,					/* Port A */
	NULL,					/* Port B */
	NULL,					/* Port C */
	port_irq_handler,			/* Port D */
	NULL,					/* Port E */
	NULL,					/* Low leakage wake up */
	NULL,					/* Low power trusted cryptographic 0 */
	NULL,					/* Universal serial bus 0 */
	NULL,					/* Analog to digital convertor 0 */
	NULL,					/* Low power timer 0 */
	NULL,					/* Real time clock seconds */
	flash_irq_handler,			/* Selectable peripheral interrupt INTMUX0-0 */
	NULL,					/* Selectable peripheral interrupt INTMUX0-1 */
	NULL,					/* Selectable peripheral interrupt INTMUX0-2 */
	NULL,					/* Selectable peripheral interrupt INTMUX0-3 */
};

static __force_inline bool pit_channel_flag(unsigned chn)
{
	return (PIT_TCTRL(chn) & PIT_TCTRL_TIE) &&
	       (PIT_TFLG(chn) & PIT_TFLG_TIF);
}

void __irq pit_irq_handler(void)
{
	if (pit_channel_flag(0))
		power_control_usb_timeout_irq_handler();
	if (pit_channel_flag(1))
		pit_wakeup_irq_handler();
}

static __force_inline void pit_init(void)
{
	BME_OR(SIM_SCGC6) = SIM_SCGC6_PIT0;

	for (int i = 0; i < 4; ++i)
		BME_AND(PIT_TCTRL(i)) = ~PIT_TCTRL_TEN;

	BME_AND(PIT_MCR) = ~PIT_MCR_MDIS;

	nvic_enable_irq_with_prio(PIT0_IRQn, PIT_IRQ_PRIO);
}

static __force_inline void clock_init(void)
{
	uint8_t c4;

	/* First go to fast FLL Bypassed External mode */

	/* 1. Set CLKDIV1 to safe divider value */
	SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(2) | SIM_CLKDIV1_OUTDIV2(6) |
		      SIM_CLKDIV1_OUTDIV4(6) | SIM_CLKDIV1_OUTDIV5(2);

	/* 2. Select Fast IRC, disable Low Power */
	MCG_C2 = (MCG_C2 & ~MCG_C2_LP) | MCG_C2_IRCS;

	/* 3. Select Internal Reference Clock as Clock Source */
	MCG_C1 = (MCG_C1 & ~(MCG_C1_CLKS_MASK | MCG_C1_IREFS)) |
		 MCG_C1_CLKS_EXTREF;

	/* 4. Wait for changes to take place */
	while ((MCG_S & (MCG_S_CLKST_MASK | MCG_S_IREFST)) !=
	       MCG_S_CLKST_EXTREF)
		nop();

	/* 5. Disable PLL Select */
	MCG_C6 &= ~MCG_C6_PLLS;

	/* Now configure FLL Engaged External mode */

	/* 0. Do we need to configure RANGE? */

	/* 1. Set FRDIV value
	 *    Note: FLL input frequency must be in range of 31.25 - 39.0625 kHz
	 *
	 *    FLL Input frequency: FLLIN = REFCLK / FRDIV = 48MHz / 1280 = 37.5kHz
	 */
	BME_BITFIELD(MCG_C1, MCG_C1_FRDIV_MASK) = MCG_C1_FRDIV_DIV1280;

	/* 2. Select IRC48 as external MCG source oscillator
	 *    REFCLK = 48 MHz
	 */
	BME_BITFIELD(MCG_C7, MCG_C7_OSCSEL_MASK) = MCG_C7_OSCSEL_OSCCLK1;

	/* 3. Select PLL or FLL as Clock Source, select external oscillator
	 * as MCG source */
	MCG_C1 = (MCG_C1 & ~(MCG_C1_CLKS_MASK | MCG_C1_IREFS)) |
		 MCG_C1_CLKS_PLLFLL;

	/* 4. Wait for IREFS change to take place */
	while (MCG_S & MCG_S_IREFST)
		nop();

	/* 5. Set DCO Range and DCO Maximum Frequency
	 *    FLL Output frequency: FLLOUT = FLLIN * 2560 = 37.5kHz * 2560 = 96MHz
	 */
	c4 = (MCG_C4 & ~(MCG_C4_DMX32 | MCG_C4_DRST_DRS_MASK)) |
	     MCG_C4_DRST_DRS_HIGH;
	MCG_C4 = c4;

	/* 6. Wait for DRST_DRS update */
	while (MCG_C4 != c4)
		nop();

	/* 7. Wait for change to FLL to take place */
	while ((MCG_S & MCG_S_CLKST_MASK) != MCG_S_CLKST_FLL)
		nop();

	/* 8. Stabilization delay */
	for (uint32_t i = 30000; i; --i)
		nop();

	/* 9. Configure clock dividers for the system:
	 *    Core/system    = MCGOUTCLK / 1, 96MHz
	 *    Bus clock      = System clock / 4, 24MHz
	 *    Flash clock    = System clock / 4, 24MHz
	 *    Fast bus clock = System clock / 2, 48MHz
	 */
	SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(1) | SIM_CLKDIV1_OUTDIV2(4) |
		      SIM_CLKDIV1_OUTDIV4(4) | SIM_CLKDIV1_OUTDIV5(2);

	/* 10. Select IRC48M as the PLLFLL peripheral clock source, do not
	 * divide, select for TPM and LPUART */
	SIM_SOPT2 = (SIM_SOPT2 & ~(SIM_SOPT2_PLLFLLSEL_MASK |
				   SIM_SOPT2_LPUARTSRC_MASK |
				   SIM_SOPT2_TPMSRC_MASK)) |
		    SIM_SOPT2_PLLFLLSEL_IRC48M |
		    SIM_SOPT2_TPMSRC(MCG) | SIM_SOPT2_LPUARTSRC(MCG);
	SIM_CLKDIV3 &= ~(SIM_CLKDIV3_PLLFLLFRAC | SIM_CLKDIV3_PLLFLLDIV_MASK);

	/* 11. Select the 32 kHz system oscillator for various peripherals */
	BME_BITFIELD(SIM_SOPT1, SIM_SOPT1_OSC32KSEL_MASK) =
		SIM_SOPT1_OSC32KSEL_SYS;
}

static void __section(".startup") platform_init(void)
{
	/* initialize system clocks to 96 MHz from FLL */
	if (BOOTLOADER_BUILD)
		clock_init();

	/*
	 * Relocate code + data to RAM (everything but .startup section).
	 * We want to run from RAM because on MKL it's not possible to program
	 * the flash memory while executing from it.
	 */
	for (uint32_t *src = &_sfreloc, *dst = &_sreloc; dst < &_ereloc;)
		*dst++ = *src++;

	/* zero out bss */
	for (uint32_t *ptr = &_sbss; ptr < &_ebss; ++ptr)
		*ptr = 0;

	/* set vector table offset */
	SCB->VTOR = (uint32_t)&isr_vector[0];

	/* disable and clear all NVIC interrupts */
	nvic_disable_all_and_clear_pending();

	/* disable SysTick, clear pending */
	systick_disable_clear();

	/* initialize PIT */
	pit_init();
}
