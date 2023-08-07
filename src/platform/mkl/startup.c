#include "cpu.h"
#include "memory_layout.h"
#include "reset.h"
#include "timer.h"
#include "mpu.h"

extern uint32_t _stack_top, _psp_top, _sfreloc, _sreloc, _ereloc, _sbss, _ebss;
extern void __noreturn main(void);

static void platform_init(void);
static void unprivileged_main(void);

void __noreturn __naked __section(".startup")
reset_handler(void)
{
	disable_irq();

	set_msp((uint32_t)&_stack_top);

	platform_init();

	/* branch instead of call so that nothing is pushed to stack */
	asm volatile("bx %0\n\t" : : "r" (unprivileged_main));
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
	NULL,					/* I2C module 0 */
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

static __force_inline void aips_reset(void)
{
	AIPS_PACR(0) = 0x54444444;
	for (unsigned i = 8; i < 128; i += 8)
		AIPS_PACR(i) = 0x44444444;
}

static __force_inline void aips_permit_user(AIPS_Slot_Type slot)
{
	BME_AND(AIPS_PACR(slot)) = ~AIPS_PACR_SP(slot);
}

static void __section(".startup") config_mpu_region(mpu_region_t reg,
						    const void *start,
						    const void *end,
						    uint32_t access)
{
	MPU_RGDn_WORD0(reg) = (uint32_t)start & MPU_RGDn_WORD0_SRTADDR;
	MPU_RGDn_WORD1(reg) = ((uint32_t)end & MPU_RGDn_WORD1_ENDADDR) - 1;
	MPU_RGDn_WORD2(reg) = access;
	MPU_RGDn_WORD3(reg) = MPU_RGDn_WORD3_VLD;
	isb();
	dsb();
}

static void __section(".startup") config_mpu(void)
{
	extern uint32_t _stext, _etext, _sdata, _psp_bottom,
			_sfirmwareflashdata, _efirmwareflashdata;

	/* supervisor rwx everywhere */
	MPU_RGDAACn(MPU_REGION_SUPERVISOR) = MPU_RGDn_WORD2_MnSM_RWX(0);

	/* the rest is for user mode */

	/* headers (checksum, features, cfg area): read only */
	config_mpu_region(MPU_REGION_HEADERS, &_sreloc, &_stext,
			  MPU_RGDn_WORD2_MnUM_R(0));

	/* code + rodata: read + execute */
	config_mpu_region(MPU_REGION_TEXT, &_stext, &_etext,
			  MPU_RGDn_WORD2_MnUM_RX(0));

	/* data + bss: read + write */
	config_mpu_region(MPU_REGION_DATA_BSS, &_sdata, &_ebss,
			  MPU_RGDn_WORD2_MnUM_RW(0));

	/*
	 * peripherals: read + write (AIPS configuration allows only some
	 * peripherals)
	 */
	config_mpu_region(MPU_REGION_PERIPHERALS, (void *)0x40020000,
			  (void *)0x60000000, MPU_RGDn_WORD2_MnUM_RW(0));

	/* flash: read only */
	config_mpu_region(MPU_REGION_FLASH, (void *)0x0, (void *)0x20000,
			  MPU_RGDn_WORD2_MnUM_R(0));

	/*
	 * flashing buffer: read + write, write can be prohibited when flashing,
	 * see firmware_flash.c
	 */
	config_mpu_region(MPU_REGION_FLASHING_BUFFER, &_sfirmwareflashdata,
			  &_efirmwareflashdata, MPU_RGDn_WORD2_MnUM_RW(0));

	/* process stack: read + write */
	config_mpu_region(MPU_REGION_PROCESS_STACK, &_psp_bottom, &_psp_top,
			  MPU_RGDn_WORD2_MnUM_RW(0));
}

static void __noinline __section(".startup") platform_init(void)
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

	/*
	 * reset peripheral access and permit access to these peripherals from
	 * unprivileged mode
	 */
	aips_reset();
	aips_permit_user(GPIO_Slot);
	aips_permit_user(CRC_Slot);
	aips_permit_user(PIT0_Slot);
	aips_permit_user(TPM0_Slot);
	aips_permit_user(TPM1_Slot);
	aips_permit_user(TPM2_Slot);
	aips_permit_user(SPI1_Slot);
	aips_permit_user(PTA_Slot);
	aips_permit_user(PTB_Slot);
	aips_permit_user(PTC_Slot);
	aips_permit_user(PTD_Slot);
	aips_permit_user(PTE_Slot);
	aips_permit_user(LPUART0_Slot);
	aips_permit_user(I2C0_Slot);
	aips_permit_user(RFSYS_Slot);

	/* enable all port clocks */
	BME_OR(SIM_SCGC5) = SIM_SCGC5_PTA | SIM_SCGC5_PTB | SIM_SCGC5_PTC |
			    SIM_SCGC5_PTD | SIM_SCGC5_PTE;

	/* configure SVC priority */
	nvic_set_priority(SVC_IRQn, 3);

	/* configure memory protection */
	config_mpu();

	/* set process stack */
	set_psp((uint32_t)&_psp_top);
}

static __noinline __naked void unprivileged_main(void)
{
	/* enable interrupts */
	enable_irq();

	/* go to unprivileged mode, use process stack pointer */
	set_control(CONTROL_nPRIV | CONTROL_SPSEL);
	isb();

	/* branch instead of call so that nothing is pushed to stack */
	asm volatile("bx %0\n\t" : : "r" (main));
}
