#include "cpu.h"
#include "pin_defs.h"

#define SLOW_MODE_PIT_PARENT_FREQ	32768U

void slow_down_system_clock(void)
{
	/* Select FLL, disable TPM and LPUART clock */
	SIM_SOPT2 = (SIM_SOPT2 & ~(SIM_SOPT2_PLLFLLSEL_MASK |
				   SIM_SOPT2_LPUARTSRC_MASK |
				   SIM_SOPT2_TPMSRC_MASK)) |
		    SIM_SOPT2_PLLFLLSEL_FLL |
		    SIM_SOPT2_TPMSRC(DIS) | SIM_SOPT2_LPUARTSRC(DIS);

	/* Set CLKDIV1 to safe divider value */
	SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(2) | SIM_CLKDIV1_OUTDIV2(6) |
		      SIM_CLKDIV1_OUTDIV4(6) | SIM_CLKDIV1_OUTDIV5(2);

	/* Go to FLL Bypassed Internal mode */
	MCG_C2 &= ~MCG_C2_IRCS;
	MCG_C1 = (MCG_C1 & ~MCG_C1_CLKS_MASK) |
		 MCG_C1_CLKS_INTREF | MCG_C1_IREFS | MCG_C1_IRCLKEN;
	while ((MCG_S & (MCG_S_CLKST_MASK | MCG_S_IREFST)) !=
	       (MCG_S_CLKST_INTREF | MCG_S_IREFST))
		nop();

	/* Go to Bypassed Low Power Internal mode */
	BME_OR(MCG_C2) = MCG_C2_LP;

	/* Set CLKDIV1 to all 1s */
	SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(1) | SIM_CLKDIV1_OUTDIV2(1) |
		      SIM_CLKDIV1_OUTDIV4(1) | SIM_CLKDIV1_OUTDIV5(1);

	/* Configure VLPR power mode */
	BME_OR(SMC_PMPROT) = SMC_PMPROT_AVLP;
	BME_BITFIELD(SMC_PMCTRL, SMC_PMCTRL_RUNM_MASK) = SMC_PMCTRL_RUNM_VLPR;
}

void __irq port_irq_handler(void)
{
	nvic_system_reset();
}

static void config_wakeup_by_button(void)
{
	/* enable port clock */
	BME_OR(SIM_SCGC5) = port_clk_bit(pin_port(FRONT_BTN_PIN));

	/* configure falling edge interrupt */
	BME_BITFIELD(PORT_PCR(pin_port(FRONT_BTN_PIN), pin_nr(FRONT_BTN_PIN)),
		     PORT_PCR_IRQC_MASK) = PORT_PCR_IRQC_FALLING;

	/* enable interrupt */
	nvic_enable_irq_with_prio(pin_irqn(FRONT_BTN_PIN), 0);
}

void __irq pit_wakeup_irq_handler(void)
{
	if (BME_LOAD_SET(PIT_TFLG(1), PIT_TFLG_TIF))
		nvic_system_reset();
}

static void config_wakeup_by_pit(uint32_t wakeup_timeout)
{
	/* enable PIT clock */
	BME_OR(SIM_SCGC6) = SIM_SCGC6_PIT0;

	/*
	 * Configure PIT channel 0 to tick every second, and chain channel 1
	 * to channel 0. Thus we will get channel 1 interrupt after
	 * @wakeup_timeout seconds.
	 */
	PIT_TCTRL(0) = 0;
	PIT_TCTRL(1) = 0;

	PIT_LDVAL(0) = SLOW_MODE_PIT_PARENT_FREQ - 1;
	PIT_LDVAL(1) = wakeup_timeout - 1;

	PIT_TCTRL(0) = PIT_TCTRL_TEN;
	PIT_TCTRL(1) = PIT_TCTRL_TEN | PIT_TCTRL_TIE | PIT_TCTRL_CHN;

	/* enable interrupt */
	nvic_enable_irq_with_prio(PIT0_IRQn, 0);
}

void platform_poweroff(bool enable_button, uint32_t wakeup_timeout)
{
	nvic_disable_all_and_clear_pending();

	/* disable unneeded peripherals */
	SIM_SCGC4 = 0xf0000030 | SIM_SCGC4_VREF;
	SIM_SCGC5 = 0x40182;
	SIM_SCGC6 = SIM_SCGC6_NVM;

	if (enable_button)
		config_wakeup_by_button();

	if (wakeup_timeout)
		config_wakeup_by_pit(wakeup_timeout);

	slow_down_system_clock();

	enable_irq();

	while (1)
		wait_for_interrupt();
}
