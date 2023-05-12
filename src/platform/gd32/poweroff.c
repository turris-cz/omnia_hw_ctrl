#if POWEROFF_WAKEUP_ENABLED

#include "cpu.h"
#include "debug.h"
#include "pin_defs.h"
#include "gd32f1x0_exti.h"
#include "gd32f1x0_syscfg.h"
#include "gd32f1x0_pmu.h"
#include "gd32f1x0_rtc.h"
#include "timer.h"

#define EXTI_RTC_BIT	BIT(17)

static uint32_t wakeup_timeout_left;
static uint32_t last_alarm_td;

static __force_inline void syscfg_exti_ctrl(gpio_t pin)
{
	volatile uint32_t *reg;
	unsigned shift, idx;
	uint32_t cr;

	if (pin == PIN_INVALID)
		return;

	switch (pin_port(pin)) {
	case PORT_A: cr = 0; break;
	case PORT_B: cr = 1; break;
	case PORT_C: cr = 2; break;
	case PORT_D: cr = 3; break;
	case PORT_F: cr = 5; break;
	default: unreachable();
	}

	shift = (pin_nr(pin) & 3) * 4;
	idx = pin_nr(pin) >> 2;
	reg = &(&SYSCFG_EXTISS0)[idx];

	*reg = (*reg & ~(0xf << shift)) | (cr << shift);
}

static void config_exti(void)
{
	EXTI_INTEN = 0;
	EXTI_EVEN = 0;
	EXTI_RTEN = 0;
	EXTI_FTEN = 0;
}

void __irq exti_irq_handler(void)
{
	nvic_system_reset();
}

static void config_wakeup_by_button(void)
{
	/* put CFG out of reset and enable clock */
	RCU_APB2RST &= ~RCU_APB2RST_CFGCMPRST;
	RCU_APB2EN |= RCU_APB2EN_CFGCMPEN;

	/* enable port clock */
	RCU_AHBEN |= port_clk_bit(pin_port(FRONT_BTN_PIN));

	/* configure front button falling edge interrupt */
	syscfg_exti_ctrl(FRONT_BTN_PIN);
	EXTI_INTEN |= pin_bit(FRONT_BTN_PIN);
	EXTI_FTEN |= pin_bit(FRONT_BTN_PIN);

	/* enable interrupt */
	nvic_enable_irq_with_prio(pin_irqn(FRONT_BTN_PIN), 0);
}

static void rtc_config_lsi_clock(void)
{
	uint32_t bdctl = RCU_BDCTL;

	/* enable IRC40K oscillator and wait for readiness */
	RCU_RSTSCK |= RCU_RSTSCK_IRC40KEN;
	while (!(RCU_RSTSCK & RCU_RSTSCK_IRC40KSTB))
		nop();

	/* reset RTC domain */
	bdctl |= RCU_BDCTL_BKPRST;
	RCU_BDCTL = bdctl;
	bdctl &= ~RCU_BDCTL_BKPRST;
	RCU_BDCTL = bdctl;

	/* disable RTC if enabled */
	bdctl &= ~RCU_BDCTL_RTCEN;
	RCU_BDCTL = bdctl;

	/* select IRC40K as RTC clock */
	bdctl = (bdctl & ~RCU_BDCTL_RTCSRC) | RCU_RTCSRC_IRC40K;
	RCU_BDCTL = bdctl;

	/* enable RTC */
	bdctl |= RCU_BDCTL_RTCEN;
	RCU_BDCTL = bdctl;
}

static void rtc_init_zero(void)
{
	/* initialize programming date and time registers */
	RTC_STAT |= RTC_STAT_INITM;
	while (!(RTC_STAT & RTC_STAT_INITF))
		nop();

	/* configure prescalers for 1 Hz clock (40,000 Hz / 125 / 320 = 1) */
	RTC_PSC = FIELD_PREP(RTC_PSC_FACTOR_A, 125 - 1) |
		  FIELD_PREP(RTC_PSC_FACTOR_S, 320 - 1);

	/* program time and date registers */
	RTC_TIME = 0;
	RTC_DATE = FIELD_PREP(RTC_DATE_DOW, 1);

	/* set format to 24 hour */
	RTC_CTL &= ~RTC_CTL_CS;

	/* exit initialization mode */
	RTC_STAT &= ~RTC_STAT_INITM;
}

static void prepare_last_alarm_td(void)
{
	uint32_t t, h, m, s;

	t = wakeup_timeout_left & 0x7fff;

	h = t / 3600;
	m = (t % 3600) / 60;
	s = t % 60;

	if (h > 18 || m > 59 || s > 59)
		unreachable();

	last_alarm_td = RTC_ALRM0TD_MSKD |
			FIELD_PREP(RTC_ALRM0TD_HRT, h / 10) |
			FIELD_PREP(RTC_ALRM0TD_HRU, h % 10) |
			FIELD_PREP(RTC_ALRM0TD_MNT, m / 10) |
			FIELD_PREP(RTC_ALRM0TD_MNU, m % 10) |
			FIELD_PREP(RTC_ALRM0TD_SCT, s / 10) |
			FIELD_PREP(RTC_ALRM0TD_SCU, s % 10);
}

static void config_rtc_alarm_time(bool force)
{
	/*
	 * early return if possible (we don't need to overwrite the registers if
	 * the alarm time does not change)
	 */
	if (!force && wakeup_timeout_left >= 0x10000) {
		wakeup_timeout_left -= 0x10000;
		return;
	}

	/* disable alarm */
	RTC_CTL &= ~RTC_CTL_ALRM0EN;
	while (!(RTC_STAT & RTC_STAT_ALRM0WF))
		nop();

	/* set time & date reg */
	RTC_ALRM0SS = 0;
	if (wakeup_timeout_left >= 0x10000) {
		RTC_ALRM0TD = RTC_ALRM0TD_MSKD | 0x181216;
		wakeup_timeout_left -= 0x10000;
	} else {
		RTC_ALRM0TD = last_alarm_td;
		wakeup_timeout_left = 0;
	}

	/* clear alarm flag */
	RTC_STAT &= ~RTC_STAT_ALRM0F;

	/* set output select */
	RTC_CTL = (RTC_CTL & ~RTC_CTL_OS) | FIELD_PREP(RTC_CTL_OS, 1);

	/* enable alarm and alarm interrupt */
	RTC_CTL |= RTC_CTL_ALRM0EN | RTC_CTL_ALRM0IE;
}

static __force_inline bool rtc_alarm_clear_up(void)
{
	uint32_t stat = RTC_STAT;
	bool up = stat & RTC_STAT_ALRM0F;

	if (up)
		RTC_STAT = stat & ~RTC_STAT_ALRM0F;

	return up;
}

void __irq rtc_irq_handler(void)
{
	/* clear EXTI RTC event pending bit */
	EXTI_PD = EXTI_RTC_BIT;

	if (!rtc_alarm_clear_up())
		return;

	if (!wakeup_timeout_left)
		nvic_system_reset();

	rtc_init_zero();
	config_rtc_alarm_time(false);
}

static void config_wakeup_by_rtc(uint32_t wakeup_timeout)
{
	/* disable RTC domain write protection */
	PMU_CTL |= PMU_CTL_BKPWEN;

	rtc_config_lsi_clock();

	/* unlock RTC registers write protection */
	RTC_WPK = 0xca;
	RTC_WPK = 0x53;

	wakeup_timeout_left = wakeup_timeout;
	prepare_last_alarm_td();
	rtc_init_zero();
	config_rtc_alarm_time(true);

	/* configure RTC alarm interrupt */
	EXTI_INTEN |= EXTI_RTC_BIT;
	EXTI_RTEN |= EXTI_RTC_BIT;

	/* enable interrupt */
	nvic_enable_irq_with_prio(RTC_IRQn, 0);
}

static void slow_down_system_clock(void)
{
	/* set system clock source to IRC8M */
	RCU_CFG0 = (RCU_CFG0 & ~RCU_CFG0_SCS) | RCU_CKSYSSRC_IRC8M;
	while ((RCU_CFG0 & RCU_CFG0_SCSS) != RCU_SCSS_IRC8M)
		nop();

	/* disable PLL clock */
	RCU_CTL0 &= ~RCU_CTL0_PLLEN;
}

void platform_poweroff(bool enable_button, uint32_t wakeup_timeout)
{
	nvic_disable_all_and_clear_pending();

	/* disable unneeded peripheral clocks */
	RCU_AHBEN = 0;
	RCU_APB1EN = 0;
	RCU_APB2EN = 0;

	/* put PWR out of reset and enable clock */
	RCU_APB1RST &= ~RCU_APB1RST_PMURST;
	RCU_APB1EN |= RCU_APB1EN_PMUEN;

	/* ensure that WFI enters STOP mode */
	SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
	PMU_CTL = (PMU_CTL & ~PMU_CTL_STBMOD) | PMU_CTL_LDOLP;

	config_exti();

	if (enable_button)
		config_wakeup_by_button();

	if (wakeup_timeout)
		config_wakeup_by_rtc(wakeup_timeout);

	slow_down_system_clock();

	enable_irq();

	while (1)
		wait_for_interrupt();
}

#endif /* POWEROFF_WAKEUP_ENABLED */
