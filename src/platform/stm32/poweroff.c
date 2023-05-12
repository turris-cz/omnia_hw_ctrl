#include "cpu.h"
#include "pin_defs.h"

#define EXTI_RTC_BIT	BIT(17)

static uint32_t wakeup_timeout_left;
static uint32_t last_alarm_td;

static __force_inline void syscfg_exti_ctrl(gpio_t pin)
{
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

	SYSCFG->EXTICR[idx] = (SYSCFG->EXTICR[idx] & ~(0xf << shift)) |
			      (cr << shift);
}

static void config_exti(void)
{
	EXTI->IMR = 0;
	EXTI->EMR = 0;
	EXTI->RTSR = 0;
	EXTI->FTSR = 0;
}

void __irq exti_irq_handler(void)
{
	nvic_system_reset();
}

static void config_wakeup_by_button(void)
{
	/* put SYSCFG out of reset and enable clock */
	RCC->APB2RSTR &= ~RCC_APB2RSTR_SYSCFGRST;
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

	/* enable port clock */
	RCC->AHBENR |= port_clk_bit(pin_port(FRONT_BTN_PIN));

	/* configure front button falling edge interrupt */
	syscfg_exti_ctrl(FRONT_BTN_PIN);
	EXTI->IMR |= pin_bit(FRONT_BTN_PIN);
	EXTI->FTSR |= pin_bit(FRONT_BTN_PIN);

	/* enable interrupt */
	nvic_enable_irq_with_prio(pin_irqn(FRONT_BTN_PIN), 0);
}

static void rtc_config_lsi_clock(void)
{
	uint32_t bdcr = RCC->BDCR;

	/* enable LSI oscillator and wait for readiness */
	RCC->CSR |= RCC_CSR_LSION;
	while (!(RCC->CSR & RCC_CSR_LSIRDY))
		nop();

	/* reset RTC domain */
	bdcr |= RCC_BDCR_BDRST;
	RCC->BDCR = bdcr;
	bdcr &= ~RCC_BDCR_BDRST;
	RCC->BDCR = bdcr;

	/* disable RTC if enabled */
	bdcr &= ~RCC_BDCR_RTCEN;
	RCC->BDCR = bdcr;

	/* select LSI as RTC clock */
	bdcr = (bdcr & ~RCC_BDCR_RTCSEL) | RCC_BDCR_RTCSEL_LSI;
	RCC->BDCR = bdcr;

	/* enable RTC */
	bdcr |= RCC_BDCR_RTCEN;
	RCC->BDCR = bdcr;
}

static void rtc_init_zero(void)
{
	/* initialize programming date and time registers */
	RTC->ISR |= RTC_ISR_INIT;
	while (!(RTC->ISR & RTC_ISR_INITF))
		nop();

	/* configure prescalers for 1 Hz clock (40,000 Hz / 125 / 320 = 1) */
	RTC->PRER = FIELD_PREP(RTC_PRER_PREDIV_A, 125 - 1) |
		    FIELD_PREP(RTC_PRER_PREDIV_S, 320 - 1);

	/* program time and date registers */
	RTC->TR = 0;
	RTC->DR = FIELD_PREP(RTC_DR_WDU, 1);

	/* set format to 24 hour */
	RTC->CR &= ~RTC_CR_FMT;

	/* exit initialization mode */
	RTC->ISR &= ~RTC_ISR_INIT;
}

static void prepare_last_alarm_td(void)
{
	uint32_t t, h, m, s;

	t = wakeup_timeout_left & 0xffff;

	h = t / 3600;
	m = (t % 3600) / 60;
	s = t % 60;

	if (h > 18 || m > 59 || s > 59)
		unreachable();

	last_alarm_td = RTC_ALRMAR_MSK4 |
			FIELD_PREP(RTC_ALRMAR_HT, h / 10) |
			FIELD_PREP(RTC_ALRMAR_HU, h % 10) |
			FIELD_PREP(RTC_ALRMAR_MNT, m / 10) |
			FIELD_PREP(RTC_ALRMAR_MNU, m % 10) |
			FIELD_PREP(RTC_ALRMAR_ST, s / 10) |
			FIELD_PREP(RTC_ALRMAR_SU, s % 10);
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
	RTC->CR &= ~RTC_CR_ALRAE;
	while (!(RTC->ISR & RTC_ISR_ALRAWF))
		nop();

	/* set time & date reg */
	RTC->ALRMASSR = 0;
	if (wakeup_timeout_left >= 0x10000) {
		RTC->ALRMAR = RTC_ALRMAR_MSK4 | 0x181216;
		wakeup_timeout_left -= 0x10000;
	} else {
		RTC->ALRMAR = last_alarm_td;
		wakeup_timeout_left = 0;
	}

	/* clear alarm flag */
	RTC->ISR &= ~RTC_ISR_ALRAF;

	/* set output select */
	RTC->CR = (RTC->CR & ~RTC_CR_OSEL) | FIELD_PREP(RTC_CR_OSEL, 1);

	/* enable alarm and alarm interrupt */
	RTC->CR |= RTC_CR_ALRAE | RTC_CR_ALRAIE;
}

static __force_inline bool rtc_alarm_clear_up(void)
{
	uint32_t isr = RTC->ISR;
	bool up = isr & RTC_ISR_ALRAF;

	if (up)
		RTC->ISR = isr & ~RTC_ISR_ALRAF;

	return up;
}

void __irq rtc_irq_handler(void)
{
	/* clear EXTI RTC event pending bit */
	EXTI->PR = EXTI_RTC_BIT;

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
	PWR->CR |= PWR_CR_DBP;

	rtc_config_lsi_clock();

	/* unlock RTC registers write protection */
	RTC->WPR = 0xca;
	RTC->WPR = 0x53;

	wakeup_timeout_left = wakeup_timeout;
	prepare_last_alarm_td();
	rtc_init_zero();
	config_rtc_alarm_time(true);

	/* configure RTC alarm interrupt */
	EXTI->IMR |= EXTI_RTC_BIT;
	EXTI->RTSR |= EXTI_RTC_BIT;

	/* enable interrupt */
	nvic_enable_irq_with_prio(RTC_IRQn, 0);
}

static void slow_down_system_clock(void)
{
	/* set system clock source to HSI */
	RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_HSI;
	while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI)
		nop();

	/* disable PLL clock */
	RCC->CR &= ~RCC_CR_PLLON;
}

void platform_poweroff(bool enable_button, uint32_t wakeup_timeout)
{
	nvic_disable_all_and_clear_pending();

	/* disable unneeded peripheral clocks */
	RCC->AHBENR = 0;
	RCC->APB1ENR = 0;
	RCC->APB2ENR = 0;

	/* put PWR out of reset and enable clock */
	RCC->APB1RSTR &= ~RCC_APB1RSTR_PWRRST;
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;

	/* ensure that WFI enters STOP mode */
	SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
	PWR->CR = (PWR->CR & ~PWR_CR_PDDS) | PWR_CR_LPDS;

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
