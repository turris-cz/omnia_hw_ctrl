#ifndef TIMER_H
#define TIMER_H

#include "cpu.h"

typedef uint8_t timer_nr_t;

typedef enum {
	timer_periph_tpm,
	timer_periph_pit,
} timer_periph_t;

#define TIMER_NR_NUM_MASK	GENMASK8(3, 0)
#define TIMER_NR_NUM(x)		FIELD_PREP(TIMER_NR_NUM_MASK, x)
#define TIMER_NR_PERIPH_MASK	GENMASK8(6, 4)
#define TIMER_NR_PERIPH(x)	FIELD_PREP(TIMER_NR_PERIPH_MASK, x)
#define TIMER_NR_HAS_SETFREQ	BIT8(7)

#define TIMER_NR(_periph, _n, _has_setfreq)				\
	((timer_nr_t)(TIMER_NR_NUM(_n) |				\
		      TIMER_NR_PERIPH(_periph) |			\
		      ((_has_setfreq) ? TIMER_NR_HAS_SETFREQ : 0)))

#define LED_TIMER		TIMER_NR(timer_periph_tpm, 1, true)
#define LED_PATTERN_TIMER	TIMER_NR(timer_periph_tpm, 2, false)
#define LED_PWM_TIMER		TIMER_NR(timer_periph_tpm, 0, false)
#define USB_TIMEOUT_TIMER	TIMER_NR(timer_periph_pit, 0, false)

/* this is set in startup.c in clock_config() */
#define TPM_PARENT_FREQ		48000000U
#define PIT_PARENT_FREQ		24000000U

/*
 * Currently we use PIT timer only for the USB power control timeout,
 * which we init with priority 3.
 */
#define PIT_IRQ_PRIO		3

static __force_inline timer_periph_t timer_periph(timer_nr_t tim_nr)
{
	return FIELD_GET(TIMER_NR_PERIPH_MASK, tim_nr);
}

static __force_inline uint8_t timer_nr(timer_nr_t tim_nr)
{
	return FIELD_GET(TIMER_NR_NUM_MASK, tim_nr);
}

static __force_inline bool timer_has_setfreq(timer_nr_t tim_nr)
{
	return tim_nr & TIMER_NR_HAS_SETFREQ;
}

static __force_inline void timer_clk_config_tpm(uint8_t nr, bool on)
{
	switch (nr) {
#define _TIMER_CLK_CFG(_n)					\
	case _n:						\
		BME_BITFIELD(SIM_SCGC6, SIM_SCGC6_TPM ## _n) =	\
			on ? SIM_SCGC6_TPM ## _n : 0;		\
		break;
	_TIMER_CLK_CFG(0)
	_TIMER_CLK_CFG(1)
	_TIMER_CLK_CFG(2)
#undef _TIMER_CLK_CFG
	default:
		unreachable();
	}
}

static __force_inline void timer_reset(timer_nr_t tim_nr)
{
	uint8_t nr = timer_nr(tim_nr);

	switch (timer_periph(tim_nr)) {
	case timer_periph_tpm:
		timer_clk_config_tpm(nr, false);
		timer_clk_config_tpm(nr, true);

		BME_BITFIELD(TPM_SC(nr), TPM_SC_CMOD_MASK) = TPM_SC_CMOD_DIS;
		TPM_SC(nr) = TPM_SC_TOF;
		TPM_CNT(nr) = 0;
		TPM_MOD(nr) = 0xffff;
		for (unsigned ch = 0; ch <= 5; ++ch) {
			TPM_CnSC(nr, 0) = TPM_CnSC_CHF;
			TPM_CnV(nr, 0) = 0;
		}
		TPM_STATUS(nr) = GENMASK(5, 0);
		TPM_COMBINE(nr) = 0;
		TPM_POL(nr) = 0;
		TPM_FILTER(nr) = 0;
		break;

	case timer_periph_pit:
		PIT_TCTRL(nr) = 0;
		PIT_TFLG(nr) = PIT_TFLG_TIF;
		PIT_LDVAL(nr) = 0;
		break;

	default:
		unreachable();
	}
}

static __force_inline uint8_t tpm_irqn(uint8_t nr)
{
	switch (nr) {
	case 0: return TPM0_IRQn;
	case 1: return TPM1_IRQn;
	case 2: return TPM2_IRQn;
	default: unreachable();
	}
}

static __force_inline uint8_t tpm_pwm_freq2psc(uint32_t freq)
{
	uint32_t psc = TPM_PARENT_FREQ / freq;

	/*
	 * For TPM timers used as PWM, we can achieve the requested frequency
	 * only via prescaling, since the TPM_MOD register is used for PWM
	 * period. So the only supported frequencies are TPM_PARENT_FREQ / 2^N,
	 * where N is an integer from 0 to 7.
	 */
	compiletime_assert(is_power_of_2(psc) && psc <= 128,
			   "Requested frequency unachievable for PWM timer (prescaler must be a power of 2 and at most 128)");

	return __bf_shf(psc);
}

static __force_inline uint8_t tpm_freq2psc(bool has_setfreq, uint32_t freq)
{
	if (has_setfreq)
		/*
		 * Always use prescaler 0 (no prescaling of parent clock) if
		 * timer_set_freq() is to be supported on this timer.
		 *
		 * This is because we want timer_set_freq() to be as simple as
		 * possible: we do not want to need to disable the timer when
		 * changing frequency. But changing the prescaler requires the
		 * timer to be disabled.
		 *
		 * Note that because the TPM_MOD register is only 16 bits wide
		 * and we do not use prescaler, TPM timers with setfreq
		 * supported can achieve only frequencies at least
		 *   TPM_PARENT_FREQ / 2^16.
		 */
		return 0;
	else
		return MIN(7, __bf_shf(TPM_PARENT_FREQ / freq));
}

static __force_inline uint16_t tpm_freq2mod(bool has_setfreq, uint32_t freq)
{
	uint32_t mod = (TPM_PARENT_FREQ / freq) /
		       (1 << tpm_freq2psc(has_setfreq, freq));

	compiletime_assert(TPM_PARENT_FREQ % freq == 0,
			   "Requested frequency unachievable");

	compiletime_assert(mod <= 65536,
			   "Requested frequency unachievable (computed modulo too high)");

	return mod - 1;
}

static __force_inline uint32_t pit_freq2ldval(uint32_t freq)
{
	compiletime_assert(PIT_PARENT_FREQ % freq == 0,
			   "Requested frequency unachievable");

	return PIT_PARENT_FREQ / freq - 1;
}

static __force_inline void _init_tpm(uint8_t nr)
{
	timer_clk_config_tpm(nr, false);
	timer_clk_config_tpm(nr, true);

	TPM_SC(nr) = TPM_SC_CMOD_DIS;
	TPM_CNT(nr) = 0;
}

static __force_inline void _timer_init_tpm(uint8_t nr, bool has_setfreq,
					   uint32_t freq, uint8_t irq_prio)
{
	_init_tpm(nr);

	TPM_CnSC(nr, 0) = 0;
	TPM_SC(nr) = TPM_SC_TOIE | TPM_SC_PS(tpm_freq2psc(has_setfreq, freq));
	TPM_MOD(nr) = tpm_freq2mod(has_setfreq, freq);

	nvic_enable_irq_with_prio(tpm_irqn(nr), irq_prio);
}

static __force_inline void _timer_init_pit(uint8_t nr, uint32_t freq,
					   uint8_t irq_prio)
{
	uint32_t tctrl;

	compiletime_assert(irq_prio == PIT_IRQ_PRIO,
			   "PIT timer must have PIT_IRQ_PRIO as interrupt priority");

	tctrl = PIT_TCTRL(nr) & ~PIT_TCTRL_TEN;
	PIT_TCTRL(nr) = tctrl;
	PIT_LDVAL(nr) = pit_freq2ldval(freq);
	PIT_TCTRL(nr) = (tctrl & ~PIT_TCTRL_CHN) | PIT_TCTRL_TIE;
}

static __force_inline void timer_init(timer_nr_t tim_nr, uint32_t freq,
				      uint8_t irq_prio)
{
	switch (timer_periph(tim_nr)) {
	case timer_periph_tpm:
		return _timer_init_tpm(timer_nr(tim_nr),
				       timer_has_setfreq(tim_nr), freq,
				       irq_prio);

	case timer_periph_pit:
		return _timer_init_pit(timer_nr(tim_nr), freq, irq_prio);

	default:
		unreachable();
	}
}

static __force_inline void timer_init_pwm(timer_nr_t tim_nr, uint32_t freq,
					  uint16_t period)
{
	uint8_t nr = timer_nr(tim_nr);

	compiletime_assert(timer_periph(tim_nr) == timer_periph_tpm,
			   "Only TPM timer can be used for PWM");

	_init_tpm(timer_nr(tim_nr));

	TPM_CnSC(nr, 0) = TPM_CnSC_MSB | TPM_CnSC_ELSA;
	TPM_SC(nr) = TPM_SC_PS(tpm_pwm_freq2psc(freq));
	TPM_MOD(nr) = period - 1;
}

static __force_inline void timer_enable(timer_nr_t tim_nr, bool on)
{
	uint8_t nr = timer_nr(tim_nr);

	switch (timer_periph(tim_nr)) {
	case timer_periph_tpm:
		BME_BITFIELD(TPM_SC(nr), TPM_SC_CMOD_MASK) =
			on ? TPM_SC_CMOD_CLK : TPM_SC_CMOD_DIS;

		/* clear counter on disable */
		if (!on)
			TPM_CNT(nr) = 0;
		break;

	case timer_periph_pit:
		BME_BITFIELD(PIT_TCTRL(nr), PIT_TCTRL_TEN) =
			on ? PIT_TCTRL_TEN : 0;

		/*
		 * No need to clear counter on disable, because on enable, the
		 * PIT timer will load the LDVAL register.
		 */
		break;

	default:
		unreachable();
	}
}

static __force_inline void timer_irq_enable(timer_nr_t tim_nr, bool on)
{
	uint8_t nr = timer_nr(tim_nr);

	switch (timer_periph(tim_nr)) {
	case timer_periph_tpm:
		BME_BITFIELD(TPM_SC(nr), TPM_SC_TOIE) =
			on ? TPM_SC_TOIE : 0;
		break;

	case timer_periph_pit:
		BME_BITFIELD(PIT_TCTRL(nr), PIT_TCTRL_TIE) =
			on ? PIT_TCTRL_TIE : 0;
		break;

	default:
		unreachable();
	}
}

static __force_inline bool timer_irq_clear_up(timer_nr_t tim_nr)
{
	switch (timer_periph(tim_nr)) {
	case timer_periph_tpm:
		return BME_LOAD_SET(TPM_SC(timer_nr(tim_nr)), TPM_SC_TOF);

	case timer_periph_pit:
		return BME_LOAD_SET(PIT_TFLG(timer_nr(tim_nr)), PIT_TFLG_TIF);

	default:
		unreachable();
	}
}

static __force_inline void timer_set_freq(timer_nr_t tim_nr, uint32_t freq)
{
	compiletime_assert(timer_periph(tim_nr) != timer_periph_tpm ||
			   timer_has_setfreq(tim_nr),
			   "Timer does not support timer_set_freq()");

	switch (timer_periph(tim_nr)) {
	case timer_periph_tpm:
		TPM_MOD(timer_nr(tim_nr)) = tpm_freq2mod(true, freq);
		break;

	case timer_periph_pit:
		PIT_LDVAL(timer_nr(tim_nr)) = pit_freq2ldval(freq);
		break;

	default:
		unreachable();
	}
}

static __force_inline uint32_t timer_get_counter(timer_nr_t tim_nr)
{
	switch (timer_periph(tim_nr)) {
	case timer_periph_tpm: return TPM_CNT(timer_nr(tim_nr));
	case timer_periph_pit: return PIT_CVAL(timer_nr(tim_nr));
	default: unreachable();
	}
}

static __force_inline void timer_set_pwm_duty(timer_nr_t tim_nr, uint16_t duty)
{
	compiletime_assert(timer_periph(tim_nr) == timer_periph_tpm,
			   "Only TPM timer supports timer_set_pwm_duty()");

	TPM_CnV(timer_nr(tim_nr), 0) = duty;
}

#endif /* TIMER_H */
