#include "time.h"
#include "power_control.h"
#include "input.h"
#include "watchdog.h"
#include "i2c_iface.h"

#define SYSTICK_PERIOD		(SYS_CORE_FREQ / HZ)

/* we want SYSTICK_PERIOD to be precise */
_Static_assert(SYS_CORE_FREQ % HZ == 0,
	       "SYS_CORE_FREQ must be divisible by HZ");

volatile uint32_t jiffies __unprivileged_rodata;

#if POWEROFF_WAKEUP_ENABLED
volatile uint32_t uptime __unprivileged_rodata;
#endif

/*******************************************************************************
  * @function   time_config
  * @brief      Setup SysTick Timer interrupt to fire every 5 ms.
  * @param      None
  * @retval     None
  *****************************************************************************/
__privileged void time_config(void)
{
	if (!systick_config(SYSTICK_PERIOD)) {
		debug("Failed configuring SysTick\n");
		/* Capture error */
		while (1);
	}

	nvic_set_priority(SysTick_IRQn, 2);
}

/* Read current value of SysTick counter and compute in how many milliseconds
 * will SysTick fire.
 * To avoid division we use comparing instead. This should be faster.
 */
static __privileged uint32_t next_tick_in(void)
{
	uint32_t res, cmp, ticks = SysTick->VAL;

	for (res = 0, cmp = 0;
	     res < JIFFY_TO_MSECS;
	     ++res, cmp += MSEC_TO_TICKS)
		if (ticks < cmp)
			break;

	return res;
}

static __privileged void wait_for_systick(void)
{
	uint32_t jiffies_at_beginning = jiffies;

	do
		wait_for_interrupt();
	while (jiffies_at_beginning == jiffies);
}

/******************************************************************************
  * @function   msleep
  * @brief      Sleeps / delays for a given time.
  * @param      ms: specifies the delay time length, in miliseconds.
  * @retval     None
  *****************************************************************************/
__privileged void msleep(uint32_t ms)
{
	uint32_t next;

	next = next_tick_in();
	if (ms < next) {
		/* if next tick comes later than we want, busy delay */
		mdelay(ms);
		return;
	}

	wait_for_systick();
	ms -= next;

	while (ms >= JIFFY_TO_MSECS) {
		wait_for_systick();
		ms -= JIFFY_TO_MSECS;
	}

	/* busy delay the rest */
	mdelay(ms);
}

static __privileged void uptime_handler(void)
{
	static uint8_t cntr __privileged_data;

	if (++cntr == HZ) {
		cntr = 0;
		++uptime;
	}
}

/******************************************************************************
  * @function   systick_irq_handler
  * @brief      Decrements the TimingDelay variable in System Timer and
  *             takes care of watchdog timeout.
  * @param      None
  * @retval     None
  *****************************************************************************/
void __irq __privileged systick_irq_handler(void)
{
	jiffies++;

	if (POWEROFF_WAKEUP_ENABLED)
		uptime_handler();
	watchdog_handler();
	button_debounce_handler();
	i2c_slave_recovery_handler(SLAVE_I2C);
}
