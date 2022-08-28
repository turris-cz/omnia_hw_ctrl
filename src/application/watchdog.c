#include "watchdog.h"
#include "power_control.h"
#include "time.h"
#include "cpu.h"

_Static_assert(HZ % 10 == 0, "HZ must be divisible by 10");

static bool enabled;
static uint8_t systick_counter;
static uint16_t timeout, counter;

void watchdog_enable(bool on)
{
	systick_counter = 0;
	counter = timeout;
	enabled = on;
}

bool watchdog_is_enabled(void)
{
	return enabled;
}

void watchdog_set_timeout(uint16_t ds)
{
	timeout = ds;
	if (enabled) {
		systick_counter = 0;
		counter = timeout;
	}
}

uint16_t watchdog_get_timeleft(void)
{
	if (enabled)
		return counter;
	else
		return timeout;
}

void watchdog_handler(void)
{
	if (!enabled)
		return;

	systick_counter++;
	if (systick_counter == HZ / 10) {
		systick_counter = 0;
		counter--;
	}

	if (!counter) {
		power_control_set_startup_condition();
		power_control_disable_regulators();
		NVIC_SystemReset();
	}
}
