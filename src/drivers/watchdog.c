#include "watchdog.h"
#include "power_control.h"
#include "time.h"

_Static_assert(HZ % 10 == 0, "HZ must be divisible by 10");

static bool enabled __unprivileged_rodata;
static uint16_t timeout __unprivileged_rodata;
static uint16_t counter __unprivileged_rodata;

static uint8_t systick_counter __privileged_data;

__privileged void watchdog_enable(bool on)
{
	disable_irq();

	systick_counter = 0;
	counter = timeout;
	enabled = on;

	enable_irq();
}

bool watchdog_is_enabled(void)
{
	return enabled;
}

__privileged void watchdog_set_timeout(uint16_t ds)
{
	timeout = ds;

	if (enabled) {
		disable_irq();

		systick_counter = 0;
		counter = timeout;

		enable_irq();
	}
}

uint16_t watchdog_get_timeleft(void)
{
	if (enabled)
		return counter;
	else
		return timeout;
}

void __privileged watchdog_handler(void)
{
	if (!enabled)
		return;

	disable_irq();

	systick_counter++;
	if (systick_counter == HZ / 10) {
		systick_counter = 0;
		counter--;
	}

	enable_irq();

	if (!counter) {
		power_control_set_startup_condition();
		power_control_disable_regulators();
		nvic_system_reset();
	}
}
