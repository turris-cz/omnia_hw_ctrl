#ifndef POWEROFF_H
#define POWEROFF_H

#include "pin_defs.h"
#include "led_driver.h"
#include "power_control.h"
#include "time.h"
#include "svc.h"

void platform_poweroff(bool enable_button, uint32_t wakeup_timeout);

static inline void poweroff(bool enable_button, uint32_t wakeup)
{
	uint32_t wakeup_timeout;

	/* turn off leds */
	led_driver_set_brightness(0);

	/* disable interrupts, clear pending */
	disable_irq();
	nvic_disable_all_and_clear_pending();

	/* disable SysTick, clear pending */
	systick_disable_clear();

	/*
	 * Disable regulators and stop driving interrupt pin to SOC.
	 * (With SOC voltage disabled, driving this pin eats around 0.2 W.)
	 */
	power_control_disable_regulators();
	gpio_init_inputs(pin_nopull, INT_MCU_PIN);

	if (wakeup > uptime)
		wakeup_timeout = wakeup - uptime;
	else
		wakeup_timeout = 0;

	platform_poweroff(enable_button, wakeup_timeout);
}
SYSCALL(poweroff, bool, uint32_t)

#endif /* POWEROFF_H */
