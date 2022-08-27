#include "watchdog.h"
#include "power_control.h"

#define WATCHDOG_TIMEOUT	120000 /* ms */

struct st_watchdog watchdog;

void watchdog_handler(void)
{
	static uint32_t wdg_cnt;

	if (watchdog.watchdog_state == RUN) {
		wdg_cnt++;

		if (wdg_cnt >= WATCHDOG_TIMEOUT) {
			power_control_set_startup_condition();
			power_control_disable_regulators();
			NVIC_SystemReset(); /* SW reset */
		}
	} else {
		wdg_cnt = 0;
	}
}
