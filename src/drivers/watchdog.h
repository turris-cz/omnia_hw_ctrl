#ifndef __WATCHDOG_H
#define __WATCHDOG_H

#include "cpu.h"
#include "svc.h"

/* default timeout in deciseconds, 2 minutes */
#define WATCHDOG_DEFAULT_TIMEOUT	1200

void watchdog_enable(bool on);
SYSCALL(watchdog_enable, bool)

bool watchdog_is_enabled(void);

/* Sets watchdog timeout in deciseconds / pings watchdog */
void watchdog_set_timeout(uint16_t ds);
SYSCALL(watchdog_set_timeout, uint16_t)

uint16_t watchdog_get_timeleft(void);

void watchdog_handler(void);

#endif /* __WATCHDOG_H */
