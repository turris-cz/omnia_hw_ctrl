#ifndef __WATCHDOG_H
#define __WATCHDOG_H

#include "compiler.h"

typedef enum watchdog_state {
	STOP	= 0,
	RUN	= 1,
	INIT	= 2,
} watchdog_state_t;

enum watchdog_status {
	WDG_DISABLE	= 0,
	WDG_ENABLE	= 1
};

struct st_watchdog {
	watchdog_state_t watchdog_state;
	uint16_t watchdog_sts;
};

extern struct st_watchdog watchdog;

void watchdog_handler(void);

#endif /* __WATCHDOG_H */
