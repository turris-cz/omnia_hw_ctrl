#ifndef POWEROFF_H
#define POWEROFF_H

#include "compiler.h"

void platform_poweroff(bool enable_button, uint32_t wakeup_timeout);

#endif /* POWEROFF_H */
