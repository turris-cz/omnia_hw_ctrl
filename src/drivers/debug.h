#ifndef DEBUG_H
#define DEBUG_H

/* ! CAUTION ! UART pins are used for CARD detection and PCI1 PLED pin ! */
#ifndef DBG_ENABLE
#error build system did not define DBG_ENABLE macro
#endif

#if DBG_ENABLE
void debug_init(void);

void debug(const char *fmt, ...);
#else
static inline void debug_init(void)
{
}

static inline void debug(const char *, ...)
{
}
#endif

#endif /* DEBUG_H */

