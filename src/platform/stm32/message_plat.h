#ifndef MESSAGE_PLAT_H
#define MESSAGE_PLAT_H

#include "stm32f0xx.h"
#include "compiler.h"

/* Before using these, crc32_enable() has to be called from crc32_plat.h */

static inline void set_message_before_switch(message_t msg)
{
	CRC->IDR = msg;
}

static inline message_t get_message_after_switch(void)
{
	message_t msg = CRC->IDR;

	CRC->IDR = EMPTY_MESSAGE;

	return msg;
}

#endif /* MESSAGE_PLAT_H */
