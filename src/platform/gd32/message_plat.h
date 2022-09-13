#ifndef MESSAGE_PLAT_H
#define MESSAGE_PLAT_H

#include "gd32f1x0_crc.h"
#include "compiler.h"

/* Before using these, crc32_enable() has to be called from crc32_plat.h */

static inline void set_message_before_switch(message_t msg)
{
	CRC_FDATA = msg;
}

static inline message_t get_message_after_switch(void)
{
	message_t msg = CRC_FDATA;

	CRC_FDATA = EMPTY_MESSAGE;

	return msg;
}

#endif /* MESSAGE_PLAT_H */
