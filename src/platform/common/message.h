#ifndef MESSAGE_H
#define MESSAGE_H

typedef enum {
	EMPTY_MESSAGE		= 0x00,
	STAY_IN_BOOTLOADER	= 0xAA,
} message_t;

#include "message_plat.h"

#endif /* MESSAGE_H */
