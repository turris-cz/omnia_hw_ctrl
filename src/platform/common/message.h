#ifndef MESSAGE_H
#define MESSAGE_H

#include "memory_layout.h"
#include "crc32_plat.h"
#include "cpu.h"

typedef enum {
	EMPTY_MESSAGE		= 0x00,
	STAY_IN_BOOTLOADER	= 0xAA,
} message_t;

#include "message_plat.h"

#define SYS_RESET_MSG_MAGIC	0xdeadbeef

#if BOOTLOADER_BUILD
static inline bool get_sys_reset_message(uint32_t *msg)
{
	uint32_t *p = (uint32_t *)(RAM_END - SYS_RESET_MSG_LENGTH);

	if (p[0] != SYS_RESET_MSG_MAGIC)
		return false;

	if (p[3] != crc32_plat(0xffffffff, p, 12))
		return false;

	*msg = p[1];

	/* invalidate */
	p[0] = 0;

	return true;
}
#else /* !BOOTLOADER_BUILD */
static inline void sys_reset_with_message(uint32_t msg)
{
	uint32_t *p = (uint32_t *)(RAM_END - SYS_RESET_MSG_LENGTH);

	crc32_enable();

	p[0] = SYS_RESET_MSG_MAGIC;
	p[1] = msg;
	p[2] = crc32_plat(0xffffffff, (void *)RAM_BEGIN, RAM_LENGTH - 0x200);
	dsb();
	p[3] = crc32_plat(0xffffffff, p, 12);

	nvic_system_reset();
}
#endif /* !BOOTLOADER_BUILD */

#endif /* MESSAGE_H */
