#ifndef RESET_COMMON_H
#define RESET_COMMON_H

#include "cpu.h"
#include "svc.h"

#define RESET_REASON_MSG_MAGIC	0xdeadbeef

typedef enum {
	NORMAL_BOOT		= 0x00,
	STAY_IN_BOOTLOADER_REQ	= 0xaa,
	APPLICATION_FAULT	= 0xee,
} reset_reason_t;

typedef struct {
	uint32_t fault;
} reset_reason_info_t;

static inline void hard_reset(void)
{
	nvic_system_reset();
}
SYSCALL(hard_reset)

#endif /* RESET_COMMON_H */
