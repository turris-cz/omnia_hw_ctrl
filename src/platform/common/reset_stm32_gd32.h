#ifndef RESET_STM32_GD32_H
#define RESET_STM32_GD32_H

#include "reset_common.h"
#include "memory_layout.h"
#include "crc32_plat.h"

static inline bool get_fault_info(uint32_t *fault)
{
	uint32_t *p = (uint32_t *)(RAM_END - RESET_REASON_MSG_LENGTH);

	if (p[0] != RESET_REASON_MSG_MAGIC)
		return false;

	if (p[3] != crc32_plat(0xffffffff, p, 12))
		return false;

	*fault = p[1];

	/* invalidate */
	p[0] = 0;

	return true;
}

static inline void set_fault_info(uint32_t fault)
{
	uint32_t *p = (uint32_t *)(RAM_END - RESET_REASON_MSG_LENGTH);

	crc32_enable();

	p[0] = RESET_REASON_MSG_MAGIC;
	p[1] = fault;
	p[2] = crc32_plat(0xffffffff, (void *)RAM_BEGIN, RAM_LENGTH - 0x200);
	dsb();
	p[3] = crc32_plat(0xffffffff, p, 12);

	nvic_system_reset();
}

static inline reset_reason_t get_reset_reason(reset_reason_info_t *info)
{
	if (get_fault_info(&info->fault)) {
		return APPLICATION_FAULT;
	} else if (CRC_FREE_DATA_REG == STAY_IN_BOOTLOADER_REQ) {
		CRC_FREE_DATA_REG = 0;
		return STAY_IN_BOOTLOADER_REQ;
	} else {
		return NORMAL_BOOT;
	}
}

static inline void set_reset_reason(reset_reason_t reason, uint32_t fault)
{
	compiletime_assert(reason != NORMAL_BOOT, "Invalid reset reason");

	switch (reason) {
	case APPLICATION_FAULT:
		return set_fault_info(fault);

	case STAY_IN_BOOTLOADER_REQ:
		CRC_FREE_DATA_REG = STAY_IN_BOOTLOADER_REQ;
		return;

	default:
		unreachable();
	}
}

#endif /* RESET_STM32_GD32_H */
