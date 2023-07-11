#include "cpu.h"
#include "reset.h"
#include "memory_layout.h"

extern uint32_t _stack_top;

static void __naked __privileged do_reset_to_other_program(void)
{
	/* Use main stack pointer */
	set_control(0);
	set_msp((uint32_t)&_stack_top);

	/* Set MPU region 0 access to default values */
	MPU_RGDAACn(0) = MPU_RGDn_WORD2_MnSM_AS_UM(0) |
			 MPU_RGDn_WORD2_MnUM_RWX(0) |
			 MPU_RGDn_WORD2_MnSM_AS_UM(2) |
			 MPU_RGDn_WORD2_MnUM_RWX(2);

	/* Disable other MPU regions */
	for (unsigned reg = 1; reg < 8; ++reg)
		MPU_RGDn_WORD3(reg) = 0;

	if (!BOOTLOADER_BUILD)
		set_reset_reason(STAY_IN_BOOTLOADER_REQ, 0);

	reset_to_address(BOOTLOADER_BUILD ? APPLICATION_BEGIN
					  : BOOTLOADER_BEGIN);
}

__privileged void plat_soft_reset_to_other_program(void)
{
	exception_frame_t *frame;

	disable_irq();

	/*
	 * Update exception frame so that it returns to the
	 * do_reset_to_other_program function.
	 */
	frame = (exception_frame_t *)get_psp();
	frame->psr = 0x1000000;
	frame->pc = (uint32_t)do_reset_to_other_program | 0x1;

	/* Set thread context to privileged mode */
	set_control(CONTROL_SPSEL);
}
