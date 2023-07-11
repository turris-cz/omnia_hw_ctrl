#include "cpu.h"
#include "reset_reason.h"
#include "memory_layout.h"

extern uint32_t _stack_top;

static void __naked __privileged do_reset_to_other_program(void)
{
	set_control(0);
	set_msp((uint32_t)&_stack_top);

	MPU_RGDAACn(0) = MPU_RGDn_WORD2_MnSM_AS_UM(0) |
			 MPU_RGDn_WORD2_MnUM_RWX(0) |
			 MPU_RGDn_WORD2_MnSM_AS_UM(2) |
			 MPU_RGDn_WORD2_MnUM_RWX(2);

	for (unsigned reg = 1; reg < 8; ++reg)
		MPU_RGDn_WORD3(reg) = 0;

	if (!BOOTLOADER_BUILD)
		set_reset_reason(STAY_IN_BOOTLOADER_REQ, 0);

	reset_to_address(BOOTLOADER_BUILD ? APPLICATION_BEGIN
					  : BOOTLOADER_BEGIN);
}

void SYSCALL(soft_reset_to_other_program)(void)
{
	exception_frame_t *frame;

	disable_irq();

	frame = (exception_frame_t *)get_psp() - 1;
	frame->psr = 0x1000000;
	frame->pc = (uint32_t)do_reset_to_other_program | 0x1;
	set_psp((uint32_t)frame);

	set_control(CONTROL_SPSEL);
}
