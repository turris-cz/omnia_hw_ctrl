#include "cpu.h"

static __naked void signal_caller(void *, void *, void *,
				  void (*)(void *, void *, void *))
{
	asm volatile(
		/* call handler(arg1, arg2, arg3) */
		"blx r3\n\t"
		/* restore state from original exception frame */
		"push {r4}\n\t"
		"add r4, sp, #20\n\t"
		"ldm r4!, {r0, r1, r2, r3}\n\t"
		"mov r12, r0\n\t"
		"mov lr, r1\n\t"
		"adds r2, #1\n\t"
		"lsls r4, r3, #22\n\t"
		"bmi.n signal_caller_unaligned\n\t"
		"msr apsr, r3\n\t"
		"add r4, sp, #32\n\t"
		"str r2, [r4]\n\t"
		"add r4, sp, #4\n\t"
		"ldm r4!, {r0, r1, r2, r3}\n\t"
		"add r4, sp, #16\n\t"
		"stm r4!, {r0, r1, r2, r3}\n\t"
		"pop {r4}\n\t"
		"add sp, #12\n\t"
		"pop {r0, r1, r2, r3, pc}\n\t"
		"signal_caller_unaligned:\n\t"
		"msr apsr, r3\n\t"
		"add r4, sp, #36\n\t"
		"str r2, [r4]\n\t"
		"add r4, sp, #4\n\t"
		"ldm r4!, {r0, r1, r2, r3}\n\t"
		"add r4, sp, #20\n\t"
		"stm r4!, {r0, r1, r2, r3}\n\t"
		"pop {r4}\n\t"
		"add sp, #16\n\t"
		"pop {r0, r1, r2, r3, pc}\n\t"
	);
}

__privileged void _push_signal(const void *handler, void *arg1, void *arg2,
			       void *arg3)
{
	exception_frame_t *frame;

	disable_irq();

	/*
	 * Push new frame onto PSP so that the exception handler returns
	 * to signal_caller with arguments arg1, arg2, arg3, handler.
	 * Signal caller then calls handler(arg1, arg2, arg3) and afterwards
	 * restores the previous state from the original frame.
	 */

	frame = (exception_frame_t *)get_psp() - 1;

	frame->r0 = (uint32_t)arg1;
	frame->r1 = (uint32_t)arg2;
	frame->r2 = (uint32_t)arg3;
	frame->r3 = (uint32_t)handler;
	frame->pc = (uint32_t)signal_caller & ~0x1U;
	frame->psr = 0x1000000;

	set_psp((uint32_t)frame);

	enable_irq();
}
