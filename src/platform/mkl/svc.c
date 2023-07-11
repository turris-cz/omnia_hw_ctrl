#if SYSCALLS

#include "cpu.h"
#include "svc.h"
#include "clock.h"
#include "i2c_slave.h"
#include "irq.h"
#include "time.h"
#include "flash.h"
#include "led_driver.h"
#include "reset_reason.h"
#include "timer.h"
#include "watchdog.h"
#include "debug.h"

void __irq svc_handler(void)
{
	exception_frame_t *frame = (void *)get_psp();
	svc_t svc = *(uint8_t *)(frame->pc - 2);
	uint32_t arg1 = frame->r0;
	uint32_t arg2 = frame->r1;

	switch (svc) {
	case SYS_clk_config:
		clk_config_supervisor(arg1, arg2);
		break;
	case SYS_enable_irq_with_prio:
		enable_irq_with_prio_supervisor(arg1, arg2);
		break;
	case SYS_time_config:
		time_config_supervisor();
		break;
	case SYS_msleep:
		msleep_supervisor(arg1);
		break;
	case SYS_flash_init:
		flash_init_supervisor();
		break;
	case SYS_flash_async_op:
		flash_async_op_supervisor((const void *)arg1);
		break;
	case SYS_watchdog_enable:
		watchdog_enable_supervisor(arg1);
		break;
	case SYS_watchdog_set_timeout:
		watchdog_set_timeout_supervisor(arg1);
		break;
	case SYS_soft_reset_to_other_program:
		soft_reset_to_other_program_supervisor();
		break;
	case SYS_hard_reset:
		hard_reset_supervisor();
		break;
	default:
		debug("unhandled svc(%u, %#10x, %#10x)\n", svc, arg1, arg2);
		break;
	}
}

#endif /* SYSCALLS */
