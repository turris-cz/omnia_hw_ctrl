#ifndef IRQ_H
#define IRQ_H

#include "cpu.h"
#include "svc.h"
#include "debug.h"

static __force_inline void enable_irq_with_prio(IRQn_Type irq, uint8_t prio)
{
	switch (irq) {
	case TPM0_IRQn:
	case TPM1_IRQn:
	case TPM2_IRQn:
	case I2C0_IRQn:
	case PortA_IRQn:
	case PortB_IRQn:
	case PortC_IRQn:
	case PortD_IRQn:
	case PortE_IRQn:
		nvic_enable_irq_with_prio(irq, prio);
		break;
	default:
		debug("Invalid IRQn %u!\n", irq);
	}
}
SYSCALL(enable_irq_with_prio, IRQn_Type, uint8_t)

#endif /* IRQ_H */
