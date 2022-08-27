/**
 ******************************************************************************
 * @file    time.c
 * @author  CZ.NIC, z.s.p.o.
 * @date    22-July-2015
 * @brief   This file ensures time driver using of System Timer
 ******************************************************************************
 ******************************************************************************
 **/
#include "debug.h"
#include "cpu.h"
#include "time.h"
#include "power_control.h"
#include "watchdog.h"

static volatile uint32_t timingdelay;

/*******************************************************************************
  * @function   time_config
  * @brief      Setup SysTick Timer for 1 msec interrupts.
  * @param      None
  * @retval     None
  *****************************************************************************/
void time_config(void)
{
	if (SysTick_Config(SYS_CORE_FREQ / 1000u)) {
		debug("Failed configuring SysTick\n");
		/* Capture error */
		while (1);
	}

	NVIC_SetPriority(SysTick_IRQn, 2);
}

/******************************************************************************
  * @function   mdelay
  * @brief      Inserts a delay time.
  * @param      ms: specifies the delay time length, in miliseconds.
  * @retval     None
  *****************************************************************************/
void mdelay(uint32_t ms)
{
	timingdelay = ms;

	while (timingdelay)
		nop();
}

/******************************************************************************
  * @function   systick_irq_handler
  * @brief      Decrements the TimingDelay variable in System Timer and
  *             takes care of watchdog timeout.
  * @param      None
  * @retval     None
  *****************************************************************************/
void __irq systick_irq_handler(void)
{
	if (timingdelay)
		timingdelay--;

	if (!BOOTLOADER_BUILD)
		watchdog_handler();
}
