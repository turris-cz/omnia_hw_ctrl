/**
 ******************************************************************************
 * @file    delay.c
 * @author  CZ.NIC, z.s.p.o.
 * @date    22-July-2015
 * @brief   This file ensures delay driver using of System Timer
 ******************************************************************************
 ******************************************************************************
 **/
#include "delay.h"
#include "power_control.h"
#include "watchdog.h"

static volatile uint32_t timingdelay;

/*******************************************************************************
  * @function   delay_systimer_config
  * @brief      Setup SysTick Timer for 1 msec interrupts.
  * @param      None
  * @retval     None
  *****************************************************************************/
void delay_systimer_config(void)
{
	if (SysTick_Config(SYS_CORE_FREQ / 1000u))
	{
		/* Capture error */
		while (1);
	}
	NVIC_SetPriority(SysTick_IRQn, 2);
}

/******************************************************************************
  * @function   delay
  * @brief      Inserts a delay time.
  * @param      nTime: specifies the delay time length, in miliseconds.
  * @retval     None
  *****************************************************************************/
void delay(volatile uint32_t nTime)
{
	timingdelay = nTime;

	while(timingdelay != 0u);
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
	if (timingdelay != 0x00)
		timingdelay--;

	if (!BOOTLOADER_BUILD)
		watchdog_handler();
}
