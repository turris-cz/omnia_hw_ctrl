/**
  ******************************************************************************
  * @file    time.h
  * @author  CZ.NIC, z.s.p.o.
  * @date    22-July-2015
  * @brief   Header file delay file
  ******************************************************************************
  ******************************************************************************
**/
#ifndef __TIME_H
#define __TIME_H

#include "compiler.h"

#define HZ		200
#define JIFFY_TO_MSECS	(1000 / HZ)

extern volatile uint32_t jiffies;

static inline uint32_t jiffies_to_msecs(uint32_t x)
{
	/* we want this to be precise */
	_Static_assert(1000 % HZ == 0, "1000 must be divisible by HZ");

	return x * JIFFY_TO_MSECS;
}

/*******************************************************************************
  * @function   time_config
  * @brief      Setup SysTick Timer interrupt to fire at 200 Hz frequency.
  * @param      None
  * @retval     None
  *****************************************************************************/
void time_config(void);

/******************************************************************************
  * @function   msleep
  * @brief      Sleeps / delays for a given time.
  * @param      ms: specifies the delay time length, in miliseconds.
  * @retval     None
  *****************************************************************************/
void msleep(uint32_t ms);

/******************************************************************************
  * @function   systick_irq_handler
  * @brief      Decrements the TimingDelay variable in System Timer and
  *             takes care of watchdog timeout.
  * @param      None
  * @retval     None
  *****************************************************************************/
void systick_irq_handler(void);

#endif /* __TIME_H */
