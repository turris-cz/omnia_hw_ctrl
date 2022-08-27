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

/*******************************************************************************
  * @function   time_config
  * @brief      Setup SysTick Timer for 1 msec interrupts.
  * @param      None
  * @retval     None
  *****************************************************************************/
void time_config(void);

/******************************************************************************
  * @function   delay
  * @brief      Inserts a delay time.
  * @param      nTime: specifies the delay time length, in miliseconds.
  * @retval     None
  *****************************************************************************/
void delay(volatile uint32_t nTime);

/******************************************************************************
  * @function   systick_irq_handler
  * @brief      Decrements the TimingDelay variable in System Timer and
  *             takes care of watchdog timeout.
  * @param      None
  * @retval     None
  *****************************************************************************/
void systick_irq_handler(void);

#endif /* __TIME_H */
