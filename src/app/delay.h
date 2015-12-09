/**
  ******************************************************************************
  * @file    delay.h
  * @author  CZ.NIC, z.s.p.o.
  * @date    22-July-2015
  * @brief   Header file delay file
  ******************************************************************************
  ******************************************************************************
**/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DELAY_H
#define __DELAY_H

#define DELAY_COUNTER_TRESHOLD        2000 /* 2 sec */

extern volatile uint32_t delay_counter_ms;
extern volatile uint16_t delay_counter_stop;

/*******************************************************************************
  * @function   delay_systimer_config
  * @brief      Setup SysTick Timer for 1 msec interrupts.
  * @param      None
  * @retval     None
  *****************************************************************************/
void delay_systimer_config(void);

/******************************************************************************
  * @function   delay
  * @brief      Inserts a delay time.
  * @param      nTime: specifies the delay time length, in miliseconds.
  * @retval     None
  *****************************************************************************/
void delay(volatile uint32_t nTime);

/******************************************************************************
  * @function   delay_timing_decrement
  * @brief      Decrements the TimingDelay variable.
  * @param      None
  * @retval     None
  *****************************************************************************/
void delay_timing_decrement(void);

#endif /* __DELAY_H */
