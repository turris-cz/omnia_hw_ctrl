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

#define NANOSECONDS_TIMER                   TIM14

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

/*******************************************************************************
  * @function   delay_250ns_timeslot_config
  * @brief      Timer configuration for slot delay in nanoseconds [250 ns].
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void delay_250ns_timeslot_config(void);

/******************************************************************************
  * @function   delay_250ns_timeslot_decrement
  * @brief      Decrements the timingdelay variable.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void delay_250ns_timeslot_decrement(void);

/******************************************************************************
  * @function   power_control_nsdelay
  * @brief      Inserts a number of delay slot.
  * @param      timeslot: 1 timeslot = 250ns
  * @retval     None.
  *****************************************************************************/
void delay_250ns_timeslot(uint32_t timeslot);

#endif /* __DELAY_H */
