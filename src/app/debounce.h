/**
 ******************************************************************************
 * @file    debounce.h
 * @author  CZ.NIC, z.s.p.o.
 * @date    21-July-2015
 * @brief   Header file for debounce.c
 ******************************************************************************
 ******************************************************************************
 **/
#ifndef __DEBOUNCE_H
#define __DEBOUNCE_H


#define DEBOUNCE_TIMER		TIM16

/*******************************************************************************
  * @function   debounce_config
  * @brief      Debouncer configuration.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void debounce_config(void);

/*******************************************************************************
  * @function   debounce_input_timer_handler
  * @brief      Main debounce function. Called in timer interrupt handler.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void debounce_input_timer_handler(void);

/*******************************************************************************
  * @function   debounce_check_inputs
  * @brief      Check input signal.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void debounce_check_inputs(void);

#endif // __DEBOUNCE_H
