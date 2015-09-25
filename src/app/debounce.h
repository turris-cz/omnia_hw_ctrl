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

struct input_sig {
    uint8_t man_res     :1;
    uint8_t sysres_out  :1;
    uint8_t dbg_res     :1;
    uint8_t m_res       :1;
    uint8_t pg          :1;
    uint8_t usb30_ovc   :1;
    uint8_t usb31_ovc   :1;
    uint8_t rtc_alarm   :1;
    uint8_t led_brt     :1;
};

extern struct input_sig debounce_input_signal;

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
