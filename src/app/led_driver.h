/**
  ******************************************************************************
  * @file    led_driver.h
  * @author  CZ.NIC, z.s.p.o.
  * @date    22-July-2015
  * @brief   Header file led driver.
  ******************************************************************************
  ******************************************************************************
**/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LED_DRIVER_H
#define __LED_DRIVER_H

#define LED_TIMER                 TIM17

/*******************************************************************************
  * @function   led_driver_config
  * @brief      Configure LED driver.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void led_driver_config(void);

/*******************************************************************************
  * @function   led_driver_save_colour
  * @brief      Save colour of LED specified in parameters to be displayed in next cycle.
  * @param      colour: LED colour (RGB range).
  * @param      led_index: position of LED (0..11) or index >=12 -> all LED (default)
  * @retval     None.
  *****************************************************************************/
void led_driver_save_colour(const uint32_t colour, const uint8_t led_index);

/*******************************************************************************
  * @function   led_driver_send_frame
  * @brief      Send frame to LED driver. It is called in timer interrupt.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void led_driver_send_frame(void);

/*******************************************************************************
  * @function   led_driver_pwm_set_brightness
  * @brief      Set PWM value.
  * @param      procent_val: PWM value in [%].
  * @retval     None.
  *****************************************************************************/
void led_driver_pwm_set_brightness(const uint16_t procent_val);

#endif /*__LED_DRIVER_H */
