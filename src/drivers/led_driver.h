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

#include "compiler.h"

#define LED_COUNT		12

enum colors {
    WHITE_COLOR        = 0xFFFFFF,
    RED_COLOR          = 0xFF0000,
    GREEN_COLOR        = 0x00FF00,
    BLUE_COLOR         = 0x0000FF,
    BLACK_COLOR        = 0x000000,
    YELLOW_COLOR       = 0xFFFF00,
};

enum led_names {
    POWER_LED       = 11,
    LAN0_LED        = 10,
    LAN1_LED        = 9,
    LAN2_LED        = 8,
    LAN3_LED        = 7,
    LAN4_LED        = 6,
    WAN_LED         = 5,
    PCI1_LED        = 3,
    PCI2_LED        = 4,
    MSATA_PCI_LED   = 2,
    USER_LED1       = 1,
    USER_LED2       = 0
};
/* PCI1 and PCI2 leds are reversed, there is a difference between numbering in schematic
editor and numbering on the case for the router */

extern uint8_t effect_reset_finished;

/*******************************************************************************
  * @function   led_driver_config
  * @brief      Configure LED driver.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void led_driver_config(void);

/*******************************************************************************
  * @function   led_set_color
  * @brief      Save color of LED specified in parameters to be displayed in next cycle.
  * @param      led_index: position of LED (0..11) or index >=12 -> all LEDs
  * @param      color: LED color (RGB range).
  * @retval     None.
  *****************************************************************************/
void led_set_color(const uint8_t led_index, const uint32_t color);

/*******************************************************************************
  * @function   led_driver_irq_handler
  * @brief      Send RGB LEDs frame. Called as LED driver timer interrupt.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void led_driver_irq_handler(void);

/*******************************************************************************
  * @function   led_driver_set_brightness
  * @brief      Set PWM value.
  * @param      procent_val: PWM value in [%].
  * @retval     None.
  *****************************************************************************/
void led_driver_set_brightness(uint8_t procent_val);

/*******************************************************************************
  * @function   led_driver_get_brightness
  * @brief      Set PWM value.
  * @param      None.
  * @retval     procent_val: PWM value in [%].
  *****************************************************************************/
uint8_t led_driver_get_brightness(void);

/*******************************************************************************
  * @function   led_driver_step_brightness
  * @brief      Decrease LED brightness by 10% each step (each function call).
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void led_driver_step_brightness(void);

/*******************************************************************************
  * @function   led_set_user_mode
  * @brief      Set mode to LED(s) - default or user mode
  * @param      led_index: position of LED (0..11) or led_index >=12 -> all LED.
  * @parame     set: true to set user mode, false to unset
  * @retval     None.
  *****************************************************************************/
void led_set_user_mode(const uint8_t led_index, const bool set);

/*******************************************************************************
  * @function   led_set_state
  * @brief      Set state of the LED(s)
  * @param      led_index: position of LED (0..11) or led_index >=12 -> all LED.
  * @parame     state: false / true
  * @retval     None.
  *****************************************************************************/
void led_set_state(const uint8_t led_index, const bool state);

/*******************************************************************************
  * @function   led_set_state_user
  * @brief      Set state of the LED(s) from user/I2C
  * @param      led_index: position of LED (0..11) or led_index >=12 -> all LED.
  * @parame     state: false / true
  * @retval     None.
  *****************************************************************************/
void led_set_state_user(const uint8_t led_index, const bool state);

/*******************************************************************************
  * @function   led_driver_knight_rider_effect
  * @brief      Display knight rider effect on LEDs.
  * @param      color: color in RGB range.
  * @retval     None.
  *****************************************************************************/
void led_driver_knight_rider_effect(uint32_t color);

/*******************************************************************************
  * @function   led_driver_knight_rider_color_effect
  * @brief      Display knight rider effect on LEDs.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void led_driver_knight_rider_color_effect(void);

/*******************************************************************************
  * @function   led_driver_double_knight_rider_effect
  * @brief      Display double knight rider effect on LEDs.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void led_driver_double_knight_rider_effect(void);

/*******************************************************************************
  * @function   led_driver_effect_irq_handler
  * @brief      Display LED effect.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void led_driver_effect_irq_handler(void);

/*******************************************************************************
  * @function   led_driver_reset_effect
  * @brief      Enable/Disable knight rider effect after reset.
  * @param      color: color in RGB range.
  * @retval     None.
  *****************************************************************************/
void led_driver_reset_effect(bool state);

#endif /*__LED_DRIVER_H */
