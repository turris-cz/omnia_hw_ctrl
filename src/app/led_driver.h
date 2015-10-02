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

#define LED_TIMER                 TIM3

#define LED_COUNT                 12

typedef enum led_modes {
    LED_DEFAULT_MODE   = 0,
    LED_USER_MODE      = 1,
}led_mode_t;

typedef enum led_states {
    LED_OFF         = 0,
    LED_ON          = 1,
}led_state_t;

enum led_numbers {
    LED0            = 0,
    LED1            = 1,
    LED2            = 2,
    LED3            = 3,
    LED4            = 4,
    LED5            = 5,
    LED6            = 6,
    LED7            = 7,
    LED8            = 8,
    LED9            = 9,
    LED10           = 10,
    LED11           = 11
};

enum led_names {
    POWER_LED       = LED0,
    WAN_LED         = LED1,
    LAN1_LED        = LED2,
    LAN2_LED        = LED3,
    LAN3_LED        = LED4,
    LAN4_LED        = LED5,
    LAN5_LED        = LED6,
    PCI0_LED        = LED7,
    PCI1_LED        = LED8,
    MSATA_PCI_LED   = LED9,
    USER_LED1       = LED10,
    USER_LED2       = LED11
};


struct led_rgb_default {
    uint8_t blue; //[0..255]
    uint8_t green;
    uint8_t red;
};

struct led_rgb_user {
    uint8_t blue; //[0..255]
    uint8_t green;
    uint8_t red;
};

struct led_rgb {
    struct led_rgb_default  led_rgb_data;       // colour data
    struct led_rgb_user     led_rgb_user_data;  // user data
    led_state_t             led_state;          // LED ON/OFF
    led_mode_t              led_mode;           // default / user mode
    uint16_t                brightness;
};

extern struct led_rgb leds[LED_COUNT];

/*******************************************************************************
  * @function   led_driver_config
  * @brief      Configure LED driver.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void led_driver_config(void);

/*******************************************************************************
  * @function   led_driver_set_colour
  * @brief      Save colour of LED specified in parameters to be displayed in next cycle.
  * @param      led_index: position of LED (0..11) or index >=12 -> all LEDs
  * @param      colour: LED colour (RGB range).
  * @retval     None.
  *****************************************************************************/
void led_driver_set_colour(const uint8_t led_index, const uint32_t colour);

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

/*******************************************************************************
  * @function   led_driver_step_brightness
  * @brief      Decrease LED brightness by 10% each step (each function call).
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void led_driver_step_brightness(void);

/*******************************************************************************
  * @function   led_driver_set_led_mode
  * @brief      Set mode to LED(s) - default or user mode
  * @param      led_index: position of LED (0..11) or led_index >=12 -> all LED.
  * @parame     led_mode: LED_DEFAULT_MODE / LED_USER_MODE
  * @retval     None.
  *****************************************************************************/
void led_driver_set_led_mode(const uint8_t led_index, const led_mode_t led_mode);

/*******************************************************************************
  * @function   led_driver_set_led_state
  * @brief      Set state of the LED(s) - LED_ON / LED_OFF
  * @param      led_index: position of LED (0..11) or led_index >=12 -> all LED.
  * @parame     led_state: LED_OFF / LED_ON
  * @retval     None.
  *****************************************************************************/
void led_driver_set_led_state(const uint8_t led_index, const led_state_t led_state);

#endif /*__LED_DRIVER_H */
