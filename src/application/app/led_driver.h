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

#define BIT(b)                    (1 << (b))

#define LED_TIMER                 TIM3
#define LED_EFFECT_TIMER          TIM6

#define LED_COUNT                 12

enum colours {
    WHITE_COLOUR        = 0xFFFFFF,
    RED_COLOUR          = 0xFF0000,
    GREEN_COLOUR        = 0x00FF00,
    BLUE_COLOUR         = 0x0000FF,
    BLACK_COLOUR        = 0x000000,
    YELLOW_COLOUR       = 0xFFFF00,
};

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
    POWER_LED       = LED11,
    LAN0_LED        = LED10,
    LAN1_LED        = LED9,
    LAN2_LED        = LED8,
    LAN3_LED        = LED7,
    LAN4_LED        = LED6,
    WAN_LED         = LED5,
    PCI1_LED        = LED3,
    PCI2_LED        = LED4,
    MSATA_PCI_LED   = LED2,
    USER_LED1       = LED1,
    USER_LED2       = LED0
};
/* PCI1 and PCI2 leds are reversed, there is a difference between numbering in schematic
editor and numbering on the case for the router */

extern uint16_t leds_user_mode, leds_state, leds_state_user, leds_color_correction;

extern uint8_t effect_reset_finished;

void led_config(void);

void led_send_frame(void);

void led_pwm_set_brightness(uint16_t procent_val);
uint16_t led_pwm_get_brightness(void);
void led_step_brightness(void);

void led_set_colour(int led, uint32_t colour);
void led_set_colour_all(uint32_t colour);
void led_compute_levels(int led, int color_correction);
void led_compute_levels_all(int color_correction);

static inline void led_set_color_correction(int led, int enable)
{
	if (enable)
		leds_color_correction |= BIT(led);
	else
		leds_color_correction &= ~BIT(led);

	led_compute_levels(led, enable);
}

static inline void led_set_color_correction_all(int enable)
{
	if (enable)
		leds_color_correction = 0xfff;
	else
		leds_color_correction = 0;

	led_compute_levels_all(enable);
}

static inline int led_is_user_mode(int led)
{
	return !!(leds_user_mode & BIT(led));
}

static inline void led_set_user_mode(int led, int enable)
{
	if (enable) {
		leds_user_mode |= BIT(led);
		leds_state |= leds_state_user & BIT(led);
	} else {
		leds_user_mode &= ~BIT(led);
	}
}

static inline void led_set_user_mode_all(int enable)
{
	if (enable) {
		leds_user_mode = 0xfff;
		leds_state = leds_state_user;
	} else {
		leds_user_mode = 0;
		leds_state = 0;
	}
}

static inline int led_state(int led)
{
	return !!(leds_state & BIT(led));
}

static inline void led_set_state(int led, int enable)
{
	if (enable)
		leds_state |= BIT(led);
	else
		leds_state &= ~BIT(led);
}

static inline void led_set_state_all(int enable)
{
	if (enable)
		leds_state = 0xfff;
	else
		leds_state = 0;
}

static inline void led_set_state_user(int led, int enable)
{
	if (enable)
		leds_state_user |= BIT(led);
	else
		leds_state_user &= ~BIT(led);
	if (!led_is_user_mode(led))
		return;
	led_set_state(led, enable);
}

static inline void led_set_state_user_all(int enable)
{
	leds_state_user = 0xfff;
	if (enable)
		leds_state |= leds_user_mode;
	else
		leds_state &= ~leds_user_mode;
}

void led_knight_rider_effect(uint32_t colour);
void led_knight_rider_colour_effect(void);
void led_double_knight_rider_effect(void);
void led_knight_rider_effect_handler(void);
void led_reset_effect(FunctionalState state);

#endif /*__LED_DRIVER_H */
