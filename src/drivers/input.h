#ifndef __INPUT_H
#define __INPUT_H

#include "bits.h"
#include "svc.h"

typedef enum {
	INPUT_REQ_NONE,
	INPUT_REQ_LIGHT_RESET,
	INPUT_REQ_HARD_RESET,
} input_req_t;

/*
 * This is for led_pins_read() from platform's pin_defs.
 * The order of the bits must be the same as in EXT_STS_LED_STATES_MASK
 * and INT_LED_STATES_MASK.
 */
enum led_pin_states_e {
	WLAN0_MSATA_LED_BIT	= BIT(0),
	WLAN1_LED_BIT		= BIT(1),
	WLAN2_LED_BIT		= BIT(2),
	WPAN0_LED_BIT		= BIT(3),
	WPAN1_LED_BIT		= BIT(4),
	WPAN2_LED_BIT		= BIT(5),
	WAN_LED0_BIT		= BIT(6),
	WAN_LED1_BIT		= BIT(7),
	LAN_LEDS_BIT_MASK	= GENMASK(19, 8),
	LAN0_LED0_BIT		= BIT(8),
	LAN0_LED1_BIT		= BIT(9),
	LAN1_LED0_BIT		= BIT(10),
	LAN1_LED1_BIT		= BIT(11),
	LAN2_LED0_BIT		= BIT(12),
	LAN2_LED1_BIT		= BIT(13),
	LAN3_LED0_BIT		= BIT(14),
	LAN3_LED1_BIT		= BIT(15),
	LAN4_LED0_BIT		= BIT(16),
	LAN4_LED1_BIT		= BIT(17),
	LAN5_LED0_BIT		= BIT(18),
	LAN5_LED1_BIT		= BIT(19),
};

typedef struct {
	bool user_mode;
	uint8_t pressed_counter;
	bool state;
} button_t;

extern button_t button;
extern volatile uint32_t input_led_pins;

/*******************************************************************************
  * @function   button_debounce_handler
  * @brief      Button debounce function. Called from SysTick handler every 5 ms.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void button_debounce_handler(void);

/*******************************************************************************
  * @function   button_counter_decrease
  * @brief      Decrease button counter by the current value in i2c status structure.
  * @param      value: decrease the button counter by this parameter
  * @retval     None.
  *****************************************************************************/
void button_counter_decrease(uint8_t value);
SYSCALL(button_counter_decrease, uint8_t)

void button_set_user_mode(bool on);
SYSCALL(button_set_user_mode, bool)

void input_signals_config(void);

void input_signals_init(void);
SYSCALL(input_signals_init)

/*******************************************************************************
  * @function   input_signals_poll
  * @brief      Poll the input signals.
  * @retval     Next state.
  *****************************************************************************/
input_req_t input_signals_poll(void);
SYSCALL(input_signals_poll)

#endif /* __INPUT_H */
