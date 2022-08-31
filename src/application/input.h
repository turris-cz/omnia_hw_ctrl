#ifndef __INPUT_H
#define __INPUT_H

#include "compiler.h"

#define MAX_BUTTON_PRESSED_COUNTER	7
#define MAX_BUTTON_DEBOUNCE_STATE	3

typedef enum button_modes {
	BUTTON_DEFAULT,
	BUTTON_USER,
} button_mode_t;

typedef enum button_states {
	BUTTON_PRESSED,
	BUTTON_RELEASED,
} button_state_t;

/* flags of input signals */
typedef struct {
	unsigned man_res	: 1;
	unsigned sysres_out	: 1;
	unsigned dbg_res	: 1;
	unsigned m_res		: 1;
	unsigned pg		: 1;
	unsigned pg_4v5		: 1;
	unsigned usb30_ovc	: 1;
	unsigned usb31_ovc	: 1;
	unsigned rtc_alarm	: 1;
	unsigned button_sts	: 1;
	unsigned card_det	: 1;
	unsigned msata_ind	: 1;
} input_state_t;

struct button_def {
	button_mode_t button_mode;
	button_state_t button_state;
	int8_t button_pressed_counter;
	uint16_t button_pin_state[MAX_BUTTON_DEBOUNCE_STATE];
	uint16_t button_debounce_state;
};

extern input_state_t input_state;
extern struct button_def button_front;

/*******************************************************************************
  * @function   input_config
  * @brief      Input configuration.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void input_config(void);

/*******************************************************************************
  * @function   button_debounce_handler
  * @brief      Button debounce function. Called from SysTick handler every 5 ms.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void button_debounce_handler(void);

/*******************************************************************************
  * @function   input_signals_handler
  * @brief      Check input signal.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void input_signals_handler(void);

/*******************************************************************************
  * @function   button_counter_increase
  * @brief      Increase button counter.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void button_counter_increase(void);

/*******************************************************************************
  * @function   button_counter_decrease
  * @brief      Decrease button counter by the current value in i2c status structure.
  * @param      value: decrease the button counter by this parameter
  * @retval     None.
  *****************************************************************************/
void button_counter_decrease(uint8_t value);

#endif /* __INPUT_H */
