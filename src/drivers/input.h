#ifndef __INPUT_H
#define __INPUT_H

#include "compiler.h"

typedef enum {
	INPUT_REQ_NONE,
	INPUT_REQ_LIGHT_RESET,
	INPUT_REQ_HARD_RESET,
} input_req_t;

typedef struct {
	bool user_mode;
	uint8_t pressed_counter;
	bool state;
} button_t;

extern button_t button;

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

void input_signals_config(void);

void input_signals_init(void);

/*******************************************************************************
  * @function   input_signals_handler
  * @brief      Check input signal.
  * @retval     Next state.
  *****************************************************************************/
input_req_t input_signals_handler(void);

#endif /* __INPUT_H */
