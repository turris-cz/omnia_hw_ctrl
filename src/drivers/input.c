#include "input.h"
#include "power_control.h"
#include "time.h"
#include "led_driver.h"
#include "debug.h"
#include "i2c_iface.h"
#include "timer.h"

#define MAX_BUTTON_PRESSED_COUNTER	7
#define MAX_BUTTON_DEBOUNCE_STATE	3

button_t button;

static void button_pressed(void)
{
	if (button.user_mode) {
		disable_irq();
		if (button.pressed_counter < MAX_BUTTON_PRESSED_COUNTER)
			button.pressed_counter++;
		enable_irq();
	} else {
		led_driver_step_brightness();
	}
}

/*******************************************************************************
  * @function   button_debounce_handler
  * @brief      Button debounce function. Called from SysTick handler every 5 ms.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void button_debounce_handler(void)
{
	static uint8_t state_cnt[2];
	bool state, prev_state;

	state = !gpio_read(FRONT_BTN_PIN);

	state_cnt[!state] = 0;
	if (state_cnt[state] < MAX_BUTTON_DEBOUNCE_STATE) {
		state_cnt[state]++;
		if (state_cnt[state] == MAX_BUTTON_DEBOUNCE_STATE) {
			prev_state = button.state;
			button.state = state;
			if (!prev_state && state)
				button_pressed();
		}
	}
}

/*******************************************************************************
  * @function   button_counter_decrease
  * @brief      Decrease button counter by the current value in i2c status structure.
  * @param      value: decrease the button counter by this parameter
  * @retval     None.
  *****************************************************************************/
void button_counter_decrease(uint8_t value)
{
	disable_irq();
	if (value <= button.pressed_counter)
		button.pressed_counter -= value;
	else
		button.pressed_counter = 0;
	enable_irq();
}

static void handle_usb_overcurrent(usb_port_t port)
{
	power_control_usb(port, false);
	power_control_usb_timeout_enable();
}

void input_signals_config(void)
{
	gpio_init_inputs(pin_pullup,
			 PCI_PLED0_PIN, PCI_PLED1_PIN, PCI_PLED2_PIN,
			 PCI_LLED1_PIN, PCI_LLED2_PIN, WAN_LED0_PIN,
			 WAN_LED1_PIN, CARD_DET_PIN, MSATA_LED_PIN,
			 MSATA_IND_PIN);

	gpio_init_inputs(pin_nopull,
			 R0_P0_LED_PIN, R1_P1_LED_PIN, R2_P2_LED_PIN,
			 C0_P3_LED_PIN, C1_LED_PIN, C2_P4_LED_PIN, C3_P5_LED_PIN);
}

/* Previously read values are needed for computing rising and falling edge */
static uint32_t prev_intr;
volatile uint32_t input_led_pins;

void input_signals_init(void)
{
	input_led_pins = led_pins_read(0);
	prev_intr = FIELD_PREP(INT_LED_STATES_MASK, input_led_pins);

	if (!gpio_read(CARD_DET_PIN))
		prev_intr |= INT_CARD_DET;

	if (gpio_read(MSATA_IND_PIN))
		prev_intr |= INT_MSATA_IND;

	if (OMNIA_BOARD_REVISION >= 32) {
		bool sfp_ndet = gpio_read(SFP_nDET_PIN);

		if (sfp_ndet)
			prev_intr |= INT_SFP_nDET;

		/* Initialize PHY/SFP switch */
		gpio_write(PHY_SFP_PIN, sfp_ndet);
	}

	disable_irq();

	button.user_mode = false;
	button.state = false;
	button.pressed_counter = 0;

	enable_irq();
}

/*******************************************************************************
  * @function   input_signals_handler
  * @brief      Check input signal.
  * @retval     Next state.
  *****************************************************************************/
input_req_t input_signals_handler(void)
{
	bool manres, sysres, mres, pg, pg_4v5, usb30_ovc, usb31_ovc;
	uint32_t intr = 0;

	power_input_pins_read(&manres, &sysres, &mres, &pg, &pg_4v5, &usb30_ovc,
			      &usb31_ovc);

	if (manres) {
		/* set CFG_CTRL pin to high state ASAP */
		gpio_write(CFG_CTRL_PIN, 1);

		return INPUT_REQ_LIGHT_RESET;
	}

	if (sysres)
		return INPUT_REQ_LIGHT_RESET;

	/* reaction: follow MRES signal */
	gpio_write(RES_RAM_PIN, mres);

	if (pg) {
		debug("PG fell low\n");
		return INPUT_REQ_HARD_RESET;
	}

	if (gpio_read_output(ENABLE_4V5_PIN) && pg_4v5) {
		debug("PG_4V5 fell low\n");
		return INPUT_REQ_HARD_RESET;
	}

	if (usb30_ovc) {
		intr |= INT_USB30_OVC;
		handle_usb_overcurrent(USB3_PORT0);
	}

	if (usb31_ovc) {
		intr |= INT_USB31_OVC;
		handle_usb_overcurrent(USB3_PORT1);
	}

	if (!gpio_read(CARD_DET_PIN))
		intr |= INT_CARD_DET;

	if (gpio_read(MSATA_IND_PIN))
		intr |= INT_MSATA_IND;

	/* read LED pins from WAN PHY, LAN switch and MiniPCIe ports */
	input_led_pins = led_pins_read(input_led_pins);
	intr |= FIELD_PREP(INT_LED_STATES_MASK, input_led_pins);

	disable_irq();

	if (button.user_mode && button.state)
		intr |= INT_BUTTON_PRESSED;

	if (OMNIA_BOARD_REVISION >= 32) {
		bool sfp_ndet = gpio_read(SFP_nDET_PIN);

		if (sfp_ndet)
			intr |= INT_SFP_nDET;

		/* change PHY/SFP switch if in automatic mode */
		if (i2c_iface.phy_sfp_auto)
			gpio_write(PHY_SFP_PIN, sfp_ndet);
	}

	/* compute rising / falling edges */
	i2c_iface.rising |= ~prev_intr & intr;
	i2c_iface.falling |= prev_intr & ~intr;

	i2c_iface_write_irq_pin();

	enable_irq();

	prev_intr = intr;

	return INPUT_REQ_NONE;
}
