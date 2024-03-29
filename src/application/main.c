#include "power_control.h"
#include "input.h"
#include "led_driver.h"
#include "i2c_iface.h"
#include "wan_lan_pci_msata.h"
#include "debug.h"
#include "message.h"
#include "cpu.h"
#include "flash.h"
#include "memory_layout.h"
#include "time.h"
#include "watchdog.h"
#include "crc32.h"

#define MAX_ERROR_COUNT		5

typedef enum {
	POWER_ON,
	LIGHT_RESET,
	HARD_RESET,
	ERROR_STATE,
	INPUT_MANAGER,
	I2C_MANAGER,
	LED_MANAGER,
	BOOTLOADER
} state_t;

static i2c_iface_priv_t i2c_iface_priv;

static i2c_slave_t i2c_slave = {
	.cb = i2c_iface_event_cb,
	.priv = &i2c_iface_priv,
};

/*******************************************************************************
  * @function   app_init
  * @brief      Initialization of MCU and its ports and peripherals.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
static void app_init(void)
{
	debug_init();

	flash_init(); /* Unlock the Flash Program Erase controller */

	time_config();
	/* init ports and peripheral */
	crc32_enable();
	power_control_io_config();
	wan_lan_pci_msata_config();
	power_control_usb_timeout_config();
	led_driver_config();

	/* new features for Omnia32 */
	if (OMNIA_BOARD_REVISION >= 32)
		periph_control_io_config();

	debug("\nInit completed.\n");
}

/*******************************************************************************
  * @function   power_on
  * @brief      Start the board / enable dc-dc regulators.
  * @param      None.
  * @retval     0 on success, -n if enableing n-th regulator failed.
  *****************************************************************************/
static int power_on(void)
{
	power_control_set_startup_condition();
	power_control_disable_regulators();

	msleep(100);

	return power_control_enable_regulators();
}

/*******************************************************************************
  * @function   light_reset
  * @brief      Perform light reset of the board.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
static void light_reset(void)
{
	led_driver_reset_effect(false);

	disable_irq();
	i2c_iface_init();
	i2c_slave_init(SLAVE_I2C, &i2c_slave, MCU_I2C_ADDR,
		       LED_CONTROLLER_I2C_ADDR, 1);
	enable_irq();

	power_control_first_startup();

	/* set active reset of peripherals after CPU reset on v32+ boards */
	if (OMNIA_BOARD_REVISION >= 32)
		periph_control_rst_init();

	watchdog_set_timeout(WATCHDOG_DEFAULT_TIMEOUT);
	watchdog_enable(true);

	led_driver_reset_effect(true);

	input_signals_config();
}

/*******************************************************************************
  * @function   led_manager
  * @brief      System LED activity (WAN, LAN, WiFi...).
  * @param      None.
  * @retval     None.
  *****************************************************************************/
static void led_manager(void)
{
	wan_led_activity();
	lan_led_activity();
	pci_led_activity();
	msata_pci_activity();
	led_states_commit();
}

/*******************************************************************************
  * @function   error_manager
  * @brief      Handle error occuring in startup.
  * @param      err_led: LED index indicating the error
  * @retval     None.
  *****************************************************************************/
static void error_manager(unsigned led)
{
	led_set_user_mode(LED_COUNT, false);
	led_set_state(LED_COUNT, false);
	led_set_color24(LED_COUNT, RED_COLOR);

	msleep(300);

	led_set_state(led, true);

	msleep(300);
}

void main(void)
{
	state_t next_state = POWER_ON;
	uint8_t error_counter = 0;
	int err;

	enable_irq();

	app_init();

	while (1) {
		switch (next_state) {
		case POWER_ON:
			err = power_on();

			if (!err)
				next_state = LIGHT_RESET;
			else
				next_state = ERROR_STATE;
			break;

		case LIGHT_RESET:
			light_reset();

			next_state = INPUT_MANAGER;
			break;

		case HARD_RESET:
			NVIC_SystemReset();
			unreachable();

		case ERROR_STATE:
			error_manager(-err - 1);
			error_counter++;

			if (error_counter >= MAX_ERROR_COUNT)
				next_state = HARD_RESET;
			break;

		case INPUT_MANAGER:
			switch (input_signals_handler()) {
			case INPUT_REQ_LIGHT_RESET:
				next_state = LIGHT_RESET;
				break;

			case INPUT_REQ_HARD_RESET:
				next_state = HARD_RESET;
				break;

			default:
				next_state = I2C_MANAGER;
				break;
			}
			break;

		case I2C_MANAGER:
			switch (i2c_iface.req) {
			case I2C_IFACE_REQ_HARD_RESET:
				next_state = HARD_RESET;
				break;

			case I2C_IFACE_REQ_BOOTLOADER:
				next_state = BOOTLOADER;
				break;

			default:
				next_state = LED_MANAGER;
				break;
			}
			break;

		case LED_MANAGER:
			if (!led_driver_reset_effect_in_progress())
				led_manager();

			next_state = INPUT_MANAGER;
			break;

		case BOOTLOADER:
			set_message_before_switch(STAY_IN_BOOTLOADER);
			reset_to_address(BOOTLOADER_BEGIN);
		}
	}
}
