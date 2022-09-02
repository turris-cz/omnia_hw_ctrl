/**
 ******************************************************************************
 * @file    app.c
 * @author  CZ.NIC, z.s.p.o.
 * @date    25-September-2015
 * @brief   Init and cyclic high level operations.
 ******************************************************************************
 ******************************************************************************
 **/
/* Includes ------------------------------------------------------------------*/
#include "power_control.h"
#include "input.h"
#include "led_driver.h"
#include "i2c_iface.h"
#include "wan_lan_pci_msata.h"
#include "debug.h"
#include "eeprom.h"
#include "cpu.h"
#include "flash.h"
#include "memory_layout.h"
#include "time.h"
#include "watchdog.h"

#define MAX_ERROR_COUNT		5

typedef enum {
	OK			= 0,
	GO_TO_LIGHT_RESET	= 1,
	GO_TO_HARD_RESET	= 2,
	GO_TO_BOOTLOADER	= 3,
} ret_value_t;

typedef enum {
	POWER_ON,
	LIGHT_RESET,
	HARD_RESET,
	ERROR_STATE,
	INPUT_MANAGER,
	I2C_MANAGER,
	LED_MANAGER,
	BOOTLOADER
} states_t;

static i2c_iface_state_t i2c_iface_state;

static i2c_slave_t i2c_slave = {
	.cb = i2c_iface_event_cb,
	.priv = &i2c_iface_state,
};

/*******************************************************************************
  * @function   app_mcu_init
  * @brief      Initialization of MCU and its ports and peripherals.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
static void app_mcu_init(void)
{
	debug_init();

	flash_init(); /* Unlock the Flash Program Erase controller */

	time_config();
	/* init ports and peripheral */
	power_control_io_config();
	wan_lan_pci_msata_config();
	power_control_usb_timeout_config();
	led_driver_config();
	i2c_slave_init(SLAVE_I2C, &i2c_slave, MCU_I2C_ADDR,
		       LED_CONTROLLER_I2C_ADDR, 1);

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
  * @retval     value: next_state.
  *****************************************************************************/
static ret_value_t light_reset(void)
{
	ret_value_t value = OK;

	led_driver_reset_effect(DISABLE);

	power_control_first_startup();

	/* set active reset of peripherals after CPU reset on v32+ boards */
	if (OMNIA_BOARD_REVISION >= 32)
		periph_control_rst_init();

	watchdog_set_timeout(WATCHDOG_DEFAULT_TIMEOUT);
	watchdog_enable(true);

	led_driver_reset_effect(ENABLE);

	input_signals_config();
	i2c_iface.phy_sfp = true;
	i2c_iface.phy_sfp_auto = true;

	return value;
}

/*******************************************************************************
  * @function   i2c_manager
  * @brief      Handle I2C communication.
  * @param      None.
  * @retval     value: next_state.
  *****************************************************************************/
static ret_value_t i2c_manager(void)
{
	i2c_iface_write_irq_pin();

	switch (i2c_iface.req) {
	case I2C_IFACE_REQ_HARD_RESET:
		return GO_TO_HARD_RESET;

	case I2C_IFACE_REQ_BOOTLOADER:
		return GO_TO_BOOTLOADER;

	default:
		return OK;
	}
}

/*******************************************************************************
  * @function   led_manager
  * @brief      System LED activity (WAN, LAN, WiFi...).
  * @param      None.
  * @retval     next_state.
  *****************************************************************************/
static ret_value_t led_manager(void)
{
	wan_led_activity();
	lan_led_activity();
	pci_led_activity();
	msata_pci_activity();
	led_states_commit();

	return OK;
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

/*******************************************************************************
  * @function   app_mcu_cyclic
  * @brief      Main cyclic function.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
static void app_mcu_cyclic(void)
{
	static states_t next_state = POWER_ON;
	static ret_value_t val = OK;
	static int err;
	static uint8_t error_counter;

	switch (next_state) {
	case POWER_ON:
		err = power_on();

		if (!err)
			next_state = LIGHT_RESET;
		else
			next_state = ERROR_STATE;
		break;

	case LIGHT_RESET:
		val = light_reset();

		next_state = INPUT_MANAGER;
		break;

	case HARD_RESET:
		NVIC_SystemReset();
		break;

	case ERROR_STATE:
		error_manager(-err - 1);
		error_counter++;

		if (error_counter >= MAX_ERROR_COUNT) {
			next_state = HARD_RESET;
			error_counter = 0;
		} else {
			next_state = ERROR_STATE;
		}
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
		val = i2c_manager();

		switch (val) {
		case GO_TO_HARD_RESET:
			next_state = HARD_RESET;
			break;

		case GO_TO_BOOTLOADER:
			next_state = BOOTLOADER;
			break;

		default:
			next_state = LED_MANAGER;
			break;
		}
		break;

	case LED_MANAGER:
		if (effect_reset_finished)
			led_manager();

		next_state = INPUT_MANAGER;
		break;

	case BOOTLOADER:
		reset_to_address(BOOTLOADER_BEGIN);
		break;
	}
}

void main(void)
{
	enable_irq();

	app_mcu_init();

	while (1)
		app_mcu_cyclic();
}
