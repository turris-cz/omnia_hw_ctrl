/**
 ******************************************************************************
 * @file    bootloader.c
 * @author  CZ.NIC, z.s.p.o.
 * @date    17-April-2016
 * @brief   Bootloader state machine.
 ******************************************************************************
 ******************************************************************************
 **/
#include "power_control.h"
#include "time.h"
#include "eeprom.h"
#include "message.h"
#include "debug.h"
#include "bootloader.h"
#include "led_driver.h"
#include "flash.h"
#include "memory_layout.h"
#include "input.h"
#include "gpio.h"
#include "timer.h"
#include "cpu.h"
#include "i2c_iface.h"
#include "crc32.h"
#include "watchdog.h"

typedef enum {
	POWER_ON,
	STARTUP_MANAGER,
	INPUT_MANAGER,
	START_APPLICATION,
	RESET_TO_APPLICATION,
	HARD_RESET,
} boot_state_t;

typedef enum {
	GO_TO_POWER_ON,
	GO_TO_STARTUP_MANAGER,
	GO_TO_INPUT_MANAGER,
	GO_TO_FLASH,
	GO_TO_APPLICATION,
} boot_value_t;

static i2c_iface_priv_t i2c_iface_priv;

static i2c_slave_t i2c_slave = {
	.cb = i2c_iface_event_cb,
	.priv = &i2c_iface_priv,
};

/*******************************************************************************
  * @function   bootloader_init
  * @brief      Init of bootloader
  * @param      None
  * @retval     None
  *****************************************************************************/
static void bootloader_init(void)
{
	debug_init();

	flash_init(); /* Unlock the Flash Program Erase controller */
	EE_Init(); /* EEPROM Init */

	/* peripheral initialization*/
	crc32_enable();
	time_config();
	led_driver_config();
	i2c_iface_init();
	i2c_slave_init(SLAVE_I2C, &i2c_slave, MCU_I2C_ADDR, 0, 1);

	timer_reset(USB_TIMEOUT_TIMER);
	enable_irq();

	led_set_color24(LED_COUNT, GREEN_COLOR);
	led_driver_reset_effect(ENABLE);

	gpio_init_outputs(pin_opendrain, pin_spd_2, 1, SYSRES_OUT_PIN); /* dont control this ! */

	debug("Bootloader init\n");
}

static bool check_app_crc(void)
{
	uint32_t len, crc, res, zero = 0, len_before_csum;
	void *data;

	data = (void *)APPLICATION_BEGIN;
	len = *(uint32_t *)APPLICATION_CRCSUM;
	crc = *(uint32_t *)(APPLICATION_CRCSUM + 4);

	if (!len || len > APPLICATION_MAX_SIZE || len % 4) {
		debug("Invalid length stored in application checksum!\n");
		return 0;
	}

	len_before_csum = APPLICATION_CRCSUM - APPLICATION_BEGIN + 4;

	crc32(&res, 0, data, len_before_csum);
	crc32(&res, res, &zero, 4);
	crc32(&res, res, data + len_before_csum + 4,
	      len - (len_before_csum + 4));

	if (res == crc)
		debug("Application checksum OK\n");
	else
		debug("Application checksum FAILED\n");

	return res == crc;
}

/*******************************************************************************
  * @function   startup_manager
  * @brief      Determine a reset reason and following reaction.
  * @param      None
  * @retval     None
  *****************************************************************************/
static boot_value_t startup_manager(void)
{
	message_t msg;
	eeprom_var_t ee_var;
	uint16_t ee_data;
	boot_value_t retval = GO_TO_INPUT_MANAGER;

	msg = get_message_after_switch();

	if (msg == STAY_IN_BOOTLOADER) {
		debug("Requsted to stay in bootloader\n");
		return GO_TO_FLASH;
	}

	ee_var = EE_ReadVariable(RESET_VIRT_ADDR, &ee_data);

	/* power on reset - first boot - everything is flashed;
	   request for reflashing has never ocurred */

	switch (ee_var) {
	case VAR_NOT_FOUND:
		retval = GO_TO_APPLICATION;
		debug("R1\n");
		break;

	case VAR_FOUND:
		switch (ee_data) {
		case BOOTLOADER_REQ:
			retval = GO_TO_FLASH;
			EE_WriteVariable(RESET_VIRT_ADDR, FLASH_CONFIRMED);
			debug("req\n");
			break;

		case FLASH_NOT_CONFIRMED:
			/* error */
			retval = GO_TO_POWER_ON;
			EE_WriteVariable(RESET_VIRT_ADDR, FLASH_CONFIRMED);
			debug("ERR\n");
			break;

		case FLASH_CONFIRMED:
			/* application was flashed correctly */
			retval = GO_TO_APPLICATION;
			debug("R2\n");
			break;

		default:
			/* flag has not been saved correctly */
			retval = GO_TO_POWER_ON; break;
		}
		break;

	case VAR_NO_VALID_PAGE :
		retval = GO_TO_POWER_ON;
		debug("Boot-No valid page\n");
		break;

	default:
		break;
	}

	/*
	 * if EEprom variable says we should boot application but application has
	 * bad CRC, just power on
	 */
	if (retval == GO_TO_APPLICATION && !check_app_crc())
		retval = GO_TO_POWER_ON;

	return retval;
}

/*******************************************************************************
  * @function   bootloader
  * @brief      Main bootloader state machine.
  * @param      None
  * @retval     None
  *****************************************************************************/
static void bootloader(void)
{
	static boot_state_t next_state = STARTUP_MANAGER;
	static boot_value_t val = GO_TO_INPUT_MANAGER;

	switch (next_state) {
	case STARTUP_MANAGER:
		val = startup_manager();

		switch (val) {
		case GO_TO_POWER_ON:
			next_state = POWER_ON;
			break;

		case GO_TO_APPLICATION:
			next_state = START_APPLICATION;
			break;

		case GO_TO_FLASH:
			input_signals_config();
			next_state = INPUT_MANAGER;
			break;

		default:
			next_state = POWER_ON;
			break;
		}
		break;

	case POWER_ON:
		power_control_io_config();
		power_control_usb_timeout_config();
		if (OMNIA_BOARD_REVISION >= 32)
			periph_control_io_config();

		power_control_set_startup_condition();
		power_control_disable_regulators();
		msleep(100);

		power_control_enable_regulators();
		led_driver_reset_effect(DISABLE);
		power_control_first_startup();

		led_set_color24(LED_COUNT, RED_COLOR);
		led_driver_reset_effect(ENABLE);

		/* set active reset of peripherals after CPU reset on v32+
		 * boards
		 */
		if (OMNIA_BOARD_REVISION >= 32)
			periph_control_rst_init();

		watchdog_set_timeout(WATCHDOG_DEFAULT_TIMEOUT);
		watchdog_enable(true);

		input_signals_config();

		next_state = INPUT_MANAGER;
		break;

	case INPUT_MANAGER:
		switch (input_signals_handler()) {
		case INPUT_REQ_LIGHT_RESET:
			next_state = RESET_TO_APPLICATION;
			break;

		case INPUT_REQ_HARD_RESET:
			next_state = HARD_RESET;
			break;

		default:
			next_state = INPUT_MANAGER;
			break;
		}
		break;

	case START_APPLICATION:
		reset_to_address(APPLICATION_BEGIN);
		break;

	case RESET_TO_APPLICATION:
		/* shutdown regulators before reset, otherwise power supply can
		* stay there and causes wrong detection of mmc during boot */
		power_control_set_startup_condition();
		power_control_disable_regulators();
		msleep(100);
		fallthrough;

	case HARD_RESET:
		NVIC_SystemReset();
		unreachable();
	}
}

void main(void)
{
	bootloader_init();

	while (1)
		bootloader();
}
