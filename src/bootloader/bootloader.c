#include "power_control.h"
#include "time.h"
#include "message.h"
#include "debug.h"
#include "led_driver.h"
#include "flash.h"
#include "memory_layout.h"
#include "input.h"
#include "gpio.h"
#include "timer.h"
#include "cpu.h"
#include "i2c_iface.h"
#include "crc32.h"

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

	/* peripheral initialization*/
	crc32_enable();
	time_config();
	led_driver_config();
	i2c_iface_init();
	i2c_slave_init(SLAVE_I2C, &i2c_slave, MCU_I2C_ADDR, 0, 2);

	timer_reset(USB_TIMEOUT_TIMER);
	enable_irq();

	led_set_color24(LED_COUNT, GREEN_COLOR);

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
	uint32_t sys_reset_msg;
	message_t msg;

	if (get_sys_reset_message(&sys_reset_msg)) {
		debug("Application faulted with fault %#04x, staying in bootloader\n",
		      sys_reset_msg & 0x3f);
		return GO_TO_POWER_ON;
	}

	msg = get_message_after_switch();

	if (msg == STAY_IN_BOOTLOADER) {
		debug("Requsted to stay in bootloader\n");
		return GO_TO_FLASH;
	}

	if (check_app_crc())
		return GO_TO_APPLICATION;
	else
		return GO_TO_POWER_ON;
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
			input_signals_init();
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

		power_control_set_startup_condition();
		power_control_disable_regulators();
		msleep(100);

		power_control_enable_regulators();
		power_control_first_startup();

		led_set_color24(LED_COUNT, RED_COLOR);

		input_signals_init();

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
		nvic_system_reset();
		unreachable();
	}
}

void main(void)
{
	bootloader_init();

	while (1)
		bootloader();
}
