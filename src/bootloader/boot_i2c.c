#include "debug.h"
#include "led_driver.h"
#include "power_control.h"
#include "eeprom.h"
#include "flash.h"
#include "i2c_iface.h"
#include "boot_i2c.h"
#include "bootloader.h"
#include "gpio.h"
#include "memory_layout.h"

__attribute__((section(".boot_version"))) uint8_t version[20] = VERSION;

#define PKT_SIZE		128

#define FILE_CMP_OK		0xBB
#define FILE_CMP_ERROR		0xDD
#define ADDR_CMP		0xFFFF

typedef struct {
	uint8_t cmd[2 + PKT_SIZE];
	uint8_t cmd_len;
	uint8_t tx_idx;
	uint16_t tx_addr;
	bool flash_erased;
	bool receiving;
	bool cmd_valid;
	boot_i2c_result_t result;
	i2c_iface_priv_t mcu_cmd_iface;
} boot_i2c_iface_priv_t;

static uint16_t get_addr(const boot_i2c_iface_priv_t *state)
{
	return (state->cmd[0] << 8) | state->cmd[1];
}

static bool has_valid_addr(const boot_i2c_iface_priv_t *state)
{
	uint16_t addr = get_addr(state);

	return (addr & (PKT_SIZE - 1)) == 0 &&
	       ((uint32_t)addr + PKT_SIZE) <= APPLICATION_MAX_SIZE;
}

static void handle_cmd(boot_i2c_iface_priv_t *state);

static void write_callback(bool success, void *priv)
{
	boot_i2c_iface_priv_t *state = priv;

	if (!success)
		debug("app write failed\n");

	state->cmd_len = 0;
	state->cmd_valid = false;
	i2c_slave_resume(SLAVE_I2C);
}

static void erase_callback(bool success, void *priv)
{
	boot_i2c_iface_priv_t *state = priv;

	state->flash_erased = success;
	if (success) {
		flash_async_write(APPLICATION_BEGIN + get_addr(state),
				  &state->cmd[2], PKT_SIZE, write_callback,
				  state);
	} else {
		debug("app erase failed\n");
		i2c_slave_resume(SLAVE_I2C);
	}
}

static void handle_cmd(boot_i2c_iface_priv_t *state)
{
	if (!state->flash_erased) {
		i2c_slave_pause(SLAVE_I2C);
		flash_async_erase(APPLICATION_BEGIN, APPLICATION_MAX_SIZE,
				  erase_callback, state);
		state->result = FLASH_CMD_RECEIVED;
	} else if (get_addr(state) == ADDR_CMP) {
		if (state->cmd[2] == FILE_CMP_OK)
			state->result = FLASH_WRITE_OK;
		else
			state->result = FLASH_WRITE_ERROR;

		state->flash_erased = false;
		state->tx_addr = 0;
		state->cmd_len = 0;
		state->cmd_valid = false;
	} else {
		i2c_slave_pause(SLAVE_I2C);
		flash_async_write(APPLICATION_BEGIN + get_addr(state),
				  &state->cmd[2], PKT_SIZE, write_callback,
				  state);
		state->result = FLASH_CMD_RECEIVED;
	}
}

static int boot_i2c_iface_event_cb(void *priv, uint8_t addr,
				   i2c_slave_event_t event, uint8_t *val)
{
	boot_i2c_iface_priv_t *state = priv;

	if (!addr && event == I2C_SLAVE_RESET)
		/* if this is an early reset, reset the MCU command interface
		 * and also the bootloader interface
		 */
		i2c_iface_event_cb(&state->mcu_cmd_iface, 0, event, val);

	else if (addr == MCU_I2C_ADDR)
		/* if this is a command for MCU I2C interface, handle MCU
		 * command interface
		 */
		return i2c_iface_event_cb(&state->mcu_cmd_iface,
					  addr, event, val);

	else if (addr != BOOTLOADER_I2C_ADDR)
		/* otherwise it must be command for bootloader I2C interface */
		return -1;

	switch (event) {
	case I2C_SLAVE_READ_REQUESTED:
		state->receiving = false;
		state->tx_idx = 0;
		if (state->cmd_len == 2 && has_valid_addr(state)) {
			state->tx_addr = get_addr(state);
			state->cmd_len = 0;
		}
		fallthrough;

	case I2C_SLAVE_READ_PROCESSED:
		if (state->tx_idx >= PKT_SIZE ||
		    state->tx_addr >= APPLICATION_MAX_SIZE)
			return -1;

		*val = *(uint8_t *)(APPLICATION_BEGIN + state->tx_addr);
		state->tx_addr++;
		state->tx_idx++;

		break;

	case I2C_SLAVE_WRITE_REQUESTED:
		state->cmd_len = 0;
		state->cmd_valid = false;
		state->receiving = true;
		break;

	case I2C_SLAVE_WRITE_RECEIVED:
		state->cmd_valid = true;

		if (state->cmd_len < sizeof(state->cmd))
			state->cmd[state->cmd_len++] = *val;
		else
			state->cmd_valid = false;

		/* check address validity */
		if (state->cmd_len == 2 && get_addr(state) != ADDR_CMP &&
		    !has_valid_addr(state))
			state->cmd_valid = false;

		/* ADDR_CMP command may contain only one data byte */
		if (state->cmd_len > 3 && get_addr(state) == ADDR_CMP)
			state->cmd_valid = false;

		if (!state->cmd_valid)
			return -1;

		break;

	case I2C_SLAVE_STOP:
		/* trigger flashing/comparing in boot_i2c_flash_data() */
		if (state->receiving && state->cmd_valid &&
		    state->cmd_len > 2) {
			handle_cmd(state);
		} else if (!state->receiving) {
			state->cmd_len = 0;
			state->cmd_valid = false;
		}
		break;

	case I2C_SLAVE_RESET:
		state->cmd_len = 0;
		state->cmd_valid = false;
		state->receiving = false;
		state->tx_idx = 0;
		state->tx_addr = 0;
		state->result = FLASH_CMD_NOT_RECEIVED;
		break;
	}

	return 0;
}

static boot_i2c_iface_priv_t boot_i2c_iface_priv;

static i2c_slave_t i2c_slave = {
	.cb = boot_i2c_iface_event_cb,
	.priv = &boot_i2c_iface_priv,
};

void boot_i2c_config(void)
{
	i2c_iface_init();
	i2c_slave_init(SLAVE_I2C, &i2c_slave, MCU_I2C_ADDR, BOOTLOADER_I2C_ADDR,
		       1);
}

boot_i2c_result_t boot_i2c_result(void)
{
	return boot_i2c_iface_priv.result;
}
