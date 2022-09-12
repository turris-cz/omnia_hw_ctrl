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

static uint16_t get_addr(const boot_i2c_iface_priv_t *priv)
{
	return (priv->cmd[0] << 8) | priv->cmd[1];
}

static bool has_valid_addr(const boot_i2c_iface_priv_t *priv)
{
	uint16_t addr = get_addr(priv);

	return (addr & (PKT_SIZE - 1)) == 0 &&
	       ((uint32_t)addr + PKT_SIZE) <= APPLICATION_MAX_SIZE;
}

static void handle_cmd(boot_i2c_iface_priv_t *priv);

static void write_callback(bool success, void *ptr)
{
	boot_i2c_iface_priv_t *priv = ptr;

	if (!success)
		debug("app write failed\n");

	priv->cmd_len = 0;
	priv->cmd_valid = false;
	i2c_slave_resume(SLAVE_I2C);
}

static void erase_callback(bool success, void *ptr)
{
	boot_i2c_iface_priv_t *priv = ptr;

	priv->flash_erased = success;
	if (success) {
		flash_async_write(APPLICATION_BEGIN + get_addr(priv),
				  &priv->cmd[2], PKT_SIZE, write_callback,
				  priv);
	} else {
		debug("app erase failed\n");
		i2c_slave_resume(SLAVE_I2C);
	}
}

static void handle_cmd(boot_i2c_iface_priv_t *priv)
{
	if (!priv->flash_erased) {
		i2c_slave_pause(SLAVE_I2C);
		flash_async_erase(APPLICATION_BEGIN, APPLICATION_MAX_SIZE,
				  erase_callback, priv);
		priv->result = FLASH_CMD_RECEIVED;
	} else if (get_addr(priv) == ADDR_CMP) {
		if (priv->cmd[2] == FILE_CMP_OK)
			priv->result = FLASH_WRITE_OK;
		else
			priv->result = FLASH_WRITE_ERROR;

		priv->flash_erased = false;
		priv->tx_addr = 0;
		priv->cmd_len = 0;
		priv->cmd_valid = false;
	} else {
		i2c_slave_pause(SLAVE_I2C);
		flash_async_write(APPLICATION_BEGIN + get_addr(priv),
				  &priv->cmd[2], PKT_SIZE, write_callback,
				  priv);
		priv->result = FLASH_CMD_RECEIVED;
	}
}

static int boot_i2c_iface_event_cb(void *ptr, uint8_t addr,
				   i2c_slave_event_t event, uint8_t *val)
{
	boot_i2c_iface_priv_t *priv = ptr;

	if (!addr && event == I2C_SLAVE_RESET)
		/* if this is an early reset, reset the MCU command interface
		 * and also the bootloader interface
		 */
		i2c_iface_event_cb(&priv->mcu_cmd_iface, 0, event, val);

	else if (addr == MCU_I2C_ADDR)
		/* if this is a command for MCU I2C interface, handle MCU
		 * command interface
		 */
		return i2c_iface_event_cb(&priv->mcu_cmd_iface,
					  addr, event, val);

	else if (addr != BOOTLOADER_I2C_ADDR)
		/* otherwise it must be command for bootloader I2C interface */
		return -1;

	switch (event) {
	case I2C_SLAVE_READ_REQUESTED:
		priv->receiving = false;
		priv->tx_idx = 0;
		if (priv->cmd_len == 2 && has_valid_addr(priv)) {
			priv->tx_addr = get_addr(priv);
			priv->cmd_len = 0;
		}
		fallthrough;

	case I2C_SLAVE_READ_PROCESSED:
		if (priv->tx_idx >= PKT_SIZE ||
		    priv->tx_addr >= APPLICATION_MAX_SIZE)
			return -1;

		*val = *(uint8_t *)(APPLICATION_BEGIN + priv->tx_addr);
		priv->tx_addr++;
		priv->tx_idx++;

		break;

	case I2C_SLAVE_WRITE_REQUESTED:
		priv->cmd_len = 0;
		priv->cmd_valid = false;
		priv->receiving = true;
		break;

	case I2C_SLAVE_WRITE_RECEIVED:
		priv->cmd_valid = true;

		if (priv->cmd_len < sizeof(priv->cmd))
			priv->cmd[priv->cmd_len++] = *val;
		else
			priv->cmd_valid = false;

		/* check address validity */
		if (priv->cmd_len == 2 && get_addr(priv) != ADDR_CMP &&
		    !has_valid_addr(priv))
			priv->cmd_valid = false;

		/* ADDR_CMP command may contain only one data byte */
		if (priv->cmd_len > 3 && get_addr(priv) == ADDR_CMP)
			priv->cmd_valid = false;

		if (!priv->cmd_valid)
			return -1;

		break;

	case I2C_SLAVE_STOP:
		/* trigger flashing/comparing in boot_i2c_flash_data() */
		if (priv->receiving && priv->cmd_valid &&
		    priv->cmd_len > 2) {
			handle_cmd(priv);
		} else if (!priv->receiving) {
			priv->cmd_len = 0;
			priv->cmd_valid = false;
		}
		break;

	case I2C_SLAVE_RESET:
		priv->cmd_len = 0;
		priv->cmd_valid = false;
		priv->receiving = false;
		priv->tx_idx = 0;
		priv->tx_addr = 0;
		priv->result = FLASH_CMD_NOT_RECEIVED;
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
