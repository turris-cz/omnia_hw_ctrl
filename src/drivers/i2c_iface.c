/**
 ******************************************************************************
 * @file    i2c_iface.c
 * @author  CZ.NIC, z.s.p.o.
 * @date    18-August-2015
 * @brief   Driver for IC2 communication with master device (main CPU).
 ******************************************************************************
 ******************************************************************************
 **/
/* Includes ------------------------------------------------------------------*/
#include "compiler.h"
#include "string.h"
#include "i2c_iface.h"
#include "debug.h"
#include "led_driver.h"
#include "wan_lan_pci_msata.h"
#include "power_control.h"
#include "input.h"
#include "eeprom.h"
#include "memory_layout.h"
#include "watchdog.h"

static const uint8_t version[] = VERSION;
static struct {
	uint32_t length;
	uint32_t crcsum;
} app_checksum __section(".crcsum");

static const uint16_t slave_features_supported =
#if OMNIA_BOARD_REVISION >= 32
	FEAT_PERIPH_MCU |
#endif
	FEAT_EXT_CMDS |
	FEAT_WDT_PING |
	FEAT_LED_GAMMA_CORRECTION;

enum boot_request_e {
	BOOTLOADER_REQ	= 0xAA,
	FLASH_ERROR	= 0x55,
	FLASH_OK	= 0x88
};

i2c_iface_t i2c_iface;

static const struct {
	gpio_t pin;
	uint16_t mask;
} ext_ctrl_pins[] = {
#define ECTRL(name) \
	{ name ## _PIN, EXT_CTL_ ## name }
	ECTRL(nRES_MMC),
	ECTRL(nRES_LAN),
	ECTRL(nRES_PHY),
	ECTRL(nPERST0),
	ECTRL(nPERST1),
	ECTRL(nPERST2),
	ECTRL(PHY_SFP),
	ECTRL(nVHV_CTRL),
#undef ECTRL
};

static inline void _set_reply(i2c_iface_state_t *state, const void *reply,
			      uint32_t len)
{
	compiletime_assert(len <= sizeof(state->reply), "reply too long");

	memcpy(state->reply, reply, len);
	state->reply_len = len;
}

#define set_reply(x) _set_reply(state, &(x), sizeof(x))

static int cmd_get_features(i2c_iface_state_t *state)
{
	debug("get_features\n");
	set_reply(slave_features_supported);

	return 0;
}

static int cmd_get_status(i2c_iface_state_t *state)
{
	debug("get_status\n");
	set_reply(i2c_iface.status_word);

	return 0;
}

static void handle_usb_power(uint8_t ctrl, uint8_t mask, usb_port_t port,
			     uint8_t ctrl_bit, uint16_t sts_bit)
{
	if (mask & ctrl_bit) {
		power_control_usb(port, ctrl & ctrl_bit);
		if (ctrl & ctrl_bit)
			i2c_iface.status_word |= sts_bit;
		else
			i2c_iface.status_word &= ~sts_bit;
	}
}

static int cmd_general_control(i2c_iface_state_t *state)
{
	struct button_def *button = &button_front;
	uint8_t ctrl, mask, set;

	ctrl = state->cmd[1];
	mask = state->cmd[2];
	set = ctrl & mask;

	debug("general_control ctrl=%#06x mask=%#06x\n", ctrl, mask);

	if (set & CTL_LIGHT_RST) {
		/* set CFG_CTRL pin to high state ASAP */
		gpio_write(CFG_CTRL_PIN, 1);
		/* reset of CPU */
		gpio_write(MANRES_PIN, 0);
		return 0;
	}

	if (set & CTL_HARD_RST) {
		i2c_iface.req = I2C_IFACE_REQ_HARD_RESET;
		return 0;
	}

	handle_usb_power(ctrl, mask, USB3_PORT0, CTL_USB30_PWRON,
			 STS_USB30_PWRON);
	handle_usb_power(ctrl, mask, USB3_PORT1, CTL_USB31_PWRON,
			 STS_USB31_PWRON);

#if USER_REGULATOR_ENABLED
	if (mask & CTL_ENABLE_4V5) {
		if (ctrl & CTL_ENABLE_4V5) {
			gpio_write(ENABLE_4V5_PIN, 1);
		} else {
			gpio_write(ENABLE_4V5_PIN, 0);
			i2c_iface.status_word &= ~STS_ENABLE_4V5;
		}
	}
#endif

	if (mask & CTL_BUTTON_MODE) {
		if (ctrl & CTL_BUTTON_MODE) {
			button->button_mode = BUTTON_USER;
			i2c_iface.status_word |= STS_BUTTON_MODE;
		} else {
			button->button_mode = BUTTON_DEFAULT;
			button->button_pressed_counter = 0;
			i2c_iface.status_word &= ~STS_BUTTON_MODE;
		}
	}

	if (set & CTL_BOOTLOADER) {
		eeprom_var_t ee_var;

		EE_Init();
		ee_var = EE_WriteVariable(RESET_VIRT_ADDR, BOOTLOADER_REQ);

		switch (ee_var) {
		case VAR_FLASH_COMPLETE:
			debug("RST: OK\n");
			break;

		case VAR_PAGE_FULL:
			debug("RST: Pg full\n");
			break;

		case VAR_NO_VALID_PAGE:
			debug("RST: No Pg\n");
			break;

		default:
			break;
		}

		i2c_iface.req = I2C_IFACE_REQ_BOOTLOADER;
	}

	return 0;
}

static int cmd_get_ext_status(i2c_iface_state_t *state)
{
	debug("get_ext_status\n");
	set_reply(i2c_iface.ext_status_dword);

	return 0;
}

static int cmd_ext_control(i2c_iface_state_t *state)
{
	uint8_t *args = &state->cmd[1];
	uint16_t ext_ctrl, mask;

	ext_ctrl = args[0] | (args[1] << 8);
	mask = args[2] | (args[3] << 8);

	debug("ext_control ctrl=%#06x mask=%#06x\n", ext_ctrl, mask);

	/* don't do anything on pre-v32 boards */
	if (OMNIA_BOARD_REVISION < 32)
		return 0;

	/* save the requested value */
	ext_ctrl = (i2c_iface.ext_control_word & ~mask) | (ext_ctrl & mask);
	i2c_iface.ext_control_word = ext_ctrl;

	/*
	 * PHY_SFP_AUTO isn't a GPIO, rather an internal setting.
	 * If set, we let PHY_SFP to be set in app.c' input_manager() according to
	 * value read from SFP_nDET, so we don't change it here.
	 * If not set, we want to set PHY_SFP according to value in
	 * ext_ctrl.
	 */
	if (ext_ctrl & EXT_CTL_PHY_SFP_AUTO)
		mask &= ~EXT_CTL_PHY_SFP;
	else
		mask |= EXT_CTL_PHY_SFP;

	for_each_const(pin, ext_ctrl_pins)
		if (mask & pin->mask)
			gpio_write(pin->pin, !!(ext_ctrl & pin->mask));

	return 0;
}

static int cmd_get_ext_control_status(i2c_iface_state_t *state)
{
	uint16_t ext_ctrl_st = 0;

	if (OMNIA_BOARD_REVISION >= 32) {
		for_each_const(pin, ext_ctrl_pins)
			if (gpio_read(pin->pin))
				ext_ctrl_st |= pin->mask;
	}

	/* PHY_SFP_AUTO isn't a GPIO, rather an internal setting about behavior */
	ext_ctrl_st |= i2c_iface.ext_control_word & EXT_CTL_PHY_SFP_AUTO;

	debug("get_ext_control_status st=%#06x\n", ext_ctrl_st);
	set_reply(ext_ctrl_st);

	return 0;
}

static int cmd_get_reset(i2c_iface_state_t *state)
{
	debug("get_reset\n");
	set_reply(i2c_iface.reset_selector);

	return 0;
}

#if USER_REGULATOR_ENABLED
static int cmd_user_voltage(i2c_iface_state_t *state)
{
	debug("user_voltage\n");
	power_control_set_voltage(state->cmd[1]);

	return 0;
}
#endif

static int cmd_led_mode(i2c_iface_state_t *state)
{
	uint8_t *args = &state->cmd[1];

	debug("led_mode\n");
	led_set_user_mode(args[0] & 0x0F, !!(args[0] & 0x10));

	return 0;
}

static int cmd_led_state(i2c_iface_state_t *state)
{
	uint8_t *args = &state->cmd[1];

	debug("led_state\n");
	led_set_state_user(args[0] & 0x0F, !!(args[0] & 0x10));

	return 0;
}

static int cmd_led_color(i2c_iface_state_t *state)
{
	uint8_t *args = &state->cmd[1];

	debug("led_color\n");
	led_set_color(args[0] & 0x0F, args[1], args[2], args[3]);

	return 0;
}

static int cmd_set_brightness(i2c_iface_state_t *state)
{
	debug("set_brightness\n");
	led_driver_set_brightness(state->cmd[1]);

	return 0;
}

static int cmd_get_brightness(i2c_iface_state_t *state)
{
	uint8_t brightness = led_driver_get_brightness();

	debug("get_brightness\n");
	set_reply(brightness);

	return 0;
}

static int cmd_set_gamma_correction(i2c_iface_state_t *state)
{
	debug("set_gamma_correction\n");
	led_driver_set_gamma_correction(state->cmd[1] & BIT(0));

	return 0;
}

static int cmd_get_gamma_correction(i2c_iface_state_t *state)
{
	uint8_t gamma_correction = led_driver_get_gamma_correction();

	debug("get_gamma_correction\n");
	set_reply(gamma_correction);

	return 0;
}

static int cmd_set_watchdog_state(i2c_iface_state_t *state)
{
	debug("watchdog_state\n");

	watchdog_enable(state->cmd[1]);

	return 0;
}

static int cmd_get_watchdog_state(i2c_iface_state_t *state)
{
	uint8_t enabled = watchdog_is_enabled();

	debug("get_watchdog_state\n");
	set_reply(enabled);

	return 0;
}

static int cmd_set_wdt_timeout(i2c_iface_state_t *state)
{
	uint8_t *args = &state->cmd[1];

	debug("set_wdt_timeout\n");
	watchdog_set_timeout(args[0] | (args[1] << 8));

	return 0;
}

static int cmd_get_wdt_timeleft(i2c_iface_state_t *state)
{
	uint16_t timeleft = watchdog_get_timeleft();

	debug("get_wdt_timeleft\n");
	set_reply(timeleft);

	return 0;
}

static int cmd_get_version(i2c_iface_state_t *state)
{
	debug("get_version\n");

	switch (state->cmd[0]) {
	case CMD_GET_FW_VERSION_BOOT:
		state->reply_len = 20;
		__builtin_memcpy(state->reply, (void *)BOOTLOADER_VERSION_POS, 20);
		break;

	case CMD_GET_FW_VERSION_APP:
		state->reply_len = 20;
		__builtin_memcpy(state->reply, version, 20);
		break;

	case CMD_GET_FW_CHECKSUM:
		state->reply_len = 8;
		__builtin_memcpy(state->reply, &app_checksum, 8);
		break;

	default:
		unreachable();
	}

	return 0;
}

typedef struct {
	uint8_t len;
	int (*handler)(i2c_iface_state_t *state);
	enum {
		MCU_ADDR_ONLY = 0,
		LED_ADDR_ONLY,
		BOTH_ADDRS,
	} availability;
} cmdinfo_t;

static const cmdinfo_t commands[] = {
	/* control & status */
	[CMD_GET_FEATURES]		= { 1, cmd_get_features },
	[CMD_GET_STATUS_WORD]		= { 1, cmd_get_status },
	[CMD_GENERAL_CONTROL]		= { 3, cmd_general_control },
	[CMD_GET_EXT_STATUS_DWORD]	= { 1, cmd_get_ext_status },
	[CMD_EXT_CONTROL]		= { 5, cmd_ext_control },
	[CMD_GET_EXT_CONTROL_STATUS]	= { 1, cmd_get_ext_control_status },
	[CMD_GET_RESET]			= { 1, cmd_get_reset },
#if USER_REGULATOR_ENABLED
	[CMD_USER_VOLTAGE]		= { 2, cmd_user_voltage },
#endif

	/* LEDs */
	[CMD_LED_MODE]			= { 2, cmd_led_mode, BOTH_ADDRS },
	[CMD_LED_STATE]			= { 2, cmd_led_state, BOTH_ADDRS },
	[CMD_LED_COLOR]			= { 5, cmd_led_color, BOTH_ADDRS },
	[CMD_SET_BRIGHTNESS]		= { 2, cmd_set_brightness, BOTH_ADDRS },
	[CMD_GET_BRIGHTNESS]		= { 1, cmd_get_brightness, BOTH_ADDRS },

	/* LEDs, commands only available at LED controller I2C address */
	[CMD_SET_GAMMA_CORRECTION]	= { 2, cmd_set_gamma_correction, LED_ADDR_ONLY },
	[CMD_GET_GAMMA_CORRECTION]	= { 1, cmd_get_gamma_correction, LED_ADDR_ONLY },

	/* watchdog */
	[CMD_SET_WATCHDOG_STATE]	= { 2, cmd_set_watchdog_state },
	[CMD_GET_WATCHDOG_STATE]	= { 1, cmd_get_watchdog_state },
	[CMD_SET_WDT_TIMEOUT]		= { 3, cmd_set_wdt_timeout },
	[CMD_GET_WDT_TIMELEFT]		= { 1, cmd_get_wdt_timeleft },

	/* version info */
	[CMD_GET_FW_VERSION_APP]	= { 1, cmd_get_version },
	[CMD_GET_FW_VERSION_BOOT]	= { 1, cmd_get_version },
	[CMD_GET_FW_CHECKSUM]		= { 1, cmd_get_version },
};

static int handle_cmd(uint8_t addr, i2c_iface_state_t *state)
{
	uint8_t cmdidx = state->cmd[0];
	const cmdinfo_t *cmd;

	if (cmdidx >= ARRAY_SIZE(commands))
		return -1;

	cmd = &commands[cmdidx];
	if (!cmd->handler)
		return -1;

	if (cmd->availability != BOTH_ADDRS &&
	    !(cmd->availability == MCU_ADDR_ONLY && addr == MCU_I2C_ADDR) &&
	    !(cmd->availability == LED_ADDR_ONLY && addr == LED_CONTROLLER_I2C_ADDR))
		return -1;

	if (state->cmd_len < cmd->len)
		return 0;
	else if (state->cmd_len == cmd->len)
		return cmd->handler(state);
	else
		return -1;
}

int i2c_iface_event_cb(void *priv, uint8_t addr, i2c_slave_event_t event,
		       uint8_t *val)
{
	i2c_iface_state_t *state = priv;

	switch (event) {
	case I2C_SLAVE_READ_PROCESSED:
		state->reply_idx++;
		fallthrough;

	case I2C_SLAVE_READ_REQUESTED:
		if (state->reply_idx >= state->reply_len)
			return -1;

		*val = state->reply[state->reply_idx];
		break;

	case I2C_SLAVE_WRITE_REQUESTED:
		break;

	case I2C_SLAVE_WRITE_RECEIVED:
		if (state->cmd_len < sizeof(state->cmd))
			state->cmd[state->cmd_len++] = *val;
		else
			return -1;

		return handle_cmd(addr, state);

	case I2C_SLAVE_STOP:
		/* delete button status and counter bit from status_word */
		if (state->cmd[0] == CMD_GET_STATUS_WORD) {
			i2c_iface.status_word &= ~STS_BUTTON_PRESSED;

			/* decrease button counter */
			button_counter_decrease(FIELD_GET(STS_BUTTON_COUNTER_MASK, i2c_iface.status_word));
		}
		fallthrough;

	case I2C_SLAVE_RESET:
		state->cmd_len = 0;
		state->reply_len = 0;
		state->reply_idx = 0;
		break;
	}

	return 0;
}
