#include "compiler.h"
#include "string.h"
#include "i2c_iface.h"
#include "debug.h"
#include "led_driver.h"
#include "power_control.h"
#include "input.h"
#include "memory_layout.h"
#include "watchdog.h"
#include "crc32.h"
#include "time.h"
#include "poweroff.h"

#if BOOTLOADER_BUILD
__attribute__((__section__(".boot_version")))
#endif
static __used const uint8_t version[] = VERSION;
static __maybe_unused struct {
	uint32_t length;
	uint32_t crcsum;
} app_checksum __section(".crcsum");

#define FEAT_IF(feat, cond)	((cond) ? FEAT_ ## feat : 0)
#define FEATURES_MAGIC		0xfea70235

__attribute__((__section__(".features")))
static const struct {
	uint32_t magic;
	uint16_t features;
	uint8_t status_features;
	uint8_t reserved;
	uint32_t csum;
} slave_features_supported = {
	.magic = FEATURES_MAGIC,
	.features =
		FEAT_IF(PERIPH_MCU, OMNIA_BOARD_REVISION >= 32) |
		FEAT_IF(LED_GAMMA_CORRECTION, !BOOTLOADER_BUILD) |
		FEAT_IF(BOOTLOADER, BOOTLOADER_BUILD) |
		FEAT_IF(LED_STATE_EXT, OMNIA_BOARD_REVISION < 32) |
		FEAT_IF(LED_STATE_EXT_V32, OMNIA_BOARD_REVISION >= 32) |
		FEAT_NEW_INT_API |
		FEAT_WDT_PING |
		FEAT_EXT_CMDS |
		FEAT_FLASHING |
		FEAT_NEW_MESSAGE_API |
		FEAT_IF(POWEROFF_WAKEUP, POWEROFF_WAKEUP_ENABLED),
	.status_features =
		STS_MCU_TYPE |
		STS_FEATURES_SUPPORTED |
		(USER_REGULATOR_ENABLED ? 0 : STS_USER_REGULATOR_NOT_SUPPORTED),
};

enum boot_request_e {
	BOOTLOADER_REQ	= 0xAA,
	FLASH_ERROR	= 0x55,
	FLASH_OK	= 0x88
};

i2c_iface_t i2c_iface;

static const struct {
	gpio_t pin;
	uint16_t bit;
	bool invert;
} sts_pins[] = {
#define STS(name, inv) \
	{ name ## _PIN, STS_ ## name, inv }
	STS(CARD_DET, true),
	STS(MSATA_IND, false),
	STS(USB30_OVC, true),
	STS(USB31_OVC, true),
	STS(USB30_PWRON, true),
	STS(USB31_PWRON, true),
#if USER_REGULATOR_ENABLED
	STS(ENABLE_4V5, false),
#endif
#undef STS
};

static const struct {
	gpio_t pin;
	uint32_t bit;
} ext_sts_pins[] = {
#define ESTS(name) \
	{ name ## _PIN, EXT_STS_ ## name }
#if OMNIA_BOARD_REVISION >= 32
	ESTS(SFP_nDET),
#endif
#undef ESTS
};

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

static inline void _set_reply(i2c_iface_priv_t *priv, const void *reply,
			      uint32_t len)
{
	compiletime_assert(len <= sizeof(priv->reply), "reply too long");

	memcpy(priv->reply, reply, len);
	priv->reply_len = len;
}

#define set_reply(x) _set_reply(priv, &(x), sizeof(x))

static __maybe_unused int cmd_get_features(i2c_iface_priv_t *priv)
{
	if (priv->cmd_len == 1) {
		/* Report the features of the running firmware */
		debug("get_features %#06x\n",
		      slave_features_supported.features);
		set_reply(slave_features_supported.features);
	} else if (priv->cmd_len == 2) {
		/*
		 * When received second argument, we report the features of the
		 * requested firmware (bootloader for 0xbb, application for
		 * 0xaa).
		 */
		typeof(slave_features_supported) *ptr;
		uint32_t features;
		uint32_t csum;

		/* clear reply from when cmd_len == 1 */
		priv->reply_len = 0;

		/* don't read from flash if flashing is busy */
		if (priv->flashing.state == FLASHING_BUSY)
			return -1;

		switch (priv->cmd[1]) {
		case 0xaa:
			ptr = (const void *)APPLICATION_FEATURES;
			break;
		case 0xbb:
			ptr = (const void *)BOOTLOADER_FEATURES;
			break;
		default:
			return -1;
		}

		if (ptr->magic != FEATURES_MAGIC)
			return -1;

		crc32(&csum, 0, ptr, 8);
		if (csum != ptr->csum)
			return -1;

		features = ptr->features | (ptr->status_features << 16);

		debug("get_features(%#04x) %#010x\n", priv->cmd[1],
		      features);
		set_reply(features);
	} else if (priv->cmd_len > 2) {
		priv->reply_len = 0;
		return -1;
	}

	return 0;
}

static void on_get_status_success(i2c_iface_priv_t *priv)
{
	uint16_t reply;
	uint8_t cntr;

	/* decrease replied button counter by the value that was successfully
	 * sent to master
	 */
	reply = priv->reply[0] | (priv->reply[1] << 8);
	cntr = FIELD_GET(STS_BUTTON_COUNTER_MASK, reply);

	button_counter_decrease(cntr);
}

static __maybe_unused int cmd_get_status(i2c_iface_priv_t *priv)
{
	uint16_t status = STS_MCU_TYPE | STS_FEATURES_SUPPORTED;

	if (!USER_REGULATOR_ENABLED)
		status |= STS_USER_REGULATOR_NOT_SUPPORTED;

	for_each_const(pin, sts_pins)
		status |= (gpio_read(pin->pin) ^ pin->invert) ? pin->bit : 0;

	status |= FIELD_PREP(STS_BUTTON_COUNTER_MASK, button.pressed_counter);
	if (button.user_mode)
		status |= STS_BUTTON_MODE;
	if (button.state)
		status |= STS_BUTTON_PRESSED;

	debug("get_status %#06x\n", status);
	set_reply(status);

	priv->on_success = on_get_status_success;

	return 0;
}

static void on_general_control_success(i2c_iface_priv_t *priv)
{
	uint8_t ctrl, mask, set;

	ctrl = priv->cmd[1];
	mask = priv->cmd[2];
	set = ctrl & mask;

	debug("general_control ctrl=%#06x mask=%#06x\n", ctrl, mask);

	/* these are ignored in bootloader */
	if (!BOOTLOADER_BUILD) {
		if (set & CTL_LIGHT_RST) {
			/* set CFG_CTRL pin to high state ASAP */
			gpio_write(CFG_CTRL_PIN, 1);
			/* reset of CPU */
			gpio_write(MANRES_PIN, 0);
			return;
		}

		if (set & CTL_HARD_RST) {
			i2c_iface.req = I2C_IFACE_REQ_HARD_RESET;
			return;
		}
	}

	if (mask & CTL_USB30_PWRON)
		power_control_usb(USB3_PORT0, ctrl & CTL_USB30_PWRON);

	if (mask & CTL_USB31_PWRON)
		power_control_usb(USB3_PORT1, ctrl & CTL_USB31_PWRON);

#if USER_REGULATOR_ENABLED
	if (mask & CTL_ENABLE_4V5)
		gpio_write(ENABLE_4V5_PIN, ctrl & CTL_ENABLE_4V5);
#endif

	if (mask & CTL_BUTTON_MODE)
		button_set_user_mode(ctrl & CTL_BUTTON_MODE);

	if (!BOOTLOADER_BUILD && (set & CTL_BOOTLOADER))
		i2c_iface.req = I2C_IFACE_REQ_BOOTLOADER;
}

static __maybe_unused int cmd_general_control(i2c_iface_priv_t *priv)
{
	uint8_t ctrl, mask, set;

	ctrl = priv->cmd[1];
	mask = priv->cmd[2];
	set = ctrl & mask;

	if (!BOOTLOADER_BUILD &&
	    (set & (CTL_LIGHT_RST | CTL_HARD_RST | CTL_BOOTLOADER))
	    && priv->flashing.state == FLASHING_BUSY)
		return -1;

	priv->on_success = on_general_control_success;

	return 0;
}

static __maybe_unused int cmd_get_ext_status(i2c_iface_priv_t *priv)
{
	uint32_t ext_status = 0;

	for_each_const(pin, ext_sts_pins)
		ext_status |= gpio_read(pin->pin) ? pin->bit : 0;

	/* fill in LED pin states */
	ext_status |= FIELD_PREP(EXT_STS_LED_STATES_MASK, input_led_pins);

	debug("get_ext_status %#010x\n", ext_status);
	set_reply(ext_status);

	return 0;
}

static __maybe_unused int cmd_ext_control(i2c_iface_priv_t *priv)
{
	uint8_t *args = &priv->cmd[1];
	uint16_t ext_ctrl, mask;

	ext_ctrl = args[0] | (args[1] << 8);
	mask = args[2] | (args[3] << 8);

	debug("ext_control ctrl=%#06x mask=%#06x\n", ext_ctrl, mask);

	/* don't do anything on pre-v32 boards */
	if (OMNIA_BOARD_REVISION < 32)
		return 0;

	/*
	 * PHY_SFP_AUTO isn't a GPIO, rather an internal setting.
	 * If set, we let PHY_SFP to be set in app.c' input_manager() according to
	 * value read from SFP_nDET, so we don't change it here.
	 * If not set, we want to set PHY_SFP according to the last configured
	 * value. This is why we also need to remember the last configure value.
	 */
	if (mask & EXT_CTL_PHY_SFP)
		i2c_iface.phy_sfp = ext_ctrl & EXT_CTL_PHY_SFP;
	if (mask & EXT_CTL_PHY_SFP_AUTO)
		i2c_iface.phy_sfp_auto = ext_ctrl & EXT_CTL_PHY_SFP_AUTO;

	if (i2c_iface.phy_sfp_auto) {
		/* Drop PHY_SFP from mask if automatic switching enabled,
		 * so that the for cycle below does not touch it.
		 */
		mask &= ~EXT_CTL_PHY_SFP;
	} else if (!(mask & EXT_CTL_PHY_SFP)) {
		/* Add PHY_SFP to mask and set it in ext_ctrl according to
		 * remembered value.
		 */
		mask |= EXT_CTL_PHY_SFP;
		ext_ctrl &= ~EXT_CTL_PHY_SFP;
		if (i2c_iface.phy_sfp)
			ext_ctrl |= EXT_CTL_PHY_SFP;
	}

	for_each_const(pin, ext_ctrl_pins)
		if (mask & pin->mask)
			gpio_write(pin->pin, !!(ext_ctrl & pin->mask));

	return 0;
}

static __maybe_unused int cmd_get_ext_control_status(i2c_iface_priv_t *priv)
{
	uint16_t ext_ctrl = 0;

	if (OMNIA_BOARD_REVISION >= 32) {
		for_each_const(pin, ext_ctrl_pins)
			if (gpio_read(pin->pin))
				ext_ctrl |= pin->mask;

		/* PHY_SFP_AUTO isn't a GPIO, rather an internal setting about
		 * behavior
		 */
		if (i2c_iface.phy_sfp_auto)
			ext_ctrl |= EXT_CTL_PHY_SFP_AUTO;
	}

	debug("get_ext_control_status st=%#06x\n", ext_ctrl);
	set_reply(ext_ctrl);

	return 0;
}

/* Interleaves bytes from two 32-bit values:
 *   v1 = abcd
 *   v2 = ABCD
 *   dst = aAbBcCdD
 */
static void interleave(uint8_t *dst, uint32_t v1, uint32_t v2)
{
	for (int i = 0; i < 4; ++i) {
		dst[2 * i] = v1 >> (8 * i);
		dst[2 * i + 1] = v2 >> (8 * i);
	}
}

/* Deinterleaves bytes into two 32-bit values:
 *   src = aAbBcCdD
 *   *v1 = abcd
 *   *v2 = ABCD
 */
static void deinterleave(const uint8_t *src, uint32_t *v1, uint32_t *v2)
{
	*v1 = *v2 = 0;

	for (int i = 0; i < 4; ++i) {
		*v1 |= src[2 * i] << (8 * i);
		*v2 |= src[2 * i + 1] << (8 * i);
	}
}

static void on_get_int_and_clear_failure(i2c_iface_priv_t *priv)
{
	uint32_t rising, falling;

	debug("get_int_and_clear failed, setting flags back\n");

	deinterleave(priv->reply, &rising, &falling);

	i2c_iface.rising |= rising;
	i2c_iface.falling |= falling;

	i2c_iface_write_irq_pin();
}

static __maybe_unused int cmd_get_int_and_clear(i2c_iface_priv_t *priv)
{
	uint8_t intr[8];

	interleave(intr, i2c_iface.rising, i2c_iface.falling);
	set_reply(intr);

	debug("cmd_get_int_and_clear rising=%#010x falling=%#010x\n",
	      i2c_iface.rising, i2c_iface.falling);

	/* Clear now. If the I2C transaction fails, we will set the cleared
	 * values again in on_get_int_and_clear_failure().
	 */
	i2c_iface.rising = i2c_iface.falling = 0;
	i2c_iface_write_irq_pin();

	priv->on_failure = on_get_int_and_clear_failure;

	return 0;
}

static __maybe_unused int cmd_get_int_mask(i2c_iface_priv_t *priv)
{
	uint8_t mask[8];

	interleave(mask, i2c_iface.rising_mask, i2c_iface.falling_mask);
	set_reply(mask);

	debug("cmd_get_int_mask rising=%#010x falling=%#010x\n",
	      i2c_iface.rising_mask, i2c_iface.falling_mask);

	return 0;
}

static __maybe_unused int cmd_set_int_mask(i2c_iface_priv_t *priv)
{
	uint8_t *args = &priv->cmd[1];

	deinterleave(args, &i2c_iface.rising_mask, &i2c_iface.falling_mask);

	debug("cmd_set_int_mask rising=%#010x falling=%#010x\n",
	      i2c_iface.rising_mask, i2c_iface.falling_mask);

	return 0;
}

static __maybe_unused int cmd_get_reset(i2c_iface_priv_t *priv)
{
	debug("get_reset\n");
	set_reply(i2c_iface.reset_selector);

	return 0;
}

#if USER_REGULATOR_ENABLED
static __maybe_unused int cmd_user_voltage(i2c_iface_priv_t *priv)
{
	debug("user_voltage\n");
	power_control_set_voltage(priv->cmd[1]);

	return 0;
}
#endif

static __maybe_unused int cmd_led_mode(i2c_iface_priv_t *priv)
{
	uint8_t *args = &priv->cmd[1];

	debug("led_mode\n");
	led_set_user_mode(args[0] & 0x0F, !!(args[0] & 0x10));

	return 0;
}

static __maybe_unused int cmd_led_state(i2c_iface_priv_t *priv)
{
	uint8_t *args = &priv->cmd[1];

	debug("led_state\n");
	led_set_state_user(args[0] & 0x0F, !!(args[0] & 0x10));

	return 0;
}

static __maybe_unused int cmd_led_color(i2c_iface_priv_t *priv)
{
	uint8_t *args = &priv->cmd[1];

	debug("led_color\n");
	led_set_color(args[0] & 0x0F, args[1], args[2], args[3]);

	return 0;
}

static __maybe_unused int cmd_set_brightness(i2c_iface_priv_t *priv)
{
	debug("set_brightness\n");
	led_driver_set_brightness(priv->cmd[1]);

	return 0;
}

static __maybe_unused int cmd_get_brightness(i2c_iface_priv_t *priv)
{
	uint8_t brightness = led_driver_get_brightness();

	debug("get_brightness\n");
	set_reply(brightness);

	return 0;
}

static __maybe_unused int cmd_set_gamma_correction(i2c_iface_priv_t *priv)
{
	debug("set_gamma_correction\n");
	led_driver_set_gamma_correction(priv->cmd[1] & BIT(0));

	return 0;
}

static __maybe_unused int cmd_get_gamma_correction(i2c_iface_priv_t *priv)
{
	uint8_t gamma_correction = led_driver_get_gamma_correction();

	debug("get_gamma_correction\n");
	set_reply(gamma_correction);

	return 0;
}

static __maybe_unused int cmd_set_watchdog_state(i2c_iface_priv_t *priv)
{
	debug("watchdog_state\n");

	watchdog_enable(priv->cmd[1]);

	return 0;
}

static __maybe_unused int cmd_get_watchdog_state(i2c_iface_priv_t *priv)
{
	uint8_t enabled = watchdog_is_enabled();

	debug("get_watchdog_state\n");
	set_reply(enabled);

	return 0;
}

static __maybe_unused int cmd_set_wdt_timeout(i2c_iface_priv_t *priv)
{
	uint8_t *args = &priv->cmd[1];

	debug("set_wdt_timeout\n");
	watchdog_set_timeout(args[0] | (args[1] << 8));

	return 0;
}

static __maybe_unused int cmd_get_wdt_timeleft(i2c_iface_priv_t *priv)
{
	uint16_t timeleft = watchdog_get_timeleft();

	debug("get_wdt_timeleft\n");
	set_reply(timeleft);

	return 0;
}

static __maybe_unused int cmd_get_version(i2c_iface_priv_t *priv)
{
	const void *ptr;

	debug("get_version\n");

	switch (priv->cmd[0]) {
	case CMD_GET_FW_VERSION_BOOT:
		if (BOOTLOADER_BUILD) {
			ptr = version;
		} else {
			if (priv->flashing.state == FLASHING_BUSY)
				return -1;

			ptr = (void *)BOOTLOADER_VERSION_POS;
		}

		priv->reply_len = 20;
		__builtin_memcpy(priv->reply, ptr, 20);
		break;

#if !BOOTLOADER_BUILD
	case CMD_GET_FW_VERSION_APP:
		priv->reply_len = 20;
		__builtin_memcpy(priv->reply, version, 20);
		break;

	case CMD_GET_FW_CHECKSUM:
		priv->reply_len = 8;
		__builtin_memcpy(priv->reply, &app_checksum, 8);
		break;
#endif

	default:
		unreachable();
	}

	return 0;
}

static __maybe_unused int cmd_set_wakeup(i2c_iface_priv_t *priv)
{
	uint32_t wakeup = get_unaligned32(&priv->cmd[1]);

	debug("set_wakeup %u -> %u\n", i2c_iface.wakeup, wakeup);

	i2c_iface.wakeup = wakeup;

	return 0;
}

static __maybe_unused int cmd_get_uptime_and_wakeup(i2c_iface_priv_t *priv)
{
	uint32_t reply[2] = { uptime, i2c_iface.wakeup };

	debug("get_wakeup %u, %u\n", reply[0], reply[1]);
	set_reply(reply);

	return 0;
}

static __maybe_unused int cmd_power_off(i2c_iface_priv_t *priv)
{
	uint32_t crc, expected_crc;
	uint16_t magic, arg;
	uint32_t wakeup_timeout;

	magic = get_unaligned16(&priv->cmd[1]);
	arg = get_unaligned16(&priv->cmd[3]);
	crc = get_unaligned32(&priv->cmd[5]);

	crc32(&expected_crc, 0xffffffff, &priv->cmd[1], 4);

	if (magic != 0xdead) {
		debug("power_off arg=%#06x invalid magic (got %#06x)\n", arg,
		      magic);
		return -1;
	}

	if (crc != expected_crc) {
		debug("power_off arg=%#06x invalid crc (got %#010x, expected %#010x)\n",
		      arg, crc, expected_crc);
		return -1;
	}

	debug("power_off %#06x\n", arg);

	led_driver_set_brightness(0);

	disable_irq();

	/*
	 * Disable regulators and stop driving interrupt pin to SOC.
	 * (With SOC voltage disabled, driving this pin eats around 0.2 W.)
	 */
	power_control_disable_regulators();
	gpio_init_inputs(pin_nopull, INT_MCU_PIN);

	if (i2c_iface.wakeup > uptime)
		wakeup_timeout = i2c_iface.wakeup - uptime;
	else
		wakeup_timeout = 0;

	platform_poweroff(arg & 1, wakeup_timeout);

	return 0;
}

typedef struct {
	uint8_t len;
	int (*handler)(i2c_iface_priv_t *priv);
	enum {
		MCU_ADDR_ONLY = 0,
		LED_ADDR_ONLY,
		BOTH_ADDRS,
	} availability;
} cmdinfo_t;

static const cmdinfo_t commands[] = {
	/* control & status */
	[CMD_GET_FEATURES]		= { 0, cmd_get_features },
	[CMD_GET_STATUS_WORD]		= { 1, cmd_get_status },
	[CMD_GENERAL_CONTROL]		= { 3, cmd_general_control },
	[CMD_GET_EXT_STATUS_DWORD]	= { 1, cmd_get_ext_status },
	[CMD_EXT_CONTROL]		= { 5, cmd_ext_control },
	[CMD_GET_EXT_CONTROL_STATUS]	= { 1, cmd_get_ext_control_status },
	[CMD_GET_RESET]			= { 1, cmd_get_reset },
#if USER_REGULATOR_ENABLED
	[CMD_USER_VOLTAGE]		= { 2, cmd_user_voltage },
#endif

	[CMD_GET_INT_AND_CLEAR]		= { 1, cmd_get_int_and_clear },
	[CMD_GET_INT_MASK]		= { 1, cmd_get_int_mask },
	[CMD_SET_INT_MASK]		= { 9, cmd_set_int_mask },

#if !BOOTLOADER_BUILD
	/* LEDs */
	[CMD_LED_MODE]			= { 2, cmd_led_mode, BOTH_ADDRS },
	[CMD_LED_STATE]			= { 2, cmd_led_state, BOTH_ADDRS },
	[CMD_LED_COLOR]			= { 5, cmd_led_color, BOTH_ADDRS },
	[CMD_SET_BRIGHTNESS]		= { 2, cmd_set_brightness, BOTH_ADDRS },
	[CMD_GET_BRIGHTNESS]		= { 1, cmd_get_brightness, BOTH_ADDRS },

	/* LEDs, commands only available at LED controller I2C address */
	[CMD_SET_GAMMA_CORRECTION]	= { 2, cmd_set_gamma_correction, LED_ADDR_ONLY },
	[CMD_GET_GAMMA_CORRECTION]	= { 1, cmd_get_gamma_correction, LED_ADDR_ONLY },

	/* version info */
	[CMD_GET_FW_VERSION_APP]	= { 1, cmd_get_version },
	[CMD_GET_FW_CHECKSUM]		= { 1, cmd_get_version },
#endif /* !BOOTLOADER_BUILD */
	[CMD_GET_FW_VERSION_BOOT]	= { 1, cmd_get_version },

	/* flashing */
	[CMD_FLASH]			= { 0, cmd_flash },

	/* watchdog */
	[CMD_SET_WATCHDOG_STATE]	= { 2, cmd_set_watchdog_state },
	[CMD_GET_WATCHDOG_STATE]	= { 1, cmd_get_watchdog_state },
	[CMD_SET_WDT_TIMEOUT]		= { 3, cmd_set_wdt_timeout },
	[CMD_GET_WDT_TIMELEFT]		= { 1, cmd_get_wdt_timeleft },

#if POWEROFF_WAKEUP_ENABLED
	/* wakeup & power off */
	[CMD_SET_WAKEUP]		= { 5, cmd_set_wakeup },
	[CMD_GET_UPTIME_AND_WAKEUP]	= { 1, cmd_get_uptime_and_wakeup },
	[CMD_POWER_OFF]			= { 9, cmd_power_off },
#endif
};

static int handle_cmd(uint8_t addr, i2c_iface_priv_t *priv)
{
	uint8_t cmdidx = priv->cmd[0];
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

	if (!cmd->len)
		return cmd->handler(priv);
	else if (priv->cmd_len < cmd->len)
		return 0;
	else if (priv->cmd_len == cmd->len)
		return cmd->handler(priv);
	else
		return -1;
}

int i2c_iface_event_cb(void *ptr, uint8_t addr, i2c_slave_event_t event,
		       uint8_t *val)
{
	i2c_iface_priv_t *priv = ptr;

	switch (event) {
	case I2C_SLAVE_READ_PROCESSED:
		priv->reply_idx++;
		fallthrough;

	case I2C_SLAVE_READ_REQUESTED:
		if (priv->reply_idx >= priv->reply_len)
			return -1;

		*val = priv->reply[priv->reply_idx];
		break;

	case I2C_SLAVE_WRITE_REQUESTED:
		break;

	case I2C_SLAVE_WRITE_RECEIVED:
		if (priv->cmd_len < sizeof(priv->cmd))
			priv->cmd[priv->cmd_len++] = *val;
		else
			return -1;

		return handle_cmd(addr, priv);

	case I2C_SLAVE_STOP:
	case I2C_SLAVE_RESET:
		if (addr) {
			if (event == I2C_SLAVE_STOP && priv->on_success)
				priv->on_success(priv);
			else if (event == I2C_SLAVE_RESET && priv->on_failure)
				priv->on_failure(priv);
		}

		priv->cmd_len = 0;
		priv->reply_len = 0;
		priv->reply_idx = 0;
		priv->on_success = NULL;
		priv->on_failure = NULL;
		break;
	}

	return 0;
}
