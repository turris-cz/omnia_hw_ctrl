/**
 ******************************************************************************
 * @file    power_control.c
 * @author  CZ.NIC, z.s.p.o.
 * @date    22-July-2015
 * @brief   Functions for control of DC/DC converters.
 ******************************************************************************
 ******************************************************************************
 **/
/* Includes ------------------------------------------------------------------*/
#include "cpu.h"
#include "power_control.h"
#include "time.h"
#include "led_driver.h"
#include "slave_i2c_device.h"
#include "debug.h"
#include "timer.h"

#if !defined(OMNIA_BOARD_REVISION)
#error build system did not define OMNIA_BOARD_REVISION macro
#endif

#if !defined(USER_REGULATOR_ENABLED)
#error build system did not define USER_REGULATOR_ENABLED macro
#endif

#if USER_REGULATOR_ENABLED && OMNIA_BOARD_REVISION >= 32
#error user regulator not supported on board revision 32 and newer
#endif

/* Private define ------------------------------------------------------------*/

/* defines for timeout handling during regulator startup */
#define DELAY_AFTER_ENABLE		5
#define DELAY_BETWEEN_READINGS		20
/* DELAY_BETWEEN_READINGS * 100 = 2 sec */
#define TIMEOUT				100

#define RESET_SELECTOR_LEVEL_TIMEOUT	10

/* sequence of enumerator entires is also power-up sequence */
typedef enum {
	REG_5V,
	REG_3V3,
	REG_1V8,
	REG_1V5,
	REG_1V35,
	REG_VTT,
	REG_1V2,
	REG_MAX,
} reg_idx_t;

/*******************************************************************************
  * @function   system_control_io_config
  * @brief      GPIO config for EN, PG, Reset and USB signals.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void power_control_io_config(void)
{
	/* Output signals */
	gpio_init_outputs(pin_pushpull, pin_spd_2, 0,
			  RES_RAM_PIN, ENABLE_5V_PIN, ENABLE_3V3_PIN,
			  ENABLE_1V35_PIN, ENABLE_4V5_PIN, ENABLE_1V8_PIN,
			  ENABLE_1V5_PIN, ENABLE_1V2_PIN, ENABLE_VTT_PIN,
			  USB30_PWRON_PIN, USB31_PWRON_PIN, CFG_CTRL_PIN);
	gpio_init_outputs(pin_pushpull, pin_spd_2, 1, INT_MCU_PIN);

	gpio_init_outputs(pin_opendrain, pin_spd_2, 0, MANRES_PIN);
	gpio_init_outputs(pin_opendrain, pin_spd_2, 1, SYSRES_OUT_PIN); /* dont control this ! */

	/* Input signals */
	gpio_init_inputs(pin_pullup,
			 PG_5V_PIN, PG_3V3_PIN, PG_1V35_PIN, PG_4V5_PIN,
			 PG_1V8_PIN, PG_1V5_PIN, PG_1V2_PIN, PG_VTT_PIN,
			 USB30_OVC_PIN, USB31_OVC_PIN, LED_BRT_PIN,
			 DBGRES_PIN, MRES_PIN, RTC_ALARM_PIN);

#if USER_REGULATOR_ENABLED
	gpio_init_outputs(pin_pushpull, pin_spd_2, 0, PRG_4V5_PIN);
#endif
}

/*******************************************************************************
  * @function   power_control_set_startup_condition
  * @brief      Set signals to reset state before board startup.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void power_control_set_startup_condition(void)
{
	gpio_write(CFG_CTRL_PIN, 1); /* disconnect switches */
	gpio_write(MANRES_PIN, 0); /* board reset activated */
	power_control_usb(USB3_PORT0, true);
	power_control_usb(USB3_PORT1, true);
}

typedef struct {
	gpio_t en, pg;
} regulator_t;

static const regulator_t regulators[] = {
#define DEF_REG(n) \
	[REG_ ## n] = { ENABLE_ ## n ## _PIN, PG_ ## n ## _PIN }
	DEF_REG(5V),
	DEF_REG(3V3),
	DEF_REG(1V8),
	DEF_REG(1V5),
	DEF_REG(1V35),
	DEF_REG(VTT),
	DEF_REG(1V2),
#undef DEF_REG
};

static bool power_control_start_regulator(const regulator_t *reg)
{
	uint16_t timeout = TIMEOUT;

	/* Some boards don't have some regulators, for example board revisions
	 * 32 and later do not have 1V5 regulator.
	 * During power-on we report a failure of power-good of a regulator by
	 * setting a specific LED to red color. In order to have consistent
	 * reporting across all boards revisions, the regulator array defines
	 * all regulators, even those that are invalid for specific boards.
	 * Pretend that regulator started up successfully if it does not exist.
	 */
	if (reg->en == PIN_INVALID)
		return true;

	/* enable regulator */
	gpio_write(reg->en, 1);

	/* wait until power-good */
	do {
		delay(DELAY_BETWEEN_READINGS);
		if (gpio_read(reg->pg))
			return true;
	} while (--timeout);

	return false;
}

/*******************************************************************************
  * @function   power_control_enable_regulators
  * @brief      Starts DC/DC regulators.
  * @param      None.
  * @retval     0 on success, -n if enableing n-th regulator failed.
  *****************************************************************************/
int power_control_enable_regulators(void)
{
	/*
	 * power-up sequence:
	 * 1) 5V regulator
	 * 2) 3.3V regulator
	 * 3) 1.8V regulator
	 * 4) 1.5V regulator - not populated on the board
	 * 5) 1.35V regulator
	 * 6) VTT regulator
	 * 7) 1.2V regulator
	 */
	const regulator_t *reg;
	int i;

	for (reg = &regulators[0], i = 1;
	     reg < &regulators[ARRAY_SIZE(regulators)];
	     ++reg, ++i)
		if (!power_control_start_regulator(reg))
			return -i;

	return 0;
}

/*******************************************************************************
  * @function   power_control_disable_regulators
  * @brief      Shutdown DC/DC regulators.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void power_control_disable_regulators(void)
{
	/* don't collapse this into one call of gpio_write_multi(), since these
	 * should be disabled in the given order */
	gpio_write(ENABLE_1V2_PIN, 0);
	gpio_write(ENABLE_1V35_PIN, 0);
	gpio_write(ENABLE_VTT_PIN, 0);
	gpio_write(ENABLE_1V5_PIN, 0);
	gpio_write(ENABLE_1V8_PIN, 0);
	gpio_write(ENABLE_3V3_PIN, 0);
	gpio_write(ENABLE_4V5_PIN, 0);
	gpio_write(ENABLE_5V_PIN, 0);
}

/*******************************************************************************
  * @function   power_control_usb
  * @brief      Enable / disable power supply for USB.
  * @param      usb_port: USB3_PORT0 or USB3_PORT1.
  * @param      on: true or false.
  * @retval     None.
  *****************************************************************************/
void power_control_usb(usb_port_t usb_port, bool on)
{
	gpio_write(usb_port == USB3_PORT0 ? USB30_PWRON_PIN : USB31_PWRON_PIN,
		   !on);
}

/*******************************************************************************
  * @function   power_control_get_usb_overcurrent
  * @brief      Get USB overcurrent status.
  * @param      usb_port: USB3_PORT0 or USB3_PORT1.
  * @retval     1 - USB overcurrent ocurred; 0 - no USB overcurrent
  *****************************************************************************/
bool power_control_get_usb_overcurrent(usb_port_t usb_port)
{
	return !gpio_read(usb_port == USB3_PORT0 ? USB30_OVC_PIN : USB31_OVC_PIN);
}

/*******************************************************************************
  * @function   power_control_get_usb_poweron
  * @brief      Get USB poweron status.
  * @param      usb_port: USB3_PORT0 or USB3_PORT1.
  * @retval     1 - USB power ON; 0 - USB power OFF
  *****************************************************************************/
bool power_control_get_usb_poweron(usb_port_t usb_port)
{
	return !gpio_read(usb_port == USB3_PORT0 ? USB30_PWRON_PIN : USB31_PWRON_PIN);
}

/*******************************************************************************
  * @function   power_control_usb_timeout_config
  * @brief      Timer configuration for USB recovery timeout.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void power_control_usb_timeout_config(void)
{
	timer_init(USB_TIMEOUT_TIMER, timer_interrupt, 8000, 8000, 5);
}

/*******************************************************************************
  * @function   power_control_usb_timeout_enable
  * @brief      Enable USB recovery timeout.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void power_control_usb_timeout_enable(void)
{
	timer_enable(USB_TIMEOUT_TIMER, true);
}

/*******************************************************************************
  * @function   power_control_usb_timeout_disable
  * @brief      Disable USB recovery timeout.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void power_control_usb_timeout_disable(void)
{
	/* disable timer and set initial condition */
	timer_enable(USB_TIMEOUT_TIMER, false);
	timer_set_counter(USB_TIMEOUT_TIMER, 0);
}

#if !BOOTLOADER_BUILD
/*******************************************************************************
  * @function   power_control_usb_timeout_irq_handler
  * @brief      Handle USB timeout.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void __irq power_control_usb_timeout_irq_handler(void)
{
	struct st_i2c_status *i2c_control = &i2c_status;

	if (!timer_irq_clear_up(USB_TIMEOUT_TIMER))
		return;

	power_control_usb(USB3_PORT0, true);
	power_control_usb(USB3_PORT1, true);

	i2c_control->status_word |= STS_USB30_PWRON | STS_USB31_PWRON;

	power_control_usb_timeout_disable();
}
#endif

static void increase_reset_selector_level(int *sel, uint8_t *level)
{
	static const uint8_t hue2rgb[128] = {
		  0,  23,  32,  39,  45,  50,  55,  60,  64,  68,  71,  75,  78,  81,  84,  87,
		 90,  93,  96,  98, 101, 103, 106, 108, 111, 113, 115, 117, 119, 122, 124, 126,
		128, 130, 132, 134, 135, 137, 139, 141, 143, 145, 146, 148, 150, 151, 153, 155,
		156, 158, 160, 161, 163, 164, 166, 167, 169, 170, 172, 173, 175, 176, 178, 179,
		181, 182, 183, 185, 186, 188, 189, 190, 192, 193, 194, 196, 197, 198, 199, 201,
		202, 203, 204, 206, 207, 208, 209, 211, 212, 213, 214, 215, 217, 218, 219, 220,
		221, 222, 224, 225, 226, 227, 228, 229, 230, 231, 233, 234, 235, 236, 237, 238,
		239, 240, 241, 242, 243, 244, 245, 246, 247, 248, 249, 250, 251, 252, 253, 254,
	};
	unsigned led;
	uint8_t r, g;

	if (*level == 0) {
		(*sel)++;
		if (*sel == 12) {
			led_set_state(LED_COUNT, false);
			*sel = 0;
		}
	}

	led = 11 - *sel;

	if (*level < 128) {
		r = hue2rgb[*level];
		g = 255;
	} else {
		r = 255;
		g = hue2rgb[255 - *level];
	}

	led_set_color(led, r, g, 0);
	if (*level == 0)
		led_set_state(led, true);

	(*level)++;
}

/*******************************************************************************
  * @function   power_control_first_startup
  * @brief      Handle SYSRES_OUT, MAN_RES, CFG_CTRL signals and factory reset
  *             during startup.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void power_control_first_startup(void)
{
	uint8_t prev_brightness, level;
	int sel;

	gpio_write(CFG_CTRL_PIN, 1);
	delay(50);
	gpio_write(MANRES_PIN, 1);

	if (BOOTLOADER_BUILD) {
		while (!gpio_read(SYSRES_OUT_PIN))
			delay(1);
	} else {
		unsigned timeout;

		/* save brightness value to restore it */
		prev_brightness = led_driver_get_brightness();
		led_set_state(LED_COUNT, false);
		led_driver_set_brightness(100);

		timeout = RESET_SELECTOR_LEVEL_TIMEOUT;
		level = 0;
		sel = -1;

		/* Read main board reset signal every 1ms.
		 * Increase reset selector level every 10ms.
		 */
		while (!gpio_read(SYSRES_OUT_PIN)) {
			delay(1);

			if (timeout--)
				continue;

			timeout = RESET_SELECTOR_LEVEL_TIMEOUT;

			increase_reset_selector_level(&sel, &level);
		}
	}

	/* 15ms delay after release of reset signal */
	delay(15);
	gpio_write(CFG_CTRL_PIN, 0);

	if (BOOTLOADER_BUILD)
		return;

	if (sel < 0)
		sel = 0;

	/* if not a normal reset, blink the selected reset selector */
	if (sel) {
		led_driver_set_brightness(0);
		delay(300);
		led_driver_set_brightness(100);
		delay(300);
		led_driver_set_brightness(0);
		delay(300);
		led_driver_set_brightness(100);
		delay(600);
	}

	/* restore brightness and color */
	led_set_state(LED_COUNT, false);
	led_driver_set_brightness(prev_brightness);

	i2c_status.reset_selector = sel;
}

#if USER_REGULATOR_ENABLED
# if SYS_CORE_FREQ == 48000000U
#  define LONG_NOP	30
#  define SHORT_NOP	6
# elif SYS_CORE_FREQ == 72000000U
#  define LONG_NOP	45
#  define SHORT_NOP	9
# else
#  error "user regulator control parameters not defined for this platform"
# endif

static __force_inline void nop_repeat(unsigned count)
{
#pragma GCC unroll 1000
	for (unsigned i = 0; i < count; ++i)
		nop();
}

/* Programming pin for user regulator.
 * Timing for logic '1' and '0' consists only of NOPs, because it must be very
 * precise. Pulse for logic '1' or '0' takes only 1 us.
 */
static __force_inline void user_reg_prg_logic(bool val)
{
	if (val) {
		gpio_write(PRG_4V5_PIN, 1);
		nop_repeat(LONG_NOP);
		gpio_write(PRG_4V5_PIN, 0);
		nop_repeat(SHORT_NOP);
	} else {
		gpio_write(PRG_4V5_PIN, 1);
		nop_repeat(SHORT_NOP);
		gpio_write(PRG_4V5_PIN, 0);
		nop_repeat(LONG_NOP);
	}
}

static __force_inline void user_reg_write_data(uint8_t data)
{
	/* start condition */
	user_reg_prg_logic(1);

	/* chip select */
	user_reg_prg_logic(0);
	user_reg_prg_logic(1);
	user_reg_prg_logic(0);
	user_reg_prg_logic(1);

	/* register address */
	user_reg_prg_logic(0);
	user_reg_prg_logic(0);
	user_reg_prg_logic(1);
	user_reg_prg_logic(0);

	/* datafield - 0xDF */
	user_reg_prg_logic((data >> 7) & 1);
	user_reg_prg_logic((data >> 6) & 1);
	user_reg_prg_logic((data >> 5) & 1);
	user_reg_prg_logic((data >> 4) & 1);

	user_reg_prg_logic((data >> 3) & 1);
	user_reg_prg_logic((data >> 2) & 1);
	user_reg_prg_logic((data >> 1) & 1);
	user_reg_prg_logic((data >> 0) & 1);

	/* stop condition */
	user_reg_prg_logic(1);
}

/*******************************************************************************
  * @function   power_control_set_voltage
  * @brief      Set required voltage to the user regulator.
  * @param      voltage: enum value for desired voltage.
  * @retval     None.
  *****************************************************************************/
void power_control_set_voltage(user_reg_voltage_t voltage)
{
	/* delay at least 10us before the next sequence */
	switch (voltage) {
	case VOLTAGE_3V3:
		user_reg_write_data(0xDF);
		break;
	case VOLTAGE_3V63:
		user_reg_write_data(0xEF);
		break;
	case VOLTAGE_4V5:
		user_reg_write_data(0xF8);
		break;
	case VOLTAGE_5V125:
		user_reg_write_data(0xFC);
		break;
	default:
		break;
	}
}
#endif /* USER_REGULATOR_ENABLED */

/*******************************************************************************
  * @function   periph_control_io_config
  * @brief      Configuration of new IO pins for Omnia32
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void periph_control_io_config(void)
{
	gpio_init_inputs(pin_pullup, SFP_nDET_PIN);
	gpio_init_outputs(pin_opendrain, pin_spd_2, 0,
			  nRES_MMC_PIN, nRES_LAN_PIN, nRES_PHY_PIN,
			  nPERST0_PIN, nPERST1_PIN, nPERST2_PIN,
			  nVHV_CTRL_PIN, PHY_SFP_PIN);

	gpio_write_multi(1, nVHV_CTRL_PIN, PHY_SFP_PIN);
}

/*******************************************************************************
  * @function   periph_control_rst_init
  * @brief      Set reset init states for peripherals for Omnia32
  * @param      None.
  * @retval     The corresponding initial settings for extended control word.
  *****************************************************************************/
uint16_t periph_control_rst_init(void)
{
	gpio_write_multi(0, nRES_MMC_PIN, nRES_LAN_PIN, nRES_PHY_PIN, nPERST0_PIN,
			 nPERST1_PIN, nPERST2_PIN);

	gpio_write_multi(1, nVHV_CTRL_PIN, PHY_SFP_PIN);

	return EXT_CTL_PHY_SFP | EXT_CTL_nVHV_CTRL;
}
