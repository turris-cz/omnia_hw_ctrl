#include "led_driver.h"
#include "power_control.h"
#include "pin_defs.h"
#include "spi.h"
#include "timer.h"
#include "cpu.h"

#define EFFECT_TIMEOUT		5

#define LED_PWM_FREQ		4000000

/* Without gamma correction we are operating at 256 color channel levels.
 * The timer interrupt is generated with frequency 960,000 Hz / 20 = 48,000 Hz,
 * and there are three color channels, so one entire color level frame is sent
 * with frequency 48,000 Hz / 3 = 16,000 Hz.
 *
 * With 256 color levels we are sending 255 color level frames [1], thus the
 * different intensity levels are generated by software PWM, which blinks at
 * frequency 16,000 Hz / 255 =~ 62.7 Hz.
 *
 * With gamma correction we have different color levels and so are working at
 * different frequencies to achieve the blinking at approximately 60 Hz.
 * Gamma correction parameters are platform specific, defined later in this
 * file.
 *
 * [1] Why are we sending a sequence of 255 frames for 256 different levels?
 *     Consider if the levels were only 4, then we would need to send only
 *     sequences of length 3:
 *                sequence
 *       level 0: 0, 0, 0
 *       level 1: 1, 0, 0
 *       level 2: 1, 1, 0
 *       level 3: 1, 1, 1
 */
#define LED_TIMER_FREQ		960000
#define LED_TIMER_PERIOD	20
#define COLOR_LEVELS		256

enum {
	RED = 0,
	GREEN = 1,
	BLUE = 2,
};

typedef struct {
	uint8_t r, g, b;
} rgb_t;

struct led {
	rgb_t color;
};

#define LED_BITS_ALL	GENMASK(13, 2)
#define LED_BIT(led)	BIT((led) + 2)

/* the following bitmasks are shifted by 2 because of how LEDs are
 * connected in hardware */
static uint16_t leds_modes_user,     /* bitmask of LEDs in user mode */
		leds_states_default, /* bitmask of LED states for default mode */
		leds_states_user,    /* bitmask of LED states for user mode */
		leds_states;         /* bitmask of actual LED states */

/* LED color levels for faster computation in LED timer handler */
static union {
	uint16_t value[12];
	uint32_t for_cmp[6];
} led_levels[3];

static struct led leds[LED_COUNT];

/* Indicates the state of initialization effect, so that application will know
 * when to start updating LED states according to PHY / switch / PCIe / mSATA
 * LED pins.
 */
reset_effect_state_t reset_effect_state;

static uint8_t pwm_brightness;

static bool gamma_correction;
static uint16_t color_levels;

#if defined(STM32F030X8) && SYS_CORE_FREQ == 48000000U
/* On STM32 (Cortex-M0 at 48 MHz) we have 640 color levels with gamma correction
 * enabled.
 *
 * Since 640 = 256 * 2.5, the frames are sent 2.5 times faster, every 400 clock
 * cycles (compare with every 1000 clock cycles with gamma correction disabled).
 *
 * On average, the interrupt takes around 163 clock cycles, so this should be
 * fine.
 */
#define LED_TIMER_PERIOD_GC	8
#define COLOR_LEVELS_GC		640

static const uint16_t gamma_table_r[256] = {
	  0,   1,   1,   1,   1,   1,   2,   2,   2,   2,   3,   3,   3,   4,   4,   4,
	  4,   5,   5,   5,   6,   6,   6,   6,   7,   7,   7,   8,   8,   8,   9,   9,
	 10,  10,  10,  11,  11,  12,  12,  13,  13,  14,  14,  15,  15,  16,  16,  17,
	 17,  18,  18,  19,  20,  20,  21,  22,  22,  23,  24,  25,  25,  26,  27,  28,
	 28,  29,  30,  31,  32,  33,  34,  35,  35,  36,  37,  38,  39,  40,  41,  42,
	 44,  45,  46,  47,  48,  49,  50,  52,  53,  54,  55,  57,  58,  59,  60,  62,
	 63,  65,  66,  67,  69,  70,  72,  73,  75,  77,  78,  80,  81,  83,  85,  86,
	 88,  90,  92,  93,  95,  97,  99, 101, 103, 105, 107, 109, 111, 113, 115, 117,
	119, 121, 123, 125, 127, 130, 132, 134, 136, 139, 141, 144, 146, 148, 151, 153,
	156, 158, 161, 164, 166, 169, 171, 174, 177, 180, 183, 185, 188, 191, 194, 197,
	200, 203, 206, 209, 212, 215, 218, 222, 225, 228, 231, 235, 238, 241, 245, 248,
	252, 255, 259, 262, 266, 269, 273, 277, 280, 284, 288, 292, 296, 300, 304, 308,
	311, 316, 320, 324, 328, 332, 336, 340, 345, 349, 353, 358, 362, 367, 371, 376,
	380, 385, 389, 394, 399, 404, 408, 413, 418, 423, 428, 433, 438, 443, 448, 453,
	458, 464, 469, 474, 480, 485, 490, 496, 501, 507, 512, 518, 524, 529, 535, 541,
	547, 552, 558, 564, 570, 576, 582, 589, 595, 601, 607, 613, 620, 626, 633, 639,
};

static const uint16_t gamma_table_gb[256] = {
	  0,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   2,   2,   2,   2,
	  2,   2,   2,   2,   3,   3,   3,   3,   3,   3,   3,   4,   4,   4,   4,   4,
	  4,   5,   5,   5,   5,   5,   6,   6,   6,   6,   7,   7,   7,   7,   8,   8,
	  8,   8,   9,   9,   9,  10,  10,  10,  11,  11,  11,  12,  12,  12,  13,  13,
	 13,  14,  14,  15,  15,  15,  16,  16,  17,  17,  18,  18,  18,  19,  19,  20,
	 20,  21,  21,  22,  23,  23,  24,  24,  25,  25,  26,  27,  27,  28,  28,  29,
	 30,  30,  31,  32,  32,  33,  34,  34,  35,  36,  37,  37,  38,  39,  40,  41,
	 41,  42,  43,  44,  45,  46,  46,  47,  48,  49,  50,  51,  52,  53,  54,  55,
	 56,  57,  58,  59,  60,  61,  62,  63,  64,  65,  66,  67,  69,  70,  71,  72,
	 73,  74,  76,  77,  78,  79,  81,  82,  83,  84,  86,  87,  88,  90,  91,  92,
	 94,  95,  97,  98, 100, 101, 103, 104, 106, 107, 109, 110, 112, 113, 115, 116,
	118, 120, 121, 123, 125, 126, 128, 130, 132, 133, 135, 137, 139, 141, 143, 144,
	146, 148, 150, 152, 154, 156, 158, 160, 162, 164, 166, 168, 170, 172, 174, 176,
	179, 181, 183, 185, 187, 189, 192, 194, 196, 199, 201, 203, 206, 208, 210, 213,
	215, 218, 220, 223, 225, 228, 230, 233, 235, 238, 241, 243, 246, 248, 251, 254,
	257, 259, 262, 265, 268, 271, 273, 276, 279, 282, 285, 288, 291, 294, 297, 300,
};
#elif defined(GD32F1x0) && SYS_CORE_FREQ == 72000000U
/* On GD32 (Cortex-M3 at 72 MHz) we have 1024 color levels with gamma correction
 * enabled.
 *
 * Since 1024 = 256 * 4, the frames are sent 4 times faster, every 375 clock
 * cycles (compare with every 1500 clock cycles with gamma correction disabled).
 *
 * On average, the interrupt takes around 138 clock cycles, so this should be
 * fine.
 */
#define LED_TIMER_PERIOD_GC	5
#define COLOR_LEVELS_GC		1024

static const uint16_t gamma_table_r[256] = {
	  0,   1,   1,   1,   2,   2,   3,   3,   4,   4,   4,   5,   5,    6,    6,    7,
	  7,   8,   8,   8,   9,   9,  10,  10,  11,  11,  12,  12,  13,   13,   14,   15,
	 15,  16,  17,  17,  18,  19,  19,  20,  21,  22,  22,  23,  24,   25,   26,   27,
	 28,  29,  30,  31,  32,  33,  34,  35,  36,  37,  38,  39,  40,   42,   43,   44,
	 45,  47,  48,  50,  51,  52,  54,  55,  57,  58,  60,  61,  63,   65,   66,   68,
	 70,  71,  73,  75,  77,  79,  81,  83,  84,  86,  88,  90,  93,   95,   97,   99,
	101, 103, 106, 108, 110, 113, 115, 118, 120, 123, 125, 128, 130,  133,  136,  138,
	141, 144, 147, 149, 152, 155, 158, 161, 164, 167, 171, 174, 177,  180,  183,  187,
	190, 194, 197, 200, 204, 208, 211, 215, 218, 222, 226, 230, 234,  237,  241,  245,
	249, 254, 258, 262, 266, 270, 275, 279, 283, 288, 292, 297, 301,  306,  311,  315,
	320, 325, 330, 335, 340, 345, 350, 355, 360, 365, 370, 376, 381,  386,  392,  397,
	403, 408, 414, 420, 425, 431, 437, 443, 449, 455, 461, 467, 473,  480,  486,  492,
	499, 505, 512, 518, 525, 532, 538, 545, 552, 559, 566, 573, 580,  587,  594,  601,
	609, 616, 624, 631, 639, 646, 654, 662, 669, 677, 685, 693, 701,  709,  717,  726,
	734, 742, 751, 759, 768, 776, 785, 794, 802, 811, 820, 829, 838,  847,  857,  866,
	875, 885, 894, 903, 913, 923, 932, 942, 952, 962, 972, 982, 992, 1002, 1013, 1023,
};

static const uint16_t gamma_table_gb[256] = {
	  0,   0,   0,   1,   1,   1,   1,   1,   2,   2,   2,   2,   3,   3,   3,   3,
	  3,   4,   4,   4,   4,   4,   5,   5,   5,   5,   6,   6,   6,   6,   7,   7,
	  7,   7,   8,   8,   8,   9,   9,   9,  10,  10,  11,  11,  11,  12,  12,  13,
	 13,  13,  14,  14,  15,  15,  16,  16,  17,  17,  18,  18,  19,  20,  20,  21,
	 21,  22,  23,  23,  24,  25,  25,  26,  27,  27,  28,  29,  30,  30,  31,  32,
	 33,  34,  34,  35,  36,  37,  38,  39,  40,  41,  42,  43,  44,  45,  46,  47,
	 48,  49,  50,  51,  52,  53,  54,  55,  56,  58,  59,  60,  61,  62,  64,  65,
	 66,  68,  69,  70,  72,  73,  74,  76,  77,  79,  80,  82,  83,  85,  86,  88,
	 89,  91,  93,  94,  96,  98,  99, 101, 103, 104, 106, 108, 110, 112, 114, 115,
	117, 119, 121, 123, 125, 127, 129, 131, 133, 135, 137, 140, 142, 144, 146, 148,
	150, 153, 155, 157, 160, 162, 164, 167, 169, 172, 174, 177, 179, 182, 184, 187,
	189, 192, 195, 197, 200, 203, 206, 208, 211, 214, 217, 220, 223, 226, 228, 231,
	234, 238, 241, 244, 247, 250, 253, 256, 259, 263, 266, 269, 273, 276, 279, 283,
	286, 290, 293, 297, 300, 304, 307, 311, 315, 318, 322, 326, 330, 333, 337, 341,
	345, 349, 353, 357, 361, 365, 369, 373, 377, 381, 386, 390, 394, 398, 403, 407,
	411, 416, 420, 425, 429, 434, 438, 443, 448, 452, 457, 462, 467, 471, 476, 481,
};
#else
#error "LED driver gamma correction parameters not implemented for this platform"
#endif

static void _led_set_levels(unsigned led)
{
	uint16_t r, g, b;
	unsigned idx;

	r = leds[led].color.r;
	g = leds[led].color.g;
	b = leds[led].color.b;

	if (!BOOTLOADER_BUILD && gamma_correction) {
		r = gamma_table_r[r];
		g = gamma_table_gb[g];
		b = gamma_table_gb[b];
	}

	/* this is due to how we compute corresponding bits in
	 * led_driver_prepare_data(). We do this rearrangement here so that
	 * we save time there. */
	idx = (led << 1) - ((led > 5) ? 11 : 0);

	/* always set bit 15 for fast comparison (two values at once) in
	 * led_driver_prepare_data() */
	led_levels[0].value[idx] = BIT(15) | r;
	led_levels[1].value[idx] = BIT(15) | g;
	led_levels[2].value[idx] = BIT(15) | b;
}

static void _led_set_color(unsigned led, uint8_t r, uint8_t g, uint8_t b)
{
	leds[led].color.r = r;
	leds[led].color.g = g;
	leds[led].color.b = b;

	_led_set_levels(led);
}

void led_set_color(unsigned led, uint8_t r, uint8_t g, uint8_t b)
{
	if (led >= LED_COUNT) {
		for (int idx = 0; idx < LED_COUNT; ++idx)
			_led_set_color(idx, r, g, b);
	} else {
		_led_set_color(led, r, g, b);
	}
}

void led_driver_set_gamma_correction(bool on)
{
	if (gamma_correction == on)
		return;

	gamma_correction = on;

	if (on) {
		color_levels = COLOR_LEVELS_GC;
		timer_set_period(LED_TIMER, LED_TIMER_PERIOD_GC);
	} else {
		color_levels = COLOR_LEVELS;
		timer_set_period(LED_TIMER, LED_TIMER_PERIOD);
	}

	for (int led = 0; led < LED_COUNT; ++led)
		_led_set_levels(led);
}

bool led_driver_get_gamma_correction(void)
{
	return gamma_correction;
}

/*******************************************************************************
  * @function   led_driver_prepare_data
  * @brief      Prepare data to be sent to LED driver.
  * @param      channel: RED, GREEN or BLUE.
  * @retval     Data to be sent to LED driver.
  *****************************************************************************/
static uint16_t led_driver_prepare_data(unsigned channel)
{
	/* Two values are compared with current level at once: the two values
	 * are stored in one uint32_t (values[N]), and current level is repeated
	 * two times in another uint32_t (level2x). Both values always have MSB
	 * set, and so subtracting current level unsets the corresponding bit if
	 * level is greater than the value.
	 *
	 * This saves some precious processor ticks. GCC with -Os should compile
	 * the interrupt handler (together with exception entry and exit) to
	 * something around 163 ticks for Cortex-M0, or 138 ticks for Cortex-M3
	 * (on average).
	 */
	static const uint32_t mask = BIT(31) | BIT(15),
			      one = 0x10001;
	static uint32_t level2x = one;
	const uint32_t *values;
	uint32_t res;

	values = led_levels[channel].for_cmp;

	res = ((values[0] - level2x) & mask) >> 1;
	res = (res | ((values[1] - level2x) & mask)) >> 1;
	res = (res | ((values[2] - level2x) & mask)) >> 1;
	res = (res | ((values[3] - level2x) & mask)) >> 1;
	res = (res | ((values[4] - level2x) & mask)) >> 1;
	res = (res | ((values[5] - level2x) & mask));

	res = (res >> 18) | ((res & 0xffffU) >> 8);

	if (channel == BLUE) {
		level2x += one;

		if ((level2x & 0xffff) >= color_levels)
			level2x = one;
	}

	return res & leds_states;
}

/*******************************************************************************
  * @function   led_driver_send_frame
  * @brief      Send frame to LED driver. It is called in LED_TIMER interrupt.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
static void led_driver_send_frame(void)
{
	static uint8_t channel = RED;
	uint16_t data;

	/* toggle SS before sending red channel */
	if (channel == RED) {
		gpio_write(LED_SPI_SS_PIN, 1);
		nop();
		gpio_write(LED_SPI_SS_PIN, 0);
	}

	data = led_driver_prepare_data(channel);
	spi_send16(LED_SPI, data);

	if (channel == BLUE)
		channel = RED;
	else
		channel++;
}

void __irq led_driver_irq_handler(void)
{
	if (unlikely(!timer_irq_clear_up(LED_TIMER)))
		return;

	led_driver_send_frame();
}

/*******************************************************************************
  * @function   led_driver_config
  * @brief      Configure LED driver.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void led_driver_config(void)
{
	/* Set initial mode, state and color */
	leds_modes_user = 0;
	leds_states_default = 0;
	leds_states_user = LED_BITS_ALL;
	leds_states = 0;
	gamma_correction = !BOOTLOADER_BUILD && OMNIA_BOARD_REVISION >= 32;
	color_levels = gamma_correction ? COLOR_LEVELS_GC : COLOR_LEVELS;

	led_set_color24(LED_COUNT, WHITE_COLOR);

	/* Configure SPI and it's pins */
	gpio_init_alts(LED_SPI_ALT_FN, pin_pushpull, pin_spd_3, pin_pulldown,
		       LED_SPI_SCK_PIN, LED_SPI_MOSI_PIN);
	gpio_init_outputs(pin_pushpull, pin_spd_3, 1, LED_SPI_SS_PIN);
	spi_init(LED_SPI);

	/* Initialize timer (every tick we send one frame) */
	timer_init(LED_TIMER, timer_interrupt,
		   gamma_correction ? LED_TIMER_PERIOD_GC : LED_TIMER_PERIOD,
		   LED_TIMER_FREQ, 0);

	/* Configure PWM timer */
	timer_init(LED_PWM_TIMER, timer_pwm, 100, LED_PWM_FREQ, 0);

	/* Set initial PWM brightness (also enables LED_TIMER) */
	led_driver_set_brightness(100);

	/* Configure boot effect */
	timer_init(LED_EFFECT_TIMER, timer_interrupt, 8000, 120000, 2);
}

/*******************************************************************************
  * @function   led_driver_set_brightness
  * @brief      Set PWM brightness.
  * @param      value: PWM value in [%].
  * @retval     None.
  *****************************************************************************/
void led_driver_set_brightness(uint8_t value)
{
	disable_irq();

	if (value > 100)
		value = 100;

	/* don't allow zero brightness in bootloader */
	if (BOOTLOADER_BUILD && value == 0)
		value = 1;

	if (value == 0 || value == 100) {
		timer_enable(LED_PWM_TIMER, false);
		gpio_init_outputs(pin_pushpull, pin_spd_1, !value, LED_PWM_PIN);
	} else {
		gpio_init_alts(LED_PWM_ALT_FN, pin_pushpull, pin_spd_1,
			       pin_pullup, LED_PWM_PIN);
		timer_enable(LED_PWM_TIMER, true);
	}

	timer_enable(LED_TIMER, !!value);

	timer_set_pulse(LED_PWM_TIMER, value);
	pwm_brightness = value;

	enable_irq();
}

/*******************************************************************************
  * @function   led_driver_get_brightness
  * @brief      Get PWM brightness.
  * @param      None.
  * @retval     PWM value in [%].
  *****************************************************************************/
uint8_t led_driver_get_brightness(void)
{
	return pwm_brightness;
}

/*******************************************************************************
  * @function   led_driver_step_brightness
  * @brief      Decrease LED brightness by step (each function call).
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void led_driver_step_brightness(void)
{
	static const uint8_t brightnesses[] = {
#if BOOTLOADER_BUILD
		/* don't allow zero brightness in bootloader */
		100, 70, 40, 25, 12, 5, 1,
#else
		100, 70, 40, 25, 12, 5, 1, 0,
#endif
	};
	uint8_t i;

	for (i = 0; i < ARRAY_SIZE(brightnesses); ++i)
		if (brightnesses[i] < pwm_brightness)
			break;

	if (i == ARRAY_SIZE(brightnesses))
		i = 0;

	pwm_brightness = brightnesses[i];
	led_driver_set_brightness(pwm_brightness);
}

static inline uint16_t led_bits(unsigned led)
{
	if (led >= LED_COUNT)
		return LED_BITS_ALL;
	else
		return LED_BIT(led);
}

void led_states_commit(void)
{
	leds_states = (leds_modes_user & leds_states_user) |
		      (~leds_modes_user & leds_states_default);
}

/*******************************************************************************
  * @function   led_set_user_mode
  * @brief      Set mode to LED(s) - default or user mode
  * @param      led: position of LED (0..11) or led >=12 -> all LED.
  * @parame     set: true to set user mode, false to unset
  * @retval     None.
  *****************************************************************************/
void led_set_user_mode(unsigned led, bool set)
{
	if (set)
		leds_modes_user |= led_bits(led);
	else
		leds_modes_user &= ~led_bits(led);

	led_states_commit();
}

void led_set_state_nocommit(unsigned led, bool state)
{
	if (state)
		leds_states_default |= led_bits(led);
	else
		leds_states_default &= ~led_bits(led);
}

/*******************************************************************************
  * @function   led_set_state
  * @brief      Set state of the LED(s)
  * @param      led: position of LED (0..11) or led >=12 -> all LED.
  * @parame     state: false / true
  * @retval     None.
  *****************************************************************************/
void led_set_state(unsigned led, bool state)
{
	led_set_state_nocommit(led, state);
	led_states_commit();
}

/*******************************************************************************
  * @function   led_set_state_user
  * @brief      Set state of the LED(s)i from user/I2C
  * @param      led: position of LED (0..11) or led >=12 -> all LED.
  * @parame     state: false / true
  * @retval     None.
  *****************************************************************************/
void led_set_state_user(unsigned led, bool state)
{
	if (state)
		leds_states_user |= led_bits(led);
	else
		leds_states_user &= ~led_bits(led);

	led_states_commit();
}

/*******************************************************************************
  * @function   led_driver_reset_effect
  * @brief      Enable/Disable knight rider effect after reset.
  * @param      color: color in RGB range.
  * @retval     None.
  *****************************************************************************/
void led_driver_reset_effect(bool state)
{
	if (state)
		reset_effect_state = EFFECT_INIT;

	timer_enable(LED_EFFECT_TIMER, state);
	if (!state)
		timer_set_counter(LED_EFFECT_TIMER, 0);

	if (!state)
		reset_effect_state = EFFECT_DISABLED;
}

/*******************************************************************************
  * @function   led_driver_knight_rider_effect_handler
  * @brief      Display knight rider effect on LEDs during startup (called in
  *             timer interrupt).
  * @param      None.
  * @retval     None.
  *****************************************************************************/
static void led_driver_knight_rider_effect_handler(void)
{
	static unsigned led, effect_timeout_cnt;

	switch (reset_effect_state) {
	case EFFECT_DISABLED:
		/* nothing to do */
		break;

	case EFFECT_INIT:
		led_set_user_mode(LED_COUNT, false);
		led_set_state(LED_COUNT, false);
		led_set_color24(LED_COUNT, WHITE_COLOR);
		led_set_state(0, true);
		led = 0;
		effect_timeout_cnt = 0;
		reset_effect_state = EFFECT_UP;
		break;

	case EFFECT_UP:
		led++;
		led_set_state(LED_COUNT - 1, false);
		led_set_state(led - 1, false);
		led_set_state(led, true);

		if (led >= LED_COUNT - 1)
			reset_effect_state = EFFECT_DOWN;
		break;

	case EFFECT_DOWN:
		led--;
		led_set_state(led + 1, false);
		led_set_state(led, true);

		if (!led)
			reset_effect_state = EFFECT_LEDSON;
		break;

	case EFFECT_LEDSON:
		led_set_state(LED_COUNT, true);
		led_set_color24(LED_COUNT, GREEN_COLOR | BLUE_COLOR);
		reset_effect_state = EFFECT_DEINIT;
		break;

	case EFFECT_DEINIT:
		effect_timeout_cnt++;

		if (effect_timeout_cnt >= EFFECT_TIMEOUT) {
			led_set_state(LED_COUNT, false);
			led_set_color24(LED_COUNT, WHITE_COLOR);
			led_set_state(POWER_LED, true);

			led_set_user_mode(LED_COUNT, false);
			led_driver_reset_effect(false);
		}
		break;
	}
}

/*******************************************************************************
  * @function   led_driver_bootloader_effect_handler
  * @brief      Display bootloader effect.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
static void led_driver_bootloader_effect_handler(void)
{
	static unsigned timeout;
	static bool on;

	timeout++;

	if (timeout >= 8) {
		timeout = 0;

		on = !on;
		led_set_state(LED_COUNT, on);
	}
}

void __irq led_driver_effect_irq_handler(void)
{
	if (!timer_irq_clear_up(LED_EFFECT_TIMER))
		return;

	if (BOOTLOADER_BUILD)
		led_driver_bootloader_effect_handler();
	else
		led_driver_knight_rider_effect_handler();
}
