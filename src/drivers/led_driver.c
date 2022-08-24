/**
 ******************************************************************************
 * @file    led_driver.c
 * @author  CZ.NIC, z.s.p.o.
 * @date    21-July-2015
 * @brief   LED RGB driver for Turris Omnia
 ******************************************************************************
 ******************************************************************************
 **/
#include "led_driver.h"
#include "delay.h"
#include "power_control.h"
#include "gpio.h"
#include "spi.h"
#include "timer.h"
#include "cpu.h"

#define LED_SPI_ALT_FN		0
#define LED_SPI_MOSI_PIN	PIN(A, 7)
#define LED_SPI_SCK_PIN		PIN(A, 5)
#define LED_SPI_SS_PIN		PIN(A, 4)

#define LED_PWM_ALT_FN		0
#define LED_PWM_PIN		PIN(A, 3)

#define COLOR_LEVELS		256
#define FULL_BRIGHTNESS		100
#define EFFECT_TIMEOUT		5

#define LED_PWM_PERIOD		2000
#define LED_PWM_FREQ		8000000

enum {
	RED = 0,
	GREEN = 1,
	BLUE = 2,
};

typedef enum led_effect_states {
	EFFECT_INIT,
	EFFECT_UP,
	EFFECT_DOWN,
	EFFECT_LEDSON,
	EFFECT_DEINIT
} effect_state_t;

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

/* Indicates that initialization effect is done, so that application will can
 * start updating LED states according to PHY / switch / PCIe / mSATA pins.
 */
bool effect_reset_finished;

static uint8_t pwm_brightness;

static bool gamma_correction;

static const uint8_t gamma_table_r[256] = {
	  0,   0,   0,   0,   0,   1,   1,   1,   1,   1,   1,   1,   1,   1,   2,   2,
	  2,   2,   2,   2,   2,   2,   2,   3,   3,   3,   3,   3,   3,   3,   3,   4,
	  4,   4,   4,   4,   4,   5,   5,   5,   5,   5,   6,   6,   6,   6,   6,   7,
	  7,   7,   7,   8,   8,   8,   8,   9,   9,   9,  10,  10,  10,  10,  11,  11,
	 11,  12,  12,  12,  13,  13,  13,  14,  14,  15,  15,  15,  16,  16,  17,  17,
	 17,  18,  18,  19,  19,  20,  20,  21,  21,  22,  22,  23,  23,  24,  24,  25,
	 25,  26,  26,  27,  28,  28,  29,  29,  30,  31,  31,  32,  32,  33,  34,  34,
	 35,  36,  37,  37,  38,  39,  39,  40,  41,  42,  43,  43,  44,  45,  46,  47,
	 47,  48,  49,  50,  51,  52,  53,  54,  54,  55,  56,  57,  58,  59,  60,  61,
	 62,  63,  64,  65,  66,  67,  68,  70,  71,  72,  73,  74,  75,  76,  77,  79,
	 80,  81,  82,  83,  85,  86,  87,  88,  90,  91,  92,  94,  95,  96,  98,  99,
	100, 102, 103, 105, 106, 108, 109, 110, 112, 113, 115, 116, 118, 120, 121, 123,
	124, 126, 128, 129, 131, 132, 134, 136, 138, 139, 141, 143, 145, 146, 148, 150,
	152, 154, 155, 157, 159, 161, 163, 165, 167, 169, 171, 173, 175, 177, 179, 181,
	183, 185, 187, 189, 191, 193, 196, 198, 200, 202, 204, 207, 209, 211, 214, 216,
	218, 220, 223, 225, 228, 230, 232, 235, 237, 240, 242, 245, 247, 250, 252, 255,
};

static const uint8_t gamma_table_gb[256] = {
	  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   1,   1,   1,   1,   1,   1,
	  1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   2,   2,   2,   2,
	  2,   2,   2,   2,   2,   2,   2,   2,   2,   3,   3,   3,   3,   3,   3,   3,
	  3,   3,   3,   4,   4,   4,   4,   4,   4,   4,   4,   5,   5,   5,   5,   5,
	  5,   5,   6,   6,   6,   6,   6,   6,   7,   7,   7,   7,   7,   8,   8,   8,
	  8,   8,   9,   9,   9,   9,   9,  10,  10,  10,  10,  11,  11,  11,  11,  12,
	 12,  12,  12,  13,  13,  13,  14,  14,  14,  14,  15,  15,  15,  16,  16,  16,
	 17,  17,  17,  18,  18,  18,  19,  19,  19,  20,  20,  20,  21,  21,  22,  22,
	 22,  23,  23,  24,  24,  24,  25,  25,  26,  26,  27,  27,  27,  28,  28,  29,
	 29,  30,  30,  31,  31,  32,  32,  33,  33,  34,  34,  35,  35,  36,  36,  37,
	 38,  38,  39,  39,  40,  40,  41,  42,  42,  43,  43,  44,  45,  45,  46,  47,
	 47,  48,  49,  49,  50,  51,  51,  52,  53,  53,  54,  55,  56,  56,  57,  58,
	 58,  59,  60,  61,  62,  62,  63,  64,  65,  66,  66,  67,  68,  69,  70,  71,
	 71,  72,  73,  74,  75,  76,  77,  78,  79,  79,  80,  81,  82,  83,  84,  85,
	 86,  87,  88,  89,  90,  91,  92,  93,  94,  95,  96,  97,  98,  99, 100, 102,
	103, 104, 105, 106, 107, 108, 109, 111, 112, 113, 114, 115, 116, 118, 119, 120,
};

static void _led_set_levels(unsigned led)
{
	uint8_t r, g, b;
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

/*******************************************************************************
  * @function   led_set_color
  * @brief      Set color of LED specified in parameters to be displayed in next cycle.
  * @param      led: position of LED (0..11) or index >=12 -> all LEDs
  * @param      color: LED color (RGB range).
  * @retval     None.
  *****************************************************************************/
void led_set_color(unsigned led, uint32_t color)
{
	uint8_t r, g, b;

	r = color >> 16;
	g = (color >> 8) & 0xFF;
	b = color & 0xFF;

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
	 * something around 160 ticks for Cortex-M0, or 135 ticks for Cortex-M3
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

		if ((level2x & 0xffff) == COLOR_LEVELS)
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

	led_set_color(LED_COUNT, WHITE_COLOR);

	/* Configure SPI and it's pins */
	gpio_init_alts(LED_SPI_ALT_FN, pin_pushpull, pin_spd_3, pin_pulldown,
		       LED_SPI_SCK_PIN, LED_SPI_MOSI_PIN);
	gpio_init_outputs(pin_pushpull, pin_spd_3, 1, LED_SPI_SS_PIN);
	spi_init(LED_SPI);

	/* Configure PWM pin and timer */
	gpio_init_alts(LED_PWM_ALT_FN, pin_pushpull, pin_spd_1, pin_pullup,
		       LED_PWM_PIN);
	timer_init(LED_PWM_TIMER, timer_pwm, LED_PWM_PERIOD, LED_PWM_FREQ, 0);
	timer_enable(LED_PWM_TIMER, true);

	/* Set initial PWM brightness */
	led_driver_set_brightness(FULL_BRIGHTNESS);

	/* Initialize timer (every tick we send one frame) */
	timer_init(LED_TIMER, timer_interrupt, 50, 2400000, 4);
	timer_enable(LED_TIMER, true);

	/* Configure boot effect */
	timer_init(LED_EFFECT_TIMER, timer_interrupt, 8000, 120000, 5);
}

/*******************************************************************************
  * @function   led_driver_set_brightness
  * @brief      Set PWM value.
  * @param      procent_val: PWM value in [%].
  * @retval     None.
  *****************************************************************************/
void led_driver_set_brightness(uint8_t procent_val)
{
	uint16_t counter_val;

	if (procent_val > FULL_BRIGHTNESS)
		procent_val = FULL_BRIGHTNESS;

	counter_val = procent_val * LED_PWM_PERIOD / FULL_BRIGHTNESS;

	timer_set_pulse(LED_PWM_TIMER, counter_val);
	pwm_brightness = procent_val;
}

/*******************************************************************************
  * @function   led_driver_get_brightness
  * @brief      Set PWM value.
  * @param      None.
  * @retval     procent_val: PWM value in [%].
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
	static const uint8_t brightnesses[] = { 100, 70, 40, 25, 12, 5, 1, 0 };
	static uint8_t step = 1;

	pwm_brightness = brightnesses[step++];
	led_driver_set_brightness(pwm_brightness);

	if (step >= ARRAY_SIZE(brightnesses))
		step = 0;
}

static inline uint16_t led_bits(unsigned led)
{
	if (led >= LED_COUNT)
		return LED_BITS_ALL;
	else
		return LED_BIT(led);
}

static void recompute_led_states(void)
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

	recompute_led_states();
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
	if (state)
		leds_states_default |= led_bits(led);
	else
		leds_states_default &= ~led_bits(led);

	recompute_led_states();
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

	recompute_led_states();
}

/*******************************************************************************
  * @function   led_driver_reset_effect
  * @brief      Enable/Disable knight rider effect after reset.
  * @param      color: color in RGB range.
  * @retval     None.
  *****************************************************************************/
void led_driver_reset_effect(bool state)
{
	timer_enable(LED_EFFECT_TIMER, state);
	if (!state)
		timer_set_counter(LED_EFFECT_TIMER, 0);
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
	static int8_t led;
	static uint8_t state_timeout_cnt;
	static effect_state_t effect_state; /* states for LED effect after reset */

	switch (effect_state)
	{
		case EFFECT_INIT:
		{
			effect_reset_finished = false;
			led_set_user_mode(LED_COUNT, false);
			led_set_state(LED_COUNT, false);
			led_set_color(LED_COUNT, WHITE_COLOR);
			led_set_state(0, true);
			effect_state = EFFECT_UP;
		} break;

		case EFFECT_UP:
		{
			led++;
			led_set_state(11, false);
			led_set_state(led - 1, false);
			led_set_state(led, true);

			if (led >= 11)
			{
				effect_state = EFFECT_DOWN; /* next state */
			}
			else
			{
				effect_state = EFFECT_UP;
			}
		} break;

		case EFFECT_DOWN:
		{
			led--;
			led_set_state(led + 1, false);
			led_set_state(led, true);

			if (led <= 0)
			{
				effect_state = EFFECT_LEDSON; /* next state */
			}
			else
			{
				effect_state = EFFECT_DOWN;
			}
		} break;

		case EFFECT_LEDSON:
		{
			led_set_state(LED_COUNT, true);
			led_set_color(LED_COUNT, GREEN_COLOR | BLUE_COLOR);
			effect_state = EFFECT_DEINIT;
		} break;

		case EFFECT_DEINIT:
		{
			state_timeout_cnt++;

			if (state_timeout_cnt >= EFFECT_TIMEOUT)
			{
				led_set_state(LED_COUNT, false);
				led_set_color(LED_COUNT, WHITE_COLOR);

				led_set_user_mode(LED_COUNT, false);
				led_driver_reset_effect(DISABLE);
				state_timeout_cnt = 0;
				effect_reset_finished = true;
				effect_state = EFFECT_INIT;
			}
			else
			{
				effect_state = EFFECT_DEINIT;
			}

		} break;
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
	static uint8_t timeout;
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
