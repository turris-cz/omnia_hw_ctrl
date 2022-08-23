/**
 ******************************************************************************
 * @file    led_driver.c
 * @author  CZ.NIC, z.s.p.o.
 * @date    21-July-2015
 * @brief   LED RGB driver for Turris Omnia
 ******************************************************************************
 ******************************************************************************
 **/
/* Includes ------------------------------------------------------------------*/
#include "led_driver.h"
#include "delay.h"
#include "power_control.h"
#include "gpio.h"
#include "spi.h"
#include "timer.h"
#include "cpu.h"

/* Private define ------------------------------------------------------------*/
#define LED_SPI_ALT_FN		0
#define LED_SPI_MOSI_PIN	PIN(A, 7)
#define LED_SPI_SCK_PIN		PIN(A, 5)
#define LED_SPI_SS_PIN		PIN(A, 4)

#define LED_PWM_ALT_FN		0
#define LED_PWM_PIN		PIN(A, 3)

#define COLOR_LEVELS                64
#define COLOR_DECIMATION            2
#define MAX_LED_BRIGHTNESS          100
#define EFFECT_TIMEOUT              5

#define LED_PWM_PERIOD		2000
#define LED_PWM_FREQ		8000000

/* Private typedef -----------------------------------------------------------*/
typedef enum rgb_color {
    RED     = 0,
    GREEN   = 1,
    BLUE    = 2,
    WHITE   = -1,
}rgb_color_t;

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
		leds_states_user;    /* bitmask of LED states for user mode */

static struct led leds[LED_COUNT];

uint8_t effect_reset_finished; /* flag is set when LED effect after reset is
finished and normal operation can take the LED control */

static uint8_t pwm_brightness;

static void _led_set_color(unsigned led, uint8_t r, uint8_t g, uint8_t b)
{
	leds[led].color.r = r;
	leds[led].color.g = g;
	leds[led].color.b = b;
}

/*******************************************************************************
  * @function   led_set_color
  * @brief      Set color of LED specified in parameters to be displayed in next cycle.
  * @param      led_index: position of LED (0..11) or index >=12 -> all LEDs
  * @param      color: LED color (RGB range).
  * @retval     None.
  *****************************************************************************/
void led_set_color(const uint8_t led_index, const uint32_t color)
{
	uint8_t r, g, b;

	r = color >> 16;
	g = (color >> 8) & 0xFF;
	b = color & 0xFF;

	if (led_index >= LED_COUNT) {
		for (int idx = 0; idx < LED_COUNT; ++idx)
			_led_set_color(idx, r, g, b);
	} else {
		_led_set_color(led_index, r, g, b);
	}
}

/*******************************************************************************
  * @function   led_driver_prepare_data
  * @brief      Prepare data to be sent to LED driver.
  * @param      color: RED, GREEN or BLUE.
  * @param      current_color_level: color density [0..255]
  * @retval     Data to be sent to LED driver.
  *****************************************************************************/
static uint16_t led_driver_prepare_data(const rgb_color_t color, const uint8_t current_color_level)
{
    uint16_t data = 0;
    uint8_t idx;
    struct led *rgb_leds = leds;

    switch (color)
    {
        case RED:
        {
            for (idx = 0; idx < LED_COUNT; idx++, rgb_leds++)
            {
                if (!(leds_modes_user & LED_BIT(idx)))
                {
                    if (leds_states_default & LED_BIT(idx))
                    {
                        if (rgb_leds->color.r > current_color_level)
                        {
                            data |= 1 << (2 + idx); /* shift by 2 - due to the HW connection */
                        }
                    }
                }
                else /* LED_USER_MODE has the same color profile as default mode now */
                {
                    if (leds_states_user & LED_BIT(idx))
                    {
                        if (rgb_leds->color.r > current_color_level)
                        {
                            data |= 1 << (2 + idx);
                        }
                    }
                }
            }
        } break;

        case GREEN:
        {
            for (idx = 0; idx < LED_COUNT; idx++, rgb_leds++)
            {
                if (!(leds_modes_user & LED_BIT(idx)))
                {
                    if (leds_states_default & LED_BIT(idx))
                    {
                        if (rgb_leds->color.g > current_color_level)
                        {
                            data |= 1 << (2 + idx);
                        }
                    }
                }
                else /* LED_USER_MODE has the same color profile as default mode now */
                {
                    if (leds_states_user & LED_BIT(idx))
                    {
                        if (rgb_leds->color.g > current_color_level)
                        {
                            data |= 1 << (2 + idx);
                        }
                    }
                }
            }
        } break;

        case BLUE:
        {
            for (idx = 0; idx < LED_COUNT; idx++, rgb_leds++)
            {
                if (!(leds_modes_user & LED_BIT(idx)))
                {
                    if (leds_states_default & LED_BIT(idx))
                    {
                        if (rgb_leds->color.b > current_color_level)
                        {
                            data |= 1 << (2 + idx);
                        }
                    }
                }
                else /* LED_USER_MODE has the same color profile as default mode now */
                {
                    if (leds_states_user & LED_BIT(idx))
                    {
                        if (rgb_leds->color.b > current_color_level)
                        {
                            data |= 1 << (2 + idx);
                        }
                    }
                }
            }
        } break;

        default:
            break;
    }

    return data;
}

/*******************************************************************************
  * @function   led_driver_send_frame
  * @brief      Send frame to LED driver. It is called in LED_TIMER interrupt.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
static void led_driver_send_frame(void)
{
    static uint8_t level, channel = RED;
    uint16_t data;

    /* toggle SS before sending red channel */
    if (channel == RED) {
        gpio_write(LED_SPI_SS_PIN, 1);
        nop();
        gpio_write(LED_SPI_SS_PIN, 0);
    }

    /* decrease 255 color levels to COLOR_LEVELS by shift (COLOR_DECIMATION) */
    data = led_driver_prepare_data(channel, level << COLOR_DECIMATION);
    spi_send16(LED_SPI, data);

    if (channel == BLUE) {
        channel = RED;

        level++;

        /* restart cycle when all levels sent */
        if (level >= COLOR_LEVELS)
            level = 0;
    } else {
        channel++;
    }
}

void __irq led_driver_irq_handler(void)
{
	if (!timer_irq_clear_up(LED_TIMER))
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
	timer_enable(LED_PWM_TIMER, 1);

	/* Set initial PWM brightness */
	led_driver_set_brightness(MAX_LED_BRIGHTNESS);

	/* Initialize timer (every tick we send one frame) */
	timer_init(LED_TIMER, timer_interrupt, 200, 2400000, 4);
	timer_enable(LED_TIMER, 1);

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

    if (procent_val > MAX_LED_BRIGHTNESS)
        procent_val = MAX_LED_BRIGHTNESS;

    counter_val = procent_val * LED_PWM_PERIOD / MAX_LED_BRIGHTNESS;

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

/*******************************************************************************
  * @function   led_set_user_mode
  * @brief      Set mode to LED(s) - default or user mode
  * @param      led_index: position of LED (0..11) or led_index >=12 -> all LED.
  * @parame     set: true to set user mode, false to unset
  * @retval     None.
  *****************************************************************************/
void led_set_user_mode(const uint8_t led_index, const bool set)
{
	if (set)
		leds_modes_user |= led_bits(led_index);
	else
		leds_modes_user &= ~led_bits(led_index);
}

/*******************************************************************************
  * @function   led_set_state
  * @brief      Set state of the LED(s)
  * @param      led_index: position of LED (0..11) or led_index >=12 -> all LED.
  * @parame     state: false / true
  * @retval     None.
  *****************************************************************************/
void led_set_state(const uint8_t led_index, const bool state)
{
	if (state)
		leds_states_default |= led_bits(led_index);
	else
		leds_states_default &= ~led_bits(led_index);
}

/*******************************************************************************
  * @function   led_set_state_user
  * @brief      Set state of the LED(s)i from user/I2C
  * @param      led_index: position of LED (0..11) or led_index >=12 -> all LED.
  * @parame     state: false / true
  * @retval     None.
  *****************************************************************************/
void led_set_state_user(const uint8_t led_index, const bool state)
{
	if (state)
		leds_states_user |= led_bits(led_index);
	else
		leds_states_user &= ~led_bits(led_index);
}

/*******************************************************************************
  * @function   led_driver_knight_rider_effect
  * @brief      Display knight rider effect on LEDs.
  * @param      color: color in RGB range.
  * @retval     None.
  *****************************************************************************/
void led_driver_knight_rider_effect(uint32_t color)
{
    int8_t led;

    led_set_state(LED_COUNT, false);
    led_set_color(LED_COUNT, color);
    led_set_state(0, true);

    for (led = 1; led < LED_COUNT; led++)
    {
        delay(70);
        led_set_state(led - 1, false);
        led_set_state(led, true);
    }

    for (led = 10; led > -1 ; led--)
    {
        delay(70);
        led_set_state(led + 1, false);
        led_set_state(led, true);
    }

    led_set_color(LED_COUNT, WHITE_COLOR);
    led_set_state(LED_COUNT, false);
}

/*******************************************************************************
  * @function   led_driver_knight_rider_color_effect
  * @brief      Display knight rider effect on LEDs.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void led_driver_knight_rider_color_effect(void)
{
    int8_t color;

    led_set_state(LED_COUNT, false);

    for (color = WHITE; color < BLUE + 1; color++)
    {
        switch (color)
        {
        case RED: led_driver_knight_rider_effect(RED_COLOR); break;
        case GREEN: led_driver_knight_rider_effect(GREEN_COLOR); break;
        case BLUE: led_driver_knight_rider_effect(BLUE_COLOR); break;
        default: led_driver_knight_rider_effect(WHITE_COLOR); break;
        }
    }
}


/*******************************************************************************
  * @function   led_driver_double_knight_rider_effect
  * @brief      Display double knight rider effect on LEDs.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void led_driver_double_knight_rider_effect(void)
{
    int8_t led_down, led_up;
    int8_t color;

    led_set_state(LED_COUNT, false);
    led_set_color(LED_COUNT, WHITE_COLOR);

    for (color = WHITE; color < BLUE + 1; color++)
    {
        switch (color)
        {
        case RED: led_set_color(LED_COUNT, RED_COLOR); break;
        case GREEN: led_set_color(LED_COUNT, GREEN_COLOR); break;
        case BLUE: led_set_color(LED_COUNT, BLUE_COLOR); break;
        default: led_set_color(LED_COUNT, WHITE_COLOR); break;
        }

        led_set_state(5, true);
        led_set_state(6, true);

        for(led_down = 4, led_up = 7; led_down > -1; led_down--, led_up++)
        {
            delay(100);
            led_set_state(led_down + 1, false);
            led_set_state(led_down, true);

            led_set_state(led_up - 1, false);
            led_set_state(led_up, true);
        }

        for(led_down = 10, led_up = 1; led_down > 5; led_down--, led_up++)
        {
            delay(100);
            led_set_state(led_down + 1, false);
            led_set_state(led_down, true);

            led_set_state(led_up - 1, false);
            led_set_state(led_up, true);
        }
    }

    led_set_color(LED_COUNT, 0xFFFFFF); //back to default color
    led_set_state(LED_COUNT, false);
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
            effect_reset_finished = RESET;
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
                effect_reset_finished = SET;
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
