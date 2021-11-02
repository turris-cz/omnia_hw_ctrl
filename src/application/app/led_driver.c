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
#include "stm32f0xx_conf.h"
#include "led_driver.h"
#include "delay.h"
#include "power_control.h"

#define NULL ((void *)0)
#define __packed                    __attribute__((packed))

/* Private define ------------------------------------------------------------*/
#define LED_SPI                     SPI1

#define LED_SPI_MOSI_PIN            GPIO_Pin_7
#define LED_SPI_MOSI_PIN_PORT       GPIOA
#define LED_SPI_MOSI_PIN_CLOCK      RCC_AHBPeriph_GPIOA
#define LED_SPI_MOSI_AF             GPIO_AF_0
#define LED_SPI_MOSI_SOURCE         GPIO_PinSource7

#define LED_SPI_SCK_PIN             GPIO_Pin_5
#define LED_SPI_SCK_PIN_PORT        GPIOA
#define LED_SPI_SCK_PIN_CLOCK       RCC_AHBPeriph_GPIOA
#define LED_SPI_SCK_AF              GPIO_AF_0
#define LED_SPI_SCK_SOURCE          GPIO_PinSource5

#define LED_SPI_SS_PIN              GPIO_Pin_4
#define LED_SPI_SS_PIN_PORT         GPIOA
#define LED_SPI_SS_PIN_CLOCK        RCC_AHBPeriph_GPIOA

#define COLOUR_LEVELS               128
#define COLOUR_DECIMATION           1

/*
 * SystemClock = 48MHz
 * LED TIM freq = SystemClock / LED_TIM_PERIODE / LED_TIM_PRESCALE = 24 kHz
 *
 * LED pattern worker is called from LED TIM interrupt routine and must be
 * executed executed 12 times per millisecond (12 kHz).
 *
 * LED_TIM_PERIODE and LED_TIM_PRESCALE must be such that the division of
 * LED TIM freq by 12 kHz yields no remainder!
 */
#define LED_TIM_PERIODE             100
#define LED_TIM_PRESCALE            20
#define LED_PATTERN_PRESCALE        (48000000 / LED_TIM_PERIODE / LED_TIM_PRESCALE / 12000)

#define MAX_LED_BRIGHTNESS          100
#define MAX_BRIGHTNESS_STEPS        8
#define EFFECT_TIMEOUT              5

/*******************************************************************************
// PWM Settings (Frequency and range)
//------------------------------------------------------------------------------
// period      = range (max = 0xFFFF => 16bit)
// Basic freq. = (APB2=48MHz) => TIM_CLK=48MHz
// period range    : 0 to 0xFFFF
// prescaler range : 0 to 0xFFFF
//
// PWM-Frq     = TIM_CLK/(period+1)/(prescaler+1)
*******************************************************************************/
#define PWM_TIM_PERIODE             2000
#define PWM_TIM_PRESCALE            6

/*--------------------------------------------------------------
// PWM Setting (Polarity)
//
// Hi => Hi-Impuls
// Lo => Lo-Impuls
//-------------------------------------------------------------*/
//#define  PWM_TIM_POLARITY           TIM_OCPolarity_High
#define PWM_TIM_POLARITY            TIM_OCPolarity_Low

#define PWM_TIMER                   TIM15

/* Private macro -------------------------------------------------------------*/
#define LATCH_HIGH                  GPIO_SetBits(LED_SPI_SS_PIN_PORT, LED_SPI_SS_PIN)
#define LATCH_LOW                   GPIO_ResetBits(LED_SPI_SS_PIN_PORT, LED_SPI_SS_PIN)

typedef enum led_effect_states {
	EFFECT_INIT,
	EFFECT_UP,
	EFFECT_DOWN,
	EFFECT_LEDSON,
	EFFECT_DEINIT
} effect_state_t;

struct led_pattern {
	unsigned color : 24;
	unsigned delta_t : 23;
	unsigned gradual : 1;
} __packed;

struct led_pattern_info {
	uint16_t length;
	struct led_pattern patterns[];
};

struct led {
	union {
		uint8_t chan[4];
		uint32_t chan32;
	};
	uint8_t level[3];

	const struct led_pattern_info *pattern;
	const struct led_pattern *start, *curr, *next, *end;
	uint32_t delta_t;
	uint16_t repeat;
};

uint16_t leds_user_mode;
uint16_t leds_state;
uint16_t leds_state_user;
uint16_t leds_color_correction;
struct led leds[LED_COUNT];

static uint16_t leds_pwm_brightness;

/* flag is set when LED effect after reset is
   finished and normal operation can take the LED control */
uint8_t effect_reset_finished;

/* values for LED brightness [%] */
static const uint16_t brightness_value[] = {100, 70, 40, 25, 12, 5, 1, 0};

/* Private functions ---------------------------------------------------------*/
static void led_timer_config_knight_rider(void);

static void led_spi_config(void)
{
	SPI_InitTypeDef  SPI_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, DISABLE);
	SPI_I2S_DeInit(LED_SPI);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

	SPI_I2S_DeInit(LED_SPI);
	SPI_InitStructure.SPI_Direction = SPI_Direction_1Line_Tx;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_Init(LED_SPI, &SPI_InitStructure);

	/* Enable the SPI peripheral */
	SPI_Cmd(LED_SPI, ENABLE);
}

static void led_io_config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Enable SCK, MOSI, and NSS GPIO clocks */
	RCC_AHBPeriphClockCmd(LED_SPI_SCK_PIN_CLOCK | LED_SPI_MOSI_PIN_CLOCK |
			      LED_SPI_SS_PIN_CLOCK, ENABLE);

	GPIO_PinAFConfig(LED_SPI_SCK_PIN_PORT, LED_SPI_SCK_SOURCE, LED_SPI_SCK_AF);
	GPIO_PinAFConfig(LED_SPI_MOSI_PIN_PORT, LED_SPI_MOSI_SOURCE, LED_SPI_MOSI_AF);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_3;

	/* SPI SCK pin configuration */
	GPIO_InitStructure.GPIO_Pin = LED_SPI_SCK_PIN;
	GPIO_Init(LED_SPI_SCK_PIN_PORT, &GPIO_InitStructure);

	/* SPI MOSI pin configuration */
	GPIO_InitStructure.GPIO_Pin =  LED_SPI_MOSI_PIN;
	GPIO_Init(LED_SPI_MOSI_PIN_PORT, &GPIO_InitStructure);

	/* NSS pin configuration */
	GPIO_InitStructure.GPIO_Pin = LED_SPI_SS_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_3;
	GPIO_Init(LED_SPI_SS_PIN_PORT, &GPIO_InitStructure);

	/* init state - latch holds previous data */
	LATCH_LOW;
}

static void led_timer_config(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, DISABLE);
	TIM_DeInit(LED_TIMER);

	/* Clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = LED_TIM_PERIODE - 1;
	TIM_TimeBaseStructure.TIM_Prescaler = LED_TIM_PRESCALE - 1;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(LED_TIMER, &TIM_TimeBaseStructure);

	TIM_ARRPreloadConfig(LED_TIMER, ENABLE);
	/* TIM Interrupts enable */
	TIM_ITConfig(LED_TIMER, TIM_IT_Update, ENABLE);

	/* TIM enable counter */
	TIM_Cmd(LED_TIMER, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 0x04;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

/*
 * Lightness-luminance correction table according to CIE 1931.
 * Red color channel goes all the way to full lightness here.
 * Green and blue's lightness go only to 2/3 of red's lightness, because these
 * LEDs are more luminuous.
 */
static const uint8_t cie1931_table_R[256] = {
	  0,   0,   0,   0,   0,   0,   0,   0,   0,   1,   1,   1,   1,   1,   1,   1,
	  1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   2,   2,   2,   2,   2,
	  2,   2,   2,   2,   2,   2,   2,   3,   3,   3,   3,   3,   3,   3,   3,   3,
	  3,   4,   4,   4,   4,   4,   4,   4,   4,   5,   5,   5,   5,   5,   5,   6,
	  6,   6,   6,   6,   6,   7,   7,   7,   7,   7,   7,   8,   8,   8,   8,   9,
	  9,   9,   9,   9,  10,  10,  10,  10,  11,  11,  11,  11,  12,  12,  12,  12,
	 13,  13,  13,  14,  14,  14,  14,  15,  15,  15,  16,  16,  16,  17,  17,  17,
	 18,  18,  18,  19,  19,  19,  20,  20,  21,  21,  21,  22,  22,  23,  23,  23,
	 24,  24,  25,  25,  26,  26,  26,  27,  27,  28,  28,  29,  29,  30,  30,  31,
	 31,  32,  32,  33,  33,  34,  34,  35,  35,  36,  37,  37,  38,  38,  39,  39,
	 40,  41,  41,  42,  42,  43,  44,  44,  45,  46,  46,  47,  48,  48,  49,  50,
	 50,  51,  52,  53,  53,  54,  55,  55,  56,  57,  58,  58,  59,  60,  61,  62,
	 62,  63,  64,  65,  66,  67,  67,  68,  69,  70,  71,  72,  73,  73,  74,  75,
	 76,  77,  78,  79,  80,  81,  82,  83,  84,  85,  86,  87,  88,  89,  90,  91,
	 92,  93,  94,  95,  96,  97,  98,  99, 100, 102, 103, 104, 105, 106, 107, 108,
	109, 111, 112, 113, 114, 115, 117, 118, 119, 120, 122, 123, 124, 125, 127, 128,
};

static const uint8_t cie1931_table_GB[256] = {
	  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   1,   1,
	  1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,
	  1,   1,   1,   1,   1,   1,   1,   1,   2,   2,   2,   2,   2,   2,   2,   2,
	  2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   3,   3,   3,   3,   3,
	  3,   3,   3,   3,   3,   3,   3,   3,   3,   4,   4,   4,   4,   4,   4,   4,
	  4,   4,   4,   4,   4,   5,   5,   5,   5,   5,   5,   5,   5,   5,   5,   6,
	  6,   6,   6,   6,   6,   6,   6,   6,   7,   7,   7,   7,   7,   7,   7,   7,
	  8,   8,   8,   8,   8,   8,   8,   9,   9,   9,   9,   9,   9,   9,  10,  10,
	 10,  10,  10,  10,  11,  11,  11,  11,  11,  11,  12,  12,  12,  12,  12,  12,
	 13,  13,  13,  13,  13,  14,  14,  14,  14,  14,  15,  15,  15,  15,  15,  16,
	 16,  16,  16,  17,  17,  17,  17,  17,  18,  18,  18,  18,  19,  19,  19,  19,
	 20,  20,  20,  20,  21,  21,  21,  21,  22,  22,  22,  22,  23,  23,  23,  24,
	 24,  24,  24,  25,  25,  25,  26,  26,  26,  26,  27,  27,  27,  28,  28,  28,
	 29,  29,  29,  30,  30,  30,  31,  31,  31,  32,  32,  32,  33,  33,  33,  34,
	 34,  34,  35,  35,  35,  36,  36,  37,  37,  37,  38,  38,  38,  39,  39,  40,
	 40,  40,  41,  41,  42,  42,  42,  43,  43,  44,  44,  45,  45,  45,  46,  46,
};

static void _led_compute_levels(struct led *led, int color_correction)
{
	uint8_t r, g, b;

	r = led->chan[0];
	g = led->chan[1];
	b = led->chan[2];

	if (color_correction) {
		led->level[0] = cie1931_table_R[r];
		led->level[1] = cie1931_table_GB[g];
		led->level[2] = cie1931_table_GB[b];
	} else {
		led->level[0] = r >> COLOUR_DECIMATION;
		led->level[1] = g >> COLOUR_DECIMATION;
		led->level[2] = b >> COLOUR_DECIMATION;
	}
}

void led_compute_levels(int led, int color_correction)
{
	_led_compute_levels(&leds[led], color_correction);
}

void led_compute_levels_all(int color_correction)
{
	struct led *led = leds;
	int i;

	for (i = 0, led = leds; i < LED_COUNT; ++i, ++led)
		_led_compute_levels(led, color_correction);
}

static void _led_set_colour(struct led *led, uint32_t colour, int color_correction)
{
	led->chan32 = __builtin_bswap32(colour << 8);
	_led_compute_levels(led, color_correction);
}

void led_set_colour(int led, uint32_t colour)
{
	_led_set_colour(&leds[led], colour, leds_color_correction & BIT(led));
}

void led_set_colour_all(uint32_t colour)
{
	struct led *led = leds;
	int i;

	for (i = 0, led = leds; i < LED_COUNT; ++i, ++led)
		_led_set_colour(led, colour, leds_color_correction & BIT(i));
}

static const struct led_pattern_info rainbow_pattern = {
	.length = 6,
	.patterns = {
		{ 0xff0000, 1000, 1 },
		{ 0xffff00, 1000, 1 },
		{ 0x00ff00, 1000, 1 },
		{ 0x00ffff, 1000, 1 },
		{ 0x0000ff, 1000, 1 },
		{ 0xff00ff, 1000, 1 },
	},
};

static const struct led_pattern_info knight_rider_pattern_red = {
	.length = 3,
	.patterns = {
		{ 0xff0000, 150, 0 },
		{ 0xff0000, 450, 1 },
		{ 0x000000, 1, 0 },
	},
};

static const struct led_pattern *
next_pattern(const struct led_pattern *cur,
	     const struct led_pattern_info *pattern)
{
	++cur;
	if (cur == pattern->patterns + pattern->length)
		cur = pattern->patterns;
	return cur;
}

static const struct led_pattern_info knight_rider_pattern;

void led_set_pattern(int l, int pattern_id, int repeat, int pos, int len,
		     int pos_t)
{
	const struct led_pattern_info *pattern;
	struct led *led = &leds[l];
	int end;

	switch (pattern_id) {
	case 1:
		pattern = &rainbow_pattern;
		break;
	case 2:
		pattern = &knight_rider_pattern_red;
		break;
	case 3:
		pattern = &knight_rider_pattern;
		break;
	default:
		led->pattern = NULL;
		led_set_colour(l, 0x000000);
		return;
	}

	if (pos > pattern->length)
		pos = 0;

	if (len > pattern->length || len < 2)
		len = pattern->length;

	end = pos + len;
	if (end >= pattern->length)
		end -= pattern->length;

	led->repeat = repeat;

	led->start = &pattern->patterns[pos];
	led->curr = led->start;
	led->end = &pattern->patterns[end];
	led->next = next_pattern(led->curr, pattern);

	if (pos_t >= led->curr->delta_t)
		pos_t = 0;

	led->delta_t = led->curr->delta_t - pos_t;

	led->pattern = pattern;
	led_set_colour(l, led->curr->color);
}

static void led_pattern_update(int l, struct led *led,
			       const struct led_pattern_info *pattern)
{
	led->curr = next_pattern(led->curr, pattern);
	led->next = next_pattern(led->next, pattern);

	if (led->next == led->end) {
		if (led->repeat == 1) {
			led_set_colour(l, led->curr->color);
			led->pattern = NULL;
			return;
		} else {
			led->next = led->start;
		}
	} else if (led->curr == led->end) {
		if (led->repeat != 0)
			--led->repeat;
		led->curr = led->start;
	}

	led->delta_t = led->curr->delta_t;
	led_set_colour(l, led->curr->color);
}

static uint32_t rgb_between(uint32_t ca, uint32_t cb, int n, int d)
{
	uint8_t a[4], b[4];
	int i;

	*(uint32_t *)a = __builtin_bswap32(ca << 8);
	*(uint32_t *)b = __builtin_bswap32(cb << 8);

	for (i = 0; i < 3; ++i)
		a[i] += ((int)b[i] - (int)a[i]) * n / d;

	return __builtin_bswap32(*(uint32_t *)a) >> 8;
}

static void led_pattern_work(int l)
{
	const struct led_pattern_info *pattern;
	struct led *led;

	led = &leds[l];
	pattern = led->pattern;

	if (!pattern)
		return;

	--led->delta_t;

	if (!led->delta_t)
		return led_pattern_update(l, led, pattern);

	if (led->curr->gradual)
		led_set_colour(l, rgb_between(led->curr->color, led->next->color, led->curr->delta_t - led->delta_t, led->curr->delta_t));
}

static void led_send_data16b(const uint16_t data)
{
	SPI_I2S_SendData16(LED_SPI, data);

	/* wait for flag */
	while (SPI_I2S_GetFlagStatus(LED_SPI, SPI_I2S_FLAG_BSY))
		;
}

static uint16_t led_prepare_data(int chan, int level)
{
	uint16_t data = 0;
	struct led *led;
	int i;

	for (i = 0, led = leds; i < LED_COUNT; ++i, ++led) {
		if (led->level[chan] > level)
			data |= BIT(i);
	}

	data &= leds_state;

	return data << 2;
}

static void led_send_frame(void)
{
	static int channel = 0;
	static int level = 0;
	uint16_t data;

	data = led_prepare_data(channel++, level);
	if (channel == 3)
		channel = 0;

	led_send_data16b(data);

	/* blue channel data were sent to driver -> enable latch to write to LEDs */
	if (channel == 0) {
		/* latch enable pulse */
		LATCH_HIGH;
		__NOP();
		LATCH_LOW;

		level++;

		 /* restart cycle - all levels were sent to driver */
		if (level >= COLOUR_LEVELS)
			level = 0;
	}
}

uint32_t last_led_timer_start, last_led_timer_end;
void led_timer_irq_handler(void)
{
	static int pattern_work_led = 0;
	static int pattern_pres_cnt = 0;

	last_led_timer_start = TIM_GetCounter(LED_TIMER);

	if (++pattern_pres_cnt == LED_PATTERN_PRESCALE)
		pattern_pres_cnt = 0;

	if (!pattern_pres_cnt) {
		led_pattern_work(pattern_work_led++);
		if (pattern_work_led == LED_COUNT)
			pattern_work_led = 0;
	}

	led_send_frame();

	last_led_timer_end = TIM_GetCounter(LED_TIMER);
}

/*******************************************************************************
  * @function   led_pwm_io_config
  * @brief      Config of PWM signal for LED driver.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
static void led_pwm_io_config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Clock Enable */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_1;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_0);
}

/*******************************************************************************
  * @function   led_pwm_timer_config
  * @brief      Timer config for PWM signal (OE pin of LED driver).
  * @param      None.
  * @retval     None.
  *****************************************************************************/
static void led_pwm_timer_config(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	uint16_t init_value;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM15, DISABLE);
	TIM_DeInit(PWM_TIMER);

	/* Clock enable */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM15, ENABLE);

	TIM_TimeBaseStructure.TIM_Period = PWM_TIM_PERIODE - 1;
	TIM_TimeBaseStructure.TIM_Prescaler = PWM_TIM_PRESCALE - 1;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(PWM_TIMER, &TIM_TimeBaseStructure);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	init_value = 0;

	TIM_OCInitStructure.TIM_Pulse = init_value;
	TIM_OCInitStructure.TIM_OCPolarity = PWM_TIM_POLARITY;
	TIM_OC2Init(PWM_TIMER, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(PWM_TIMER, TIM_OCPreload_Enable);

	/* Timer enable */
	TIM_ARRPreloadConfig(PWM_TIMER, ENABLE);
	TIM_Cmd(PWM_TIMER, ENABLE);
	TIM_CtrlPWMOutputs(PWM_TIMER, ENABLE);
}

/*******************************************************************************
  * @function   led_pwm_config
  * @brief      Configuration of PWM functionality.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
static void led_pwm_config(void)
{
	led_pwm_io_config();
	led_pwm_timer_config();
}

/*******************************************************************************
  * @function   led_init_led
  * @brief      Enable all LED to default mode (LEDs are ON).
  * @param      None.
  * @retval     None.
  *****************************************************************************/
static void led_init_led(void)
{
	leds_user_mode = 0;
	leds_state = 0;
	leds_state_user = 0xfff;
	leds_color_correction = 0xfff;

	led_set_colour_all(WHITE_COLOUR);
}

/*******************************************************************************
  * @function   led_config
  * @brief      Configure LED driver.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void led_config(void)
{
	led_io_config();
	led_spi_config();

	/* set mode and state - default after reset */
	led_init_led();

	led_pwm_config();
	/* 100% brightness after reset */
	led_pwm_set_brightness(MAX_LED_BRIGHTNESS);

	led_timer_config();
	led_timer_config_knight_rider();
}

/*******************************************************************************
  * @function   led_pwm_set_brightness
  * @brief      Set PWM value.
  * @param      procent_val: PWM value in [%].
  * @retval     None.
  *****************************************************************************/
void led_pwm_set_brightness(uint16_t procent_val)
{
	uint16_t counter_val;

	if (procent_val > MAX_LED_BRIGHTNESS)
		procent_val = MAX_LED_BRIGHTNESS;

	counter_val = procent_val * PWM_TIM_PERIODE / MAX_LED_BRIGHTNESS;

	PWM_TIMER->CCR2 = counter_val;
	leds_pwm_brightness = procent_val;
}

/*******************************************************************************
  * @function   led_pwm_get_brightness
  * @brief      Set PWM value.
  * @param      None.
  * @retval     procent_val: PWM value in [%].
  *****************************************************************************/
uint16_t led_pwm_get_brightness(void)
{
	return leds_pwm_brightness;
}

/*******************************************************************************
  * @function   led_step_brightness
  * @brief      Decrease LED brightness by step (each function call).
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void led_step_brightness(void)
{
    static uint8_t step = 1;

    leds_pwm_brightness = brightness_value[step++];
    led_pwm_set_brightness(leds_pwm_brightness);

    if (step >= MAX_BRIGHTNESS_STEPS)
        step = 0;
}

/*******************************************************************************
  * @function   led_knight_rider_effect
  * @brief      Display knight rider effect on LEDs.
  * @param      colour: colour in RGB range.
  * @retval     None.
  *****************************************************************************/
void led_knight_rider_effect(uint32_t colour)
{
	int led;

	led_set_state_all(0);
	led_set_colour_all(colour);
	led_set_state(LED0, 1);

	for (led = LED1; led < LED_COUNT; led++) {
		delay(70);
		led_set_state(led - 1, 0);
		led_set_state(led, 1);
	}

	for (led = LED10; led > -1 ; led--) {
		delay(70);
		led_set_state(led + 1, 0);
		led_set_state(led, 1);
	}

	led_set_colour_all(WHITE_COLOUR);
	led_set_state_all(0);
}

/*******************************************************************************
  * @function   led_knight_rider_colour_effect
  * @brief      Display knight rider effect on LEDs.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void led_knight_rider_colour_effect(void)
{
	led_set_state_all(0);

	led_knight_rider_effect(WHITE_COLOUR);
	led_knight_rider_effect(RED_COLOUR);
	led_knight_rider_effect(GREEN_COLOUR);
	led_knight_rider_effect(BLUE_COLOUR);
}

/*******************************************************************************
  * @function   led_double_knight_rider_effect
  * @brief      Display double knight rider effect on LEDs.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void led_double_knight_rider_effect(void)
{
	static const uint32_t colours[4] = { WHITE_COLOUR, RED_COLOUR, GREEN_COLOUR, BLUE_COLOUR };
	int d, u, c;

	led_set_state_all(0);

	for (c = 0; c < 4; ++c) {
		led_set_colour_all(colours[c]);

		led_set_state(LED5, 1);
		led_set_state(LED6, 1);

		for (d = LED4, u = LED7; d > -1; d--, u++) {
			delay(100);

			led_set_state(d + 1, 0);
			led_set_state(d, 1);

			led_set_state(u - 1, 0);
			led_set_state(u, 1);
		}

		for (d = LED10, u = LED1; d > LED5; d--, u++) {
			delay(100);

			led_set_state(d + 1, 0);
			led_set_state(d, 1);

			led_set_state(u - 1, 0);
			led_set_state(u, 1);
		}
	}

	led_set_colour_all(WHITE_COLOUR);
	led_set_state_all(0);
}

/*******************************************************************************
  * @function   led_timer_config_knight_rider
  * @brief      Timer config for knight rider effect after reset.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
static void led_timer_config_knight_rider(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, DISABLE);
	TIM_DeInit(LED_EFFECT_TIMER);

	/* Clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);

	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = 8000 - 1;
	TIM_TimeBaseStructure.TIM_Prescaler = 400 - 1;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(LED_EFFECT_TIMER, &TIM_TimeBaseStructure);

	TIM_ARRPreloadConfig(LED_EFFECT_TIMER, ENABLE);
	/* TIM Interrupts enable */
	TIM_ITConfig(LED_EFFECT_TIMER, TIM_IT_Update, ENABLE);

	/* TIM enable counter */
	/* TIM_Cmd(LED_EFFECT_TIMER, ENABLE); */

	/* Timer is enable after reset */

	NVIC_InitStructure.NVIC_IRQChannel = TIM6_DAC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 0x05;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

/*******************************************************************************
  * @function   led_reset_effect
  * @brief      Enable/Disable knight rider effect after reset.
  * @param      colour: colour in RGB range.
  * @retval     None.
  *****************************************************************************/
void led_reset_effect(FunctionalState state)
{
	if (state == ENABLE) {
		TIM_Cmd(LED_EFFECT_TIMER, ENABLE);
	} else {
		TIM_Cmd(LED_EFFECT_TIMER, DISABLE);
		LED_EFFECT_TIMER->CNT = 0;
	}
}

static const struct led_pattern_info knight_rider_pattern = {
	.length = 3,
	.patterns = {
		{ 0xffffff, 70, 0 },
		{ 0xffffff, 300, 1 },
		{ 0x000000, 1, 0 },
	},
};

/*******************************************************************************
  * @function   led_knight_rider_effect_handler
  * @brief      Display knight rider effect on LEDs during startup (called in
  *             timer interrupt).
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void led_knight_rider_effect_handler(void)
{
	static int8_t led;
	static uint8_t state_timeout_cnt;
	static effect_state_t effect_state; /* states for LED effect after reset */

	switch (effect_state) {
	case EFFECT_INIT:
		effect_reset_finished = RESET;
		led_set_state_all(1);
		led_set_colour_all(0x0);
//		led_set_color_correction_all(1);
		led_set_pattern(led, 3, 1, 0, 0, 0);
		effect_state = EFFECT_UP;
		break;

	case EFFECT_UP:
		led++;
		led_set_pattern(led, 3, 1, 0, 0, 0);

		if (led >= LED11)
			effect_state = EFFECT_DOWN; /* next state */
		break;

	case EFFECT_DOWN:
		led--;
		led_set_pattern(led, 3, 1, 0, 0, 0);

		if (led <= 0)
			effect_state = EFFECT_LEDSON; /* next state */
		break;

	case EFFECT_LEDSON:
		led_set_state_all(1);
		led_set_colour_all(GREEN_COLOUR | BLUE_COLOUR);
		effect_state = EFFECT_DEINIT;
		break;

	case EFFECT_DEINIT:
		state_timeout_cnt++;

		if (state_timeout_cnt >= EFFECT_TIMEOUT) {
			led_set_state_all(0);
//			led_set_color_correction_all(0);
			led_set_colour_all(WHITE_COLOUR);

			led_set_user_mode_all(0);
			led_reset_effect(DISABLE);
			state_timeout_cnt = 0;
			effect_reset_finished = SET;
			effect_state = EFFECT_INIT;
		} else {
			effect_state = EFFECT_DEINIT;
		}

		break;
	}
}
