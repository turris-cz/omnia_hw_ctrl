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

#define COLOUR_LEVELS               16
#define COLOUR_DECIMATION           4 /* 2exp(4) = 16 colour levels */
#define MAX_LED_BRIGHTNESS          100
#define MAX_BRIGHTNESS_STEPS        8

/*******************************************************************************
// PWM Settings (Frequency und range)
//------------------------------------------------------------------------------
// period      = range (max = 0xFFFF => 16bit)
// Basic freq. = (APB2=48MHz) => TIM_CLK=48MHz
// period range    : 0 to 0xFFFF
// prescaler range : 0 to 0xFFFF
//
// PWM-Frq     = TIM_CLK/(period+1)/(prescaler+1)
*******************************************************************************/
#define PWM_TIM_PERIODE             0xFF /* period   (0xFF => 8bit) */
#define PWM_TIM_PRESCALE            0xFF /* prescaler */

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

/* Private typedef -----------------------------------------------------------*/
typedef enum rgb_colour {
    RED     = 0,
    GREEN   = 1,
    BLUE    = 2,
    WHITE   = -1,
}rgb_colour_t;


struct led_rgb leds[LED_COUNT];

/* values for LED brightness [%] */
static const uint16_t brightness_value[] = {100, 70, 40, 25, 12, 5, 1, 0};

/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
  * @function   led_driver_spi_config
  * @brief      SPI config for led driver serial register.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
static void led_driver_spi_config(void)
{
    SPI_InitTypeDef  SPI_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

    SPI_I2S_DeInit(LED_SPI);
    SPI_InitStructure.SPI_Direction = SPI_Direction_1Line_Tx;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_128;
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_Init(LED_SPI, &SPI_InitStructure);

    /* Enable the SPI peripheral */
    SPI_Cmd(LED_SPI, ENABLE);
}

/*******************************************************************************
  * @function   led_driver_io_config
  * @brief      GPIO config for led driver serial register.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
static void led_driver_io_config(void)
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

/*******************************************************************************
  * @function   led_driver_timer_config
  * @brief      Timer config for led driver serial register - send data regularly.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
static void led_driver_timer_config(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Clock enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    /* Time base configuration */
    TIM_TimeBaseStructure.TIM_Period = 0xFF - 1;
    TIM_TimeBaseStructure.TIM_Prescaler = 0xFF - 1;
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

/*******************************************************************************
  * @function   led_driver_set_colour
  * @brief      Set colour of LED specified in parameters to be displayed in next cycle.
  * @param      led_index: position of LED (0..11) or index >=12 -> all LEDs
  * @param      colour: LED colour (RGB range).
  * @retval     None.
  *****************************************************************************/
void led_driver_set_colour(const uint8_t led_index, const uint32_t colour)
{
    uint8_t idx;
    struct led_rgb *rgb_leds = leds;

    if (led_index >= LED_COUNT) /* all LEDs */
    {
        for (idx = 0; idx < LED_COUNT; idx++, rgb_leds++)
        {
            if (rgb_leds->led_mode == LED_DEFAULT_MODE)
            {
                rgb_leds->led_rgb_default.red = colour >> 16;
                rgb_leds->led_rgb_default.green = (colour >> 8) & 0xFF;
                rgb_leds->led_rgb_default.blue = colour & 0xFF;
            }
            else /* LED_USER_MODE */
            {
                rgb_leds->led_rgb_user.red = colour >> 16;
                rgb_leds->led_rgb_user.green = (colour >> 8) & 0xFF;
                rgb_leds->led_rgb_user.blue = colour & 0xFF;
            }
        }
    }
    else /* individual LED */
    {
        rgb_leds += led_index;

        if (rgb_leds->led_mode == LED_DEFAULT_MODE)
        {
            rgb_leds->led_rgb_default.red = colour >> 16;
            rgb_leds->led_rgb_default.green = (colour >> 8) & 0xFF;
            rgb_leds->led_rgb_default.blue = colour & 0xFF;
        }
        else /* LED_USER_MODE */
        {
            rgb_leds->led_rgb_user.red = colour >> 16;
            rgb_leds->led_rgb_user.green = (colour >> 8) & 0xFF;
            rgb_leds->led_rgb_user.blue = colour & 0xFF;
        }
    }
}

/*******************************************************************************
  * @function   led_driver_send_data16b
  * @brief      Send SPI data to the LED driver.
  * @param      data: 16bit data buffer.
  * @retval     None.
  *****************************************************************************/
static void led_driver_send_data16b(const uint16_t data)
{
    SPI_I2S_SendData16(LED_SPI, data);
    /* wait for flag */
    while(SPI_I2S_GetFlagStatus(LED_SPI, SPI_I2S_FLAG_BSY));
}

/*******************************************************************************
  * @function   led_driver_prepare_data
  * @brief      Prepare data to be sent to LED driver.
  * @param      colour: RED, GREEN or BLUE.
  * @param      current_colour_level: colour density [0..255]
  * @retval     Data to be sent to LED driver.
  *****************************************************************************/
static uint16_t led_driver_prepare_data(const rgb_colour_t colour, const uint8_t current_colour_level)
{
    uint16_t data = 0;
    uint8_t idx;
    struct led_rgb *rgb_leds = leds;

    switch (colour)
    {
        case RED:
        {
            for (idx = 0; idx < LED_COUNT; idx++, rgb_leds++)
            {
                if (rgb_leds->led_mode == LED_DEFAULT_MODE)
                {
                    if (rgb_leds->led_state_default == LED_ON)
                    {
                        if (rgb_leds->led_rgb_default.red > current_colour_level)
                        {
                            data |= 1 << (2 + idx); /* shift by 2 - due to the HW connection */
                        }
                    }
                }
                else /* LED_USER_MODE */
                {
                    if (rgb_leds->led_state_user == LED_ON)
                    {
                        if (rgb_leds->led_rgb_user.red > current_colour_level)
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
                if (rgb_leds->led_mode == LED_DEFAULT_MODE)
                {
                    if (rgb_leds->led_state_default == LED_ON)
                    {
                        if (rgb_leds->led_rgb_default.green > current_colour_level)
                        {
                            data |= 1 << (2 + idx);
                        }
                    }
                }
                else /* LED_USER_MODE */
                {
                    if (rgb_leds->led_state_user == LED_ON)
                    {
                        if (rgb_leds->led_rgb_user.green > current_colour_level)
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
                if (rgb_leds->led_mode == LED_DEFAULT_MODE)
                {
                    if (rgb_leds->led_state_default == LED_ON)
                    {
                        if (rgb_leds->led_rgb_default.blue > current_colour_level)
                        {
                            data |= 1 << (2 + idx);
                        }
                    }
                }
                else /* LED_USER_MODE */
                {
                    if (rgb_leds->led_state_user == LED_ON)
                    {
                        if (rgb_leds->led_rgb_user.blue > current_colour_level)
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
void led_driver_send_frame(void)
{
    static uint8_t level;
    uint16_t data;

    data = led_driver_prepare_data(RED, level << COLOUR_DECIMATION);
    led_driver_send_data16b(data);

    data = led_driver_prepare_data(GREEN, level << COLOUR_DECIMATION);
    led_driver_send_data16b(data);

    data = led_driver_prepare_data(BLUE, level << COLOUR_DECIMATION);
    led_driver_send_data16b(data);

     /* latch enable pulse */
    LATCH_HIGH;
    __NOP();
    LATCH_LOW;

    level++;

    /* restart cycle - all levels were sent to driver */
    if (level >= COLOUR_LEVELS)
        level = 0;
}

/*******************************************************************************
  * @function   led_driver_pwm_io_config
  * @brief      Config of PWM signal for LED driver.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
static void led_driver_pwm_io_config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* Clock Enable */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_0);
}

/*******************************************************************************
  * @function   led_driver_pwm_timer_config
  * @brief      Timer config for PWM signal (OE pin of LED driver).
  * @param      None.
  * @retval     None.
  *****************************************************************************/
static void led_driver_pwm_timer_config(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;
    uint16_t init_value;

    /* Clock enable */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM15, ENABLE);

    TIM_TimeBaseStructure.TIM_Period = PWM_TIM_PERIODE;
    TIM_TimeBaseStructure.TIM_Prescaler = PWM_TIM_PRESCALE;
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
  * @function   led_driver_pwm_config
  * @brief      Configuration of PWM functionality.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
static void led_driver_pwm_config(void)
{
    led_driver_pwm_io_config();
    led_driver_pwm_timer_config();
}

/*******************************************************************************
  * @function   led_driver_init_led
  * @brief      Enable all LED to default mode (LEDs are ON).
  * @param      None.
  * @retval     None.
  *****************************************************************************/
static void led_driver_init_led(void)
{
    uint8_t idx;
    struct led_rgb *rgb_leds = leds;

    /* user mode - all LEDs white and ON */
    for (idx = 0; idx < LED_COUNT; idx++, rgb_leds++)
    {
        rgb_leds->led_state_user = LED_ON;
        rgb_leds->led_mode = LED_USER_MODE;
    }
    led_driver_set_colour(LED_COUNT, WHITE_COLOUR); /* all LEDs black */

    /* default mode - all LEDS OFF and white */
    for (idx = 0, rgb_leds = leds; idx < LED_COUNT; idx++, rgb_leds++)
    {
        rgb_leds->led_state_default = LED_OFF;
        rgb_leds->led_mode = LED_DEFAULT_MODE;
    }
    led_driver_set_colour(LED_COUNT, WHITE_COLOUR); /* all LEDs white */
}

/*******************************************************************************
  * @function   led_driver_config
  * @brief      Configure LED driver.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void led_driver_config(void)
{
    led_driver_io_config();
    led_driver_spi_config();

    led_driver_init_led(); /* set mode and state - default after reset */

    led_driver_pwm_config();
    led_driver_pwm_set_brightness(MAX_LED_BRIGHTNESS); /* 100% brightness after reset */

    led_driver_timer_config();
}

/*******************************************************************************
  * @function   led_driver_pwm_set_brightness
  * @brief      Set PWM value.
  * @param      procent_val: PWM value in [%].
  * @retval     None.
  *****************************************************************************/
void led_driver_pwm_set_brightness(uint16_t procent_val)
{
    uint16_t counter_val;
    struct led_rgb *rgb_leds = leds;

    if (procent_val > MAX_LED_BRIGHTNESS)
        procent_val = MAX_LED_BRIGHTNESS;

    counter_val = procent_val * PWM_TIM_PERIODE / MAX_LED_BRIGHTNESS;

    PWM_TIMER->CCR2 = counter_val;
    rgb_leds->brightness = procent_val;
}

/*******************************************************************************
  * @function   led_driver_step_brightness
  * @brief      Decrease LED brightness by step (each function call).
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void led_driver_step_brightness(void)
{
    struct led_rgb *rgb_leds = leds;
    static uint8_t step = 1;

    rgb_leds->brightness = brightness_value[step++];
    led_driver_pwm_set_brightness(rgb_leds->brightness);

    if (step >= MAX_BRIGHTNESS_STEPS)
        step = 0;
}

/*******************************************************************************
  * @function   led_driver_set_led_mode
  * @brief      Set mode to LED(s) - default or user mode
  * @param      led_index: position of LED (0..11) or led_index >=12 -> all LED.
  * @parame     led_mode: LED_DEFAULT_MODE / LED_USER_MODE
  * @retval     None.
  *****************************************************************************/
void led_driver_set_led_mode(const uint8_t led_index, const led_mode_t led_mode)
{
    uint8_t idx;
    struct led_rgb *rgb_leds = leds;

    if (led_index >= LED_COUNT)  /* all LED */
    {
        for (idx = 0; idx < LED_COUNT; idx++, rgb_leds++)
        {
            rgb_leds->led_mode = led_mode;
        }
    }
    else /* or individual LED */
    {
        rgb_leds += led_index;
        rgb_leds->led_mode = led_mode;
    }
}

/*******************************************************************************
  * @function   led_driver_set_led_state
  * @brief      Set state of the LED(s) - LED_ON / LED_OFF
  * @param      led_index: position of LED (0..11) or led_index >=12 -> all LED.
  * @parame     led_state: LED_OFF / LED_ON
  * @retval     None.
  *****************************************************************************/
void led_driver_set_led_state(const uint8_t led_index, const led_state_t led_state)
{
    uint8_t idx;
    struct led_rgb *rgb_leds = leds;

    if (led_index >= LED_COUNT) /* all LED */
    {
        for (idx = 0; idx < LED_COUNT; idx++, rgb_leds++)
        {
            if (rgb_leds->led_mode == LED_DEFAULT_MODE)
            {
                rgb_leds->led_state_default = led_state;
            }
            else
            {
                rgb_leds->led_state_user = led_state;
            }
        }
    }
    else /* or individual LED */
    {
        rgb_leds += led_index;

        if (rgb_leds->led_mode == LED_DEFAULT_MODE)
            {
                rgb_leds->led_state_default = led_state;
            }
            else
            {
                rgb_leds->led_state_user = led_state;
            }
    }
}

/*******************************************************************************
  * @function   led_driver_knight_rider_effect
  * @brief      Display knight rider effect on LEDs.
  * @param      colour: colour in RGB range.
  * @retval     None.
  *****************************************************************************/
void led_driver_knight_rider_effect(uint32_t colour)
{
    int8_t led;

    led_driver_set_led_state(LED_COUNT, LED_OFF);
    led_driver_set_colour(LED_COUNT, colour);
    led_driver_set_led_state(LED0, LED_ON);

    for (led = LED1; led < LED_COUNT; led++)
    {
        delay(70);
        led_driver_set_led_state(led - 1, LED_OFF);
        led_driver_set_led_state(led, LED_ON);
    }

    for (led = LED10; led > -1 ; led--)
    {
        delay(70);
        led_driver_set_led_state(led + 1, LED_OFF);
        led_driver_set_led_state(led, LED_ON);
    }

    led_driver_set_colour(LED_COUNT, WHITE_COLOUR);
    led_driver_set_led_state(LED_COUNT, LED_OFF);
}

/*******************************************************************************
  * @function   led_driver_knight_rider_colour_effect
  * @brief      Display knight rider effect on LEDs.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void led_driver_knight_rider_colour_effect(void)
{
    int8_t colour;

    led_driver_set_led_state(LED_COUNT, LED_OFF);

    for (colour = WHITE; colour < BLUE + 1; colour++)
    {
        switch (colour)
        {
        case RED: led_driver_knight_rider_effect(RED_COLOUR); break;
        case GREEN: led_driver_knight_rider_effect(GREEN_COLOUR); break;
        case BLUE: led_driver_knight_rider_effect(BLUE_COLOUR); break;
        default: led_driver_knight_rider_effect(WHITE_COLOUR); break;
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
    int8_t colour;

    led_driver_set_led_state(LED_COUNT, LED_OFF);
    led_driver_set_colour(LED_COUNT, WHITE_COLOUR);

    for (colour = WHITE; colour < BLUE + 1; colour++)
    {
        switch (colour)
        {
        case RED: led_driver_set_colour(LED_COUNT, RED_COLOUR); break;
        case GREEN: led_driver_set_colour(LED_COUNT, GREEN_COLOUR); break;
        case BLUE: led_driver_set_colour(LED_COUNT, BLUE_COLOUR); break;
        default: led_driver_set_colour(LED_COUNT, WHITE_COLOUR); break;
        }

        led_driver_set_led_state(LED5, LED_ON);
        led_driver_set_led_state(LED6, LED_ON);

        for(led_down = LED4, led_up = LED7; led_down > -1; led_down--, led_up++)
        {
            delay(100);
            led_driver_set_led_state(led_down + 1, LED_OFF);
            led_driver_set_led_state(led_down, LED_ON);

            led_driver_set_led_state(led_up - 1, LED_OFF);
            led_driver_set_led_state(led_up, LED_ON);
        }

        for(led_down = LED10, led_up = LED1; led_down > LED5; led_down--, led_up++)
        {
            delay(100);
            led_driver_set_led_state(led_down + 1, LED_OFF);
            led_driver_set_led_state(led_down, LED_ON);

            led_driver_set_led_state(led_up - 1, LED_OFF);
            led_driver_set_led_state(led_up, LED_ON);
        }
    }

    led_driver_set_colour(LED_COUNT, 0xFFFFFF); //back to default colour
    led_driver_set_led_state(LED_COUNT, LED_OFF);
}
