/**
 ******************************************************************************
 * @file    led_driver.c
 * @author  CZ.NIC, z.s.p.o.
 * @date    21-July-2015
 * @brief   LED RGB driver for Turris Lite
 ******************************************************************************
 ******************************************************************************
 **/
/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_conf.h"
#include "led_driver.h"

/* Private define ------------------------------------------------------------*/
#define LED_SPI						SPI1

#define LED_SPI_MOSI_PIN			GPIO_Pin_7
#define LED_SPI_MOSI_PIN_PORT		GPIOA
#define LED_SPI_MOSI_PIN_CLOCK      RCC_AHBPeriph_GPIOA
#define LED_SPI_MOSI_AF             GPIO_AF_0
#define LED_SPI_MOSI_SOURCE         GPIO_PinSource7

#define LED_SPI_SCK_PIN				GPIO_Pin_5
#define LED_SPI_SCK_PIN_PORT		GPIOA
#define LED_SPI_SCK_PIN_CLOCK       RCC_AHBPeriph_GPIOA
#define LED_SPI_SCK_AF              GPIO_AF_0
#define LED_SPI_SCK_SOURCE          GPIO_PinSource5

#define LED_SPI_SS_PIN				GPIO_Pin_4
#define LED_SPI_SS_PIN_PORT 		GPIOA
#define LED_SPI_SS_PIN_CLOCK        RCC_AHBPeriph_GPIOA

#define COLOUR_LEVELS				16
#define COLOUR_DECIMATION           4 // 2exp(4) = 16 colour levels
#define MAX_LED_BRIGHT_STEP         10

/*******************************************************************************
// PWM Settings (Frequency und range)
//------------------------------------------------------------------------------
// period      = range (max = 0xFFFF => 16bit)
// Basic freq. = (APB2=8MHz) => TIM_CLK=8MHz
// period      : 0 to 0xFFFF
// prescaler   : 0 to 0xFFFF
//
// PWM-Frq     = TIM_CLK/(period+1)/(prescaler+1)
*******************************************************************************/
#define PWM_TIM_PERIODE             0xFF // period   (0xFF => 8bit)
#define PWM_TIM_PRESCALE            0xFF // prescaler (256 => 122Hz)

//--------------------------------------------------------------
// PWM Setting (Polarity)
//
// Hi => Hi-Impuls
// Lo => Lo-Impuls
//--------------------------------------------------------------
//#define  PWM_TIM_POLARITY           TIM_OCPolarity_High
#define PWM_TIM_POLARITY            TIM_OCPolarity_Low

#define PWM_TIMER                   TIM15

/* Private macro -------------------------------------------------------------*/
#define LATCH_HIGH                  GPIO_SetBits(LED_SPI_SS_PIN_PORT, LED_SPI_SS_PIN)
#define LATCH_LOW					GPIO_ResetBits(LED_SPI_SS_PIN_PORT, LED_SPI_SS_PIN)

/* Private typedef -----------------------------------------------------------*/
typedef enum rgb_colour {
    RED     = 0,
    GREEN   = 1,
    BLUE    = 2
}rgb_colour_t;


struct led_rgb leds[LED_COUNT];

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
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
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
  * @brief      Timer config for led driver serial register - send data regulary.
  *             Timing: 40 Hz
  * @param      None.
  * @retval     None.
  *****************************************************************************/
static void led_driver_timer_config(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    // Clock enable
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM17, ENABLE);

    /* Time base configuration */
    TIM_TimeBaseStructure.TIM_Period = 200 - 1;
    TIM_TimeBaseStructure.TIM_Prescaler = 1000 - 1;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(LED_TIMER, &TIM_TimeBaseStructure);

    TIM_ARRPreloadConfig(LED_TIMER, ENABLE);
    /* TIM Interrupts enable */
    TIM_ITConfig(LED_TIMER, TIM_IT_Update, ENABLE);

    /* TIM enable counter */
    TIM_Cmd(LED_TIMER, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = TIM17_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority = 0x04;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/*******************************************************************************
  * @function   led_driver_save_colour
  * @brief      Save colour of LED specified in parameters to be displayed in next cycle.
  * @param      colour: LED colour (RGB range).
  * @param      led_index: position of LED (0..11) or index >=12 -> all LED (default)
  * @retval     None.
  *****************************************************************************/
void led_driver_save_colour(const uint32_t colour, const uint8_t led_index)
{
    uint8_t idx;
    struct led_rgb *rgb_leds = leds;

    if (led_index >= LED_COUNT)
    {
        for (idx = 0; idx < LED_COUNT; idx++, rgb_leds++)
        {
            rgb_leds->led_rgb_st.red = colour >> 16;
            rgb_leds->led_rgb_st.green = (colour >> 8) & 0xFF;
            rgb_leds->led_rgb_st.blue = colour & 0xFF;
        }
    }
    else
    {
        rgb_leds += led_index;
        rgb_leds->led_rgb_st.red = colour >> 16;
        rgb_leds->led_rgb_st.green = (colour >> 8) & 0xFF;
        rgb_leds->led_rgb_st.blue = colour & 0xFF;
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
    //wait for flag
    while(SPI_I2S_GetFlagStatus(LED_SPI, SPI_I2S_FLAG_BSY));
}

/*******************************************************************************
  * @function   led_driver_prepare_data
  * @brief      Prepare data to be sent to LED driver.
  * @param      colour: RED, GREEN or BLUE.
  * @param      led_index: position of LED (0..11) or index >=12 -> all LED (default)
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
                if (rgb_leds->led_status == LED_ENABLE)
                {
                    if (rgb_leds->led_rgb_st.red > current_colour_level)
                    {
                        data |= 1 << (2 + idx); //shift by 2 - due to the HW connection
                    }
                }
            }
        } break;

        case GREEN:
        {
            for (idx = 0; idx < LED_COUNT; idx++, rgb_leds++)
            {
                if (rgb_leds->led_status == LED_ENABLE)
                {
                    if (rgb_leds->led_rgb_st.green > current_colour_level)
                    {
                        data |= 1 << (2 + idx);
                    }
                }
            }
        } break;

        case BLUE:
        {
            for (idx = 0; idx < LED_COUNT; idx++, rgb_leds++)
            {
                if (rgb_leds->led_status == LED_ENABLE)
                {
                    if (rgb_leds->led_rgb_st.blue > current_colour_level)
                    {
                        data |= 1 << (2 + idx);
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
    led_driver_send_data16b(~data);

    data = led_driver_prepare_data(GREEN, level << COLOUR_DECIMATION);
    led_driver_send_data16b(~data);

    data = led_driver_prepare_data(BLUE, level << COLOUR_DECIMATION);
    led_driver_send_data16b(~data);

    //latch enable pulse
    LATCH_HIGH;
    //delay(1); //TODO: some longer delay necessary?
    __NOP(); // or NO OPERATION is sufficient?
    LATCH_LOW;

    level++;

    if (level >= COLOUR_LEVELS)//restart cycle - all levels were sent to driver
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

    // Clock Enable
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

    // Clock enable
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

    // Timer enable
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
  * @brief      Enable all LED.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
static void led_driver_init_led(void)
{
    uint8_t idx;
    struct led_rgb *rgb_leds = leds;

    for (idx = 0; idx < LED_COUNT; idx++, rgb_leds++)
    {
        rgb_leds->led_status = LED_ENABLE;
    }
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

    led_driver_save_colour(0xFFFFFF, LED_COUNT); //all LED colour set to white
    led_driver_save_colour(0, 0); // except of the first led - it doesnt shine
    led_driver_pwm_set_brightness(100);//100% brightness after reset
    led_driver_init_led();

    led_driver_timer_config();
    led_driver_pwm_config();
}

/*******************************************************************************
  * @function   led_driver_pwm_set_brightness
  * @brief      Set PWM value.
  * @param      procent_val: PWM value in [%].
  * @retval     None.
  *****************************************************************************/
void led_driver_pwm_set_brightness(const uint16_t procent_val)
{
    uint16_t counter_val;

    counter_val = procent_val * PWM_TIM_PERIODE / 100;

    if(counter_val > PWM_TIM_PERIODE)
        counter_val = PWM_TIM_PERIODE;

    PWM_TIMER->CCR2 = counter_val;
}

/*******************************************************************************
  * @function   led_driver_step_brightness
  * @brief      Decrease LED brightness by 10% each step (each function call).
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void led_driver_step_brightness(void)
{
    static uint16_t led_brightness_step = 1;

    if (led_brightness_step > MAX_LED_BRIGHT_STEP)
        led_brightness_step = 0;

    led_driver_pwm_set_brightness(100 - (led_brightness_step * 10));
    led_brightness_step++;
}
