/**
 ******************************************************************************
 * @file    slave_i2c_device.c
 * @author  CZ.NIC, z.s.p.o.
 * @date    18-August-2015
 * @brief   Driver for IC2 communication with master device (main CPU).
 ******************************************************************************
 ******************************************************************************
 **/
/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_conf.h"
#include "slave_i2c_device.h"
#include "debug_serial.h"
#include "led_driver.h"
#include "wan_lan_pci_status.h"
#include "power_control.h"
#include "delay.h"
#include "debounce.h"

#define I2C_SDA_SOURCE                  GPIO_PinSource7
#define I2C_SCL_SOURCE                  GPIO_PinSource6

#define I2C_ALTERNATE_FUNCTION          GPIO_AF_1
#define I2C_TIMING                      0x10800000 //100kHz for 48MHz system clock

#define I2C_GPIO_CLOCK                  RCC_AHBPeriph_GPIOF
#define I2C_PERIPH_NAME                 I2C2
#define I2C_PERIPH_CLOCK                RCC_APB1Periph_I2C2
#define I2C_DATA_PIN                    GPIO_Pin_7 // I2C2_SDA - GPIOF
#define I2C_CLK_PIN                     GPIO_Pin_6 // I2C2_SCL - GPIOF
#define I2C_GPIO_PORT                   GPIOF

#define I2C_SLAVE_ADDRESS               0x55 //address in linux: 0x2A

#define CMD_INDEX                       0
#define STATUS_WORD_LENGHT              2 // bytes

enum i2c_commands {
    CMD_GET_STATUS_WORD                 = 0x01, // slave sends back status word
    CMD_GENERAL_CONTROL                 = 0x02,
    CMD_LED_MODE                        = 0x03, // default/user
    CMD_LED_STATE                       = 0x04, // LED on/off
    CMD_LED_COLOUR_PART1                = 0x05, // LED number + RED
    CMD_LED_COLOUR_PART2                = 0x06, // GREEN + BLUE
    CMD_LED_BRIGHTNESS                  = 0x07,
};

enum i2_control_byte_mask {
    LIGHT_RST_MASK                      = 0x01,
    HARD_RST_MASK                       = 0x02,
    FACTORY_RST_MASK                    = 0x04,
    SFP_DIS_MASK                        = 0x08,
    USB30_PWRON_MASK                    = 0x10,
    USB31_PWRON_MASK                    = 0x20,
    ENABLE_4V5_MASK                     = 0x40,
    BUTTON_MODE_MASK                    = 0x80,
};

struct st_i2c_status i2c_status;

/*******************************************************************************
  * @function   slave_i2c_config
  * @brief      Configuration of pins for I2C.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
static void slave_i2c_io_config(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;

    /* I2C Peripheral Disable */
    RCC_APB1PeriphClockCmd(I2C_PERIPH_CLOCK, DISABLE);

    /* I2C Periph clock enable */
    RCC_APB1PeriphClockCmd(I2C_PERIPH_CLOCK, ENABLE);

    RCC_AHBPeriphClockCmd(I2C_GPIO_CLOCK, ENABLE);

    /* Connect PXx to I2C_SCL */
    GPIO_PinAFConfig(I2C_GPIO_PORT, I2C_SCL_SOURCE, I2C_ALTERNATE_FUNCTION);

    /* Connect PXx to I2C_SDA */
    GPIO_PinAFConfig(I2C_GPIO_PORT, I2C_SDA_SOURCE, I2C_ALTERNATE_FUNCTION);

    /* Configure I2C pins: SCL */
    GPIO_InitStructure.GPIO_Pin = I2C_CLK_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_Init(I2C_GPIO_PORT, &GPIO_InitStructure);

    /* Configure I2C pins: SDA */
    GPIO_InitStructure.GPIO_Pin = I2C_DATA_PIN;
    GPIO_Init(I2C_GPIO_PORT, &GPIO_InitStructure);
}

/*******************************************************************************
  * @function   slave_i2c_periph_config
  * @brief      Configuration of I2C peripheral as a slave.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
static void slave_i2c_periph_config(void)
{
    I2C_InitTypeDef  I2C_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    I2C_DeInit(I2C_PERIPH_NAME);
    I2C_Cmd(I2C_PERIPH_NAME, DISABLE);

    /* I2C configuration */
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_AnalogFilter = I2C_AnalogFilter_Enable;
    I2C_InitStructure.I2C_DigitalFilter = 0x00;
    I2C_InitStructure.I2C_OwnAddress1 = I2C_SLAVE_ADDRESS;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_Timing = I2C_TIMING;

    /* Apply I2C configuration after enabling it */
    I2C_Init(I2C_PERIPH_NAME, &I2C_InitStructure);

    /* Address match and receive interrupt */
    I2C_ITConfig(I2C_PERIPH_NAME, I2C_IT_ADDRI | I2C_IT_RXI | I2C_IT_STOPI, ENABLE);

    /* I2C Peripheral Enable */
    I2C_Cmd(I2C_PERIPH_NAME, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = I2C2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority = 0x01;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/*******************************************************************************
  * @function   slave_i2c_timeout_config
  * @brief      Timer configuration for I2C timeout recovery.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
static void slave_i2c_timeout_config(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    // Clock enable
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

    /* Time base configuration - 1sec interrupt */
    TIM_TimeBaseStructure.TIM_Period = 8000 - 1;
    TIM_TimeBaseStructure.TIM_Prescaler = 6000 - 1;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(I2C_TIMEOUT_TIMER, &TIM_TimeBaseStructure);

    TIM_ARRPreloadConfig(I2C_TIMEOUT_TIMER, ENABLE);
    /* TIM Interrupts enable */
    TIM_ITConfig(I2C_TIMEOUT_TIMER, TIM_IT_Update, ENABLE);

    /* TIM enable counter */
    //TIM_Cmd(TIMEOUT_I2C_TIMER, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = TIM1_BRK_UP_TRG_COM_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority = 0x05;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/*******************************************************************************
  * @function   slave_i2c_timeout_enable
  * @brief      Start timeout measuring.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
static void slave_i2c_timeout_enable(void)
{
    /* TIM enable counter */
    TIM_Cmd(I2C_TIMEOUT_TIMER, ENABLE);
}

/*******************************************************************************
  * @function   slave_i2c_timeout_disable
  * @brief      Stop timeout measuring.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
static void slave_i2c_timeout_disable(void)
{
    /* TIM disable counter */
    TIM_Cmd(I2C_TIMEOUT_TIMER, DISABLE);
    I2C_TIMEOUT_TIMER->CNT = 0;
}

/*******************************************************************************
  * @function   slave_i2c_timeout_handler
  * @brief      Timeout occures -> reset I2C peripheral.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void slave_i2c_timeout_handler(void)
{
    slave_i2c_periph_config();
    slave_i2c_timeout_disable();
}

/*******************************************************************************
  * @function   slave_i2c_config
  * @brief      Configuration of I2C peripheral and its timeout.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void slave_i2c_config(void)
{
    slave_i2c_io_config();
    slave_i2c_periph_config();
    slave_i2c_timeout_config();
}

/*******************************************************************************
  * @function   slave_i2c_check_control_byte
  * @brief      Decodes a control byte and perform suitable reaction.
  * @param      control_byte: control byte sent from master (CPU)
  * @param      state: pointer to next step (if necessary)
  * @retval     None.
  *****************************************************************************/
static void slave_i2c_check_control_byte(uint8_t control_byte, ret_value_t *state)
{
    struct st_i2c_status *i2c_control = &i2c_status;
    struct button_def *button = &button_front;
    *state = OK;

    if (control_byte & LIGHT_RST_MASK)
    {
        *state = GO_TO_LIGHT_RESET;
        return;
    }

    if (control_byte & HARD_RST_MASK)
    {
        *state = GO_TO_HARD_RESET;
        return;
    }

    if (control_byte & FACTORY_RST_MASK)
    {
        *state = GO_TO_FACTORY_RESET;
        return;
    }

    if (control_byte & SFP_DIS_MASK)
        wan_sfp_set_tx_status(DISABLE);
    else
        wan_sfp_set_tx_status(ENABLE);

    if(wan_sfp_get_tx_status())
        i2c_control->status_word |= SFP_DIS_STSBIT;
    else
        i2c_control->status_word &= (~SFP_DIS_STSBIT);

    if (control_byte & USB30_PWRON_MASK)
    {
        power_control_usb(USB3_PORT0, USB_ON);
        i2c_control->status_word |= USB30_PWRON_STSBIT;
    }
    else
    {
        power_control_usb(USB3_PORT0, USB_OFF);
        i2c_control->status_word &= (~USB30_PWRON_STSBIT);
    }

    if (control_byte & USB31_PWRON_MASK)
    {
        power_control_usb(USB3_PORT1, USB_ON);
        i2c_control->status_word |= USB31_PWRON_STSBIT;
    }
    else
    {
        power_control_usb(USB3_PORT1, USB_OFF);
        i2c_control->status_word &= (~USB31_PWRON_STSBIT);
    }

    if (control_byte & ENABLE_4V5_MASK)
    {
        power_control_start_regulator(REG_4V5); //TODO: read return value
        i2c_control->status_word |= ENABLE_4V5_STSBIT;
    }
    else
    {
        GPIO_ResetBits(ENABLE_4V5_PIN_PORT, ENABLE_4V5_PIN);
        i2c_control->status_word &= (~ENABLE_4V5_STSBIT);
    }

    if (control_byte & BUTTON_MODE_MASK)
    {
       button->button_mode = BUTTON_USER;
       i2c_control->status_word |= BUTTON_MODE_STSBIT;
    }
    else
    {
       button->button_mode = BUTTON_DEFAULT;
       button->button_pressed_counter = 0;
       i2c_control->status_word &= (~BUTTON_MODE_STSBIT);
    }
}

/*******************************************************************************
  * @function   slave_i2c_clear_buffers
  * @brief      Clear RX and TX buffers.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
static void slave_i2c_clear_buffers(void)
{
    uint8_t i;
    struct st_i2c_status *i2c_state = &i2c_status;

    // clear buffers
    for (i = 0; i < MAX_RX_BUFFER_SIZE; i++)
    {
        i2c_state->rx_buf[i] = 0;
    }

    for (i = 0; i < MAX_TX_BUFFER_SIZE; i++)
    {
        i2c_state->tx_buf[i] = 0;
    }
    DBG("clear buffers\r\n");
}

/*******************************************************************************
  * @function   slave_i2c_handler
  * @brief      Interrupt handler for I2C communication.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void slave_i2c_handler(void)
{
    struct st_i2c_status *i2c_state = &i2c_status;

    /* Test on I2C Address match interrupt */
    if(I2C_GetITStatus(I2C_PERIPH_NAME, I2C_IT_ADDR) == SET)
    {
        /* Clear IT pending bit */
        I2C_ClearITPendingBit(I2C_PERIPH_NAME, I2C_IT_ADDR);

        slave_i2c_timeout_enable();
        DBG("address\r\n");
    }

    /* transmit data */
    if (I2C_GetITStatus(I2C_PERIPH_NAME, I2C_IT_TXIS) == SET)
    {
        if (i2c_state->tx_data_ctr >= STATUS_WORD_LENGHT)
            I2C_ITConfig(I2C_PERIPH_NAME, I2C_IT_TXI, DISABLE); // disable TX interrupt
        else
            I2C_SendData(I2C_PERIPH_NAME, i2c_state->tx_buf[i2c_state->tx_data_ctr++]);
    }

    /* receive data */
    if (I2C_GetITStatus(I2C_PERIPH_NAME, I2C_IT_RXNE) == SET)
    {
        i2c_state->rx_buf[i2c_state->rx_data_ctr++] = I2C_ReceiveData(I2C_PERIPH_NAME);

        // first byte received
        if (i2c_state->rx_buf[CMD_INDEX] == CMD_GET_STATUS_WORD)
        {
            i2c_state->data_tx_complete = 1;
        }
    }

    /* stop detection */
    if (I2C_GetITStatus(I2C_PERIPH_NAME, I2C_IT_STOPF) == SET)
    {
        I2C_ClearITPendingBit(I2C_PERIPH_NAME, I2C_IT_STOPF);

        if (i2c_state->data_tx_complete) // data have been sent to master
            i2c_state->data_tx_complete = 0;
        else                             // data have been received from master
            i2c_state->data_rx_complete = 1;

        // clear counters
        i2c_state->rx_data_ctr = 0;
        i2c_state->tx_data_ctr = 0;

        slave_i2c_timeout_disable();
    }
}

/*******************************************************************************
  * @function   slave_i2c_process_data
  * @brief      Process incoming/outcoming data.
  * @param      None.
  * @retval     Next reaction (if necessary).
  *****************************************************************************/
ret_value_t slave_i2c_process_data(void)
{
    struct st_i2c_status *i2c_state = &i2c_status;
    struct button_def *button = &button_front;
    static uint8_t led_index;
    static uint32_t colour;
    ret_value_t state = OK;

    if (i2c_state->data_tx_complete) /* slave TX (master expects data) */
    {
        DBG("\r\nTX command: ");
        DBG((const char*)i2c_state->rx_buf);
        DBG("\r\n");

        /* prepare data to be sent to the master */
        i2c_state->tx_buf[0] = i2c_state->status_word & 0x00FF;
        i2c_state->tx_buf[1] = (i2c_state->status_word & 0xFF00) >> 8;

        /* decrease button counter by the value is going to be sent */
        button->button_pressed_counter -= (i2c_state->status_word & BUTTON_COUNTER_VALBITS) >> 13;

        if (button->button_pressed_counter <= 0) /* limitation */
            button->button_pressed_counter = 0;

        I2C_ITConfig(I2C_PERIPH_NAME, I2C_IT_TXI , ENABLE);

        DBG("status word: ");
        DBG((const char*)i2c_state->tx_buf);
        DBG("\r\n");
    }

    if (i2c_state->data_rx_complete) /* slave RX (master sends data) */
    {
        DBG("\r\nRX data: ");
        DBG((const char*)i2c_state->rx_buf);
        DBG("\r\n");

        switch(i2c_state->rx_buf[CMD_INDEX])
        {
            case CMD_GENERAL_CONTROL:
            {
                slave_i2c_check_control_byte(i2c_state->rx_buf[1], &state);

                DBG("set control byte: ");
                DBG((const char*)i2c_state->rx_buf + 1);
                DBG("\r\n");
            } break;

            case CMD_LED_MODE:
            {
                led_driver_set_led_mode(i2c_state->rx_buf[1] & 0x0F,
                                        (i2c_state->rx_buf[1] & 0x10) >> 4);

                DBG("set LED mode - LED index : ");
                DBG((const char*)(i2c_state->rx_buf[1] & 0x0F));
                DBG("\r\nLED mode: ");
                DBG((const char*)((i2c_state->rx_buf[1] & 0x0F) >> 4));
                DBG("\r\n");
            } break;

            case CMD_LED_STATE:
            {
                led_driver_set_led_state(i2c_state->rx_buf[1] & 0x0F,
                                        (i2c_state->rx_buf[1] & 0x10) >> 4);

                DBG("set LED state - LED index : ");
                DBG((const char*)(i2c_state->rx_buf[1] & 0x0F));
                DBG("\r\nLED state: ");
                DBG((const char*)((i2c_state->rx_buf[1] & 0x0F) >> 4));
                DBG("\r\n");
            } break;

            case CMD_LED_COLOUR_PART1:
            {
                led_index = i2c_state->rx_buf[1] & 0x0F;
                colour = i2c_state->rx_buf[2] << 16;

                DBG("set LED colour - LED index : ")
                DBG((const char*)&led_index);
                DBG("\r\nRED: ");
                DBG((const char*)(i2c_state->rx_buf + 2));
                DBG("\r\n");
            } break;

            case CMD_LED_COLOUR_PART2:
            {
                colour |= (i2c_state->rx_buf[1] << 8) | i2c_state->rx_buf[2];

                led_driver_set_colour(led_index, colour);

                DBG("\r\nGREEN: ");
                DBG((const char*)(i2c_state->rx_buf + 1));
                DBG("\r\nBLUE: ");
                DBG((const char*)(i2c_state->rx_buf + 2));
                DBG("\r\n");
            } break;

            case CMD_LED_BRIGHTNESS:
            {
                led_driver_pwm_set_brightness(i2c_state->rx_buf[1]);

                DBG("brightness: ");
                DBG((const char*)(i2c_state->rx_buf + 1));
                DBG("\r\n");
            } break;

            default: /* unexpected command received */
            {
                slave_i2c_clear_buffers();
                // clear counters
                i2c_state->rx_data_ctr = 0;
                i2c_state->tx_data_ctr = 0;
                slave_i2c_config();

                DBG("unexpected command: ");
                DBG((const char*)i2c_state->rx_buf);
                DBG("\r\n");
            } break;
        }

        // clear flag
        i2c_state->data_rx_complete = 0;
    }

    return state;
}
