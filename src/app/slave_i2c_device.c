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
#define I2C_TIMING                      0x10800000 /* 100kHz for 48MHz system clock */

#define I2C_GPIO_CLOCK                  RCC_AHBPeriph_GPIOF
#define I2C_PERIPH_NAME                 I2C2
#define I2C_PERIPH_CLOCK                RCC_APB1Periph_I2C2
#define I2C_DATA_PIN                    GPIO_Pin_7 /* I2C2_SDA - GPIOF */
#define I2C_CLK_PIN                     GPIO_Pin_6 /* I2C2_SCL - GPIOF */
#define I2C_GPIO_PORT                   GPIOF

#define I2C_SLAVE_ADDRESS               0x55  /* address in linux: 0x2A */

#define CMD_INDEX                       0
#define ONE_BYTE_EXPECTED               1
#define TWO_BYTES_EXPECTED              2

enum i2c_commands {
    CMD_GET_STATUS_WORD                 = 0x01, /* slave sends back status word */
    CMD_GENERAL_CONTROL                 = 0x02,
    CMD_LED_MODE                        = 0x03, /* default/user */
    CMD_LED_STATE                       = 0x04, /* LED on/off */
    CMD_LED_COLOUR_PART1                = 0x05, /* LED number + RED */
    CMD_LED_COLOUR_PART2                = 0x06, /* GREEN + BLUE */
    CMD_LED_BRIGHTNESS                  = 0x07,
    CMD_USER_VOLTAGE                    = 0x08,
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

    I2C_SlaveByteControlCmd(I2C_PERIPH_NAME, ENABLE);
    I2C_ReloadCmd(I2C_PERIPH_NAME, ENABLE);

    /* Address match and receive interrupt */
    I2C_ITConfig(I2C_PERIPH_NAME, I2C_IT_ADDRI | I2C_IT_TCI | I2C_IT_STOPI, ENABLE);

    /* I2C Peripheral Enable */
    I2C_Cmd(I2C_PERIPH_NAME, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = I2C2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority = 0x01;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
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

    /* clear buffers */
    for (i = 0; i < MAX_RX_BUFFER_SIZE; i++)
    {
        i2c_state->rx_buf[i] = 0;
    }

    for (i = 0; i < MAX_TX_BUFFER_SIZE; i++)
    {
        i2c_state->tx_buf[i] = 0;
    }

    i2c_state->rx_data_ctr = 0;
    i2c_state->tx_data_ctr = 0;

    DBG("clear buffers\r\n");
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
}

/*******************************************************************************
  * @function   slave_i2c_check_control_byte
  * @brief      Decodes a control byte and perform suitable reaction.
  * @param      control_byte: control byte sent from master (CPU)
  * @param      state: pointer to next step (if necessary)
  * @retval     None.
  *****************************************************************************/
static void slave_i2c_check_control_byte(uint8_t control_byte, slave_i2c_states_t *state)
{
    struct st_i2c_status *i2c_control = &i2c_status;
    struct button_def *button = &button_front;
    *state = SLAVE_I2C_OK;
    error_type_t pwr_error = NO_ERROR;

    if (control_byte & LIGHT_RST_MASK)
    {
        *state = SLAVE_I2C_LIGHT_RST;
        /* set CFG_CTRL pin to high state ASAP */
        GPIO_SetBits(CFG_CTRL_PIN_PORT, CFG_CTRL_PIN);
        GPIO_ResetBits(MANRES_PIN_PORT, MANRES_PIN);
        return;
    }

    if (control_byte & HARD_RST_MASK)
    {
        *state = SLAVE_I2C_HARD_RST;
        return;
    }

    if (control_byte & FACTORY_RST_MASK)
    {
        *state = SLAVE_I2C_FACTORY_RST;
        return;
    }

    if (control_byte & SFP_DIS_MASK)
    {
        wan_sfp_set_tx_status(DISABLE);
    }
    else
    {
        wan_sfp_set_tx_status(ENABLE);
    }

    if(wan_sfp_get_tx_status())
    {
        i2c_control->status_word |= SFP_DIS_STSBIT;
    }
    else
    {
        i2c_control->status_word &= (~SFP_DIS_STSBIT);
    }

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
        pwr_error = power_control_start_regulator(REG_4V5);

        if (pwr_error == NO_ERROR)
            i2c_control->status_word |= ENABLE_4V5_STSBIT;
        else
            *state = SLAVE_I2C_PWR4V5_ERROR;
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
  * @function   slave_i2c_handler
  * @brief      Interrupt handler for I2C communication.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void slave_i2c_handler(void)
{
    struct st_i2c_status *i2c_state = &i2c_status;
    static uint16_t direction;
    static uint16_t nack;

    /* address match interrupt */
    if(I2C_GetITStatus(I2C_PERIPH_NAME, I2C_IT_ADDR) == SET)
    {
        /* Clear IT pending bit */
        I2C_ClearITPendingBit(I2C_PERIPH_NAME, I2C_IT_ADDR);

        /* Check if transfer direction is read (slave transmitter) */
        if ((I2C_PERIPH_NAME->ISR & I2C_ISR_DIR) == I2C_ISR_DIR)
        {
            direction = I2C_Direction_Transmitter;
            DBG("S.TX\r\n");
        }
        else
        {
            direction = I2C_Direction_Receiver;
            I2C_NumberOfBytesConfig(I2C_PERIPH_NAME, ONE_BYTE_EXPECTED);
            DBG("S.RX\r\n");
        }

        I2C_AcknowledgeConfig(I2C_PERIPH_NAME, ENABLE);
    }
    /* transmit interrupt */
    else if (I2C_GetITStatus(I2C_PERIPH_NAME, I2C_IT_TXIS) == SET)
    {
        I2C_SendData(I2C_PERIPH_NAME, i2c_state->tx_buf[i2c_state->tx_data_ctr++]);

        if (i2c_state->tx_data_ctr >= MAX_TX_BUFFER_SIZE)
        {
            i2c_state->tx_data_ctr = 0;
        }
        DBG("send\r\n");
    }
    /* transfer complet interrupt */
    else if (I2C_GetITStatus(I2C_PERIPH_NAME, I2C_IT_TCR) == SET)
    {
        if(direction == I2C_Direction_Receiver)
        {
            i2c_state->rx_buf[i2c_state->rx_data_ctr++] = I2C_ReceiveData(I2C_PERIPH_NAME);

            if (i2c_state->rx_data_ctr == 1)
            {
                /* check if the command exists and send ACK */
                switch(i2c_state->rx_buf[CMD_INDEX])
                {
                    case CMD_GENERAL_CONTROL:
                    case CMD_LED_MODE:
                    case CMD_LED_STATE:
                    case CMD_LED_BRIGHTNESS:
                    case CMD_LED_COLOUR_PART1:
                    case CMD_LED_COLOUR_PART2:
                    case CMD_USER_VOLTAGE:
                    {
                        I2C_AcknowledgeConfig(I2C_PERIPH_NAME, ENABLE);
                        I2C_NumberOfBytesConfig(I2C_PERIPH_NAME, ONE_BYTE_EXPECTED);
                        nack = 0;
                        DBG("ACK\r\n");
                    }break;

                    case CMD_GET_STATUS_WORD:
                    {
                        /* prepare data to be sent to the master */
                        i2c_state->tx_buf[0] = i2c_state->status_word & 0x00FF;
                        i2c_state->tx_buf[1] = (i2c_state->status_word & 0xFF00) >> 8;

                        I2C_ITConfig(I2C_PERIPH_NAME, I2C_IT_TXI , ENABLE);
                        I2C_AcknowledgeConfig(I2C_PERIPH_NAME, ENABLE);
                        I2C_NumberOfBytesConfig(I2C_PERIPH_NAME, TWO_BYTES_EXPECTED);
                        nack = 0;
                        DBG("ACK\r\n");
                    }break;

                    default: /* command doesnt exist - send NACK */
                    {
                        I2C_AcknowledgeConfig(I2C_PERIPH_NAME, DISABLE);
                        nack = 1;
                        I2C_NumberOfBytesConfig(I2C_PERIPH_NAME, ONE_BYTE_EXPECTED);
                        DBG("NACK\r\n");
                    } break;
                }
            }
            else if (i2c_state->rx_data_ctr > 1) /* send ACK after every byte received */
            {
                I2C_AcknowledgeConfig(I2C_PERIPH_NAME, ENABLE);
                I2C_NumberOfBytesConfig(I2C_PERIPH_NAME, ONE_BYTE_EXPECTED);

                if (i2c_state->rx_data_ctr >= MAX_RX_BUFFER_SIZE)
                {
                    i2c_state->rx_data_ctr = 0;
                }
                DBG("ACKever\r\n");
            }

            //DBG("RX1: \r\n");
           // DBG((const char*)(i2c_state->rx_buf));
           // DBG("\r\n");
        }
        else /* I2C_Direction_Transmitter */
        {
            I2C_AcknowledgeConfig(I2C_PERIPH_NAME, DISABLE);
            i2c_state->data_tx_complete = 1;
            DBG("NACKtx\r\n");
        }
    }

    /* stop flag */
    else if (I2C_GetITStatus(I2C_PERIPH_NAME, I2C_IT_STOPF) == SET)
    {
        I2C_ClearITPendingBit(I2C_PERIPH_NAME, I2C_IT_STOPF);

        if (direction == I2C_Direction_Receiver)
        {
            if (!nack)
            {
                /* disable I2C interrupt */
                I2C_ITConfig(I2C_PERIPH_NAME, I2C_IT_ADDRI | I2C_IT_TCI | I2C_IT_STOPI, DISABLE);
                i2c_state->data_rx_complete = 1;
            }
            else
            {
                i2c_state->data_rx_complete = 0;
            }
        }
        else if (i2c_state->data_tx_complete) /* data have been sent to master */
        {
            i2c_state->data_tx_complete = 0;
            i2c_state->tx_data_ctr = 0;
            /* decrease button counter by the value has been sent */
            button_counter_decrease((i2c_state->status_word & BUTTON_COUNTER_VALBITS) >> 13);
        }

        DBG("STOP\r\n");
        i2c_state->rx_data_ctr = 0;

        direction = I2C_Direction_Receiver;
        I2C_NumberOfBytesConfig(I2C_PERIPH_NAME, ONE_BYTE_EXPECTED);
    }
}

/*******************************************************************************
  * @function   slave_i2c_process_data
  * @brief      Process incoming/outcoming data.
  * @param      None.
  * @retval     Next reaction (if necessary).
  *****************************************************************************/
slave_i2c_states_t slave_i2c_process_data(void)
{
    struct st_i2c_status *i2c_state = &i2c_status;
    static uint8_t led_index, led_colour_part_one_complete;
    static uint32_t colour;
    slave_i2c_states_t state = SLAVE_I2C_OK;

    if (i2c_state->data_rx_complete) /* slave RX (master sends data) */
    {
        //DBG("\r\nRX data: ");
       // DBG((const char*)i2c_state->rx_buf);
       // DBG("\r\n");

        /* clear flag */
        i2c_state->data_rx_complete = 0;

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

                led_colour_part_one_complete = 1;

                DBG("set LED colour - LED index : ")
                DBG((const char*)&led_index);
                DBG("\r\nRED: ");
                DBG((const char*)(i2c_state->rx_buf + 2));
                DBG("\r\n");
            } break;

            case CMD_LED_COLOUR_PART2:
            {
                colour |= (i2c_state->rx_buf[1] << 8) | i2c_state->rx_buf[2];

                if(led_colour_part_one_complete)
                {
                    led_driver_set_colour(led_index, colour);
                    led_colour_part_one_complete = 0;
                    colour = 0;
                }

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

            case CMD_USER_VOLTAGE:
            {
                power_control_set_voltage(i2c_state->rx_buf[1]);

                DBG("user voltage: ");
                DBG((const char*)(i2c_state->rx_buf + 1));
                DBG("\r\n");
            } break;

            default:
            {
                I2C_SoftwareResetCmd(I2C_PERIPH_NAME);

                DBG("unexpected command: ");
                DBG((const char*)i2c_state->rx_buf);
                DBG("\r\n");
            } break;
        }

        slave_i2c_clear_buffers();
        /* enable I2C interrupt again */
        I2C_ITConfig(I2C_PERIPH_NAME, I2C_IT_ADDRI | I2C_IT_TCI | I2C_IT_STOPI, ENABLE);
    }

    return state;
}
