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

static const uint8_t version[] = VERSION;

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
//#define ONE_BYTE_EXPECTED               1
//#define TWO_BYTES_EXPECTED              2
//#define TWENTY_BYTES_EXPECTED           20

enum i2c_commands {
    CMD_GET_STATUS_WORD                 = 0x01, /* slave sends status word back */
    CMD_GENERAL_CONTROL                 = 0x02,
    CMD_LED_MODE                        = 0x03, /* default/user */
    CMD_LED_STATE                       = 0x04, /* LED on/off */
    CMD_LED_COLOUR                      = 0x05, /* LED number + RED + GREEN + BLUE */
    CMD_USER_VOLTAGE                    = 0x06,
    CMD_SET_BRIGHTNESS                  = 0x07,
    CMD_GET_BRIGHTNESS                  = 0x08,
    CMD_GET_RESET                       = 0x09,
    CMD_GET_FW_VERSION                  = 0x0A,
    CMD_WATCHDOG                        = 0x0B
};

enum i2c_control_byte_mask {
    LIGHT_RST_MASK                      = 0x01,
    HARD_RST_MASK                       = 0x02,
    FACTORY_RST_MASK                    = 0x04,
    SFP_DIS_MASK                        = 0x08,
    USB30_PWRON_MASK                    = 0x10,
    USB31_PWRON_MASK                    = 0x20,
    ENABLE_4V5_MASK                     = 0x40,
    BUTTON_MODE_MASK                    = 0x80,
};

typedef enum i2c_responses {
    ACK_AND_WAIT_FOR_SECOND_START       = 0,
    ACK                                 = 1,
    NACK                                = 2
} i2c_response_t;

enum expected_bytes_in_cmd {
    ONE_BYTE_EXPECTED                   = 1,
    TWO_BYTES_EXPECTED                  = 2,
    FIVE_BYTES_EXPECTED                 = 5,
    TWENTY_BYTES_EXPECTED               = 20
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

    /* Address match, transfer complete, stop and transmit interrupt */
    I2C_ITConfig(I2C_PERIPH_NAME, I2C_IT_ADDRI | I2C_IT_TCI | I2C_IT_STOPI | I2C_IT_TXI, ENABLE);

    /* I2C Peripheral Enable */
    I2C_Cmd(I2C_PERIPH_NAME, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = I2C2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority = 0x01;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
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
        /* not used */
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
    static i2c_response_t response = NACK;
    struct led_rgb *led = leds;
    uint16_t idx;
    uint8_t led_index;
    uint32_t colour;

    __disable_irq();

    /* address match interrupt */
    if(I2C_GetITStatus(I2C_PERIPH_NAME, I2C_IT_ADDR) == SET)
    {
        /* default state */
        direction = I2C_Direction_Receiver;
        i2c_state->rx_data_ctr = 0;
        i2c_state->tx_data_ctr = 0;

        /* Clear IT pending bit */
        I2C_ClearITPendingBit(I2C_PERIPH_NAME, I2C_IT_ADDR);

        /* Check if transfer direction is read (slave transmitter) */
        if ((I2C_PERIPH_NAME->ISR & I2C_ISR_DIR) == I2C_ISR_DIR)
        {
            direction = I2C_Direction_Transmitter;
            //TODO: vynulovat zde TX counter ?
            DBG("S.TX\r\n");
        }
        else
        {
            direction = I2C_Direction_Receiver;
            I2C_NumberOfBytesConfig(I2C_PERIPH_NAME, ONE_BYTE_EXPECTED);
            DBG("S.RX\r\n");
        }
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
    /* transfer complet interrupt (TX and RX) */
    else if (I2C_GetITStatus(I2C_PERIPH_NAME, I2C_IT_TCR) == SET)
    {
        if(direction == I2C_Direction_Receiver)
        {
            /* if data are already processed */
           // if ((!i2c_state->data_rx_complete) && (!i2c_state->data_tx_complete))
           // {
                i2c_state->rx_buf[i2c_state->rx_data_ctr++] = I2C_ReceiveData(I2C_PERIPH_NAME);

                /* find out the requested command (register) */
               // if (i2c_state->rx_data_ctr == 1)
               // {
                    /* check if the command exists and send ACK */
                    switch(i2c_state->rx_buf[CMD_INDEX])
                    {
                        case CMD_GENERAL_CONTROL:
                        {
                            if((i2c_state->rx_data_ctr -1) == ONE_BYTE_EXPECTED)
                            {
                            //TODO: druhy parameter neni treba, jde to pres i2c_state
                                slave_i2c_check_control_byte(i2c_state->rx_buf[1], &i2c_state->state);
                            }
                            DBG("ACK\r\n");
                            I2C_AcknowledgeConfig(I2C_PERIPH_NAME, ENABLE);
                            /* release SCL line */
                            I2C_NumberOfBytesConfig(I2C_PERIPH_NAME, ONE_BYTE_EXPECTED);
                            response = ACK;
                        } break;

                        case CMD_LED_MODE:
                        {
                            if((i2c_state->rx_data_ctr -1) == ONE_BYTE_EXPECTED)
                            {
                                led_driver_set_led_mode(i2c_state->rx_buf[1] & 0x0F,
                                            (i2c_state->rx_buf[1] & 0x10) >> 4);

                                DBG("set LED mode - LED index : ");
                                DBG((const char*)(i2c_state->rx_buf[1] & 0x0F));
                                DBG("\r\nLED mode: ");
                                DBG((const char*)((i2c_state->rx_buf[1] & 0x0F) >> 4));
                                DBG("\r\n");
                            }
                            DBG("ACK\r\n");
                            I2C_AcknowledgeConfig(I2C_PERIPH_NAME, ENABLE);
                            /* release SCL line */
                            I2C_NumberOfBytesConfig(I2C_PERIPH_NAME, ONE_BYTE_EXPECTED);
                            response = ACK;
                        } break;

                        case CMD_LED_STATE:
                        {
                            if((i2c_state->rx_data_ctr -1) == ONE_BYTE_EXPECTED)
                            {
                                led_driver_set_led_state(i2c_state->rx_buf[1] & 0x0F,
                                        (i2c_state->rx_buf[1] & 0x10) >> 4);

                                DBG("set LED state - LED index : ");
                                DBG((const char*)(i2c_state->rx_buf[1] & 0x0F));
                                DBG("\r\nLED state: ");
                                DBG((const char*)((i2c_state->rx_buf[1] & 0x0F) >> 4));
                                DBG("\r\n");
                            }
                            DBG("ACK\r\n");
                            I2C_AcknowledgeConfig(I2C_PERIPH_NAME, ENABLE);
                            /* release SCL line */
                            I2C_NumberOfBytesConfig(I2C_PERIPH_NAME, ONE_BYTE_EXPECTED);
                            response = ACK;
                        } break;

                        case CMD_LED_COLOUR:
                        {
                            if((i2c_state->rx_data_ctr -1) == FIVE_BYTES_EXPECTED)
                            {
                                led_index = i2c_state->rx_buf[1] & 0x0F;
                                /* colour = Red + Green + Blue */
                                colour = (i2c_state->rx_buf[2] << 16) | (i2c_state->rx_buf[3] << 8) | i2c_state->rx_buf[4];

                                led_driver_set_colour(led_index, colour);

                                DBG("set LED colour - LED index : ")
                                DBG((const char*)&led_index);
                                DBG("\r\nRED: ");
                                DBG((const char*)(i2c_state->rx_buf + 2));
                                DBG("\r\n");
                            }
                            DBG("ACK\r\n");
                            I2C_AcknowledgeConfig(I2C_PERIPH_NAME, ENABLE);
                            /* release SCL line */
                            I2C_NumberOfBytesConfig(I2C_PERIPH_NAME, ONE_BYTE_EXPECTED);
                            response = ACK;
                        } break;

                        case CMD_SET_BRIGHTNESS:
                        {
                            if((i2c_state->rx_data_ctr -1) == ONE_BYTE_EXPECTED)
                            {
                                led_driver_pwm_set_brightness(i2c_state->rx_buf[1]);

                                DBG("brightness: ");
                                DBG((const char*)(i2c_state->rx_buf + 1));
                                DBG("\r\n");
                            }
                            DBG("ACK\r\n");
                            I2C_AcknowledgeConfig(I2C_PERIPH_NAME, ENABLE);
                            /* release SCL line */
                            I2C_NumberOfBytesConfig(I2C_PERIPH_NAME, ONE_BYTE_EXPECTED);
                            response = ACK;
                        } break;

                        case CMD_USER_VOLTAGE:
                        {
                            if((i2c_state->rx_data_ctr -1) == ONE_BYTE_EXPECTED)
                            {
                                power_control_set_voltage(i2c_state->rx_buf[1]);

                                DBG("user voltage: ");
                                DBG((const char*)(i2c_state->rx_buf + 1));
                                DBG("\r\n");
                            }
                            DBG("ACK\r\n");
                            I2C_AcknowledgeConfig(I2C_PERIPH_NAME, ENABLE);
                            /* release SCL line */
                            I2C_NumberOfBytesConfig(I2C_PERIPH_NAME, ONE_BYTE_EXPECTED);
                            response = ACK;
                        } break;

                        case CMD_WATCHDOG:
                        {
                            if((i2c_state->rx_data_ctr -1) == ONE_BYTE_EXPECTED)
                            {
                                watchdog_sts = i2c_state->rx_buf[1];

                                DBG("WDT: ");
                                DBG((const char*)(i2c_state->rx_buf + 1));
                                DBG("\r\n");
                            }
                            DBG("ACK\r\n");
                            I2C_AcknowledgeConfig(I2C_PERIPH_NAME, ENABLE);
                            /* release SCL line */
                            I2C_NumberOfBytesConfig(I2C_PERIPH_NAME, ONE_BYTE_EXPECTED);
                            response = ACK;
                        } break;

                        case CMD_GET_STATUS_WORD:
                        {
                            /* prepare data to be sent to the master */
                            i2c_state->tx_buf[0] = i2c_state->status_word & 0x00FF;
                            i2c_state->tx_buf[1] = (i2c_state->status_word & 0xFF00) >> 8;
                            DBG("STS\r\n");

                            I2C_AcknowledgeConfig(I2C_PERIPH_NAME, ENABLE);
                            I2C_NumberOfBytesConfig(I2C_PERIPH_NAME, TWO_BYTES_EXPECTED);
                            response = ACK_AND_WAIT_FOR_SECOND_START;
                        } break;

                        case CMD_GET_BRIGHTNESS:
                        {
                            i2c_state->tx_buf[0] = led->brightness;
                            DBG("brig\r\n");

                            I2C_AcknowledgeConfig(I2C_PERIPH_NAME, ENABLE);
                            I2C_NumberOfBytesConfig(I2C_PERIPH_NAME, ONE_BYTE_EXPECTED);

                            response = ACK_AND_WAIT_FOR_SECOND_START;
                        } break;

                        case CMD_GET_RESET:
                        {
                            i2c_state->tx_buf[0] = i2c_state->reset_type;
                            DBG("RST\r\n");

                            I2C_AcknowledgeConfig(I2C_PERIPH_NAME, ENABLE);
                            I2C_NumberOfBytesConfig(I2C_PERIPH_NAME, ONE_BYTE_EXPECTED);

                            response = ACK_AND_WAIT_FOR_SECOND_START;
                        } break;

                        case CMD_GET_FW_VERSION:
                        {
                            for (idx = 0; idx < MAX_TX_BUFFER_SIZE; idx++)
                            {
                                i2c_state->tx_buf[idx] = version[idx];
                            }
                            DBG("FW\r\n");

                            I2C_AcknowledgeConfig(I2C_PERIPH_NAME, ENABLE);
                            I2C_NumberOfBytesConfig(I2C_PERIPH_NAME, TWENTY_BYTES_EXPECTED);

                            response = ACK_AND_WAIT_FOR_SECOND_START;
                        } break;

                        default: /* command doesnt exist - send NACK */
                        {
                            DBG("NACK\r\n");
                            I2C_AcknowledgeConfig(I2C_PERIPH_NAME, DISABLE);
                            I2C_NumberOfBytesConfig(I2C_PERIPH_NAME, ONE_BYTE_EXPECTED);
                            response = NACK;
                        } break;
                    }
                //}
//                else if (i2c_state->rx_data_ctr > 1) /* send ACK after every byte received */
//                {
//                    DBG("ACKever\r\n");
//                    I2C_AcknowledgeConfig(I2C_PERIPH_NAME, ENABLE);
//                    I2C_NumberOfBytesConfig(I2C_PERIPH_NAME, ONE_BYTE_EXPECTED);

//                    if (i2c_state->rx_data_ctr >= MAX_RX_BUFFER_SIZE)
//                    {
//                        i2c_state->rx_data_ctr = 0;
//                    }
//                    response = ACK;
//                }
//            }
//            else /* else if data are not yet processed */
//            {
//                DBG("NOT\r\n");
//                I2C_AcknowledgeConfig(I2C_PERIPH_NAME, DISABLE);
//                I2C_NumberOfBytesConfig(I2C_PERIPH_NAME, ONE_BYTE_EXPECTED);
//                response = NACK;
//            }
        }
        else /* I2C_Direction_Transmitter */
        {
            DBG("ACKtx\r\n");
            I2C_NumberOfBytesConfig(I2C_PERIPH_NAME, ONE_BYTE_EXPECTED);
            i2c_state->data_tx_complete = 1;
        }
    }

    /* stop flag */
    else if (I2C_GetITStatus(I2C_PERIPH_NAME, I2C_IT_STOPF) == SET)
    {
        I2C_ClearITPendingBit(I2C_PERIPH_NAME, I2C_IT_STOPF);

        if (direction == I2C_Direction_Receiver)
        {
            if (response == ACK)
            {
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


            /* delete button status and counter bit from status_word */
            if (i2c_state->rx_buf[CMD_INDEX] == CMD_GET_STATUS_WORD)
            {
                i2c_state->status_word &= ~BUTTON_PRESSED_STSBIT;
                /* decrease button counter by the value has been sent */
                button_counter_decrease((i2c_state->status_word & BUTTON_COUNTER_VALBITS) >> 13);
            }
        }

        DBG("STOP\r\n");
       // i2c_state->rx_data_ctr = 0;
       // i2c_state->tx_data_ctr = 0;

        //direction = I2C_Direction_Receiver;
      // I2C_NumberOfBytesConfig(I2C_PERIPH_NAME, ONE_BYTE_EXPECTED);
    }

   __enable_irq();
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
    uint8_t led_index;
    uint32_t colour;
    slave_i2c_states_t state = SLAVE_I2C_OK;

    if (i2c_state->data_rx_complete) /* slave RX (master sends data) */
    {
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

            case CMD_LED_COLOUR:
            {
                led_index = i2c_state->rx_buf[1] & 0x0F;
                /* colour = Red + Green + Blue */
                colour = (i2c_state->rx_buf[2] << 16) | (i2c_state->rx_buf[3] << 8) | i2c_state->rx_buf[4];

                led_driver_set_colour(led_index, colour);

                DBG("set LED colour - LED index : ")
                DBG((const char*)&led_index);
                DBG("\r\nRED: ");
                DBG((const char*)(i2c_state->rx_buf + 2));
                DBG("\r\n");
            } break;

            case CMD_SET_BRIGHTNESS:
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

            case CMD_WATCHDOG:
            {
                watchdog_sts = i2c_state->rx_buf[1];
            } break;

            default: /* it should never happen */
            {
                I2C_SoftwareResetCmd(I2C_PERIPH_NAME);
                slave_i2c_config();

                DBG("unexpected command: ");
                DBG((const char*)i2c_state->rx_buf);
                DBG("\r\n");
            } break;
        }

        /* clear flag */
        i2c_state->data_rx_complete = 0;
    }

    return state;
}
