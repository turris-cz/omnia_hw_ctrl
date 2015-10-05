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

#define I2C_SLAVE_ADDRESS               0x55

enum i2c_basic_commands {
    CMD_SLAVE_RX                        = 0x01, // slave only receives data
    CMD_SLAVE_TX                        = 0x02, // slave sends back status word
};

enum i2c_commands {
    CMD_GENERAL_CONTROL                 = 0x10,
    CMD_LED_MODE                        = 0x20, // default/user
    CMD_LED_STATE                       = 0x21, // LED on/off
    CMD_LED_COLOUR                      = 0x22,
    CMD_LED_BRIGHTNESS                  = 0x23,
};

enum i2_control_byte_mask {
    LIGHT_RST_MASK                      = 0x01,
    HARD_RST_MASK                       = 0x02,
    FACTORY_RST_MASK                    = 0x04,
    SFP_DIS_MASK                        = 0x08,
    USB30_PWRON_MASK                    = 0x10,
    USB31_PWRON_MASK                    = 0x20,
    ENABLE_4V5_MASK                     = 0x40,
};

struct st_i2c_status i2c_status;

/*******************************************************************************
  * @function   slave_i2c_config
  * @brief      Configuration of I2C peripheral as a slave.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void slave_i2c_config(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    I2C_InitTypeDef  I2C_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

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

    /* I2C configuration */
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_AnalogFilter = I2C_AnalogFilter_Enable;
    I2C_InitStructure.I2C_DigitalFilter = 0x00;
    I2C_InitStructure.I2C_OwnAddress1 = I2C_SLAVE_ADDRESS;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_Timing = I2C_TIMING; //TODO: check i2c speed !

    /* Apply I2C configuration after enabling it */
    I2C_Init(I2C_PERIPH_NAME, &I2C_InitStructure);

    /* Address match and receive interrupt */
    I2C_ITConfig(I2C_PERIPH_NAME, I2C_IT_ADDRI | I2C_IT_RXI | I2C_IT_STOPI, ENABLE);

    /* I2C Peripheral Enable */
    I2C_Cmd(I2C_PERIPH_NAME, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = I2C2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority = 0x01; //TODO: set and check the highest priority
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/*******************************************************************************
  * @function   slave_i2c_wait_for_flag
  * @brief      Wait for given flag during ic2 communication.
  * @param      flag: possible flags can be found in stm32f0xx_i2c.h
  * @retval     None.
  *****************************************************************************/
static void slave_i2c_wait_for_flag(uint32_t flag)
{
    uint32_t i2c_timeout = 2000000u;
    struct st_i2c_status *i2c_state = &i2c_status;

    if(flag == I2C_FLAG_BUSY)
    {
        while(I2C_GetFlagStatus(I2C_PERIPH_NAME, flag) != RESET)
        {
            if(i2c_timeout-- == 0)
            {
                i2c_state->timeout = 1;
                return;
            }
        }
    }
    else
    {
        while(I2C_GetFlagStatus(I2C_PERIPH_NAME, flag) == RESET)
        {
            if(i2c_timeout-- == 0)
            {
                i2c_state->timeout = 1;
                return;
            }
        }
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

    /* Test on I2C Address match interrupt */
    if((I2C_GetITStatus(I2C_PERIPH_NAME, I2C_IT_ADDR) == SET ))
    {
        if (i2c_state->rx_buf[0] == CMD_SLAVE_TX)
        {
            /* at the moment the command from master is stored in rx_buf[0]
            and the master is waiting for data */
            i2c_state->address_match_slave_tx = 1;
            DBG("slave tx adddress match\r\n");
        }
        else
        {
            i2c_state->address_match_slave_rx = 1;
            DBG("slave rx adddress match\r\n");
        }

        /* Clear IT pending bit */
        I2C_ClearITPendingBit(I2C_PERIPH_NAME, I2C_IT_ADDR);
    }

    // transmit data
    if ((I2C_GetITStatus(I2C_PERIPH_NAME, I2C_IT_TXIS) == SET) && (i2c_state->address_match_slave_tx == 1))
    {
        I2C_SendData(I2C_PERIPH_NAME, i2c_state->tx_buf[i2c_state->tx_data_ctr++]);
        DBG((const char*)&i2c_state->tx_buf[i2c_state->tx_data_ctr - 1]);
    }

    // receive data
    if ((I2C_GetITStatus(I2C_PERIPH_NAME, I2C_IT_RXNE) == SET) && (i2c_state->address_match_slave_rx == 1))
    {
        i2c_state->rx_buf[i2c_state->rx_data_ctr++] = I2C_ReceiveData(I2C_PERIPH_NAME);
    }

    // stop detection after data from master are received
    if ((I2C_GetITStatus(I2C_PERIPH_NAME, I2C_IT_STOPF) == SET) && (i2c_state->address_match_slave_rx == 1))
    {
        i2c_state->rx_buf[i2c_state->rx_data_ctr++] = I2C_ReceiveData(I2C_PERIPH_NAME);

        I2C_ClearITPendingBit(I2C_PERIPH_NAME, I2C_IT_STOPF);

        DBG((const char*)i2c_state->rx_buf);
        DBG(" -> data received\r\n");

        // reception phase complete
        i2c_state->data_rx_complete = 1;
        i2c_state->address_match_slave_rx = 0;
        // disable interrupt in order to process incoming data
        I2C_ITConfig(I2C_PERIPH_NAME, I2C_IT_ADDRI | I2C_IT_RXI | I2C_IT_STOPI | I2C_IT_TXI, DISABLE);
    }

    if ((I2C_GetITStatus(I2C_PERIPH_NAME, I2C_IT_STOPF) == SET) && (i2c_state->address_match_slave_tx == 1))
    {
        I2C_ClearITPendingBit(I2C_PERIPH_NAME, I2C_IT_STOPF);

        DBG("\r\n");
        DBG((const char*)i2c_state->tx_buf);
        DBG(" -> data transmitted\r\n");
        // transmit phase complete
        i2c_state->address_match_slave_tx = 0;
        i2c_state->data_tx_complete = 1;

        // disable interrupt in order to clear all buffers in data processing
        I2C_ITConfig(I2C_PERIPH_NAME, I2C_IT_ADDRI | I2C_IT_RXI | I2C_IT_STOPI | I2C_IT_TXI, DISABLE);
    }
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

    if (control_byte & USB30_PWRON_MASK)
        power_control_usb(USB3_PORT0, USB_ON);
    else
        power_control_usb(USB3_PORT0, USB_OFF);

    if (control_byte & USB31_PWRON_MASK)
        power_control_usb(USB3_PORT1, USB_ON);
    else
        power_control_usb(USB3_PORT1, USB_OFF);

    if (control_byte & ENABLE_4V5_MASK)
        GPIO_SetBits(ENABLE_4V5_PIN_PORT, ENABLE_4V5_PIN);
    else
        GPIO_ResetBits(ENABLE_4V5_PIN_PORT, ENABLE_4V5_PIN);
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
    uint16_t i;
    uint32_t colour;
    ret_value_t state = OK;

    if (i2c_state->data_rx_complete)
    {
        // clear flag
        i2c_state->data_rx_complete = 0;

        switch(i2c_state->rx_buf[0])
        {
            case CMD_SLAVE_TX: // slave TX (master expects data)
            {
                //prepare data to be sent to the master
                i2c_state->tx_buf[0] = i2c_state->status_word & 0x00FF;
                i2c_state->tx_buf[1] = (i2c_state->status_word & 0xFF00) >> 8;

                I2C_ITConfig(I2C_PERIPH_NAME, I2C_IT_ADDRI | I2C_IT_RXI | I2C_IT_STOPI | I2C_IT_TXI , ENABLE);

            } break;

            case CMD_SLAVE_RX: // slave RX
            {
                DBG("process RX data\r\n");
                switch (i2c_state->rx_buf[1])
                {
                case CMD_GENERAL_CONTROL:
                    slave_i2c_check_control_byte(i2c_state->rx_buf[2], &state);
                    break;

                case CMD_LED_MODE:
                    led_driver_set_led_mode(i2c_state->rx_buf[2] & 0x0F,
                                            (i2c_state->rx_buf[2] & 0x10) >> 4);
                    break;

                case CMD_LED_STATE:
                    led_driver_set_led_state(i2c_state->rx_buf[2] & 0x0F,
                                            (i2c_state->rx_buf[2] & 0x10) >> 4);
                    break;

                case CMD_LED_COLOUR:
                {
                    colour = (i2c_state->rx_buf[3] << 16) |
                              (i2c_state->rx_buf[4] << 8) | i2c_state->rx_buf[5];

                    led_driver_set_colour(i2c_state->rx_buf[2] & 0x0F, colour);
                }
                    break;

                case CMD_LED_BRIGHTNESS:
                    led_driver_pwm_set_brightness(i2c_state->rx_buf[2]);
                    break;

                default:
                    break;
                }

                i2c_state->rx_data_ctr = 0;
                I2C_ITConfig(I2C_PERIPH_NAME, I2C_IT_ADDRI | I2C_IT_RXI | I2C_IT_STOPI, ENABLE);
            } break;

            default:
                break;
        }
    }

    if (i2c_state->data_tx_complete)
    {
        // clear flag - all data were sent
        i2c_state->data_tx_complete = 0;

        // clear buffers
        for (i = 0; i < MAX_RX_BUFFER_SIZE; i++)
        {
            i2c_state->rx_buf[i] = 0;
        }

        for (i = 0; i < MAX_TX_BUFFER_SIZE; i++)
        {
            i2c_state->tx_buf[i] = 0;
        }

        // clear counters
        i2c_state->rx_data_ctr = 0;
        i2c_state->tx_data_ctr = 0;
        DBG("clear buffers\r\n");

        // enable interrupt again
        I2C_ITConfig(I2C_PERIPH_NAME, I2C_IT_ADDRI | I2C_IT_RXI | I2C_IT_STOPI, ENABLE);
    }

    return state;
}
