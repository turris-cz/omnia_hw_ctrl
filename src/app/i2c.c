#include "stm32f0xx_conf.h"
#include "slave_i2c_device.h"

#define I2C_PERIPH_NAME                 I2C2

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

void i2c_slave(void)
{
    struct st_i2c_status *i2c_state = &i2c_status;

    static uint16_t direction;

    if(I2C_GetFlagStatus(I2C_PERIPH_NAME, I2C_FLAG_ADDR) == SET)
    {
        I2C_ClearFlag(I2C_PERIPH_NAME, I2C_FLAG_ADDR);

        if ((I2C_PERIPH_NAME->ISR & I2C_ISR_DIR) == I2C_ISR_DIR)
        {
            //I2C_PERIPH_NAME->CR1 |= I2C_CR1_TXIE; /* Set transmit IT */
            direction = I2C_Direction_Transmitter;
        }
        else
        {
            direction = I2C_Direction_Receiver;
        }
    }
    else if(I2C_GetFlagStatus(I2C_PERIPH_NAME, I2C_FLAG_TCR) == SET)
    {
        if (direction == I2C_Direction_Receiver)
        {
            i2c_state->rx_buf[i2c_state->rx_data_ctr] = I2C_ReceiveData(I2C_PERIPH_NAME);
            i2c_state->rx_data_ctr++;

            if (i2c_state->rx_data_ctr == 1) /* byte 0 */
            {
                switch(i2c_state->rx_buf[0])
                {
                    case CMD_LED_MODE:
                    case CMD_LED_STATE: I2C_AcknowledgeConfig(I2C_PERIPH_NAME, ENABLE); break;
                    case CMD_GET_STATUS_WORD:
                    {
                        i2c_state->tx_buf[0] = i2c_state->status_word & 0x00FF;
                        i2c_state->tx_buf[1] = (i2c_state->status_word & 0xFF00) >> 8;
                        I2C_AcknowledgeConfig(I2C_PERIPH_NAME, ENABLE);
                        I2C_NumberOfBytesConfig(I2C_PERIPH_NAME, 2);
                        return;
                    }
                    break;
                    default: I2C_AcknowledgeConfig(I2C_PERIPH_NAME, DISABLE);break;
                }
                I2C_NumberOfBytesConfig(I2C_PERIPH_NAME, 1);
            }
            else /* byte 1, 2 */
            {
                I2C_AcknowledgeConfig(I2C_PERIPH_NAME, ENABLE);
                I2C_NumberOfBytesConfig(I2C_PERIPH_NAME, 1);
            }
        }
        else /* I2C_Direction_Transmitter */
        {
            /* nic */
        }

    }
    else if(I2C_GetFlagStatus(I2C_PERIPH_NAME, I2C_FLAG_TXIS) == SET)
    {
        I2C_SendData(I2C_PERIPH_NAME, i2c_state->tx_buf[i2c_state->tx_data_ctr++]);

        if (i2c_state->tx_data_ctr >= MAX_TX_BUFFER_SIZE)
        {
            i2c_state->tx_data_ctr = 0;
        }
    }
    else if (I2C_GetFlagStatus(I2C_PERIPH_NAME, I2C_FLAG_STOPF) == SET)
    {
        I2C_ClearFlag(I2C_PERIPH_NAME, I2C_FLAG_STOPF);
        i2c_state->tx_data_ctr = 0;
        i2c_state->rx_data_ctr = 0;
    }
}
