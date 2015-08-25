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

#define I2C_SDA_SOURCE                  GPIO_PinSource7
#define I2C_SCL_SOURCE                  GPIO_PinSource6

#define I2C_ALTERNATE_FUNCTION          GPIO_AF_1
#define I2C_TIMING                      0x1045061D

#define I2C_GPIO_CLOCK                  RCC_AHBPeriph_GPIOF
#define I2C_PERIPH_NAME                 I2C2
#define I2C_PERIPH_CLOCK                RCC_APB1Periph_I2C2
#define I2C_DATA_PIN                    GPIO_Pin_7 // I2C2_SDA - GPIOF
#define I2C_CLK_PIN                     GPIO_Pin_6 // I2C2_SCL - GPIOF
#define I2C_GPIO_PORT                   GPIOF

#define I2C_SLAVE_ADDRESS               0x55
#define MAX_BUFFER_SIZE                 10

struct st_i2c_status {
    uint8_t address_match_slave_rx   : 1;
    uint8_t address_match_slave_tx   : 1;
    uint8_t data_rx_complete         : 1; // stop flag detected - all data received
    uint8_t data_tx_complete         : 1; // stop flag detected - all data sent
    uint8_t timeout                  : 1;
    uint8_t data_ctr;                     // data counter
    uint8_t rx_buf[MAX_BUFFER_SIZE];      // RX buffer
    uint8_t tx_buf[MAX_BUFFER_SIZE];      // TX buffer
};

static struct st_i2c_status i2c_state;

/*******************************************************************************
  * @function   slave_i2c_config
  * @brief      Configuration of I2C peripheral as a slave.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
static void slave_i2c_config(void)
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
    struct st_i2c_status *i2c_status = &i2c_state;

    if(flag == I2C_FLAG_BUSY)
    {
        while(I2C_GetFlagStatus(I2C_PERIPH_NAME, flag) != RESET)
        {
            if(i2c_timeout-- == 0)
            {
                i2c_status->timeout = 1;
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
                i2c_status->timeout = 1;
                return;
            }
        }
    }
}
