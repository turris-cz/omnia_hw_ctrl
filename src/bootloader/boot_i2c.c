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
#include "debug_serial.h"
#include "boot_led_driver.h"
#include "power_control.h"
#include "delay.h"
#include "eeprom.h"
#include "flash.h"
#include "boot_i2c.h"
#include "bootloader.h"
#include <string.h>


__attribute__((section(".boot_version"))) uint8_t version[20] = VERSION;

#define I2C_GPIO_CLOCK                  RCU_GPIOF
#define I2C_PERIPH_NAME                 I2C1
#define I2C_PERIPH_CLOCK                RCU_I2C1
#define I2C_DATA_PIN                    GPIO_PIN_7 /* I2C2_SDA - GPIOF */
#define I2C_CLK_PIN                     GPIO_PIN_6 /* I2C2_SCL - GPIOF */
#define I2C_GPIO_PORT                   GPIOF

#define I2C_SLAVE_ADDRESS               0x58  /* address in linux: 0x2C */

#define LOW_ADDR_BYTE_IDX               1
#define HIGH_ADDR_BYTE_IDX              0
#define DATA_START_BYTE_IDX             2

#define FILE_CMP_OK                     0xBB
#define FILE_CMP_ERROR                  0xDD
#define ADDR_CMP                        0xFFFF

#define ONE_BYTE_EXPECTED               1

struct st_i2c_status i2c_status;

static uint8_t flash_erase_sts; /* indicates start of flashing */

/*******************************************************************************
  * @function   boot_i2c_config
  * @brief      Configuration of pins for I2C.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
static void boot_i2c_io_config(void)
{
    /* I2C Peripheral Disable */
    rcu_periph_clock_disable(I2C_PERIPH_CLOCK);

    /* I2C Periph clock enable */
    rcu_periph_clock_enable(I2C_PERIPH_CLOCK);

    rcu_periph_clock_enable(I2C_GPIO_CLOCK);

    /* Configure I2C pins: SCL */
    gpio_mode_set(I2C_GPIO_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, I2C_CLK_PIN);
    gpio_output_options_set(I2C_GPIO_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, I2C_CLK_PIN);

    gpio_mode_set(I2C_GPIO_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, I2C_DATA_PIN);
    gpio_output_options_set(I2C_GPIO_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, I2C_DATA_PIN);
}

/*******************************************************************************
  * @function   boot_i2c_periph_config
  * @brief      Configuration of I2C peripheral as a slave.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
static void boot_i2c_periph_config(void)
{
    i2c_deinit(I2C_PERIPH_NAME);
    i2c_disable(I2C_PERIPH_NAME);


    /* I2C clock configure */
    i2c_clock_config(I2C_PERIPH_NAME, 100000, I2C_DTCY_2);
    /* I2C address configure */

    i2c_mode_addr_config(I2C_PERIPH_NAME, I2C_I2CMODE_ENABLE, I2C_ADDFORMAT_7BITS, I2C_SLAVE_ADDRESS);

    i2c_interrupt_enable(I2C_PERIPH_NAME, I2C_INT_BUF);
    i2c_interrupt_enable(I2C_PERIPH_NAME, I2C_INT_EV);

    /* enable I2C */
    i2c_enable(I2C_PERIPH_NAME);
    /* enable acknowledge */
    i2c_ack_config(I2C_PERIPH_NAME, I2C_ACK_ENABLE);

    nvic_irq_enable(I2C1_EV_IRQn, 0, 1);
}

/*******************************************************************************
  * @function   boot_i2c_config
  * @brief      Configuration of I2C peripheral and its timeout.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void boot_i2c_config(void)
{
    boot_i2c_io_config();
    boot_i2c_periph_config();
}

/*******************************************************************************
  * @function   boot_i2c_handler
  * @brief      Interrupt handler for I2C communication.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void boot_i2c_handler(void)
{
    struct st_i2c_status *i2c_state = &i2c_status;
    //static uint16_t direction;
    static uint32_t flash_address = APPLICATION_ADDRESS;
    static uint8_t data;
    static uint8_t rx_dir;

    __disable_irq();

    if (!flash_erase_sts) /* we are at the beginning again */
    {
        flash_address = APPLICATION_ADDRESS;
    }

    /* address match interrupt */
    if(i2c_interrupt_flag_get(I2C_PERIPH_NAME, I2C_INT_FLAG_ADDSEND) == SET)
    {
        /* clear the ADDSEND bit */
        i2c_interrupt_flag_clear(I2C_PERIPH_NAME, I2C_INT_FLAG_ADDSEND);
        DBG_UART("ADDR\r\n");
    }

    /* transfer complete interrupt (TX and RX) */
    else if(i2c_interrupt_flag_get(I2C_PERIPH_NAME, I2C_INT_FLAG_RBNE))
    {
        i2c_state->rx_buf[i2c_state->rx_data_ctr++] = i2c_data_receive(I2C_PERIPH_NAME);
        rx_dir = 1;
    }

    /* transmit interrupt */
    else if((i2c_interrupt_flag_get(I2C_PERIPH_NAME, I2C_INT_FLAG_TBE)) && (!i2c_interrupt_flag_get(I2C1, I2C_INT_FLAG_AERR)))
    {
        flash_read(&flash_address, &data);
        i2c_data_transmit(I2C_PERIPH_NAME, data);
        i2c_state->tx_data_ctr++;

        if (i2c_state->tx_data_ctr >= I2C_DATA_PACKET_SIZE)
        {
            i2c_state->tx_data_ctr = 0;
        }
        DBG_UART("send\r\n");
    }

    /* stop flag */
    else if(i2c_interrupt_flag_get(I2C_PERIPH_NAME, I2C_INT_FLAG_STPDET))
    {
        i2c_enable(I2C_PERIPH_NAME); /* clear the STPDET bit */

        if (rx_dir == 1)
            i2c_state->data_rx_complete = 1;

        /* disable I2C interrupts ? */

        DBG_UART("STOP\r\n");
    }

    __enable_irq();
}

/*******************************************************************************
  * @function   clear_rxbuf
  * @brief      Delete RX buffer.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
static void clear_rxbuf(void)
{
    struct st_i2c_status *i2c_state = &i2c_status;
    uint16_t idx;

    for (idx = 0; idx < I2C_DATA_PACKET_SIZE; idx++)
    {
        i2c_state->rx_buf[idx + DATA_START_BYTE_IDX] = 0;
    }
}

/*******************************************************************************
  * @function   boot_i2c_flash_data
  * @brief      Flash received data.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
flash_i2c_states_t boot_i2c_flash_data(void)
{
    struct st_i2c_status *i2c_state = &i2c_status;
    uint16_t rx_cmd, idx, data_length = I2C_DATA_PACKET_SIZE / 4;
    uint8_t data[I2C_DATA_PACKET_SIZE];
    static flash_i2c_states_t flash_status = FLASH_CMD_NOT_RECEIVED;
    static uint32_t flash_address = APPLICATION_ADDRESS;

    if (i2c_state->data_rx_complete)
    {
        memset(data, 0, sizeof(data));

        flash_status = FLASH_CMD_RECEIVED;

        rx_cmd = (i2c_state->rx_buf[HIGH_ADDR_BYTE_IDX] << 8) | \
                            (i2c_state->rx_buf[LOW_ADDR_BYTE_IDX]);

        /* copy data */
        for(idx = 0; idx < I2C_DATA_PACKET_SIZE; idx++)
        {
            data[idx] = i2c_state->rx_buf[idx + DATA_START_BYTE_IDX];
        }

        if (!flash_erase_sts) /* enter the flash sequence, erase pages */
        {
            flash_erase(flash_address);
            flash_erase_sts = 1;
            DBG_UART("FL_NOT_CONF\r\n");
        }

        if (rx_cmd == ADDR_CMP) /* flashing is complete, linux send result of comparison */
        {
            if (data[0] == FILE_CMP_OK)
            {
                flash_status = FLASH_WRITE_OK;
                DBG_UART("WRITE_OK\n\r");
            }
            else
            {
                flash_status = FLASH_WRITE_ERROR;
                DBG_UART("WRITE ERR\n\r");
            }

            flash_address = APPLICATION_ADDRESS;
            flash_erase_sts = 0;
            i2c_state->tx_data_ctr = 0;
        }
        else /* write incoming data */
        {
            flash_write(&flash_address, (uint32_t*)data, data_length);
        }

        clear_rxbuf();

        i2c_state->data_rx_complete = 0;
        i2c_state->rx_data_ctr = 0;

        //i2c_interrupt_enable(I2C_PERIPH_NAME, I2C_INT_BUF);
        //i2c_interrupt_enable(I2C1, I2C_INT_EV)
    }

    return flash_status;
}
