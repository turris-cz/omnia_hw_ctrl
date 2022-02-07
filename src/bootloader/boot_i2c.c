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

#define FILE_CMP_OK                     0xBB
#define FILE_CMP_ERROR                  0xDD
#define ADDR_CMP                        0xFFFF

struct st_i2c_status i2c_status;

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

    i2c_interrupt_enable(I2C_PERIPH_NAME, I2C_INT_EV);

    i2c_stretch_scl_low_config(I2C_PERIPH_NAME, I2C_SCLSTRETCH_ENABLE);

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

static uint16_t boot_i2c_get_addr(void)
{
    return (i2c_status.rx_addr[0] << 8) | i2c_status.rx_addr[1];
}

static uint32_t boot_i2c_is_addr_valid(uint16_t addr)
{
    return (addr & (I2C_DATA_PACKET_SIZE - 1)) == 0 &&
           ((uint32_t)addr + I2C_DATA_PACKET_SIZE) <= APPLICATION_MAX_SIZE;
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
    static uint8_t rx_dir, tx_dir;
    uint16_t stat0;

    __disable_irq();

    stat0 = I2C_STAT0(I2C_PERIPH_NAME);

    /* acknowledge not received interrupt */
    if (tx_dir && (stat0 & I2C_STAT0_AERR))
    {
        DBG_UART("AERR\r\n");

        /* clear AERR */
        I2C_STAT0(I2C_PERIPH_NAME) &= ~I2C_STAT0_AERR;
        i2c_interrupt_disable(I2C_PERIPH_NAME, I2C_INT_BUF);
        i2c_interrupt_disable(I2C_PERIPH_NAME, I2C_INT_ERR);

        i2c_state->tx_ptr = NULL;
        i2c_state->tx_data_ctr = 0;

        tx_dir = 0;
    }

    /* stop flag */
    else if (rx_dir && (stat0 & I2C_STAT0_STPDET))
    {
        DBG_UART("STOP\r\n");

        /* clear the STPDET bit */
        i2c_enable(I2C_PERIPH_NAME);

        i2c_interrupt_disable(I2C_PERIPH_NAME, I2C_INT_BUF);

        if (i2c_state->rx_data_ctr > 2)
        {
            uint16_t addr = boot_i2c_get_addr();

            if (boot_i2c_is_addr_valid(addr))
            {
                i2c_state->prog_addr = addr;
                i2c_state->prog_len = i2c_state->rx_data_ctr - 2;
                i2c_state->prog_state = PROG_START;

                i2c_ack_config(I2C_PERIPH_NAME, I2C_ACK_DISABLE);
                DBG_UART("prog start\r\n");
            }
            else if (addr == ADDR_CMP && i2c_state->rx_data_ctr == 3)
            {
                if (i2c_state->rx_buf[0] == FILE_CMP_OK)
                    i2c_state->prog_state = PROG_OK;
                else
                    i2c_state->prog_state = PROG_ERROR;
            }
            else
            {
                DBG_UART("prog invalid address\r\n");
            }
        }
        else if (i2c_state->rx_data_ctr == 2)
        {
            uint16_t addr = boot_i2c_get_addr();

            if (boot_i2c_is_addr_valid(addr))
            {
                DBG_UART("recv request\r\n");
                i2c_state->tx_ptr = (uint8_t *)(APPLICATION_ADDRESS + addr);
            }
        }

        rx_dir = 0;
    }

    /* transmit interrupt */
    else if (tx_dir && (stat0 & I2C_STAT0_TBE))
    {
        if (i2c_state->tx_data_ctr < I2C_DATA_PACKET_SIZE)
        {
            uint8_t c = *i2c_state->tx_ptr++;

            i2c_data_transmit(I2C_PERIPH_NAME, c);
            i2c_state->tx_data_ctr++;

            DBG_UART("send flash byte\r\n");
        }
        else
        {
            DBG_UART("send 0xff\r\n");
            i2c_data_transmit(I2C_PERIPH_NAME, 0xff);
        }
    }

    /* receive interrupt */
    else if (rx_dir && (stat0 & I2C_STAT0_RBNE))
    {
        uint8_t c = i2c_data_receive(I2C_PERIPH_NAME);

        if (i2c_state->prog_state != PROG_WAITING)
            goto end;

        if (i2c_state->rx_data_ctr < I2C_ADDRESS_FIELD_SIZE)
        {
            i2c_state->rx_addr[i2c_state->rx_data_ctr++] = c;

            DBG_UART("recv addr\r\n");
        }
        else if (i2c_state->rx_data_ctr < MAX_RX_BUFFER_SIZE)
        {
            i2c_state->rx_buf[i2c_state->rx_data_ctr - 2] = c;
            i2c_state->rx_data_ctr++;

            DBG_UART("recv data\r\n");
        }
    }

    /* address match interrupt */
    else if (stat0 & I2C_STAT0_ADDSEND)
    {
        uint16_t stat1;

        /* reading stat1 after stat0 clears the ADDSEND bit */
        stat1 = I2C_STAT1(I2C_PERIPH_NAME);

        if (stat1 & I2C_STAT1_TR) {
            DBG_UART("ADDR tx\r\n");
            tx_dir = 1;
            i2c_interrupt_enable(I2C_PERIPH_NAME, I2C_INT_ERR);
        } else {
            DBG_UART("ADDR rx\r\n");
            rx_dir = 1;
            i2c_state->tx_data_ctr = 0;
        }

        /* enable RBNE/TBE interrupt */
        i2c_interrupt_enable(I2C_PERIPH_NAME, I2C_INT_BUF);

        i2c_state->rx_data_ctr = 0;
    }

end:
    __enable_irq();
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
    static flash_i2c_states_t result;

    if (i2c_state->prog_state == PROG_START) {
        uint32_t addr = APPLICATION_ADDRESS + i2c_state->prog_addr;

        if ((i2c_state->prog_addr & (FLASH_PAGE_SIZE - 1)) == 0)
        {
            DBG_UART("erasing\r\n");
            fmc_page_erase(addr);
        }

        DBG_UART("writing\r\n");
        flash_write(&addr, i2c_state->rx_buf_longs, (i2c_state->prog_len + 3) / 4);

        __disable_irq();

        i2c_state->prog_state = PROG_WAITING;
        i2c_ack_config(I2C_PERIPH_NAME, I2C_ACK_ENABLE);
        DBG_UART("prog waiting\r\n");

        __enable_irq();

        result = FLASH_CMD_RECEIVED;
    }
    else if (i2c_state->prog_state == PROG_OK)
    {
        result = FLASH_WRITE_OK;
        i2c_state->prog_state = PROG_WAITING;
    }
    else if (i2c_state->prog_state == PROG_ERROR)
    {
        result = FLASH_WRITE_ERROR;
        i2c_state->prog_state = PROG_WAITING;
    }
    else
    {
        result = FLASH_CMD_NOT_RECEIVED;
    }

    return result;
}
