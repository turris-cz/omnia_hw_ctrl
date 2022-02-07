/**
 ******************************************************************************
 * @file    boot_i2c.h
 * @author  CZ.NIC, z.s.p.o.
 * @date    15-April-2016
 * @brief   Header for I2C driver.
 ******************************************************************************
 ******************************************************************************
 **/

#include "gd32f1x0.h"

#ifndef BOOT_I2C_H
#define BOOT_I2C_H

#define I2C_ADDRESS_FIELD_SIZE              2
#define I2C_DATA_PACKET_SIZE                128
#define MAX_RX_BUFFER_SIZE                  (I2C_ADDRESS_FIELD_SIZE + I2C_DATA_PACKET_SIZE)

typedef enum flash_i2c_states {
    FLASH_CMD_RECEIVED,
    FLASH_CMD_NOT_RECEIVED,
    FLASH_WRITE_OK,
    FLASH_WRITE_ERROR
}flash_i2c_states_t;

struct st_i2c_status {
    uint8_t rx_data_ctr;                  // RX data counter
    uint8_t tx_data_ctr;                  // TX data counter
    volatile uint8_t *tx_ptr;
    uint8_t rx_addr[I2C_ADDRESS_FIELD_SIZE];
    union {
        uint8_t rx_buf[I2C_DATA_PACKET_SIZE];
        uint32_t rx_buf_longs[I2C_DATA_PACKET_SIZE / sizeof(uint32_t)];
    };
    enum {
        PROG_WAITING = 0,
        PROG_START,
        PROG_OK,
        PROG_ERROR,
    } prog_state;
    uint16_t prog_addr;
    uint8_t prog_len;
};

extern struct st_i2c_status i2c_status;

/*******************************************************************************
  * @function   slave_i2c_config
  * @brief      Configuration of I2C peripheral as a slave.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void boot_i2c_config(void);

/*******************************************************************************
  * @function   slave_i2c_handler
  * @brief      Interrupt handler for I2C communication.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void boot_i2c_handler(void);

/*******************************************************************************
  * @function   boot_i2c_flash_data
  * @brief      Flash received data.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
flash_i2c_states_t boot_i2c_flash_data(void);

#endif /* BOOT_I2C_H */

