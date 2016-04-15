/**
 ******************************************************************************
 * @file    boot_i2c.h
 * @author  CZ.NIC, z.s.p.o.
 * @date    15-April-2016
 * @brief   Header for I2C driver.
 ******************************************************************************
 ******************************************************************************
 **/

#ifndef BOOT_I2C_H
#define BOOT_I2C_H

#define I2C_ADRESS_FIELD_SIZE               2
#define I2C_DATA_PACKET_SIZE                128
#define MAX_RX_BUFFER_SIZE                  (I2C_ADRESS_FIELD_SIZE + I2C_DATA_PACKET_SIZE)
#define MAX_TX_BUFFER_SIZE                  20

typedef enum slave_i2c_states {
    SLAVE_I2C_OK,
    SLAVE_I2C_LIGHT_RST,
    SLAVE_I2C_HARD_RST,
    SLAVE_I2C_PWR4V5_ERROR,
}slave_i2c_states_t;

struct st_i2c_status {
    uint16_t status_word;
    uint8_t reset_type;
    slave_i2c_states_t state;             // reported in main state machine
    uint8_t rx_data_ctr;                  // RX data counter
    uint8_t tx_data_ctr;                  // TX data counter
    uint8_t rx_buf[MAX_RX_BUFFER_SIZE];   // RX buffer
    uint8_t tx_buf[MAX_TX_BUFFER_SIZE];   // TX buffer
    uint8_t data_rx_complete         : 1; // stop flag detected - all data received
    uint8_t data_tx_complete         : 1; // stop flag detected - all data sent
};

extern struct st_i2c_status i2c_status;

/*******************************************************************************
  * @function   slave_i2c_config
  * @brief      Configuration of I2C peripheral as a slave.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void slave_i2c_config(void);

/*******************************************************************************
  * @function   slave_i2c_handler
  * @brief      Interrupt handler for I2C communication.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void slave_i2c_handler(void);

void slave_i2c_process_data(void);

#endif /* BOOT_I2C_H */

