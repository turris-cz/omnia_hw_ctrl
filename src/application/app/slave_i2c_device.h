/**
 ******************************************************************************
 * @file    slave_i2c_device.h
 * @author  CZ.NIC, z.s.p.o.
 * @date    25-October-2021
 * @brief   Header for I2C driver.
 ******************************************************************************
 ******************************************************************************
 **/

#ifndef SLAVE_I2C_DEVICE_H
#define SLAVE_I2C_DEVICE_H

#define MAX_RX_BUFFER_SIZE                 10
#define MAX_TX_BUFFER_SIZE                 20

typedef enum slave_i2c_states {
    SLAVE_I2C_OK,
    SLAVE_I2C_LIGHT_RST,
    SLAVE_I2C_HARD_RST,
    SLAVE_I2C_PWR4V5_ENABLE,
    SLAVE_I2C_GO_TO_BOOTLOADER
}slave_i2c_states_t;

struct st_i2c_status {
    uint16_t status_word;
    uint8_t reset_type;
    slave_i2c_states_t state;             // reported in main state machine
    uint8_t rx_data_ctr;                  // RX data counter
    uint8_t tx_data_ctr;                  // TX data counter
    uint8_t tx_data_len;                  // TX buffer length
    uint8_t rx_buf[MAX_RX_BUFFER_SIZE];   // RX buffer
    uint8_t tx_buf[MAX_TX_BUFFER_SIZE];   // TX buffer
    uint8_t data_tx_complete         : 1; // stop flag detected - all data sent
    enum {
        STOPPED = 0,
        RECEIVING,
        TRANSMITTING,
        TRANSMITTED,
    } handler_state;
};

extern struct st_i2c_status i2c_status;

enum status_word_bits {
    GD32_MCU_STSBIT        = 0x0001,
    CARD_DET_STSBIT        = 0x0010,
    MSATA_IND_STSBIT       = 0x0020,
    USB30_OVC_STSBIT       = 0x0040,
    USB31_OVC_STSBIT       = 0x0080,
    USB30_PWRON_STSBIT     = 0x0100,
    USB31_PWRON_STSBIT     = 0x0200,
    ENABLE_4V5_STSBIT      = 0x0400,
    BUTTON_MODE_STSBIT     = 0x0800,
    BUTTON_PRESSED_STSBIT  = 0x1000,
    BUTTON_COUNTER_VALBITS = 0xE000
};

/*
 * Bit meanings in status_word:
 *  Bit Nr. |   Meanings
 * -----------------
 *      0   |   dont care
 *      1   |   dont care
 *      2   |   dont care
 *      3   |   dont care
 *      4   |   CARD_DET        : 1 - mSATA/PCIe card detected, 0 - no card
 *      5   |   mSATA_IND       : 1 - mSATA card inserted, 0 - PCIe card inserted
 *      6   |   USB30_OVC       : 1 - USB3-port0 overcurrent, 0 - no overcurrent
 *      7   |   USB31_OVC       : 1 - USB3-port1 overcurrent, 0 - no overcurrent
 *      8   |   USB30_PWRON     : 1 - USB3-port0 power ON, 0 - USB-port0 power off
 *      9   |   USB31_PWRON     : 1 - USB3-port1 power ON, 0 - USB-port1 power off
 *     10   |   ENABLE_4V5      : 1 - 4.5V power is enabled, 0 - 4.5V power is disabled
 *     11   |   BUTTON_MODE     : 1 - user mode, 0 - default mode (brightness settings)
 *     12   |   BUTTON_PRESSED  : 1 - button pressed in user mode, 0 - button not pressed
 * 13..15   |   BUTTON_COUNT    : number of pressing of the button (max. 7) - valid in user mode
*/

/*
 * Byte meanings in reset_byte:
 *  Byte Nr. |   Meanings
 * -----------------
 *   1.B    |   RESET_TYPE      : 0 - normal reset, 1 - previous snapshot,
 *                              2 - normal factory reset, 3 - hard factory reset
*/

/*
 * Bit meanings in control_byte:
 *  Bit Nr. |   Meanings
 * -----------------
 *      0   |   LIGHT_RST   : 1 - do light reset, 0 - no reset
 *      1   |   HARD_RST    : 1 - do hard reset, 0 - no reset
 *      2   |   dont care
 *      3   |   USB30_PWRON : 1 - USB3-port0 power ON, 0 - USB-port0 power off
 *      4   |   USB31_PWRON : 1 - USB3-port1 power ON, 0 - USB-port1 power off
 *      5   |   ENABLE_4V5  : 1 - 4.5V power supply ON, 0 - 4.5V power supply OFF
 *      6   |   BUTTON_MODE : 1 - user mode, 0 - default mode (brightness settings)
 *      7   |   BOOTLOADER  : 1 - jump to bootloader
*/

/*
 * Bit meanings in led_mode_byte:
 *  Bit Nr. |   Meanings
 * -----------------
 *   0..3   |   LED number [0..11] (or in case setting of all LED at once -> LED number = 12)
 *      4   |   LED mode    : 1 - USER mode, 0 - default mode
 *   5..7   |   dont care
*/

/*
 * Bit meanings in led_state_byte:
 *  Bit Nr. |   Meanings
 * -----------------
 *   0..3   |   LED number [0..11] (or in case setting of all LED at once -> LED number = 12)
 *      4   |   LED state    : 1 - LED ON, 0 - LED OFF
 *   5..7   |   dont care
*/

/*
 * Bit meanings in led_colour:
 * Byte Nr. |  Bit Nr. |   Meanings
 * -----------------
 *  1.B     |  0..3   |   LED number [0..11] (or in case setting of all LED at once -> LED number = 12)
 *  1.B     |  4..7   |   dont care
 *  2.B     |  8..15  |   red colour [0..255]
 *  3.B     |  16..23 |   green colour [0..255]
 *  4.B     |  24..31 |   blue colour [0..255]
*/

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

#endif /* SLAVE_I2C_DEVICE_H */

