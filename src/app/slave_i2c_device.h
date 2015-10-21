/**
 ******************************************************************************
 * @file    slave_i2c_device.h
 * @author  CZ.NIC, z.s.p.o.
 * @date    18-August-2015
 * @brief   Header for I2C driver.
 ******************************************************************************
 ******************************************************************************
 **/
#include "app.h"

#ifndef SLAVE_I2C_DEVICE_H
#define SLAVE_I2C_DEVICE_H

#define MAX_RX_BUFFER_SIZE                 10
#define MAX_TX_BUFFER_SIZE                 2

struct st_i2c_status {
    uint8_t data_rx_complete         : 1; // stop flag detected - all data received
    uint8_t data_tx_complete         : 1; // stop flag detected - all data sent
    uint8_t timeout                  : 1;
    uint8_t status_word_not_sent     : 1; // 1 -indicates that status word is not sent, 0 - status word sent
    uint8_t rx_data_ctr;                  // RX data counter
    uint8_t tx_data_ctr;                  // TX data counter
    uint8_t rx_buf[MAX_RX_BUFFER_SIZE];   // RX buffer
    uint8_t tx_buf[MAX_TX_BUFFER_SIZE];   // TX buffer
    uint16_t status_word_orig;            // original status detected after startup
    uint16_t status_word;
};

extern struct st_i2c_status i2c_status;

/*
 * Bit meanings in status_word:
 *  Bit Nr. |   Meanings
 * -----------------
 *      0   |   SFP_DET         : 1 - SFP detected, 0 - SFP not detected
 *      1   |   SFP_LOS         : 1 - SFP receiver lost signal, 0 - no lost
 *      2   |   SFP_FLT         : 1 - SFP trasmitter fault, 0 - no TX fault
 *      3   |   SFP_DIS         : 1 - SFP TX disabled; 0 - SFP TX enabled
 *      4   |   CARD_DET        : 1 - mSATA/PCIe card detected, 0 - no card
 *      5   |   mSATA_IND       : 1 - mSATA card inserted, 0 - PCIe card inserted
 *      6   |   USB30_OVC       : 1 - USB3-port0 overcurrent, 0 - no overcurrent
 *      7   |   USB31_OVC       : 1 - USB3-port1 overcurrent, 0 - no overcurrent
 *      8   |   USB30_PWRON     : 1 - USB3-port0 power ON, 0 - USB-port0 power off
 *      9   |   USB31_PWRON     : 1 - USB3-port1 power ON, 0 - USB-port1 power off
 *     10   |   ENABLE_4V5      : 1 - 4.5V power is enabled, 0 - 4.5V power is disabled
 *     11   |   BUTTON_PRESSED  : 1 - button pressed in user mode, 0 - basic state (no action)
 * 12..15   |   dont care
*/

/*
 * Bit meanings in control_byte:
 *  Bit Nr. |   Meanings
 * -----------------
 *      0   |   LIGHT_RST   : 1 - do light reset, 0 - no reset
 *      1   |   HARD_RST    : 1 - do hard reset, 0 - no reset
 *      2   |   FACTORY_RST : 1 - do factory reset, 0 - no reset
 *      3   |   SFP_DIS     : 1 - SFP TX disabled; 0 - SFP TX enabled
 *      4   |   USB30_PWRON : 1 - USB3-port0 power ON, 0 - USB-port0 power off
 *      5   |   USB31_PWRON : 1 - USB3-port1 power ON, 0 - USB-port1 power off
 *      6   |   ENABLE_4V5  : 1 - 4.5V power supply ON, 0 - 4.5V power supply OFF
 *      7   |   BUTTON_MODE : 1 - user mode, 0 - default mode (brightness settings)
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

enum status_word_bits {
    SFP_DET_STSBIT         = 0x0001,
    SFP_LOS_STSBIT         = 0x0002,
    SFP_FLT_STSBIT         = 0x0004,
    SFP_DIS_STSBIT         = 0x0008,
    CARD_DET_STSBIT        = 0x0010,
    MSATA_IND_STSBIT       = 0x0020,
    USB30_OVC_STSBIT       = 0x0040,
    USB31_OVC_STSBIT       = 0x0080,
    USB30_PWRON_STSBIT     = 0x0100,
    USB31_PWRON_STSBIT     = 0x0200,
    ENABLE_4V5_STSBIT      = 0x0400,
    BUTTON_PRESSED_STSBIT  = 0x0800,
};

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

/*******************************************************************************
  * @function   slave_i2c_process_data
  * @brief      Process incoming/outcoming data.
  * @param      system_status_word: status word to be sent to the master.
  * @retval     Next reaction (if necessary).
  *****************************************************************************/
ret_value_t slave_i2c_process_data(uint16_t system_status_word);

#endif // SLAVE_I2C_DEVICE_H

