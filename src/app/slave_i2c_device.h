/**
 ******************************************************************************
 * @file    slave_i2c_device.h
 * @author  CZ.NIC, z.s.p.o.
 * @date    18-August-2015
 * @brief   Header for I2C driver.
 ******************************************************************************
 ******************************************************************************
 **/
#ifndef SLAVE_I2C_DEVICE_H
#define SLAVE_I2C_DEVICE_H

extern uint16_t i2c_status_word;

/*
 * Bit meanings in i2c_status_word:
 *  Bit Nr. |   Meanings
 * -----------------
 *      0   |   SFP_DET     : 1 - SFP detected, 0 - SFP not detected
 *      1   |   SFP_LOS     : 1 - SFP receiver lost signal, 0 - no lost
 *      2   |   SFP_FLT     : 1 - SFP trasmitter fault, 0 - no TX fault
 *      3   |   SFP_DIS     : 1 - SFP TX disabled; 0 - SFP TX enabled
 *      4   |   CARD_DET    : 1 - mSATA/PCIe card detected, 0 - no card
 *      5   |   mSATA_IND   : 1 - mSATA card inserted, 0 - PCIe card inserted
 *      6   |   USB30_OVC   : 1 - USB3-port0 overcurrent, 0 - no overcurrent
 *      7   |   USB31_OVC   : 1 - USB3-port1 overcurrent, 0 - no overcurrent
 *      8   |   USB30_PWRON : 1 - USB3-port0 power ON, 0 - USB-port0 power off
 *      9   |   USB31_PWRON : 1 - USB3-port1 power ON, 0 - USB-port1 power off
*/
enum status_word_bits {
    SFP_DET_BIT         = 0x0001,
    SFP_LOS_BIT         = 0x0002,
    SFP_FLT_BIT         = 0x0004,
    SFP_DIS_BIT         = 0x0008,
    CARD_DET_BIT        = 0x0010,
    MSATA_IND_BIT       = 0x0020,
    USB30_OVC_BIT       = 0x0040,
    USB31_OVC_BIT       = 0x0080,
    USB30_PWRON_BIT     = 0x0100,
    USB31_PWRON_BIT     = 0x0200,
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
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void slave_i2c_process_data(void);

#endif // SLAVE_I2C_DEVICE_H

