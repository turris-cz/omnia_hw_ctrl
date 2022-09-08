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
    uint16_t ext_status_word;
    uint16_t ext_control_word;
    uint8_t reset_type;
    slave_i2c_states_t state;             // reported in main state machine
    uint8_t rx_data_ctr;                  // RX data counter
    uint8_t tx_data_ctr;                  // TX data counter
    uint8_t rx_buf[MAX_RX_BUFFER_SIZE];   // RX buffer
    uint8_t tx_buf[MAX_TX_BUFFER_SIZE];   // TX buffer
    uint8_t data_tx_complete         : 1; // stop flag detected - all data sent
};

extern struct st_i2c_status i2c_status;

enum status_word_bits {
    GD32_MCU_STSBIT        = 0x0001,
    MKL_MCU_STSBIT         = 0x0002,
    FEATURES_SUPPORTED_STSBIT = 0x0004,
    USER_REGULATOR_NOT_SUPPORTED_STSBIT = 0x0008,
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

enum features_bits {
    PERIPH_MCU_SUPPORTED   = 0x0001,
    EXT_CMDS_SUPPORTED     = 0x0002,
};

enum ext_status_word_bits {
    SFP_nDET_STSBIT        = 0x0001,
};

enum i2c_ext_control_mask {
    RES_MMC_MASK                        = 0x0001,
    RES_LAN_MASK                        = 0x0002,
    RES_PHY_MASK                        = 0x0004,
    PERST0_MASK                         = 0x0008,
    PERST1_MASK                         = 0x0010,
    PERST2_MASK                         = 0x0020,
    PHY_SFP_MASK                        = 0x0040,
    PHY_SFP_AUTO_MASK                   = 0x0080,
    VHV_CTRL_MASK                       = 0x0100,
};

/*
 * Bit meanings in status_word:
 *  Bit Nr. |   Meanings
 * -----------------
 *    0,1   |   MCU_TYPE        : 00 -> STM32
 *                                01 -> GD32
 *                                10 -> MKL
 *                                11 -> reserved
 *
 * Caution! STM32 and GD32 uses Atsha for security, MKL doesn't!!!!!!!!!
 * IT IS NECESSARY TO READ AND DECODE THE FIRST TWO BITS PROPERLY!
 *
 *      2   |   FEATURES_SUPPORT: 1 - get features command supported, 0 - get features command not supported
 *      3   |   USER_REG_NOT_SUP: 1 - user regulator not supported (always "1" since GD32 MCU), 0 - user regulator may be supported (old STM32 MCU)
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
 * Bit meanings in features:
 *  Bit Nr. |   Meanings
 * -----------------
 *      0   |   PERIPH_MCU      : 1 - resets (eMMC, PHY, switch, PCIe), SerDes switch (PHY vs SFP cage) and VHV control are connected to MCU
 *                                    (available to set via CMD_EXT_CONTROL command)
 *                                0 - otherwise
 *      1   |   EXT_CMDS        : 1 - extended control and status commands are available, 0 - otherwise
 *  2..15   |   reserved
*/

/*
 * Bit meanings in ext_status_word:
 *  Bit Nr. |   Meanings
 * -----------------
 *      0   |   SFP_nDET        : 1 - no SFP detected, 0 - SFP detected
 *  1..15   |   reserved
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
 * Bit meanings in extended control byte:
 *  Bit Nr. |   Meanings
 * -----------------
 *      0   |   RES_MMC   : 1 - reset of MMC, 0 - no reset
 *      1   |   RES_LAN   : 1 - reset of LAN switch, 0 - no reset
 *      2   |   RES_PHY   : 1 - reset of PHY WAN, 0 - no reset
 *      3   |   PERST0    : 1 - reset of PCIE0, 0 - no reset
 *      4   |   PERST1    : 1 - reset of PCIE1, 0 - no reset
 *      5   |   PERST2    : 1 - reset of PCIE2, 0 - no reset
 *      6   |   PHY_SFP   : 1 - PHY WAN mode, 0 - SFP WAN mode
 *      7   |   PHY_SFP_AUTO : 1 - automatically switch between PHY and SFP WAN modes
 *                             0 - PHY/SFP WAN mode determined by value written to PHY_SFP bit
 *      8   |   VHV_CTRL  : 1 - VHV control not active, 0 - VHV control voltage active
 *  9..15   |   reserved
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

