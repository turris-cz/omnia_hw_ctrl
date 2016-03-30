/**
 ******************************************************************************
 * @file    pca9538_emu.h
 * @author  CZ.NIC, z.s.p.o.
 * @date    29-March-2016
 * @brief   Header for emulation of PCA9538 expander.
 ******************************************************************************
 ******************************************************************************
 **/
#ifndef PCA9538_EMULATOR_H
#define PCA9538_EMULATOR_H

enum pca9538_commands {
    INPUT_PORT_REG                  = 0x00,
    OUTPUT_PORT_REG                 = 0x01,
    POLARITY_INV_REG                = 0x02,
    CONFIG_REG                      = 0x03,
};

/*******************************************************************************
  * @function   pca9538_set_config
  * @brief      Write byte to configuration register and configure pins.
  * @param      config_reg: value to be written to configuration register.
  * @retval     None.
  *****************************************************************************/
void pca9538_set_config(uint8_t config_reg);

/*******************************************************************************
  * @function   pca9538_set_polarity_inv
  * @brief      Write byte to polarity inversion register.
  * @param      polarity: value to be written to polarity inversion register.
  * @retval     None.
  *****************************************************************************/
void pca9538_set_polarity_inv(uint8_t polarity);

/*******************************************************************************
  * @function   pca9538_write_output
  * @brief      Write byte to output register.
  * @param      output_reg: value to be written to output register.
  * @retval     None.
  *****************************************************************************/
void pca9538_write_output(uint8_t output_reg);

/*******************************************************************************
  * @function   pca9538_read_input
  * @brief      Read byte from input port.
  * @param      None.
  * @retval     Input port state.
  *****************************************************************************/
uint8_t pca9538_read_input(void);

/*******************************************************************************
  * @function   pca9538_reset
  * @brief      Set register of emulated PCA9538 to default values.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void pca9538_reset(void);

#endif /* PCA9538_EMULATOR_H */
