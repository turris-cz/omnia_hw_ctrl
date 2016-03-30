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

#define PCA9538_PIN_DEFAULT_CONFIG       0xFF /* default value for config reg */

void pca9534_set_config(uint8_t reg);
void pca9534_write_output(uint8_t reg);
uint8_t pca9534_read_input(void);

#endif /* PCA9538_EMULATOR_H */
