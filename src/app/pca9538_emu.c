/**
 ******************************************************************************
 * @file    pca9534_emu.c
 * @author  CZ.NIC, z.s.p.o.
 * @date    29-March-2016
 * @brief   Emulator of PCA9534 GPIO I2C expander.
 ******************************************************************************
 ******************************************************************************
 **/

#include "stm32f0xx_conf.h"
#include "wan_lan_pci_status.h"

#define REGISTER_LENGTH             8

enum pca9534_commands {
    INPUT_PORT_REG                  = 0x00,
    OUTPUT_PORT_REG                 = 0x01,
    POLARITY_INV_REG                = 0x02,
    CONFIG_REG                      = 0x03,
};

enum pca9534_config_reg {
    CONFIG_OUTPUT                   = 0x00,
    CONFIG_INPUT                    = 0x01
};

enum pca9538_port_config {
    PIN0_SFP_DET                    = 0x01,
    PIN1_SFP_FLT                    = 0x02,
    PIN2_SFP_LOST                   = 0x04,
    PIN3_SFP_DIS                    = 0x08
};

struct pca9534_st{
    uint8_t config_reg;
    uint8_t pol_inv_reg;
};

struct pca9534_st pca9534;

/*******************************************************************************
  * @function   wan_sfp_connector_detection
  * @brief      Detect inserted SFP+ connector.
  * @param      None.
  * @retval     1 - SFP detected, 0 - SFP not detected.
  *****************************************************************************/
uint8_t pca9534_read_input(void)
{
    uint8_t input_port = 0;

    input_port |= wan_sfp_connector_detection();
    input_port |= (wan_sfp_fault_detection() << 1);
    input_port |= (wan_sfp_lost_detection() << 2);
    input_port |= (wan_sfp_get_tx_status() << 3);

    return input_port;
}

void pca9534_write_output(uint8_t reg)
{
    uint8_t idx, mask = 0x01;
    struct pca9534_st *expander = &pca9534;
    uint8_t reg_masked;

    for (idx = 0; idx < REGISTER_LENGTH; idx++)
    {
        config_reg_masked = expander->config_reg & mask;
        /* only output pins can be written */
        if((config_reg_masked) == CONFIG_OUTPUT)
        {
            reg_masked = mask & reg;

            switch(reg_masked)
            {
                case PIN0_SFP_DET:
                {
                    GPIO_WriteBit(SFP_DET_PIN_PORT, SFP_DET_PIN, reg_masked);
                } break;

                case PIN1_SFP_FLT:
                {
                    GPIO_WriteBit(SFP_FLT_PIN_PORT, SFP_FLT_PIN, reg_masked);
                } break;

                case PIN2_SFP_LOST:
                {
                    GPIO_WriteBit(SFP_LOS_PIN_PORT, SFP_LOS_PIN, reg_masked);
                } break;

                case PIN3_SFP_DIS:
                {
                    GPIO_WriteBit(SFP_DIS_PIN_PORT, SFP_DIS_PIN, reg_masked);
                } break;
            }
        }
        mask <<= idx;
    }
}

void pca9534_set_polarity_inv(void)
{

}

void pca9534_set_config(uint8_t reg)
{
    struct pca9534_st *expander = &pca9534;

    expander->config_reg = reg;
}
