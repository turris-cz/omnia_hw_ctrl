/**
 ******************************************************************************
 * @file    pca9538_emu.c
 * @author  CZ.NIC, z.s.p.o.
 * @date    29-March-2016
 * @brief   Emulator of PCA9538 GPIO I2C expander.
 ******************************************************************************
 ******************************************************************************
 **/

#include "stm32f0xx_conf.h"
#include "wan_lan_pci_status.h"

#define REGISTER_LENGTH             8

enum pca9538_commands {
    INPUT_PORT_REG                  = 0x00,
    OUTPUT_PORT_REG                 = 0x01,
    POLARITY_INV_REG                = 0x02,
    CONFIG_REG                      = 0x03,
};

enum pca9538_config_reg {
    CONFIG_OUTPUT                   = 0x00,
    CONFIG_INPUT                    = 0x01
};

enum pca9538_pin_mask {
    PIN0_SFP_DET_MASK               = 0x01,
    PIN1_SFP_FLT_MASK               = 0x02,
    PIN2_SFP_LOST_MASK              = 0x04,
    PIN3_SFP_DIS_MASK               = 0x08
};

typedef enum pca9538_pin_config {
    PIN0_SFP_DET                    = 0x00,
    PIN1_SFP_FLT                    = 0x01,
    PIN2_SFP_LOST                   = 0x02,
    PIN3_SFP_DIS                    = 0x03
} pin_config_t;

struct pca9538_st{
    uint8_t config_reg;
    uint8_t pol_inv_reg;
};

static struct pca9538_st pca9538;

/*******************************************************************************
  * @function   pca9534_read_input
  * @brief      Read byte from input port.
  * @param      None.
  * @retval     Input port state.
  *****************************************************************************/
uint8_t pca9534_read_input(void)
{
    uint8_t input_port = 0;
    struct pca9538_st *expander = &pca9538;

    /* check setting in polarity inversion register */
    if (expander->pol_inv_reg & PIN0_SFP_DET_MASK)
    {
        input_port |= PIN0_SFP_DET_MASK & (~(wan_sfp_connector_detection()));
    }
    else
    {
        input_port |= wan_sfp_connector_detection();
    }

    if (expander->pol_inv_reg & PIN1_SFP_FLT_MASK)
    {
        input_port |= PIN1_SFP_FLT_MASK & (~(wan_sfp_fault_detection() << 1));
    }
    else
    {
        input_port |= (wan_sfp_fault_detection() << 1);
    }

    if (expander->pol_inv_reg & PIN2_SFP_LOST_MASK)
    {
        input_port |= PIN2_SFP_LOST_MASK & (~(wan_sfp_lost_detection() << 2));
    }
    else
    {
        input_port |= (wan_sfp_lost_detection() << 2);
    }

    if (expander->pol_inv_reg & PIN3_SFP_DIS_MASK)
    {
        input_port |= PIN3_SFP_DIS_MASK & (~(wan_sfp_get_tx_status() << 3));
    }
    else
    {
        input_port |= (wan_sfp_get_tx_status() << 3);
    }

    return input_port;
}

/*******************************************************************************
  * @function   pca9534_write_output
  * @brief      Write byte to output register.
  * @param      output_reg: value to be written to output register.
  * @retval     None.
  *****************************************************************************/
void pca9534_write_output(uint8_t output_reg)
{
    uint8_t pin_value, config_reg_masked, mask = 0x01;
    struct pca9538_st *expander = &pca9538;
    pin_config_t pin;

    for (pin = 0; pin < REGISTER_LENGTH; pin++)
    {
        config_reg_masked = expander->config_reg & mask;
        /* only output pins can be written */
        if((config_reg_masked) == CONFIG_OUTPUT)
        {
            pin_value = mask & output_reg;

            switch(pin)
            {
                case PIN0_SFP_DET:
                {
                    GPIO_WriteBit(SFP_DET_PIN_PORT, SFP_DET_PIN, pin_value);
                } break;

                case PIN1_SFP_FLT:
                {
                    GPIO_WriteBit(SFP_FLT_PIN_PORT, SFP_FLT_PIN, pin_value);
                } break;

                case PIN2_SFP_LOST:
                {
                    GPIO_WriteBit(SFP_LOS_PIN_PORT, SFP_LOS_PIN, pin_value);
                } break;

                case PIN3_SFP_DIS:
                {
                    GPIO_WriteBit(SFP_DIS_PIN_PORT, SFP_DIS_PIN, pin_value);
                } break;

                default:
                    break;
            }
        }
        mask <<= 1;
    }
}

/*******************************************************************************
  * @function   pca9534_set_polarity_inv
  * @brief      Write byte to polarity inversion register.
  * @param      polarity: value to be written to polarity inversion register.
  * @retval     None.
  *****************************************************************************/
void pca9534_set_polarity_inv(uint8_t polarity)
{
    struct pca9538_st *expander = &pca9538;

    expander->pol_inv_reg = polarity;
}

/*******************************************************************************
  * @function   pca9534_set_config
  * @brief      Write byte to configuration register and configure pins.
  * @param      config_reg: value to be written to configuration register.
  * @retval     None.
  *****************************************************************************/
void pca9534_set_config(uint8_t config_reg)
{
    struct pca9538_st *expander = &pca9538;
    pin_config_t pin;
    uint8_t config_reg_masked, mask = 0x01;
    GPIO_InitTypeDef  GPIO_InitStructure;

    expander->config_reg = config_reg;

    for (pin = 0; pin < REGISTER_LENGTH; pin++)
    {
        config_reg_masked = config_reg & mask;

        if (config_reg_masked == CONFIG_OUTPUT) /* pin configured as output */
        {
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
            GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
            GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_1;
            GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

            switch(pin)
            {
                case PIN0_SFP_DET:
                {
                    GPIO_InitStructure.GPIO_Pin = SFP_DET_PIN;
                    GPIO_Init(SFP_DET_PIN_PORT, &GPIO_InitStructure);
                } break;

                case PIN1_SFP_FLT:
                {
                    GPIO_InitStructure.GPIO_Pin = SFP_FLT_PIN;
                    GPIO_Init(SFP_FLT_PIN_PORT, &GPIO_InitStructure);
                } break;

                case PIN2_SFP_LOST:
                {
                    GPIO_InitStructure.GPIO_Pin = SFP_LOS_PIN;
                    GPIO_Init(SFP_LOS_PIN_PORT, &GPIO_InitStructure);
                } break;

                case PIN3_SFP_DIS:
                {
                    GPIO_InitStructure.GPIO_Pin = SFP_DIS_PIN;
                    GPIO_Init(SFP_DIS_PIN_PORT, &GPIO_InitStructure);
                } break;

                default:
                    break;
            }
        }
        else /* pin configured as input */
        {
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
            GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_1;
            GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

            switch(pin)
            {
                case PIN0_SFP_DET:
                {
                    GPIO_InitStructure.GPIO_Pin = SFP_DET_PIN;
                    GPIO_Init(SFP_DET_PIN_PORT, &GPIO_InitStructure);
                } break;

                case PIN1_SFP_FLT:
                {
                    GPIO_InitStructure.GPIO_Pin = SFP_FLT_PIN;
                    GPIO_Init(SFP_FLT_PIN_PORT, &GPIO_InitStructure);
                } break;

                case PIN2_SFP_LOST:
                {
                    GPIO_InitStructure.GPIO_Pin = SFP_LOS_PIN;
                    GPIO_Init(SFP_LOS_PIN_PORT, &GPIO_InitStructure);
                } break;

                case PIN3_SFP_DIS:
                {
                    GPIO_InitStructure.GPIO_Pin = SFP_DIS_PIN;
                    GPIO_Init(SFP_DIS_PIN_PORT, &GPIO_InitStructure);
                } break;

                default:
                    break;
            }
        }

        mask <<= 1;
    }
}
