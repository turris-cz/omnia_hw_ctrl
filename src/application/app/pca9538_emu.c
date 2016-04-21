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
#include "msata_pci.h"
#include "pca9538_emu.h"

#define REGISTER_LENGTH               8

#define PCA9538_REG_CONFIG_DEFAULT       0xFF /* default value for config reg */
#define PCA9538_REG_POLARITY_DEFAULT     0x00 /* default value for polarity reg */
#define PCA9538_REG_OUTPUT_DEFAULT       0xFF /* default value for output reg */

enum pca9538_config_reg {
    CONFIG_OUTPUT                   = 0x00,
    CONFIG_INPUT                    = 0x01
};

enum pca9538_pin_mask {
    PIN0_MASK                       = 0x01, /* MSATALED */
    PIN1_MASK                       = 0x02, /* PCI_PLED0 */
    PIN2_MASK                       = 0x04, /* PCI_LLED1 */
    PIN3_MASK                       = 0x08, /* PCI_PLED1 */
    PIN4_MASK                       = 0x10, /* PCI_LLED2 */
    PIN5_MASK                       = 0x20, /* PCI_PLED1 */
    PIN6_MASK                       = 0x40, /* not used */
    PIN7_MASK                       = 0x80  /* not used */
};

enum pca9538_pin_number {
    PIN0                            = 0x00,
    PIN1                            = 0x01,
    PIN2                            = 0x02,
    PIN3                            = 0x03,
    PIN4                            = 0x04,
    PIN5                            = 0x05,
    PIN6                            = 0x06,
    PIN7                            = 0x07
};

struct st_pca9538 pca9538;

/*******************************************************************************
  * @function   pca9538_read_input
  * @brief      Read byte from input port.
  * @param      None.
  * @retval     Input port state.
  *****************************************************************************/
uint8_t pca9538_read_input(void)
{
    uint8_t input_port = 0, input = 0, pin;
    struct st_pca9538 *expander = &pca9538;

    for(pin = PIN0; pin < REGISTER_LENGTH; pin++)
    {
        switch(pin)
        {
            case PIN0:
            {
                input = GPIO_ReadInputDataBit(MSATALED_PIN_PORT, MSATALED_PIN);

                /* check setting in polarity inversion register */
                if (expander->pol_inv_reg & PIN0_MASK)
                {
                    input_port |= PIN0_MASK & (~input);
                }
                else
                {
                    input_port |= input;
                }
            } break;

            case PIN1:
            {
                input = GPIO_ReadInputDataBit(PCI_PLED0_PIN_PORT, PCI_PLED0_PIN);

                if (expander->pol_inv_reg & PIN1_MASK)
                {
                    input_port |= PIN1_MASK & (~(input << PIN1));
                }
                else
                {
                    input_port |= (input << PIN1);
                }
            } break;

            case PIN2:
            {
                input = GPIO_ReadInputDataBit(PCI_LLED1_PIN_PORT, PCI_LLED1_PIN);

                if (expander->pol_inv_reg & PIN2_MASK)
                {
                    input_port |= PIN2_MASK & (~(input << PIN2));
                }
                else
                {
                    input_port |= (input << PIN2);
                }
            } break;

            case PIN3:
            {
                input = GPIO_ReadInputDataBit(PCI_PLED1_PIN_PORT, PCI_PLED1_PIN);

                if (expander->pol_inv_reg & PIN3_MASK)
                {
                    input_port |= PIN3_MASK & (~(input << PIN3));
                }
                else
                {
                    input_port |= (input << PIN3);
                }
            } break;

            case PIN4:
            {
                input = GPIO_ReadInputDataBit(PCI_LLED2_PIN_PORT, PCI_LLED2_PIN);

                if (expander->pol_inv_reg & PIN4_MASK)
                {
                    input_port |= PIN4_MASK & (~(input << PIN4));
                }
                else
                {
                    input_port |= (input << PIN4);
                }
            } break;

            case PIN5:
            {
                input = GPIO_ReadInputDataBit(PCI_PLED2_PIN_PORT, PCI_PLED2_PIN);

                if (expander->pol_inv_reg & PIN5_MASK)
                {
                    input_port |= PIN5_MASK & (~(input << PIN5));
                }
                else
                {
                    input_port |= (input << PIN5);
                }
            } break;

            default:
                break;
        }
    }

    return input_port;
}

/*******************************************************************************
  * @function   pca9538_write_output
  * @brief      Write byte to output register.
  * @param      output_reg: value to be written to output register.
  * @retval     None.
  *****************************************************************************/
void pca9538_write_output(uint8_t output_reg)
{
    /* no write posibility at the moment */
    return;
}

/*******************************************************************************
  * @function   pca9538_set_polarity_inv
  * @brief      Write byte to polarity inversion register.
  * @param      polarity: value to be written to polarity inversion register.
  * @retval     None.
  *****************************************************************************/
void pca9538_set_polarity_inv(uint8_t polarity)
{
    struct st_pca9538 *expander = &pca9538;

    expander->pol_inv_reg = polarity;
}

/*******************************************************************************
  * @function   pca9538_set_config
  * @brief      Write byte to configuration register and configure pins.
  * @param      config_reg: value to be written to configuration register.
  * @retval     None.
  *****************************************************************************/
void pca9538_set_config(uint8_t config_reg)
{
    struct st_pca9538 *expander = &pca9538;
    uint8_t pin;
    uint8_t config_reg_masked, mask = 0x01;
    GPIO_InitTypeDef  GPIO_InitStructure;

    expander->config_reg = config_reg;

    for (pin = PIN0; pin < REGISTER_LENGTH; pin++)
    {
        config_reg_masked = config_reg & mask;

        if (config_reg_masked == CONFIG_OUTPUT) /* pin configured as output */
        {
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
            GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
            GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_1;
            GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

            /* no output pins used at the moment */
        }
        else /* pin configured as input */
        {
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
            GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_1;
            GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

            switch(pin)
            {
                case PIN0:
                {
                    GPIO_InitStructure.GPIO_Pin = MSATALED_PIN;
                    GPIO_Init(MSATALED_PIN_PORT, &GPIO_InitStructure);
                } break;

                case PIN1:
                {
                    GPIO_InitStructure.GPIO_Pin = PCI_PLED0_PIN;
                    GPIO_Init(PCI_PLED0_PIN_PORT, &GPIO_InitStructure);
                } break;

                case PIN2:
                {
                    GPIO_InitStructure.GPIO_Pin = PCI_LLED1_PIN;
                    GPIO_Init(PCI_LLED1_PIN_PORT, &GPIO_InitStructure);
                } break;

                case PIN3:
                {
                    GPIO_InitStructure.GPIO_Pin = PCI_PLED1_PIN;
                    GPIO_Init(PCI_PLED1_PIN_PORT, &GPIO_InitStructure);
                } break;

                case PIN4:
                {
                    GPIO_InitStructure.GPIO_Pin = PCI_LLED2_PIN;
                    GPIO_Init(PCI_LLED2_PIN_PORT, &GPIO_InitStructure);
                } break;

                case PIN5:
                {
                    GPIO_InitStructure.GPIO_Pin = PCI_PLED2_PIN;
                    GPIO_Init(PCI_PLED2_PIN_PORT, &GPIO_InitStructure);
                } break;

                default:
                    break;
            }
        }

        mask <<= 1;
    }
}

/*******************************************************************************
  * @function   pca9538_reset
  * @brief      Set register of emulated PCA9538 to default values.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void pca9538_reset(void)
{
    pca9538_set_config(PCA9538_REG_CONFIG_DEFAULT); /* all pins are input */
    pca9538_set_polarity_inv(PCA9538_REG_POLARITY_DEFAULT); /* no polarity inversion */
}
