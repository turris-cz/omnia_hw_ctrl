/**
 ******************************************************************************
 * @file    msata_pci.c
 * @author  CZ.NIC, z.s.p.o.
 * @date    27-October-2021
 * @brief   Driver for PCIe and mSATA indication.
 ******************************************************************************
 ******************************************************************************
 **/
/* Includes ------------------------------------------------------------------*/
#include "msata_pci.h"
#include "led_driver.h"
#include "wan_lan_pci_status.h"
#include "debug_serial.h"

/*******************************************************************************
  * @function   msata_pci_io_config
  * @brief      GPIO configuration for mSATA and PCIe indication signals.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
static void msata_pci_io_config(void)
{
    rcu_periph_clock_enable(CARD_DET_PIN_PERIPH_CLOCK);
    rcu_periph_clock_enable(MSATALED_PIN_PERIPH_CLOCK);
    rcu_periph_clock_enable(MSATAIND_PIN_PERIPH_CLOCK);

    /* Input signals */

    /*
     * CARD_DET pin is also used as MCU's UART TX pin, so only configure it as
     * GPIO if debugging is disabled
     */
    if (!DBG_ENABLE)
        gpio_mode_set(CARD_DET_PIN_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, CARD_DET_PIN);
    gpio_mode_set(MSATALED_PIN_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, MSATALED_PIN);
    gpio_mode_set(MSATAIND_PIN_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, MSATAIND_PIN);
}

/*******************************************************************************
  * @function   msata_pci_indication_config
  * @brief      Main configuration function for mSATA and PCIe status indication.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void msata_pci_indication_config(void)
{
    msata_pci_io_config();
}

/*******************************************************************************
  * @function   msata_pci_activity
  * @brief      Toggle LED according to the activity of the connected card.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void msata_pci_activity(void)
{
    uint8_t msata_pci_activity, pci_pled0;
    struct led_rgb *rgb_leds = leds;

    msata_pci_activity = (uint8_t)(gpio_input_bit_get(MSATALED_PIN_PORT, MSATALED_PIN));
    pci_pled0 = (uint8_t)(gpio_input_bit_get(PCI_PLED0_PIN_PORT, PCI_PLED0_PIN));

    if (rgb_leds[MSATA_PCI_LED].led_mode == LED_DEFAULT_MODE)
    {
        if (!msata_pci_activity || !pci_pled0)
        {
            rgb_leds[MSATA_PCI_LED].led_state_default = LED_ON;
        }
        else
        {
            rgb_leds[MSATA_PCI_LED].led_state_default = LED_OFF;
        }
    }
}

/*******************************************************************************
  * @function   msata_pci_card_detection
  * @brief      Detect inserted card (whether a card is inserted or not)
  * @param      None.
  * @retval     1 - a card inserted, 0 - no card inserted.
  *****************************************************************************/
inline uint8_t msata_pci_card_detection(void)
{
    /* inverted due to the HW connection
    HW connection: 1 = no card inserted, 0 = card inserted */
    return (!(gpio_input_bit_get(CARD_DET_PIN_PORT, CARD_DET_PIN)));
}

/*******************************************************************************
  * @function   msata_pci_type_card_detection
  * @brief      Detect a type of inserted card - mSATA or miniPCIe
  * @param      None.
  * @retval     1 - mSATA card inserted, 0 - miniPCIe card inserted.
  *****************************************************************************/
inline uint8_t msata_pci_type_card_detection(void)
{
    return ((uint8_t)(gpio_input_bit_get(MSATAIND_PIN_PORT, MSATAIND_PIN)));
}
