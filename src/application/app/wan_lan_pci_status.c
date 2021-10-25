/**
 ******************************************************************************
 * @file    wan_lan_pci_status.c
 * @author  CZ.NIC, z.s.p.o.
 * @date    10-August-2015
 * @brief   Driver for WAN, LAN and PCIe status indication.
 ******************************************************************************
 ******************************************************************************
 **/
#include "wan_lan_pci_status.h"
#include "led_driver.h"

enum lan_led_masks {
    LAN_LED_MASK        = 0x1947,
    LAN_R0_MASK         = 0x0001,
    LAN_R1_MASK         = 0x0002,
    LAN_R2_MASK         = 0x0004,
    LAN_C0_MASK         = 0x0040,
    LAN_C1_MASK         = 0x0100,
    LAN_C2_MASK         = 0x0800,
    LAN_C3_MASK         = 0x1000,
};

/*******************************************************************************
  * @function   wan_lan_pci_io_config
  * @brief      GPIO configuration for WAN, LAN and PCIe indication signals.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
static void wan_lan_pci_io_config(void)
{
    /* GPIO Periph clock enable */
    rcu_periph_clock_enable(PCI_PLED0_PIN_PERIPH_CLOCK);
    rcu_periph_clock_enable(PCI_LLED1_PIN_PERIPH_CLOCK);
    rcu_periph_clock_enable(PCI_PLED1_PIN_PERIPH_CLOCK);
    rcu_periph_clock_enable(PCI_LLED2_PIN_PERIPH_CLOCK);
    rcu_periph_clock_enable(PCI_PLED2_PIN_PERIPH_CLOCK);
    rcu_periph_clock_enable(WAN_LED0_PIN_PERIPH_CLOCK);
    rcu_periph_clock_enable(SFP_DIS_PIN_PERIPH_CLOCK);
    rcu_periph_clock_enable(R0_P0_LED_PIN_PERIPH_CLOCK);

    /* for compatibility with older versions of board */
    gpio_mode_set(SFP_DIS_PIN_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, SFP_DIS_PIN);
    gpio_output_options_set(SFP_DIS_PIN_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_2MHZ, SFP_DIS_PIN);
    gpio_bit_reset(SFP_DIS_PIN_PORT, SFP_DIS_PIN);

    /* PCIe LED pins */
    gpio_mode_set(PCI_LLED2_PIN_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, PCI_LLED2_PIN);
    gpio_mode_set(PCI_LLED1_PIN_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, PCI_LLED1_PIN);
    gpio_mode_set(PCI_PLED0_PIN_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, PCI_PLED0_PIN);
    gpio_mode_set(PCI_PLED1_PIN_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, PCI_PLED1_PIN);
    gpio_mode_set(PCI_PLED2_PIN_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, PCI_PLED2_PIN);


    /* WAN LED pins */
    gpio_mode_set(WAN_LED0_PIN_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, WAN_LED0_PIN);

    /* LAN LED input pins */
    gpio_mode_set(R0_P0_LED_PIN_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, R0_P0_LED_PIN);
    gpio_mode_set(R1_P1_LED_PIN_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, R1_P1_LED_PIN);
    gpio_mode_set(R2_P2_LED_PIN_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, R2_P2_LED_PIN);
    gpio_mode_set(C0_P3_LED_PIN_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, C0_P3_LED_PIN);
    gpio_mode_set(C1_LED_PIN_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, C1_LED_PIN);
    gpio_mode_set(C2_P4_LED_PIN_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, C2_P4_LED_PIN);
    gpio_mode_set(C3_P5_LED_PIN_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, C3_P5_LED_PIN);
}


/*******************************************************************************
  * @function   wan_lan_pci_config
  * @brief      Main configuration function for WAN, LAN and PCIe status indication.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void wan_lan_pci_config(void)
{
    wan_lan_pci_io_config();
}

/*******************************************************************************
  * @function   wan_led_activity
  * @brief      Toggle WAN LED according to the WAN activity.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void wan_led_activity(void)
{
    uint8_t led0_status;
    struct led_rgb *rgb_leds = leds;

    if (rgb_leds[WAN_LED].led_mode == LED_DEFAULT_MODE)
    {
        led0_status = (uint8_t)(gpio_input_bit_get(WAN_LED0_PIN_PORT, WAN_LED0_PIN));

        if (led0_status == 0)
        {
            rgb_leds[WAN_LED].led_state_default = LED_ON;
        }
        else
        {
            rgb_leds[WAN_LED].led_state_default = LED_OFF;
        }
    }
}

/*******************************************************************************
  * @function   pci_led_activity
  * @brief      Toggle PCIe LED according to the PCIe activity.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void pci_led_activity(void)
{
    uint8_t pcie_led1, pcie_pled1, pcie_led2, pcie_pled2;
    struct led_rgb *rgb_leds = leds;

    pcie_led2 = (uint8_t)(gpio_input_bit_get(PCI_LLED2_PIN_PORT, PCI_LLED2_PIN));
    pcie_led1 = (uint8_t)(gpio_input_bit_get(PCI_LLED1_PIN_PORT, PCI_LLED1_PIN));
    pcie_pled1 = (uint8_t)(gpio_input_bit_get(PCI_PLED1_PIN_PORT, PCI_PLED1_PIN));
    pcie_pled2 = (uint8_t)(gpio_input_bit_get(PCI_PLED2_PIN_PORT, PCI_PLED2_PIN));

    if (rgb_leds[PCI2_LED].led_mode == LED_DEFAULT_MODE)
    {
        if((pcie_led2 == 0) || (pcie_pled2 == 0))
        {
            rgb_leds[PCI2_LED].led_state_default = LED_ON;
        }
        else
        {
            rgb_leds[PCI2_LED].led_state_default = LED_OFF;
        }
    }

    if (rgb_leds[PCI1_LED].led_mode == LED_DEFAULT_MODE)
    {
        if ((pcie_led1 == 0) || (pcie_pled1 == 0))
        {
            rgb_leds[PCI1_LED].led_state_default = LED_ON;
        }
        else
        {
            rgb_leds[PCI1_LED].led_state_default = LED_OFF;
        }
    }
}

/*******************************************************************************
  * @function   lan_led_activity
  * @brief      Toggle LAN LEDs according to the LAN status.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void lan_led_activity(void)
{
    uint16_t lan_led;
    struct led_rgb *rgb_leds = leds;

    lan_led = gpio_input_port_get(LAN_LED_PORT) & LAN_LED_MASK;

    if (rgb_leds[LAN0_LED].led_mode == LED_DEFAULT_MODE)
    {
        if((lan_led & LAN_R0_MASK) == 0)
        {
            if (lan_led & LAN_C0_MASK)
                rgb_leds[LAN0_LED].led_state_default = LED_ON;
        }
        else
        {
            if (lan_led & LAN_C0_MASK)
                rgb_leds[LAN0_LED].led_state_default = LED_OFF;
        }
    }

    if (rgb_leds[LAN1_LED].led_mode == LED_DEFAULT_MODE)
    {
        if((lan_led & LAN_R0_MASK) == 0)
        {
            if (lan_led & LAN_C1_MASK)
                rgb_leds[LAN1_LED].led_state_default = LED_ON;
        }
        else
        {
            if (lan_led & LAN_C1_MASK)
                rgb_leds[LAN1_LED].led_state_default = LED_OFF;
        }
    }

    if (rgb_leds[LAN2_LED].led_mode == LED_DEFAULT_MODE)
    {
        if((lan_led & LAN_R1_MASK) == 0)
        {
            if (lan_led & LAN_C0_MASK)
                rgb_leds[LAN2_LED].led_state_default = LED_ON;
        }
        else
        {
            if (lan_led & LAN_C0_MASK)
                rgb_leds[LAN2_LED].led_state_default = LED_OFF;
        }
    }

    if (rgb_leds[LAN3_LED].led_mode == LED_DEFAULT_MODE)
    {
        if((lan_led & LAN_R1_MASK) == 0)
        {
            if (lan_led & LAN_C1_MASK)
                rgb_leds[LAN3_LED].led_state_default = LED_ON;
        }
        else
        {
            if (lan_led & LAN_C1_MASK)
                rgb_leds[LAN3_LED].led_state_default = LED_OFF;
        }
    }

    if (rgb_leds[LAN4_LED].led_mode == LED_DEFAULT_MODE)
    {
        if((lan_led & LAN_R2_MASK) == 0)
        {
            if (lan_led & LAN_C0_MASK)
                rgb_leds[LAN4_LED].led_state_default = LED_ON;
        }
        else
        {
            if (lan_led & LAN_C0_MASK)
                rgb_leds[LAN4_LED].led_state_default = LED_OFF;
        }
    }
}
