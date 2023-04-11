#include "wan_lan_pci_msata.h"
#include "led_driver.h"
#include "pin_defs.h"
#include "debug.h"

/*******************************************************************************
  * @function   wan_lan_pci_msata_config
  * @brief      Configuration for WAN, LAN, PCIe and mSATA status indication.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void wan_lan_pci_msata_config(void)
{
	gpio_init_inputs(pin_pullup,
			 PCI_PLED0_PIN, PCI_PLED1_PIN, PCI_PLED2_PIN,
			 PCI_LLED1_PIN, PCI_LLED2_PIN, WAN_LED0_PIN,
			 WAN_LED1_PIN, CARD_DET_PIN, MSATA_LED_PIN,
			 MSATA_IND_PIN);

	gpio_init_inputs(pin_nopull,
			 R0_P0_LED_PIN, R1_P1_LED_PIN, R2_P2_LED_PIN,
			 C0_P3_LED_PIN, C1_LED_PIN, C2_P4_LED_PIN, C3_P5_LED_PIN);
}

/*******************************************************************************
  * @function   msata_pci_activity
  * @brief      Toggle LED according to the activity of the connected card.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void msata_pci_activity(void)
{
	led_set_state_nocommit(MSATA_PCI_LED, !gpio_read(MSATA_LED_PIN));
}

/*******************************************************************************
  * @function   wan_led_activity
  * @brief      Toggle WAN LED according to the WAN activity.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void wan_led_activity(void)
{
	led_set_state_nocommit(WAN_LED, !gpio_read(WAN_LED0_PIN));
}

/*******************************************************************************
  * @function   pci_led_activity
  * @brief      Toggle PCIe LED according to the PCIe activity.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void pci_led_activity(void)
{
	uint8_t pcie_led1, pcie_led2;

	pcie_led2 = gpio_read(PCI_LLED2_PIN);
	pcie_led1 = gpio_read(PCI_LLED1_PIN);

	if (OMNIA_BOARD_REVISION < 32) {
		uint8_t pcie_pled1, pcie_pled2;

		pcie_pled1 = gpio_read(PCI_PLED1_PIN);
		pcie_pled2 = gpio_read(PCI_PLED2_PIN);

		pcie_led1 = pcie_led1 && pcie_pled1;
		pcie_led2 = pcie_led2 && pcie_pled2;
	}

	led_set_state_nocommit(PCI2_LED, !pcie_led2);
	led_set_state_nocommit(PCI1_LED, !pcie_led1);
}

/*******************************************************************************
  * @function   lan_led_activity
  * @brief      Toggle LAN LEDs according to the LAN status.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void lan_led_activity(void)
{
	bool c0, c1, nr0, nr1, nr2;

	lan_led_pins_read(&c0, &c1, &nr0, &nr1, &nr2);

	if (c0) {
		led_set_state_nocommit(LAN0_LED, nr0);
		led_set_state_nocommit(LAN2_LED, nr1);
		led_set_state_nocommit(LAN4_LED, nr2);
	}

	if (c1) {
		led_set_state_nocommit(LAN1_LED, nr0);
		led_set_state_nocommit(LAN3_LED, nr1);
	}
}
