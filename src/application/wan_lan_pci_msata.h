#ifndef WAN_LAN_PCI_MSATA_H
#define WAN_LAN_PCI_MSATA_H

#include "compiler.h"

/*******************************************************************************
  * @function   wan_lan_pci_msata_config
  * @brief      Configuration for WAN, LAN, PCIe and mSATA status indication.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void wan_lan_pci_msata_config(void);

/*******************************************************************************
  * @function   msata_pci_activity
  * @brief      Toggle LED according to the activity of the connected card.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void msata_pci_activity(void);

/*******************************************************************************
  * @function   wan_led_activity
  * @brief      Toggle WAN LED according to the WAN activity.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void wan_led_activity(void);

/*******************************************************************************
  * @function   pci_led_activity
  * @brief      Toggle PCIe LED according to the PCIe activity.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void pci_led_activity(void);

/*******************************************************************************
  * @function   lan_led_activity
  * @brief      Toggle LAN LEDs according to the LAN status.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void lan_led_activity(void);

#endif /* WAN_LAN_PCI_MSATA_H */
