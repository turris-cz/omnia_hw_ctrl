/**
 ******************************************************************************
 * @file    wan_lan_pci_msata.h
 * @author  CZ.NIC, z.s.p.o.
 * @date    10-August-2015
 * @brief   Header file for driver of WAN, LAN and PCIe status indication.
 ******************************************************************************
 ******************************************************************************
 **/
#ifndef WAN_LAN_PCI_MSATA_H
#define WAN_LAN_PCI_MSATA_H

#include "gpio.h"

/* PCIe status LED */
#define PCI_PLED0_PIN		PIN(F, 5, OMNIA_BOARD_REVISION < 32)
#define PCI_LLED1_PIN		PIN(C, 2)
#define PCI_PLED1_PIN		PIN(A, 10, OMNIA_BOARD_REVISION < 32 && !DBG_ENABLE)
#define PCI_LLED2_PIN		PIN(C, 1)
#define PCI_PLED2_PIN		PIN(F, 4, OMNIA_BOARD_REVISION < 32)

/* WAN LED */
#define WAN_LED0_PIN		PIN(F, 0)
#define WAN_LED1_PIN		PIN(F, 1, OMNIA_BOARD_REVISION >= 32)

/* LAN LED */
#define LAN_LED_PORT		PORT_A
#define R0_P0_LED_PIN		PIN(A, 0)
#define R1_P1_LED_PIN		PIN(A, 1)
#define R2_P2_LED_PIN		PIN(A, 2)
#define C0_P3_LED_PIN		PIN(A, 6)
#define C1_LED_PIN		PIN(A, 8)
#define C2_P4_LED_PIN		PIN(A, 11)
#define C3_P5_LED_PIN		PIN(A, 12)

/* mSATA/PCI detection and LED */
#define CARD_DET_PIN		PIN(A, 9, !DBG_ENABLE)
#define MSATALED_PIN		PIN(A, 15)
#define MSATAIND_PIN		PIN(C, 14)

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
  * @function   msata_pci_card_detection
  * @brief      Detect inserted card (whether is a card inserted or not)
  * @param      None.
  * @retval     1 - a card inserted, 0 - no card inserted.
  *****************************************************************************/
bool msata_pci_card_detection(void);

/*******************************************************************************
  * @function   msata_pci_type_card_detection
  * @brief      Detect a type of inserted card - mSATA or miniPCIe
  * @param      None.
  * @retval     1 - mSATA card inserted, 0 - miniPCIe card inserted.
  *****************************************************************************/
bool msata_pci_type_card_detection(void);

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