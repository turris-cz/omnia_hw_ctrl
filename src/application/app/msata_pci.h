/**
 ******************************************************************************
 * @file    msata_pci.h
 * @author  CZ.NIC, z.s.p.o.
 * @date    27-October-2021
 * @brief   Header file for PCIe and mSATA driver.
 ******************************************************************************
 ******************************************************************************
 **/
#ifndef MSATA_PCI_H
#define MSATA_PCI_H

#include "gd32f1x0_libopt.h"

#define CARD_DET_PIN_PERIPH_CLOCK           RCU_GPIOA
#define CARD_DET_PIN_PORT                   GPIOA
#define CARD_DET_PIN                        GPIO_PIN_9
/*#define CARD_DET_PIN_EXTIPORT               EXTI_PortSourceGPIOA
#define CARD_DET_PIN_EXTIPINSOURCE          EXTI_PinSource9
#define CARD_DET_PIN_EXTILINE               EXTI_Line9
*/
#define MSATALED_PIN_PERIPH_CLOCK           RCU_GPIOA
#define MSATALED_PIN_PORT                   GPIOA
#define MSATALED_PIN                        GPIO_PIN_15
/*#define MSATALED_PIN_EXTIPORT               EXTI_PortSourceGPIOA
#define MSATALED_PIN_EXTIPINSOURCE          EXTI_PinSource15
#define MSATALED_PIN_EXTILINE               EXTI_Line15*/

#define MSATAIND_PIN_PERIPH_CLOCK           RCU_GPIOC
#define MSATAIND_PIN_PORT                   GPIOC
#define MSATAIND_PIN                        GPIO_PIN_14
/*#define MSATAIND_PIN_EXTIPORT               EXTI_PortSourceGPIOC
#define MSATAIND_PIN_EXTIPINSOURCE          EXTI_PinSource14
#define MSATAIND_PIN_EXTILINE               EXTI_Line14*/

/*******************************************************************************
  * @function   msata_pci_indication_config
  * @brief      Main configuration function for mSATA and PCIe indication.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void msata_pci_indication_config(void);

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
uint8_t msata_pci_card_detection(void);

/*******************************************************************************
  * @function   msata_pci_type_card_detection
  * @brief      Detect a type of inserted card - mSATA or miniPCIe
  * @param      None.
  * @retval     1 - mSATA card inserted, 0 - miniPCIe card inserted.
  *****************************************************************************/
uint8_t msata_pci_type_card_detection(void);

#endif // MSATA_PCI_H

