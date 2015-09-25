/**
 ******************************************************************************
 * @file    msata_pci.h
 * @author  CZ.NIC, z.s.p.o.
 * @date    06-August-2015
 * @brief   Header file for PCIe and mSATA driver.
 ******************************************************************************
 ******************************************************************************
 **/
#ifndef MSATA_PCI_H
#define MSATA_PCI_H

#define CARD_DET_PIN_PERIPH_CLOCK           RCC_AHBPeriph_GPIOA
#define CARD_DET_PIN_PORT                   GPIOA
#define CARD_DET_PIN                        GPIO_Pin_9
#define CARD_DET_PIN_EXTIPORT               EXTI_PortSourceGPIOA
#define CARD_DET_PIN_EXTIPINSOURCE          EXTI_PinSource9
#define CARD_DET_PIN_EXTILINE               EXTI_Line9

#define MSATALED_PIN_PERIPH_CLOCK           RCC_AHBPeriph_GPIOA
#define MSATALED_PIN_PORT                   GPIOA
#define MSATALED_PIN                        GPIO_Pin_15
#define MSATALED_PIN_EXTIPORT               EXTI_PortSourceGPIOA
#define MSATALED_PIN_EXTIPINSOURCE          EXTI_PinSource15
#define MSATALED_PIN_EXTILINE               EXTI_Line15

#define MSATAIND_PIN_PERIPH_CLOCK           RCC_AHBPeriph_GPIOC
#define MSATAIND_PIN_PORT                   GPIOC
#define MSATAIND_PIN                        GPIO_Pin_14
#define MSATAIND_PIN_EXTIPORT               EXTI_PortSourceGPIOC
#define MSATAIND_PIN_EXTIPINSOURCE          EXTI_PinSource14
#define MSATAIND_PIN_EXTILINE               EXTI_Line14

struct msata_pci_ind {
    uint8_t card_det:   1; /* 0: some card detected, 1: no card inserted */
    uint8_t msata_ind:  1; /* 0: msata card inserted, 1: pcie card inserted */
};

extern struct msata_pci_ind msata_pci_status;

/*******************************************************************************
  * @function   msata_pci_indication_config
  * @brief      Main configuration function for mSATA and PCIe indication.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void msata_pci_indication_config(void);

/*******************************************************************************
  * @function   msata_pci_activity_handler
  * @brief      Toggle LED according to the activity of the connected card.
  *             Called in EXTI interrupt handler.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void msata_pci_activity_handler(void);

/*******************************************************************************
  * @function   msata_pci_card_detection
  * @brief      Detect inserted card (whether is a card inserted or not)
  * @param      None.
  * @retval     1 - a card inserted, 0 - no card inserted.
  *****************************************************************************/
__inline__ uint8_t msata_pci_card_detection(void);

/*******************************************************************************
  * @function   msata_pci_type_card_detection
  * @brief      Detect a type of inserted card - mSATA or miniPCIe
  * @param      None.
  * @retval     1 - mSATA card inserted, 0 - miniPCIe card inserted.
  *****************************************************************************/
__inline__ uint8_t msata_pci_type_card_detection(void);

#endif // MSATA_PCI_H

