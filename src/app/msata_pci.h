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

struct msata_pci_ind {
    uint8_t card_det:   1; /* 0: some card detected, 1: no card inserted */
    uint8_t msata_ind:  1; /* 0: msata card inserted, 1: pcie card inserted */
    uint8_t msata_led:  1; /* card activity */
};

extern struct msata_pci_ind msata_pci_status;

/*******************************************************************************
  * @function   msata_pci_indication_config
  * @brief      Main configuration function for mSATA and PCIe indication.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void msata_pci_indication_config(void);

#endif // MSATA_PCI_H

