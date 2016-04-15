/**
 ******************************************************************************
 * @file    flash.h
 * @author  CZ.NIC, z.s.p.o.
 * @date    13-April-2016
 * @brief   Header for driver for writing onto flash memory (used by IAP).
 ******************************************************************************/

#ifndef __FLASH_H
#define __FLASH_H

#define USER_FLASH_LAST_PAGE_ADDRESS  0x0800D800
#define USER_FLASH_END_ADDRESS        0x0800DFFF /* 64 KBytes */
#define FLASH_PAGE_SIZE               0x400      /* 1 Kbytes */

/* define the address from where user application will be loaded,
   the application address should be a start sector address */
#define APPLICATION_ADDRESS     (uint32_t)0x08004000

/* Get the number of Sector from where the user program will be loaded */
#define  FLASH_PAGE_NUMBER      (uint32_t)((APPLICATION_ADDRESS - 0x08000000) >> 12)

/* Compute the mask to test if the Flash memory, where the user program will be
   loaded, is write protected */
#define  FLASH_PROTECTED_PAGES   ((uint32_t)~((1 << FLASH_PAGE_NUMBER) - 1))

/* define the user application size */
#define USER_FLASH_SIZE   (USER_FLASH_END_ADDRESS - APPLICATION_ADDRESS + 1)

#endif /* __FLASH_H */


void flash_config(void);
uint32_t flash_erase(uint32_t start_sector);

/*******************************************************************************
  * @brief  This function write incoming data to application address.
  * @param  data: incoming data
  * @param  data_lenght: legth of incoming data
  * @retval 0: Data successfully written to Flash memory
  *         1: Error occurred while writing data in Flash memory
  *         2: Written Data in flash memory is different from expected one
  *****************************************************************************/
uint32_t flash_new_data(uint32_t* data, uint16_t data_length);
