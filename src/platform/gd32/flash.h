/**
 ******************************************************************************
 * @file    flash.h
 * @author  CZ.NIC, z.s.p.o.
 * @date    13-April-2016
 * @brief   Header for driver for writing onto flash memory (used by IAP).
 ******************************************************************************/

#ifndef __FLASH_H
#define __FLASH_H

#include "gd32f1x0_fmc.h"
#include "compiler.h"

#define FLASH_PAGE_SIZE		0x400

/*******************************************************************************
  * @brief  Unlocks Flash for write access
  * @param  None
  * @retval None
  *****************************************************************************/
static __force_inline void flash_init(void)
{
	/* Unlock the Program memory */
	fmc_unlock();

	/* Clear all FLASH flags */
	fmc_flag_clear(FMC_FLAG_PGERR);
	fmc_flag_clear(FMC_FLAG_WPERR);
	fmc_flag_clear(FMC_FLAG_END);
	fmc_flag_clear(FMC_FLAG_BUSY);
}

static __force_inline bool flash_erase_page(uint32_t page)
{
	return fmc_page_erase(page) == FMC_READY;
}

static __force_inline bool flash_program_word(uint32_t addr, uint32_t data)
{
	return fmc_word_program(addr, data) == FMC_READY;
}

static __force_inline bool flash_program_halfword(uint32_t addr, uint16_t data)
{
	return fmc_halfword_program(addr, data) == FMC_READY;
}

/*******************************************************************************
  * @brief  This function does an erase of all user flash area
  * @param  start_page: start of user flash area
  * @param  len: length in bytes
  * @retval true: user flash area successfully erased
  *         false: error occurred
  *****************************************************************************/
static __force_inline int flash_erase(uint32_t start_page, uint16_t len)
{
	for (uint32_t page = start_page;
	     page < start_page + len;
	     page += FLASH_PAGE_SIZE)
		if (!flash_erase_page(page))
			return false;

	return true;
}

/*******************************************************************************
  * @brief  This function writes a data buffer in flash.
  * @note   After writing data buffer, the flash content is checked.
  * @param  address: start address for writing data buffer, 4-aligned
  * @param  data: pointer on data buffer
  * @param  length: length of data buffer (unit is 8-bit word), 4-aligned
  * @retval true: Data successfully written to Flash memory
  *         false: Error occurred while writing data in Flash memory
  *****************************************************************************/
static __force_inline bool flash_write(uint32_t address, uint8_t *data, uint16_t length)
{
	uint32_t end = address + length;

	for (; address < end; address += 4, data += 4) {
		uint32_t word = (data[3] << 24) | (data[2] << 16) |
				(data[1] << 8) | data[0];

		if (!flash_program_word(address, word))
			return false; /* error writing */

		if (*(volatile uint32_t *)address != word)
			return false; /* error comparing */
	}

	return true;
}

#endif /* __FLASH_H */
