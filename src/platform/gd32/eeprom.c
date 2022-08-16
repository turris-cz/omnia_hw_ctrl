/**
  ******************************************************************************
  * @file    STM32F0xx_EEPROM_Emulation/src/eeprom.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    29-May-2012
  * @brief   This file provides all the EEPROM emulation firmware functions.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/** @addtogroup STM32F0xx_EEPROM_Emulation
  * @{
  */

/* Includes ------------------------------------------------------------------*/
#include "eeprom.h"
#include "debug.h"
#include "gd32f1x0_fmc.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Global variable used to store variable value in read sequence */
static uint16_t DataVar = 0;

/* Virtual address defined by the user: 0xFFFF value is prohibited */
static uint16_t VirtAddVarTab[NB_OF_VAR] = {WDG_VIRT_ADDR, RESET_VIRT_ADDR};

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
static fmc_state_enum EE_Format(void);
static uint16_t EE_VerifyPageFullWriteVariable(uint16_t VirtAddress, uint16_t Data);
static uint16_t EE_PageTransfer(uint16_t VirtAddress, uint16_t Data);
static uint16_t EE_FindValidPage(uint8_t Operation);

/**
  * @brief  Restore the pages to a known good state in case of page's status
  *   corruption after a power loss.
  * @param  None.
  * @retval - Flash error code: on write Flash error
  *         - FLASH_COMPLETE: on success
  */
uint16_t EE_Init(void)
{
  uint16_t PageStatus0 = 6, PageStatus1 = 6;
  uint16_t VarIdx = 0;
  uint16_t EepromStatus = 0, ReadStatus = 0;
  int16_t x = -1;
  fmc_state_enum fmc_state;

  /* Get Page0 status */
  PageStatus0 = (*(__IO uint16_t*)PAGE0_BASE_ADDRESS);
  /* Get Page1 status */
  PageStatus1 = (*(__IO uint16_t*)PAGE1_BASE_ADDRESS);

  /* Check for invalid header states and repair if necessary */
  switch (PageStatus0)
  {
    case ERASED:
      if (PageStatus1 == VALID_PAGE) /* Page0 erased, Page1 valid */
      {
        /* Erase Page0 */
        fmc_state = fmc_page_erase(PAGE0_BASE_ADDRESS);
        /* If erase operation was failed, a Flash error code is returned */
        if (fmc_state != FMC_READY)
        {
          return fmc_state;
        }
      }
      else if (PageStatus1 == RECEIVE_DATA) /* Page0 erased, Page1 receive */
      {
        /* Erase Page0 */
        fmc_state = fmc_page_erase(PAGE0_BASE_ADDRESS);
        /* If erase operation was failed, a Flash error code is returned */
        if (fmc_state != FMC_READY)
        {
          return fmc_state;
        }
        /* Mark Page1 as valid */
        fmc_state = fmc_halfword_program(PAGE1_BASE_ADDRESS, VALID_PAGE);
        /* If program operation was failed, a Flash error code is returned */
        if (fmc_state != FMC_READY)
        {
          return fmc_state;
        }
      }
      else /* First EEPROM access (Page0&1 are erased) or invalid state -> format EEPROM */
      {
        /* Erase both Page0 and Page1 and set Page0 as valid page */
        fmc_state = EE_Format();
        /* If erase/program operation was failed, a Flash error code is returned */
        if (fmc_state != FMC_READY)
        {
          return fmc_state;
        }
      }
      break;

    case RECEIVE_DATA:
      if (PageStatus1 == VALID_PAGE) /* Page0 receive, Page1 valid */
      {
        /* Transfer data from Page1 to Page0 */
        for (VarIdx = 0; VarIdx < NB_OF_VAR; VarIdx++)
        {
          if (( *(__IO uint16_t*)(PAGE0_BASE_ADDRESS + 6)) == VirtAddVarTab[VarIdx])
          {
            x = VarIdx;
          }
          if (VarIdx != x)
          {
            /* Read the last variables' updates */
            ReadStatus = EE_ReadVariable(VirtAddVarTab[VarIdx], &DataVar);
            /* In case variable corresponding to the virtual address was found */
            if (ReadStatus != 0x1)
            {
              /* Transfer the variable to the Page0 */
              EepromStatus = EE_VerifyPageFullWriteVariable(VirtAddVarTab[VarIdx], DataVar);
              /* If program operation was failed, a Flash error code is returned */
              if (EepromStatus != FMC_READY)
              {
                return EepromStatus;
              }
            }
          }
        }
        /* Mark Page0 as valid */
        fmc_state = fmc_halfword_program(PAGE0_BASE_ADDRESS, VALID_PAGE);
        /* If program operation was failed, a Flash error code is returned */
        if (fmc_state != FMC_READY)
        {
          return fmc_state;
        }
        /* Erase Page1 */
        fmc_state = fmc_page_erase(PAGE1_BASE_ADDRESS);
        /* If erase operation was failed, a Flash error code is returned */
        if (fmc_state != FMC_READY)
        {
          return fmc_state;
        }
      }
      else if (PageStatus1 == ERASED) /* Page0 receive, Page1 erased */
      {
        /* Erase Page1 */
        fmc_state = fmc_page_erase(PAGE1_BASE_ADDRESS);
        /* If erase operation was failed, a Flash error code is returned */
        if (fmc_state != FMC_READY)
        {
          return fmc_state;
        }
        /* Mark Page0 as valid */
        fmc_state = fmc_halfword_program(PAGE0_BASE_ADDRESS, VALID_PAGE);
        /* If program operation was failed, a Flash error code is returned */
        if (fmc_state != FMC_READY)
        {
          return fmc_state;
        }
      }
      else /* Invalid state -> format eeprom */
      {
        /* Erase both Page0 and Page1 and set Page0 as valid page */
        fmc_state = EE_Format();
        /* If erase/program operation was failed, a Flash error code is returned */
        if (fmc_state != FMC_READY)
        {
          return fmc_state;
        }
      }
      break;

    case VALID_PAGE:
      if (PageStatus1 == VALID_PAGE) /* Invalid state -> format eeprom */
      {
        /* Erase both Page0 and Page1 and set Page0 as valid page */
        fmc_state = EE_Format();
        /* If erase/program operation was failed, a Flash error code is returned */
        if (fmc_state != FMC_READY)
        {
          return fmc_state;
        }
      }
      else if (PageStatus1 == ERASED) /* Page0 valid, Page1 erased */
      {
        /* Erase Page1 */
        fmc_state = fmc_page_erase(PAGE1_BASE_ADDRESS);
        /* If erase operation was failed, a Flash error code is returned */
        if (fmc_state != FMC_READY)
        {
          return fmc_state;
        }
      }
      else /* Page0 valid, Page1 receive */
      {
        /* Transfer data from Page0 to Page1 */
        for (VarIdx = 0; VarIdx < NB_OF_VAR; VarIdx++)
        {
          if ((*(__IO uint16_t*)(PAGE1_BASE_ADDRESS + 6)) == VirtAddVarTab[VarIdx])
          {
            x = VarIdx;
          }
          if (VarIdx != x)
          {
            /* Read the last variables' updates */
            ReadStatus = EE_ReadVariable(VirtAddVarTab[VarIdx], &DataVar);
            /* In case variable corresponding to the virtual address was found */
            if (ReadStatus != 0x1)
            {
              /* Transfer the variable to the Page1 */
              EepromStatus = EE_VerifyPageFullWriteVariable(VirtAddVarTab[VarIdx], DataVar);
              /* If program operation was failed, a Flash error code is returned */
              if (EepromStatus != FMC_READY)                                       // TODO - zkontrolovat
              {
                return EepromStatus;
              }
            }
          }
        }
        /* Mark Page1 as valid */
        fmc_state = fmc_halfword_program(PAGE1_BASE_ADDRESS, VALID_PAGE);
        /* If program operation was failed, a Flash error code is returned */
        if (fmc_state != FMC_READY)
        {
          return fmc_state;
        }
        /* Erase Page0 */
        fmc_state = fmc_page_erase(PAGE0_BASE_ADDRESS);
        /* If erase operation was failed, a Flash error code is returned */
        if (fmc_state != FMC_READY)
        {
          return fmc_state;
        }
      }
      break;

    default:  /* Any other state -> format eeprom */
      /* Erase both Page0 and Page1 and set Page0 as valid page */
      fmc_state = EE_Format();
      /* If erase/program operation was failed, a Flash error code is returned */
      if (fmc_state != FMC_READY)
      {
        return fmc_state;
      }
      break;
  }

  return FLASH_COMPLETE;                                        // TODO - nastavit spravnou navratovou hodnotu
}

/**
  * @brief  Returns the last stored variable data, if found, which correspond to
  *   the passed virtual address
  * @param  VirtAddress: Variable virtual address
  * @param  Data: Global variable contains the read variable value
  * @retval Success or error status:
  *           - 0: if variable was found
  *           - 1: if the variable was not found
  *           - NO_VALID_PAGE: if no valid page was found.
  */
uint16_t EE_ReadVariable(uint16_t VirtAddress, uint16_t* Data)
{
  uint16_t ValidPage = PAGE0;
  uint16_t AddressValue = 0x5555, ReadStatus = VAR_NOT_FOUND;
  uint32_t Address = 0x08010000, PageStartAddress = 0x08010000;

  /* Get active Page for read operation */
  ValidPage = EE_FindValidPage(READ_FROM_VALID_PAGE);

  /* Check if there is no valid page */
  if (ValidPage == NO_VALID_PAGE)
  {
    return  NO_VALID_PAGE;
  }

  /* Get the valid Page start Address */
  PageStartAddress = (uint32_t)(EEPROM_START_ADDRESS + (uint32_t)(ValidPage * PAGE_SIZE));

  /* Get the valid Page end Address */
  Address = (uint32_t)((EEPROM_START_ADDRESS - 2) + (uint32_t)((1 + ValidPage) * PAGE_SIZE));

  /* Check each active page address starting from end */
  while (Address > (PageStartAddress + 2))
  {
    /* Get the current location content to be compared with virtual address */
    AddressValue = (*(__IO uint16_t*)Address);

    /* Compare the read address with the virtual address */
    if (AddressValue == VirtAddress)
    {
      /* Get content of Address-2 which is variable value */
      *Data = (*(__IO uint16_t*)(Address - 2));

      /* In case variable value is read, reset ReadStatus flag */
      ReadStatus = VAR_FOUND;

      break;
    }
    else
    {
      /* Next address location */
      Address = Address - 4;
    }
  }

  /* Return ReadStatus value: (0: variable exist, 1: variable doesn't exist) */
  return ReadStatus;
}

/**
  * @brief  Writes/upadtes variable data in EEPROM.
  * @param  VirtAddress: Variable virtual address
  * @param  Data: 16 bit data to be written
  * @retval Success or error status:
  *           - FLASH_COMPLETE: on success
  *           - PAGE_FULL: if valid page is full
  *           - NO_VALID_PAGE: if no valid page was found
  *           - Flash error code: on write Flash error
  */
uint16_t EE_WriteVariable(uint16_t VirtAddress, uint16_t Data)
{
  uint16_t Status = 0;

  /* Write the variable virtual address and value in the EEPROM */
  Status = EE_VerifyPageFullWriteVariable(VirtAddress, Data);

  /* In case the EEPROM active page is full */
  if (Status == PAGE_FULL)
  {
    /* Perform Page transfer */
    Status = EE_PageTransfer(VirtAddress, Data);
  }

  /* Return last operation status */
  return Status;
}

/**
  * @brief  Erases PAGE0 and PAGE1 and writes VALID_PAGE header to PAGE0
  * @param  None
  * @retval Status of the last operation (Flash write or erase) done during
  *         EEPROM formating
  */
static fmc_state_enum EE_Format(void)
{
  fmc_state_enum fmc_state;

  /* Erase Page0 */
  fmc_state = fmc_page_erase(PAGE0_BASE_ADDRESS);

  /* If erase operation was failed, a Flash error code is returned */
  if (fmc_state != FMC_READY)
  {
    return fmc_state;
  }

  /* Set Page0 as valid page: Write VALID_PAGE at Page0 base address */
  fmc_state = fmc_halfword_program(PAGE0_BASE_ADDRESS, VALID_PAGE);

  /* If program operation was failed, a Flash error code is returned */
  if (fmc_state != FMC_READY)
  {
    return fmc_state;
  }

  /* Erase Page1 */
  fmc_state = fmc_page_erase(PAGE1_BASE_ADDRESS);

  /* Return Page1 erase operation status */
  return fmc_state;
}

/**
  * @brief  Find valid Page for write or read operation
  * @param  Operation: operation to achieve on the valid page.
  *   This parameter can be one of the following values:
  *     @arg READ_FROM_VALID_PAGE: read operation from valid page
  *     @arg WRITE_IN_VALID_PAGE: write operation from valid page
  * @retval Valid page number (PAGE0 or PAGE1) or NO_VALID_PAGE in case
  *   of no valid page was found
  */
static uint16_t EE_FindValidPage(uint8_t Operation)
{
  uint16_t PageStatus0 = 6, PageStatus1 = 6;

  /* Get Page0 actual status */
  PageStatus0 = (*(__IO uint16_t*)PAGE0_BASE_ADDRESS);

  /* Get Page1 actual status */
  PageStatus1 = (*(__IO uint16_t*)PAGE1_BASE_ADDRESS);

  /* Write or read operation */
  switch (Operation)
  {
    case WRITE_IN_VALID_PAGE:   /* ---- Write operation ---- */
      if (PageStatus1 == VALID_PAGE)
      {
        /* Page0 receiving data */
        if (PageStatus0 == RECEIVE_DATA)
        {
          return PAGE0;         /* Page0 valid */
        }
        else
        {
          return PAGE1;         /* Page1 valid */
        }
      }
      else if (PageStatus0 == VALID_PAGE)
      {
        /* Page1 receiving data */
        if (PageStatus1 == RECEIVE_DATA)
        {
          return PAGE1;         /* Page1 valid */
        }
        else
        {
          return PAGE0;         /* Page0 valid */
        }
      }
      else
      {
        return NO_VALID_PAGE;   /* No valid Page */
      }

    case READ_FROM_VALID_PAGE:  /* ---- Read operation ---- */
      if (PageStatus0 == VALID_PAGE)
      {
        return PAGE0;           /* Page0 valid */
      }
      else if (PageStatus1 == VALID_PAGE)
      {
        return PAGE1;           /* Page1 valid */
      }
      else
      {
        return NO_VALID_PAGE ;  /* No valid Page */
      }

    default:
      return PAGE0;             /* Page0 valid */
  }
}

/**
  * @brief  Verify if active page is full and Writes variable in EEPROM.
  * @param  VirtAddress: 16 bit virtual address of the variable
  * @param  Data: 16 bit data to be written as variable value
  * @retval Success or error status:
  *           - FLASH_COMPLETE: on success
  *           - PAGE_FULL: if valid page is full
  *           - NO_VALID_PAGE: if no valid page was found
  *           - Flash error code: on write Flash error
  */
static uint16_t EE_VerifyPageFullWriteVariable(uint16_t VirtAddress, uint16_t Data)
{
  fmc_state_enum fmc_state;
  uint16_t ValidPage = PAGE0;
  uint32_t Address = 0x08010000, PageEndAddress = 0x080107FF;

  /* Get valid Page for write operation */
  ValidPage = EE_FindValidPage(WRITE_IN_VALID_PAGE);

  /* Check if there is no valid page */
  if (ValidPage == NO_VALID_PAGE)
  {
    return  NO_VALID_PAGE;
  }

  /* Get the valid Page start Address */
  Address = (uint32_t)(EEPROM_START_ADDRESS + (uint32_t)(ValidPage * PAGE_SIZE));

  /* Get the valid Page end Address */
  PageEndAddress = (uint32_t)((EEPROM_START_ADDRESS - 2) + (uint32_t)((1 + ValidPage) * PAGE_SIZE));

  /* Check each active page address starting from begining */
  while (Address < PageEndAddress)
  {
    /* Verify if Address and Address+2 contents are 0xFFFFFFFF */
    if ((*(__IO uint32_t*)Address) == 0xFFFFFFFF)
    {
      /* Set variable data */
      fmc_state = fmc_halfword_program(Address, Data);
      /* If program operation was failed, a Flash error code is returned */
      if (fmc_state != FMC_READY)
      {
        return fmc_state;
      }
      /* Set variable virtual address */
      fmc_state = fmc_halfword_program(Address + 2, VirtAddress);
      /* Return program operation status */
      return fmc_state;
    }
    else
    {
      /* Next address location */
      Address = Address + 4;
    }
  }

  /* Return PAGE_FULL in case the valid page is full */
  return PAGE_FULL;
}

/**
  * @brief  Transfers last updated variables data from the full Page to
  *   an empty one.
  * @param  VirtAddress: 16 bit virtual address of the variable
  * @param  Data: 16 bit data to be written as variable value
  * @retval Success or error status:
  *           - FLASH_COMPLETE: on success
  *           - PAGE_FULL: if valid page is full
  *           - NO_VALID_PAGE: if no valid page was found
  *           - Flash error code: on write Flash error
  */
static uint16_t EE_PageTransfer(uint16_t VirtAddress, uint16_t Data)
{
  fmc_state_enum fmc_state;
  uint32_t NewPageAddress = 0x080103FF, OldPageAddress = 0x08010000;
  uint16_t ValidPage = PAGE0, VarIdx = 0;
  uint16_t EepromStatus = 0, ReadStatus = 0;

  /* Get active Page for read operation */
  ValidPage = EE_FindValidPage(READ_FROM_VALID_PAGE);

  if (ValidPage == PAGE1)       /* Page1 valid */
  {
    /* New page address where variable will be moved to */
    NewPageAddress = PAGE0_BASE_ADDRESS;

    /* Old page address where variable will be taken from */
    OldPageAddress = PAGE1_BASE_ADDRESS;
    debug("PAGE1 VALID\n");
  }
  else if (ValidPage == PAGE0)  /* Page0 valid */
  {
    /* New page address where variable will be moved to */
    NewPageAddress = PAGE1_BASE_ADDRESS;

    /* Old page address where variable will be taken from */
    OldPageAddress = PAGE0_BASE_ADDRESS;
    debug("PAGE0 VALID\n");
  }
  else
  {
    debug("NO VALID PAGE\n");
    return NO_VALID_PAGE;       /* No valid Page */
  }

  /* Set the new Page status to RECEIVE_DATA status */
  fmc_state = fmc_halfword_program(NewPageAddress, RECEIVE_DATA);
  /* If program operation was failed, a Flash error code is returned */
  if (fmc_state != FMC_READY)
  {
    return fmc_state;
  }

  /* Write the variable passed as parameter in the new active page */
  EepromStatus = EE_VerifyPageFullWriteVariable(VirtAddress, Data);
  /* If program operation was failed, a Flash error code is returned */
  if (EepromStatus != FMC_READY)
  {
    return EepromStatus;
  }

  /* Transfer process: transfer variables from old to the new active page */
  for (VarIdx = 0; VarIdx < NB_OF_VAR; VarIdx++)
  {
    if (VirtAddVarTab[VarIdx] != VirtAddress)  /* Check each variable except the one passed as parameter */
    {
      /* Read the other last variable updates */
      ReadStatus = EE_ReadVariable(VirtAddVarTab[VarIdx], &DataVar);
      /* In case variable corresponding to the virtual address was found */
      if (ReadStatus != 0x1)
      {
        /* Transfer the variable to the new active page */
        EepromStatus = EE_VerifyPageFullWriteVariable(VirtAddVarTab[VarIdx], DataVar);
        /* If program operation was failed, a Flash error code is returned */
        if (EepromStatus != FMC_READY)
        {
          return EepromStatus;
        }
      }
    }
  }

  /* Erase the old Page: Set old Page status to ERASED status */
  fmc_state = fmc_page_erase(OldPageAddress);
  /* If erase operation was failed, a Flash error code is returned */
  if (fmc_state != FMC_READY)
  {
    return fmc_state;
  }

  /* Set new Page status to VALID_PAGE status */
  fmc_state = fmc_halfword_program(NewPageAddress, VALID_PAGE);
  /* If program operation was failed, a Flash error code is returned */
  if (fmc_state != FMC_READY)
  {
    return fmc_state;
  }

  /* Return last operation flash status */
  return fmc_state;
}

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
