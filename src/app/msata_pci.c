/**
 ******************************************************************************
 * @file    msata_pci.c
 * @author  CZ.NIC, z.s.p.o.
 * @date    06-August-2015
 * @brief   Driver for PCIe and mSATA indication.
 ******************************************************************************
 ******************************************************************************
 **/
/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_conf.h"
#include "msata_pci.h"

struct msata_pci_ind msata_pci_status;

/*******************************************************************************
  * @function   msata_pci_io_config
  * @brief      GPIO configuration for mSATA and PCIe indication signals.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
static void msata_pci_io_config(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;

    /* GPIO Periph clock enable */
    RCC_AHBPeriphClockCmd(CARD_DET_PIN_PERIPH_CLOCK | MSATALED_PIN_PERIPH_CLOCK
                          | MSATAIND_PIN_PERIPH_CLOCK, ENABLE);

    /* Input signals */
    GPIO_InitStructure.GPIO_Pin = CARD_DET_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(CARD_DET_PIN_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = MSATALED_PIN;
    GPIO_Init(MSATALED_PIN_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = MSATAIND_PIN;
    GPIO_Init(MSATAIND_PIN_PORT, &GPIO_InitStructure);
}

/*******************************************************************************
  * @function   msata_pci_exti_config
  * @brief      EXTI configuration for PCIe and mSATA indication signals.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
static void msata_pci_exti_config(void)
{
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Enable SYSCFG clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

    SYSCFG_EXTILineConfig(CARD_DET_PIN_EXTIPORT, CARD_DET_PIN_EXTIPINSOURCE);

    /* configure all ext. interrupt on rising and falling edge */
    EXTI_InitStructure.EXTI_Line = CARD_DET_PIN_EXTILINE;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    SYSCFG_EXTILineConfig(MSATALED_PIN_EXTIPORT, MSATALED_PIN_EXTIPINSOURCE);
    EXTI_InitStructure.EXTI_Line = MSATALED_PIN_EXTILINE;
    EXTI_Init(&EXTI_InitStructure);

    /* Enable and set EXTI Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = EXTI4_15_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority = 0x04;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/*******************************************************************************
  * @function   msata_pci_indication_config
  * @brief      Main configuration function for mSATA and PCIe status indication.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void msata_pci_indication_config(void)
{
    msata_pci_io_config();
    //msata_pci_exti_config();
    //msata_pci_card_detection();//read status of already inserted card after the reset
}

/*******************************************************************************
  * @function   msata_pci_activity_handler
  * @brief      Toggle LED according to the activity of the connected card.
  *             Called in EXTI interrupt handler.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void msata_pci_activity_handler(void)
{
    uint8_t msata_pci_activity;

    msata_pci_activity = GPIO_ReadInputDataBit(MSATALED_PIN_PORT, MSATALED_PIN);

    if (msata_pci_activity)
    {
        //TODO: LED actvity off
    }
    else
    {
        //TODO: LED activity on
    }
}

/*******************************************************************************
  * @function   msata_pci_card_detection
  * @brief      Detect inserted card - PCIe or mSATA card.
  *             Called in EXTI interrupt handler and during the initialization.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void msata_pci_card_detection(void)
{
    struct msata_pci_ind *msata_pci_detect = &msata_pci_status;

    msata_pci_detect->card_det = GPIO_ReadInputDataBit(CARD_DET_PIN_PORT, CARD_DET_PIN);
    msata_pci_detect->msata_ind = GPIO_ReadInputDataBit(MSATAIND_PIN_PORT, MSATAIND_PIN);

    if (msata_pci_detect->card_det == 0) //a card is inserted
    {
        if (msata_pci_detect->msata_ind)
        {
            //TODO: do some action -> PCIe card detected
        }
        else
        {
            //TODO: do some action -> mSATA card detected
        }
    }
    else
    {
        //TODO: do some action - no card inserted
    }
}
