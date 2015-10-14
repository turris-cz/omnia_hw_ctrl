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
#include "led_driver.h"

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

    SYSCFG_EXTILineConfig(MSATALED_PIN_EXTIPORT, MSATALED_PIN_EXTIPINSOURCE);

    /* configure all ext. interrupt on rising and falling edge */
    EXTI_InitStructure.EXTI_Line = MSATALED_PIN_EXTILINE;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
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
    msata_pci_exti_config();
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
    struct led_rgb *rgb_leds = leds;

    msata_pci_activity = GPIO_ReadInputDataBit(MSATALED_PIN_PORT, MSATALED_PIN);

    if (rgb_leds[MSATA_PCI_LED].led_mode == LED_DEFAULT_MODE)
    {
        if (msata_pci_activity)
        {
            rgb_leds[MSATA_PCI_LED].led_state = LED_OFF;
        }
        else
        {
            rgb_leds[MSATA_PCI_LED].led_state = LED_ON;
        }
    }
}

/*******************************************************************************
  * @function   msata_pci_card_detection
  * @brief      Detect inserted card (whether a card is inserted or not)
  * @param      None.
  * @retval     1 - a card inserted, 0 - no card inserted.
  *****************************************************************************/
inline uint8_t msata_pci_card_detection(void)
{
    /* inverted due to the HW connection
    HW connection: 1 = no card inserted, 0 = card inserted */
    return (!(GPIO_ReadInputDataBit(CARD_DET_PIN_PORT, CARD_DET_PIN)));
}

/*******************************************************************************
  * @function   msata_pci_type_card_detection
  * @brief      Detect a type of inserted card - mSATA or miniPCIe
  * @param      None.
  * @retval     1 - mSATA card inserted, 0 - miniPCIe card inserted.
  *****************************************************************************/
inline uint8_t msata_pci_type_card_detection(void)
{
    /* inverted due to the HW connection
    HW connection: 1 = miniPCIE inserted, 0 = mSATA card inserted */
    return (!(GPIO_ReadInputDataBit(MSATAIND_PIN_PORT, MSATAIND_PIN)));
}
