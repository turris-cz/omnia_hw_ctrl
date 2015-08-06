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

    /* Enable periph clock */
    RCC_AHBPeriphClockCmd(CARD_DET_PIN_PERIPH_CLOCK | MSATALED_PIN_PERIPH_CLOCK
                          | MSATAIND_PIN_PERIPH_CLOCK, ENABLE);

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

    SYSCFG_EXTILineConfig(MSATAIND_PIN_EXTIPORT, MSATAIND_PIN_EXTIPINSOURCE);
    EXTI_InitStructure.EXTI_Line = MSATAIND_PIN_EXTILINE;
    EXTI_Init(&EXTI_InitStructure);

    /* Enable and set EXTI Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = EXTI4_15_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority = 0x04;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}
