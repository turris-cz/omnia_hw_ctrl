/**
 ******************************************************************************
 * @file    wan_lan_pci_status.h
 * @author  CZ.NIC, z.s.p.o.
 * @date    10-August-2015
 * @brief   Header file for driver of WAN, LAN and PCIe status indication.
 ******************************************************************************
 ******************************************************************************
 **/
#ifndef WAN_LAN_PCI_STATUS_H
#define WAN_LAN_PCI_STATUS_H

#include "stm32f0xx_conf.h"

/* PCIe status LED */
#define PCI_LED0_PIN_PERIPH_CLOCK           RCC_AHBPeriph_GPIOC
#define PCI_LED0_PIN_PORT                   GPIOC
#define PCI_LED0_PIN                        GPIO_Pin_1

#define PCI_LED1_PIN_PERIPH_CLOCK           RCC_AHBPeriph_GPIOC
#define PCI_LED1_PIN_PORT                   GPIOC
#define PCI_LED1_PIN                        GPIO_Pin_2

/* WAN LED */
#define WAN_LED0_PIN_PERIPH_CLOCK           RCC_AHBPeriph_GPIOF
#define WAN_LED0_PIN_PORT                   GPIOF
#define WAN_LED0_PIN                        GPIO_Pin_0

#define WAN_LED1_PIN_PERIPH_CLOCK           RCC_AHBPeriph_GPIOF
#define WAN_LED1_PIN_PORT                   GPIOF
#define WAN_LED1_PIN                        GPIO_Pin_1

/* SFP status */
#define SFP_DIS_PIN_PERIPH_CLOCK            RCC_AHBPeriph_GPIOD
#define SFP_DIS_PIN_PORT                    GPIOD
#define SFP_DIS_PIN                         GPIO_Pin_2

#define SFP_LOS_PIN_PERIPH_CLOCK            RCC_AHBPeriph_GPIOF
#define SFP_LOS_PIN_PORT                    GPIOF
#define SFP_LOS_PIN                         GPIO_Pin_4
#define SFP_LOS_PIN_EXTIPORT                EXTI_PortSourceGPIOF
#define SFP_LOS_PIN_EXTIPINSOURCE           EXTI_PinSource4
#define SFP_LOS_PIN_EXTILINE                EXTI_Line4

#define SFP_FLT_PIN_PERIPH_CLOCK            RCC_AHBPeriph_GPIOF
#define SFP_FLT_PIN_PORT                    GPIOF
#define SFP_FLT_PIN                         GPIO_Pin_5
#define SFP_FLT_PIN_EXTIPORT                EXTI_PortSourceGPIOF
#define SFP_FLT_PIN_EXTIPINSOURCE           EXTI_PinSource5
#define SFP_FLT_PIN_EXTILINE                EXTI_Line5

#define SFP_DET_PIN_PERIPH_CLOCK            RCC_AHBPeriph_GPIOA
#define SFP_DET_PIN_PORT                    GPIOA
#define SFP_DET_PIN                         GPIO_Pin_10
#define SFP_DET_PIN_EXTIPORT                EXTI_PortSourceGPIOA
#define SFP_DET_PIN_EXTIPINSOURCE           EXTI_PinSource10
#define SFP_DET_PIN_EXTILINE                EXTI_Line10

/* LAN LED */
#define LAN_LED_PORT                        GPIOA
#define R0_P0_LED_PIN_PERIPH_CLOCK          RCC_AHBPeriph_GPIOA
#define R0_P0_LED_PIN_PORT                  GPIOA
#define R0_P0_LED_PIN                       GPIO_Pin_0

#define R1_P1_LED_PIN_PERIPH_CLOCK          RCC_AHBPeriph_GPIOA
#define R1_P1_LED_PIN_PORT                  GPIOA
#define R1_P1_LED_PIN                       GPIO_Pin_1

#define R2_P2_LED_PIN_PERIPH_CLOCK          RCC_AHBPeriph_GPIOA
#define R2_P2_LED_PIN_PORT                  GPIOA
#define R2_P2_LED_PIN                       GPIO_Pin_2

#define C0_P3_LED_PIN_PERIPH_CLOCK          RCC_AHBPeriph_GPIOA
#define C0_P3_LED_PIN_PORT                  GPIOA
#define C0_P3_LED_PIN                       GPIO_Pin_6

#define C1_LED_PIN_PERIPH_CLOCK             RCC_AHBPeriph_GPIOA
#define C1_LED_PIN_PORT                     GPIOA
#define C1_LED_PIN                          GPIO_Pin_8

#define C2_P4_LED_PIN_PERIPH_CLOCK          RCC_AHBPeriph_GPIOA
#define C2_P4_LED_PIN_PORT                  GPIOA
#define C2_P4_LED_PIN                       GPIO_Pin_11

#define C3_P5_LED_PIN_PERIPH_CLOCK          RCC_AHBPeriph_GPIOA
#define C3_P5_LED_PIN_PORT                  GPIOA
#define C3_P5_LED_PIN                       GPIO_Pin_12


/*******************************************************************************
  * @function   wan_lan_pci_config
  * @brief      Main configuration function for WAN, LAN and PCIe status indication.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void wan_lan_pci_config(void);

/*******************************************************************************
  * @function   wan_sfp_connector_detection
  * @brief      Detect inserted SFP+ connector.
  * @param      None.
  * @retval     1 - SFP detected, 0 - SFP not detected.
  *****************************************************************************/
__inline__ uint8_t wan_sfp_connector_detection(void);

/*******************************************************************************
  * @function   wan_sfp_fault_detection
  * @brief      Detect a SFP fault.
  * @param      None.
  * @retval     1 - SFP TX fault, 0 - SFP no TX fault.
  *****************************************************************************/
__inline__ uint8_t wan_sfp_fault_detection(void);

/*******************************************************************************
  * @function   wan_sfp_lost_detection
  * @brief      Detect a lost communication.
  * @param      None.
  * @retval     1 - SFP lost, 0 - no SFP lost
  *****************************************************************************/
__inline__ uint8_t wan_sfp_lost_detection(void);

/******************************************************************************
  * @function   wan_sfp_set_tx_status
  * @brief      Enable/Disable SFP transmitting.
  * @param      sfp_status: ENABLE or DISABLE.
  * @retval     None.
  *****************************************************************************/
void wan_sfp_set_tx_status(FunctionalState sfp_status);

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
  * @function   pci_led_activity
  * @brief      Toggle LAN LEDs according to the LAN status.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void lan_led_activity(void);

#endif // WAN_LAN_PCI_STATUS_H
