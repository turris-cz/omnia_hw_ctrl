/**
 ******************************************************************************
 * @file    wan_lan_pci_status.c
 * @author  CZ.NIC, z.s.p.o.
 * @date    10-August-2015
 * @brief   Driver for WAN, LAN and PCIe status indication.
 ******************************************************************************
 ******************************************************************************
 **/
#include "wan_lan_pci_status.h"
#include "led_driver.h"

/*******************************************************************************
  * @function   wan_lan_pci_io_config
  * @brief      GPIO configuration for WAN, LAN and PCIe indication signals.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
static void wan_lan_pci_io_config(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;

    /* GPIO Periph clock enable */
    RCC_AHBPeriphClockCmd(CFG_CTRL_PIN_PERIPH_CLOCK | PCI_LED0_PIN_PERIPH_CLOCK
                          | PCI_LED1_PIN_PERIPH_CLOCK | WAN_LED0_PIN_PERIPH_CLOCK
                          | WAN_LED1_PIN_PERIPH_CLOCK | SFP_DIS_PIN_PERIPH_CLOCK
                          | SFP_LOS_PIN_PERIPH_CLOCK | SFP_FLT_PIN_PERIPH_CLOCK
                          | SFP_DET_PIN_PERIPH_CLOCK | R0_P0_LED_PIN_PERIPH_CLOCK
                          | R1_P1_LED_PIN_PERIPH_CLOCK | R2_P2_LED_PIN_PERIPH_CLOCK
                          | C0_P3_LED_PIN_PERIPH_CLOCK | C1_LED_PIN_PERIPH_CLOCK
                          | C2_P4_LED_PIN_PERIPH_CLOCK | C3_P5_LED_PIN_PERIPH_CLOCK,
                          ENABLE);

    /* CFG_CTRL pin */
    GPIO_InitStructure.GPIO_Pin = CFG_CTRL_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_2;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(CFG_CTRL_PIN_PORT, &GPIO_InitStructure);

    /* PCIe LED pins */
    GPIO_InitStructure.GPIO_Pin = PCI_LED0_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(PCI_LED0_PIN_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = PCI_LED1_PIN;
    GPIO_Init(PCI_LED1_PIN_PORT, &GPIO_InitStructure);

    /* WAN LED pins */
    GPIO_InitStructure.GPIO_Pin = WAN_LED0_PIN;
    GPIO_Init(WAN_LED0_PIN_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = WAN_LED1_PIN;
    GPIO_Init(WAN_LED1_PIN_PORT, &GPIO_InitStructure);

    /* SFP_DIS output pin */
    GPIO_InitStructure.GPIO_Pin = SFP_DIS_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_2;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(SFP_DIS_PIN_PORT, &GPIO_InitStructure);

    /* SFP input pins */
    GPIO_InitStructure.GPIO_Pin = SFP_LOS_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(SFP_LOS_PIN_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = SFP_FLT_PIN;
    GPIO_Init(SFP_FLT_PIN_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = SFP_DET_PIN;
    GPIO_Init(SFP_DET_PIN_PORT, &GPIO_InitStructure);

    /* LAN LED input pins */
    GPIO_InitStructure.GPIO_Pin = R0_P0_LED_PIN;
    GPIO_Init(R0_P0_LED_PIN_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = R1_P1_LED_PIN;
    GPIO_Init(R1_P1_LED_PIN_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = R2_P2_LED_PIN;
    GPIO_Init(R2_P2_LED_PIN_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = C0_P3_LED_PIN;
    GPIO_Init(C0_P3_LED_PIN_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = C1_LED_PIN;
    GPIO_Init(C1_LED_PIN_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = C2_P4_LED_PIN;
    GPIO_Init(C2_P4_LED_PIN_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = C3_P5_LED_PIN;
    GPIO_Init(C3_P5_LED_PIN_PORT, &GPIO_InitStructure);
}

/*******************************************************************************
  * @function   wan_lan_pci_exti_config
  * @brief      EXTI configuration for WAN, LAN and PCIe indication signals.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
static void wan_lan_pci_exti_config(void)
{
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Enable SYSCFG clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

    SYSCFG_EXTILineConfig(SFP_LOS_PIN_EXTIPORT, SFP_LOS_PIN_EXTIPINSOURCE);

    /* configure all ext. interrupt on rising and falling edge */
    EXTI_InitStructure.EXTI_Line = SFP_LOS_PIN_EXTILINE;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    SYSCFG_EXTILineConfig(SFP_FLT_PIN_EXTIPORT, SFP_FLT_PIN_EXTIPINSOURCE);
    EXTI_InitStructure.EXTI_Line = SFP_FLT_PIN_EXTILINE;
    EXTI_Init(&EXTI_InitStructure);

    SYSCFG_EXTILineConfig(SFP_DET_PIN_EXTIPORT, SFP_DET_PIN_EXTIPINSOURCE);
    EXTI_InitStructure.EXTI_Line = SFP_DET_PIN_EXTILINE;
    EXTI_Init(&EXTI_InitStructure);

    /* Enable and set EXTI Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = EXTI4_15_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority = 0x04;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/*******************************************************************************
  * @function   wan_lan_pci_config
  * @brief      Main configuration function for WAN, LAN and PCIe status indication.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void wan_lan_pci_config(void)
{
    wan_lan_pci_io_config();
    wan_lan_pci_exti_config();
    wan_sfp_set_tx_status(ENABLE);
    /* read status of signals after the reset */
    wan_sfp_connector_detection();
    wan_sfp_fault_detection();
    wan_sfp_lost_detection();
}

/*******************************************************************************
  * @function   wan_sfp_connector_detection
  * @brief      Detect inserted SFP+ connector.
  *             Called in EXTI interrupt handler and during the initialization.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void wan_sfp_connector_detection(void)
{
    uint8_t sfp_detected;

    sfp_detected = GPIO_ReadInputDataBit(SFP_DET_PIN_PORT, SFP_DET_PIN);

    if (sfp_detected)
    {
        //TODO: log. 1 means SFP connector unconnected ?
    }
    else
    {
        //TODO: log. 0 means SFP connector connected ?
    }
}

/*******************************************************************************
  * @function   wan_sfp_fault_detection
  * @brief      Detect a SFP fault.
  *             Called in EXTI interrupt handler and during the initialization.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void wan_sfp_fault_detection(void)
{
    uint8_t sfp_fault_detected;

    sfp_fault_detected = GPIO_ReadInputDataBit(SFP_FLT_PIN_PORT, SFP_FLT_PIN);

    if (sfp_fault_detected)
    {
        //TODO: do some action - no fault
    }
    else
    {
        //TODO: do some action - fault occures
    }
}

/*******************************************************************************
  * @function   wan_sfp_lost_detection
  * @brief      Detect a lost communication.
  *             Called in EXTI interrupt handler and during the initialization.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void wan_sfp_lost_detection(void)
{
    uint8_t sfp_lost_detected;

    sfp_lost_detected = GPIO_ReadInputDataBit(SFP_LOS_PIN_PORT, SFP_LOS_PIN);

    if (sfp_lost_detected)
    {
        //TODO: do some action - no lost detected
    }
    else
    {
        //TODO: do some action - lost detected
    }
}

/*******************************************************************************
  * @function   wan_sfp_set_tx_status
  * @brief      Enable/Disable SFP transmitting.
  * @param      sfp_status: ENABLE or DISABLE.
  * @retval     None.
  *****************************************************************************/
void wan_sfp_set_tx_status(FunctionalState sfp_status)
{
    if (sfp_status == ENABLE)
    {
        GPIO_SetBits(SFP_DIS_PIN_PORT, SFP_DIS_PIN);
    }
    else
    {
        GPIO_ResetBits(SFP_DIS_PIN_PORT, SFP_DIS_PIN);
    }
}

/*******************************************************************************
  * @function   wan_led_activity
  * @brief      Toggle WAN LED according to the WAN activity.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void wan_led_activity(void)
{
    uint8_t led0_status, led1_status;
    struct led_rgb *rgb_leds = leds;

    led0_status = GPIO_ReadInputDataBit(WAN_LED0_PIN_PORT, WAN_LED0_PIN);
    led1_status = GPIO_ReadInputDataBit(WAN_LED1_PIN_PORT, WAN_LED1_PIN);

    if (led0_status == 0) //TODO: check real LED polarity
    {
        //TODO: assign LED indexes (as a pointer) to real meanings
        rgb_leds[0].led_status = LED_ENABLE;
    }
    else
    {
        led0_status = rgb_leds[0].led_status = LED_DISABLE;
    }

    if (led1_status == 0)
    {
        rgb_leds[1].led_status = LED_ENABLE;
    }
    else
    {
        rgb_leds[1].led_status = LED_DISABLE;
    }
}

/*******************************************************************************
  * @function   pci_led_activity
  * @brief      Toggle PCIe LED according to the PCIe activity.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void pci_led_activity(void)
{
    uint8_t pcie_led0, pcie_led1;
    struct led_rgb *rgb_leds = leds;

    pcie_led0 = GPIO_ReadInputDataBit(PCI_LED0_PIN_PORT, PCI_LED0_PIN);
    pcie_led1 = GPIO_ReadInputDataBit(PCI_LED1_PIN_PORT, PCI_LED1_PIN);

    //TODO: assign LED indexes to real meanings
    if(pcie_led0 == 0)
    {
        //TODO: change to dynamic access
        rgb_leds[2].led_status = LED_ENABLE;
    }
    else
    {
        rgb_leds[2].led_status = LED_DISABLE;
    }

    if (pcie_led1 == 0)
    {
        rgb_leds[3].led_status = LED_ENABLE;
    }
    else
    {
        rgb_leds[3].led_status = LED_DISABLE;
    }
}
