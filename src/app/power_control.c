/**
 ******************************************************************************
 * @file    power_control.c
 * @author  CZ.NIC, z.s.p.o.
 * @date    22-July-2015
 * @brief   Functions for control of DC/DC converters.
 ******************************************************************************
 ******************************************************************************
 **/
/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_conf.h"
#include "power_control.h"
#include "delay.h"
#include "led_driver.h"

/* Private define ------------------------------------------------------------*/
#define USE_4V5_POWER
/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
  * @function   system_control_io_config
  * @brief      GPIO config for EN, PG, Reset and USB signals.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void power_control_io_config(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;

    /* GPIO Periph clock enable */
    RCC_AHBPeriphClockCmd(RES_RAM_PIN_PERIPH_CLOCK | ENABLE_5V_PIN_PERIPH_CLOCK |
       ENABLE_3V3_PIN_PERIPH_CLOCK | ENABLE_1V35_PIN_PERIPH_CLOCK |
       ENABLE_4V5_PIN_PERIPH_CLOCK | ENABLE_1V8_PIN_PERIPH_CLOCK |
       ENABLE_1V5_PIN_PERIPH_CLOCK | ENABLE_1V2_PIN_PERIPH_CLOCK |
       ENABLE_VTT_PIN_PERIPH_CLOCK | USB30_PWRON_PIN_PERIPH_CLOCK |
       USB31_PWRON_PIN_PERIPH_CLOCK | SYSRES_OUT_PIN_PERIPH_CLOCK |
       INT_MCU_PIN_PERIPH_CLOCK | MANRES_PIN_PERIPH_CLOCK, ENABLE);

    /* Output signals */
    GPIO_InitStructure.GPIO_Pin = RES_RAM_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_2;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(RES_RAM_PIN_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = INT_MCU_PIN;
    GPIO_Init(INT_MCU_PIN_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = ENABLE_5V_PIN;
    GPIO_Init(ENABLE_5V_PIN_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = ENABLE_3V3_PIN;
    GPIO_Init(ENABLE_3V3_PIN_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = ENABLE_1V35_PIN;
    GPIO_Init(ENABLE_1V35_PIN_PORT, &GPIO_InitStructure);
#ifdef USE_4V5_POWER
    GPIO_InitStructure.GPIO_Pin = ENABLE_4V5_PIN;
    GPIO_Init(ENABLE_4V5_PIN_PORT, &GPIO_InitStructure);
#endif
    GPIO_InitStructure.GPIO_Pin = ENABLE_1V8_PIN;
    GPIO_Init(ENABLE_1V8_PIN_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = ENABLE_1V5_PIN;
    GPIO_Init(ENABLE_1V5_PIN_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = ENABLE_1V2_PIN;
    GPIO_Init(ENABLE_1V2_PIN_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = ENABLE_VTT_PIN;
    GPIO_Init(ENABLE_VTT_PIN_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = USB30_PWRON_PIN;
    GPIO_Init(USB30_PWRON_PIN_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = USB31_PWRON_PIN;
    GPIO_Init(USB31_PWRON_PIN_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = CFG_CTRL_PIN;
    GPIO_Init(CFG_CTRL_PIN_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = SYSRES_OUT_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_2;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(SYSRES_OUT_PIN_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = MANRES_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_2;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(MANRES_PIN_PORT, &GPIO_InitStructure);


    /* Input signals */

    /* GPIO Periph clock enable */
    RCC_AHBPeriphClockCmd(DGBRES_PIN_PERIPH_CLOCK | MRES_PIN_PERIPH_CLOCK |
                          PG_5V_PIN_PERIPH_CLOCK | PG_3V3_PIN_PERIPH_CLOCK |
                          PG_1V35_PIN_PERIPH_CLOCK | PG_4V5_PIN_PERIPH_CLOCK |
                          PG_1V8_PIN_PERIPH_CLOCK | PG_1V5_PIN_PERIPH_CLOCK |
                          PG_1V2_PIN_PERIPH_CLOCK | PG_VTT_PIN_PERIPH_CLOCK |
                          USB30_OVC_PIN_PERIPH_CLOCK | USB31_OVC_PIN_PERIPH_CLOCK |
                          RTC_ALARM_PIN_PERIPH_CLOCK | LED_BRT_PIN_PERIPH_CLOCK,
                          ENABLE);

    GPIO_InitStructure.GPIO_Pin = MRES_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(MRES_PIN_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = DGBRES_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(DGBRES_PIN_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = PG_5V_PIN;
    GPIO_Init(PG_5V_PIN_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = PG_3V3_PIN;
    GPIO_Init(PG_3V3_PIN_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = PG_1V35_PIN;
    GPIO_Init(PG_1V35_PIN_PORT, &GPIO_InitStructure);
#ifdef USE_4V5_POWER
    GPIO_InitStructure.GPIO_Pin = PG_4V5_PIN;
    GPIO_Init(PG_4V5_PIN_PORT, &GPIO_InitStructure);
#endif
    GPIO_InitStructure.GPIO_Pin = PG_1V8_PIN;
    GPIO_Init(PG_1V8_PIN_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = PG_1V5_PIN;
    GPIO_Init(PG_1V5_PIN_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = PG_1V2_PIN;
    GPIO_Init(PG_1V2_PIN_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = PG_VTT_PIN;
    GPIO_Init(PG_VTT_PIN_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = USB30_OVC_PIN;
    GPIO_Init(USB30_OVC_PIN_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = USB31_OVC_PIN;
    GPIO_Init(USB31_OVC_PIN_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = RTC_ALARM_PIN;
    GPIO_Init(RTC_ALARM_PIN_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = LED_BRT_PIN;
    GPIO_Init(LED_BRT_PIN_PORT, &GPIO_InitStructure);

    GPIO_SetBits(SYSRES_OUT_PIN_PORT, SYSRES_OUT_PIN); //dont control this !
    GPIO_SetBits(INT_MCU_PIN_PORT, INT_MCU_PIN);
}

/*******************************************************************************
  * @function   power_control_set_startup_condition
  * @brief      Set signals to reset state before board startup.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void power_control_set_startup_condition(void)
{
    GPIO_SetBits(CFG_CTRL_PIN_PORT, CFG_CTRL_PIN); //disconnect switches
    GPIO_ResetBits(MANRES_PIN_PORT, MANRES_PIN); //board reset activated
    power_control_usb(USB3_PORT0, USB_ON);
    power_control_usb(USB3_PORT1, USB_ON);
}

/*******************************************************************************
  * @function   power_control_enable_regulator
  * @brief      Starts DC/DC regulators.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void power_control_enable_regulator(void)
{
    /*
     * power-up sequence:
     * 1) 5V regulator
     * 2) 4.5V regulator - if used
     * 3) 3.3V regulator
     * 4) 1.8V regulator
     * 5) 1.5V regulator
     * 6) 1.35V regulator
     *    VTT regulator
     * 7) 1.2V regulator
     */
    GPIO_SetBits(ENABLE_5V_PIN_PORT, ENABLE_5V_PIN);
    while(!(GPIO_ReadInputDataBit(PG_5V_PIN_PORT, PG_5V_PIN)))
        ;

#ifdef USE_4V5_POWER
    GPIO_SetBits(ENABLE_4V5_PIN_PORT, ENABLE_4V5_PIN);
    while(!(GPIO_ReadInputDataBit(PG_4V5_PIN_PORT, PG_4V5_PIN)))
        ;
#endif

    GPIO_SetBits(ENABLE_3V3_PIN_PORT, ENABLE_3V3_PIN);
    while(!(GPIO_ReadInputDataBit(PG_3V3_PIN_PORT, PG_3V3_PIN)))
        ;

    GPIO_SetBits(ENABLE_1V8_PIN_PORT, ENABLE_1V8_PIN);
    while(!(GPIO_ReadInputDataBit(PG_1V8_PIN_PORT, PG_1V8_PIN)))
        ;

    GPIO_SetBits(ENABLE_1V5_PIN_PORT, ENABLE_1V5_PIN);
    while(!(GPIO_ReadInputDataBit(PG_1V5_PIN_PORT, PG_1V5_PIN)))
        ;

    GPIO_SetBits(ENABLE_1V35_PIN_PORT, ENABLE_1V35_PIN);
    GPIO_SetBits(ENABLE_VTT_PIN_PORT, ENABLE_VTT_PIN);
    while(!(GPIO_ReadInputDataBit(PG_1V35_PIN_PORT, PG_1V35_PIN)))
        ;
    while(!(GPIO_ReadInputDataBit(PG_VTT_PIN_PORT, PG_VTT_PIN)))
        ;

    GPIO_SetBits(ENABLE_1V2_PIN_PORT, ENABLE_1V2_PIN);
    while(!(GPIO_ReadInputDataBit(PG_1V2_PIN_PORT, PG_1V2_PIN)))
        ;
}

/*******************************************************************************
  * @function   power_control_disable_regulator
  * @brief      Shutdown DC/DC regulators.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void power_control_disable_regulator(void)
{
    GPIO_ResetBits(ENABLE_1V2_PIN_PORT, ENABLE_1V2_PIN);
    GPIO_ResetBits(ENABLE_1V35_PIN_PORT, ENABLE_1V35_PIN);
    GPIO_ResetBits(ENABLE_VTT_PIN_PORT, ENABLE_VTT_PIN);
    GPIO_ResetBits(ENABLE_1V5_PIN_PORT, ENABLE_1V5_PIN);
    GPIO_ResetBits(ENABLE_1V8_PIN_PORT, ENABLE_1V8_PIN);
    GPIO_ResetBits(ENABLE_3V3_PIN_PORT, ENABLE_3V3_PIN);
#ifdef USE_4V5_POWER
    GPIO_ResetBits(ENABLE_4V5_PIN_PORT, ENABLE_4V5_PIN);
#endif
    GPIO_ResetBits(ENABLE_5V_PIN_PORT, ENABLE_5V_PIN);
}

/*******************************************************************************
  * @function   power_control_usb
  * @brief      Enable / disable power supply for USB.
  * @param      usb_port: USB3_PORT0 or USB3_PORT1.
  * @param      usb_state: USB_ON or USB_OFF.
  * @retval     None.
  *****************************************************************************/
void power_control_usb(usb_ports_t usb_port, usb_state_t usb_state)
{
    if (usb_port == USB3_PORT0)
    {
        if (usb_state == USB_ON)
            GPIO_ResetBits(USB30_PWRON_PIN_PORT, USB30_PWRON_PIN);
        else
            GPIO_SetBits(USB30_PWRON_PIN_PORT, USB30_PWRON_PIN);
    }
    else //USB3_PORT1
    {
        if (usb_state == USB_ON)
            GPIO_ResetBits(USB31_PWRON_PIN_PORT, USB31_PWRON_PIN);
        else
            GPIO_SetBits(USB31_PWRON_PIN_PORT, USB31_PWRON_PIN);
    }
}

/*******************************************************************************
  * @function   power_control_get_usb_overcurrent
  * @brief      Get USB overcurrent status.
  * @param      usb_port: USB3_PORT0 or USB3_PORT1.
  * @retval     1 - USB overcurrent ocurred; 0 - no USB overcurrent
  *****************************************************************************/
uint8_t power_control_get_usb_overcurrent(usb_ports_t usb_port)
{
    if (usb_port == USB3_PORT0)
        return (!(GPIO_ReadInputDataBit(USB30_OVC_PIN_PORT, USB30_OVC_PIN)));
    else //USB3_PORT1
        return (!(GPIO_ReadInputDataBit(USB31_OVC_PIN_PORT, USB31_OVC_PIN)));
}

/*******************************************************************************
  * @function   power_control_get_usb_poweron
  * @brief      Get USB poweron status.
  * @param      usb_port: USB3_PORT0 or USB3_PORT1.
  * @retval     1 - USB power ON; 0 - USB power OFF
  *****************************************************************************/
uint8_t power_control_get_usb_poweron(usb_ports_t usb_port)
{
    if (usb_port == USB3_PORT0)
        return (!(GPIO_ReadInputDataBit(USB30_PWRON_PIN_PORT, USB30_PWRON_PIN)));
    else //USB3_PORT1
        return (!(GPIO_ReadInputDataBit(USB31_PWRON_PIN_PORT, USB31_PWRON_PIN)));
}

/*******************************************************************************
  * @function   debounce_usb_timeout_timer_config
  * @brief      Timer configuration for USB recovery timeout.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void power_control_usb_timeout_config(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    // Clock enable
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14, ENABLE);

    /* Time base configuration - 1sec interrupt */
    TIM_TimeBaseStructure.TIM_Period = 8000 - 1;
    TIM_TimeBaseStructure.TIM_Prescaler = 6000 - 1;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(USB_TIMEOUT_TIMER, &TIM_TimeBaseStructure);

    TIM_ARRPreloadConfig(USB_TIMEOUT_TIMER, ENABLE);
    /* TIM Interrupts enable */
    TIM_ITConfig(USB_TIMEOUT_TIMER, TIM_IT_Update, ENABLE);

    /* TIM enable counter */
    //TIM_Cmd(USB_TIMEOUT_TIMER, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = TIM14_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority = 0x05;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/*******************************************************************************
  * @function   power_control_first_startup
  * @brief      Handle SYSRES_OUT, MAN_RES and CFG_CTRL signals during startup.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void power_control_first_startup(void)
{
    GPIO_SetBits(CFG_CTRL_PIN_PORT, CFG_CTRL_PIN);
    delay(200);
    GPIO_SetBits(MANRES_PIN_PORT, MANRES_PIN);

    // wait for main board reset signal
    while (!GPIO_ReadInputDataBit(SYSRES_OUT_PIN_PORT, SYSRES_OUT_PIN))
        ;

    delay(5); // 5ms delay after releasing of reset signal
    GPIO_ResetBits(CFG_CTRL_PIN_PORT, CFG_CTRL_PIN);
}

/*******************************************************************************
  * @function   power_control_second_startup
  * @brief      Second reset due to wrong startup.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void power_control_second_startup(void)
{
    delay(100);
    GPIO_SetBits(CFG_CTRL_PIN_PORT, CFG_CTRL_PIN);
    GPIO_ResetBits(MANRES_PIN_PORT, MANRES_PIN);
    delay(200);
    GPIO_SetBits(MANRES_PIN_PORT, MANRES_PIN);

    while (!GPIO_ReadInputDataBit(SYSRES_OUT_PIN_PORT, SYSRES_OUT_PIN))
        ;

    delay(5);
    GPIO_ResetBits(CFG_CTRL_PIN_PORT, CFG_CTRL_PIN);
}

/*******************************************************************************
  * @function   power_control_set_power_led
  * @brief      Set on power LED.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void power_control_set_power_led(void)
{
    struct led_rgb *rgb_leds = leds;

    rgb_leds[POWER_LED].led_status = LED_ON;
    led_driver_set_colour(POWER_LED, 0xFFFFFF);
}
