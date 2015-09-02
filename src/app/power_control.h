/**
 ******************************************************************************
 * @file    power_control.c
 * @author  CZ.NIC, z.s.p.o.
 * @date    22-July-2015
 * @brief   Header file for control of DC/DC converters.
 ******************************************************************************
 ******************************************************************************
 **/
#ifndef POWER_CONTROL_H
#define POWER_CONTROL_H

//Outputs
#define RES_RAM_PIN_PERIPH_CLOCK            RCC_AHBPeriph_GPIOC
#define RES_RAM_PIN_PORT                    GPIOC
#define RES_RAM_PIN                         GPIO_Pin_3

#define ENABLE_5V_PIN_PERIPH_CLOCK          RCC_AHBPeriph_GPIOC
#define ENABLE_5V_PIN_PORT                  GPIOC
#define ENABLE_5V_PIN                       GPIO_Pin_4

#define ENABLE_3V3_PIN_PERIPH_CLOCK         RCC_AHBPeriph_GPIOC
#define ENABLE_3V3_PIN_PORT                 GPIOC
#define ENABLE_3V3_PIN                      GPIO_Pin_5

#define ENABLE_1V35_PIN_PERIPH_CLOCK        RCC_AHBPeriph_GPIOC
#define ENABLE_1V35_PIN_PORT                GPIOC
#define ENABLE_1V35_PIN                     GPIO_Pin_6

#define ENABLE_4V5_PIN_PERIPH_CLOCK         RCC_AHBPeriph_GPIOC
#define ENABLE_4V5_PIN_PORT                 GPIOC
#define ENABLE_4V5_PIN                      GPIO_Pin_7

#define ENABLE_1V8_PIN_PERIPH_CLOCK         RCC_AHBPeriph_GPIOC
#define ENABLE_1V8_PIN_PORT                 GPIOC
#define ENABLE_1V8_PIN                      GPIO_Pin_8

#define ENABLE_1V5_PIN_PERIPH_CLOCK         RCC_AHBPeriph_GPIOC
#define ENABLE_1V5_PIN_PORT                 GPIOC
#define ENABLE_1V5_PIN                      GPIO_Pin_9

#define ENABLE_1V2_PIN_PERIPH_CLOCK         RCC_AHBPeriph_GPIOC
#define ENABLE_1V2_PIN_PORT                 GPIOC
#define ENABLE_1V2_PIN                      GPIO_Pin_10

#define ENABLE_VTT_PIN_PERIPH_CLOCK         RCC_AHBPeriph_GPIOC
#define ENABLE_VTT_PIN_PORT                 GPIOC
#define ENABLE_VTT_PIN                      GPIO_Pin_11

#define USB30_PWRON_PIN_PERIPH_CLOCK        RCC_AHBPeriph_GPIOC
#define USB30_PWRON_PIN_PORT                GPIOC
#define USB30_PWRON_PIN                     GPIO_Pin_12

#define USB31_PWRON_PIN_PERIPH_CLOCK        RCC_AHBPeriph_GPIOC
#define USB31_PWRON_PIN_PORT                GPIOC
#define USB31_PWRON_PIN                     GPIO_Pin_13

#define CFG_CTRL_PIN_PERIPH_CLOCK           RCC_AHBPeriph_GPIOC
#define CFG_CTRL_PIN_PORT                   GPIOC
#define CFG_CTRL_PIN                        GPIO_Pin_15

//Inputs
#define MANRES_PIN_PERIPH_CLOCK             RCC_AHBPeriph_GPIOB
#define MANRES_PIN_PORT                     GPIOB
#define MANRES_PIN                          GPIO_Pin_0

#define SYSRES_OUT_PIN_PERIPH_CLOCK         RCC_AHBPeriph_GPIOB
#define SYSRES_OUT_PIN_PORT                 GPIOB
#define SYSRES_OUT_PIN                      GPIO_Pin_1

#define DGBRES_PIN_PERIPH_CLOCK             RCC_AHBPeriph_GPIOB
#define DGBRES_PIN_PORT                     GPIOB
#define DGBRES_PIN                          GPIO_Pin_2

#define MRES_PIN_PERIPH_CLOCK               RCC_AHBPeriph_GPIOB
#define MRES_PIN_PORT                       GPIOB
#define MRES_PIN                            GPIO_Pin_3

#define PG_5V_PIN_PERIPH_CLOCK              RCC_AHBPeriph_GPIOB
#define PG_5V_PIN_PORT                      GPIOB
#define PG_5V_PIN                           GPIO_Pin_4

#define PG_3V3_PIN_PERIPH_CLOCK             RCC_AHBPeriph_GPIOB
#define PG_3V3_PIN_PORT                     GPIOB
#define PG_3V3_PIN                          GPIO_Pin_5

#define PG_1V35_PIN_PERIPH_CLOCK            RCC_AHBPeriph_GPIOB
#define PG_1V35_PIN_PORT                    GPIOB
#define PG_1V35_PIN                         GPIO_Pin_6

#define PG_4V5_PIN_PERIPH_CLOCK             RCC_AHBPeriph_GPIOB
#define PG_4V5_PIN_PORT                     GPIOB
#define PG_4V5_PIN                          GPIO_Pin_7

#define PG_1V8_PIN_PERIPH_CLOCK             RCC_AHBPeriph_GPIOB
#define PG_1V8_PIN_PORT                     GPIOB
#define PG_1V8_PIN                          GPIO_Pin_8

#define PG_1V5_PIN_PERIPH_CLOCK             RCC_AHBPeriph_GPIOB
#define PG_1V5_PIN_PORT                     GPIOB
#define PG_1V5_PIN                          GPIO_Pin_9

#define PG_1V2_PIN_PERIPH_CLOCK             RCC_AHBPeriph_GPIOB
#define PG_1V2_PIN_PORT                     GPIOB
#define PG_1V2_PIN                          GPIO_Pin_10

#define PG_VTT_PIN_PERIPH_CLOCK             RCC_AHBPeriph_GPIOB
#define PG_VTT_PIN_PORT                     GPIOB
#define PG_VTT_PIN                          GPIO_Pin_11

#define USB30_OVC_PIN_PERIPH_CLOCK          RCC_AHBPeriph_GPIOB
#define USB30_OVC_PIN_PORT                  GPIOB
#define USB30_OVC_PIN                       GPIO_Pin_12

#define USB31_OVC_PIN_PERIPH_CLOCK          RCC_AHBPeriph_GPIOB
#define USB31_OVC_PIN_PORT                  GPIOB
#define USB31_OVC_PIN                       GPIO_Pin_13

#define RTC_ALARM_PIN_PERIPH_CLOCK          RCC_AHBPeriph_GPIOB
#define RTC_ALARM_PIN_PORT                  GPIOB
#define RTC_ALARM_PIN                       GPIO_Pin_14

#define LED_BRT_PIN_PERIPH_CLOCK            RCC_AHBPeriph_GPIOB
#define LED_BRT_PIN_PORT                    GPIOB
#define LED_BRT_PIN                         GPIO_Pin_15

typedef enum usb_states {
    USB_OFF = 0,
    USB_ON  = 1
}usb_state_t;

typedef enum usb_ports {
    USB3_PORT0 = 0,
    USB3_PORT1 = 1
}usb_ports_t;

/*******************************************************************************
  * @function   system_control_io_config
  * @brief      GPIO config for EN, PG, Reset and USB control signals.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void power_control_io_config(void);

/*******************************************************************************
  * @function   power_control_start_regulator
  * @brief      Starts DC/DC regulators.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void power_control_enable_regulator(void);

/*******************************************************************************
  * @function   power_control_usb
  * @brief      Enable / disable power supply for USB.
  * @param      usb_port: USB3_PORT0 or USB3_PORT1.
  * @param      usb_state: USB_ON or USB_OFF.
  * @retval     None.
  *****************************************************************************/
void power_control_usb(usb_ports_t usb_port, usb_state_t usb_state);

void power_control_rst_pwr_rtc_signal_manager();

/*******************************************************************************
  * @function   sysres_out_startup
  * @brief      Handle SYSRES_OUT and CFG_CTRL signals during startup.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void sysres_out_startup(void);

#endif // POWER_CONTROL_H

