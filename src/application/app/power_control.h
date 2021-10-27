/**
 ******************************************************************************
 * @file    power_control.c
 * @author  CZ.NIC, z.s.p.o.
 * @date    26-October-2021
 * @brief   Header file for control of DC/DC converters.
 ******************************************************************************
 ******************************************************************************
 **/
#ifndef POWER_CONTROL_H
#define POWER_CONTROL_H

#include "gd32f1x0_libopt.h"

#define USB_TIMEOUT_TIMER                   TIMER16

/* Outputs */
#define INT_MCU_PIN_PERIPH_CLOCK            RCU_GPIOC
#define INT_MCU_PIN_PORT                    GPIOC
#define INT_MCU_PIN                         GPIO_PIN_0

#define RES_RAM_PIN_PERIPH_CLOCK            RCU_GPIOC
#define RES_RAM_PIN_PORT                    GPIOC
#define RES_RAM_PIN                         GPIO_PIN_3

#define ENABLE_5V_PIN_PERIPH_CLOCK          RCU_GPIOC
#define ENABLE_5V_PIN_PORT                  GPIOC
#define ENABLE_5V_PIN                       GPIO_PIN_4

#define ENABLE_3V3_PIN_PERIPH_CLOCK         RCU_GPIOC
#define ENABLE_3V3_PIN_PORT                 GPIOC
#define ENABLE_3V3_PIN                      GPIO_PIN_5

#define ENABLE_1V35_PIN_PERIPH_CLOCK        RCU_GPIOC
#define ENABLE_1V35_PIN_PORT                GPIOC
#define ENABLE_1V35_PIN                     GPIO_PIN_6

#define ENABLE_4V5_PIN_PERIPH_CLOCK         RCU_GPIOC
#define ENABLE_4V5_PIN_PORT                 GPIOC
#define ENABLE_4V5_PIN                      GPIO_PIN_7

#define ENABLE_1V8_PIN_PERIPH_CLOCK         RCU_GPIOC
#define ENABLE_1V8_PIN_PORT                 GPIOC
#define ENABLE_1V8_PIN                      GPIO_PIN_8

#define ENABLE_1V5_PIN_PERIPH_CLOCK         RCU_GPIOC
#define ENABLE_1V5_PIN_PORT                 GPIOC
#define ENABLE_1V5_PIN                      GPIO_PIN_9

#define ENABLE_1V2_PIN_PERIPH_CLOCK         RCU_GPIOC
#define ENABLE_1V2_PIN_PORT                 GPIOC
#define ENABLE_1V2_PIN                      GPIO_PIN_10

#define ENABLE_VTT_PIN_PERIPH_CLOCK         RCU_GPIOC
#define ENABLE_VTT_PIN_PORT                 GPIOC
#define ENABLE_VTT_PIN                      GPIO_PIN_11

#define USB30_PWRON_PIN_PERIPH_CLOCK        RCU_GPIOC
#define USB30_PWRON_PIN_PORT                GPIOC
#define USB30_PWRON_PIN                     GPIO_PIN_12

#define USB31_PWRON_PIN_PERIPH_CLOCK        RCU_GPIOC
#define USB31_PWRON_PIN_PORT                GPIOC
#define USB31_PWRON_PIN                     GPIO_PIN_13

#define CFG_CTRL_PIN_PERIPH_CLOCK           RCU_GPIOC
#define CFG_CTRL_PIN_PORT                   GPIOC
#define CFG_CTRL_PIN                        GPIO_PIN_15

#define PRG_4V5_PIN_PERIPH_CLOCK            RCU_GPIOF
#define PRG_4V5_PIN_PORT                    GPIOF
#define PRG_4V5_PIN                         GPIO_PIN_1

/* Inputs */
#define MANRES_PIN_PERIPH_CLOCK             RCU_GPIOB
#define MANRES_PIN_PORT                     GPIOB
#define MANRES_PIN                          GPIO_PIN_0

#define SYSRES_OUT_PIN_PERIPH_CLOCK         RCU_GPIOB
#define SYSRES_OUT_PIN_PORT                 GPIOB
#define SYSRES_OUT_PIN                      GPIO_PIN_1

#define DGBRES_PIN_PERIPH_CLOCK             RCU_GPIOB
#define DGBRES_PIN_PORT                     GPIOB
#define DGBRES_PIN                          GPIO_PIN_2

#define MRES_PIN_PERIPH_CLOCK               RCU_GPIOB
#define MRES_PIN_PORT                       GPIOB
#define MRES_PIN                            GPIO_PIN_3

#define PG_5V_PIN_PERIPH_CLOCK              RCU_GPIOB
#define PG_5V_PIN_PORT                      GPIOB
#define PG_5V_PIN                           GPIO_PIN_4

#define PG_3V3_PIN_PERIPH_CLOCK             RCU_GPIOB
#define PG_3V3_PIN_PORT                     GPIOB
#define PG_3V3_PIN                          GPIO_PIN_5

#define PG_1V35_PIN_PERIPH_CLOCK            RCU_GPIOB
#define PG_1V35_PIN_PORT                    GPIOB
#define PG_1V35_PIN                         GPIO_PIN_6

#define PG_4V5_PIN_PERIPH_CLOCK             RCU_GPIOB
#define PG_4V5_PIN_PORT                     GPIOB
#define PG_4V5_PIN                          GPIO_PIN_7

#define PG_1V8_PIN_PERIPH_CLOCK             RCU_GPIOB
#define PG_1V8_PIN_PORT                     GPIOB
#define PG_1V8_PIN                          GPIO_PIN_8

#define PG_1V5_PIN_PERIPH_CLOCK             RCU_GPIOB
#define PG_1V5_PIN_PORT                     GPIOB
#define PG_1V5_PIN                          GPIO_PIN_9

#define PG_1V2_PIN_PERIPH_CLOCK             RCU_GPIOB
#define PG_1V2_PIN_PORT                     GPIOB
#define PG_1V2_PIN                          GPIO_PIN_10

#define PG_VTT_PIN_PERIPH_CLOCK             RCU_GPIOB
#define PG_VTT_PIN_PORT                     GPIOB
#define PG_VTT_PIN                          GPIO_PIN_11

#define USB30_OVC_PIN_PERIPH_CLOCK          RCU_GPIOB
#define USB30_OVC_PIN_PORT                  GPIOB
#define USB30_OVC_PIN                       GPIO_PIN_12

#define USB31_OVC_PIN_PERIPH_CLOCK          RCU_GPIOB
#define USB31_OVC_PIN_PORT                  GPIOB
#define USB31_OVC_PIN                       GPIO_PIN_13

#define RTC_ALARM_PIN_PERIPH_CLOCK          RCU_GPIOB
#define RTC_ALARM_PIN_PORT                  GPIOB
#define RTC_ALARM_PIN                       GPIO_PIN_14

#define LED_BRT_PIN_PERIPH_CLOCK            RCU_GPIOB
#define LED_BRT_PIN_PORT                    GPIOB
#define LED_BRT_PIN                         GPIO_PIN_15


typedef enum voltages {
    VOLTAGE_33 = 1,
    VOLTAGE_36 = 2,
    VOLTAGE_45 = 3,
    VOLTAGE_51 = 4,
}voltage_value_t;

typedef enum usb_states {
    USB_OFF = 0,
    USB_ON  = 1
}usb_state_t;

typedef enum usb_ports {
    USB3_PORT0 = 0,
    USB3_PORT1 = 1
}usb_ports_t;

typedef enum reg_types {
    REG_5V,
    REG_3V3,
    REG_1V35,
    REG_4V5,
    REG_1V8,
    REG_1V5,
    REG_1V2,
    REG_VTT,
}reg_type_t;

typedef enum error_types {
    NO_ERROR,
    PG_5V_ERROR,
    PG_3V3_ERROR,
    PG_1V35_ERROR,
    PG_4V5_ERROR,
    PG_1V8_ERROR,
    PG_1V5_ERROR,
    PG_1V2_ERROR,
    PG_VTT_ERROR,
}error_type_t;

typedef enum reset_types {
    NORMAL_RESET            = 0,
    PREVIOUS_SNAPSHOT       = 1,
    NORMAL_FACTORY_RESET    = 2,
    HARD_FACTORY_RESET      = 3,
    USER_RESET1             = 4,
    USER_RESET2             = 5,
    USER_RESET3             = 6,
    USER_RESET4             = 7,
    USER_RESET5             = 8,
    USER_RESET6             = 9,
    USER_RESET7             = 10,
    USER_RESET8             = 11,
}reset_type_t;

/*******************************************************************************
  * @function   system_control_io_config
  * @brief      GPIO config for EN, PG, Reset and USB control signals.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void power_control_io_config(void);

/*******************************************************************************
  * @function   power_control_start_regulators
  * @brief      Starts DC/DC regulators.
  * @param      None.
  * @retval     Error if timeout elapsed.
  *****************************************************************************/
error_type_t power_control_enable_regulators(void);

/*******************************************************************************
  * @function   power_control_disable_regulators
  * @brief      Shutdown DC/DC regulators.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void power_control_disable_regulators(void);

/*******************************************************************************
  * @function   power_control_usb
  * @brief      Enable / disable power supply for USB.
  * @param      usb_port: USB3_PORT0 or USB3_PORT1.
  * @param      usb_state: USB_ON or USB_OFF.
  * @retval     None.
  *****************************************************************************/
void power_control_usb(usb_ports_t usb_port, usb_state_t usb_state);

/*******************************************************************************
  * @function   power_control_first_startup
  * @brief      Handle SYSRES_OUT, MAN_RES and CFG_CTRL signals during startup.
  * @param      None.
  * @retval     Type of factory reset.
  *****************************************************************************/
reset_type_t power_control_first_startup(void);

/*******************************************************************************
  * @function   power_control_usb_timeout_config
  * @brief      Timer configuration for USB recovery timeout.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void power_control_usb_timeout_config(void);

/*******************************************************************************
  * @function   power_control_get_usb_overcurrent
  * @brief      Get USB overcurrent status.
  * @param      usb_port: USB3_PORT0 or USB3_PORT1.
  * @retval     1 - USB overcurrent ocurred; 0 - no USB overcurrent
  *****************************************************************************/
uint8_t power_control_get_usb_overcurrent(usb_ports_t usb_port);

/*******************************************************************************
  * @function   power_control_get_usb_poweron
  * @brief      Get USB poweron status.
  * @param      usb_port: USB3_PORT0 or USB3_PORT1.
  * @retval     1 - USB power ON; 0 - USB power OFF
  *****************************************************************************/
uint8_t power_control_get_usb_poweron(usb_ports_t usb_port);

/*******************************************************************************
  * @function   power_control_set_startup_condition
  * @brief      Set signals to reset state before board startup.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void power_control_set_startup_condition(void);

/*******************************************************************************
  * @function   power_control_start_regulator
  * @brief      Start DC/DC regulator and handle timeout.
  * @param      regulator: regulator type.
  * @retval     error, if problem with PG signal occures.
  *****************************************************************************/
error_type_t power_control_start_regulator(reg_type_t regulator);

/*******************************************************************************
  * @function   power_control_set_power_led
  * @brief      Set on power LED.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void power_control_set_power_led(void);

/*******************************************************************************
  * @function   power_led_activity
  * @brief      Set on power LED.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void power_led_activity(void);

/*******************************************************************************
  * @function   power_control_usb_timeout_enable
  * @brief      Enable USB recovery timeout.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void power_control_usb_timeout_enable(void);

/*******************************************************************************
  * @function   power_control_usb_timeout_disable
  * @brief      Disable USB recovery timeout.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void power_control_usb_timeout_disable(void);

/*******************************************************************************
  * @function   power_control_set_voltage
  * @brief      Set required voltage to the user regulator.
  * @param      voltage: enum value for desired voltage.
  * @retval     None.
  *****************************************************************************/
void power_control_set_voltage(voltage_value_t voltage);

#endif // POWER_CONTROL_H

