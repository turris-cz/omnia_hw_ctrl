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

#include "gpio.h"

/* Outputs */
#define INT_MCU_PIN		PIN(C, 0)
#define RES_RAM_PIN		PIN(C, 3, OMNIA_BOARD_REVISION < 32)
#define ENABLE_5V_PIN		PIN(C, 4)
#define ENABLE_3V3_PIN		PIN(C, 5)
#define ENABLE_1V35_PIN		PIN(C, 6)
#define ENABLE_4V5_PIN		PIN(C, 7, USER_REGULATOR_ENABLED)
#define ENABLE_1V8_PIN		PIN(C, 8)
#define ENABLE_1V5_PIN		PIN(C, 9, OMNIA_BOARD_REVISION < 32)
#define ENABLE_1V2_PIN		PIN(C, 10)
#define ENABLE_VTT_PIN		PIN(C, 11)
#define USB30_PWRON_PIN		PIN(C, 12)
#define USB31_PWRON_PIN		PIN(C, 13)
#define CFG_CTRL_PIN		PIN(C, 15)
#define PRG_4V5_PIN		PIN(F, 1, USER_REGULATOR_ENABLED)
/* v32 specific outputs */
#define nPERST1_PIN		PIN(A, 10, OMNIA_BOARD_REVISION >= 32 && !DBG_ENABLE)
#define nRES_MMC_PIN		PIN(B, 2, OMNIA_BOARD_REVISION >= 32)
#define nRES_LAN_PIN		PIN(B, 3, OMNIA_BOARD_REVISION >= 32)
#define nRES_PHY_PIN		PIN(B, 7, OMNIA_BOARD_REVISION >= 32)
#define nVHV_CTRL_PIN		PIN(B, 14, OMNIA_BOARD_REVISION >= 32)
#define PHY_SFP_PIN		PIN(C, 3, OMNIA_BOARD_REVISION >= 32)
#define nPERST2_PIN		PIN(F, 4, OMNIA_BOARD_REVISION >= 32)
#define nPERST0_PIN		PIN(F, 5, OMNIA_BOARD_REVISION >= 32)

/* Inputs */
#define MANRES_PIN		PIN(B, 0)
#define SYSRES_OUT_PIN		PIN(B, 1)
#define DBGRES_PIN		PIN(B, 2, OMNIA_BOARD_REVISION < 32)
#define MRES_PIN		PIN(B, 3, OMNIA_BOARD_REVISION < 32)
#define PG_5V_PIN		PIN(B, 4)
#define PG_3V3_PIN		PIN(B, 5)
#define PG_1V35_PIN		PIN(B, 6)
#define PG_4V5_PIN		PIN(B, 7)
#define PG_1V8_PIN		PIN(B, 8)
#define PG_1V5_PIN		PIN(B, 9)
#define PG_1V2_PIN		PIN(B, 10)
#define PG_VTT_PIN		PIN(B, 11)
#define USB30_OVC_PIN		PIN(B, 12)
#define USB31_OVC_PIN		PIN(B, 13)
#define RTC_ALARM_PIN		PIN(B, 14, OMNIA_BOARD_REVISION < 32)
#define FRONT_BTN_PIN		PIN(B, 15)
/* v32 specific inputs */
#define SFP_nDET_PIN		PIN(D, 2, OMNIA_BOARD_REVISION >= 32)

typedef enum {
	VOLTAGE_3V3 = 1,
	VOLTAGE_3V63 = 2,
	VOLTAGE_4V5 = 3,
	VOLTAGE_5V125 = 4,
} user_reg_voltage_t;

typedef enum {
	USB3_PORT0 = 0,
	USB3_PORT1 = 1
} usb_port_t;

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
  * @retval     0 on success, -n if enableing n-th regulator failed.
  *****************************************************************************/
int power_control_enable_regulators(void);

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
  * @param      on: true or false.
  * @retval     None.
  *****************************************************************************/
void power_control_usb(usb_port_t usb_port, bool on);

/*******************************************************************************
  * @function   power_control_first_startup
  * @brief      Handle SYSRES_OUT, MAN_RES and CFG_CTRL signals during startup.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void power_control_first_startup(void);

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
bool power_control_get_usb_overcurrent(usb_port_t usb_port);

/*******************************************************************************
  * @function   power_control_get_usb_poweron
  * @brief      Get USB poweron status.
  * @param      usb_port: USB3_PORT0 or USB3_PORT1.
  * @retval     1 - USB power ON; 0 - USB power OFF
  *****************************************************************************/
bool power_control_get_usb_poweron(usb_port_t usb_port);

/*******************************************************************************
  * @function   power_control_set_startup_condition
  * @brief      Set signals to reset state before board startup.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void power_control_set_startup_condition(void);

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
  * @function   power_control_usb_timeout_irq_handler
  * @brief      Handle USB timeout.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void power_control_usb_timeout_irq_handler(void);

/*******************************************************************************
  * @function   power_control_set_voltage
  * @brief      Set required voltage to the user regulator.
  * @param      voltage: enum value for desired voltage.
  * @retval     None.
  *****************************************************************************/
void power_control_set_voltage(user_reg_voltage_t voltage);

/*******************************************************************************
  * @function   periph_control_io_config
  * @brief      Configuration of new IO pins for Omnia32
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void periph_control_io_config(void);

/*******************************************************************************
  * @function   periph_control_rst_init
  * @brief      Set reset init states for peripherals for Omnia32
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void periph_control_rst_init(void);

#endif // POWER_CONTROL_H

