/**
 ******************************************************************************
 * @file    app.h
 * @author  CZ.NIC, z.s.p.o.
 * @date    25-September-2015
 * @brief   Header file for app.c
 ******************************************************************************
 ******************************************************************************
 **/
#ifndef __APP_H
#define __APP_H

typedef enum {
    FAIL                = -1,
    OK                  = 0,
    GO_TO_LIGHT_RESET   = 1,
    GO_TO_HARD_RESET    = 2,
}ret_value_t;

typedef enum {
    POWER_ON,
    LIGHT_RESET,
    HARD_RESET,
    FACTORY_RESET,
    LOAD_SETTINGS,
    ERROR_STATE,
    INPUT_MANAGER,
    I2C_MANAGER,
    LED_MANAGER,
} states_t;

/*******************************************************************************
  * @function   app_mcu_init
  * @brief      Initialization of MCU and its ports and peripherals.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void app_mcu_init(void);

/*******************************************************************************
  * @function   app_mcu_cyclic
  * @brief      Main cyclic function.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void app_mcu_cyclic(void);

#endif // __APP_H
