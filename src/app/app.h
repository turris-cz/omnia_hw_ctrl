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
    FAIL    = -1,
    OK      = 0,
}return_value_t;

typedef enum {
    POWER_ON,
    LIGHT_RESET,
    HARD_RESET,
    FACTORY_RESET,
    SYSTEM_START,
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

#endif // __APP_H
