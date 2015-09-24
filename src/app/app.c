/**
 ******************************************************************************
 * @file    app.c
 * @author  CZ.NIC, z.s.p.o.
 * @date    25-September-2015
 * @brief   Init and cyclic high level operations.
 ******************************************************************************
 ******************************************************************************
 **/
/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx.h"
#include "app.h"
#include "power_control.h"
#include "delay.h"
#include "debounce.h"
#include "led_driver.h"
#include "msata_pci.h"
#include "slave_i2c_device.h"
#include "wan_lan_pci_status.h"


static states_t state = POWER_ON;

/*******************************************************************************
  * @function   app_mcu_init
  * @brief      Initialization of MCU and its ports and peripherals.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void app_mcu_init(void)
{
    SystemCoreClockUpdate(); // set HSI and PLL
    delay_systimer_config();
    //init ports and peripheral
    power_control_io_config();
    debounce_config();
    led_driver_config(); //TODO: set all LED colour all to white and then black
    msata_pci_indication_config();
    wan_lan_pci_config();
    slave_i2c_config();
}

static return_value_t power_on(void)
{
    //TODO: add return value
    power_control_enable_regulator();

    return OK;
}

static return_value_t system_start(void)
{
//Marvell CPU should send settings now

}

void app_cyclic(void)
{
    return_value_t val;

    switch(state)
    {
    case POWER_ON:
        {
            val = power_on();

            if(val == OK)
                state = LIGHT_RESET;
            else
                state = ERROR_STATE;
        }
        break;

    case LIGHT_RESET:
        break;

    case HARD_RESET:
        break;

    case FACTORY_RESET:
        break;

    case SYSTEM_START:
        break;

    case ERROR_STATE:
        break;

    case INPUT_MANAGER:
        break;

    case I2C_MANAGER:
        break;

    case LED_MANAGER:
        break;
    }
}
