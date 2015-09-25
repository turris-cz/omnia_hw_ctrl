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


static states_t next_state = POWER_ON;

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
    led_driver_config(); //TODO: set all LED colour all to white and then black
    msata_pci_indication_config();
    wan_lan_pci_config();
    slave_i2c_config();
}

static ret_value_t power_on(void)
{
    power_control_set_startup_condition();
    power_control_disable_regulator();
    delay(100);
    //TODO: add return value
    power_control_enable_regulator();

    return OK;
}

static ret_value_t load_settings(void)
{
    debounce_config();
//Marvell CPU should send settings now (led brigthness and colour)

    return OK;
}

static ret_value_t light_reset(void)
{
    power_control_first_startup();
    power_control_second_startup();
//TODO: add return value
    return OK;
}

static ret_value_t input_manager(void)
{
    ret_value_t val = OK;
    struct input_sig *input_state = &debounce_input_signal;

    debounce_check_inputs();

    if(input_state->man_res)
    {
        val = GO_TO_LIGHT_RESET;
        input_state->man_res = 0;
    }

    if(input_state->pg)
    {
        val = GO_TO_HARD_RESET;
        input_state->pg = 0;
    }

    if(input_state->usb30_ovc)
    {//TODO
       input_state->usb30_ovc = 0;
    }

    if(input_state->usb31_ovc)
    {
        input_state->usb31_ovc = 0;
    }

    return val;
}

static ret_value_t led_manager(void)
{
    wan_led_activity();
    lan_led_activity();

    return OK;
}

void app_cyclic(void)
{
    ret_value_t val;

    switch(next_state)
    {
    case POWER_ON:
        {
            val = power_on();

            if(val == OK)
                next_state = LIGHT_RESET;
            else
                next_state = ERROR_STATE;
        }
        break;

    case LIGHT_RESET:
        {
            val = light_reset();

            if (val == OK)
                next_state = LOAD_SETTINGS;
            else
                next_state = ERROR_STATE;
        }
        break;

    case HARD_RESET: next_state = POWER_ON;//TODO: go to POWER_ON or reset MCU ?
        break;

    case FACTORY_RESET:
        break;

    case LOAD_SETTINGS:
        {
            load_settings();
            next_state = INPUT_MANAGER;
        }
        break;

    case ERROR_STATE:
        break;

    case INPUT_MANAGER:
        {
            val = input_manager();

            switch(val)
            {
                case GO_TO_LIGHT_RESET: next_state = LIGHT_RESET; break;
                case GO_TO_HARD_RESET: next_state = HARD_RESET; break;
                default: next_state = I2C_MANAGER; break;
            }
        }
        break;

    case I2C_MANAGER: next_state = LED_MANAGER;
        break;

    case LED_MANAGER:
        {
            led_manager();
            next_state = INPUT_MANAGER;
        }
        break;
    }
}
