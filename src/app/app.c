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

#define SET_INTERRUPT_TO_CPU       GPIO_ResetBits(INT_MCU_PIN_PORT, INT_MCU_PIN)
#define RESET_INTERRUPT_TO_CPU     GPIO_SetBits(INT_MCU_PIN_PORT, INT_MCU_PIN)

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
    led_driver_config(); //TODO: set all LED colour to white and then black
    msata_pci_indication_config();
    wan_lan_pci_config();
    power_control_usb_timeout_config();
    slave_i2c_config();
}

/*******************************************************************************
  * @function   power_on
  * @brief      Start the board / enable dc-dc regulators.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
static ret_value_t power_on(void)
{
    power_control_set_startup_condition();
    power_control_disable_regulator();
    delay(100);
    //TODO: add return value
    power_control_enable_regulator();

    return OK;
}

static uint16_t get_status_word(void)
{
    uint16_t status_word = 0;

    if (wan_sfp_connector_detection())
    {
        status_word |= SFP_DET_BIT;
        wan_sfp_set_tx_status(ENABLE);
        status_word &= (~SFP_DIS_BIT);
    }

    if (wan_sfp_lost_detection())
        status_word |= SFP_LOS_BIT;

    if (wan_sfp_fault_detection())
        status_word |= SFP_FLT_BIT;

    if (msata_pci_card_detection())
        status_word |= CARD_DET_BIT;

    if (msata_pci_type_card_detection())
        status_word |= MSATA_IND_BIT;

    if (power_control_get_usb_overcurrent(USB3_PORT0))
        status_word |= USB30_OVC_BIT;

    if (power_control_get_usb_overcurrent(USB3_PORT1))
        status_word |= USB31_OVC_BIT;

    if (power_control_get_usb_poweron(USB3_PORT0))
        status_word |= USB30_PWRON_BIT;

    if (power_control_get_usb_poweron(USB3_PORT1))
        status_word |= USB31_PWRON_BIT;

    return status_word;
}
static ret_value_t load_settings(void)
{
    debounce_config();

    i2c_status_word = get_status_word();
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

    if (input_state->sysres_out)
    {
        val = GO_TO_LIGHT_RESET; //TODO: reaction - light reset ?
        input_state->sysres_out = 0;
    }

    if(input_state->pg)
    {
        val = GO_TO_HARD_RESET;
        input_state->pg = 0;
    }

    if(input_state->usb30_ovc)
    {
        i2c_status_word |= USB30_OVC_BIT;
        input_state->usb30_ovc = 0;
        //TODO: vypnout USB a nastavit bit v i2c_status_word
    }

    if(input_state->usb31_ovc)
    {
        i2c_status_word |= USB31_OVC_BIT;
        input_state->usb31_ovc = 0;
    }

    if (input_state->led_brt)
    {
        led_driver_step_brightness();
        input_state->led_brt = 0;
    }

    if(input_state->sfp_det) //flag is cleared in debounce function
        i2c_status_word |= SFP_DET_BIT;

    if(input_state->sfp_los)
        i2c_status_word |= SFP_LOS_BIT;

    if(input_state->sfp_flt)
        i2c_status_word |= SFP_FLT_BIT;

    return val;
}

static ret_value_t ic2_manager(void)
{
    static uint16_t last_status_word;

    if (i2c_status_word != last_status_word)
    {
        SET_INTERRUPT_TO_CPU;
        last_status_word = i2c_status_word;
    }
    else
        RESET_INTERRUPT_TO_CPU;

    slave_i2c_process_data();

    return OK;
}

static ret_value_t led_manager(void)
{
    wan_led_activity();
    lan_led_activity();

    return OK;
}

void app_mcu_cyclic(void)
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

    case I2C_MANAGER:
        {
            ic2_manager();
            next_state = LED_MANAGER;
        }
        break;

    case LED_MANAGER:
        {
            led_manager();
            next_state = INPUT_MANAGER;
        }
        break;
    }
}
