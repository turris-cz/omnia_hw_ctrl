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
#include "debug_serial.h"
#include "eeprom.h"

#define MAX_ERROR_COUNT            5
#define SET_INTERRUPT_TO_CPU       GPIO_ResetBits(INT_MCU_PIN_PORT, INT_MCU_PIN)
#define RESET_INTERRUPT_TO_CPU     GPIO_SetBits(INT_MCU_PIN_PORT, INT_MCU_PIN)
#define RESET_TYPE_STARTBIT        13

/*******************************************************************************
  * @function   app_mcu_init
  * @brief      Initialization of MCU and its ports and peripherals.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void app_mcu_init(void)
{
    struct st_watchdog *wdg = &watchdog;
    eeprom_var_t ee_var;

    SystemCoreClockUpdate(); /* set HSI and PLL */
    FLASH_Unlock(); /* Unlock the Flash Program Erase controller */
    EE_Init(); /* EEPROM Init */

    ee_var = EE_ReadVariable(WDG_VIRT_ADDR, (uint16_t *)&wdg->watchdog_sts);

    switch(ee_var)
    {
        case VAR_NOT_FOUND:
        {
            wdg->watchdog_sts = WDG_ENABLE;
            EE_WriteVariable(WDG_VIRT_ADDR, (uint16_t)wdg->watchdog_sts);
            DBG("Init - EEPROM var not found\r\n");
        } break;

        case VAR_FOUND: DBG("Init - EEPROM var found\r\n");
            break;

        case VAR_NO_VALID_PAGE : DBG("Init - No valid page\r\n");
            break;

        default:
            break;
    }

    delay_systimer_config();
    /* init ports and peripheral */
    power_control_io_config();
    msata_pci_indication_config();
    wan_lan_pci_config();
    power_control_usb_timeout_config();
    led_driver_config();
    slave_i2c_config();
    debug_serial_config();

    DBG("\r\nInit completed.\r\n");
}

/*******************************************************************************
  * @function   app_get_status_word
  * @brief      Set status word after reset.
  * @param      None.
  * @retval     system_status_word.
  *****************************************************************************/
static uint16_t app_get_status_word(void)
{
    uint16_t status_word = 0;

    if (wan_sfp_connector_detection())
    {
        status_word |= SFP_DET_STSBIT;
        wan_sfp_set_tx_status(ENABLE);
    }

    if(wan_sfp_get_tx_status())
        status_word |= SFP_DIS_STSBIT;
    else
        status_word &= (~SFP_DIS_STSBIT);

    if (wan_sfp_lost_detection())
        status_word |= SFP_LOS_STSBIT;

    if (wan_sfp_fault_detection())
        status_word |= SFP_FLT_STSBIT;

    if (msata_pci_card_detection())
        status_word |= CARD_DET_STSBIT;

    if (msata_pci_type_card_detection())
        status_word |= MSATA_IND_STSBIT;

    if (power_control_get_usb_overcurrent(USB3_PORT0))
        status_word |= USB30_OVC_STSBIT;

    if (power_control_get_usb_overcurrent(USB3_PORT1))
        status_word |= USB31_OVC_STSBIT;

    if (power_control_get_usb_poweron(USB3_PORT0))
        status_word |= USB30_PWRON_STSBIT;

    if (power_control_get_usb_poweron(USB3_PORT1))
        status_word |= USB31_PWRON_STSBIT;

    if(GPIO_ReadInputDataBit(ENABLE_4V5_PIN_PORT, ENABLE_4V5_PIN))
        status_word |= ENABLE_4V5_STSBIT;

    return status_word;
}

/*******************************************************************************
  * @function   power_on
  * @brief      Start the board / enable dc-dc regulators.
  * @param      None.
  * @retval     value: next state.
  *****************************************************************************/
static ret_value_t power_on(void)
{
    ret_value_t value = OK;
    error_type_t error = NO_ERROR;

    power_control_set_startup_condition();
    power_control_disable_regulators();
    delay(100);

    error = power_control_enable_regulators();

    switch(error)
    {
        case PG_5V_ERROR: value = GO_TO_5V_ERROR; break;
        case PG_3V3_ERROR: value = GO_TO_3V3_ERROR; break;
        case PG_1V35_ERROR: value = GO_TO_1V35_ERROR; break;
        case PG_4V5_ERROR: value = GO_TO_4V5_ERROR; break;
        case PG_1V8_ERROR: value = GO_TO_1V8_ERROR; break;
        case PG_1V5_ERROR: value = GO_TO_1V5_ERROR; break;
        case PG_1V2_ERROR: value = GO_TO_1V2_ERROR; break;
        case PG_VTT_ERROR: value = GO_TO_VTT_ERROR; break;
        default: value = OK; break;
    }

    return value;
}

/*******************************************************************************
  * @function   light_reset
  * @brief      Perform light reset of the board.
  * @param      None.
  * @retval     value: next_state.
  *****************************************************************************/
static ret_value_t light_reset(void)
{
    ret_value_t value = OK;
    reset_type_t reset_event = NORMAL_RESET;
    struct st_i2c_status *i2c_control = &i2c_status;
    struct st_watchdog *wdg = &watchdog;

    reset_event = power_control_first_startup();

    i2c_control->reset_type = reset_event;

    if(wdg->watchdog_sts == WDG_ENABLE)
    {
        wdg->watchdog_state = RUN;
        DBG("RST - WDG runs\r\n");
    }
    else
    {
        wdg->watchdog_state = STOP;
        DBG("RST - WDG doesnt run\r\n")
    }

    return value;
}

/*******************************************************************************
  * @function   load_settings
  * @brief      Set other initialization.
  * @param      None.
  * @retval     next_state.
  *****************************************************************************/
static ret_value_t load_settings(void)
{
    struct st_i2c_status *i2c_control = &i2c_status;

    debounce_config(); /* start evaluation of inputs */
    i2c_control->status_word = app_get_status_word();

    return OK;
}

/*******************************************************************************
  * @function   input_manager
  * @brief      Evaluate input signals and their reaction.
  * @param      None.
  * @retval     value: next_state.
  *****************************************************************************/
static ret_value_t input_manager(void)
{
    ret_value_t value = OK;
    struct input_sig *input_state = &debounce_input_signal;
    struct st_i2c_status *i2c_control = &i2c_status;
    struct button_def *button = &button_front;

    debounce_check_inputs();

    /* manual reset button */
    if(input_state->man_res == ACTIVATED)
    {
        value = GO_TO_LIGHT_RESET;
        input_state->man_res = DEACTIVATED;
    }

    /* sw reset */
    if (input_state->sysres_out == ACTIVATED)
    {
        value = GO_TO_LIGHT_RESET;
        input_state->sysres_out = DEACTIVATED;
    }

    /* PG signals from all DC/DC regulator (except of 4.5V user regulator) */
    if(input_state->pg == ACTIVATED)
    {
        DBG("PG all regulators\r\n");
        value = GO_TO_HARD_RESET;
        input_state->pg = DEACTIVATED;
    }

    /* PG signal from 4.5V user controlled regulator */
    if(input_state->pg_4v5 == ACTIVATED)
    {
        DBG("PG from 4V5\r\n");
        value = GO_TO_HARD_RESET;
        input_state->pg_4v5 = DEACTIVATED;
    }

    /* USB30 overcurrent */
    if(input_state->usb30_ovc == ACTIVATED)
    {
        i2c_control->status_word |= USB30_OVC_STSBIT;
        input_state->usb30_ovc = DEACTIVATED;
        power_control_usb(USB3_PORT0, USB_OFF); /* USB power off */

        if(!power_control_get_usb_poweron(USB3_PORT0))  /* update status word */
            i2c_control->status_word &= (~USB30_PWRON_STSBIT);

        /* USB timeout set to 1 sec */
        power_control_usb_timeout_enable();
    }

    /* USB31 overcurrent */
    if(input_state->usb31_ovc == ACTIVATED)
    {
        i2c_control->status_word |= USB31_OVC_STSBIT;
        input_state->usb31_ovc = DEACTIVATED;

        power_control_usb(USB3_PORT1, USB_OFF); /* USB power off */

        if(!power_control_get_usb_poweron(USB3_PORT1)) /* update status word */
            i2c_control->status_word &= (~USB31_PWRON_STSBIT);

        /* USB timeout set to 1 sec */
        power_control_usb_timeout_enable();
    }

    /* front button */
    if (input_state->button_sts == ACTIVATED)
    {
        if (button->button_mode == BUTTON_DEFAULT)
            led_driver_step_brightness();
        else /* user button mode */
            button_counter_increase();

        input_state->button_sts = DEACTIVATED;
    }

    /* in case of user button mode:
     * store information in status_word - how many times a button was pressed  */
    if(button->button_mode != BUTTON_DEFAULT)
    {
        if (button->button_pressed_counter)
        {
            i2c_control->status_word &= ~BUTTON_COUNTER_VALBITS;
            i2c_control->status_word |= (button->button_pressed_counter << 13) & BUTTON_COUNTER_VALBITS;
            i2c_control->status_word |= BUTTON_PRESSED_STSBIT;
        }
        else
        {
            i2c_control->status_word &= (~(BUTTON_PRESSED_STSBIT | BUTTON_COUNTER_VALBITS));
        }
    }

    /* these flags are automatically cleared in debounce function */
    if(input_state->sfp_det == ACTIVATED)
        i2c_control->status_word |= SFP_DET_STSBIT;
    else
        i2c_control->status_word &= (~SFP_DET_STSBIT);

    if(input_state->sfp_los == ACTIVATED)
        i2c_control->status_word |= SFP_LOS_STSBIT;
    else
        i2c_control->status_word &= (~SFP_LOS_STSBIT);

    if(input_state->sfp_flt == ACTIVATED)
        i2c_control->status_word |= SFP_FLT_STSBIT;
    else
        i2c_control->status_word &= (~SFP_FLT_STSBIT);

    if(input_state->card_det == ACTIVATED)
        i2c_control->status_word |= CARD_DET_STSBIT;
    else
        i2c_control->status_word &= (~CARD_DET_STSBIT);

    if(input_state->msata_ind == ACTIVATED)
        i2c_control->status_word |= MSATA_IND_STSBIT;
    else
        i2c_control->status_word &= (~MSATA_IND_STSBIT);

    return value;
}

/*******************************************************************************
  * @function   ic2_manager
  * @brief      Handle I2C communication.
  * @param      None.
  * @retval     value: next_state.
  *****************************************************************************/
static ret_value_t ic2_manager(void)
{
    struct st_i2c_status *i2c_control = &i2c_status;
    static uint16_t last_status_word;
    ret_value_t value = OK;

    if (i2c_control->status_word != last_status_word)
        SET_INTERRUPT_TO_CPU;
    else
        RESET_INTERRUPT_TO_CPU;

    last_status_word = i2c_control->status_word;

    switch(i2c_control->state)
    {
        case SLAVE_I2C_LIGHT_RST:       value = GO_TO_LIGHT_RESET; break;
        case SLAVE_I2C_HARD_RST:        value = GO_TO_HARD_RESET; break;
        case SLAVE_I2C_PWR4V5_ERROR:    value = GO_TO_4V5_ERROR; break;
        default:                        value = OK; break;
    }

    return value;
}

/*******************************************************************************
  * @function   led_manager
  * @brief      System LED activity (WAN, LAN, WiFi...).
  * @param      None.
  * @retval     next_state.
  *****************************************************************************/
static ret_value_t led_manager(void)
{
    wan_led_activity();
    lan_led_activity();
    pci_led_activity();
    msata_pci_activity();

    return OK;
}

/*******************************************************************************
  * @function   error_manager
  * @brief      Handle error occuring in startup.
  * @param      error_state: type of error.
  * @retval     None.
  *****************************************************************************/
static void error_manager(ret_value_t error_state)
{
    led_driver_set_led_mode(LED_COUNT, LED_DEFAULT_MODE);
    led_driver_set_led_state(LED_COUNT, LED_OFF);
    led_driver_set_colour(LED_COUNT, RED_COLOUR);

    delay(300);

    switch(error_state)
    {
        case GO_TO_5V_ERROR: led_driver_set_led_state(LED0, LED_ON); break;
        case GO_TO_3V3_ERROR: led_driver_set_led_state(LED1, LED_ON); break;
        case GO_TO_1V8_ERROR: led_driver_set_led_state(LED2, LED_ON); break;
        case GO_TO_1V5_ERROR: led_driver_set_led_state(LED3, LED_ON); break;
        case GO_TO_1V35_ERROR: led_driver_set_led_state(LED4, LED_ON); break;
        case GO_TO_VTT_ERROR: led_driver_set_led_state(LED5, LED_ON); break;
        case GO_TO_1V2_ERROR: led_driver_set_led_state(LED6, LED_ON); break;
        case GO_TO_4V5_ERROR: led_driver_set_led_state(LED7, LED_ON); break;

        default: led_driver_set_led_state(LED_COUNT, LED_ON); break;
    }

    delay(300);
}

/*******************************************************************************
  * @function   app_mcu_cyclic
  * @brief      Main cyclic function.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void app_mcu_cyclic(void)
{
    static states_t next_state = POWER_ON;
    static ret_value_t val = OK;
    static uint8_t error_counter;

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

            next_state = LOAD_SETTINGS;
        }
        break;

        case HARD_RESET:
        {
            next_state = POWER_ON;
        }
        break;

        case LOAD_SETTINGS:
        {
            load_settings();

            next_state = LED_EFFECT;
        }
        break;

        case LED_EFFECT:
        {
            led_driver_knight_rider_effect(WHITE_COLOUR);
            led_driver_set_colour(LED_COUNT, GREEN_COLOUR | BLUE_COLOUR);
            led_driver_set_led_state(LED_COUNT, LED_ON);
            delay(300);
            led_driver_set_led_state(LED_COUNT, LED_OFF);
            led_driver_set_colour(LED_COUNT, WHITE_COLOUR);

            led_driver_set_led_mode(LED_COUNT, LED_DEFAULT_MODE);
            power_control_set_power_led(); /* power led ON */

            next_state = INPUT_MANAGER;
        }
        break;

        case ERROR_STATE:
        {
            error_manager(val);
            error_counter++;

            if(error_counter >= MAX_ERROR_COUNT)
            {
                next_state = HARD_RESET;
                error_counter = 0;
            }
            else
            {
                next_state = ERROR_STATE;
            }
        }
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
            val = ic2_manager();

            switch(val)
            {
                case GO_TO_LIGHT_RESET: next_state = LIGHT_RESET; break;
                case GO_TO_HARD_RESET: next_state = HARD_RESET; break;
                case GO_TO_4V5_ERROR: next_state = ERROR_STATE; break;
                default: next_state = LED_MANAGER; break;
            }
        }
        break;

        case LED_MANAGER:
        {
            led_manager();
            next_state = INPUT_MANAGER;
        }
        break;

        default: /* it should never occur */
        {
            DBG("DEFAULT_STATE\r\n");
            next_state = HARD_RESET;
        }
        break;
    }
}
