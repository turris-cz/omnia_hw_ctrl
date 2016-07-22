/**
 ******************************************************************************
 * @file    bootloader.c
 * @author  CZ.NIC, z.s.p.o.
 * @date    17-April-2016
 * @brief   Bootloader state machine.
 ******************************************************************************
 ******************************************************************************
 **/
#include "boot_i2c.h"
#include "power_control.h"
#include "delay.h"
#include "eeprom.h"
#include "debug_serial.h"
#include "bootloader.h"
#include "led_driver.h"
#include "flash.h"
#include "debounce.h"

#define DELAY_TIMEOUT       5
#define MAX_TIMEOUT         (10*1000) /* 10 sec */
#define MAX_TIMEOUT_CNT     (MAX_TIMEOUT/DELAY_TIMEOUT)

typedef enum bootloader_states {
    POWER_ON,
    RESET_MANAGER,
    TIMEOUT_MANAGER,
    FLASH_MANAGER,
    START_APPLICATION
} boot_state_t;

typedef enum bootloader_return_val {
    GO_TO_POWER_ON,
    GO_TO_RESET_MANAGER,
    GO_TO_TIMEOUT_MANAGER,
    GO_TO_FLASH,
    GO_TO_APPLICATION
} boot_value_t;

typedef void (*pFunction)(void);

/*******************************************************************************
  * @function   bootloader_init
  * @brief      Init of bootloader
  * @param      None
  * @retval     None
  *****************************************************************************/
void bootloader_init(void)
{
     /* system initialization */
    SystemInit();
    SystemCoreClockUpdate(); /* set HSI and PLL */

    /* peripheral initialization*/
    delay_systimer_config();
    led_driver_config();
    boot_i2c_config();

    FLASH_Unlock(); /* Unlock the Flash Program Erase controller */
    EE_Init(); /* EEPROM Init */
    flash_config();
    TIM_DeInit(DEBOUNCE_TIMER);
    TIM_DeInit(USB_TIMEOUT_TIMER);
    __enable_irq();

    led_driver_set_colour(LED_COUNT, GREEN_COLOUR);
    led_driver_reset_effect(ENABLE);
    debug_serial_config();
    DBG("Init\r\n");
}

/*******************************************************************************
  * @function   bootloader_init
  * @brief      Init of bootloader.
  * @param      None
  * @retval     None
  *****************************************************************************/
static void start_application(void)
{
    pFunction app_entry;
    uint32_t app_stack;

    __disable_irq();

    /* Get the application stack pointer (First entry in the application vector table) */
    app_stack = (uint32_t) *((volatile uint32_t*)APPLICATION_ADDRESS);

    /* Get the application entry point (Second entry in the application vector table) */
    app_entry = (pFunction) *(volatile uint32_t*) (APPLICATION_ADDRESS + 4);

    /* Set the application stack pointer */
    __set_MSP(app_stack);

    /* ISB = instruction synchronization barrier. It flushes the pipeline of
     * the processor, so that all instructions following the ISB are fetched
     * from cache or memory again, after the ISB instruction has been completed.
     * Must be called after changing stack pointer according to the documentation.
    */
    __ISB();

    /* Start the application */
    app_entry();
}

/*******************************************************************************
  * @function   reset_manager
  * @brief      Determine a reset reason and following reaction.
  * @param      None
  * @retval     None
  *****************************************************************************/
static boot_value_t reset_manager(void)
{
    eeprom_var_t ee_var;
    uint16_t ee_data;
    boot_value_t retval = GO_TO_RESET_MANAGER;

    ee_var = EE_ReadVariable(RESET_VIRT_ADDR, &ee_data);

    switch(ee_var)
    {
        /* power on reset - first boot - everything is flashed;
           request for reflashing has never ocurred */
        case VAR_NOT_FOUND:
        {
            retval = GO_TO_APPLICATION;

            DBG("R1\r\n");
        } break;

        case VAR_FOUND:
        {
            switch (ee_data)
            {
                case BOOTLOADER_REQ:
                {
                    retval = GO_TO_FLASH;
                    DBG("req\r\n");
                } break;

                case FLASH_NOT_CONFIRMED: /* error */
                {
                    retval = GO_TO_POWER_ON;
                    DBG("ERR\r\n");
                } break;

                case FLASH_CONFIRMED: /* application was flashed correctly */
                {
                    retval = GO_TO_APPLICATION;
                    DBG("R2\r\n");
                } break;

                /* flag has not been saved correctly */
                default: retval = GO_TO_POWER_ON; break;
            }
        } break;

        case VAR_NO_VALID_PAGE : DBG("Boot-No valid page\r\n");
            break;

        default:
            break;
    }

    return retval;
}

/*******************************************************************************
  * @function   bootloader
  * @brief      Main bootloader state machine.
  * @param      None
  * @retval     None
  *****************************************************************************/
void bootloader(void)
{
    static boot_state_t next_state = RESET_MANAGER;
    static boot_value_t val = GO_TO_RESET_MANAGER;
    static flash_i2c_states_t flash_sts = FLASH_CMD_NOT_RECEIVED;
    static uint16_t delay_cnt;
    static uint8_t skip_timeout; /* 0 - leave bootloader after timeout, 1 - stay in bootloader */

    switch(next_state)
    {
        case RESET_MANAGER:
        {
            val = reset_manager();

            switch (val)
            {
                case GO_TO_POWER_ON:    next_state = POWER_ON; break;
                case GO_TO_APPLICATION: next_state = START_APPLICATION; break;
                case GO_TO_FLASH:       next_state = FLASH_MANAGER; break;
                default:                next_state = TIMEOUT_MANAGER; break;
            }
        } break;

        case POWER_ON:
        {
            power_control_io_config();
            power_control_set_startup_condition();
            power_control_disable_regulators();
            delay(100);

            power_control_enable_regulators();
            power_control_first_startup();

            next_state = FLASH_MANAGER;
            skip_timeout = 1; /* dont leave bootloader */
        } break;

        case TIMEOUT_MANAGER:
        {
            if (skip_timeout)
            {
                delay_cnt = 0;
            }
            else
            {
                delay(DELAY_TIMEOUT);
                delay_cnt++;
            }

            if(delay_cnt > MAX_TIMEOUT_CNT)
            {
                EE_WriteVariable(RESET_VIRT_ADDR, FLASH_CONFIRMED); /* old, but valid FW */
                DBG("F_CONF_T\r\n");
                /* shutdown regulators before reset, otherwise power supply can
                 * stay there and causes wrong detection of mmc during boot */
                power_control_disable_regulators();
                delay(100);
                NVIC_SystemReset();
            }
            else
            {
                next_state = FLASH_MANAGER;
            }
        } break;

        case FLASH_MANAGER:
        {
            flash_sts = boot_i2c_flash_data();

            switch(flash_sts)
            {
                case FLASH_CMD_RECEIVED: /* flashing has just started */
                {
                    next_state = FLASH_MANAGER;
                    skip_timeout = 1;
                } break;

                case FLASH_CMD_NOT_RECEIVED: /* nothing has received */
                {
                    next_state = TIMEOUT_MANAGER;
                } break;

                case FLASH_WRITE_OK: /* flashing was successfull */
                {
                    EE_WriteVariable(RESET_VIRT_ADDR, FLASH_CONFIRMED);
                    DBG("F_CONF\r\n");
                    NVIC_SystemReset();
                } break;

                case FLASH_WRITE_ERROR: /* flashing was corrupted */
                {
                    /* flag FLASH_NOT_CONFIRMED is already set */
                    NVIC_SystemReset();
                } break;
            }
        } break;

        case START_APPLICATION:
        {
            start_application();
        } break;
    }
}
