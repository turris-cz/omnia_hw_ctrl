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

#define DELAY_TIMEOUT       5
#define MAX_TIMEOUT         (20*1000) /* 20 sec */
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


void bootloader(void)
{
    static boot_state_t next_state = RESET_MANAGER;
    static boot_value_t val = GO_TO_RESET_MANAGER;
    static flash_i2c_states_t flash_sts = FLASH_CMD_NOT_RECEIVED;
    static uint16_t delay_cnt;
    static uint8_t skip_timeout;

    switch(next_state)
    {
        case RESET_MANAGER:
        {
            //val = reset_manager();
            switch (val)
            {
                case GO_TO_POWER_ON:            next_state = POWER_ON; break;
                case GO_TO_APPLICATION:         next_state = START_APPLICATION; break;
                default: next_state = TIMEOUT_MANAGER; break;
            }
        } break;

        case POWER_ON:
        {
            power_control_set_startup_condition();
            power_control_disable_regulators();
            delay(100);

            power_control_enable_regulators();
            power_control_first_startup();

            next_state = FLASH_MANAGER;
            skip_timeout = 1;
        } break;

        case TIMEOUT_MANAGER:
        {
            if (skip_timeout)
            {
                delay_cnt = 0;
               // skip_timeout = 0;
            }
            else
            {
                delay(DELAY_TIMEOUT);
                delay_cnt++;
            }

            if(delay_cnt > MAX_TIMEOUT_CNT)
            {
                next_state = START_APPLICATION;
                delay_cnt = 0;
            }
            else
            {
                next_state = FLASH_MANAGER;
            }
        } break;

        case FLASH_MANAGER:
        {
            flash_sts = boot_i2c_flash_data();
            //TODO: ukonceni z flashovani - go to application
            if(flash_sts == FLASH_CMD_RECEIVED)
            {
                next_state = FLASH_MANAGER;
                skip_timeout = 1;
            }
            else
            {
                next_state = TIMEOUT_MANAGER;
            }

        } break;

        case START_APPLICATION:
        {

        } break;
    }
}
