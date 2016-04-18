/**
 ******************************************************************************
 * @file    boot_main.c
 * @author  CZ.NIC, z.s.p.o.
 * @date    15-April-2016
 * @brief   Main program body
 ******************************************************************************
 ******************************************************************************
 **/

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx.h"
#include "delay.h"
#include "led_driver.h"
#include "boot_i2c.h"
#include "eeprom.h"
#include "debug_serial.h"

#include "flash.h"
#include "power_control.h"
#include "bootloader.h"

#define LED_INDICATION_DELAY       500

typedef void (*pFunction)(void);

void start_application(void);

enum boot_requests {
    //BOOTLOADER_REQ                      = 0xAA,
    FLASH_ERROR                 = 0x55,
    FLASH_OK                     = 0x88
};

/*******************************************************************************
 * @brief  Main program.
 * @param  None
 * @retval None
 ******************************************************************************/
int boot_main(void)
{
    SYSCFG_MemoryRemapConfig(SYSCFG_MemoryRemap_Flash);

    eeprom_var_t ee_var;
    uint16_t ee_data;

//    /* system initialization */
//    SystemInit();
//    SystemCoreClockUpdate(); /* set HSI and PLL */

//    /* peripheral initialization*/
//    delay_systimer_config();
//    led_driver_config();
//    boot_i2c_config();

//    FLASH_Unlock(); /* Unlock the Flash Program Erase controller */
//    EE_Init(); /* EEPROM Init */

//    __enable_irq();

    bootloader_init();


    int dummy, i;
    unsigned long int log = 0;

    for (i = 1; i < 11; i++)
    {
        dummy += 1000;
        delay(LED_INDICATION_DELAY);
        log += 10000;

        if (i%2)
        {
            led_driver_set_led_state(LED11, LED_OFF);
        }
        else
        {
            led_driver_set_led_state(LED11, LED_ON);
        }
    }

    //flash_config();

    ee_var = EE_ReadVariable(RESET_VIRT_ADDR, &ee_data);

    switch(ee_var)
    {
        /* power on reset - first boot - everything is flashed;
           request for reflashing has never ocurred */
        case VAR_NOT_FOUND:
        {
            start_application();

            DBG("POR1\r\n");
        } break;

        case VAR_FOUND:
        {
            if (ee_data == BOOTLOADER_REQ)
            {
                //TODO - smazat pak po naflashovani BOOTLOADER_REQ nebo nastavit na FLASH_OK
                //kdyz neprijde request do 30s, skocit do aplikace
                power_control_io_config();
                power_control_set_startup_condition();
                power_control_disable_regulators();
                delay(100);

                power_control_enable_regulators();
                power_control_first_startup();
                start_application();
                DBG("Boot\r\n");
            }
            else
            {
                /* application was flashed correctly */
                if (ee_data == FLASH_OK)
                {
                    /* power on reset */
                    start_application();
                    DBG("POR2\r\n");
                }
                else /* error - reset */
                {
                    //wait 30s for flash request and then reset
                    //TODO - turn on power
                }
            }
        } break;

        case VAR_NO_VALID_PAGE : DBG("Boot-No valid page\r\n");
            break;

        default:
            break;
    }

    uint32_t flash_status;

    while(1)
    {
        flash_status = boot_i2c_flash_data();

        if (flash_status == 3)
        {
            EE_WriteVariable(RESET_VIRT_ADDR, FLASH_OK);
            start_application();
        }
    }
}


void start_application(void)
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