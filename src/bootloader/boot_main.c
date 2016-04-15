/**
 ******************************************************************************
 * @file    main.c
 * @author  CZ.NIC, z.s.p.o.
 * @date    21-July-2015
 * @brief   Main program body
 ******************************************************************************
 ******************************************************************************
 **/

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx.h"
#include "delay.h"
#include "led_driver.h"
#include "boot_i2c.h"


#define LED_INDICATION_DELAY       500

#define APPLICATION_ADDRESS         0x08004000
#define RAM_ADDRESS                 0x20000000
#define BOOTLOADER_ADDRESS          0x08000000

typedef void (*pFunction)(void);

void start_application(void);


/*******************************************************************************
 * @brief  Main program.
 * @param  None
 * @retval None
 ******************************************************************************/
int boot_main(void)
{
    SYSCFG_MemoryRemapConfig(SYSCFG_MemoryRemap_Flash);

    /* system initialization */
    SystemInit();
    SystemCoreClockUpdate(); /* set HSI and PLL */

    /* peripheral initialization*/
    delay_systimer_config();
    led_driver_config();
    slave_i2c_config();

    __enable_irq();

    led_driver_set_colour(LED_COUNT, GREEN_COLOUR);
    led_driver_set_led_state(LED_COUNT, LED_ON);

    int dummy, i;
    unsigned long int log = 0;

    for (i = 1; i < 21; i++)
    {
        dummy += 1000;
        delay(LED_INDICATION_DELAY);
        log += 10000;

        if (i%2)
        {
            led_driver_set_led_state(LED_COUNT, LED_OFF);
        }
        else
        {
            led_driver_set_led_state(LED_COUNT, LED_ON);
        }
    }

    slave_i2c_process_data();

    start_application();


    while(1)
    {
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
