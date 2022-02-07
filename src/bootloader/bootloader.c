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
#include "boot_led_driver.h"
#include "flash.h"
#include "debounce.h"

typedef enum bootloader_states {
    POWER_ON,
    STARTUP_MANAGER,
    RESET_MANAGER,
    FLASH_MANAGER,
    START_APPLICATION,
    RESET_TO_APPLICATION
} boot_state_t;

typedef enum bootloader_return_val {
    GO_TO_POWER_ON,
    GO_TO_STARTUP_MANAGER,
    GO_TO_RESET_MANAGER,
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
    nvic_priority_group_set(NVIC_PRIGROUP_PRE1_SUB3);

   // timer_deinit(LED_TIMER);

    /* peripheral initialization*/
    delay_systimer_config();
    led_driver_config();
    boot_i2c_config();

    fmc_unlock(); /* Unlock the Flash Program Erase controller */
    EE_Init(); /* EEPROM Init */
    flash_config();
   // timer_deinit(DEBOUNCE_TIMER);
   // timer_deinit(USB_TIMEOUT_TIMER);
    __enable_irq();

    led_driver_set_colour(LED_COUNT, GREEN_COLOUR);
    led_driver_reset_effect(ENABLE);
    debug_serial_config();

    /* pin settings for SYSRES_OUT signal */

    rcu_periph_clock_enable(SYSRES_OUT_PIN_PERIPH_CLOCK);
    gpio_mode_set(SYSRES_OUT_PIN_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, SYSRES_OUT_PIN);
    gpio_output_options_set(SYSRES_OUT_PIN_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_2MHZ, SYSRES_OUT_PIN);

    gpio_bit_set(SYSRES_OUT_PIN_PORT, SYSRES_OUT_PIN); /* dont control this ! */

    DBG_UART("Init\r\n");
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

    spi_disable(SPI0);
    spi_i2s_deinit(SPI0);
    timer_disable(LED_TIMER);
    timer_deinit(LED_TIMER);
    timer_disable(DEBOUNCE_TIMER);
    timer_deinit(DEBOUNCE_TIMER);
    timer_disable(LED_EFFECT_TIMER);
    timer_deinit(LED_EFFECT_TIMER);
    timer_disable(USB_TIMEOUT_TIMER);
    timer_deinit(USB_TIMEOUT_TIMER);
    i2c_disable(I2C1);
    i2c_deinit(I2C1);
    usart_disable(USART0);
    usart_deinit(USART0);
    gpio_deinit(GPIOA);
    gpio_deinit(GPIOB);
    gpio_deinit(GPIOC);
    gpio_deinit(GPIOD);
    gpio_deinit(GPIOF);

    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL = 0;

    rcu_deinit();



    /* Get the application stack pointer (First entry in the application vector table) */
    app_stack = (uint32_t) *((volatile uint32_t*)APPLICATION_ADDRESS);

    /* Get the application entry point (Second entry in the application vector table) */
    app_entry = (pFunction) *(volatile uint32_t*) (APPLICATION_ADDRESS + 4);


    SCB->VTOR = APPLICATION_ADDRESS;

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
  * @function   startup_manager
  * @brief      Determine a reset reason and following reaction.
  * @param      None
  * @retval     None
  *****************************************************************************/
static boot_value_t startup_manager(void)
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

            DBG_UART("R1\r\n");
        } break;

        case VAR_FOUND:
        {
            switch (ee_data)
            {
                case BOOTLOADER_REQ:
                {
                    retval = GO_TO_FLASH;
                    DBG_UART("req\r\n");
                } break;

                case FLASH_NOT_CONFIRMED: /* error */
                {
                    retval = GO_TO_POWER_ON;
                    DBG_UART("ERR\r\n");
                } break;

                case FLASH_CONFIRMED: /* application was flashed correctly */
                {
                    retval = GO_TO_APPLICATION;
                    DBG_UART("R2\r\n");
                } break;

                /* flag has not been saved correctly */
                default: retval = GO_TO_POWER_ON; break;
            }
        } break;

        case VAR_NO_VALID_PAGE :
        {
            retval = GO_TO_POWER_ON;
            DBG_UART("Boot-No valid page\r\n");
        }
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
    static boot_state_t next_state = STARTUP_MANAGER;
    static boot_value_t val = GO_TO_RESET_MANAGER;
    static flash_i2c_states_t flash_sts = FLASH_CMD_NOT_RECEIVED;
    static uint8_t flash_confirmed;
    static uint8_t power_supply_failure; /* if power supply disconnection occurred */
    uint8_t system_reset;

    switch(next_state)
    {
        case STARTUP_MANAGER:
        {
            val = startup_manager();

            switch (val)
            {
                case GO_TO_POWER_ON:
                {
                    EE_WriteVariable(RESET_VIRT_ADDR, FLASH_NOT_CONFIRMED);
                    next_state = POWER_ON;
                } break;
                case GO_TO_APPLICATION:
                {
                    next_state = START_APPLICATION;
                } break;
                case GO_TO_FLASH:
                {
                    EE_WriteVariable(RESET_VIRT_ADDR, FLASH_NOT_CONFIRMED);
                    next_state = FLASH_MANAGER;
                } break;

                default: next_state = POWER_ON;    break;
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
            power_supply_failure = 1;
            next_state = FLASH_MANAGER;
        } break;

        case FLASH_MANAGER:
        {
            flash_sts = boot_i2c_flash_data();

            switch(flash_sts)
            {
                case FLASH_CMD_RECEIVED: /* flashing has just started */
                {
                    next_state = RESET_MANAGER;
                } break;

                case FLASH_CMD_NOT_RECEIVED: /* nothing has received */
                {
                    next_state = RESET_MANAGER;
                } break;

                case FLASH_WRITE_OK: /* flashing was successfull */
                {
                    if (!flash_confirmed)
                    {
                        EE_WriteVariable(RESET_VIRT_ADDR, FLASH_CONFIRMED);
                        flash_confirmed = 1;
                    }

                    next_state = RESET_MANAGER;
                    DBG_UART("F_CONF\r\n");
                } break;

                case FLASH_WRITE_ERROR: /* flashing was corrupted */
                {
                    /* flag FLASH_NOT_CONFIRMED is already set */
                    next_state = RESET_MANAGER;
                } break;
            }
        } break;

        case RESET_MANAGER:
        {
            system_reset = gpio_output_bit_get(SYSRES_OUT_PIN_PORT, SYSRES_OUT_PIN);

            if(system_reset == 0) /* reset is active in low level */
            {
                next_state = RESET_TO_APPLICATION;
            }
            else
            {
                next_state = FLASH_MANAGER;
            }
        } break;

        case START_APPLICATION:
        {
            start_application();
        } break;

        case RESET_TO_APPLICATION:
        {
            /* power supply wasnt disconnected and no command for flashing was received */
            if ((power_supply_failure == 0) && (flash_sts == FLASH_CMD_NOT_RECEIVED))
            {
                /* we have old, but valid FW */
                EE_WriteVariable(RESET_VIRT_ADDR, FLASH_CONFIRMED);
            }

            /* shutdown regulators before reset, otherwise power supply can
            * stay there and causes wrong detection of mmc during boot */
            power_control_set_startup_condition();
            power_control_disable_regulators();
            delay(100);
            NVIC_SystemReset();
        } break;
    }
}
