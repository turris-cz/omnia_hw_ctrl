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
#include "gd32f1x0.h"
#include "app.h"
#include "led_driver.h"

#define APPLICATION_ADDRESS         0x08005000
#define RAM_ADDRESS                 0x20000000
#define BOOTLOADER_ADDRESS          0x08000000

typedef void (*pFunction)(void);

void start_bootloader(void);

/*******************************************************************************
 * @brief  Main program.
 * @param  None
 * @retval None
 ******************************************************************************/
int main(void)
{
    /* Copy interrupt vector table to the RAM. */
//    volatile uint32_t *vector_table = (volatile uint32_t *)RAM_ADDRESS;
//    uint32_t vector_index = 0;

//    for(vector_index = 0; vector_index < 68; vector_index++)
//    {
//        vector_table[vector_index] = *(volatile uint32_t*)((uint32_t)APPLICATION_ADDRESS + (vector_index << 2));
//    }

    /* force AHB reset */
    rcu_periph_reset_enable(RCU_GPIOARST);
    rcu_periph_reset_enable(RCU_GPIOBRST);
    rcu_periph_reset_enable(RCU_GPIOCRST);
    rcu_periph_reset_enable(RCU_GPIODRST);
    rcu_periph_reset_enable(RCU_GPIOFRST);
    //rcu_periph_reset_enable(RCU_SPI0RST);

//    RCC_AHBPeriphResetCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB | \
//        RCC_AHBPeriph_GPIOC | RCC_AHBPeriph_GPIOD | RCC_AHBPeriph_GPIOF, ENABLE);

//    /* Enable the SYSCFG peripheral clock*/
//    RCC_APB2PeriphResetCmd(RCC_APB2Periph_SYSCFG, ENABLE);

    /* release AHB reset */
    rcu_periph_reset_disable(RCU_GPIOARST);
    rcu_periph_reset_disable(RCU_GPIOBRST);
    rcu_periph_reset_disable(RCU_GPIOCRST);
    rcu_periph_reset_disable(RCU_GPIODRST);
    rcu_periph_reset_disable(RCU_GPIOFRST);
    //rcu_periph_reset_disable(RCU_SPI0RST);
    //spi_i2s_deinit(SPI0);
    //rcu_periph_clock_disable(RCU_SPI0);

//    RCC_AHBPeriphResetCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB | \
//        RCC_AHBPeriph_GPIOC | RCC_AHBPeriph_GPIOD | RCC_AHBPeriph_GPIOF, DISABLE);

//    RCC_APB2PeriphResetCmd(RCC_APB2Periph_SYSCFG, DISABLE);

    /* Remap SRAM */
   // SYSCFG_MemoryRemapConfig(SYSCFG_MemoryRemap_SRAM);



    app_mcu_init();

    while(1)
    {
       app_mcu_cyclic();
    }
}

void start_bootloader(void)
{
    pFunction boot_entry;
    uint32_t boot_stack;

    __disable_irq();

    /* Get the Bootloader stack pointer (First entry in the Bootloader vector table) */
    boot_stack = (uint32_t) *((volatile uint32_t*)BOOTLOADER_ADDRESS);

    /* Get the Bootloader entry point (Second entry in the Bootloader vector table) */
    boot_entry = (pFunction) *(volatile uint32_t*) (BOOTLOADER_ADDRESS + 4);

    /* Set the Bootloader stack pointer */
    __set_MSP(boot_stack);

        /* ISB = instruction synchronization barrier. It flushes the pipeline of
     * the processor, so that all instructions following the ISB are fetched
     * from cache or memory again, after the ISB instruction has been completed.
     * Must be called after changing stack pointer according to the documentation.
    */
    __ISB();

    /* Start the Bootloader */
    boot_entry();

    while(1);
}
