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
#include "stm32f0xx_conf.h"
#include "app.h"

#define APPLICATION_ADDRESS        0x08004000
#define RAM_ADDRESS                0x20000000

volatile uint8_t version[20] __attribute__((section (".version_section"))) = VERSION;


/*******************************************************************************
 * @brief  Main program.
 * @param  None
 * @retval None
 ******************************************************************************/
int main(void)
{
    /* Copy interrupt vector table to the RAM. */
    volatile uint32_t *vector_table = (volatile uint32_t *)RAM_ADDRESS;
    uint32_t vector_index = 0;

    for(vector_index = 0; vector_index < 48; vector_index++)
    {
        vector_table[vector_index] = *(volatile uint32_t*)((uint32_t)APPLICATION_ADDRESS + (vector_index << 2));
    }

    /* force AHB reset */
    RCC_AHBPeriphResetCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB | \
        RCC_AHBPeriph_GPIOC | RCC_AHBPeriph_GPIOD | RCC_AHBPeriph_GPIOF, ENABLE);

    /* Enable the SYSCFG peripheral clock*/
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_SYSCFG, ENABLE);

    /* release AHB reset */
    RCC_AHBPeriphResetCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB | \
        RCC_AHBPeriph_GPIOC | RCC_AHBPeriph_GPIOD | RCC_AHBPeriph_GPIOF, DISABLE);

    /* Remap SRAM */
    SYSCFG_MemoryRemapConfig(SYSCFG_MemoryRemap_SRAM);

    __enable_irq();

    app_mcu_init();

    while(1)
    {
        app_mcu_cyclic();
    }
}
