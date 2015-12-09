/**
 ******************************************************************************
 * @file    delay.c
 * @author  CZ.NIC, z.s.p.o.
 * @date    22-July-2015
 * @brief   This file ensures delay driver using of System Timer
 ******************************************************************************
 ******************************************************************************
 **/
/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx.h"
#include "delay.h"

static volatile uint32_t timingdelay;
/* counter[ms] used for system initialization of the board
 * (reading configuration through I2C) */
volatile uint32_t delay_counter_ms;
/* after reading configuration "delay_counter_ms" is not needed and is stopped */
volatile uint16_t delay_counter_stop;

/* Private functions ---------------------------------------------------------*/
/*******************************************************************************
  * @function   delay_systimer_config
  * @brief      Setup SysTick Timer for 1 msec interrupts.
  * @param      None
  * @retval     None
  *****************************************************************************/
void delay_systimer_config(void)
{
    if (SysTick_Config(SystemCoreClock / 1000u))
    {
        /* Capture error */
        while (1);
    }
    NVIC_SetPriority(SysTick_IRQn, 2);
}

/******************************************************************************
  * @function   delay
  * @brief      Inserts a delay time.
  * @param      nTime: specifies the delay time length, in miliseconds.
  * @retval     None
  *****************************************************************************/
void delay(volatile uint32_t nTime)
{
    timingdelay = nTime;

    while(timingdelay != 0u);
}

/******************************************************************************
  * @function   delay_timing_decrement
  * @brief      Decrements the TimingDelay variable.
  * @param      None
  * @retval     None
  *****************************************************************************/
void delay_timing_decrement(void)
{
    if (timingdelay != 0x00)
    {
        timingdelay--;
    }

    if (!delay_counter_stop)
        delay_counter_ms++;
}
