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
static volatile uint32_t timing250ns;

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
}

/*******************************************************************************
  * @function   delay_250ns_timeslot_config
  * @brief      Timer configuration for slot delay in nanoseconds [250 ns].
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void delay_250ns_timeslot_config(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Clock enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14, ENABLE);

    /* Time base configuration - 250 ns interrupt */
    TIM_TimeBaseStructure.TIM_Period = 4 - 1;
    TIM_TimeBaseStructure.TIM_Prescaler = 3 - 1;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(NANOSECONDS_TIMER, &TIM_TimeBaseStructure);

    TIM_ARRPreloadConfig(NANOSECONDS_TIMER, ENABLE);
    /* TIM Interrupts enable */
    TIM_ITConfig(NANOSECONDS_TIMER, TIM_IT_Update, ENABLE);

    /* TIM enable counter */
    TIM_Cmd(NANOSECONDS_TIMER, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = TIM14_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/*******************************************************************************
  * @function   delay_250ns_timeslot_disable
  * @brief      Disable nanosecond timer.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void delay_250ns_timeslot_disable(void)
{
    TIM_Cmd(NANOSECONDS_TIMER, DISABLE);
    NANOSECONDS_TIMER->CNT = 0;
}

/******************************************************************************
  * @function   power_control_nsdelay
  * @brief      Inserts a number of delay slot.
  * @param      timeslot: 1 timeslot = 250ns
  * @retval     None.
  *****************************************************************************/
void delay_250ns_timeslot(uint32_t timeslot)
{
    timing250ns = timeslot;

    while(timing250ns != 0u);
}

/******************************************************************************
  * @function   delay_250ns_timeslot_decrement
  * @brief      Decrements the timingdelay variable.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void delay_250ns_timeslot_decrement(void)
{
    if (timing250ns != 0x00)
    {
        timing250ns--;
    }
}
