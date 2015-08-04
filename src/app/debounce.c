/**
 ******************************************************************************
 * @file    debounce.c
 * @author  CZ.NIC, z.s.p.o.
 * @date    21-July-2015
 * @brief   Debounce switches and inputs from PG signals
 ******************************************************************************
 ******************************************************************************
 **/
#include "stm32f0xx.h"
#include "debounce.h"
#include "stm32f0xx_conf.h"

/* max count of input states to be checked */
enum max_input_states {
    MAX_STATES_MANRES               = 8,
    MAX_STATES_SYSRES_OUT           = 3,
    MAX_STATES_DBG                  = 3,
    MAX_STATES_MRES                 = 3,
    MAX_STATES_PG_5V                = 3,
    MAX_STATES_PG_3V3               = 3,
    MAX_STATES_PG_1V35              = 3,
    MAX_STATES_PG_4V5               = 3,
    MAX_STATES_PG_1V8               = 3,
    MAX_STATES_PG_1V5               = 3,
    MAX_STATES_PG_1V2               = 3,
    MAX_STATES_PG_VTT               = 3,
    MAX_STATES_USB30_OVC            = 3,
    MAX_STATES_USB31_OVC            = 3,
    MAX_STATES_RTC_ALARM            = 3,
    MAX_STATES_LED_BRT              = 10
};

enum input_mask {
    MAN_RES_MASK                    = 0x0001,
    SYSRES_OUT_MASK                 = 0x0002,
    DBG_RES_MASK                    = 0x0004,
    MRES_MASK                       = 0x0008,
    PG_5V_MASK                      = 0x0010,
    PG_3V3_MASK                     = 0x0020,
    PG_1V35_MASK                    = 0x0040,
    PG_4V5_MASK                     = 0x0080,
    PG_1V8_MASK                     = 0x0100,
    PG_1V5_MASK                     = 0x0200,
    PG_1V2_MASK                     = 0x0400,
    PG_VTT_MASK                     = 0x0800,
    USB30_OVC_MASK                  = 0x1000,
    USB31_OVC_MASK                  = 0x2000,
    RTC_ALARM_MASK                  = 0x4000,
    LED_BRT_MASK                    = 0x8000,
};

struct input_sig debounce_input_signal;


#define  DEBOUNCE_TIM_PERIODE       (200 - 1)//200 = 5ms
#define  DEBOUNCE_TIM_PRESCALER     (200 - 1)

/*******************************************************************************
  * @function   debounce_timer_config
  * @brief      Timer configuration for debouncing. Regulary interrupt every 5ms.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
static void debounce_timer_config(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    // Clock enable
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM16, ENABLE);

    /* Time base configuration */
    TIM_TimeBaseStructure.TIM_Period = DEBOUNCE_TIM_PERIODE;
    TIM_TimeBaseStructure.TIM_Prescaler = DEBOUNCE_TIM_PRESCALER;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(DEBOUNCE_TIMER, &TIM_TimeBaseStructure);

    TIM_ARRPreloadConfig(DEBOUNCE_TIMER, ENABLE);
    /* TIM Interrupts enable */
    TIM_ITConfig(DEBOUNCE_TIMER, TIM_IT_Update, ENABLE);

    /* TIM enable counter */
    TIM_Cmd(DEBOUNCE_TIMER, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = TIM16_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority = 0x03;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/*******************************************************************************
  * @function   debounce_man_res
  * @brief      Debounce of MANRES input.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
static uint16_t debounce_man_res(const uint16_t input)
{
    static uint16_t counter;
    uint16_t output = 0;

    if (input) //signal released
    {
        if (counter > 0)
            counter--;
    }
    else //signal falls to low
    {
        if (counter < MAX_STATES_MANRES)
            counter++;
    }

    if (counter == 0)
        output = 0;
    else
    {
        if(counter >= MAX_STATES_MANRES)
        {
            output = 1;
            counter = MAX_STATES_MANRES;
        }
    }
    return output;
}

/*******************************************************************************
  * @function   debounce_sysres_out
  * @brief      Debounce of SYSRES_OUT input.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
static uint16_t debounce_sysres_out(const uint16_t input)
{
    static uint16_t counter;
    uint16_t output = 0;

    if (input) //signal released
    {
        if (counter > 0)
            counter--;
    }
    else //signal falls to low
    {
        if (counter < MAX_STATES_SYSRES_OUT)
            counter++;
    }

    if (counter == 0)
        output = 0;
    else
    {
        if(counter >= MAX_STATES_SYSRES_OUT)
        {
            output = 1;
            counter = MAX_STATES_SYSRES_OUT;
        }
    }
    return output;
}

/*******************************************************************************
  * @function   debounce_input_timer_handler
  * @brief      Main debounce function. Called in timer interrupt handler.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void debounce_input_timer_handler(void)
{
    uint16_t port_state;
    struct input_sig *input_state = &debounce_input_signal;

    port_state = GPIO_ReadInputData(GPIOB); //read whole port

    if (debounce_man_res(port_state & MAN_RES_MASK))
        input_state->man_res = 1;
    if (debounce_sysres_out((port_state & SYSRES_OUT_MASK) >> 1))
        input_state->sysres_out = 1;
    //TODO - continue or rewrite it to save space ?

}

/*******************************************************************************
  * @function   debounce_config
  * @brief      Debouncer configuration.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void debounce_config(void)
{
    debounce_timer_config();
}
