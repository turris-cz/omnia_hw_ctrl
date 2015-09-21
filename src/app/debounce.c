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
#include "power_control.h"
#include "delay.h"
#include "led_driver.h"

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

#define MAX_INPUT_STATES            3
static uint16_t debounced_state;
static uint16_t port_state[MAX_INPUT_STATES];


#define  DEBOUNCE_TIM_PERIODE       (300 - 1)//300 -> 5ms; 600 -> 10ms
#define  DEBOUNCE_TIM_PRESCALER     (800 - 1)

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
  * @function   debounce_input_timer_handler
  * @brief      Main debounce function. Called in timer interrupt handler.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void debounce_input_timer_handler(void)
{
    static uint16_t idx;

    port_state[idx] = ~(GPIO_ReadInputData(GPIOB)); //read whole port
    idx++;

    if (idx >= MAX_INPUT_STATES)
        idx = 0;
}

/*******************************************************************************
  * @function   debounce_check_inputs
  * @brief      Check input signal.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void debounce_check_inputs(void)
{
    uint16_t i, port_changed;
    static uint16_t last_debounce_state;
    static uint8_t man_reset;

    last_debounce_state = debounced_state;

    debounced_state = 0xFFFF; //init for calculation - include of all 16 inputs

    for (i = 0; i < MAX_INPUT_STATES; i++)
    {
        debounced_state = debounced_state & port_state[i];
    }

    //TODO: reaction on level or edge (falling) ?
    port_changed = (debounced_state ^ last_debounce_state) & debounced_state;
    //port_changed = debounced_state;

    if (port_changed & MAN_RES_MASK)
    {
        //manual reset occured: set init state - disconnect switches
        GPIO_SetBits(CFG_CTRL_PIN_PORT, CFG_CTRL_PIN);
        man_reset = 1;
    }
    else
    {
        if (man_reset) //manual reset ocurred last cycle
        {
            sysres_out_startup();
            reset();
            man_reset = 0;
        }
    }

    if (port_changed & SYSRES_OUT_MASK)
    {
        //no reaction necessary
    }

    if (port_changed & DBG_RES_MASK)
    {
        //no reaction necessary
    }

    // reaction: follow MRES signal
    if (port_changed & MRES_MASK)
    {
        GPIO_ResetBits(RES_RAM_PIN_PORT, RES_RAM_PIN);
    }
    else
    {
        GPIO_SetBits(RES_RAM_PIN_PORT, RES_RAM_PIN);
    }

    if ((port_changed & PG_5V_MASK) || (port_changed & PG_3V3_MASK) ||
         (port_changed & PG_1V35_MASK) || (port_changed & PG_4V5_MASK) ||
         (port_changed & PG_1V8_MASK) || (port_changed & PG_1V5_MASK) ||
         (port_changed & PG_1V2_MASK) || (port_changed & PG_VTT_MASK))
    {
        power_control_disable_regulator();
        /* 100ms delay between the first and last voltage power-down
         * defined in Marvell HW specification, pg.97 (power-down sequence) */
        delay(100);
        NVIC_SystemReset(); // SW reset
    }

    if (port_changed & USB30_OVC_MASK)
    {
        power_control_usb(USB3_PORT0, USB_OFF);
        //TODO - when USB_ON again?
        TIM_Cmd(USB_TIMEOUT_TIMER, ENABLE);
    }


    if (port_changed & USB31_OVC_MASK)
    {
        power_control_usb(USB3_PORT1, USB_OFF);
        //TODO - when USB_ON again?
        TIM_Cmd(USB_TIMEOUT_TIMER, ENABLE);
    }


    if (port_changed & RTC_ALARM_MASK)
    {
        //TODO - probably no reaction needed
    }

    if (port_changed & LED_BRT_MASK)
    {
        led_driver_step_brightness();
    }

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
