/**
  ******************************************************************************
  * @file    stm32f10x_it.c
  * @author  MCD Application Team
  * @version V3.3.0
  * @date    04/16/2010
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and peripherals
  *          interrupt service routine.
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2010 STMicroelectronics</center></h2>
  */

/* Includes ------------------------------------------------------------------*/

#include "boot_led_driver.h"
#include "gd32f1x0_it.h"
#include "debounce.h"
#include "delay.h"
#include "power_control.h"
#include "debug_serial.h"

#include "boot_i2c.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSV_Handler exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
    delay_timing_decrement();
}

/******************************************************************************/
/*            STM32F10x Peripherals Interrupt Handlers                        */
/******************************************************************************/

/**
  * @brief  This function handles TIM16 global interrupt request.
  * @param  None
  * @retval None
  */
void TIMER15_IRQHandler(void)
{
    if (timer_interrupt_flag_get(DEBOUNCE_TIMER, TIMER_INT_FLAG_UP) == SET)
    {
        timer_interrupt_flag_clear(DEBOUNCE_TIMER, TIMER_INT_FLAG_UP);
    }
}

/**
  * @brief  This function handles TIM3 global interrupt request.
  * @param  None
  * @retval None
  */
void TIMER2_IRQHandler(void)
{
    if(SET == timer_interrupt_flag_get(LED_TIMER, TIMER_INT_UP))
    {
        led_driver_send_frame();
        timer_interrupt_flag_clear(LED_TIMER, TIMER_INT_UP);
    }
}


/**
  * @brief  This function handles TIM17 global interrupt request.
  * @param  None
  * @retval None
  */
void TIMER16_IRQHandler(void)
{
    if (timer_interrupt_flag_get(USB_TIMEOUT_TIMER, TIMER_INT_FLAG_UP) == SET)
        timer_interrupt_flag_clear(USB_TIMEOUT_TIMER, TIMER_INT_FLAG_UP);
}

/**
  * @brief  This function handles I2C global interrupt request.
  * @param  None
  * @retval None
  */
void I2C1_EV_IRQHandler(void)
{
    boot_i2c_handler();
}

#define LED_BLINK_TIMEOUT   8
/**
  * @brief  This function handles TIM3 global interrupt request.
  * @param  None
  * @retval None
  */
void TIMER5_DAC_IRQHandler(void)
{
    static uint8_t led_blink;
    static uint8_t timeout;

    if (timer_interrupt_flag_get(LED_EFFECT_TIMER, TIMER_INT_FLAG_UP) == SET)
    {
        timeout++;

        if (timeout >= LED_BLINK_TIMEOUT)
        {
            timeout = 0;

            switch(led_blink)
            {
                case 0:
                {
                    led_driver_set_led_state(LED_COUNT, LED_ON);
                    led_blink++;
                } break;

                case 1:
                {
                    led_driver_set_led_state(LED_COUNT, LED_OFF);
                    led_blink = 0;
                }break;
            }
        }

        timer_interrupt_flag_clear(LED_EFFECT_TIMER, TIMER_INT_FLAG_UP);
    }
}

/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
