/*!
    \file  gd32f1x0_it.c
    \brief interrupt service routines

    \version 2016-01-15, V1.0.0, demo for GD32F1x0
    \version 2016-05-13, V2.0.0, demo for GD32F1x0
    \version 2019-11-20, V3.0.0, demo for GD32F1x0
*/

/*
    Copyright (c) 2019, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this 
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice, 
       this list of conditions and the following disclaimer in the documentation 
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors 
       may be used to endorse or promote products derived from this software without 
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
OF SUCH DAMAGE.
*/

#include "gd32f1x0_it.h"
#include "led_driver.h"
#include "debounce.h"
#include "delay.h"
#include "msata_pci.h"
#include "wan_lan_pci_status.h"
#include "slave_i2c_device.h"
#include "power_control.h"
#include "debug_serial.h"

/*!
    \brief      this function handles NMI exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void NMI_Handler(void)
{
}

/*!
    \brief      this function handles HardFault exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void HardFault_Handler(void)
{
    /* if Hard Fault exception occurs, go to infinite loop */
    while (1);
}

/*!
    \brief      this function handles MemManage exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void MemManage_Handler(void)
{
    /* if Memory Manage exception occurs, go to infinite loop */
    while (1);
}

/*!
    \brief      this function handles BusFault exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void BusFault_Handler(void)
{
    /* if Bus Fault exception occurs, go to infinite loop */
    while (1);
}

/*!
    \brief      this function handles UsageFault exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void UsageFault_Handler(void)
{
    /* if Usage Fault exception occurs, go to infinite loop */
    while (1);
}

/*!
    \brief      this function handles SVC exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void SVC_Handler(void)
{
}

/*!
    \brief      this function handles DebugMon exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void DebugMon_Handler(void)
{
}

/*!
    \brief      this function handles PendSV exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void PendSV_Handler(void)
{
}

/*!
    \brief      this function handles SysTick exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void SysTick_Handler(void)
{
    delay_timing_decrement();
}


/**
  * @brief  This function handles TIM16 global interrupt request.
  * @param  None
  * @retval None
  */
void TIM15_IRQHandler(void)
{
    if (timer_interrupt_flag_get(DEBOUNCE_TIMER, TIMER_INT_FLAG_UP) != RESET)
    {
        debounce_input_timer_handler();
        timer_interrupt_flag_clear(DEBOUNCE_TIMER, TIMER_INT_FLAG_UP);
    }
}

/**
  * @brief  This function handles TIM3 global interrupt request.
  * @param  None
  * @retval None
  */
void TIM2_IRQHandler(void)
{
    if (timer_interrupt_flag_get(LED_TIMER, TIMER_INT_FLAG_UP) != RESET)
    {
        led_driver_send_frame();
        timer_interrupt_flag_clear(LED_TIMER, TIMER_INT_FLAG_UP);
    }
}


/**
  * @brief  This function handles TIM17 global interrupt request.
  * @param  None
  * @retval None
  */
void TIM16_IRQHandler(void)
{
    struct st_i2c_status *i2c_control = &i2c_status;

    if (timer_interrupt_flag_get(USB_TIMEOUT_TIMER, TIMER_INT_FLAG_UP) != RESET)
    {
        power_control_usb(USB3_PORT0, USB_ON);
        power_control_usb(USB3_PORT1, USB_ON);

        i2c_control->status_word |= USB30_PWRON_STSBIT | USB31_PWRON_STSBIT;

        power_control_usb_timeout_disable();

        timer_interrupt_flag_clear(USB_TIMEOUT_TIMER, TIMER_INT_FLAG_UP);
    }
}

/**
  * @brief  This function handles I2C global interrupt request.
  * @param  None
  * @retval None
  */
void I2C2_IRQHandler(void)
{
    slave_i2c_handler();
}

/**
  * @brief  This function handles TIM3 global interrupt request.
  * @param  None
  * @retval None
  */
void TIM5_IRQHandler(void)
{
    if (timer_interrupt_flag_get(LED_EFFECT_TIMER, TIMER_INT_FLAG_UP) != RESET)
    {
        led_driver_knight_rider_effect_handler();
        timer_interrupt_flag_clear(LED_EFFECT_TIMER, TIMER_INT_FLAG_UP);
    }
}

void WWDG_IRQHandler(void)
{
    while(1);
}
