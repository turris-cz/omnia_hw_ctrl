/**
 ******************************************************************************
 * @file    debug_serial.c
 * @author  CZ.NIC, z.s.p.o.
 * @date    25-August-2015
 * @brief   Debug prints on serial port.
 ******************************************************************************
 ******************************************************************************
 **/
#include "stm32f0xx_conf.h"
#include "debug_serial.h"

#define SERIAL_PORT      USART1

#if DBG_ENABLE

/*******************************************************************************
  * @function   debug_serial_config
  * @brief      Configuration of UART peripheral.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void debug_serial_config(void)
{
    USART_InitTypeDef USART_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, DISABLE);
    USART_DeInit(SERIAL_PORT);

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_1);

    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_9 | GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(SERIAL_PORT, &USART_InitStructure);

    USART_Cmd(SERIAL_PORT, ENABLE);
}

/*******************************************************************************
  * @function   debug_print
  * @brief      Send data over the serial port.
  * @param      buffer: pointer to data buffer.
  * @retval     None.
  *****************************************************************************/
void debug_print(const char *buffer)
{
    while (*buffer)
    {
        /* Loop until the end of transmission */
        while (USART_GetFlagStatus(SERIAL_PORT, USART_FLAG_TXE) == RESET)
            ;
        USART_SendData(SERIAL_PORT, *buffer++);
    }
    while (USART_GetFlagStatus(SERIAL_PORT, USART_FLAG_TC) == RESET)
        ;
}

#endif
