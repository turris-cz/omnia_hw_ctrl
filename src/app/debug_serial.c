/**
 ******************************************************************************
 * @file    debug_serial.c
 * @author  CZ.NIC, z.s.p.o.
 * @date    25-August-2015
 * @brief   Debug prints on serial port.
 ******************************************************************************
 ******************************************************************************
 **/
#include "string.h"
#include "stm32f0xx_conf.h"

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

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_1);

    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_2 | GPIO_Pin_3;
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
    USART_Init(USART2, &USART_InitStructure);

    USART_Cmd(USART2, ENABLE);
    //TODO: change to UART1 on Turris Lite !!!
}

/*******************************************************************************
  * @function   debug_send_data
  * @brief      Send data over the serial port.
  * @param      buffer: pointer to data buffer.
  * @param      count: number of bytes
  * @retval     None.
  *****************************************************************************/
static void debug_send_data(const char *buffer, uint16_t count)
{
    while(count--)
    {
        USART_SendData(USART2, *buffer++);
        /* Loop until the end of transmission */
        while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET)
            ;
        while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == SET)
            ;
    }
}

/*******************************************************************************
  * @function   debug_print
  * @brief      Send buffer over the serial port.
  * @param      buffer: pointer to data buffer.
  * @retval     None.
  *****************************************************************************/
void debug_print(const char *buffer)
{
    uint16_t count = strlen(buffer);
    debug_send_data(buffer, count);
}
