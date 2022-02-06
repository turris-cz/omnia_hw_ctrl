/**
 ******************************************************************************
 * @file    debug_serial.c
 * @author  CZ.NIC, z.s.p.o.
 * @date    29-October-2021
 * @brief   Debug prints on serial port.
 ******************************************************************************
 ******************************************************************************
 **/
#include "string.h"
#include "gd32f1x0.h"
#include "debug_serial.h"

#define SERIAL_PORT      USART0

#if DBG_ENABLE && !defined(DBG_BAUDRATE)
#error build system did not define DBG_BAUDRATE macro
#endif

/*******************************************************************************
  * @function   debug_serial_config
  * @brief      Configuration of UART peripheral.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void debug_serial_config(void)
{
#if DBG_ENABLE
    /* enable GPIO clock */
    rcu_periph_clock_enable(RCU_GPIOA);

    /* enable USART clock */
    rcu_periph_clock_enable(RCU_USART0);

    /* connect port to USARTx_Tx */
    gpio_af_set(GPIOA, GPIO_AF_1, GPIO_PIN_9);

    /* connect port to USARTx_Rx */
    gpio_af_set(GPIOA, GPIO_AF_1, GPIO_PIN_10);

    /* configure USART Tx as alternate function push-pull */
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_9);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ, GPIO_PIN_9);

    /* configure USART Rx as alternate function push-pull */
    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_10);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_10MHZ, GPIO_PIN_10);

    /* USART configure */
    usart_deinit(SERIAL_PORT);
    usart_baudrate_set(SERIAL_PORT, DBG_BAUDRATE);
    usart_parity_config(SERIAL_PORT, USART_PM_NONE);
    usart_word_length_set(SERIAL_PORT, USART_WL_8BIT);
    usart_stop_bit_set(SERIAL_PORT, USART_STB_1BIT);
    usart_receive_config(SERIAL_PORT, USART_RECEIVE_ENABLE);
    usart_transmit_config(SERIAL_PORT, USART_TRANSMIT_ENABLE);
    usart_enable(SERIAL_PORT);
#endif
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
        /* Loop until the end of transmission */
        while(usart_flag_get(SERIAL_PORT, USART_FLAG_TBE) == RESET)
            ;
        usart_data_transmit(SERIAL_PORT, *buffer++);
    }
    while(usart_flag_get(SERIAL_PORT, USART_FLAG_TC) == RESET)
        ;
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
