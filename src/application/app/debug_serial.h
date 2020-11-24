/**
 ******************************************************************************
 * @file    debug_serial.c
 * @author  CZ.NIC, z.s.p.o.
 * @date    25-August-2015
 * @brief   Debug prints on serial port.
 ******************************************************************************
 ******************************************************************************
 **/
#ifndef DEBUG_SERIAL_H
#define DEBUG_SERIAL_H

/* ! CAUTION ! UART pins are used for CARD and SFP detection ! */
#define DBG_ENABLE      0

#if DBG_ENABLE

/*******************************************************************************
  * @function   debug_serial_config
  * @brief      Configuration of UART peripheral.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void debug_serial_config(void);

/*******************************************************************************
  * @function   debug_print
  * @brief      Send buffer over the serial port.
  * @param      buffer: pointer to data buffer.
  * @retval     None.
  *****************************************************************************/
void debug_print(const char *buffer);

#else /* !DBG_ENABLE */

static inline void debug_serial_config(void)
{
}

static inline void debug_print(const char *buffer)
{
}

#endif /* !DBG_ENABLE */

static inline void DBG(const char *buf)
{
	return debug_print(buf);
}

#endif // DEBUG_SERIAL_H

