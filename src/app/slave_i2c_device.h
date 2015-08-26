/**
 ******************************************************************************
 * @file    slave_i2c_device.h
 * @author  CZ.NIC, z.s.p.o.
 * @date    18-August-2015
 * @brief   Header for I2C driver.
 ******************************************************************************
 ******************************************************************************
 **/
#ifndef SLAVE_I2C_DEVICE_H
#define SLAVE_I2C_DEVICE_H

/*******************************************************************************
  * @function   slave_i2c_config
  * @brief      Configuration of I2C peripheral as a slave.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void slave_i2c_config(void);

/*******************************************************************************
  * @function   slave_i2c_handler
  * @brief      Interrupt handler for I2C communication.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void slave_i2c_handler(void);

/*******************************************************************************
  * @function   slave_i2c_process_data
  * @brief      Process incoming/outcoming data.
  * @param      None.
  * @retval     None.
  *****************************************************************************/
void slave_i2c_process_data(void);

#endif // SLAVE_I2C_DEVICE_H

