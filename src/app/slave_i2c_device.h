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

struct st_i2c_status {
    uint8_t address_matched : 1;
    uint8_t data_received   : 1;
    uint8_t timeout         : 1;
    uint8_t data;
};

extern struct st_i2c_status i2c_state;

#endif // SLAVE_I2C_DEVICE_H

