#ifndef BOOT_I2C_H
#define BOOT_I2C_H

typedef enum {
	FLASH_CMD_NOT_RECEIVED,
	FLASH_CMD_RECEIVED,
	FLASH_WRITE_OK,
	FLASH_WRITE_ERROR
} boot_i2c_result_t;

void boot_i2c_config(void);
boot_i2c_result_t boot_i2c_result(void);

#endif /* BOOT_I2C_H */

