#include "i2c_slave.h"
#include "debug.h"

i2c_slave_t *i2c_slave_ptr[2] = {};

void __irq i2c_slave_irq_handler(void)
{
	i2c_nr_t nr = i2c_nr_in_irq();
	i2c_slave_t *slave = i2c_slave_ptr[nr];
	bool handled = true;
	uint8_t flt, st;

	flt = I2C_FLT(nr);
	st = I2C_S(nr);

	if (flt & I2C_FLT_STOPF) {
		BME_OR(I2C_FLT(nr)) = I2C_FLT_STOPF;
		BME_AND(I2C_C1(nr)) = ~I2C_C1_TXAK;
		BME_OR(I2C_S(nr)) = I2C_S_IICIF;

		if (slave->addr) {
			slave->state = I2C_SLAVE_STOP;
			slave->cb(slave->priv, slave->addr, slave->state,
				  &slave->val);
		}
		slave->addr = 0;
		goto handled;
	} else if (flt & I2C_FLT_STARTF) {
		BME_OR(I2C_FLT(nr)) = I2C_FLT_STARTF;
	}

	BME_OR(I2C_S(nr)) = I2C_S_IICIF;
	if (!slave->addr && (flt & I2C_FLT_STARTF))
		goto handled;

	if (st & I2C_S_ARBL)
		BME_OR(I2C_S(nr)) = I2C_S_ARBL;

	if (st & I2C_S_IAAS) {
		/* read address */
		slave->addr = I2C_D(nr) >> 1;

		if (st & I2C_S_SRW) {
			slave->state = I2C_SLAVE_READ_REQUESTED;
			slave->eof = slave->cb(slave->priv, slave->addr,
					       slave->state, &slave->val);
			slave->state = I2C_SLAVE_READ_PROCESSED;

			BME_OR(I2C_C1(nr)) = I2C_C1_TX;

			if (slave->eof)
				I2C_D(nr) = 0xff;
			else
				I2C_D(nr) = slave->val;
		} else {
			slave->state = I2C_SLAVE_WRITE_REQUESTED;
			slave->cb(slave->priv, slave->addr, slave->state,
				  &slave->val);
			slave->state = I2C_SLAVE_WRITE_RECEIVED;

			/* dummy read */
			(void)I2C_D(nr);

			BME_AND(I2C_C1(nr)) = ~(I2C_C1_TX | I2C_C1_TXAK);
		}
	} else if (st & I2C_S_ARBL) {
		/* do not handle TCF if ARBL */
	} else if (slave->addr && (st & I2C_S_TCF) &&
		   slave->state == I2C_SLAVE_WRITE_RECEIVED) {
		slave->val = I2C_D(nr);
		slave->state = I2C_SLAVE_WRITE_RECEIVED;
		if (slave->cb(slave->priv, slave->addr, slave->state,
			      &slave->val))
			BME_OR(I2C_C1(nr)) = I2C_C1_TXAK;
	} else if (slave->addr && (st & I2C_S_TCF) &&
		   slave->state == I2C_SLAVE_READ_PROCESSED) {
		if (st & I2C_S_RXAK) {
			BME_AND(I2C_C1(nr)) = ~I2C_C1_TX;

			/* dummy read */
			(void)I2C_D(nr);
		} else {
			if (!slave->eof)
				slave->eof = slave->cb(slave->priv, slave->addr,
						       slave->state,
						       &slave->val);

			if (slave->eof)
				I2C_D(nr) = 0xff;
			else
				I2C_D(nr) = slave->val;
		}
	} else {
		handled = false;
	}

handled:
	if (handled) {
		slave->timeout = I2C_SLAVE_TIMEOUT_JIFFIES;
		slave->unhandled = 0;
	} else {
		slave->unhandled++;
	}

	if (slave->unhandled == I2C_SLAVE_UNHANDLED_LIMIT) {
		/* too many unhandled interrupts, reset */
		i2c_slave_reset(nr);
	}
}
