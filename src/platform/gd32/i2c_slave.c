#include "i2c_slave.h"
#include "debug.h"
#include "cpu.h"

i2c_slave_t *i2c_slave_ptr[2];

void __irq i2c_slave_irq_handler(void)
{
	i2c_nr_t i2c_nr = i2c_nr_in_irq();
	i2c_slave_t *slave = i2c_slave_ptr[i2c_nr];
	uint32_t i2c = i2c_to_plat(i2c_nr);
	bool handled = true;
	uint16_t stat0;
	int ret;

	stat0 = I2C_STAT0(i2c);

	if (stat0 & I2C_STAT0_BERR) {
		debug("i2c bus error, resetting (STAT0 = %#06x)\n", stat0);
		i2c_slave_reset(i2c_nr);
	} else if ((stat0 & I2C_STAT0_AERR) ||
		   (slave->state != I2C_SLAVE_STOP &&
		    (stat0 & I2C_STAT0_STPDET))) {
		/*
		 * Acknowledge not received (stop during transmit)
		 * or Stop detection flag (stop during receive)
		 */

		if (stat0 & I2C_STAT0_AERR)
			/* clear AERR */
			I2C_STAT0(i2c) = ~I2C_STAT0_AERR;
		else
			/* clear STPDET by writing I2C_CTL0 and also enable ACK
			 * for next ADDSEND if it was disabled */
			I2C_CTL0(i2c) |= I2C_CTL0_ACKEN;

		slave->state = I2C_SLAVE_STOP;
		slave->cb(slave->priv, slave->addr, slave->state, &slave->val);

		/* disable RX & TX interrupts */
		I2C_CTL1(i2c) &= ~I2C_CTL1_BUFIE;

		/* disable all interrupts if pause requested */
		if (slave->paused) {
			I2C_CTL1(i2c) &= ~(I2C_CTL1_EVIE | I2C_CTL1_ERRIE);
			NVIC_DisableIRQ(i2c_ev_irqn(i2c_nr));
			NVIC_DisableIRQ(i2c_err_irqn(i2c_nr));
		}
	} else if ((slave->state == I2C_SLAVE_WRITE_REQUESTED ||
		    slave->state == I2C_SLAVE_WRITE_RECEIVED) &&
		   (stat0 & I2C_STAT0_RBNE)) {
		/* Data not empty during receiving */

		slave->val = I2C_DATA(i2c);
		slave->state = I2C_SLAVE_WRITE_RECEIVED;
		ret = slave->cb(slave->priv, slave->addr, slave->state,
				&slave->val);

		/* Disable ACK on error. Note that unlike on STM32, on GD32
		 * writing the ACKEN register does not send ACK/NACK response
		 * for the byte we have read from I2C_DATA just now. Instead
		 * it sets whether the next received byte will be ACKed or
		 * NACKed (if we disable ACKEN, we won't even receive RBNE
		 * interrupt for next byte). */
		if (ret)
			I2C_CTL0(i2c) &= ~I2C_CTL0_ACKEN;
	} else if ((slave->state == I2C_SLAVE_READ_REQUESTED ||
		    slave->state == I2C_SLAVE_READ_PROCESSED) &&
		    (stat0 & I2C_STAT0_TBE)) {
		/* Data empty during transmitting */

		if (!slave->eof) {
			ret = slave->cb(slave->priv, slave->addr, slave->state,
					&slave->val);
			if (ret)
				slave->eof = true;
		}

		/* if no more reply bytes are available, write 0xff to master */
		if (slave->eof)
			I2C_DATA(i2c) = 0xff;
		else
			I2C_DATA(i2c) = slave->val;

		slave->state = I2C_SLAVE_READ_PROCESSED;
	} else if (stat0 & I2C_STAT0_ADDSEND) {
		/* Address matched */

		uint16_t stat1;

		/* reading stat1 after stat0 clears the ADDSEND bit */
		stat1 = I2C_STAT1(i2c);

		if (stat1 & I2C_STAT1_DUMODF)
			slave->addr = FIELD_GET(I2C_SADDR1_ADDRESS2,
						I2C_SADDR1(i2c));
		else
			slave->addr = FIELD_GET(I2C_SADDR0_ADDRESS,
						I2C_SADDR0(i2c));

		if (stat1 & I2C_STAT1_TR) {
			slave->state = I2C_SLAVE_READ_REQUESTED;
			slave->eof = false;
		} else {
			slave->state = I2C_SLAVE_WRITE_REQUESTED;
			slave->cb(slave->priv, slave->addr, slave->state,
				  &slave->val);
		}

		/* enable TBE/RBNE interrupts */
		I2C_CTL1(i2c) |= I2C_CTL1_BUFIE;
	} else {
		handled = false;
	}

	if (handled) {
		slave->timeout = I2C_SLAVE_TIMEOUT_JIFFIES;
		slave->unhandled = 0;
	} else {
		slave->unhandled++;
	}

	if (slave->unhandled == I2C_SLAVE_UNHANDLED_LIMIT) {
		/* too many unhandled interrupts, reset */
		i2c_slave_reset(i2c_nr);
	}
}
