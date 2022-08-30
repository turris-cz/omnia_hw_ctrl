#include "i2c_slave.h"
#include "debug.h"
#include "cpu.h"

i2c_slave_t *i2c_slave_ptr[2];

static __force_inline void send_ack_and_release_scl(I2C_TypeDef *i2c, bool ack)
{
	/* SCL is released by writing value >= 1 to I2C_CR2_NBYTES.
	 * Since I2C_CR2_NBYTES is already set to 1 in i2c_slave_init(), we can
	 * simply set/unset NACK (and the code also writes 1 to NBYTES). */

	if (ack)
		i2c->CR2 &= ~I2C_CR2_NACK;
	else
		i2c->CR2 |= I2C_CR2_NACK;
}

void __irq i2c_slave_irq_handler(void)
{
	i2c_nr_t i2c_nr = i2c_nr_in_irq();
	i2c_slave_t *slave = i2c_slave_ptr[i2c_nr - 1];
	I2C_TypeDef *i2c = i2c_to_plat(i2c_nr);
	bool handled = true;
	uint32_t isr;
	int ret;

	isr = i2c->ISR;

	if (isr & (I2C_ISR_TIMEOUT | I2C_ISR_ARLO | I2C_ISR_BERR)) {
		i2c->ICR = I2C_ICR_TIMOUTCF | I2C_ICR_ARLOCF | I2C_ICR_BERRCF;
		debug("i2c bus error, resetting (ISR = %#010x)\n", isr);
		i2c_slave_reset(i2c_nr);
	}

	/* Stop detection flag */
	else if (slave->state != I2C_SLAVE_STOP && (isr & I2C_ISR_STOPF)) {
		slave->state = I2C_SLAVE_STOP;
		slave->cb(slave->priv, slave->addr, slave->state, &slave->val);

		/* disable RX & TX interrupts */
		i2c->CR1 &= ~(I2C_CR1_RXIE | I2C_CR1_TXIE);

		/* disable all interrupts if pause requested */
		if (slave->paused)
			i2c->CR1 &= ~(I2C_CR1_ADDRIE | I2C_CR1_ERRIE |
				      I2C_CR1_STOPIE);

		/* clear */
		i2c->ICR = I2C_ICR_STOPCF;
	}

	/* Receive data register not empty */
	else if ((slave->state == I2C_SLAVE_WRITE_REQUESTED ||
		  slave->state == I2C_SLAVE_WRITE_RECEIVED) &&
		 (isr & I2C_ISR_RXNE)) {
		slave->val = i2c->RXDR;
		slave->state = I2C_SLAVE_WRITE_RECEIVED;
		ret = slave->cb(slave->priv, slave->addr, slave->state, &slave->val);

		/* send NACK on error */
		send_ack_and_release_scl(i2c, !ret);
	}

	/* Transmit interrupt status */
	else if ((slave->state == I2C_SLAVE_READ_REQUESTED ||
		  slave->state == I2C_SLAVE_READ_PROCESSED) &&
		 (isr & I2C_ISR_TXIS)) {
		if (!slave->eof) {
			ret = slave->cb(slave->priv, slave->addr, slave->state,
					&slave->val);
			if (ret)
				slave->eof = true;
		}

		/* if no more reply bytes are available, write 0xff to master */
		if (slave->eof)
			i2c->TXDR = 0xff;
		else
			i2c->TXDR = slave->val;

		slave->state = I2C_SLAVE_READ_PROCESSED;
	}

	/* Address matched */
	else if (isr & I2C_ISR_ADDR) {
		slave->addr = (isr & I2C_ISR_ADDCODE) >> __bf_shf(I2C_ISR_ADDCODE);

		if (isr & I2C_ISR_DIR) {
			slave->state = I2C_SLAVE_READ_REQUESTED;
			slave->eof = false;

			/* disable slave byte control, enable TX interrupts */
			i2c->CR1 = (i2c->CR1 & ~I2C_CR1_SBC) | I2C_CR1_TXIE;

			/* flush TXDR */
			i2c->ISR |= I2C_ISR_TXE;
		} else {
			/* enable slave byte control and RX interrupts */
			i2c->CR1 |= I2C_CR1_SBC | I2C_CR1_RXIE;

			send_ack_and_release_scl(i2c, 1);

			slave->state = I2C_SLAVE_WRITE_REQUESTED;
			slave->cb(slave->priv, slave->addr, slave->state,
				  &slave->val);
		}

		/* clear */
		i2c->ICR = I2C_ICR_ADDRCF;
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
