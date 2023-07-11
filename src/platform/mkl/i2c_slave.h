#ifndef I2C_H
#define I2C_H

#include "debug.h"
#include "gpio.h"
#include "time.h"
#include "cpu.h"
#include "clock.h"

typedef uint8_t i2c_nr_t;

#define I2C_SLAVE_TIMEOUT_MS		35
#define I2C_SLAVE_TIMEOUT_JIFFIES	(I2C_SLAVE_TIMEOUT_MS / JIFFY_TO_MSECS)
#define I2C_SLAVE_UNHANDLED_LIMIT	100

#define I2C0_PINS_ALT_FN	2
#define I2C0_SCL_PIN		PIN(B, 2)
#define I2C0_SDA_PIN		PIN(B, 3)

#define SLAVE_I2C		0

typedef enum {
	I2C_SLAVE_STOP = 0,
	I2C_SLAVE_READ_REQUESTED,
	I2C_SLAVE_WRITE_REQUESTED,
	I2C_SLAVE_READ_PROCESSED,
	I2C_SLAVE_WRITE_RECEIVED,
	I2C_SLAVE_RESET,
} i2c_slave_event_t;

typedef struct {
	i2c_slave_event_t state;
	uint8_t addr;
	uint8_t val;
	uint8_t unhandled;
	bool eof;
	uint32_t reset_at_jiffies;
	void *priv;
	int (*cb)(void *priv, uint8_t addr, i2c_slave_event_t event, uint8_t *val);
} i2c_slave_t;

extern i2c_slave_t *i2c_slave_ptr[2];

static __force_inline void i2c_clk_config(i2c_nr_t nr, bool on)
{
	switch (nr) {
	case 0: sys_clk_config(I2C0_Slot, on); break;
	case 1: sys_clk_config(I2C1_Slot, on); break;
	default: unreachable();
	}
}

static __force_inline void i2c_reset(i2c_nr_t nr)
{
	i2c_slave_ptr[nr] = NULL;

	i2c_clk_config(nr, false);
	i2c_clk_config(nr, true);

	BME_BITFIELD(I2C_C1(nr), I2C_C1_IICEN) = 0;
	I2C_A1(nr) = 0;
	I2C_F(nr) = 0;
	I2C_C1(nr) = 0;
	I2C_S(nr) = I2C_S_TCF | I2C_S_ARBL | I2C_S_IICIF;
	(void)I2C_D(nr);
	I2C_C2(nr) = 0;
	I2C_FLT(nr) = I2C_FLT_STOPF | I2C_FLT_STARTF;
	I2C_RA(nr) = 0;
	I2C_SMB(nr) = I2C_SMB_SLTF | I2C_SMB_SHTF2;
	I2C_A2(nr) = 0xc2;
	I2C_SLTH(nr) = 0;
	I2C_SLTL(nr) = 0;
	I2C_S2(nr) = I2C_S2_ERROR;
}

static __force_inline uint8_t i2c_irqn(i2c_nr_t nr)
{
	switch (nr) {
	case 0: return I2C0_IRQn;

	/* I2C1 has IRQ accessible through INTMUX, which needs to be congiured
	 * for usage. */

	default: unreachable();
	}
}

static __force_inline i2c_nr_t i2c_nr_in_irq(void)
{
	/* This saves some space in the resulting binary.
	 * Disable if you want to use both I2Cs. */
	if (1)
		return SLAVE_I2C;

	switch ((get_ipsr() & 0x3f) - 16) {
	case I2C0_IRQn: return 0;

	/* I2C1 has IRQ accessible through INTMUX, which needs to be congiured
	 * for usage. */

	default: unreachable();
	}
}

static __force_inline void i2c_init_pins(i2c_nr_t nr)
{
	/* If you want to implement other I2C peripheral, disable also the early
	 * return in i2c_nr_in_irq(). */
	compiletime_assert(nr == SLAVE_I2C, "Invalid I2C peripheral used");

	gpio_init_alts(I2C0_PINS_ALT_FN, pin_opendrain, pin_spd_1,
		       pin_nopull, I2C0_SCL_PIN, I2C0_SDA_PIN);
}

static inline void _i2c_slave_init(i2c_nr_t nr, i2c_slave_t *slave,
				   uint8_t first_addr, uint8_t last_addr,
				   uint8_t, bool reset_event)
{
	i2c_reset(nr);

	i2c_init_pins(nr);

	if (last_addr) {
		I2C_C2(nr) = I2C_C2_RMEN | I2C_C2_HDRS;
		I2C_A1(nr) = I2C_A1_AD(first_addr - 1);
		I2C_RA(nr) = I2C_RA_RAD(last_addr);
	} else {
		I2C_C2(nr) = I2C_C2_HDRS;
		I2C_A1(nr) = I2C_A1_AD(first_addr);
	}

	I2C_FLT(nr) = I2C_FLT_SSIE;
	I2C_C1(nr) = 0;

	slave->state = I2C_SLAVE_STOP;
	slave->unhandled = 0;
	i2c_slave_ptr[nr] = slave;

	if (reset_event)
		slave->cb(slave->priv, 0, I2C_SLAVE_RESET, &slave->val);

	I2C_C1(nr) = I2C_C1_IICEN;
}

static inline void i2c_slave_init(i2c_nr_t nr, i2c_slave_t *slave,
				  uint8_t first_addr, uint8_t last_addr,
				  uint8_t irq_prio)
{
	compiletime_assert(first_addr && (!last_addr || first_addr < last_addr),
			   "Invalid addresses");

	_i2c_slave_init(nr, slave, first_addr, last_addr, irq_prio, true);
}

static __force_inline void i2c_slave_reset(i2c_nr_t nr)
{
	i2c_slave_t *slave = i2c_slave_ptr[nr];
	uint8_t first_addr, last_addr;

	slave->cb(slave->priv, slave->addr, I2C_SLAVE_RESET, &slave->val);

	first_addr = FIELD_GET(I2C_A1_AD_MASK, I2C_A1(nr));

	if (I2C_C2(nr) & I2C_C2_RMEN) {
		++first_addr;
		last_addr = FIELD_GET(I2C_RA_RAD_MASK, I2C_RA(nr));
	} else {
		last_addr = 0;
	}

	_i2c_slave_init(nr, slave, first_addr, last_addr, 0, false);
}

static __force_inline void i2c_slave_recovery_handler(i2c_nr_t)
{
	/*
	 * No recovery from systick_handler, recovery is done by polling in
	 * i2c_slave_recovery_poll().
	 */
}

static __force_inline void i2c_slave_recovery_poll(i2c_nr_t nr)
{
	i2c_slave_t *slave = i2c_slave_ptr[nr];

	if (slave && slave->state != I2C_SLAVE_STOP &&
	    jiffies > slave->reset_at_jiffies) {
		debug("i2c timed out, resetting\n");
		i2c_slave_reset(nr);
	}
}

static __force_inline void i2c_slave_poll(i2c_nr_t nr)
{
	i2c_slave_t *slave;
	uint8_t flt, st;
	bool handled;

	st = I2C_S(nr);

	if (!(st & I2C_S_IICIF)) {
		i2c_slave_recovery_poll(nr);
		return;
	}

	handled = true;
	slave = i2c_slave_ptr[nr];
	flt = I2C_FLT(nr);

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
		slave->reset_at_jiffies = jiffies + I2C_SLAVE_TIMEOUT_JIFFIES;
		slave->unhandled = 0;
	} else {
		slave->unhandled++;
	}

	if (slave->unhandled == I2C_SLAVE_UNHANDLED_LIMIT) {
		/* too many unhandled interrupts, reset */
		i2c_slave_reset(nr);
	}
}

#endif /* I2C_H */
