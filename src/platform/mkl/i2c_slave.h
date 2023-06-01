#ifndef I2C_H
#define I2C_H

#include "debug.h"
#include "gpio.h"
#include "time.h"
#include "cpu.h"

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
	uint8_t timeout, unhandled;
	bool eof;
	void *priv;
	int (*cb)(void *priv, uint8_t addr, i2c_slave_event_t event, uint8_t *val);
} i2c_slave_t;

extern i2c_slave_t *i2c_slave_ptr[2];

static __force_inline void i2c_clk_config(i2c_nr_t nr, bool on)
{
	switch (nr) {
#define _I2C_CLK_CFG(_n)					\
	case _n:						\
		BME_BITFIELD(SIM_SCGC4, SIM_SCGC4_I2C ## _n) =	\
			on ? SIM_SCGC4_I2C ## _n : 0;		\
		break;
	_I2C_CLK_CFG(0)
	_I2C_CLK_CFG(1)
#undef _I2C_CLK_CFG
	default:
		unreachable();
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
				   uint8_t irq_prio, bool reset_event)
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
	I2C_C1(nr) = I2C_C1_IICIE;

	slave->state = I2C_SLAVE_STOP;
	slave->unhandled = 0;
	i2c_slave_ptr[nr] = slave;

	if (reset_event)
		slave->cb(slave->priv, 0, I2C_SLAVE_RESET, &slave->val);

	I2C_C1(nr) = I2C_C1_IICEN | I2C_C1_IICIE;

	nvic_enable_irq_with_prio(i2c_irqn(nr), irq_prio);
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

	_i2c_slave_init(nr, slave, first_addr, last_addr,
			nvic_get_priority(i2c_irqn(nr)), false);
}

static __force_inline void i2c_slave_recovery_handler(i2c_nr_t nr)
{
	i2c_slave_t *slave = i2c_slave_ptr[nr];

	if (!slave)
		return;

	disable_irq();

	if (slave->state != I2C_SLAVE_STOP) {
		if (slave->timeout) {
			slave->timeout--;
		} else {
			debug("i2c timed out, resetting\n");
			i2c_slave_reset(nr);
		}
	}

	enable_irq();
}

void i2c_slave_irq_handler(void);

#endif /* I2C_H */
