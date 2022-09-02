#ifndef I2C_H
#define I2C_H

#include "gd32f1x0_rcu.h"
#include "gd32f1x0_i2c.h"
#include "compiler.h"
#include "debug.h"
#include "bits.h"
#include "gpio.h"
#include "time.h"
#include "cpu.h"

typedef uint8_t i2c_nr_t;

#define I2C_SLAVE_TIMEOUT_MS		35
#define I2C_SLAVE_TIMEOUT_JIFFIES	(I2C_SLAVE_TIMEOUT_MS / JIFFY_TO_MSECS)
#define I2C_SLAVE_UNHANDLED_LIMIT	100

#define I2C1_PINS_ALT_FN	0
#define I2C1_SCL_PIN		PIN(F, 6)
#define I2C1_SDA_PIN		PIN(F, 7)

#define SLAVE_I2C		1

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
	bool paused, eof;
	void *priv;
	int (*cb)(void *priv, uint8_t addr, i2c_slave_event_t event, uint8_t *val);
} i2c_slave_t;

extern i2c_slave_t *i2c_slave_ptr[2];

static __force_inline uint32_t i2c_to_plat(i2c_nr_t i2c_nr)
{
	switch (i2c_nr) {
	case 0: return I2C0;
	case 1: return I2C1;
	default: unreachable();
	}
}

static __force_inline void i2c_clk_config(i2c_nr_t i2c_nr, bool on)
{
	switch (i2c_nr) {
#define _I2C_CLK_CFG(_bus, _n)								\
	case _n:									\
		if (on)									\
			RCU_ ## _bus ## EN |= RCU_ ## _bus ## EN_I2C ## _n ## EN;	\
		else									\
			RCU_ ## _bus ## EN &= ~RCU_ ## _bus ## EN_I2C ## _n ## EN;	\
		break;
	_I2C_CLK_CFG(APB1, 0)
	_I2C_CLK_CFG(APB1, 1)
#undef _I2C_CLK_CFG
	default:
		unreachable();
	}
}

static __force_inline void i2c_reset(i2c_nr_t i2c_nr)
{
	switch (i2c_nr) {
#define _I2C_RESET(_bus, _n)								\
	case _n:									\
		RCU_ ## _bus ## RST |= RCU_ ## _bus ## RST_I2C ## _n ## RST;		\
		RCU_ ## _bus ## RST &= ~RCU_ ## _bus ## RST_I2C ## _n ## RST;		\
		break;
	_I2C_RESET(APB1, 0)
	_I2C_RESET(APB1, 1)
#undef _I2C_RESET
	default:
		unreachable();
	}
}

static __force_inline uint8_t i2c_ev_irqn(i2c_nr_t i2c_nr)
{
	switch (i2c_nr) {
	case 0: return I2C0_EV_IRQn;
	case 1: return I2C1_EV_IRQn;
	default: unreachable();
	}
}

static __force_inline uint8_t i2c_err_irqn(i2c_nr_t i2c_nr)
{
	switch (i2c_nr) {
	case 0: return I2C0_ER_IRQn;
	case 1: return I2C1_ER_IRQn;
	default: unreachable();
	}
}

static __force_inline i2c_nr_t i2c_nr_in_irq(void)
{
	/* This saves some space in the resulting binary.
	 * Disable if you want to use both I2Cs. */
	if (1)
		return SLAVE_I2C;

	switch ((__get_IPSR() & 0x3f) - 16) {
	case I2C0_EV_IRQn:
	case I2C0_ER_IRQn:
		return 0;
	case I2C1_EV_IRQn:
	case I2C1_ER_IRQn:
		return 1;
	default: unreachable();
	}
}

static __force_inline void i2c_init_pins(i2c_nr_t i2c_nr)
{
	/* If you want to implement other I2C peripheral, disable also the early
	 * return in i2c_nr_in_irq(). */
	compiletime_assert(i2c_nr == SLAVE_I2C, "Invalid I2C peripheral used");

	gpio_init_alts(I2C1_PINS_ALT_FN, pin_opendrain, pin_spd_3,
		       pin_pullup, I2C1_SCL_PIN, I2C1_SDA_PIN);
}

static inline void _i2c_slave_init(i2c_nr_t i2c_nr, i2c_slave_t *slave,
				   uint8_t addr1, uint8_t addr2,
				   uint8_t irq_prio, bool reset_event)
{
	uint32_t i2c = i2c_to_plat(i2c_nr);

	i2c_init_pins(i2c_nr);

	i2c_clk_config(i2c_nr, false);
	i2c_reset(i2c_nr);
	i2c_clk_config(i2c_nr, true);

	/* clock at 72 MHz */
	I2C_CTL1(i2c) = FIELD_PREP(I2C_CTL1_I2CCLK, 72);
	I2C_RT(i2c) = 72;
	I2C_CKCFG(i2c) = FIELD_PREP(I2C_CKCFG_CLKC, 360);

	I2C_SADDR0(i2c) = FIELD_PREP(I2C_SADDR0_ADDRESS, addr1);
	if (addr2)
		I2C_SADDR1(i2c) = FIELD_PREP(I2C_SADDR1_ADDRESS2, addr2) |
				  I2C_SADDR1_DUADEN;
	I2C_CTL1(i2c) |= I2C_CTL1_EVIE | I2C_CTL1_ERRIE;

	slave->state = I2C_SLAVE_STOP;
	slave->paused = false;
	slave->unhandled = 0;
	i2c_slave_ptr[i2c_nr] = slave;

	if (reset_event)
		slave->cb(slave->priv, 0, I2C_SLAVE_RESET, NULL);

	nvic_enable_irq(i2c_ev_irqn(i2c_nr), irq_prio);
	nvic_enable_irq(i2c_err_irqn(i2c_nr), irq_prio);
	I2C_CTL0(i2c) = I2C_CTL0_I2CEN | I2C_CTL0_ACKEN;
}

static inline void i2c_slave_init(i2c_nr_t i2c_nr, i2c_slave_t *slave,
				  uint8_t addr1, uint8_t addr2,
				  uint8_t irq_prio)
{
	_i2c_slave_init(i2c_nr, slave, addr1, addr2, irq_prio, true);
}

static __force_inline void i2c_slave_reset(i2c_nr_t i2c_nr)
{
	i2c_slave_t *slave = i2c_slave_ptr[i2c_nr];
	uint32_t i2c = i2c_to_plat(i2c_nr);

	slave->cb(slave->priv, slave->addr, I2C_SLAVE_RESET, &slave->val);
	_i2c_slave_init(i2c_nr, slave,
			FIELD_GET(I2C_SADDR0_ADDRESS, I2C_SADDR0(i2c)),
			FIELD_GET(I2C_SADDR1_ADDRESS2, I2C_SADDR1(i2c)),
			NVIC_GetPriority(i2c_ev_irqn(i2c_nr)), false);
}

/* should be called only from slave callback, disable I2C interrupts
 * after end of transaction */
static __force_inline void i2c_slave_pause(i2c_nr_t i2c_nr)
{
	i2c_slave_ptr[i2c_nr]->paused = true;
}

static __force_inline void i2c_slave_resume(i2c_nr_t i2c_nr)
{
	i2c_slave_ptr[i2c_nr]->paused = false;
	I2C_CTL1(i2c_to_plat(i2c_nr)) |= I2C_CTL1_EVIE | I2C_CTL1_ERRIE;
}

static __force_inline void i2c_slave_recovery_handler(i2c_nr_t i2c_nr)
{
	i2c_slave_t *slave = i2c_slave_ptr[i2c_nr];

	disable_irq();

	if (slave->state != I2C_SLAVE_STOP) {
		if (slave->timeout) {
			slave->timeout--;
		} else {
			debug("i2c timed out, resetting\n");
			i2c_slave_reset(i2c_nr);
		}
	}

	enable_irq();
}

void i2c_slave_irq_handler(void);

#endif /* I2C_H */
