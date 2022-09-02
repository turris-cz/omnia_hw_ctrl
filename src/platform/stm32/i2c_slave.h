#ifndef I2C_H
#define I2C_H

#include "stm32f0xx_i2c.h"
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

#define I2C2_PINS_ALT_FN	1
#define I2C2_SCL_PIN		PIN(F, 6)
#define I2C2_SDA_PIN		PIN(F, 7)

#define SLAVE_I2C		2

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

static __force_inline I2C_TypeDef *i2c_to_plat(i2c_nr_t i2c_nr)
{
	switch (i2c_nr) {
	case 1: return I2C1;
	case 2: return I2C2;
	default: unreachable();
	}
}

static __force_inline void i2c_clk_config(i2c_nr_t i2c_nr, bool on)
{
	switch (i2c_nr) {
#define _I2C_CLK_CFG(_bus, _n)								\
	case _n:									\
		if (on)									\
			RCC->_bus ## ENR |= RCC_ ## _bus ## ENR_I2C ## _n ## EN;	\
		else									\
			RCC->_bus ## ENR &= ~RCC_ ## _bus ## ENR_I2C ## _n ## EN;	\
		break;
	_I2C_CLK_CFG(APB1, 1)
	_I2C_CLK_CFG(APB1, 2)
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
		RCC->_bus ## RSTR |= RCC_ ## _bus ## RSTR_I2C ## _n ## RST;		\
		RCC->_bus ## RSTR &= ~RCC_ ## _bus ## RSTR_I2C ## _n ## RST;		\
		break;
	_I2C_RESET(APB1, 1)
	_I2C_RESET(APB1, 2)
#undef _I2C_RESET
	default:
		unreachable();
	}
}

static __force_inline uint8_t i2c_irqn(i2c_nr_t i2c_nr)
{
	switch (i2c_nr) {
	case 1: return I2C1_IRQn;
	case 2: return I2C2_IRQn;
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
	case I2C1_IRQn: return 1;
	case I2C2_IRQn: return 2;
	default: unreachable();
	}
}

static __force_inline void i2c_init_pins(i2c_nr_t i2c_nr)
{
	/* If you want to implement other I2C peripheral, disable also the early
	 * return in i2c_nr_in_irq(). */
	compiletime_assert(i2c_nr == SLAVE_I2C, "Invalid I2C peripheral used");

	gpio_init_alts(I2C2_PINS_ALT_FN, pin_opendrain, pin_spd_1,
		       pin_nopull, I2C2_SCL_PIN, I2C2_SDA_PIN);
}

static inline void _i2c_slave_init(i2c_nr_t i2c_nr, i2c_slave_t *slave,
				   uint8_t addr1, uint8_t addr2,
				   uint8_t irq_prio, bool reset_event)
{
	I2C_TypeDef *i2c = i2c_to_plat(i2c_nr);

	i2c_clk_config(i2c_nr, false);
	i2c_reset(i2c_nr);
	i2c_clk_config(i2c_nr, true);

	i2c_init_pins(i2c_nr);

	/* 100kHz for 48MHz system clock */
	i2c->TIMINGR = 0x10800000;

	/* Analog filter enable.
	 * Mode I2C.
	 * Slave byte control enable.
	 * Enable address match, transfer complete, stop and transmit interrupt.
	 */
	i2c->CR1 = I2C_AnalogFilter_Enable | I2C_Mode_I2C | I2C_CR1_SBC |
		   I2C_CR1_ADDRIE | I2C_CR1_ERRIE | I2C_CR1_STOPIE;

	/* Reload enable, NBYTES = 1. */
	i2c->CR2 = I2C_CR2_RELOAD | FIELD_PREP(I2C_CR2_NBYTES, 1);

	i2c->OAR1 = I2C_AcknowledgedAddress_7bit | (addr1 << 1) |
		    I2C_OAR1_OA1EN;
	if (addr2)
		i2c->OAR2 = I2C_OAR2_OA2EN | (addr2 << 1) |
			    (I2C_OA2_Mask01 << 8);

	slave->state = I2C_SLAVE_STOP;
	slave->paused = false;
	slave->unhandled = 0;
	i2c_slave_ptr[i2c_nr - 1] = slave;

	if (reset_event)
		slave->cb(slave->priv, 0, I2C_SLAVE_RESET, &slave->val);

	/* peripheral enable */
	i2c->CR1 |= I2C_CR1_PE;

	nvic_enable_irq(i2c_irqn(i2c_nr), irq_prio);
}

static inline void i2c_slave_init(i2c_nr_t i2c_nr, i2c_slave_t *slave,
				  uint8_t addr1, uint8_t addr2,
				  uint8_t irq_prio)
{
	_i2c_slave_init(i2c_nr, slave, addr1, addr2, irq_prio, true);
}

#define I2C_OAR_OA_7B		0xfe

static __force_inline void i2c_slave_reset(i2c_nr_t i2c_nr)
{
	i2c_slave_t *slave = i2c_slave_ptr[i2c_nr - 1];
	I2C_TypeDef *i2c = i2c_to_plat(i2c_nr);

	slave->cb(slave->priv, slave->addr, I2C_SLAVE_RESET, &slave->val);
	_i2c_slave_init(i2c_nr, slave,
			FIELD_GET(I2C_OAR_OA_7B, i2c->OAR1),
			FIELD_GET(I2C_OAR_OA_7B, i2c->OAR2),
			NVIC_GetPriority(i2c_irqn(i2c_nr)), false);
}

/* should be called only from slave callback, disable I2C interrupts
 * after end of transaction */
static __force_inline void i2c_slave_pause(i2c_nr_t i2c_nr)
{
	i2c_slave_ptr[i2c_nr - 1]->paused = true;
}

static __force_inline void i2c_slave_resume(i2c_nr_t i2c_nr)
{
	i2c_slave_ptr[i2c_nr - 1]->paused = false;
	i2c_to_plat(i2c_nr)->CR1 |= I2C_CR1_ADDRIE | I2C_CR1_ERRIE |
				    I2C_CR1_STOPIE;
}

static __force_inline void i2c_slave_recovery_handler(i2c_nr_t i2c_nr)
{
	i2c_slave_t *slave = i2c_slave_ptr[i2c_nr - 1];

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
