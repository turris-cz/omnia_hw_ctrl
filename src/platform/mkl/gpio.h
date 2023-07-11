#ifndef GPIO_H
#define GPIO_H

#include "cpu.h"

typedef enum {
	PORT_A = 0,
	PORT_B,
	PORT_C,
	PORT_D,
	PORT_E,
} port_t;

#define NPORTS 5

#define PIN_PORT_MASK	GENMASK8(7, 5)
#define PIN_NR_MASK	GENMASK8(4, 0)

#define PIN_BIT_TYPE_T	uint32_t

#include "gpio_common.h"

typedef enum {
	pin_pushpull,
	pin_opendrain,
} pin_out_t;

typedef enum {
	pin_nopull,
	pin_pullup,
	pin_pulldown,
} pin_pull_t;

typedef enum {
	pin_spd_1,
	pin_spd_2,
	pin_spd_3,
} pin_spd_t;

static __force_inline uint32_t port_clk_bit(port_t port)
{
	switch (port) {
	case PORT_A: return SIM_SCGC5_PTA;
	case PORT_B: return SIM_SCGC5_PTB;
	case PORT_C: return SIM_SCGC5_PTC;
	case PORT_D: return SIM_SCGC5_PTD;
	case PORT_E: return SIM_SCGC5_PTE;
	default: unreachable();
	}
}

static __force_inline uint8_t pin_irqn(gpio_t pin)
{
	switch (pin_port(pin)) {
	case PORT_A: return PortA_IRQn;
	case PORT_B: return PortB_IRQn;
	case PORT_C: return PortC_IRQn;
	case PORT_D: return PortD_IRQn;
	case PORT_E: return PortE_IRQn;
	default: unreachable();
	}
}

static __force_inline bool gpio_read(gpio_t pin)
{
	if (pin == PIN_INVALID)
		return 0;

	return !!(GPIO_PDIR(pin_port(pin)) & pin_bit(pin));
}

static __force_inline uint32_t gpio_read_port(port_t port)
{
	return GPIO_PDIR(port);
}

static __force_inline bool gpio_read_output(gpio_t pin)
{
	if (pin == PIN_INVALID)
		return false;

	return !!(GPIO_PDOR(pin_port(pin)) & pin_bit(pin));
}

static __force_inline void gpio_write(gpio_t pin, bool value)
{
	if (pin == PIN_INVALID)
		return;

	if (value)
		GPIO_PSOR(pin_port(pin)) = pin_bit(pin);
	else
		GPIO_PCOR(pin_port(pin)) = pin_bit(pin);
}

static __force_inline void gpio_set_mode(gpio_t pin, pin_mode_t mode)
{
	if (pin == PIN_INVALID)
		return;

	/* first set mux to GPIO or alternative function */
	BME_BITFIELD(PORT_PCR(pin_port(pin), pin_nr(pin)), PORT_PCR_MUX_MASK) =
		pin_mode_is_alt(mode) ? PORT_PCR_MUX(pin_mode_to_alt_fn(mode))
				      : PORT_PCR_MUX_GPIO;

	if (pin_mode_is_alt(mode))
		return;

	/* if GPIO, set direction */
	BME_BITFIELD(GPIO_PDDR(pin_port(pin)), pin_bit(pin)) =
		mode == pin_mode_out ? pin_bit(pin) : 0;
}

static inline void gpio_write_multi_list(bool value, unsigned int len,
					 const gpio_t *pins)
{
	uint32_t bits[NPORTS] = {};

	for (unsigned int i = 0; i < len; ++i)
		bits[pin_port(pins[i])] |= pin_bit(pins[i]);

	for (unsigned int i = 0; i < NPORTS; ++i) {
		if (!bits[i])
			continue;

		if (value)
			GPIO_PSOR(i) = bits[i];
		else
			GPIO_PCOR(i) = bits[i];
	}
}

static inline void gpio_dir_multi_list(bool out, unsigned int len,
				       const gpio_t *pins)
{
	uint32_t bits[NPORTS] = {};

	for (unsigned int i = 0; i < len; ++i)
		bits[pin_port(pins[i])] |= pin_bit(pins[i]);

	for (unsigned int i = 0; i < NPORTS; ++i) {
		if (!bits[i])
			continue;

		if (out)
			BME_OR(GPIO_PDDR(i)) = bits[i];
		else
			BME_AND(GPIO_PDDR(i)) = ~bits[i];
	}
}

static inline void gpio_init_list(pin_mode_t mode, pin_out_t otype,
				  pin_spd_t spd, pin_pull_t pull,
				  bool init_val, unsigned int len,
				  const gpio_t *pins)
{
	uint32_t pcr;

	/* don't initialize port clocks, on MKL this is done in startup.c */

	/* prepare PCR value */
	if (mode == pin_mode_in)
		pcr = PORT_PCR_IRQC_BOTH | PORT_PCR_ISF;
	else
		pcr = PORT_PCR_IRQC_DIS | PORT_PCR_ISF;

	if (pin_mode_is_alt(mode))
		pcr |= PORT_PCR_MUX(pin_mode_to_alt_fn(mode));
	else
		pcr |= PORT_PCR_MUX_GPIO;

	if (pull == pin_pullup)
		pcr |= PORT_PCR_PE | PORT_PCR_PS;
	else if (pull == pin_pulldown)
		pcr |= PORT_PCR_PE;

	if (otype == pin_opendrain)
		pcr |= PORT_PCR_ODE;

	if (spd == pin_spd_1)
		pcr |= PORT_PCR_SRE;

	/* initialize individual pins */
	for (unsigned int i = 0; i < len; ++i)
		if (pins[i] != PIN_INVALID) {
			if (pins[i] == PIN(E, 9) || pins[i] == PIN(E, 10))
				PORT_PCR(pin_port(pins[i]), pin_nr(pins[i])) = pcr | PORT_PCR_IRQC_BOTH;
			else
				PORT_PCR(pin_port(pins[i]), pin_nr(pins[i])) = pcr;
		}

	/* set initial output values if output mode */
	if (mode == pin_mode_out)
		gpio_write_multi_list(init_val, len, pins);

	/* set direction */
	if (!pin_mode_is_alt(mode))
		gpio_dir_multi_list(mode == pin_mode_out, len, pins);
}

#endif /* GPIO_H */
