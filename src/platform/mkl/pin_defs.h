#ifndef PIN_DEFS_H
#define PIN_DEFS_H

#include "gpio.h"
#include "input.h"

/* Power control outputs */
#define INT_MCU_PIN		PIN(C, 15)
#define RES_RAM_PIN		PIN_INVALID
#define ENABLE_5V_PIN		PIN(E, 6)
#define ENABLE_3V3_PIN		PIN(E, 8)
#define ENABLE_1V35_PIN		PIN(D, 15)
#define ENABLE_4V5_PIN		PIN_INVALID
#define ENABLE_1V8_PIN		PIN(A, 5)
#define ENABLE_1V5_PIN		PIN_INVALID
#define ENABLE_1V2_PIN		PIN(A, 19)
#define ENABLE_VTT_PIN		PIN(A, 17)
#define USB30_PWRON_PIN		PIN(A, 29)
#define USB31_PWRON_PIN		PIN(D, 8)
#define CFG_CTRL_PIN		PIN(D, 9)
#define PRG_4V5_PIN		PIN_INVALID

#define nPERST1_PIN		PIN(A, 1, !DBG_ENABLE)
#define nRES_MMC_PIN		PIN(E, 11)
#define nRES_LAN_PIN		PIN(B, 0)
#define nRES_PHY_PIN		PIN(B, 7)
#define nVHV_CTRL_PIN		PIN(E, 7)
#define PHY_SFP_PIN		PIN(C, 19)
#define nPERST2_PIN		PIN(D, 6)
#define nPERST0_PIN		PIN(E, 0)

/* Power control inputs */
#define MANRES_PIN		PIN(E, 9)
#define SYSRES_OUT_PIN		PIN(E, 10)
#define DBGRES_PIN		PIN_INVALID
#define MRES_PIN		PIN_INVALID
#define PG_5V_PIN		PIN(B, 20)
#define PG_3V3_PIN		PIN(B, 6)
#define PG_1V35_PIN		PIN(B, 9)
#define PG_4V5_PIN		PIN_INVALID
#define PG_1V8_PIN		PIN(B, 16)
#define PG_1V5_PIN		PIN(B, 4)
#define PG_1V2_PIN		PIN(D, 1)
#define PG_VTT_PIN		PIN(D, 11)
#define USB30_OVC_PIN		PIN(D, 10)
#define USB31_OVC_PIN		PIN(D, 12)
#define RTC_ALARM_PIN		PIN_INVALID
#define FRONT_BTN_PIN		PIN(D, 13)
#define SFP_nDET_PIN		PIN(B, 1)

/* LED driver pins */
#define LED_SPI_ALT_FN		7
#define LED_SPI_MOSI_PIN	PIN(E, 3)
#define LED_SPI_SCK_PIN		PIN(E, 2)
#define LED_SPI_SS_PIN		PIN(E, 5)

#define LED_PWM_ALT_FN		4
#define LED_PWM_PIN		PIN(C, 1)

/* PCIe status LED */
#define PCI_PLED0_PIN		PIN_INVALID
#define PCI_LLED1_PIN		PIN(C, 18)
#define PCI_PLED1_PIN		PIN_INVALID
#define PCI_LLED2_PIN		PIN(C, 14)
#define PCI_PLED2_PIN		PIN_INVALID

/* WAN LED */
#define WAN_LED0_PIN		PIN(C, 3)
#define WAN_LED1_PIN		PIN(C, 4)

/* LAN LED */
#define R0_P0_LED_PIN		PIN(D, 5)
#define R1_P1_LED_PIN		PIN(D, 4)
#define R2_P2_LED_PIN		PIN(D, 7)
#define C0_P3_LED_PIN		PIN(E, 4)
#define C1_LED_PIN		PIN(A, 13)
#define C2_P4_LED_PIN		PIN(A, 15)
#define C3_P5_LED_PIN		PIN(A, 14)

/* mSATA/PCI detection and LED */
#define CARD_DET_PIN		PIN(A, 2, !DBG_ENABLE)
#define MSATA_LED_PIN		PIN(A, 18)
#define MSATA_IND_PIN		PIN(C, 0)

static inline uint32_t led_pins_read(uint32_t prev)
{
	bool c0, c1, c2, c3, nr0, nr1, nr2;
	uint32_t a = (uint32_t)&GPIO_PDIR(PORT_A),
		 d = (uint32_t)&GPIO_PDIR(PORT_D),
		 e = (uint32_t)&GPIO_PDIR(PORT_E),
		 c = (uint32_t)&GPIO_PDIR(PORT_C);
	uint32_t res = 0;

	/*
	 * On STM32/GD32, it is possible to read the row and column pins from
	 * the switch at once, since they all share the same GPIO port. This is
	 * not possible on MKL boards, because the pins are spread through
	 * different GPIO ports (A, D and E). Instead we read the GPIO port
	 * input registers with subsequent ldr instructions (with disabled
	 * interrupts). Although port C is not connected to any switch LED pin,
	 * we also read it while interrupts are disabled, since it takes only 2
	 * more ticks.
	 */
	asm volatile(
		"cpsid	i\n\t"
		"ldr	%0, [%0, #0]\n\t"
		"ldr	%1, [%1, #0]\n\t"
		"ldr	%2, [%2, #0]\n\t"
		"ldr	%3, [%3, #0]\n\t"
		"cpsie	i\n\t"
		: "+r" (a), "+r" (d), "+r" (e), "+r" (c)
	);

#define _FILL_LED(port, pin, bit)			\
	if (pin_bit(pin) && !(port & pin_bit(pin)))	\
		res |= bit;

	_FILL_LED(a, MSATA_LED_PIN, WLAN0_MSATA_LED_BIT)
	_FILL_LED(c, PCI_LLED1_PIN, WLAN1_LED_BIT);
	_FILL_LED(c, PCI_LLED2_PIN, WLAN2_LED_BIT);
	_FILL_LED(c, WAN_LED0_PIN, WAN_LED0_BIT);
	_FILL_LED(c, WAN_LED1_PIN, WAN_LED1_BIT);
#undef _FILL_LED

	c0 = e & pin_bit(C0_P3_LED_PIN);
	c1 = a & pin_bit(C1_LED_PIN);
	c2 = a & pin_bit(C2_P4_LED_PIN);
	c3 = a & pin_bit(C3_P5_LED_PIN);
	d = ~d;
	nr0 = d & pin_bit(R0_P0_LED_PIN);
	nr1 = d & pin_bit(R1_P1_LED_PIN);
	nr2 = d & pin_bit(R2_P2_LED_PIN);

	res |= prev & LAN_LEDS_BIT_MASK;

#define _FILL_LAN(col, l0, l1, l2)	\
	if (col) {			\
		res &= ~(l0 | l1 | l2);	\
		if (nr0)		\
			res |= l0;	\
		if (nr1)		\
			res |= l1;	\
		if (nr2)		\
			res |= l2;	\
	}

	_FILL_LAN(c0, LAN0_LED0_BIT, LAN2_LED0_BIT, LAN4_LED0_BIT)
	_FILL_LAN(c1, LAN1_LED0_BIT, LAN3_LED0_BIT, LAN5_LED0_BIT)
	_FILL_LAN(c2, LAN0_LED1_BIT, LAN2_LED1_BIT, LAN4_LED1_BIT)
	_FILL_LAN(c3, LAN1_LED1_BIT, LAN3_LED1_BIT, LAN5_LED1_BIT)
#undef _FILL_LAN

	return res;
}

/* read power related input pins, fill true if low */
static inline void power_input_pins_read(bool *manres, bool *sysres, bool *mres,
					 bool *pg, bool *pg_4v5,
					 bool *usb30_ovc, bool *usb31_ovc)
{
	uint32_t b, d, e;

	b = ~gpio_read_port(PORT_B);
	d = ~gpio_read_port(PORT_D);
	e = ~gpio_read_port(PORT_E);

	*manres = e & pin_bit(MANRES_PIN);
	*sysres = e & pin_bit(SYSRES_OUT_PIN);
	*mres = false;
	*pg = (b & (pin_bit(PG_5V_PIN) | pin_bit(PG_3V3_PIN) |
		    pin_bit(PG_1V35_PIN) | pin_bit(PG_1V8_PIN) |
		    pin_bit(PG_1V5_PIN))) ||
	      (d & (pin_bit(PG_1V2_PIN) | pin_bit(PG_VTT_PIN)));
	*pg_4v5 = false;
	*usb30_ovc = d & pin_bit(USB30_OVC_PIN);
	*usb31_ovc = d & pin_bit(USB31_OVC_PIN);
}

#endif /* PIN_DEFS_H */
