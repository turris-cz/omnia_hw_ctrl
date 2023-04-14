#ifndef PIN_DEFS_H
#define PIN_DEFS_H

#include "gpio.h"
#include "input.h"

/* Power control outputs */
#define INT_MCU_PIN		PIN(C, 0)
#define RES_RAM_PIN		PIN(C, 3, OMNIA_BOARD_REVISION < 32)
#define ENABLE_5V_PIN		PIN(C, 4)
#define ENABLE_3V3_PIN		PIN(C, 5)
#define ENABLE_1V35_PIN		PIN(C, 6)
#define ENABLE_4V5_PIN		PIN(C, 7, USER_REGULATOR_ENABLED)
#define ENABLE_1V8_PIN		PIN(C, 8)
#define ENABLE_1V5_PIN		PIN(C, 9, OMNIA_BOARD_REVISION < 32)
#define ENABLE_1V2_PIN		PIN(C, 10)
#define ENABLE_VTT_PIN		PIN(C, 11)
#define USB30_PWRON_PIN		PIN(C, 12)
#define USB31_PWRON_PIN		PIN(C, 13)
#define CFG_CTRL_PIN		PIN(C, 15)
#define PRG_4V5_PIN		PIN(F, 1, USER_REGULATOR_ENABLED)
/* v32 specific outputs */
#define nPERST1_PIN		PIN(A, 10, OMNIA_BOARD_REVISION >= 32 && !DBG_ENABLE)
#define nRES_MMC_PIN		PIN(B, 2, OMNIA_BOARD_REVISION >= 32)
#define nRES_LAN_PIN		PIN(B, 3, OMNIA_BOARD_REVISION >= 32)
#define nRES_PHY_PIN		PIN(B, 7, OMNIA_BOARD_REVISION >= 32)
#define nVHV_CTRL_PIN		PIN(B, 14, OMNIA_BOARD_REVISION >= 32)
#define PHY_SFP_PIN		PIN(C, 3, OMNIA_BOARD_REVISION >= 32)
#define nPERST2_PIN		PIN(F, 4, OMNIA_BOARD_REVISION >= 32)
#define nPERST0_PIN		PIN(F, 5, OMNIA_BOARD_REVISION >= 32)

/* Power control inputs */
#define MANRES_PIN		PIN(B, 0)
#define SYSRES_OUT_PIN		PIN(B, 1)
#define DBGRES_PIN		PIN(B, 2, OMNIA_BOARD_REVISION < 32)
#define MRES_PIN		PIN(B, 3, OMNIA_BOARD_REVISION < 32)
#define PG_5V_PIN		PIN(B, 4)
#define PG_3V3_PIN		PIN(B, 5)
#define PG_1V35_PIN		PIN(B, 6)
#define PG_4V5_PIN		PIN(B, 7)
#define PG_1V8_PIN		PIN(B, 8)
#define PG_1V5_PIN		PIN(B, 9)
#define PG_1V2_PIN		PIN(B, 10)
#define PG_VTT_PIN		PIN(B, 11)
#define USB30_OVC_PIN		PIN(B, 12)
#define USB31_OVC_PIN		PIN(B, 13)
#define RTC_ALARM_PIN		PIN(B, 14, OMNIA_BOARD_REVISION < 32)
#define FRONT_BTN_PIN		PIN(B, 15)
/* v32 specific inputs */
#define SFP_nDET_PIN		PIN(D, 2, OMNIA_BOARD_REVISION >= 32)

/* LED driver pins */
#define LED_SPI_ALT_FN		0
#define LED_SPI_MOSI_PIN	PIN(A, 7)
#define LED_SPI_SCK_PIN		PIN(A, 5)
#define LED_SPI_SS_PIN		PIN(A, 4)

#define LED_PWM_ALT_FN		0
#define LED_PWM_PIN		PIN(A, 3)

/* PCIe status LED */
#define PCI_PLED0_PIN		PIN(F, 5, OMNIA_BOARD_REVISION < 32)
#define PCI_LLED1_PIN		PIN(C, 2)
#define PCI_PLED1_PIN		PIN(A, 10, OMNIA_BOARD_REVISION < 32 && !DBG_ENABLE)
#define PCI_LLED2_PIN		PIN(C, 1)
#define PCI_PLED2_PIN		PIN(F, 4, OMNIA_BOARD_REVISION < 32)

/* WAN LED */
#define WAN_LED0_PIN		PIN(F, 0)
#define WAN_LED1_PIN		PIN(F, 1, OMNIA_BOARD_REVISION >= 32)

/* LAN LED */
#define R0_P0_LED_PIN		PIN(A, 0)
#define R1_P1_LED_PIN		PIN(A, 1)
#define R2_P2_LED_PIN		PIN(A, 2)
#define C0_P3_LED_PIN		PIN(A, 6)
#define C1_LED_PIN		PIN(A, 8)
#define C2_P4_LED_PIN		PIN(A, 11)
#define C3_P5_LED_PIN		PIN(A, 12)

/* mSATA/PCI detection and LED */
#define CARD_DET_PIN		PIN(A, 9, !DBG_ENABLE)
#define MSATA_LED_PIN		PIN(A, 15)
#define MSATA_IND_PIN		PIN(C, 14)

static inline uint32_t led_pins_read(uint32_t prev)
{
	bool c0, c1, c2, c3, nr0, nr1, nr2;
	uint16_t a, c, f;
	uint32_t res = 0;

	a = gpio_read_port(PORT_A);
	c = gpio_read_port(PORT_C);
	f = gpio_read_port(PORT_F);

#define _FILL_LED(port, pin, bit)			\
	if (pin_bit(pin) && !(port & pin_bit(pin)))	\
		res |= bit;

	_FILL_LED(a, MSATA_LED_PIN, WLAN0_MSATA_LED_BIT)
	_FILL_LED(c, PCI_LLED1_PIN, WLAN1_LED_BIT);
	_FILL_LED(c, PCI_LLED2_PIN, WLAN2_LED_BIT);
	_FILL_LED(f, PCI_PLED0_PIN, WPAN0_LED_BIT);
	_FILL_LED(a, PCI_PLED1_PIN, WPAN1_LED_BIT);
	_FILL_LED(f, PCI_PLED2_PIN, WPAN2_LED_BIT);
	_FILL_LED(f, WAN_LED0_PIN, WAN_LED0_BIT);
	_FILL_LED(f, WAN_LED1_PIN, WAN_LED1_BIT);
#undef _FILL_LED

	c0 = a & pin_bit(C0_P3_LED_PIN);
	c1 = a & pin_bit(C1_LED_PIN);
	c2 = a & pin_bit(C2_P4_LED_PIN);
	c3 = a & pin_bit(C3_P5_LED_PIN);
	a = ~a;
	nr0 = a & pin_bit(R0_P0_LED_PIN);
	nr1 = a & pin_bit(R1_P1_LED_PIN);
	nr2 = a & pin_bit(R2_P2_LED_PIN);

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

static inline void lan_led_pins_read(bool *c0, bool *c1, bool *nr0, bool *nr1,
				     bool *nr2)
{
	uint16_t port = gpio_read_port(PORT_A);

	*c0 = port & pin_bit(C0_P3_LED_PIN);
	*c1 = port & pin_bit(C1_LED_PIN);

	port = ~port;

	*nr0 = port & pin_bit(R0_P0_LED_PIN);
	*nr1 = port & pin_bit(R1_P1_LED_PIN);
	*nr2 = port & pin_bit(R2_P2_LED_PIN);
}

/* read power related input pins, fill true if low */
static inline void power_input_pins_read(bool *manres, bool *sysres, bool *mres,
					 bool *pg, bool *pg_4v5,
					 bool *usb30_ovc, bool *usb31_ovc)
{
	uint16_t low = ~gpio_read_port(PORT_B);

	*manres = low & pin_bit(MANRES_PIN);
	*sysres = low & pin_bit(SYSRES_OUT_PIN);
	*mres = low & pin_bit(MRES_PIN);
	*pg = low & (pin_bit(PG_5V_PIN) | pin_bit(PG_3V3_PIN) |
		     pin_bit(PG_1V35_PIN) | pin_bit(PG_1V8_PIN) |
		     pin_bit(PG_1V5_PIN) | pin_bit(PG_1V2_PIN) |
		     pin_bit(PG_VTT_PIN));
	*pg_4v5 = low & pin_bit(PG_4V5_PIN);
	*usb30_ovc = low & pin_bit(USB30_OVC_PIN);
	*usb31_ovc = low & pin_bit(USB31_OVC_PIN);
}

#endif /* PIN_DEFS_H */
