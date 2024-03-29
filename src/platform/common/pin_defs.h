#ifndef PIN_DEFS_H
#define PIN_DEFS_H

#include "gpio.h"

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
#define LAN_LED_PORT		PORT_A
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

#endif /* PIN_DEFS_H */
