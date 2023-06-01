#include "cpu.h"

typedef struct {
	uint8_t tag[4];

	uint32_t crcStartAddress;
	uint32_t crcByteCount;
	uint32_t crcExpectedValue;

	uint8_t enabledPeripherals;
	uint8_t i2cSlaveAddress;
	uint16_t peripheralDetectionTimeout;

	uint16_t usbVid;
	uint16_t usbPid;
	uint32_t usbStringsPointer;

	uint8_t clockFlags;
	uint8_t clockDivider;

	uint8_t bootFlags;

	uint8_t pad_byte;
	uint32_t reserved0;

	uint32_t keyblobDataPointer;

	uint32_t reserved1[2];

	uint32_t qspiConfigBlockPointer;

	uint32_t reserved2[3];
} bca_t;

typedef struct {
	uint32_t backdoor_key[2];
	uint32_t fprot;
	uint8_t fsec;
	uint8_t fopt;
	uint16_t reserved;
} fcf_t;

static __used __section(".bca") const bca_t bca = {
	.tag = "kcfg",

	.crcStartAddress = 0xffffffff,
	.crcByteCount = 0xffffffff,
	.crcExpectedValue = 0xffffffff,

	.enabledPeripherals = 0x03,
	.i2cSlaveAddress = 0xff,
	.peripheralDetectionTimeout = 10,

	.usbVid = 0xffff,
	.usbPid = 0xffff,
	.usbStringsPointer = 0xffffffff,

	.clockFlags = 0xff,
	.clockDivider = 0xff,

	.bootFlags = 0b00111111,

	.pad_byte = 0xff,
	.reserved0 = 0xffffffff,

	.keyblobDataPointer = 0xffffffff,

	.reserved1 = { 0xffffffff, 0xffffffff },

	.qspiConfigBlockPointer = 0xffffffff,

	.reserved2 = { 0xffffffff, 0xffffffff, 0xffffffff },
};

static __used __section(".fcf") const fcf_t fcf = {
	.backdoor_key = { 0xffffffff, 0xffffffff },
	.fprot = 0xffffffff,
	.fsec = FSEC_SEC_UNSECURE | FSEC_FSLACC_GRANT | FSEC_MEEN_EN |
		FSEC_KEYEN_EN,
	.fopt = FOPT_LPBOOT | FOPT_BOOTPIN_OPT | FOPT_NMI_DIS | FOPT_FAST_INIT |
		FOPT_BOOTSRC_SEL_BOOTLOADER,
	.reserved = 0xffff,
};
