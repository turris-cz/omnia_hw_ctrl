#ifndef MEMORY_LAYOUT_H
#define MEMORY_LAYOUT_H

#ifdef RAM_BUILD

/* build for RAM, useful for development, since we don't need to update flash */
# define BOOTLOADER_BEGIN	0x20000000
# define APPLICATION_END	0x2000a000
# define RAM_BEGIN		0x2000a000
# define RAM_LENGTH		0x6000

#else

/*
 * build for flash, we also need to define flash configuration area for
 * bootloader linker script
 */
# define CFG_AREA_BEGIN		0x3c0
# define CFG_AREA_LENGTH	0x50

# define BOOTLOADER_BEGIN	0x0
# define APPLICATION_END	0x10000
# define RAM_BEGIN		0x1fffa000
# define RAM_LENGTH		0x18000

#endif

#define BOOTLOADER_VERSION_POS	(BOOTLOADER_BEGIN + ISR_VECTOR_LENGTH)
#define BOOTLOADER_FEATURES	(BOOTLOADER_VERSION_POS + 20)
#define BOOTLOADER_MAX_SIZE	APPLICATION_OFFSET

#define APPLICATION_BEGIN	(BOOTLOADER_BEGIN + APPLICATION_OFFSET)
#define APPLICATION_CRCSUM	(APPLICATION_BEGIN + ISR_VECTOR_LENGTH)
#define APPLICATION_FEATURES	(APPLICATION_CRCSUM + 8)
#define APPLICATION_MAX_SIZE	(APPLICATION_END - APPLICATION_BEGIN)

/* On MKL, the reset reason message is in RFSYS, not in RAM */
#define RESET_REASON_MSG_LENGTH	0

#define RAM_END			(RAM_BEGIN + RAM_LENGTH)

#endif /* MEMORY_LAYOUT_H */
