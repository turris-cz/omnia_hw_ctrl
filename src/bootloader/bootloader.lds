ENTRY(reset_handler)

#include <memory_layout.h>

MEMORY
{
	isr_vector (r)	: ORIGIN = BOOTLOADER_BEGIN,				LENGTH = ISR_VECTOR_LENGTH
	boot_version(r)	: ORIGIN = ORIGIN(isr_vector) + LENGTH(isr_vector),	LENGTH = 0x14
	features (r)	: ORIGIN = ORIGIN(boot_version) + LENGTH(boot_version),	LENGTH = 12
#if defined(CFG_AREA_BEGIN)
	cfg_area (r)	: ORIGIN = CFG_AREA_BEGIN,				LENGTH = CFG_AREA_LENGTH
	flash (rx)	: ORIGIN = ORIGIN(cfg_area) + LENGTH(cfg_area),		LENGTH = APPLICATION_BEGIN - ORIGIN(flash)
#else
	flash (rx)	: ORIGIN = ORIGIN(features) + LENGTH(features),		LENGTH = APPLICATION_BEGIN - ORIGIN(flash)
#endif
	ram (xrw)	: ORIGIN = RAM_BEGIN,					LENGTH = RAM_LENGTH
}

#if defined(MKL81)
# define FLASH_SEGMENT_AT(_seg) >ram AT>_seg
# define ALIGNMENT 32
#else
# define FLASH_SEGMENT_AT(_seg) >_seg
# define ALIGNMENT 4
#endif

_stack_top = ORIGIN(ram) + LENGTH(ram) - RESET_REASON_MSG_LENGTH;

SECTIONS
{
	.isr_vector : {
		_sfreloc = LOADADDR(.isr_vector);
		_sreloc = ABSOLUTE(.);
		KEEP(*(.isr_vector))
	} FLASH_SEGMENT_AT(isr_vector)

	.boot_version : {
		KEEP(*(.boot_version))
	} FLASH_SEGMENT_AT(boot_version)

	.features : {
		KEEP(*(.features))
	} FLASH_SEGMENT_AT(features)

#if defined(CFG_AREA_BEGIN)
	.cfg_area ORIGIN(ram) + CFG_AREA_BEGIN : {
		KEEP(*(.bca))
		KEEP(*(.fcf))
	} FLASH_SEGMENT_AT(cfg_area)
#endif

	.text : {
		. = ALIGN(ALIGNMENT);
		_stext = ABSOLUTE(.);
		*(.text*)
		*(.rodata*)
		. = ALIGN(ALIGNMENT);
		_etext = ABSOLUTE(.);

		*(.privilegedtext*)
	} FLASH_SEGMENT_AT(flash)

	.privilegeddata : {
		. = ALIGN(ALIGNMENT);
		*(.privilegeddata*)
		. = ALIGN(ALIGNMENT);
	} >ram AT>flash

	.data : {
		. = ALIGN(ALIGNMENT);
		_sfdata = LOADADDR(.data);
		_sdata = ABSOLUTE(.);
		*(.data*)
		. = ALIGN(ALIGNMENT);
		_edata = ABSOLUTE(.);
		_ereloc = ABSOLUTE(.);
	} >ram AT>flash

	.bss : {
		. = ALIGN(ALIGNMENT);
		_sbss = ABSOLUTE(.);
		*(.bss*)
		. = ALIGN(ALIGNMENT);
		_ebss = ABSOLUTE(.);
	} >ram

	.startup : {
		KEEP(*(.startup));
	} >flash

	/DISCARD/ : {
		libgcc.a ( * )
		*(.interp*)
		*(.dynsym)
		*(.dynstr*)
		*(.dynamic*)
		*(.gnu*)
		*(.rel*)
		*(.ARM*)
	}
}
