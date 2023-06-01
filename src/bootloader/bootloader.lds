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

_stack_top = ORIGIN(ram) + LENGTH(ram) - RESET_REASON_MSG_LENGTH;

SECTIONS
{
	.isr_vector : {
		_sfreloc = LOADADDR(.isr_vector);
		_sreloc = ABSOLUTE(.);
		KEEP(*(.isr_vector))
	} RELOC_SEGMENT_AT(isr_vector)

	.boot_version : {
		KEEP(*(.boot_version))
	} RELOC_SEGMENT_AT(boot_version)

	.features : {
		KEEP(*(.features))
	} RELOC_SEGMENT_AT(features)

#if defined(CFG_AREA_BEGIN)
	.cfg_area ORIGIN(ram) + CFG_AREA_BEGIN : {
		KEEP(*(.bca))
		KEEP(*(.fcf))
	} RELOC_SEGMENT_AT(cfg_area)
#endif

	.text : {
		. = ALIGN(4);
		*(.text*)
		*(.rodata*)
		. = ALIGN(4);
	} RELOC_SEGMENT_AT(flash)

	.data : {
		. = ALIGN(4);
		_sfdata = LOADADDR(.data);
		_sdata = ABSOLUTE(.);
		*(.data*)
		. = ALIGN(4);
		_edata = ABSOLUTE(.);
		_ereloc = ABSOLUTE(.);
	} >ram AT>flash

	.startup : {
		KEEP(*(.startup));
	} >flash

	.bss : {
		. = ALIGN(4);
		_sbss = ABSOLUTE(.);
		*(.bss*)
		. = ALIGN(4);
		_ebss = ABSOLUTE(.);
	} >ram

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
