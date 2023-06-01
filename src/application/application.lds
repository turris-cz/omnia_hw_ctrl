ENTRY(reset_handler)

#include <memory_layout.h>

MEMORY
{
	isr_vector (r)	: ORIGIN = APPLICATION_BEGIN,				LENGTH = ISR_VECTOR_LENGTH
	crcsum (r)	: ORIGIN = ORIGIN(isr_vector) + LENGTH(isr_vector),	LENGTH = 8
	features (r)	: ORIGIN = ORIGIN(crcsum) + LENGTH(crcsum),		LENGTH = 12
	flash (rx)	: ORIGIN = ORIGIN(features) + LENGTH(features),		LENGTH = APPLICATION_END - ORIGIN(flash)
	ram (xrw)	: ORIGIN = RAM_BEGIN,					LENGTH = RAM_LENGTH
}

#if defined(MKL81)
# define FLASH_SEGMENT_AT(_seg) >ram AT>_seg
#else
# define FLASH_SEGMENT_AT(_seg) >_seg
#endif

_stack_top = ORIGIN(ram) + LENGTH(ram);

SECTIONS
{
	.isr_vector : {
		_sfreloc = LOADADDR(.isr_vector);
		_sreloc = ABSOLUTE(.);
		KEEP(*(.isr_vector))
	} FLASH_SEGMENT_AT(isr_vector)

	.crcsum : {
		KEEP(*(.crcsum))
	} FLASH_SEGMENT_AT(crcsum)

	.features : {
		KEEP(*(.features))
	} FLASH_SEGMENT_AT(features)

	.text : {
		. = ALIGN(4);
		*(.text*)
		*(.rodata*)
		. = ALIGN(4);
	} FLASH_SEGMENT_AT(flash)

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
