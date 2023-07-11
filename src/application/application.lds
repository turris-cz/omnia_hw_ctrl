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
# define ALIGNMENT 32
#else
# define FLASH_SEGMENT_AT(_seg) >_seg
# define ALIGNMENT 4
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
