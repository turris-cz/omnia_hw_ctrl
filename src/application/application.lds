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

_stack_top = ORIGIN(ram) + LENGTH(ram);

SECTIONS
{
	.isr_vector : {
		KEEP(*(.isr_vector))
	} >isr_vector

	.crcsum : {
		KEEP(*(.crcsum))
	} >crcsum

	.features : {
		KEEP(*(.features))
	} >features

	.text : {
		KEEP(*(.startup));
		*(.text*)
		*(.rodata*)
	} >flash

	. = ALIGN(4);
	_sfdata = .;

	.data : AT (_sfdata) {
		_sdata = ABSOLUTE(.);
		*(.data*)
		. = ALIGN(4);
		_edata = ABSOLUTE(.);
	} >ram

	.bss : {
		. = ALIGN(4);
		_sbss = ABSOLUTE(.);
		*(.bss*)
		. = ALIGN(4);
		_ebss = ABSOLUTE(.);
	}

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
