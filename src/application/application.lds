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

#define STACKS_END	(ORIGIN(ram) + LENGTH(ram))

#ifdef MAIN_STACK_SIZE
# ifdef PROCESS_STACK_SIZE
_psp_bottom = STACKS_END - MAIN_STACK_SIZE - PROCESS_STACK_SIZE;
_psp_top = STACKS_END - MAIN_STACK_SIZE;
# endif
_stack_bottom = STACKS_END - MAIN_STACK_SIZE;
#endif
_stack_top = STACKS_END;

SECTIONS
{
	.isr_vector : {
		_sfreloc = LOADADDR(.isr_vector);
		_sreloc = ABSOLUTE(.);
		KEEP(*(.isr_vector))
	} RELOC_SEGMENT_AT(isr_vector)

	.crcsum : {
		KEEP(*(.crcsum))
	} RELOC_SEGMENT_AT(crcsum)

	.features : {
		KEEP(*(.features))
	} RELOC_SEGMENT_AT(features)

	.text : {
		. = ALIGN(ALIGNMENT);
		_stext = ABSOLUTE(.);
		*(.text*)
		*(.rodata*)
		*(.unprivilegedrodata*)
		. = ALIGN(ALIGNMENT);
		_etext = ABSOLUTE(.);

		*(.privilegedtext*)
	} RELOC_SEGMENT_AT(flash)

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

	.firmwareflashdata (NOLOAD) : {
		. = ALIGN(ALIGNMENT);
		_sfirmwareflashdata = ABSOLUTE(.);
		*(.firmwareflashdata*)
		. = ALIGN(ALIGNMENT);
		_efirmwareflashdata = ABSOLUTE(.);
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
