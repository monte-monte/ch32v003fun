ENTRY( InterruptVector )

/* Run from RAM example, small RAM footprint so should run on any CH32V's */

MEMORY
{
#if TARGET_MCU_LD == 0 /* v003 */
	RAM (xrw)  : ORIGIN = 0x20000000, LENGTH = 2K
#elif TARGET_MCU_LD == 1 /* v10x */
	#if MCU_PACKAGE == 1
		RAM (xrw) : ORIGIN = 0x20000000, LENGTH = 20K
	#elif MCU_PACKAGE == 2
		RAM (xrw) : ORIGIN = 0x20000000, LENGTH = 10K
	#else
		#error "Unknown MCU package"
	#endif
#elif TARGET_MCU_LD == 2 /* v20x */
	#if MCU_PACKAGE == 1
		RAM (xrw) : ORIGIN = 0x20000000, LENGTH = 20K
	#elif MCU_PACKAGE == 2
		RAM (xrw) : ORIGIN = 0x20000000, LENGTH = 10K
	#elif MCU_PACKAGE == 3
		RAM (xrw) : ORIGIN = 0x20000000, LENGTH = 64K
	#else
		#error "Unknown MCU package"
	#endif
#elif TARGET_MCU_LD == 3 /* v30x */
	#if MCU_PACKAGE == 1
		#if TARGET_MCU_MEMORY_SPLIT == 1
			RAM (xrw) : ORIGIN = 0x20000000, LENGTH = 96K
		#elif TARGET_MCU_MEMORY_SPLIT == 2
			RAM (xrw) : ORIGIN = 0x20000000, LENGTH = 64K
		#elif TARGET_MCU_MEMORY_SPLIT == 3
			RAM (xrw) : ORIGIN = 0x20000000, LENGTH = 32K
		#else
			RAM (xrw) : ORIGIN = 0x20000000, LENGTH = 128K
		#endif
	#elif MCU_PACKAGE == 2
		RAM (xrw) : ORIGIN = 0x20000000, LENGTH = 32K
	#else
		#error "Unknown MCU package"
	#endif
#elif TARGET_MCU_LD == 4 /* x03x */
	#if MCU_PACKAGE == 1
		RAM (xrw) : ORIGIN = 0x20000000, LENGTH = 20K
	#else
		#error "Unknown MCU package"
	#endif
#elif TARGET_MCU_LD == 5
	/* CH32V002 */
	RAM (xrw) : ORIGIN = 0x20000000, LENGTH = 4K
#elif TARGET_MCU_LD == 6
	/* CH32V005, CH32V004 */
	RAM (xrw) : ORIGIN = 0x20000000, LENGTH = 6K
#elif TARGET_MCU_LD == 7
	/* CH32V006, CH32V007 */
	RAM (xrw) : ORIGIN = 0x20000000, LENGTH = 8K
#elif TARGET_MCU_LD == 8
	/* CH582/3/4/5 */
	#if MCU_PACKAGE == 2 || MCU_PACKAGE == 3
		RAM (xrw) : ORIGIN = 0x20000000, LENGTH = 32K
	#elif MCU_PACKAGE == 4
		RAM (xrw) : ORIGIN = 0x20000000, LENGTH = 96K
	#elif MCU_PACKAGE == 5
		RAM (xrw) : ORIGIN = 0x20000000, LENGTH = 128K
	#else
		#error "Unknown MCU package"
	#endif
#elif TARGET_MCU_LD == 9
	/* CH591/2 */
	#if MCU_PACKAGE == 1
		RAM (xrw) : ORIGIN = 0x20000000, LENGTH = 26K
	#elif MCU_PACKAGE == 2
		RAM (xrw) : ORIGIN = 0x20000000, LENGTH = 26K
	#else
		#error "Unknown MCU package"
	#endif
#elif TARGET_MCU_LD == 10
	/* CH57x */
	#if MCU_PACKAGE == 0 || MCU_PACKAGE == 2
		RAM (xrw) : ORIGIN = 0x20000000, LENGTH = 12K
	#elif MCU_PACKAGE == 1
		RAM (xrw) : ORIGIN = 0x20003800, LENGTH = 18K
	#elif MCU_PACKAGE == 3
		RAM (xrw) : ORIGIN = 0x20003800, LENGTH = 18K
	#else
		#error "Unknown MCU package"
	#endif
#elif TARGET_MCU_LD == 11
	/* CH32H41x */
	#if MCU_PACKAGE == 1 || MCU_PACKAGE == 3
		RAM (xrw) : ORIGIN = 0x20100000, LENGTH = 512K
	#elif MCU_PACKAGE == 2
		RAM (xrw) : ORIGIN = 0x20100000, LENGTH = 512K
	#else
		#error "Unknown MCU package"
	#endif
#else
	#error "Unknown MCU target"
#endif
}

SECTIONS
{
	.init :
	{ 
		_sinit = .;
		. = ALIGN(4);
		KEEP(*(SORT_NONE(.init)))
		. = ALIGN(4);
		_einit = .;
	} >RAM AT>RAM

#if TARGET_MCU_LD == 8 || TARGET_MCU_LD == 9 || TARGET_MCU_LD == 10
		.highcodelalign : 
		{       
			. = ALIGN(4);
			PROVIDE(_highcode_lma = .); 
		} >RAM AT>RAM 
		
		.highcode :
		{
			. = ALIGN(4);
			PROVIDE(_highcode_vma_start = .);
			*(.highcode*)
			. = ALIGN(4);
			PROVIDE(_highcode_vma_end = .);
		} >RAM AT>RAM

    .data :
    {
		. = ALIGN(4);
		__global_pointer$ = . + 0x7f8;
		*(.gnu.linkonce.r.*)
		*(.data .data.*)
		*(.gnu.linkonce.d.*)
		. = ALIGN(4);
		*(.sdata .sdata.*)
		*(.sdata2*)
		*(.gnu.linkonce.s.*)
		. = ALIGN(4);
		*(.srodata.cst16)
		*(.srodata.cst8)
		*(.srodata.cst4)
		*(.srodata.cst2)
		*(.srodata .srodata.*)
		. = ALIGN(4);
		PROVIDE( _edata = .);
	} >RAM AT>RAM
#endif

	.text :
	{
		. = ALIGN(4);
		*(.text)
		*(.text.*)
		*(.rodata)
		*(.rodata*)
		*(.gnu.linkonce.t.*)
		. = ALIGN(4);
	} >RAM AT>RAM 

	.fini :
	{
		KEEP(*(SORT_NONE(.fini)))
		. = ALIGN(4);
	} >RAM AT>RAM

	PROVIDE( _etext = . );
	PROVIDE( _eitcm = . );	

	.preinit_array :
	{
		PROVIDE_HIDDEN (__preinit_array_start = .);
		KEEP (*(.preinit_array))
		PROVIDE_HIDDEN (__preinit_array_end = .);
	} >RAM AT>RAM 
	
	.init_array :
	{
		PROVIDE_HIDDEN (__init_array_start = .);
		KEEP (*(SORT_BY_INIT_PRIORITY(.init_array.*) SORT_BY_INIT_PRIORITY(.ctors.*)))
		KEEP (*(.init_array EXCLUDE_FILE (*crtbegin.o *crtbegin?.o *crtend.o *crtend?.o ) .ctors))
		PROVIDE_HIDDEN (__init_array_end = .);
	} >RAM AT>RAM 
	
	.fini_array :
	{
		PROVIDE_HIDDEN (__fini_array_start = .);
		KEEP (*(SORT_BY_INIT_PRIORITY(.fini_array.*) SORT_BY_INIT_PRIORITY(.dtors.*)))
		KEEP (*(.fini_array EXCLUDE_FILE (*crtbegin.o *crtbegin?.o *crtend.o *crtend?.o ) .dtors))
		PROVIDE_HIDDEN (__fini_array_end = .);
	} >RAM AT>RAM 
	
	.ctors :
	{
		/* gcc uses crtbegin.o to find the start of
		 the constructors, so we make sure it is
		 first.	Because this is a wildcard, it
		 doesn't matter if the user does not
		 actually link against crtbegin.o; the
		 linker won't look for a file to match a
		 wildcard.	The wildcard also means that it
		 doesn't matter which directory crtbegin.o
		 is in.	*/
		KEEP (*crtbegin.o(.ctors))
		KEEP (*crtbegin?.o(.ctors))
		/* We don't want to include the .ctor section from
		 the crtend.o file until after the sorted ctors.
		 The .ctor section from the crtend file contains the
		 end of ctors marker and it must be last */
		KEEP (*(EXCLUDE_FILE (*crtend.o *crtend?.o ) .ctors))
		KEEP (*(SORT(.ctors.*)))
		KEEP (*(.ctors))
	} >RAM AT>RAM 
	
	.dtors :
	{
		KEEP (*crtbegin.o(.dtors))
		KEEP (*crtbegin?.o(.dtors))
		KEEP (*(EXCLUDE_FILE (*crtend.o *crtend?.o ) .dtors))
		KEEP (*(SORT(.dtors.*)))
		KEEP (*(.dtors))
	} >RAM AT>RAM 

	.dalign :
	{
		. = ALIGN(4);
		PROVIDE(_data_vma = .);
	} >RAM AT>RAM	

	.dlalign :
	{
		. = ALIGN(4); 
		PROVIDE(_data_lma = .);
	} >RAM AT>RAM
#if TARGET_MCU_LD != 8 && TARGET_MCU_LD != 9 && TARGET_MCU_LD != 10
	.data :
	{
		. = ALIGN(4);
		__global_pointer$ = . + 0x7f8;
		*(.gnu.linkonce.r.*)
		*(.data .data.*)
		*(.gnu.linkonce.d.*)
		. = ALIGN(4);
		*(.sdata .sdata.*)
		*(.sdata2*)
		*(.gnu.linkonce.s.*)
		. = ALIGN(4);
		*(.srodata.cst16)
		*(.srodata.cst8)
		*(.srodata.cst4)
		*(.srodata.cst2)
		*(.srodata .srodata.*)
		. = ALIGN(4);
		PROVIDE( _edata = .);
	} >RAM AT>RAM
#endif
	.bss :
	{
		. = ALIGN(4);
		PROVIDE( _sbss = .);
		*(.sbss*)
		*(.gnu.linkonce.sb.*)
		*(.bss*)
		*(.gnu.linkonce.b.*)	
		*(COMMON*)
		. = ALIGN(4);
		PROVIDE( _ebss = .);
	} >RAM AT>RAM

	PROVIDE( _end = _ebss);
	PROVIDE( end = . );

	PROVIDE( _eusrstack = ORIGIN(RAM) + LENGTH(RAM));	
}




