ENTRY( _start )

__stack_size = 8192;

PROVIDE( _stack_size = __stack_size );


MEMORY
{
	FLASH (rx)          : ORIGIN = 0x00000000, LENGTH = 56K      /* Bootloader 0x00000-0x0DFFF; 0x0E000-0x0FFFF reserved */
	RAM_CODE (xrw)      : ORIGIN = 0x20100000, LENGTH = 64K      /* Same as Link_CH32H417_v3f.ld */
	RAM_LOAD (xrw)      : ORIGIN = 0x20110000, LENGTH = 256      /* Same as Link_CH32H417_v3f.ld */
	RAM (xrw)           : ORIGIN = 0x20121000, LENGTH = 0x2F000  /* Same as Link_CH32H417_v3f.ld */
	/* ---- Shared SRAM (must match all 3 linker scripts) ---- */
	USBHS_DMA (rw)      : ORIGIN = 0x20150000, LENGTH = 0x2000
	OBJECT_BUF (rw)     : ORIGIN = 0x20152000, LENGTH = 0x10000
	IPC_HEADER (rw)     : ORIGIN = 0x20162000, LENGTH = 0x1000
	IPC_CMD (rw)        : ORIGIN = 0x20163000, LENGTH = 0x2100
	IPC_EVT_SAFE (rw)   : ORIGIN = 0x20165100, LENGTH = 0x4100
	IPC_EVT_LOSSY (rw)  : ORIGIN = 0x20169200, LENGTH = 0x8100
	SHM_SPARE (rw)      : ORIGIN = 0x20171300, LENGTH = 0xECF8
	BOOT_MAGIC (rw)     : ORIGIN = 0x2017FFF8, LENGTH = 8
}

/*
 * CH32H417 latency tester - Bootloader linker script
 * - FLASH starts at 0x00000 (56KB); no .app_meta (BL reads app metadata from 0x10400).
 * - Shared SRAM is reserved in MEMORY, but BL only uses .boot_magic.
 */

SECTIONS
{
	.init :
	{
		_sinit = .;
		. = ALIGN(4);
		KEEP(*(SORT_NONE(.init)))
		. = ALIGN(4);
		_einit = .;
	} >FLASH AT>FLASH

	.loadcodelalign :
	{
		. = ALIGN(4);
		PROVIDE(_loadcode_lma = .);
	} >FLASH AT>FLASH

	.loadcode :
	{
		. = ALIGN(4);
		PROVIDE(_loadcode_vma_start = .);
		*(.load);
		. = ALIGN(4);
		PROVIDE(_loadcode_vma_end = .);
	} >RAM_LOAD AT>FLASH

	.highcodelalign :
	{
		. = ALIGN(4);
		PROVIDE(_highcode_lma = .);
	} >FLASH AT>FLASH

	.highcode :
	{
		. = ALIGN(4);
		PROVIDE(_highcode_vma_start = .);
		*(.vector);
		KEEP(*(SORT_NONE(.vector_handler)))
		*(.text)
		*(.text.*)
		*(.rodata)
		*(.rodata*)
		*(.sdata2.*)
		*(.glue_7)
		*(.glue_7t)
		*(.gnu.linkonce.t.*)
		. = ALIGN(4);
		PROVIDE(_highcode_vma_end = .);
	} >RAM_CODE AT>FLASH

	.text :
	{
		. = ALIGN(4);
		KEEP(*(SORT_NONE(.handle_reset)))
		. = ALIGN(4);
	} >FLASH AT>FLASH

	.highcodelalign1 :
	{
		. = ALIGN(4);
		PROVIDE(_highcode_lma1 = .);
	} >FLASH AT>FLASH

	.fini :
	{
		. = ALIGN(4);
		PROVIDE(_highcode_vma_start1 = .);
		KEEP(*(SORT_NONE(.fini)))
		. = ALIGN(4);
	} >RAM_CODE AT>FLASH

	PROVIDE( _etext = . );
	PROVIDE( _eitcm = . );

	.preinit_array :
	{
		PROVIDE_HIDDEN (__preinit_array_start = .);
		KEEP (*(.preinit_array))
		PROVIDE_HIDDEN (__preinit_array_end = .);
	} >RAM_CODE AT>FLASH

	.init_array :
	{
		PROVIDE_HIDDEN (__init_array_start = .);
		KEEP (*(SORT_BY_INIT_PRIORITY(.init_array.*) SORT_BY_INIT_PRIORITY(.ctors.*)))
		KEEP (*(.init_array EXCLUDE_FILE (*crtbegin.o *crtbegin?.o *crtend.o *crtend?.o ) .ctors))
		PROVIDE_HIDDEN (__init_array_end = .);
	} >RAM_CODE AT>FLASH

	.fini_array :
	{
		PROVIDE_HIDDEN (__fini_array_start = .);
		KEEP (*(SORT_BY_INIT_PRIORITY(.fini_array.*) SORT_BY_INIT_PRIORITY(.dtors.*)))
		KEEP (*(.fini_array EXCLUDE_FILE (*crtbegin.o *crtbegin?.o *crtend.o *crtend?.o ) .dtors))
		PROVIDE_HIDDEN (__fini_array_end = .);
	} >RAM_CODE AT>FLASH

	.ctors :
	{
		KEEP (*crtbegin.o(.ctors))
		KEEP (*crtbegin?.o(.ctors))
		KEEP (*(EXCLUDE_FILE (*crtend.o *crtend?.o ) .ctors))
		KEEP (*(SORT(.ctors.*)))
		KEEP (*(.ctors))
	} >RAM_CODE AT>FLASH

	.dtors :
	{
		KEEP (*crtbegin.o(.dtors))
		KEEP (*crtbegin?.o(.dtors))
		KEEP (*(EXCLUDE_FILE (*crtend.o *crtend?.o ) .dtors))
		KEEP (*(SORT(.dtors.*)))
		KEEP (*(.dtors))
		. = ALIGN(4);
		PROVIDE(_highcode_vma_end1 = .);
	} >RAM_CODE AT>FLASH

	.dalign :
	{
		. = ALIGN(4);
		PROVIDE(_data_vma = .);
	} >RAM AT>FLASH

	.dlalign :
	{
		. = ALIGN(4);
		PROVIDE(_data_lma = .);
	} >FLASH AT>FLASH

	.data :
	{
		. = ALIGN(4);
		*(.gnu.linkonce.r.*)
		*(.data .data.*)
		*(.gnu.linkonce.d.*)
		. = ALIGN(8);
		PROVIDE( __global_pointer$ = . + 0x800 );
		*(.sdata .sdata.*)
		*(.sdata2.*)
		*(.gnu.linkonce.s.*)
		. = ALIGN(8);
		*(.srodata.cst16)
		*(.srodata.cst8)
		*(.srodata.cst4)
		*(.srodata.cst2)
		*(.srodata .srodata.*)
		. = ALIGN(4);
		PROVIDE( _edata = .);
	} >RAM AT>FLASH

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
	} >RAM AT>FLASH

	PROVIDE( _end = _ebss);
	PROVIDE( end = . );
	PROVIDE( _heap_start = . );
	PROVIDE( __heap_start = . );

	.stack ORIGIN(RAM) + LENGTH(RAM) - __stack_size :
	{
		PROVIDE( _heap_end = . );
		PROVIDE( __heap_end = . );
		. = ALIGN(4);
		PROVIDE(_susrstack = . );
		. = . + __stack_size;
		PROVIDE( _eusrstack = .);
	} >RAM

	/* Boot magic (0x2017FFF8, 8 bytes, NOLOAD); only shared region BL touches. */
	.boot_magic (NOLOAD) :
	{
		. = ALIGN(4);
		__boot_magic_start__ = .;
		KEEP(*(.boot_magic .boot_magic.*))
		. = ALIGN(4);
		__boot_magic_end__ = .;
	} >BOOT_MAGIC
}

/* Shared SRAM placement symbols (checked across all 3 ELFs by tools/check_placement.py). */
__lat_shm_usbhs_dma_origin     = ORIGIN(USBHS_DMA);
__lat_shm_usbhs_dma_length     = LENGTH(USBHS_DMA);
__lat_shm_object_buf_origin    = ORIGIN(OBJECT_BUF);
__lat_shm_object_buf_length    = LENGTH(OBJECT_BUF);
__lat_shm_ipc_header_origin    = ORIGIN(IPC_HEADER);
__lat_shm_ipc_header_length    = LENGTH(IPC_HEADER);
__lat_shm_ipc_cmd_origin       = ORIGIN(IPC_CMD);
__lat_shm_ipc_cmd_length       = LENGTH(IPC_CMD);
__lat_shm_ipc_evt_safe_origin  = ORIGIN(IPC_EVT_SAFE);
__lat_shm_ipc_evt_safe_length  = LENGTH(IPC_EVT_SAFE);
__lat_shm_ipc_evt_lossy_origin = ORIGIN(IPC_EVT_LOSSY);
__lat_shm_ipc_evt_lossy_length = LENGTH(IPC_EVT_LOSSY);
__lat_shm_spare_origin         = ORIGIN(SHM_SPARE);
__lat_shm_spare_length         = LENGTH(SHM_SPARE);
__lat_shm_boot_magic_origin    = ORIGIN(BOOT_MAGIC);
__lat_shm_boot_magic_length    = LENGTH(BOOT_MAGIC);

/* ---- Shared SRAM fixed addresses (must match LAT_SHM_* in src/shared/lat_ipc.h) ---- */
ASSERT(ORIGIN(USBHS_DMA) == 0x20150000 && LENGTH(USBHS_DMA) == 0x2000, "LAT: USBHS_DMA must be 0x20150000/0x2000")
ASSERT(ORIGIN(OBJECT_BUF) == 0x20152000 && LENGTH(OBJECT_BUF) == 0x10000, "LAT: OBJECT_BUF must be 0x20152000/0x10000")
ASSERT(ORIGIN(IPC_HEADER) == 0x20162000 && LENGTH(IPC_HEADER) == 0x1000, "LAT: IPC_HEADER must be 0x20162000/0x1000")
ASSERT(ORIGIN(IPC_CMD) == 0x20163000 && LENGTH(IPC_CMD) == 0x2100, "LAT: IPC_CMD must be 0x20163000/0x2100")
ASSERT(ORIGIN(IPC_EVT_SAFE) == 0x20165100 && LENGTH(IPC_EVT_SAFE) == 0x4100, "LAT: IPC_EVT_SAFE must be 0x20165100/0x4100")
ASSERT(ORIGIN(IPC_EVT_LOSSY) == 0x20169200 && LENGTH(IPC_EVT_LOSSY) == 0x8100, "LAT: IPC_EVT_LOSSY must be 0x20169200/0x8100")
ASSERT(ORIGIN(SHM_SPARE) == 0x20171300 && LENGTH(SHM_SPARE) == 0xECF8, "LAT: SHM_SPARE must be 0x20171300/0xECF8")
ASSERT(ORIGIN(BOOT_MAGIC) == 0x2017FFF8 && LENGTH(BOOT_MAGIC) == 8, "LAT: BOOT_MAGIC must be 0x2017FFF8/8")

/* ---- No gaps or overlaps ---- */
ASSERT(ORIGIN(RAM) + LENGTH(RAM) == ORIGIN(USBHS_DMA), "LAT: RAM must end exactly at USBHS_DMA")
ASSERT(ORIGIN(OBJECT_BUF) == ORIGIN(USBHS_DMA) + LENGTH(USBHS_DMA), "LAT: gap/overlap USBHS_DMA -> OBJECT_BUF")
ASSERT(ORIGIN(IPC_HEADER) == ORIGIN(OBJECT_BUF) + LENGTH(OBJECT_BUF), "LAT: gap/overlap OBJECT_BUF -> IPC_HEADER")
ASSERT(ORIGIN(IPC_CMD) == ORIGIN(IPC_HEADER) + LENGTH(IPC_HEADER), "LAT: gap/overlap IPC_HEADER -> IPC_CMD")
ASSERT(ORIGIN(IPC_EVT_SAFE) == ORIGIN(IPC_CMD) + LENGTH(IPC_CMD), "LAT: gap/overlap IPC_CMD -> IPC_EVT_SAFE")
ASSERT(ORIGIN(IPC_EVT_LOSSY) == ORIGIN(IPC_EVT_SAFE) + LENGTH(IPC_EVT_SAFE), "LAT: gap/overlap IPC_EVT_SAFE -> IPC_EVT_LOSSY")
ASSERT(ORIGIN(SHM_SPARE) == ORIGIN(IPC_EVT_LOSSY) + LENGTH(IPC_EVT_LOSSY), "LAT: gap/overlap IPC_EVT_LOSSY -> SHM_SPARE")
ASSERT(ORIGIN(BOOT_MAGIC) == ORIGIN(SHM_SPARE) + LENGTH(SHM_SPARE), "LAT: gap/overlap SHM_SPARE -> BOOT_MAGIC")
ASSERT(ORIGIN(BOOT_MAGIC) + LENGTH(BOOT_MAGIC) == 0x20180000, "LAT: BOOT_MAGIC must end at the top of SRAM (0x20180000)")

/* ---- Boot magic ---- */
ASSERT(ADDR(.boot_magic) == ORIGIN(BOOT_MAGIC) && g_boot_magic == __boot_magic_start__, "LAT: g_boot_magic must be at 0x2017FFF8")
ASSERT(__boot_magic_end__ - __boot_magic_start__ <= LENGTH(BOOT_MAGIC), "LAT: .boot_magic overflows BOOT_MAGIC (8 bytes)")

/* ---- RAM and stack ---- */
ASSERT(_eusrstack == ORIGIN(USBHS_DMA), "LAT: BL stack must end exactly where the shared SRAM starts")
ASSERT(_ebss <= _susrstack, "LAT: BL .data+.bss overlaps the stack")
ASSERT(_highcode_vma_end1 <= ORIGIN(RAM_CODE) + LENGTH(RAM_CODE), "LAT: BL code exceeds RAM_CODE (64KB)")

/* ---- gp-relative relaxation guard (see Link_CH32H417_v3f.ld) ---- */
ASSERT(__global_pointer$ - 0x800 >= ORIGIN(RAM_CODE) + LENGTH(FLASH),
       "LAT: gp-relative window reaches addresses RAM_CODE can occupy before relaxation - move RAM higher")

/* ---- Flash image end (0x00000-0x0DFFF) ---- */
__flash_image_end__ = LOADADDR(.data) + SIZEOF(.data);
ASSERT(__flash_image_end__ <= ORIGIN(FLASH) + LENGTH(FLASH), "LAT: bootloader image exceeds 0x00000-0x0DFFF (56KB)")
ASSERT(ORIGIN(FLASH) + LENGTH(FLASH) == 0x0000E000, "LAT: bootloader Flash window must end at 0xE000")
