ENTRY( _start )

__stack_size = 4096;

PROVIDE( _stack_size = __stack_size );


MEMORY
{
	FLASH (rx)          : ORIGIN = 0x00030000, LENGTH = 256K                   /* V5F app 0x30000-0x6FFFF, copied to ITCM */
	RAM_CODE (xrw)      : ORIGIN = 0x200A0000, LENGTH = 128K                   /* ITCM: executable code */
	RAM (xrw)           : ORIGIN = (0x200C0000+512+256), LENGTH = (256K-512-256) /* DTCM: .data, .bss, stack */
	RAM_LOAD (xrw)      : ORIGIN = (0x200C0000+512), LENGTH = 256              /* DTCM: startup .load */
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
 * CH32H417 latency tester - V5F application linker script
 * - FLASH starts at 0x30000 (256KB) for NVIC_WakeUp_V5F(0x30000).
 * - All .text/.text.* goes to .highcode (ITCM); only .init/_start and handle_reset stay in Flash.
 * - .rodata/.srodata/.sdata2 go to DTCM .data to preserve ITCM for code.
 * - Shared SRAM NOLOAD sections use fixed addresses matching V3F and BL.
 * - gp-relative relaxation is disabled via -Wl,--no-relax to avoid cross-TCM miscalculation.
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
		*(.rodata)
		*(.rodata*)
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

	/*
	 * Shared SRAM NOLOAD sections (same as Link_CH32H417_v3f.ld).
	 * V5F must not touch anything except the header until lat_ipc_v5f_validate() passes.
	 */
	.usbhs_dma (NOLOAD) :
	{
		. = ALIGN(64);
		__usbhs_dma_start__ = .;
		KEEP(*(.usbhs_dma .usbhs_dma.*))
		. = ALIGN(4);
		__usbhs_dma_end__ = .;
	} >USBHS_DMA

	.object_buf (NOLOAD) :
	{
		. = ALIGN(64);
		__object_buf_start__ = .;
		KEEP(*(.object_buf .object_buf.*))
		. = ALIGN(4);
		__object_buf_end__ = .;
	} >OBJECT_BUF

	.ipc_header (NOLOAD) :
	{
		. = ALIGN(64);
		__ipc_header_start__ = .;
		KEEP(*(.ipc_header .ipc_header.*))
		. = ALIGN(4);
		__ipc_header_end__ = .;
	} >IPC_HEADER

	.ipc_cmd (NOLOAD) :
	{
		. = ALIGN(64);
		__ipc_cmd_start__ = .;
		KEEP(*(.ipc_cmd .ipc_cmd.*))
		. = ALIGN(4);
		__ipc_cmd_end__ = .;
	} >IPC_CMD

	.ipc_evt_safe (NOLOAD) :
	{
		. = ALIGN(64);
		__ipc_evt_safe_start__ = .;
		KEEP(*(.ipc_evt_safe .ipc_evt_safe.*))
		. = ALIGN(4);
		__ipc_evt_safe_end__ = .;
	} >IPC_EVT_SAFE

	.ipc_evt_lossy (NOLOAD) :
	{
		. = ALIGN(64);
		__ipc_evt_lossy_start__ = .;
		KEEP(*(.ipc_evt_lossy .ipc_evt_lossy.*))
		. = ALIGN(4);
		__ipc_evt_lossy_end__ = .;
	} >IPC_EVT_LOSSY

	/*
	 * V5F-only work area (48KB). V3F/BL place nothing here (V3F zero-clears on boot).
	 */
	.shm_spare (NOLOAD) :
	{
		. = ALIGN(64);
		__shm_spare_start__ = .;
		KEEP(*(.shm_spare .shm_spare.*))
		. = ALIGN(4);
		__shm_spare_end__ = .;
	} >SHM_SPARE

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

/* ---- Contiguous layout with no gaps or overlaps, ending at 0x20180000 ---- */
ASSERT(ORIGIN(OBJECT_BUF) == ORIGIN(USBHS_DMA) + LENGTH(USBHS_DMA), "LAT: gap/overlap USBHS_DMA -> OBJECT_BUF")
ASSERT(ORIGIN(IPC_HEADER) == ORIGIN(OBJECT_BUF) + LENGTH(OBJECT_BUF), "LAT: gap/overlap OBJECT_BUF -> IPC_HEADER")
ASSERT(ORIGIN(IPC_CMD) == ORIGIN(IPC_HEADER) + LENGTH(IPC_HEADER), "LAT: gap/overlap IPC_HEADER -> IPC_CMD")
ASSERT(ORIGIN(IPC_EVT_SAFE) == ORIGIN(IPC_CMD) + LENGTH(IPC_CMD), "LAT: gap/overlap IPC_CMD -> IPC_EVT_SAFE")
ASSERT(ORIGIN(IPC_EVT_LOSSY) == ORIGIN(IPC_EVT_SAFE) + LENGTH(IPC_EVT_SAFE), "LAT: gap/overlap IPC_EVT_SAFE -> IPC_EVT_LOSSY")
ASSERT(ORIGIN(SHM_SPARE) == ORIGIN(IPC_EVT_LOSSY) + LENGTH(IPC_EVT_LOSSY), "LAT: gap/overlap IPC_EVT_LOSSY -> SHM_SPARE")
ASSERT(ORIGIN(BOOT_MAGIC) == ORIGIN(SHM_SPARE) + LENGTH(SHM_SPARE), "LAT: gap/overlap SHM_SPARE -> BOOT_MAGIC")
ASSERT(ORIGIN(BOOT_MAGIC) + LENGTH(BOOT_MAGIC) == 0x20180000, "LAT: BOOT_MAGIC must end at the top of SRAM (0x20180000)")

/* ---- V5F ITCM and DTCM do not overlap shared SRAM ---- */
ASSERT(ORIGIN(RAM) + LENGTH(RAM) <= 0x20100000, "LAT: V5F DTCM region must end at or below 0x20100000")
ASSERT(ORIGIN(RAM_CODE) + LENGTH(RAM_CODE) <= 0x200C0000, "LAT: V5F ITCM region must end at or below 0x200C0000")

/* ---- Section bounds: start at region origin, size within region ---- */
ASSERT(ADDR(.usbhs_dma) == ORIGIN(USBHS_DMA) && g_usbhs_dma == __usbhs_dma_start__, "LAT: g_usbhs_dma must start USBHS_DMA")
ASSERT(__usbhs_dma_end__ - __usbhs_dma_start__ <= LENGTH(USBHS_DMA), "LAT: .usbhs_dma overflows USBHS_DMA")
ASSERT(__usbhs_dma_end__ - __usbhs_dma_start__ >= 2048, "LAT: USBHS DMA buffers must be at least 2 x 1024 bytes")
ASSERT((g_usbhs_dma % 4) == 0, "LAT: USBHS DMA buffers must be 4-byte aligned")
ASSERT(ADDR(.object_buf) == ORIGIN(OBJECT_BUF) && g_object_buf == __object_buf_start__, "LAT: g_object_buf must start OBJECT_BUF")
ASSERT(__object_buf_end__ - __object_buf_start__ <= LENGTH(OBJECT_BUF), "LAT: .object_buf overflows OBJECT_BUF")
ASSERT(__object_buf_end__ - __object_buf_start__ >= 65536, "LAT: OBJECT_BUF must be at least 64KB")
ASSERT(ADDR(.ipc_header) == ORIGIN(IPC_HEADER) && g_ipc_header == __ipc_header_start__, "LAT: g_ipc_header must start IPC_HEADER")
ASSERT(__ipc_header_end__ - __ipc_header_start__ <= LENGTH(IPC_HEADER), "LAT: .ipc_header overflows IPC_HEADER")
ASSERT(ADDR(.ipc_cmd) == ORIGIN(IPC_CMD) && g_ipc_cmd == __ipc_cmd_start__, "LAT: g_ipc_cmd must start IPC_CMD")
ASSERT(__ipc_cmd_end__ - __ipc_cmd_start__ <= LENGTH(IPC_CMD), "LAT: .ipc_cmd overflows IPC_CMD")
ASSERT(ADDR(.ipc_evt_safe) == ORIGIN(IPC_EVT_SAFE) && g_ipc_evt_safe == __ipc_evt_safe_start__, "LAT: g_ipc_evt_safe must start IPC_EVT_SAFE")
ASSERT(__ipc_evt_safe_end__ - __ipc_evt_safe_start__ <= LENGTH(IPC_EVT_SAFE), "LAT: .ipc_evt_safe overflows IPC_EVT_SAFE")
ASSERT(ADDR(.ipc_evt_lossy) == ORIGIN(IPC_EVT_LOSSY) && g_ipc_evt_lossy == __ipc_evt_lossy_start__, "LAT: g_ipc_evt_lossy must start IPC_EVT_LOSSY")
ASSERT(__ipc_evt_lossy_end__ - __ipc_evt_lossy_start__ <= LENGTH(IPC_EVT_LOSSY), "LAT: .ipc_evt_lossy overflows IPC_EVT_LOSSY")
ASSERT(ADDR(.boot_magic) == ORIGIN(BOOT_MAGIC) && g_boot_magic == __boot_magic_start__, "LAT: g_boot_magic must be at 0x2017FFF8")
ASSERT(__boot_magic_end__ - __boot_magic_start__ <= LENGTH(BOOT_MAGIC), "LAT: .boot_magic overflows BOOT_MAGIC (8 bytes)")
ASSERT(__shm_spare_end__ - __shm_spare_start__ <= LENGTH(SHM_SPARE), "LAT: .shm_spare overflows SHM_SPARE")

/* ---- DTCM and stack ---- */
ASSERT(_eusrstack == ORIGIN(RAM) + LENGTH(RAM), "LAT: V5F stack must end at the top of DTCM RAM")
ASSERT(_ebss <= _susrstack, "LAT: V5F .data+.bss overlaps the stack")

/* ---- ITCM (128KB) limit ---- */
ASSERT(_highcode_vma_end1 <= ORIGIN(RAM_CODE) + LENGTH(RAM_CODE), "LAT: V5F code exceeds ITCM (128KB)")

/* ---- Flash image end (0x30000-0x6FFFF) ---- */
__flash_image_end__ = LOADADDR(.data) + SIZEOF(.data);
ASSERT(__flash_image_end__ <= ORIGIN(FLASH) + LENGTH(FLASH), "LAT: V5F image exceeds its 256KB Flash window (0x30000-0x6FFFF)")
ASSERT(ORIGIN(FLASH) + LENGTH(FLASH) == 0x00070000, "LAT: V5F Flash window must end at 0x70000")
