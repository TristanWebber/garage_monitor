/* SRAM1 contains general purpose memory addressable by both the instruction bus (as iram) and the data bus (as dram). The address space of the two buses is offset by 0x70_0000, but they are in the same memory, so care must be taken to prevent the iram and dram colliding */

/* SRAM1 can be accessed by instruction bus (0x4038_000 to 0x403C_AE00) */
iram_seg_start = 0x40380000;
/* SRAM1 can be accessed by data bus (0x3FC8_000 to 0x3FCC_AE00) */
dram_seg_start = 0x3FC80000;

/* dram immediately follows iram, so offset the start of the dram space by LENGTH(iram_seg)*/

MEMORY
{
  iram_seg (RWX) :                  org = iram_seg_start, len = 32k
  dram_seg (RW) :                   org = dram_seg_start + LENGTH(iram_seg), len = 32k
}

ENTRY(call_start_cpu0)

SECTIONS {
  .text : {
    . = ALIGN(16);
    *(.text)
    *(.text*)
  } > iram_seg

  .data : {
    . = ALIGN(16);
    _sdata = .;
    *(.data)
    *(.data*)
    *(.sdata)
    *(.sdata*)
    *(.srodata)
    *(.srodata*)
    *(.rodata)
    *(.rodata*)
    *(.gnu.linkonce.r.*)
    *(.rodata1)
    *(.riscv.*)
    . = ALIGN(16);
    _edata = .;
  } > dram_seg

  .bss (NOLOAD): {
    . = ALIGN(16);
    _sbss = .;
    *(.bss)
    *(.bss*)
    *(.sbss)
    *(.sbss*)
    *(COMMON)
    . = ALIGN(16);
    _ebss = .;
  } > dram_seg

  . = ALIGN(16);
  PROVIDE(end = .);
  PROVIDE(_end = .);
}
