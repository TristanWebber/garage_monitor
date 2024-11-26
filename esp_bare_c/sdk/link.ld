/* The offset between Dbus and Ibus. Used to convert between 0x403xxxxx and 0x3fcxxxxx addresses. */
iram_dram_offset = 0x700000;

/* We consider 0x3fcdc710 to be the last usable address for 2nd stage bootloader stack overhead, dram_seg,
 * and work out iram_seg and iram_loader_seg addresses from there, backwards.
 */

/* These lengths can be adjusted, if necessary: */
bootloader_usable_dram_end = 0x3fcdc710;
bootloader_stack_overhead = 0x2000; /* For safety margin between bootloader data section and startup stacks */
bootloader_dram_seg_len = 0x5000;
bootloader_iram_loader_seg_len = 0x7000;
bootloader_iram_seg_len = 0x2000;

/* Start of the lower region is determined by region size and the end of the higher region */
bootloader_dram_seg_end = bootloader_usable_dram_end - bootloader_stack_overhead;
bootloader_dram_seg_start = bootloader_dram_seg_end - bootloader_dram_seg_len;
bootloader_iram_loader_seg_start = bootloader_dram_seg_start - bootloader_iram_loader_seg_len + iram_dram_offset;
bootloader_iram_seg_start = bootloader_iram_loader_seg_start - bootloader_iram_seg_len;

MEMORY
{
  iram_seg (RWX) :                  org = bootloader_iram_seg_start, len = bootloader_iram_seg_len
  iram_loader_seg (RWX) :           org = bootloader_iram_loader_seg_start, len = bootloader_iram_loader_seg_len
  dram_seg (RW) :                   org = bootloader_dram_seg_start, len = bootloader_dram_seg_len
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
