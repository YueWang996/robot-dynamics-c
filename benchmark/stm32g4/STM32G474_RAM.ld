/* SPDX-License-Identifier: Apache-2.0 */
/*
 * Run the whole image out of SRAM, so the G4 is measured the same way the
 * RP2350 is (that build is linked copy_to_ram). Removes flash wait states and
 * ART cache behaviour from the comparison.
 *
 * G474RE: SRAM1 80K + SRAM2 16K + CCM SRAM 32K, contiguous from 0x20000000.
 */
ENTRY(Reset_Handler)

MEMORY
{
  RAM (xrw) : ORIGIN = 0x20000000, LENGTH = 128K
}

_estack = ORIGIN(RAM) + LENGTH(RAM);
_Min_Heap_Size  = 0x400;
_Min_Stack_Size = 0x1000;

SECTIONS
{
  .isr_vector : { KEEP(*(.isr_vector)) . = ALIGN(8); } >RAM
  .text : {
    . = ALIGN(8);
    *(.text) *(.text*) *(.glue_7) *(.glue_7t) *(.eh_frame)
    KEEP(*(.init)) KEEP(*(.fini))
    . = ALIGN(8);
    _etext = .;
  } >RAM
  .rodata : { . = ALIGN(8); *(.rodata) *(.rodata*) . = ALIGN(8); } >RAM
  .ARM.extab : { *(.ARM.extab* .gnu.linkonce.armextab.*) } >RAM
  .ARM : { __exidx_start = .; *(.ARM.exidx*) __exidx_end = .; } >RAM

  .preinit_array : { PROVIDE_HIDDEN(__preinit_array_start = .);
    KEEP(*(.preinit_array*)) PROVIDE_HIDDEN(__preinit_array_end = .); } >RAM
  .init_array : { PROVIDE_HIDDEN(__init_array_start = .);
    KEEP(*(SORT(.init_array.*))) KEEP(*(.init_array*))
    PROVIDE_HIDDEN(__init_array_end = .); } >RAM
  .fini_array : { PROVIDE_HIDDEN(__fini_array_start = .);
    KEEP(*(SORT(.fini_array.*))) KEEP(*(.fini_array*))
    PROVIDE_HIDDEN(__fini_array_end = .); } >RAM

  /* Loaded in place: VMA == LMA, so the startup copy loop is a no-op. */
  _sidata = LOADADDR(.data);
  .data : { . = ALIGN(8); _sdata = .; *(.data) *(.data*) . = ALIGN(8); _edata = .; } >RAM

  . = ALIGN(8);
  .bss : { _sbss = .; __bss_start__ = _sbss;
           *(.bss) *(.bss*) *(COMMON)
           . = ALIGN(8); _ebss = .; __bss_end__ = _ebss; } >RAM

  ._user_heap_stack : {
    . = ALIGN(8); PROVIDE(end = .); PROVIDE(_end = .);
    . = . + _Min_Heap_Size; . = . + _Min_Stack_Size; . = ALIGN(8);
  } >RAM

  .ARM.attributes 0 : { *(.ARM.attributes) }
}
