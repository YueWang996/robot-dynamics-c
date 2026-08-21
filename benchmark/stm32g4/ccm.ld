/* SPDX-License-Identifier: Apache-2.0
 *
 * Run the library's code out of the STM32G474's CCM SRAM instead of flash.
 * Spliced into the Cube-generated linker script by `make CCMBENCH=1`, ahead
 * of .text so it claims these sections first.
 *
 * At 170 MHz the flash needs four wait states, and the ART accelerator's
 * instruction cache is 1 KB -- smaller than rd_aba's inner loop, so the loop
 * body is re-fetched every iteration. CCM SRAM sits at 0x10000000 on the
 * I-Code bus and takes no wait states. Measured worth 21% on rd_aba.
 *
 * Loaded from flash and copied by ccm_init() in main.c before anything else
 * runs. Only the hot entry points go here: 27.7 KB of the 32 KB region, which
 * leaves no room for rd_chain_build, and it is a one-time call anyway.
 *
 * This is a characterisation option, not how the library ships. Most parts
 * have no tightly-coupled memory, and the numbers in benchmark/results/ are
 * from flash so they stay comparable with the boards that do not.
 */
  .ccmram :
  {
    . = ALIGN(4);
    _sccmram = .;
    *(.text.rd_aba .text.rd_crba .text.rnea_impl .text.rd_rnea)
    *(.text.rd_gravity .text.rd_nonlinear_terms .text.rd_coriolis)
    *(.text.rd_jacobian .text.rd_update_kinematics .text.rd_fk_frame)
    *(.text.rd_forward_kinematics .text.rd_spatial_velocity)
    *(.text.rd_spatial_acceleration .text.algo_*)
    . = ALIGN(4);
    _eccmram = .;
  } >CCMSRAM AT> ROM
  _siccmram = LOADADDR(.ccmram);
