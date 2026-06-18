/* machine/phaethon1/phaethon1-impl.h - implementation header file for
   Phaethon 1 emulation:  */

/*
 * Copyright (c) 2026 Jason R. Thorpe.
 * All rights reserved.
 */

/*
 * Copyright (c) 2003 Matt Fredette
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *      This product includes software developed by Matt Fredette.
 * 4. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _MACHINE_PHAETHON1_IMPL_H
#define _MACHINE_PHAETHON1_IMPL_H

#include <tme/common.h>

/* includes: */
#include <tme/generic/bus.h>
#include <tme/generic/ic.h>
#include <tme/machine/sun.h>
#include <tme/ic/m68k.h>
#include <tme/element.h>

/* macros: */

/* the Phaethon 1 control register file can be thought of as 16-bit
   big-endian memory, so we use a struct tme_ic to hold the register
   file, and use the TME_IC_BUS_IREGN macros to map it using real
   Phaethon 1 control addresses.

   The addressing scheme of the Phaeton 1's control space is a little
   funky:

   Bits 1..3 == 0 -> board control space
   Bits 1..3 != 0 -> MMU control space

   XXX Should do this differently maybe?  Give MMU its own IC?  */
#define _TME_PH1_CONTROL_IREG8(addr) \
  tme_ph1_ic.tme_ic_ireg_uint8(TME_IC_BUS_IREG8(1, TME_ENDIAN_BIG, addr))
#define _TME_PH1_CONTROL_IREG16(addr) \
  tme_ph1_ic.tme_ic_ireg_uint16(TME_IC_BUS_IREG16(1, TME_ENDIAN_BIG, addr))
/* board control space: */
#define tme_ph1_dd7seg_u  _TME_PH1_CONTROL_IREG8(0x00)
#define tme_ph1_dd7seg_l  _TME_PH1_CONTROL_IREG8(0x01)
#define tme_ph1_cfgsw     _TME_PH1_CONTROL_IREG16(0x10)
#define tme_ph1_sysen     _TME_PH1_CONTROL_IREG8(0x20)
#define tme_ph1_intrset   _TME_PH1_CONTROL_IREG8(0x40)
#define tme_ph1_intrclr   _TME_PH1_CONTROL_IREG8(0x50)
#define tme_ph1_brdrev    _TME_PH1_CONTROL_IREG8(0xe0)
#define tme_ph1_pldrev    _TME_PH1_CONTROL_IREG8(0xf0)
/* mmu control space: */
#define tme_ph1_segmap0   _TME_PH1_CONTROL_IREG16((1 << 1))
#define tme_ph1_segmap    _TME_PH1_CONTROL_IREG16((2 << 1))
#define tme_ph1_context   _TME_PH1_CONTROL_IREG8((3 << 1))
#define tme_ph1_pagemap_u _TME_PH1_CONTROL_IREG16((4 << 1))
#define tme_ph1_pagemap_l _TME_PH1_CONTROL_IREG16((5 << 1))
#define tme_ph1_buserror  _TME_PH1_CONTROL_IREG8((6 << 1))

#define TME_PH1_BRDREV    1 /* indicates TME emulator */
#define TME_PH1_PLDREV    0 /* TME PLD revision 0 */

#define TME_PH1_CONTROL_REG_MMU_P(addr) \
  (((addr) & (7 << 1)) != 0)
#define TME_PH1_CONTROL_REG_MMU_SEL(addr) \
  ((addr) & 0x0f)
#define TME_PH1_CONTROL_REG_BOARD_SEL(addr) \
  ((addr) & 0xf0)

/* System Enable Register bits: */
#define TME_PH1_SYSEN_MMU     0x01  /* enable the MMU */
#define TME_PH1_SYSEN_INT     0x02  /* enable interrupts */
/* The remaining System Enable Register bits are used by software. */

/* Interrupt Set/Clear Register bits: */
#define TME_PH1_SWINT_IPL1    0x01
#define TME_PH1_SWINT_IPL2    0x02
#define TME_PH1_SWINT_MASK    (TME_PH1_SWINT_IPL1 | TME_PH1_SWINT_IPL2)

/* this recovers the control address of a register.  it should
   optimize right down to a constant: */
#define TME_PH1_CONTROL_ADDRESS(reg)  \
  TME_IC_IREG_BUS(1, TME_ENDIAN_BIG, reg, tme_ph1, tme_ph1_ic.)

/* this starts a bus cycle structure: */
#define TME_PH1_CONTROL_BUS_CYCLE(sun2, addr, cycle)  \
  TME_IC_IREG_BUS_CYCLE(1, TME_ENDIAN_BIG, &ph1->tme_ph1_ic, addr, cycle)

/* MMU configuration: */
#define TME_PH1_ADDRESS_BITS    (24)
#define TME_PH1_PAGE_SIZE_LOG2  (12)
#define TME_PH1_PAGE_SIZE       (1 << TME_PH1_PAGE_SIZE_LOG2)
#define TME_PH1_PMEG_OFFSET_BITS (3)
#define TME_PH1_NUM_CONTEXTS    (64)  /* kernel is always context 0 */
#define TME_PH1_NUM_PMEGS       (32768)

/* identifiers for the different buses.
   N.B. these map directly to bits 26..27 of the physical address!  */
#define TME_PH1_BUS_RAM         (0)
#define TME_PH1_BUS_ROM         (1)
#define TME_PH1_BUS_OBIO        (2)
#define TME_PH1_BUS_VME         (3)
#define TME_PH1_BUS_COUNT       (4)

#define TME_PH1_PHYS_TO_BUS_MASK  (3)
#define TME_PH1_PHYS_TO_BUS_SHIFT (26)

#define TME_PH1_PHYS_TO_BUS(phys) \
  (((phys) >> TME_PH1_PHYS_TO_BUS_SHIFT) & TME_PH1_PHYS_TO_BUS_MASK)

/* Not all of the 4 Phaethon 1 busses fully decode the address space.

   RAM    Fully decoded (26 address bits)
   ROM    Lower 20 bits only; ROM repeats every 1MB
   OBIO   Lower 12 bits only; I/O space repeats every 4K
   VME    Lower 24 bits only; VME space repeats every 16MB  */
#define TME_PH1_RAM_ADDRESS_BITS  (26)
#define TME_PH1_ROM_ADDRESS_BITS  (20)
#define TME_PH1_OBIO_ADDRESS_BITS (12)
#define TME_PH1_VME_ADDRESS_BITS  (24)

#define TME_PH1_LOG_HANDLE(ph1) \
  (&(ph1)->tme_ph1_element->tme_element_log_handle)

/* types: */

/* a Phaethon 1 mainbus connection: */
struct tme_ph1_bus_connection {

  /* the generic bus connection: */
  struct tme_bus_connection tme_ph1_bus_connection;

  /* which bus this is: */
  unsigned int tme_ph1_bus_connection_which;
};

/* a Phaethon 1 */
struct tme_ph1 {

  /* our IC data structure, containing our various registers: */
  struct tme_ic tme_ph1_ic;

  /* backpointer to our element: */
  struct tme_element *tme_ph1_element;

  /* the MMU: */
  void *tme_ph1_mmu;

  /* the CPU: */
  struct tme_m68k_bus_connection *tme_ph1_m68k;

  /* the different busses: */
  struct tme_bus_connection *tme_ph1_buses[TME_PH1_BUS_COUNT];
#define tme_ph1_ram   tme_ph1_buses[TME_PH1_BUS_RAM]
#define tme_ph1_rom   tme_ph1_buses[TME_PH1_BUS_ROM]
#define tme_ph1_obio  tme_ph1_buses[TME_PH1_BUS_OBIO]
#define tme_ph1_vme   tme_ph1_buses[TME_PH1_BUS_VME]

  /* the interrupt lines that are being asserted: */
  tme_uint8_t tme_ph1_int_signals[(TME_M68K_IPL_MAX + 1 + 7) >> 3];

  /* the software interrupt pending register: */
  tme_uint8_t tme_ph1_swint_pending;

  /* dd7seg is a write-only register; this holds the real value: */
  tme_uint8_t tme_ph1_dd7seg_upper_value;
  tme_uint8_t tme_ph1_dd7seg_lower_value;

  /* cfgsw is a read-only register; this holds the real value: */
  tme_uint16_t tme_ph1_cfgsw_value;

  /* buserror is a read-only register; this holds the real value: */
  tme_uint8_t tme_ph1_buserror_value;

  /* the last ipl we gave to the CPU: */
  unsigned int tme_ph1_int_ipl_last;

  /* the m68k bus context register: */
  tme_bus_context_t *tme_ph1_m68k_bus_context;
};

/* prototypes: */
void  _tme_ph1_mmu_new _TME_P((struct tme_ph1 *));
int   _tme_ph1_m68k_tlb_fill _TME_P((struct tme_m68k_bus_connection *,
                                     struct tme_m68k_tlb *,
                                     unsigned int, tme_uint32_t,
                                     unsigned int));
int   _tme_ph1_bus_tlb_fill _TME_P((struct tme_bus_connection *,
                                    struct tme_bus_tlb *,
                                    tme_bus_addr_t,
                                    unsigned int));
int   _tme_ph1_mmu_tlb_set_add _TME_P((struct tme_bus_connection *,
                                       struct tme_bus_tlb_set_info *));

tme_uint16_t _tme_ph1_mmu_sme_get _TME_P((struct tme_ph1 *,
                                          tme_uint8_t,
                                          tme_uint32_t));
void  _tme_ph1_mmu_sme_set _TME_P((struct tme_ph1 *,
                                   tme_uint8_t,
                                   tme_uint32_t,
                                   tme_uint16_t));

tme_uint32_t _tme_ph1_mmu_pme_get _TME_P((struct tme_ph1 *, tme_uint32_t));
void  _tme_ph1_mmu_pme_set _TME_P((struct tme_ph1 *, tme_uint32_t,
                                   tme_uint32_t));

void  _tme_ph1_mmu_context_set _TME_P((struct tme_ph1 *));

int   _tme_ph1_control_cycle_handler _TME_P((void *,
                                             struct tme_bus_cycle *));
int   _tme_ph1_ipl_check _TME_P((struct tme_ph1 *));

#endif /* _MACHINE_PHAETHON1_IMPL_H */
