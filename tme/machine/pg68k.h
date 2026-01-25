/* tme/machine/pg68k.h - public header file for 68k Playground emulation:
   (Phaethon 1, ...)  */

/*
 * Copyright (c) 2026 Jason R. Thorpe
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

#ifndef _TME_MACHINE_PG68K_H
#define _TME_MACHINE_PG68K_H

#include <tme/common.h>

/* includes: */
#include <tme/element.h>
#include <tme/generic/bus.h>

/* macros: */

/* pg68k MMU (for 68010, 68020) segment map entry bits.
   segment map entries are 16-bits  */
#define TME_PGMMU_SME_PMEG_MASK (0x7fff)  /* pmeg index for segment */
#define TME_PGMMU_SME_V         (0x8000)  /* segmap entry valid */

/* ...and page map entry bits.
   page map entries are 32-bits  */
#define TME_PGMMU_PME_PFN_MASK  (0x0000ffff)  /* page frame number */
#define TME_PGMMU_PME_MOD       TME_BIT(24)   /* page was modified */
#define TME_PGMMU_PME_REF       TME_BIT(25)   /* page was referenced */
#define TME_PGMMU_PME_K         TME_BIT(29)   /* kernel-access only page */
#define TME_PGMMU_PME_W         TME_BIT(30)   /* writable page */
#define TME_PGMMU_PME_V         TME_BIT(31)   /* pagemap entry valid */

/* a filled TLB entry can be valid for the system, or the user, or
   both.  because TME_BIT is a little too paranoid and uses a type
   cast, we can't use it and then use these macros in preprocessor
   expressions: */
#define TME_PGMMU_TLB_SYSTEM    (1 << 0)
#define TME_PGMMU_TLB_USER      (1 << 1)

/* structures: */

/* the parameters for a pg68k MMU: */
struct tme_pg68k_mmu_info {

  /* the mainbus element: */
  struct tme_element *tme_pg68k_mmu_info_element;

  /* the number of bits in an address: */
  tme_uint8_t tme_pg68k_mmu_info_address_bits;

  /* the number of bits in a page offset: */
  tme_uint8_t tme_pg68k_mmu_info_pgoffset_bits;

  /* the number of bits in a pmeg offset: */
  tme_uint8_t tme_pg68k_mmu_info_pmeg_offset_bits;

  /* pgoffset_bits + pmeg_offset_bits -> segoffset_bits */

  /* address_bits - (pgoffset_bits + pmeg_offset_bits) -> segment_bits */
 
  /* the number of contexts: */
  tme_uint8_t tme_pg68k_mmu_info_num_contexts;

  /* the number of PMEGs: */
  unsigned int tme_pg68k_mmu_info_num_pmegs;

  /* a TLB filler: */
  void *tme_pg68k_mmu_info_tlb_fill_private;
  int (*tme_pg68k_mmu_info_tlb_fill) _TME_P((void *,
                                             struct tme_bus_tlb *,
                                             tme_uint32_t *, /* PME */
                                             tme_uint32_t *, /* phys addr out */
                                             unsigned int)); /* r/w cycles */

  /* the page-invalid cycle handler: */
  void *tme_pg68k_mmu_info_invalid_private;
  tme_bus_cycle_handler tme_pg68k_mmu_info_invalid;

  /* the privilege error cycle handler: */
  void *tme_pg68k_mmu_info_priv_private;
  tme_bus_cycle_handler tme_pg68k_mmu_info_priv;

  /* the protection error cycle handler: */
  void *tme_pg68k_mmu_info_prot_private;
  tme_bus_cycle_handler tme_pg68k_mmu_info_prot;
};

/* prototypes: */

/* MMU support: */
void *tme_pg68k_mmu_new _TME_P((struct tme_pg68k_mmu_info *));

void tme_pg68k_mmu_pme_set _TME_P((void *,
                                   unsigned int,
                                   tme_uint32_t));
tme_uint32_t tme_pg68k_mmu_pme_get _TME_P((void *,
                                           unsigned int));

void tme_pg68k_mmu_sme_set _TME_P((void *,
                                   tme_uint8_t,
                                   tme_uint32_t,
                                   tme_uint16_t));
tme_uint16_t tme_pg68k_mmu_sme_get _TME_P((void *,
                                           tme_uint8_t,
                                           tme_uint32_t));

unsigned int tme_pg68k_mmu_tlb_fill _TME_P((void *,
                                            struct tme_bus_tlb *,
                                            tme_uint8_t,  /* context */
                                            tme_uint32_t, /* address */
                                            unsigned int, /* cycles */
                                            int));        /* is_super */

void tme_pg68k_mmu_tlbs_invalidate _TME_P((void *));

int tme_pg68k_mmu_tlb_set_add _TME_P((void *,
                                      struct tme_bus_tlb_set_info *));

#endif /* !_TME_MACHINE_PG68K_H */
