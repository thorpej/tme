/* machine/phaethon1/phaethon1-mmu.c - implementation of Phaethon 1
   MMU emulation: */

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

#include <tme/common.h>

/* includes: */
#include "phaethon1-impl.h"

/* our m68k TLB filler: */
int
tme_ph1_m68k_tlb_fill(struct tme_m68k_bus_connection *conn_m68k,
                      struct tme_m68k_tlb *tlb_m68k,
                      unsigned int function_code,
                      tme_uint32_t address,
                      unsigned int cycles)
{
  struct tme_ph1 *ph1;
  struct tme_bus_tlb *tlb;
  unsigned int function_codes_mask;
  struct tme_bus_tlb tlb_mapping;
  tme_uint32_t context;
  unsigned short tlb_flags;
  int is_super;

  /* recover our ph1: */
  ph1 = (struct tme_ph1 *) conn_m68k->tme_m68k_bus_connection.tme_bus_connection.tme_connection_element->tme_element_private;

  /* get the generic bus TLB: */
  tlb = &tlb_m68k->tme_m68k_tlb_bus_tlb;

  /* if this is function code four, we handle this ourselves: */
  if (function_code == TME_M68K_FC_4) {

    /* initialize the TLB entry: */
    tme_bus_tlb_initialize(tlb);

    /* we cover the entire address space: */
    tlb->tme_bus_tlb_addr_first = 0;
    tlb->tme_bus_tlb_addr_last = 0 - (tme_bus_addr32_t) 1;

    /* we allow reading and writing: */
    tlb->tme_bus_tlb_cycles_ok = TME_BUS_CYCLE_READ | TME_BUS_CYCLE_WRITE;

    /* our bus cycle handler: */
    tlb->tme_bus_tlb_cycle_private = ph1;
    tlb->tme_bus_tlb_cycle = _tme_ph1_control_cycle_handler;

    /* this is good for function code four only: */
    tlb_m68k->tme_m68k_tlb_function_codes_mask = TME_BIT(TME_M68K_FC_4);

    /* done: */
    return (TME_OK);
  }

  /* this must be or a user or supervisor program or data function
     code: */
  assert(function_code == TME_M68K_FC_UD
         || function_code == TME_M68K_FC_UP
         || function_code == TME_M68K_FC_SD
         || function_code == TME_M68K_FC_SP);

  /* if the MMU is disabled: */
  if (__tme_predict_false((ph1->tme_ph1_sysen & TME_PH1_SYSEN_MMU) == 0)) {

    /* All bus cycles go to the firmware ROM when the MMU is disabled.  */
    (*ph1->tme_ph1_rom->tme_bus_tlb_fill)
      (ph1->tme_ph1_rom,
       tlb,
       address & ((1 << TME_PH1_ROM_ADDRESS_BITS) - 1),
       cycles);

    /* create the mapping TLB entry: */
    tlb_mapping.tme_bus_tlb_addr_first = address & (((tme_bus_addr32_t) 0) - (TME_PH1_ROM_PHYS_MASK + 1));
    tlb_mapping.tme_bus_tlb_addr_last = address | TME_PH1_ROM_PHYS_MASK;
    tlb_mapping.tme_bus_tlb_cycles_ok
      = TME_BUS_CYCLE_READ;

    /* map the filled TLB entry: */
    tme_bus_tlb_map(tlb,
                    address & ((1 << TME_PH1_ROM_ADDRESS_BITS) - 1),
                    &tlb_mapping,
                    address);

    /* good for all function codes: */
    tlb_m68k->tme_m68k_tlb_function_codes_mask
      = TME_BIT(TME_M68K_FC_UD)
        | TME_BIT(TME_M68K_FC_UP)
        | TME_BIT(TME_M68K_FC_SD)
        | TME_BIT(TME_M68K_FC_SP);

    /* done: */
    return (TME_OK);
  }

  /* if this is a user program or data function code: */
  if (function_code == TME_M68K_FC_UD
      || function_code == TME_M68K_FC_UP) {
    context = ph1->tme_ph1_context;
    is_super = 0;
    function_codes_mask = (TME_BIT(TME_M68K_FC_UD) + TME_BIT(TME_M68K_FC_UP));
  }

  /* otherwise, this is a supervisor program or data function code: */
  else {
    context = 0;
    is_super = 1;
    function_codes_mask = (TME_BIT(TME_M68K_FC_SD) + TME_BIT(TME_M68K_FC_SP));
  }

  /* fill this TLB entry from the MMU: */
  tme_pgmmu_tlb_fill(ph1->tme_ph1_mmu,
                     tlb,
                     context,
                     address,
                     cycles,
                     is_super);

  /* TLB entries are good only for the program and data function
     codes for the user or supervisor, but never both, because
     the two types of accesses go through different contexts.
     The supervisor always uses context 0, and the user uses
     whatever is in the context register.  That may, in fact,
     be 0, but that should be rare so we don't really need to
     go our of our way to optimize it in any way.  */
  tlb_m68k->tme_m68k_tlb_function_codes_mask = function_codes_mask;

  return (TME_OK);
}

/* this is called to fill in the physical address information
   (and adjust for incomplete address decoding, as needed) for
   the pg68k MMU TLB filler.  */
static int
tme_ph1_tlb_fill_mmu(void *_ph1,
                     struct tme_bus_tlb *tlb,
                     tme_uint32_t pme,
                     tme_uint32_t *addressp,
                     unsigned int cycles)
{
  static const tme_uint32_t phys_addr_masks[] = {
    [TME_PH1_BUS_RAM]  = (1 << TME_PH1_RAM_ADDRESS_BITS) - 1,
    [TME_PH1_BUS_ROM]  = (1 << TME_PH1_ROM_ADDRESS_BITS) - 1,
    [TME_PH1_BUS_OBIO] = (1 << TME_PH1_OBIO_ADDRESS_BITS) - 1,
    [TME_PH1_BUS_VME]  = (1 << TME_PH1_VME_ADDRESS_BITS) - 1,
  };
  static const tme_bus_fault_handler bus_fault_handlers[] = {
    [TME_PH1_BUS_RAM]  = tme_ph1_ram_bus_fault_handler,
    [TME_PH1_BUS_ROM]  = tme_ph1_rom_bus_fault_handler,
    [TME_PH1_BUS_OBIO] = tme_ph1_obio_bus_fault_handler,
    [TME_PH1_BUS_VME]  = tme_ph1_vme_bus_fault_handler,
  };
  struct tme_ph1 *ph1;
  tme_uint32_t physaddr;
  unsigned int bus_type;
  struct tme_bus_connection *conn_bus;
  int rc;

  /* recover our ph1: */
  ph1 = (struct tme_ph1 *) _ph1;

  /* get the physical page frame and bus type: */
  physaddr = (pme & TME_PGMMU_PME_PFN_MASK) << TME_PH1_PAGE_SIZE_LOG2;
  bus_type = TME_PH1_PHYS_TO_BUS(address);

  /* clamp the physical address according to how completely (or not)
     the bus decodes.  */
  physaddr &= phys_addr_masks[bus_type];

  /* add in the page offset to finish the address: */
  physaddr |= *addressp & (TME_PH1_PAGE_SIZE - 1);
  *addressp = physaddr;

  /* call the bus TLB filler: */
  conn_bus = ph1->tme_ph1_buses[bus_type];
  rc = ((*conn_bus->tme_bus_tlb_fill)
        (conn_bus, tlb, physaddr, cycles));

  /* if the bus TLB filler succeeded, add our bus fault handler: */
  if (rc == TME_OK) {
    TME_BUS_TLB_FAULT_HANDLER(tlb,
                              bus_fault_handlers[bus_type],
                              ph1);
  }

  return (rc);
}

/* this gets a segmap entry from the MMU: */
tme_uint16_t
tme_ph1_mmu_sme_get(struct tme_ph1 *ph1,
                    tme_uint8_t context,
                    tme_uint32_t control_address)
{
  control_address = (control_address >> 4) << 4;
  return (tme_pg68k_mmu_sme_get(ph1->tme_ph1_mmu,
                                context,
                                control_address));
}

/* this sets a segmap entry into the MMU: */
void
tme_ph1_mmu_sme_set(struct tme_ph1 *ph1,
                    tme_uint8_t context,
                    tme_uint32_t control_address,
                    tme_uint16_t sme)
{
  control_address = (control_address >> 4) << 4;
  tme_pg68k_mmu_sme_set(ph1->tme_ph1_mmu,
                        context,
                        control_address,
                        sme);
}

/* this gets a PME from the MMU: */
tme_uint32_t
tme_ph1_mmu_pme_get(struct tme_ph1 *ph1,
                    tme_uint32_t control_address)
{
  unsigned int pme_index = (control_address >> 4);
  return (tme_pg68k_mmu_pme_get(ph1->tme_ph1_mmu,
                                pme_index));
}

/* this sets a PME into the MMU: */
int
tme_ph1_mmu_pme_set(struct tme_ph1 *ph1,
                    tme_uint32_t control_address,
                    tme_uint32_t pme)
{
  unsigned int pme_index = (control_address >> 4);
#ifndef TME_NO_LOG
  static const char bus_names[] = {
    [TME_PH1_BUS_RAM]  = "ram",
    [TME_PH1_BUS_ROM]  = "rom",
    [TME_PH1_BUS_OBIO] = "obio",
    [TME_PH1_BUS_VME]  = "vme",
  };
  tme_bus_addr32_t physical_address;
  unsigned int bus_type;

  /* this silences gcc -Wuninitialized: */
  bus_name = NULL;

  /* log this setting: */
  physical_address = ((pme & TME_PGMMU_PME_PFN_MASK) << TME_PH1_PAGE_SIZE_LOG2);
  physical_address &= ~(TME_PH1_PHYS_TO_BUS_MASK << TME_PH1_PHYS_TO_BUS_SHIFT)
  bus_type = TME_PH1_PHYS_TO_BUS(physical_address);
  tme_log(TME_PH1_LOG_HANDLE(ph1), 1000, TME_OK,
          (TME_PH1_LOG_HANDLE(ph1),
           _("pte_set: PGMAP[%u] <- 0x%08x (%s 0x%08x)"),
           ph1->tme_ph1_context,
           pme_index,
           pme,
           bus_names[bus_type],
           physical_address));
#endif /* TME_NO_LOG */

  return (tme_pg68k_mmu_pme_set(ph1->tme_ph1_mmu,
                                pme_index,
                                pme));
}

/* this is called when the context register is set: */
void
tme_ph1_mmu_context_set(struct tme_ph1 *ph1)
{
  tme_uint8_t context;

  /* N.B. even though user and supervisor reference use two different
     contexts, we ensure that TLB entries for one are never usable by
     the other.  */

  tme_log(TME_PH1_LOG_HANDLE(ph1), 1000, TME_OK,
          (TME_PH1_LOG_HANDLE(ph1),
           _("context now #%d"),
           ph1->tme_ph1_context));

  /* when the MMU is disabled, all regular bus cycles go to the firmware
     ROM, so we have a fake context that is used used when that is the case. */
  if (__tme_predict_false((ph1->tme_ph1_sysen & TME_PH1_SYSEN_MMU) == 0)) {
    context = TME_PH1_NUM_CONTEXTS;
  }
  else {
    context = ph1->tme_ph1_context;
  }

  /* update the m68k bus context register: */
  *ph1->tme_ph1_m68k_bus_context = context;
}

/* this adds a new TLB set: */
int
tme_ph1_mmu_tlb_set_add(struct tme_bus_connection *conn_bus_asker,
                        struct tme_bus_tlb_set_info *tlb_set_info)
{
  struct tme_ph1 *ph1;
  int rc;

  /* recover our ph1: */
  ph1 = (struct tme_ph1 *) conn_bus_asker->tme_bus_connection.tme_connection_element->tme_element_private;

  /* add the TLB set to the MMU: */
  rc = tme_pg68k_mmu_tlb_set_add(ph1->tme_ph1_mmu,
                                 tlb_set_info);
  assert(rc == TME_OK);

  /* if this is the TLB set from the m68k: */
  if (conn_bus_asker->tme_bus_connection.tme_connection_type
      == TME_CONNECTION_BUS_M68K) {

    /* the m68k must expose a bus context register: */
    assert(tlb_set_info->tme_bus_tlb_set_info_bus_context != NULL);

    /* save the pointer to the m68k bus context register, and
       initialize it: */
    ph1->tme_ph1_m68k_bus_context
      = tlb_set_info->tme_bus_tlb_set_info_bus_context;
    tme_ph1_mmu_context_set(ph1);

    /* return the maximum context number.  there are 64 contexts
       when the MMU is enabled.  when the MMU is disabled, all
       regular bus cycles go to the ROM, so we create a fake
       65th context that is used when MMU is disabled.  */
    tlb_set_info->tme_bus_tlb_set_info_bus_context_max
      = TME_PH1_NUM_CONTEXTS;
  }

  return (rc);
}

/* this creates a Phaethon 1 MMU: */
void
tme_ph1_mmu_new(struct tme_ph1 *ph1)
{
  struct tme_pg68k_mmu_info mmu_info;

  memset(&mmu_info, 0, sizeof(mmu_info));

  mmu_info.tme_pg68k_mmu_info_element = ph1->tme_ph1_element;

  mmu_info.tme_pg68k_mmu_info_address_bits = TME_PH1_ADDRESS_BITS;
  mmu_info.tme_pg68k_mmu_info_pgoffset_bits = TME_PH1_PAGE_SIZE_LOG2;
  mmu_info.tme_pg68k_mmu_info_pmeg_offset_bits = TME_PH1_PMEG_OFFSET_BITS;
  mmu_info.tme_pg68k_mmu_info_num_contexts = TME_PH1_NUM_CONTEXTS;
  mmu_info.tme_pg68k_mmu_info_num_pmegs = TME_PH1_NUM_PMEGS;

  mmu_info.tme_pg68k_mmu_info_tlb_fill_private = ph1;
  mmu_info.tme_pg68k_mmu_info_tlb_fill = tme_ph1_tlb_fill_mmu;

  mmu_info.tme_pg68k_mmu_info_invalid_private = ph1;
  mmu_info.tme_pg68k_mmu_info_invalid = tme_ph1_mmu_invalid;

  mmu_info.tme_pg68k_mmu_info_priv_private = ph1;
  mmu_info.tme_pg68k_mmu_info_priv = tme_ph1_mmu_priv;

  mmu_info.tme_pg68k_mmu_info_prot_private = ph1;
  mmu_info.tme_pg68k_mmu_info_prot = tme_ph1_mmu_prot;
}
