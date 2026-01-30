/* machine/phaethon1/phaethon1-control.c - implementation of
   Phaethon 1 control space emulation:  */

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

#include <tme/common.h>

/* includes: */
#include <tme/machine/pg68k.h>
#include "phaethon1-impl.h"

/* macros: */

static void
_tme_ph1_dd7seg_update_display(struct tme_ph1 *ph1)
{
  /* XXX */
}

/* the bus cycle handler for function code 4 space: */
int
_tme_ph1_control_cycle_handler(void *_ph1,
                               struct tme_bus_cycle *cycle_init)
{
  struct tme_ph1 *ph1;
  struct tme_bus_cycle cycle_resp;
  tme_bus_addr32_t reg, address;
  tme_uint32_t pme;
  int rc, needs_ipl_check;
  tme_uint8_t old_sysen_value;

  /* recover our ph1: */
  ph1 = (struct tme_ph1 *) _ph1;

  /* get the register and address and index: */
  reg = address = cycle_init->tme_bus_cycle_address;
  if (TME_PH1_CONTROL_REG_MMU_P(reg)) {
    reg = TME_PH1_CONTROL_REG_MMU_SEL(reg);
  } else {
    reg = TME_PH1_CONTROL_REG_BOARD_SEL(reg);
  }

  /* true if we need to perform an ipl check. */
  needs_ipl_check = FALSE;

  /* remember the current value of the system enable register. */
  old_sysen_value = ph1->tme_ph1_sysen;

  /* this macro evaluates to TRUE whenever a register is maybe being
     accessed:  */
#define _TME_PH1_REG_ACCESSED(icreg)                            \
  ((TME_PH1_CONTROL_ADDRESS(icreg) == 0)                        \
   ? (reg < sizeof(ph1->icreg))                                 \
   : TME_RANGES_OVERLAP(reg,                                    \
                        reg                                     \
                        + cycle_init->tme_bus_cycle_size - 1,   \
                        !TME_PH1_CONTROL_ADDRESS(icreg) +       \
                        TME_PH1_CONTROL_ADDRESS(icreg),         \
                        TME_PH1_CONTROL_ADDRESS(icreg)          \
                        + sizeof(ph1->icreg) - 1))

  if (cycle_init->tme_bus_cycle_type == TME_BUS_CYCLE_READ) {

    /* the dd7seg register is write-only, so we provide data pins all
       pulled-up on read.  */
    if (_TME_PH1_REG_ACCESSED(tme_ph1_dd7seg_u)
        || _TME_PH1_REG_ACCESSED(tme_ph1_dd7seg_l)) {
      ph1->tme_ph1_dd7seg_u = 0xff;
      ph1->tme_ph1_dd7seg_l = 0xff;
    }

    /* the cfgsw register is read-only, so we need to fetch the real
       value for reads.  */
    if (_TME_PH1_REG_ACCESSED(tme_ph1_cfgsw)) {
      ph1->tme_ph1_cfgsw = ph1->tme_ph1_cfgsw_value;
    }

    /* reads of either the intrset or intrclr register return the
       current value of the swint_pending internal register.  */
    if (_TME_PH1_REG_ACCESSED(tme_ph1_intrset)) {
      ph1->tme_ph1_intrset = ph1->tme_ph1_swint_pending;
    }
    if (_TME_PH1_REG_ACCESSED(tme_ph1_intrclr)) {
      ph1->tme_ph1_intrclr = ph1->tme_ph1_swint_pending;
    }

    /* the buserror register is read-only, and it is reset to 0 when
       it is read.  */
    if (_TME_PH1_REG_ACCESSED(tme_ph1_buserror)) {
      ph1->tme_ph1_buserror = ph1->tme_ph1_buserror_value;
      ph1->tme_ph1_buserror_value = 0;
    }
  }

  /* whenever the segmap register is accessed, we need to fill it
     before running the cycle:  */
  if (_TME_PH1_REG_ACCESSED(tme_ph1_segmap0)) {
    ph1->tme_ph1_segmap0 = _tme_ph1_mmu_sme_get(ph1,
                                                0,
                                                address);
  }
  if (_TME_PH1_REG_ACCESSED(tme_ph1_segmap)) {
    ph1->tme_ph1_segmap = _tme_ph1_mmu_sme_get(ph1,
                                               ph1->tme_ph1_context,
                                               address);
  }

  /* whenever the pagemap register is accessed, we need to fill it
     before running the cycle:  */
  if (_TME_PH1_REG_ACCESSED(tme_ph1_pagemap_u)
      || _TME_PH1_REG_ACCESSED(tme_ph1_pagemap_l)) {
    pme = _tme_ph1_mmu_pme_get(ph1, address);
    ph1->tme_ph1_pagemap_u = (pme >> 16);
    ph1->tme_ph1_pagemap_l = (pme & 0xffff);
  }

  /* run the cycle: */
  TME_PH1_CONTROL_BUS_CYCLE(ph1, reg, &cycle_resp);
  cycle_resp.tme_bus_cycle_type = (cycle_init->tme_bus_cycle_type
                                   ^ (TME_BUS_CYCLE_WRITE
                                      | TME_BUS_CYCLE_READ));
  cycle_resp.tme_bus_cycle_lane_routing = cycle_init->tme_bus_cycle_lane_routing;
  tme_bus_cycle_xfer(cycle_init, &cycle_resp);

  /* take action when these registers are written: */
  if (cycle_init->tme_bus_cycle_type == TME_BUS_CYCLE_WRITE) {

    if (_TME_PH1_REG_ACCESSED(tme_ph1_dd7seg_u)
        || _TME_PH1_REG_ACCESSED(tme_ph1_dd7seg_l)) {

      if (_TME_PH1_REG_ACCESSED(tme_ph1_dd7seg_u)) {
        ph1->tme_ph1_dd7seg_upper_value = ph1->tme_ph1_dd7seg_u;
      }
      if (_TME_PH1_REG_ACCESSED(tme_ph1_dd7seg_l)) {
        ph1->tme_ph1_dd7seg_lower_value = ph1->tme_ph1_dd7seg_l;
      }
      _tme_ph1_dd7seg_update_display(ph1);
    }

    if (_TME_PH1_REG_ACCESSED(tme_ph1_intrset)) {
      ph1->tme_ph1_swint_pending |= (ph1->tme_ph1_intrset & TME_PH1_SWINT_MASK);
      needs_ipl_check = TRUE;
    }
    if (_TME_PH1_REG_ACCESSED(tme_ph1_intrclr)) {
      ph1->tme_ph1_swint_pending &= ~(ph1->tme_ph1_intrclr & TME_PH1_SWINT_MASK);
      needs_ipl_check = TRUE;
    }

    if (_TME_PH1_REG_ACCESSED(tme_ph1_sysen)) {
      /* If the interrupt enable bit changed, an IPL check is needed. */
      if ((ph1->tme_ph1_sysen ^ old_sysen_value) & TME_PH1_SYSEN_INT) {
        needs_ipl_check = TRUE;
      }

      /* If the MMU enable bit changed, force-reload the context. */
      if ((ph1->tme_ph1_sysen ^ old_sysen_value) & TME_PH1_SYSEN_MMU) {
        _tme_ph1_mmu_context_set(ph1);
      }
    }

    if (_TME_PH1_REG_ACCESSED(tme_ph1_segmap0)) {
      _tme_ph1_mmu_sme_set(ph1,
                           0,
                           address,
                           ph1->tme_ph1_segmap0);
    }
    if (_TME_PH1_REG_ACCESSED(tme_ph1_segmap)) {
      _tme_ph1_mmu_sme_set(ph1,
                           ph1->tme_ph1_context,
                           address,
                           ph1->tme_ph1_segmap);
    }

    if (_TME_PH1_REG_ACCESSED(tme_ph1_context)) {
      ph1->tme_ph1_context &= (TME_PGMMU_NUM_CONTEXTS - 1);
      _tme_ph1_mmu_context_set(ph1);
    }

    if (_TME_PH1_REG_ACCESSED(tme_ph1_pagemap_u)
        || _TME_PH1_REG_ACCESSED(tme_ph1_pagemap_l)) {
      pme = ph1->tme_ph1_pagemap_u;
      pme = (pme << 16) | ph1->tme_ph1_pagemap_l;
      _tme_ph1_mmu_pme_set(ph1, address, pme);
    }

    if (needs_ipl_check) {
      rc = _tme_ph1_ipl_check(ph1);
      assert(rc == TME_OK);
      (void)rc;
    }
  }

  return (TME_OK);
}
