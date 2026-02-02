/* machine/pg68k/pg68k-mmu.c - 68K Playround MMU emulation: */

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
#include <tme/machine/pg68k.h>

/* macros: */
#define TME_PGMMU_PMEG_TLBS     (16)

/* structures: */

/* an allocated TLB set in a pg68k MMU: */
struct tme_pg68k_mmu_tlb_set {

  /* the next allocated TLB set: */
  struct tme_pg68k_mmu_tlb_set *tme_pg68k_mmu_tlb_set_next;

  /* the TLB set information: */
  struct tme_bus_tlb_set_info tme_pg68k_mmu_tlb_set_info;
};

/* one PMEG in a pg68k MMU: */
struct tme_pg68k_mmu_pmeg {

  /* the current list of TLBs using a page table entry in this PMEG, and
     the head within that list: */
  struct tme_token *tme_pg68k_mmu_pmeg_tlb_tokens[TME_PGMMU_PMEG_TLBS];
  unsigned int tme_pg68k_mmu_pmeg_tlbs_head;
};

/* the private structure for the pg68k MMU: */
struct tme_pg68k_mmu {

  /* the information provided by the encapsulating system: */
  struct tme_pg68k_mmu_info tme_pg68k_mmu_info;
#define tme_pg68k_mmu_element \
  tme_pg68k_mmu_info.tme_pg68k_mmu_info_element
#define tme_pg68k_mmu_address_bits \
  tme_pg68k_mmu_info.tme_pg68k_mmu_info_address_bits
#define tme_pg68k_mmu_pgoffset_bits \
  tme_pg68k_mmu_info.tme_pg68k_mmu_info_pgoffset_bits
#define tme_pg68k_mmu_pmeg_offset_bits \
  tme_pg68k_mmu_info.tme_pg68k_mmu_info_pmeg_offset_bits
#define tme_pg68k_mmu_num_contexts \
  tme_pg68k_mmu_info.tme_pg68k_mmu_info_num_contexts
#define tme_pg68k_mmu_num_pmegs \
  tme_pg68k_mmu_info.tme_pg68k_mmu_info_num_pmegs
#define tme_pg68k_mmu_tlb_fill_phys_private \
  tme_pg68k_mmu_info.tme_pg68k_mmu_info_tlb_fill_phys_private
#define tme_pg68k_mmu_tlb_fill_phys \
  tme_pg68k_mmu_info.tme_pg68k_mmu_info_tlb_fill_phys
#define tme_pg68k_mmu_invalid_private \
  tme_pg68k_mmu_info.tme_pg68k_mmu_info_invalid_private
#define tme_pg68k_mmu_invalid \
  tme_pg68k_mmu_info.tme_pg68k_mmu_info_invalid
#define tme_pg68k_mmu_priv_private \
  tme_pg68k_mmu_info.tme_pg68k_mmu_info_priv_private
#define tme_pg68k_mmu_priv \
  tme_pg68k_mmu_info.tme_pg68k_mmu_info_priv
#define tme_pg68k_mmu_prot_private \
  tme_pg68k_mmu_info.tme_pg68k_mmu_info_prot_private
#define tme_pg68k_mmu_prot \
  tme_pg68k_mmu_info.tme_pg68k_mmu_info_prot

  /* the number of bits in a segment offset: */
  tme_uint8_t tme_pg68k_mmu_segoffset_bits;

  /* the number of bits in the segment map index: */
  tme_uint8_t tme_pg68k_mmu_segment_bits;

  /* the segment map: */
  tme_uint16_t *tme_pg68k_mmu_segmap;
  struct tme_token **tme_pg68k_mmu_segmap_tlb_tokens;

  /* the PMEGs: */
  struct tme_pg68k_mmu_pmeg *tme_pg68k_mmu_pmegs;

  /* the pagemap: */
  tme_uint32_t *tme_pg68k_mmu_pagemap;

  /* the allocated TLB sets: */
  struct tme_pg68k_mmu_tlb_set *tme_pg68k_mmu_tlb_sets;
};

/* this creates a pg68k MMU: */
void *
tme_pg68k_mmu_new(struct tme_pg68k_mmu_info *info)
{
  struct tme_pg68k_mmu *mmu;
  unsigned int map_count;

  /* allocate the new private structure: */
  mmu = tme_new0(struct tme_pg68k_mmu, 1);

  /* copy the client-provided information: */
  mmu->tme_pg68k_mmu_info = *info;

  /* calculate some values derived from provided information: */
  mmu->tme_pg68k_mmu_segoffset_bits
    = mmu->tme_pg68k_mmu_pgoffset_bits + mmu->tme_pg68k_mmu_pmeg_offset_bits;
  mmu->tme_pg68k_mmu_segment_bits
    = mmu->tme_pg68k_mmu_address_bits - mmu->tme_pg68k_mmu_segoffset_bits;

  /* allocate the segment map: */
  map_count = (mmu->tme_pg68k_mmu_num_contexts
               * (1 << mmu->tme_pg68k_mmu_segment_bits));
  mmu->tme_pg68k_mmu_segmap = tme_new0(tme_uint16_t, map_count);
  /* XXX should initialize with junk */

  mmu->tme_pg68k_mmu_segmap_tlb_tokens
    = tme_new0(struct tme_token *, map_count);

  /* allocate the PMEGs: */
  mmu->tme_pg68k_mmu_pmegs
    = tme_new0(struct tme_pg68k_mmu_pmeg, mmu->tme_pg68k_mmu_num_pmegs);

  /* allocate the page map: */
  map_count = (mmu->tme_pg68k_mmu_num_pmegs
               * (1 << mmu->tme_pg68k_mmu_pmeg_offset_bits));
  mmu->tme_pg68k_mmu_pagemap = tme_new0(tme_uint32_t, map_count);
  /* XXX should initialize with junk */

  /* done: */
  return (mmu);
}

/* given a context and an address, returns indices for the segmap and,
   if the segment is valid, page map entries.  */
int
tme_pg68k_mmu_lookup(void *_mmu,
                     tme_uint8_t context,
                     tme_uint32_t address,
                     unsigned int *sme_indexp,
                     unsigned int *pme_indexp)
{
  unsigned int segment;
  unsigned int pme_index;
  unsigned int sme_index;
  unsigned int sme;

  struct tme_pg68k_mmu *mmu = _mmu;

  /* lose the page offset bits: */
  address >>= mmu->tme_pg68k_mmu_pgoffset_bits;
  
  /* get the index of the PME within the PMEG */
  pme_index = (address
               & (TME_BIT(mmu->tme_pg68k_mmu_pmeg_offset_bits) - 1));
  address >>= mmu->tme_pg68k_mmu_pmeg_offset_bits;

  /* get the segment number */
  segment = (address
             & (TME_BIT(mmu->tme_pg68k_mmu_segment_bits) - 1));

  /* get the segment map index */
  sme_index = ((context << mmu->tme_pg68k_mmu_segment_bits)
               | segment);
  if (sme_indexp) {
    *sme_indexp = sme_index;
  }

  /* get the segment map entry */
  sme = mmu->tme_pg68k_mmu_segmap[sme_index];
  if ((sme & TME_PGMMU_SME_V) == 0) {
    return (EINVAL);
  } else {
    if (pme_indexp) {
      pme_index += ((sme & TME_PGMMU_SME_PMEG_MASK)
                    << mmu->tme_pg68k_mmu_pmeg_offset_bits);
      *pme_indexp = pme_index;
    }
    return (TME_OK);
  }
}

/* this invalidates all TLB entries that may be affected by changes to
   a PMEG:  */
static void
tme_pg68k_mmu_pmeg_invalidate(struct tme_pg68k_mmu *mmu,
                              unsigned int pmeg_index)
{
  struct tme_pg68k_mmu_pmeg *pmeg;
  int tlb_i;
  struct tme_token *token;

  pmeg = mmu->tme_pg68k_mmu_pmegs + pmeg_index;

  /* invalidate all of the TLBs: */
  for (tlb_i = 0; tlb_i < TME_PGMMU_PMEG_TLBS; tlb_i++) {
    token = pmeg->tme_pg68k_mmu_pmeg_tlb_tokens[tlb_i];
    pmeg->tme_pg68k_mmu_pmeg_tlb_tokens[tlb_i] = NULL;
    if (token != NULL) {
      tme_token_invalidate(token);
    }
  }
}

/* this invalidates all TLB entries that may be affected by changes to
   a segmap entry.  these will be "page invalid" entries that would have
   been filled when a segmap entry was invalid.  */
static void
tme_pg68k_mmu_segment_invalidate(struct tme_pg68k_mmu *mmu,
                                 unsigned int sme_index)
{
  struct tme_token *token;

  token = mmu->tme_pg68k_mmu_segmap_tlb_tokens[sme_index];
  mmu->tme_pg68k_mmu_segmap_tlb_tokens[sme_index] = NULL;
  if (token != NULL) {
    tme_token_invalidate(token);
  }
}

#define CLAMP_PME_INDEX(m, i)                                     \
  ((i) & ((((m)->tme_pg68k_mmu_num_pmegs - 1)                     \
           << (m)->tme_pg68k_mmu_pmeg_offset_bits)                \
          | (TME_BIT((m)->tme_pg68k_mmu_pmeg_offset_bits) - 1)))

/* this gets a PME: */
tme_uint32_t
tme_pg68k_mmu_pme_get(void *_mmu,
                      unsigned int pme_index)
{
  struct tme_pg68k_mmu *mmu;

  mmu = (struct tme_pg68k_mmu *) _mmu;
  pme_index = CLAMP_PME_INDEX(mmu, pme_index);
  return (mmu->tme_pg68k_mmu_pagemap[pme_index]);
}

/* this sets a PME: */
void
tme_pg68k_mmu_pme_set(void *_mmu,
                      unsigned int pme_index,
                      tme_uint32_t pme)
{
  struct tme_pg68k_mmu *mmu;

  mmu = (struct tme_pg68k_mmu *) _mmu;
  pme_index = CLAMP_PME_INDEX(mmu, pme_index);

  /* invalidate all TLB entries that are affected by changes to this PMEG: */
  tme_pg68k_mmu_pmeg_invalidate(mmu,
                              pme_index >> mmu->tme_pg68k_mmu_pmeg_offset_bits);

  mmu->tme_pg68k_mmu_pagemap[pme_index] = pme;
}

#undef CLAMP_PME_INDEX

/* this gets a segmap entry */
tme_uint16_t
tme_pg68k_mmu_sme_get(void *_mmu,
                      tme_uint8_t context,
                      tme_uint32_t address)
{
  struct tme_pg68k_mmu *mmu;
  unsigned int sme_index;
  tme_uint16_t sme;

  mmu = (struct tme_pg68k_mmu *) _mmu;
  (void) tme_pg68k_mmu_lookup(mmu, context, address, &sme_index, NULL);
  sme = mmu->tme_pg68k_mmu_segmap[sme_index];
  tme_log(&mmu->tme_pg68k_mmu_element->tme_element_log_handle, 1000, TME_OK,
          (&mmu->tme_pg68k_mmu_element->tme_element_log_handle,
           "sme_get: SEGMAP[%u:0x%08x] -> 0x%04x",
           context,
           address,
           sme));
  return (sme);
}

/* this sets a segmap entry */
void
tme_pg68k_mmu_sme_set(void *_mmu,
                      tme_uint8_t context,
                      tme_uint32_t address,
                      tme_uint16_t sme)
{
  struct tme_pg68k_mmu *mmu;
  unsigned int sme_index;
  tme_uint16_t old_sme;

  mmu = (struct tme_pg68k_mmu *) _mmu;
  (void) tme_pg68k_mmu_lookup(mmu, context, address, &sme_index, NULL);

  old_sme = mmu->tme_pg68k_mmu_segmap[sme_index];

  /* if old segmap entry was valid, invalidate all TLB entries that are
     affected by changes to this PMEG - losing a spot in the segment map
     counts as such a change: */
  if (old_sme & TME_PGMMU_SME_V) {
    tme_pg68k_mmu_pmeg_invalidate(mmu, old_sme & TME_PGMMU_SME_PMEG_MASK);
  }

  /* otherwise, the old segment was invalid and may have negative TLB
     entries at the segment level; if the new segment is valid, then
     we must invalidate those: */
  else if (sme & TME_PGMMU_SME_V) {
    tme_pg68k_mmu_segment_invalidate(mmu, sme_index);
  }

  mmu->tme_pg68k_mmu_segmap[sme_index] = sme;
  tme_log(&mmu->tme_pg68k_mmu_element->tme_element_log_handle, 1000, TME_OK,
          (&mmu->tme_pg68k_mmu_element->tme_element_log_handle,
           "sme_get: SEGMAP[%u:0x%08x] <- 0x%04x",
           context,
           address,
           sme));
}

/* this fills a TLB entry: */
unsigned int
tme_pg68k_mmu_tlb_fill(void *_mmu,
                       struct tme_bus_tlb *tlb,
                       tme_uint8_t context,
                       tme_uint32_t address,
                       unsigned int cycles,
                       int is_super)
{
  struct tme_pg68k_mmu *mmu;
  struct tme_pg68k_mmu_pmeg *pmeg;
  struct tme_token *token_old;
  struct tme_bus_tlb tlb_virtual;
  unsigned int sme_index;
  unsigned int pme_index;
  tme_bus_addr32_t addr_first, addr_last;
  tme_uint32_t physical_address;
  tme_uint16_t sme;
  tme_uint32_t pme;
  unsigned int tlb_i;
  int rc;

  /* lookup this address: */
  mmu = (struct tme_pg68k_mmu *) _mmu;
  rc = tme_pg68k_mmu_lookup(mmu, context, address, &sme_index, &pme_index);

  /* if the lookup failed, it means we have an invalid segment.  in this
     case, we need to fill an invalid TLB entry that covers the entire
     segment.  */
  if (__tme_predict_false(rc != TME_OK)) {
    token_old = mmu->tme_pg68k_mmu_segmap_tlb_tokens[sme_index];
    if (token_old != NULL
        && token_old != tlb->tme_bus_tlb_token) {
      tme_token_invalidate(token_old);
    }
    mmu->tme_pg68k_mmu_segmap_tlb_tokens[sme_index] = tlb->tme_bus_tlb_token;
    addr_first = (address & ~(TME_BIT(mmu->tme_pg68k_mmu_segoffset_bits) - 1));
    addr_last = (address | (TME_BIT(mmu->tme_pg68k_mmu_segoffset_bits) - 1));
    goto fill_invalid_entry;
  }

  /* round the address to a page: */
  addr_first = (address & ~(TME_BIT(mmu->tme_pg68k_mmu_pgoffset_bits) - 1));
  addr_last = (address | (TME_BIT(mmu->tme_pg68k_mmu_pgoffset_bits) - 1));

  /* remember this TLB entry in the PMEG: */
  sme = mmu->tme_pg68k_mmu_segmap[sme_index];
  pmeg = mmu->tme_pg68k_mmu_pmegs + (sme & TME_PGMMU_SME_PMEG_MASK);
  tlb_i = pmeg->tme_pg68k_mmu_pmeg_tlbs_head;
  token_old = pmeg->tme_pg68k_mmu_pmeg_tlb_tokens[tlb_i];
  if (token_old != NULL
      && token_old != tlb->tme_bus_tlb_token) {
    tme_token_invalidate(token_old);
  }
  pmeg->tme_pg68k_mmu_pmeg_tlb_tokens[tlb_i]
    = tlb->tme_bus_tlb_token;
  pmeg->tme_pg68k_mmu_pmeg_tlbs_head = (tlb_i + 1) & (TME_PGMMU_PMEG_TLBS - 1);

  /* if this page is invalid, return the page-invalid cycle handler,
     which is valid for reading and writing: */
  pme = mmu->tme_pg68k_mmu_pagemap[pme_index];
  if ((pme & TME_PGMMU_PME_V) == 0) {
    fill_invalid_entry:
    tme_bus_tlb_initialize(tlb);
    tlb->tme_bus_tlb_addr_first = addr_first;
    tlb->tme_bus_tlb_addr_last = addr_last;
    tlb->tme_bus_tlb_cycles_ok = TME_BUS_CYCLE_READ | TME_BUS_CYCLE_WRITE;
    tlb->tme_bus_tlb_cycle_private =
                     mmu->tme_pg68k_mmu_info.tme_pg68k_mmu_info_invalid_private;
    tlb->tme_bus_tlb_cycle = mmu->tme_pg68k_mmu_info.tme_pg68k_mmu_info_invalid;
    return (TME_PGMMU_TLB_SYSTEM | TME_PGMMU_TLB_USER);
  }

  /* if the page requires supervisor privilege and it's not the supervisor
     performing the access, then return the privilege-violation cycle handler,
     which is valid for reading and writing: */
  if ((pme & TME_PGMMU_PME_K) != 0 && !is_super) {
    tme_bus_tlb_initialize(tlb);
    tlb->tme_bus_tlb_addr_first = addr_first;
    tlb->tme_bus_tlb_addr_last = addr_last;
    tlb->tme_bus_tlb_cycles_ok = TME_BUS_CYCLE_READ | TME_BUS_CYCLE_WRITE;
    tlb->tme_bus_tlb_cycle_private =
                        mmu->tme_pg68k_mmu_info.tme_pg68k_mmu_info_priv_private;
    tlb->tme_bus_tlb_cycle = mmu->tme_pg68k_mmu_info.tme_pg68k_mmu_info_priv;
    return (TME_PGMMU_TLB_USER);
  }

  /* if the page is not writable and the access is a write cycle, then
     return the protection-error cycle handler, which is valid only for
     writing: */
  if ((pme & TME_PGMMU_PME_W) == 0 && (cycles & TME_BUS_CYCLE_WRITE) != 0) {
    tme_bus_tlb_initialize(tlb);
    tlb->tme_bus_tlb_addr_first = addr_first;
    tlb->tme_bus_tlb_addr_last = addr_last;
    tlb->tme_bus_tlb_cycles_ok = TME_BUS_CYCLE_WRITE;
    tlb->tme_bus_tlb_cycle_private =
                        mmu->tme_pg68k_mmu_info.tme_pg68k_mmu_info_prot_private;
    tlb->tme_bus_tlb_cycle = mmu->tme_pg68k_mmu_info.tme_pg68k_mmu_info_prot;
    return (TME_PGMMU_TLB_SYSTEM | TME_PGMMU_TLB_USER);
  }

  /* this access is OK.  fill the TLB with physical bus information.
     we pass in the virtual address as the initial physical address
     because maybe the virtual part of the address van influence the
     physical address: */
  physical_address = address;
  (*mmu->tme_pg68k_mmu_tlb_fill_phys)
    (mmu->tme_pg68k_mmu_tlb_fill_phys_private,
     tlb,
     pme,
     &physical_address,
     cycles);

  /* create the mapping TLB entry, and update the PME flags: */
  tlb_virtual.tme_bus_tlb_addr_first = addr_first;
  tlb_virtual.tme_bus_tlb_addr_last = addr_last;
  tlb_virtual.tme_bus_tlb_cycles_ok = TME_BUS_CYCLE_READ;
  pme |= TME_PGMMU_PME_REF;
  if (cycles & TME_BUS_CYCLE_WRITE) {
    pme |= TME_PGMMU_PME_MOD;
  }
  if ((pme & (TME_PGMMU_PME_W|TME_PGMMU_PME_MOD))
      == (TME_PGMMU_PME_W|TME_PGMMU_PME_MOD)) {
    tlb_virtual.tme_bus_tlb_cycles_ok |= TME_BUS_CYCLE_WRITE;
  }
  mmu->tme_pg68k_mmu_pagemap[pme_index] = pme;

  /* map the filled TLB entry: */
  tme_bus_tlb_map(tlb, physical_address, &tlb_virtual, address);

  /* valid pages can always be accessed by the supervisor, but
     kernel-privelege pages cannot be accessed by the user.  */
  return (TME_PGMMU_TLB_SYSTEM
          | ((pme & TME_PGMMU_PME_K) == 0
             ? TME_PGMMU_TLB_USER
             : 0));
}

/* this invalidates all TLB entries in all TLB sets: */
void
tme_pg68k_mmu_tlbs_invalidate(void *_mmu)
{
  struct tme_pg68k_mmu *mmu;
  struct tme_pg68k_mmu_tlb_set *tlb_set;

  /* recover our MMU: */
  mmu = (struct tme_pg68k_mmu *) _mmu;

  /* invalidate all TLB entries in all sets: */
  for (tlb_set = mmu->tme_pg68k_mmu_tlb_sets;
       tlb_set != NULL;
       tlb_set = tlb_set->tme_pg68k_mmu_tlb_set_next) {
    tme_bus_tlb_set_invalidate(&tlb_set->tme_pg68k_mmu_tlb_set_info);
  }
}

/* this adds a new TLB set: */
int
tme_pg68k_mmu_tlb_set_add(void *_mmu,
                          struct tme_bus_tlb_set_info *tlb_set_info)
{
  struct tme_pg68k_mmu *mmu;
  struct tme_pg68k_mmu_tlb_set *tlb_set;

  /* recover our MMU: */
  mmu = (struct tme_pg68k_mmu *) _mmu;

  /* remember this set: */
  tlb_set = tme_new0(struct tme_pg68k_mmu_tlb_set, 1);
  tlb_set->tme_pg68k_mmu_tlb_set_next = mmu->tme_pg68k_mmu_tlb_sets;
  tlb_set->tme_pg68k_mmu_tlb_set_info = *tlb_set_info;
  mmu->tme_pg68k_mmu_tlb_sets = tlb_set;

  return (TME_OK);
}
