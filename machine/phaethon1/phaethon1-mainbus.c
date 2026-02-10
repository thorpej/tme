/* machine/phaethon1/phaethon1-mainbus.c - implementation of
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

#include <tme/common.h>

/* includes: */
#include "phaethon1-impl.h"
#include <tme/machine/pg68k.h>
#include <tme/ata/ata-controller.h>
#include <tme/ic/tl16c550.h>

/* this possibly updates that the interrupt priority level driven to the CPU: */
int
_tme_ph1_ipl_check(struct tme_ph1 *ph1)
{
  tme_uint8_t sysen;
  tme_uint8_t swint;
  unsigned int ipl, ipl_index;
  tme_uint8_t ipl_mask;

  /* get the system enable register: */
  sysen = ph1->tme_ph1_sysen;

  /* get the swint pending register: */
  swint = ph1->tme_ph1_swint_pending;

  /* assume that interrupts are completely masked: */
  ipl = TME_M68K_IPL_NONE;

  /* if interrupts are enabled: */
  if (sysen & TME_PH1_SYSEN_INT) {

    /* find the highest ipl now asserted on the buses: */
    for (ipl = TME_M68K_IPL_MAX;
         ipl > TME_M68K_IPL_NONE;
         ipl--) {
      ipl_index = ipl >> 3;
      ipl_mask = TME_BIT(ipl & 7);
      if (ph1->tme_ph1_int_signals[ipl_index] & ipl_mask) {
        break;
      }
    }

    /* check the soft interrupts: */
    if (swint & TME_PH1_SWINT_IPL2) {
      ipl = TME_MAX(ipl, 2);
    }
    else if (swint & TME_PH1_SWINT_IPL1) {
      ipl = TME_MAX(ipl, 1);
    }
  }

  /* possibly update the CPU: */
  if (ipl != ph1->tme_ph1_int_ipl_last) {
    ph1->tme_ph1_int_ipl_last = ipl;
    return ((*ph1->tme_ph1_m68k->tme_m68k_bus_interrupt)
            (ph1->tme_ph1_m68k, ipl));
  }
  return (TME_OK);
}

/* our mainbus signal handler: */
static int
_tme_ph1_bus_signal(struct tme_bus_connection *conn_bus_raiser,
                    unsigned int signal)
{
  struct tme_ph1 *ph1;
  int signal_asserted;
  unsigned int ipl, ipl_index;
  tme_uint8_t ipl_mask;

  /* recover our ph1: */
  ph1 = (struct tme_ph1 *) conn_bus_raiser->tme_bus_connection.tme_connection_element->tme_element_private;

  /* see whether the signal is asserted or negated: */
  signal_asserted = TRUE;
  switch (signal & TME_BUS_SIGNAL_LEVEL_MASK) {
  case TME_BUS_SIGNAL_LEVEL_NEGATED:
    signal_asserted = FALSE;
    break;
  case TME_BUS_SIGNAL_LEVEL_ASSERTED:
    break;
  default:
    abort();
  }
  signal = TME_BUS_SIGNAL_WHICH(signal);

  /* dispatch on the signal: */

  /* halt: */
  if (signal == TME_BUS_SIGNAL_HALT) {
    abort();
  }

  /* reset: */
  else if (signal == TME_BUS_SIGNAL_RESET) {
    /* XXX reset is just ignored for now: */
  }

  /* an interrupt signal: */
  else if (TME_BUS_SIGNAL_IS_INT(signal)) {
    ipl = TME_BUS_SIGNAL_INDEX_INT(signal);
    if (ipl >= TME_M68K_IPL_MIN
        && ipl <= TME_M68K_IPL_MAX) {

      /* update this ipl in the byte array: */
      ipl_index = ipl >> 3;
      ipl_mask = TME_BIT(ipl & 7);
      ph1->tme_ph1_int_signals[ipl_index]
        = ((ph1->tme_ph1_int_signals[ipl_index]
            & ~ipl_mask)
           | (signal_asserted
              ? ipl_mask
              : 0));

      /* possibly update the ipl being driven to the CPU: */
      return (_tme_ph1_ipl_check(ph1));
    }
  }

  /* an unknown signal: */
  else {
    abort();
  }

  return (TME_OK);
}

/* this handles a CPU interrupt acknowledge: */
static int
_tme_ph1_bus_intack(struct tme_bus_connection *conn_m68k,
                    unsigned int ipl,
                    int *vector)
{
  /* There is no need to consult any of the busses or check the swint_pending
     register, because the Phaethon 1's interrupt controller only supports
     auto-vector interrupts and IACK cycles are never sent to the VMEbus.  */
  *vector = TME_BUS_INTERRUPT_VECTOR_UNDEF;
  return (TME_OK);
}

/* our command function: */
static int
_tme_ph1_command(struct tme_element *element,
                 const char * const * args,
                 char **_output)
{
  struct tme_ph1 *ph1;
  int do_reset;

  /* recover our ph1: */
  ph1 = (struct tme_ph1 *) element->tme_element_private;

  /* assume no reset: */
  do_reset = FALSE;

  /* the "power" command: */
  if (TME_ARG_IS(args[1], "power")) {

    if (TME_ARG_IS(args[2], "up")
        && args[3] == NULL) {
      do_reset = TRUE;
    }

    else if (TME_ARG_IS(args[2], "down")
             && args[3] == NULL) {
      /* nothing */
    }

    /* return an error: */
    else {
      tme_output_append_error(_output,
                              "%s %s power [ up | down ]",
                              _("usage:"),
                              args[0]);
      return (EINVAL);
    }
  }

  /* any other command: */
  else {
    if (args[1] != NULL) {
      tme_output_append_error(_output,
                               "%s '%s', ",
                               _("unknown command"),
                               args[1]);
    }
    tme_output_append_error(_output,
                            _("available %s commands: %s"),
                            args[0],
                            "power");
    return (EINVAL);
  }

  /* reset the CPU: */
  if (do_reset) {
    (*ph1->tme_ph1_m68k->tme_m68k_bus_connection.tme_bus_signal)
      (&ph1->tme_ph1_m68k->tme_m68k_bus_connection,
       TME_BUS_SIGNAL_RESET
       | TME_BUS_SIGNAL_LEVEL_NEGATED
       | TME_BUS_SIGNAL_EDGE);

    /* reset all busses: */
    (*ph1->tme_ph1_ram->tme_bus_signal)
      (ph1->tme_ph1_ram,
       TME_BUS_SIGNAL_RESET
       | TME_BUS_SIGNAL_LEVEL_NEGATED
       | TME_BUS_SIGNAL_EDGE);

    (*ph1->tme_ph1_rom->tme_bus_signal)
      (ph1->tme_ph1_rom,
       TME_BUS_SIGNAL_RESET
       | TME_BUS_SIGNAL_LEVEL_NEGATED
       | TME_BUS_SIGNAL_EDGE);

    (*ph1->tme_ph1_obio->tme_bus_signal)
      (ph1->tme_ph1_obio,
       TME_BUS_SIGNAL_RESET
       | TME_BUS_SIGNAL_LEVEL_NEGATED
       | TME_BUS_SIGNAL_EDGE);

    (*ph1->tme_ph1_vme->tme_bus_signal)
      (ph1->tme_ph1_vme,
       TME_BUS_SIGNAL_RESET
       | TME_BUS_SIGNAL_LEVEL_NEGATED
       | TME_BUS_SIGNAL_EDGE);
  }

  return (TME_OK);
}

/* the connection scorer: */
static int
_tme_ph1_connection_score(struct tme_connection *conn,
                          unsigned int *_score)
{
  struct tme_m68k_bus_connection *conn_m68k;
  struct tme_ph1_bus_connection *conn_ph1;
  struct tme_bus_connection *conn_bus;
  struct tme_ph1 *ph1;
  unsigned int score;

  /* recover our ph1: */
  ph1 = (struct tme_ph1 *) conn->tme_connection_element->tme_element_private;

  /* assume that this connection is useless: */
  score = 0;

  /* dispatch on the connection type: */
  conn_m68k = (struct tme_m68k_bus_connection *) conn->tme_connection_other;
  conn_ph1 = (struct tme_ph1_bus_connection *) conn;
  conn_bus = (struct tme_bus_connection *) conn->tme_connection_other;

  switch (conn->tme_connection_type) {
    /* this must be an m68k chip, and not another bus: */
  case TME_CONNECTION_BUS_M68K:
    if (conn_bus->tme_bus_tlb_set_add == NULL
        && conn_m68k->tme_m68k_bus_tlb_fill == NULL) {
      score = 10;
    }
    break;

    /* this must be a bus, and not a chip: */
  case TME_CONNECTION_BUS_GENERIC:
    if (conn_bus->tme_bus_tlb_set_add != NULL
        && conn_bus->tme_bus_tlb_fill != NULL
        && ph1->tme_ph1_buses[conn_ph1->tme_ph1_bus_connection_which] == NULL) {
      score = 1;
    }
    break;

  default:  abort();
  }

  *_score = score;
  return (TME_OK);
}

/* this makes a new connection: */
static int
_tme_ph1_connection_make(struct tme_connection *conn,
                         unsigned int state)
{
  struct tme_ph1 *ph1;
  struct tme_m68k_bus_connection *conn_m68k;
  struct tme_ph1_bus_connection *conn_ph1;
  struct tme_bus_connection *conn_bus;
  struct tme_connection *conn_other;

  /* recover our ph1: */
  ph1 = (struct tme_ph1 *) conn->tme_connection_element->tme_element_private;

  /* dispatch on the connection type: */
  conn_other = conn->tme_connection_other;
  conn_m68k = (struct tme_m68k_bus_connection *) conn_other;
  conn_ph1 = (struct tme_ph1_bus_connection *) conn;
  conn_bus = (struct tme_bus_connection *) conn_other;

  switch (conn->tme_connection_type) {
  case TME_CONNECTION_BUS_M68K:
    ph1->tme_ph1_m68k = conn_m68k;
    break;

  case TME_CONNECTION_BUS_GENERIC:
    ph1->tme_ph1_buses[conn_ph1->tme_ph1_bus_connection_which] = conn_bus;
    break;

  default:  assert(FALSE);
  }
  return (TME_OK);
}

/* this breaks a connection: */
static int
_tme_ph1_connection_break(struct tme_connection *conn,
                          unsigned int state)
{
  abort();
}

/* this makes new connection sides: */
static int
_tme_ph1_connections_new(struct tme_element *element,
                         const char * const *args,
                         struct tme_connection **_conns,
                         char **_output)
{
  struct tme_m68k_bus_connection *conn_m68k;
  struct tme_ph1_bus_connection *conn_ph1;
  struct tme_bus_connection *conn_bus;
  struct tme_connection *conn;
  struct tme_ph1 *ph1;
  char *free_buses;
  int which_bus;

  /* recover our ph1: */
  ph1 = (struct tme_ph1 *) element->tme_element_private;

  /* if we have no arguments and don't have a CPU yet,
     we can take an m68k connection:  */
  if (args[1] == NULL
      && ph1->tme_ph1_m68k == NULL) {

    /* create our side of an m68k bus connection: */
    conn_m68k = tme_new0(struct tme_m68k_bus_connection, 1);
    conn_bus = &conn_m68k->tme_m68k_bus_connection;
    conn = &conn_bus->tme_bus_connection;

    /* fill in the generic connection: */
    conn->tme_connection_next = *_conns;
    conn->tme_connection_type = TME_CONNECTION_BUS_M68K;
    conn->tme_connection_score = _tme_ph1_connection_score;
    conn->tme_connection_make = _tme_ph1_connection_make;
    conn->tme_connection_break = _tme_ph1_connection_break;

    /* fill in the generic bus connection: */
    conn_bus->tme_bus_signal = _tme_ph1_bus_signal;
    conn_bus->tme_bus_intack = _tme_ph1_bus_intack;
    conn_bus->tme_bus_tlb_set_add = _tme_ph1_mmu_tlb_set_add;

    /* fill in the m68k bus connection: */
    conn_m68k->tme_m68k_bus_tlb_fill = _tme_ph1_m68k_tlb_fill;

    /* add in this connection side possibility: */
    *_conns = conn;
  }

  /* otherwise, we must have an argument and it must be for a bus that
     we don't have yet:  */
  else {

    free_buses = NULL;
    which_bus = -1;

    if (ph1->tme_ph1_ram == NULL) {
      tme_output_append(&free_buses, " ram");
    }
    if (TME_ARG_IS(args[1], "ram")) {
      which_bus = TME_PH1_BUS_RAM;
    }

    if (ph1->tme_ph1_rom == NULL) {
      tme_output_append(&free_buses, " rom");
    }
    if (TME_ARG_IS(args[1], "rom")) {
      which_bus = TME_PH1_BUS_ROM;
    }

    if (ph1->tme_ph1_obio == NULL) {
      tme_output_append(&free_buses, " obio");
    }
    if (TME_ARG_IS(args[1], "obio")) {
      which_bus = TME_PH1_BUS_OBIO;
    }

    if (ph1->tme_ph1_vme == NULL) {
      tme_output_append(&free_buses, " vme");
    }
    if (TME_ARG_IS(args[1], "vme")) {
      which_bus = TME_PH1_BUS_VME;
    }

    if (args[1] == NULL
        || which_bus < 0
        || ph1->tme_ph1_buses[which_bus] != NULL) {
      if (free_buses != NULL) {
        tme_output_append_error(_output,
                                "%s%s",
                                _("remaining buses:"),
                                free_buses);
        tme_free(free_buses);
      }
      else {
        tme_output_append_error(_output, _("all buses present"));
      }
      return (EINVAL);
    }
    if (free_buses != NULL) {
      tme_free(free_buses);
    }

    if (args[2] != NULL) {
      tme_output_append_error(_output,
                              "%s %s",
                              args[2],
                              _("unexpected"));
      return (EINVAL);
    }

    /* create our side of a generic bus connection: */
    conn_ph1 = tme_new0(struct tme_ph1_bus_connection, 1);
    conn_bus = &conn_ph1->tme_ph1_bus_connection;
    conn = &conn_bus->tme_bus_connection;
    conn->tme_connection_next = *_conns;

    /* fill in the generic connection: */
    conn->tme_connection_type = TME_CONNECTION_BUS_GENERIC;
    conn->tme_connection_score = _tme_ph1_connection_score;
    conn->tme_connection_make = _tme_ph1_connection_make;
    conn->tme_connection_break = _tme_ph1_connection_break;

    conn_bus->tme_bus_signal = _tme_ph1_bus_signal;
    conn_bus->tme_bus_intack = NULL;
    conn_bus->tme_bus_tlb_set_add = _tme_ph1_mmu_tlb_set_add;
    conn_bus->tme_bus_tlb_fill = _tme_ph1_bus_tlb_fill;

    /* fill in the ph1 connection: */
    conn_ph1->tme_ph1_bus_connection_which = which_bus;

    /* add in this connection side possibility: */
    *_conns = conn;
  }

  /* done: */
  return (TME_OK);
}

/* this creates a new Phaethon 1 element: */
TME_ELEMENT_NEW_DECL(tme_machine_phaethon1) {
  int usage;
  struct tme_ph1 *ph1;
  int arg_i;

  arg_i = 1;
  usage = FALSE;

  /* we take have no arguments: */
  if (args[arg_i] != NULL) {
    tme_output_append_error(_output,
                            "%s %s, ",
                            args[arg_i],
                            _("unexpected"));
    usage = TRUE;
  }

  /* if our usage was bad: */
  if (usage) {
    tme_output_append_error(_output,
                            "%s %s",
                            _("usage:"),
                            args[0]);
    return (EINVAL);
  }

  /* allocate and inititialize a new Phaethon 1: */
  ph1 = tme_new0(struct tme_ph1, 1);
  ph1->tme_ph1_element = element;

  /* set the dd7seg register: */
  ph1->tme_ph1_dd7seg_upper_value = 0;
  ph1->tme_ph1_dd7seg_lower_value = 0;

  /* set the cfgsw register: */
  ph1->tme_ph1_cfgsw_value = 0;

  /* set the system enable register: */
  ph1->tme_ph1_sysen = 0;

  /* set the context register: */
  ph1->tme_ph1_context = 0;

  /* set the bus error register: */
  ph1->tme_ph1_buserror_value = 0;

  /* set the software interrupt pending register: */
  ph1->tme_ph1_swint_pending = 0;

  /* the MMU: */
  _tme_ph1_mmu_new(ph1);

  /* the busses: */
  ph1->tme_ph1_ram = NULL;
  ph1->tme_ph1_rom = NULL;
  ph1->tme_ph1_obio = NULL;
  ph1->tme_ph1_vme = NULL;

  /* we don't have the CPU's bus context register yet: */
  ph1->tme_ph1_m68k_bus_context = NULL;

  /* fill the element: */
  element->tme_element_private = ph1;
  element->tme_element_connections_new = _tme_ph1_connections_new;
  element->tme_element_command = _tme_ph1_command;

  return (TME_OK);
}

/* this creates a Phaethon1 "com" UART instance: */
TME_ELEMENT_SUB_NEW_DECL(tme_machine_phaethon1,com) {
  struct tme_tl16c550_socket socket;
  char *sub_args[2];

  /* create the tl16c550 socket: */
  socket.tme_tl16c550_socket_version = TME_TL16C550_SOCKET_0;
  socket.tme_tl16c550_socket_addr_shift = 1;
  socket.tme_tl16c550_socket_port_least_lane = 1; /* D15-D8 */

  /* create the tl16c550: */
  sub_args[0] = "tme/ic/tl16c550";
  sub_args[1] = NULL;
  return (tme_element_new(element,
                          (const char * const *) sub_args,
                          &socket,
                          _output));
}

/* this creates a Phaethon1 "timer" instance: */
TME_ELEMENT_SUB_NEW_DECL(tme_machine_phaethon1,timer)
{
	struct tme_pg68k_timer_socket socket;

  /* create the pgtimer socket: */
  socket.tme_pg68k_timer_socket_version = TME_PG68K_TIMER_SOCKET_0;
  socket.tme_pg68k_timer_socket_addr_shift = 1;
  socket.tme_pg68k_timer_socket_port_least_lane = 1;		/* D15-D8 */
  socket.tme_pg68k_timer_socket_clock_basic = 10000000; /* 10MHz */
  socket.tme_pg68k_timer_socket_int_signal = TME_BUS_SIGNAL_INT(6);

	return (tme_pg68k_timer(element, &socket, _output));
}

/* this creates a Phaethon1 "ata" instance: */
TME_ELEMENT_SUB_NEW_DECL(tme_machine_phaethon1,ata)
{
	struct tme_ata_controller_socket socket;
  char *sub_args[2];

  /* create the ATA controller socket: */
  socket.tme_ata_controller_socket_version = TME_ATA_CONTROLLER_SOCKET_0;
  socket.tme_ata_controller_socket_addr_shift = 1;
  socket.tme_ata_controller_socket_port_least_lane = 1;				/* D15-D8 */

  /* create the ata controller: */
  sub_args[0] = "tme/ata/controller";
  sub_args[1] = NULL;
  return (tme_element_new(element,
                          (const char * const *) sub_args,
                          &socket,
                          _output));
}
