/* machine/pg68k/pg68k-timer.c - implementation of the system "hardclock"
   timer built into the I/O controller on 68k Playground machines.  */

/*
 * Copyright (c) 2026 Jason R. Thorpe
 * All rights reserved.
 */

/*
 * Copyright (c) 2004 Matt Fredette
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
#include <tme/generic/bus-device.h>
#include <tme/machine/pg68k.h>
#include <sys/types.h>
#include <sys/time.h>

/* macros: */
#define PGTIMER_REG_CSR   0
#define   CSR_ENAB        TME_BIT(0)  /* timer is enabled */
#define   CSR_INT         TME_BIT(1)  /* interrupt is pending */

#define PGTIMER_REG_VAL   1

#define PGTIMER_NUM_REGS  2

/* define this to track interrupt rates, reporting once every N
   seconds:  */
#if 1
#define TME_PGTIMER_TRACK_INT_RATE  (10)
#endif

#define TME_PGTIMER_LOG_HANDLE(t) \
        (&(t)->tme_pgtimer_element->tme_element_log_handle)

/* structures: */
struct tme_pgtimer {

  /* our simple bus device header: */
  struct tme_bus_device tme_pgtimer_device;
#define tme_pgtimer_element tme_pgtimer_device.tme_bus_device_element

  /* our socket: */
  struct tme_pg68k_timer_socket tme_pgtimer_socket;
#define tme_pgtimer_addr_shift  \
        tme_pgtimer_socket.tme_pg68k_timer_socket_addr_shift
#define tme_pgtimer_port_least_lane \
        tme_pgtimer_socket.tme_pg68k_timer_socket_port_least_lane
#define tme_pgtimer_clock_basic \
        tme_pgtimer_socket.tme_pg68k_timer_socket_clock_basic
#define tme_pgtimer_int_signal \
        tme_pgtimer_socket.tme_pg68k_timer_socket_int_signal

  /* our mutex: */
  tme_mutex_t tme_pgtimer_mutex;

  /* our timer condition: */
  tme_cond_t tme_pgtimer_cond;

  /* this is non-zero iff callouts are running: */
  int tme_pgtimer_callouts_running;

  /* the control / status register: */
  tme_uint8_t tme_pgtimer_reg_csr;

  /* reload value: */
  tme_uint16_t tme_pgtimer_reload;

  /* pre-computed real-time duration of programmed timer interval.
     this gets calculated when the ENAB bit is set in the CSR. */
  unsigned long tme_pgtimer_interval_usec;

  /* notes if we were sleeping for the next interval to expire. */
  int tme_pgtimer_sleeping;

  /* if our interrupt output is currently asserted: */
  int tme_pgtimer_int_asserted;

#ifdef TME_PGTIMER_TRACK_INT_RATE
  unsigned long tme_pgtimer_int_sample;
  struct timeval tme_pgtimer_int_sample_time;
#endif
};

/* the bus router: */
static const tme_bus_lane_t tme_pgtimer_router[TME_BUS_ROUTER_SIZE(TME_BUS8_LOG2)] = {

  /* [gen]  initiator port size: 8 bits
     [gen]  initiator port least lane: 0: */
  /* D7-D0 */   TME_BUS_LANE_ROUTE(0),
};

/* this resets the pgtimer: */
static void
_tme_pgtimer_reset(struct tme_pgtimer *t)
{
  t->tme_pgtimer_reg_csr = 0;
  t->tme_pgtimer_reload  = 0;

  /* callout after resetting to update interrupt output. */
}

/* the pgtimer callout function.  it must be called with the mutex locked: */
static void
_tme_pgtimer_callout(struct tme_pgtimer *t)
{
  /* all we need to do here is check to see if int_pending is the same
     as int_asserted, and if it's not, then we need to change int_asserted
     to match.  if we get an error in the process of doing this, we
     simply wait for the counter expire again, which will set int_pending
     and call us again.  */
  struct tme_bus_connection *conn_bus;
  int int_asserted_new;
  int rc;

  if (t->tme_pgtimer_callouts_running) {
    return;
  }

  t->tme_pgtimer_callouts_running = TRUE;

  /* get our bus connection: */
  conn_bus
    = tme_memory_atomic_pointer_read(struct tme_bus_connection *,
        t->tme_pgtimer_device.tme_bus_device_connection,
        &t->tme_pgtimer_device.tme_bus_device_connection_rwlock);

  for (;;) {
    int_asserted_new = (t->tme_pgtimer_reg_csr == (CSR_INT | CSR_ENAB));
    if (int_asserted_new == t->tme_pgtimer_int_asserted) {
      break;
    }

    /* unlock our mutex: */
    tme_mutex_unlock(&t->tme_pgtimer_mutex);

    /* call out the bus interrupt signal: */
    rc = (*conn_bus->tme_bus_signal)
      (conn_bus,
       t->tme_pgtimer_int_signal
       | (int_asserted_new
          ? TME_BUS_SIGNAL_LEVEL_ASSERTED
          : TME_BUS_SIGNAL_LEVEL_NEGATED));

    /* lock our mutex: */
    tme_mutex_lock(&t->tme_pgtimer_mutex);

    /* if this callout was successful, note the new state of the
       interrupt signal. */
    if (rc == TME_OK) {
      t->tme_pgtimer_int_asserted = int_asserted_new;
      /* will double-check at the top of the loop. */
    } else {
      break;
    }
  }

  t->tme_pgtimer_callouts_running = FALSE;
}

/* the timer thread: */
static void
_tme_pgtimer_th_timer(struct tme_pgtimer *t)
{
#ifdef TME_PGTIMER_TRACK_INT_RATE
  struct timeval now;
#endif /* TME_PGTIMER_TRACK_INT_RATE */
  int was_sleeping;

  /* lock the mutex: */
  tme_mutex_lock(&t->tme_pgtimer_mutex);

  /* loop forever: */
  for (;;) {

    /* if we were sleeping: */
    was_sleeping = t->tme_pgtimer_sleeping;
    t->tme_pgtimer_sleeping = FALSE;
    if (was_sleeping) {

#ifdef TME_PGTIMER_TRACK_INT_RATE
      /* if no interrupt is pending, we will deliver another interrupt: */
      if (t->tme_pgtimer_reg_csr == CSR_ENAB) {
        t->tme_pgtimer_int_sample++;
      }

      /* if the sample time has finished, report on the interrupt rate: */
      gettimeofday(&now, NULL);
      if (now.tv_sec > t->tme_pgtimer_int_sample_time.tv_sec
          || (now.tv_sec == t->tme_pgtimer_int_sample_time.tv_sec
              && now.tv_usec > t->tme_pgtimer_int_sample_time.tv_usec)) {
        if (t->tme_pgtimer_int_sample > 0) {
          tme_log(TME_PGTIMER_LOG_HANDLE(t),
                  0, TME_OK,
                  (TME_PGTIMER_LOG_HANDLE(t),
                   "timer interrupt rate: %ld/sec",
                   (t->tme_pgtimer_int_sample
                    / (TME_PGTIMER_TRACK_INT_RATE
                       + (unsigned long) (now.tv_sec
                                          - t->tme_pgtimer_int_sample_time.tv_sec)))));
        }

        /* reset the sample: */
        t->tme_pgtimer_int_sample_time.tv_sec
          = now.tv_sec + TME_PGTIMER_TRACK_INT_RATE;
        t->tme_pgtimer_int_sample_time.tv_usec = now.tv_usec;
        t->tme_pgtimer_int_sample = 0;
      }
#endif /* TME_PGTIMER_TRACK_INT_RATE */

      /* if the timer is still enabled, note the pending interrupt. */
      if (t->tme_pgtimer_reg_csr & CSR_ENAB) {
        t->tme_pgtimer_reg_csr |= CSR_INT;
      } else {
        t->tme_pgtimer_reg_csr = 0;
      }

      /* callout to update our interrupt signal: */
      _tme_pgtimer_callout(t);
    }

    /* if the timer is disabled, wait for that to change. */
    if ((t->tme_pgtimer_reg_csr & CSR_ENAB) == 0) {
      tme_cond_wait_yield(&t->tme_pgtimer_cond,
                          &t->tme_pgtimer_mutex);
      continue;
    }

    /* snooze until the next interval expires. */
    t->tme_pgtimer_sleeping = TRUE;

    /* unlock our mutex: */
    tme_mutex_unlock(&t->tme_pgtimer_mutex);

    /* sleep: */
    tme_thread_sleep_yield(0, t->tme_pgtimer_interval_usec);

    /* lock our mutex: */
    tme_mutex_lock(&t->tme_pgtimer_mutex);
  }
  /* NOTREACHED */
}

static void
_tme_pgtimer_compute_interval(struct tme_pgtimer *t)
{
  /* the input clock is divided by 16 */
  unsigned int timer_freq = t->tme_pgtimer_clock_basic / 16;

  /* from the timer frequency, calculate the ns per clock tick */
  unsigned int ns_per_tick = 1000000000 / timer_freq;

  /* get the number of ticks from the reload value */
  unsigned int interval_ticks = t->tme_pgtimer_reload;

  /* compute the ns per interval */
  unsigned int interval_ns = interval_ticks * ns_per_tick;

  /* and there we have our usec per interval */
  t->tme_pgtimer_interval_usec = interval_ns / 1000;
}

/* the pgtimer bus cycle handler: */
static int
_tme_pgtimer_bus_cycle(void *_pgtimer, struct tme_bus_cycle *cycle_init)
{
  struct tme_pgtimer *t;
  tme_bus_addr32_t address, address_last;
  tme_uint8_t buffer, value;
  struct tme_bus_cycle cycle_resp;
  int need_callout;
  int reg;

  /* recover our data structure: */
  t = (struct tme_pgtimer *) _pgtimer;

  /* the requested cycle must be within range: */
  address_last = t->tme_pgtimer_device.tme_bus_device_address_last;
  address = cycle_init->tme_bus_cycle_address;
  assert(address <= address_last);
  assert(cycle_init->tme_bus_cycle_size <= (address_last - address) + 1);
  (void)address_last;

  /* get the register being accessed: */
  reg = address >> t->tme_pgtimer_addr_shift;

  /* lock the mutex: */
  tme_mutex_lock(&t->tme_pgtimer_mutex);

  /* assume we don't need any callouts: */
  need_callout = 0;

  /* if this is a write: */
  if (cycle_init->tme_bus_cycle_type == TME_BUS_CYCLE_WRITE) {

    /* run the bus cycle: */
    cycle_resp.tme_bus_cycle_buffer = &buffer;
    cycle_resp.tme_bus_cycle_lane_routing = tme_pgtimer_router;
    cycle_resp.tme_bus_cycle_address = 0;
    cycle_resp.tme_bus_cycle_buffer_increment = 1;
    cycle_resp.tme_bus_cycle_type = TME_BUS_CYCLE_READ;
    cycle_resp.tme_bus_cycle_size = sizeof(buffer);
    cycle_resp.tme_bus_cycle_port =
      TME_BUS_CYCLE_PORT(t->tme_pgtimer_port_least_lane,
                         TME_BUS8_LOG2);
    tme_bus_cycle_xfer(cycle_init, &cycle_resp);
    value = buffer;

    /* log this write: */
    tme_log(TME_PGTIMER_LOG_HANDLE(t), 100000, TME_OK,
            (TME_PGTIMER_LOG_HANDLE(t),
             "REG %d <- 0x%02x", reg, value));

    switch (reg) {

    case PGTIMER_REG_CSR:
      /* The ENAB bit is the only writable one, and we only need
         to take action if it changes. changing it clears the other
         bit in the register (INT).  */

      value &= CSR_ENAB;
      if ((value ^ t->tme_pgtimer_reg_csr) & CSR_ENAB) {
        /* if enabling, recompute the timer interval. */
        if (value & CSR_ENAB) {
          _tme_pgtimer_compute_interval(t);
          /* notify the timer thread */
          tme_cond_notify(&t->tme_pgtimer_cond, FALSE);
        }
        t->tme_pgtimer_reg_csr = value;
        need_callout = TRUE;
      }
      break;

      /* writing to the value register clears the CSR. */
    case PGTIMER_REG_VAL:
      t->tme_pgtimer_reg_csr = 0;
      t->tme_pgtimer_reload = (t->tme_pgtimer_reload << 8) | value;
      need_callout = TRUE;
      break;

    default:
      abort();
    }
  }

  /* otherwise, this is a read: */
  else {
    assert(cycle_init->tme_bus_cycle_type == TME_BUS_CYCLE_READ);

    switch (reg) {

    case PGTIMER_REG_CSR:
      value = t->tme_pgtimer_reg_csr;
      /* reading the CSR clears any pending interrupt. */
      if (value & CSR_INT) {
        t->tme_pgtimer_reg_csr &= ~CSR_INT;
        need_callout = TRUE;
      }
      break;

    case PGTIMER_REG_VAL:
      /* this is a write-only register; hardware returns 0xff on read */
      value = 0xff;
      break;

    default:
      abort();
    }

    /* run the bus cycle: */
    buffer = value;
    cycle_resp.tme_bus_cycle_buffer = &buffer;
    cycle_resp.tme_bus_cycle_lane_routing = tme_pgtimer_router;
    cycle_resp.tme_bus_cycle_address = 0;
    cycle_resp.tme_bus_cycle_buffer_increment = 1;
    cycle_resp.tme_bus_cycle_type = TME_BUS_CYCLE_WRITE;
    cycle_resp.tme_bus_cycle_size = sizeof(buffer);
    cycle_resp.tme_bus_cycle_port =
      TME_BUS_CYCLE_PORT(t->tme_pgtimer_port_least_lane,
                         TME_BUS8_LOG2);
    tme_bus_cycle_xfer(cycle_init, &cycle_resp);
  }

  /* run the callout, if needed. */
  if (need_callout) {
    _tme_pgtimer_callout(t);
  }

  /* unlock the mutex: */
  tme_mutex_unlock(&t->tme_pgtimer_mutex);

  /* no faults: */
  return (TME_OK);
}

/* the TLB filler: */
static int
_tme_pgtimer_tlb_fill(void *_pgtimer,
                      struct tme_bus_tlb *tlb,
                      tme_bus_addr_t address,
                      unsigned int cycles)
{
  struct tme_pgtimer *t;
  tme_bus_addr32_t address_last;

  /* recover our data structure: */
  t = (struct tme_pgtimer *) _pgtimer;

  /* the address must be within range: */
  address_last = t->tme_pgtimer_device.tme_bus_device_address_last;
  assert(address <= address_last);

  /* initialize the TLB entry: */
  tme_bus_tlb_initialize(tlb);

  /* this TLB entry can cover the whole device: */
  tlb->tme_bus_tlb_addr_first = 0;
  tlb->tme_bus_tlb_addr_last = address_last;

  /* allow reading and writing: */
  tlb->tme_bus_tlb_cycles_ok = TME_BUS_CYCLE_READ | TME_BUS_CYCLE_WRITE;

  /* our bus cycle handler: */
  tlb->tme_bus_tlb_cycle_private = t;
  tlb->tme_bus_tlb_cycle = _tme_pgtimer_bus_cycle;

  return (TME_OK);
}

/* the bus signal handler: */
static int
_tme_pgtimer_signal(void *_pgtimer,
                    unsigned int signal)
{
  struct tme_pgtimer *t;
  unsigned int level;
  int need_callout;

  /* recover our data structure: */
  t = (struct tme_pgtimer *) _pgtimer;

  /* assume we won't need any new callouts: */
  need_callout = FALSE;

  /* lock the mutex: */
  tme_mutex_lock(&t->tme_pgtimer_mutex);

  /* take out the signal level: */
  level = signal & TME_BUS_SIGNAL_LEVEL_MASK;
  signal = TME_BUS_SIGNAL_WHICH(signal);

  /* dispatch on the generic bus signals: */
  switch (signal) {

  case TME_BUS_SIGNAL_RESET:
    if (level == TME_BUS_SIGNAL_LEVEL_ASSERTED) {
      _tme_pgtimer_reset(t);
      need_callout = TRUE;
    }
    break;

  default:
    break;
  }

  if (need_callout) {
    _tme_pgtimer_callout(t);
  }

  /* unlock the mutex: */
  tme_mutex_unlock(&t->tme_pgtimer_mutex);

  /* no faults: */
  return (TME_OK);
}

/* the new pgtimer function: */
int
tme_pg68k_timer(struct tme_element *element,
                const struct tme_pg68k_timer_socket *socket,
                char **_output)
{
  struct tme_pgtimer *t;
  struct tme_pg68k_timer_socket socket_real;

  /* dispatch on our socket version: */
  if (socket == NULL) {
    tme_output_append_error(_output, _("need an ic socket"));
    return (ENXIO);
  }
  switch (socket->tme_pg68k_timer_socket_version) {
  case TME_PG68K_TIMER_SOCKET_0:
    socket_real = *socket;
    break;
  default:
    tme_output_append_error(_output, _("socket type"));
    return (EOPNOTSUPP);
  }

  t = tme_new0(struct tme_pgtimer, 1);
  t->tme_pgtimer_socket = socket_real;
  t->tme_pgtimer_element = element;
  _tme_pgtimer_reset(t);

  /* The I/O controller will only ACK the 3 individual registers, so
     don't round the last address up to a power of two! */

  /* initialize our simple bus device descriptor: */
  t->tme_pgtimer_device.tme_bus_device_tlb_fill = _tme_pgtimer_tlb_fill;
  t->tme_pgtimer_device.tme_bus_device_signal = _tme_pgtimer_signal;
  t->tme_pgtimer_device.tme_bus_device_address_last
    = (PGTIMER_NUM_REGS - 1) << t->tme_pgtimer_addr_shift;

  /* fill the element: */
  element->tme_element_private = t;
  element->tme_element_connections_new = tme_bus_device_connections_new;

  /* start the timer thread: */
  tme_mutex_init(&t->tme_pgtimer_mutex);
  tme_cond_init(&t->tme_pgtimer_cond_reader);
  tme_thread_create((tme_thread_t) _tme_pgtimer_th_timer, t);

  return (TME_OK);
}
