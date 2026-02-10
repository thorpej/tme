/* ic/tl16c550.c - implementation of TI TL16C550 emulation: */

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
#include <tme/generic/bus-device.h>
#include <tme/generic/serial.h>
#include <tme/ic/tl16c550.h>

/* macros: */

/*
 * ACE registers.  Data register in real hardware is actually two
 * separate registers, one write-only and the other read-only.
 */

/* receiver buffer register (R) */
#define ACE_REG_RBR   0

/* transmit holding register (W) (divisor latch LSB when DLAB=1) */
#define ACE_REG_THR   0

/* interrupt enable register (divisor latch MSB when DLAB=1) */
#define ACE_REG_IER   1
#define IER_ERBI      TME_BIT(0)  /* receive data available */
#define IER_ETBEI     TME_BIT(1)  /* transmit holding register empty */
#define IER_ELSI      TME_BIT(2)  /* line status change */
#define IER_EDSSI     TME_BIT(3)  /* modem status change */
#define IER_mask      0x0f

/* interrupt ident register (R) */
#define ACE_REG_IIR   2
#define IIR_NOPEND    0x01        /* set if no interrupt pending */
#define IIR_MLS       0x00        /* modem status changed */
#define IIR_TXRDY     0x02        /* transmitter ready */
#define IIR_RXRDY     0x04        /* receiver ready */
#define IIR_RLS       0x06        /* line status changed */
#define IIR_RXTOUT    0x0c        /* receiver timeout */
#define IIR_FIFOEN    0xc0        /* set when FIFOs enabled */

/* FIFO control register (W) */
#define ACE_REG_FCR   2
#define FCR_FIFOEN    TME_BIT(0)  /* FIFO enable */
#define FCR_RFR       TME_BIT(1)  /* Rx FIFO reset */
#define FCR_TFR       TME_BIT(2)  /* Tx FIFO reset */
#define FCR_DMA       TME_BIT(3)  /* DMA mode select */
#define FCR_RXT_1     (0  << 6)   /* Rx FIFO trigger: 1 byte */
#define FCR_RXT_4     (1  << 6)   /* Rx FIFO trigger: 4 bytes */
#define FCR_RXT_8     (2  << 6)   /* Rx FIFO trigger: 8 bytes */
#define FCR_RXT_14    (3  << 6)   /* Rx FIFO trigger: 14 bytes */
#define FCR_mask      0xc9

/* line control register */
#define ACE_REG_LCR   3
#define LCR_D5        0x00        /* 5 data bits */
#define LCR_D6        0x01        /* 6 data bits */
#define LCR_D7        0x02        /* 7 data bits */
#define LCR_D8        0x03        /* 8 data bits */
#define LCR_dmask     0x03
#define LCR_S2        TME_BIT(2)  /* 1=2 stop bits, 0=1 stop bit */
#define LCR_PEN       TME_BIT(3)  /* parity enable */
#define LCR_PEVEN     TME_BIT(4)  /* even parity */
#define LCR_PSTICK    TME_BIT(5)  /* stick parity */
#define LCR_BREAK     TME_BIT(6)  /* send BREAK */
#define LCR_DLAB      TME_BIT(7)  /* divisor latch access bit */

/* modem control register */
#define ACE_REG_MCR   4
#define MCR_DTR       TME_BIT(0)  /* assert DTR */
#define MCR_RTS       TME_BIT(1)  /* assert RTS */
#define MCR_OUT1      TME_BIT(2)  /* see data sheet */
#define MCR_OUT2      TME_BIT(3)  /* see data sheet */
#define MCR_LOOP      TME_BIT(4)  /* loopback test mode */
#define MCR_AFE       TME_BIT(5)  /* auto-flow control enable */
#define MCR_mask      0x3f

/* line status register */
#define ACE_REG_LSR   5
#define LSR_DR        TME_BIT(0)  /* data ready */
#define LSR_OE        TME_BIT(1)  /* overrun error */
#define LSR_PE        TME_BIT(2)  /* parity error */
#define LSR_FE        TME_BIT(3)  /* framing error */
#define LSR_BI        TME_BIT(4)  /* BREAK interrupt */
#define LSR_THRE      TME_BIT(5)  /* Tx holding register empty */
#define LSR_TEMT      TME_BIT(6)  /* transmitter empty (FIFO + SR) */
#define LSR_FIFO_ERR  TME_BIT(7)  /* at least one error in Rx FIFO */

/* modem status register */
#define ACE_REG_MSR   6
#define MSR_DCTS      TME_BIT(0)  /* CTS changed */
#define MSR_DDSR      TME_BIT(1)  /* DSR changed */
#define MSR_TERI      TME_BIT(2)  /* RI toggled low -> high */
#define MSR_DDCD      TME_BIT(3)  /* DCD changed */
#define MSR_CTS       TME_BIT(4)  /* CTS asserted */
#define MSR_DSR       TME_BIT(5)  /* DSR asserted */
#define MSR_RI        TME_BIT(6)  /* RI asserted */
#define MSR_DCD       TME_BIT(7)  /* DCD asserted */

/* scratch register */
#define ACE_REG_SCR   7

#define ACE_NUM_REGS  8           /* number of register addresses */

/* buffer sizes: */
#define TME_ACE_BUFFER_SIZE_TX	(16)
#define TME_ACE_BUFFER_SIZE_RX	(128)

/* UART callout flags: */
#define TME_ACE_CALLOUT_CHECK		(0)
#define TME_ACE_CALLOUT_RUNNING	TME_BIT(0)
#define TME_ACE_CALLOUTS_MASK		(-2)
#define  TME_ACE_CALLOUT_CTRL   TME_BIT(1)
#define  TME_ACE_CALLOUT_CONFIG	TME_BIT(2)
#define  TME_ACE_CALLOUT_READ   TME_BIT(3)
#define	 TME_ACE_CALLOUT_INT    TME_BIT(4)

#define TME_ACE_LOG_HANDLE(a) (&(a)->tme_ace_element->tme_element_log_handle)

/* XXX does not implement the auto CTS/RTS logic for the TL16C550's
   Automatic Flow Control.  */

/* structures: */

/* the chip: */
struct tme_ace {

  /* our simple bus device header: */
  struct tme_bus_device tme_ace_device;
#define tme_ace_element tme_ace_device.tme_bus_device_element

  /* our socket: */
  struct tme_tl16c550_socket tme_ace_socket;
#define tme_ace_addr_shift      tme_ace_socket.tme_tl16c550_socket_addr_shift
#define tme_ace_port_least_lane tme_ace_socket.tme_tl16c550_socket_port_least_lane
#define tme_ace_baudclk         tme_ace_socket.tme_tl16c550_socket_baudclk

  /* the mutex protecting the chip: */
  tme_mutex_t tme_ace_mutex;

  /* our registers */
  tme_uint8_t tme_ace_reg_rbr;
  tme_uint8_t tme_ace_reg_ier;
  tme_uint8_t tme_ace_reg_iir;
  tme_uint8_t tme_ace_reg_fcr;
  tme_uint8_t tme_ace_reg_lcr;
  tme_uint8_t tme_ace_reg_mcr;
  tme_uint8_t tme_ace_reg_lsr;
  tme_uint8_t tme_ace_reg_msr;
  tme_uint8_t tme_ace_reg_scr;
  tme_uint8_t tme_ace_reg_div_lsb;
  tme_uint8_t tme_ace_reg_div_msb;

  /* the serial connected to this UART: */
  struct tme_serial_connection *tme_ace_connection;

  /* our transmitter buffer: */
  struct tme_serial_buffer tme_ace_buffer_tx;

  /* our receiver buffer: */
  struct tme_serial_buffer tme_ace_buffer_rx;

  /* our callout flags: */
  int tme_ace_callout_flags;

  /* internal control flags: */
  int tme_ace_internal_flags;

#define ACE_F_TXRDY_PENDING     TME_BIT(0)
#define ACE_F_TXFIFO_RESETTING  TME_BIT(1)

#ifndef TME_NO_LOG
  tme_uint8_t tme_ace_last_read_reg;
  tme_uint8_t tme_ace_last_read_value;
#endif /* !TME_NO_LOG */

  /* if our interrupt line is currently asserted: */
  int tme_ace_int_asserted;
};

/* the tl16c550 bus router: */
static const tme_bus_lane_t tme_ace_router[TME_BUS_ROUTER_SIZE(TME_BUS8_LOG2)] = {
  
  /* [gen]  initiator port size: 8 bits
     [gen]  initiator port least lane: 0: */
  /* D7-D0 */	TME_BUS_LANE_ROUTE(0),
};

/* this resets the tl16c550 registers: */
static void
_tme_ace_reset_registers(struct tme_ace *ace)
{
  /* See "Table 2. ACE Reset Functions" from the data sheet. */
  ace->tme_ace_reg_ier = 0;
  ace->tme_ace_reg_iir = IIR_NOPEND;
  ace->tme_ace_reg_fcr = 0;
  ace->tme_ace_reg_lcr = 0;
  ace->tme_ace_reg_mcr = 0;
  ace->tme_ace_reg_lsr = LSR_THRE | LSR_TEMT;
  ace->tme_ace_reg_msr = 0;
}

/* this resets the tl16c550 Rx FIFO: */
static int
_tme_ace_reset_rx_fifo(struct tme_ace *ace)
{
  int new_callouts = 0;

  if (!tme_serial_buffer_is_empty(&ace->tme_ace_buffer_rx)) {
    ace->tme_ace_buffer_rx.tme_serial_buffer_head
      = ace->tme_ace_buffer_rx.tme_serial_buffer_tail = 0;
    ace->tme_ace_reg_lsr
      &= ~(LSR_DR | LSR_OE | LSR_PE | LSR_FE | LSR_BI | LSR_FIFO_ERR);
    new_callouts |= TME_ACE_CALLOUT_INT;
  }

  return (new_callouts);
}

/* this resets the tl16c550 Tx FIFO: */
static int
_tme_ace_reset_tx_fifo(struct tme_ace *ace)
{
  if (!tme_serial_buffer_is_empty(&ace->tme_ace_buffer_tx)) {
    ace->tme_ace_buffer_tx.tme_serial_buffer_head
      = ace->tme_ace_buffer_tx.tme_serial_buffer_tail = 0;
    ace->tme_ace_reg_lsr |= (LSR_THRE | LSR_TEMT);
    /* XXX does a real 16550 generate a TXRDY interrupt when the Tx FIFO
       has been cleared programatically? */
  }

  return (0);
}

/* this resets the tl16c550: */
static int
_tme_ace_reset(struct tme_ace *ace)
{
  /* cancel any pending callouts: */
  ace->tme_ace_callout_flags &= ~TME_ACE_CALLOUTS_MASK;

  _tme_ace_reset_rx_fifo(ace);
  _tme_ace_reset_tx_fifo(ace);
  _tme_ace_reset_registers(ace);
  ace->tme_ace_internal_flags = 0;

  /* assume that the interrupt signal has changed: */
  return (TME_ACE_CALLOUT_INT);
}

/* this computes the new interrupt posture: */
static int
_tme_ace_int_pending(struct tme_ace *ace)
{
  tme_uint8_t new_iir;
  int int_pending;

  /* assume no interrupt pending: */
  int_pending = FALSE;

  /* IN PRIORITY ORDER: */

  /* if any of the line status error bits are set, then we have a
     line status interrupt.  */
  if (ace->tme_ace_reg_lsr & (LSR_OE | LSR_PE | LSR_FE | LSR_BI)) {
    new_iir = IIR_RLS;
    if (ace->tme_ace_reg_ier & IER_ELSI) {
      int_pending = TRUE;
    }
  }

  /* if there is Rx data available, then we have a receive data
     available interrupt.  */
  else if (ace->tme_ace_reg_lsr & LSR_DR) {
    new_iir = IIR_RXRDY;
    if (ace->tme_ace_reg_ier & IER_ERBI) {
      int_pending = TRUE;
    }
  }

  /* if we have a rising-edge of THR-empty, then we have a transmit
     holding register empty interrupt.  */
  else if ((ace->tme_ace_reg_lsr & LSR_THRE) != 0
           && (ace->tme_ace_internal_flags & ACE_F_TXRDY_PENDING)) {
    new_iir = IIR_TXRDY;
    if (ace->tme_ace_reg_ier & IER_ETBEI) {
      int_pending = TRUE;
    }
  }

  /* if any of the modem status change bits are set, then we have a
     modem status changed interrupt.  */
  else if (ace->tme_ace_reg_msr & (MSR_DCTS | MSR_DDSR | MSR_TERI | MSR_DDCD)) {
    new_iir = IIR_MLS;
    if (ace->tme_ace_reg_ier & IER_EDSSI) {
      int_pending = TRUE;
    }
  }

  /* otherwise, nothing! */
  else {
    new_iir = IIR_NOPEND;
  }

  ace->tme_ace_reg_iir = new_iir;
  return (int_pending);
}

/* this attempts to refill the receive FIFO: */
static int
_tme_ace_rx_fifo_refill(struct tme_ace *ace)
{
  tme_uint8_t byte_buffer, byte;
  tme_serial_data_flags_t data_flags_buffer, data_flags;
  tme_uint8_t lsr;
  int new_callouts;
  unsigned int rc;

  /* assume we won't need any new callouts, and that no receive interrupt
     should be pending.  */
  new_callouts = 0;

  /* if the Rx buffer is currently full, after we copy out the
     next character we want to call out to read more data: */
  if (tme_serial_buffer_is_full(&ace->tme_ace_buffer_rx)) {
    new_callouts |= TME_ACE_CALLOUT_READ;
  }

  /* get the next byte from our Rx buffer: */
  rc = tme_serial_buffer_copyout(&ace->tme_ace_buffer_rx,
                                 &byte_buffer, 1,
                                 &data_flags_buffer,
                                 TME_SERIAL_COPY_NORMAL);

  lsr = ace->tme_ace_reg_lsr;

  /* if the Rx buffer was empty, the Rx FIFO is now empty: */
  if (rc == 0) {
    lsr &= ~LSR_DR;
  }

  /* otherwise we have another byte for the Rx FIFO: */
  else {
    byte = byte_buffer;
    data_flags = data_flags_buffer;

    /* put the byte into the RBR: */
    ace->tme_ace_reg_rbr = byte;

    /* update the LSR: */
    if (data_flags & TME_SERIAL_DATA_BAD_FRAME) {
      lsr |= LSR_FE;
    }
    else {
      lsr &= ~LSR_FE;
    }
    if (data_flags & TME_SERIAL_DATA_OVERRUN) {
      lsr |= LSR_OE;
    }
    if (data_flags & TME_SERIAL_DATA_BAD_PARITY) {
      lsr |= LSR_PE;
    }
    else {
      lsr &= ~LSR_PE;
    }

    /* XXX Don't emulate FIFO_ERR at all */

    /* we have data available! */
    lsr |= LSR_DR;
  }

  /* recompute interrupts if the LSR changed: */
  if (ace->tme_ace_reg_lsr != lsr) {
    ace->tme_ace_reg_lsr = lsr;
    new_callouts |= TME_ACE_CALLOUT_INT;
  }

  /* done: */
  return (new_callouts);
}

/* the tl16c550 callout function.  it must be called with the mutex locked: */
static void
_tme_ace_callout(struct tme_ace *ace,
                 int new_callouts)
{
  struct tme_serial_connection *conn_serial;
  struct tme_bus_connection *conn_bus;
  unsigned int ctrl;
  tme_uint8_t buffer_input[32];
  unsigned int buffer_input_size;
  tme_serial_data_flags_t data_flags;
  struct tme_serial_config config;
  int callouts;
  int later_callouts;
  int int_asserted;
  int rc;

  /* add in any new callouts: */
  ace->tme_ace_callout_flags |= new_callouts;

  /* if this function is already running in another thread, return
     now.  the other thread will do our work: */
  if (ace->tme_ace_callout_flags & TME_ACE_CALLOUT_RUNNING) {
    return;
  }

  /* callouts are now running: */
  ace->tme_ace_callout_flags |= TME_ACE_CALLOUT_RUNNING;

  /* assume that we won't need any later callouts: */
  later_callouts = 0;

  /* assume that we won't be changing the interrupt output: */
  int_asserted = -1;

  /* get our serial connection. */
  conn_serial = ace->tme_ace_connection;

  /* loop while callouts are needed: */
  for (;;) {

    callouts = ace->tme_ace_callout_flags & TME_ACE_CALLOUTS_MASK;
    if (callouts == 0) {
      break;
    }

    /* clear the needed callouts: */
    ace->tme_ace_callout_flags &= ~TME_ACE_CALLOUTS_MASK;

    /* if we need to call out new control information: */
    if (callouts & TME_ACE_CALLOUT_CTRL) {

      /* form the new ctrl: */
      ctrl = 0;
      if (ace->tme_ace_reg_mcr & MCR_DTR) {
        ctrl |= TME_SERIAL_CTRL_DTR;
      }
      if (ace->tme_ace_reg_mcr & MCR_RTS) {
        ctrl |= TME_SERIAL_CTRL_RTS;
      }
      if (ace->tme_ace_reg_lcr & LCR_BREAK) {
        ctrl |= TME_SERIAL_CTRL_BREAK;
      }
      if (!tme_serial_buffer_is_empty(&ace->tme_ace_buffer_tx)) {
        ctrl |= TME_SERIAL_CTRL_OK_READ;
      }

      /* unlock the mutex: */
      tme_mutex_unlock(&ace->tme_ace_mutex);
      
      /* do the callout: */
      rc = (conn_serial != NULL
        ? ((*conn_serial->tme_serial_connection_ctrl)
            (conn_serial,
             ctrl))
	      : TME_OK);
      
      /* lock the mutex: */
      tme_mutex_lock(&ace->tme_ace_mutex);
      
      /* if the callout was unsuccessful, remember that at some
         later time this callout should be attempted again: */
      if (rc != TME_OK) {
        later_callouts |= TME_ACE_CALLOUT_CTRL;
      }
    }
      
    /* if we need to call out new config information: */
    if (callouts & TME_ACE_CALLOUT_CONFIG) {

      /* form the new config: */
      memset(&config, 0, sizeof(config));

      /* the number of data bits per character.  */
      switch (ace->tme_ace_reg_lcr & LCR_dmask) {
      case LCR_D5:
        config.tme_serial_config_bits_data = 5;
        break;
      case LCR_D6:
        config.tme_serial_config_bits_data = 6;
        break;
      case LCR_D7:
        config.tme_serial_config_bits_data = 7;
        break;
      case LCR_D8:
        config.tme_serial_config_bits_data = 8;
        break;
      }

      config.tme_serial_config_bits_stop
        = (ace->tme_ace_reg_lcr & LCR_S2) ? 2 : 1;

      /* the parity: */
      /* XXX STICK parity not emulated. */
      if (ace->tme_ace_reg_lcr & LCR_PEN) {
        config.tme_serial_config_parity
          = (ace->tme_ace_reg_lcr & LCR_PEVEN)
            ? TME_SERIAL_PARITY_EVEN
            : TME_SERIAL_PARITY_ODD;
      } else {
        config.tme_serial_config_parity = TME_SERIAL_PARITY_NONE;
      }

      /* the baud rate: */
      /* XXX TBD compute from baudclk input and divisor latch */
      config.tme_serial_config_baud = 9600;
      
      /* flags: */
      /* XXX TBD: */
      config.tme_serial_config_flags = 0;
      
      /* unlock the mutex: */
      tme_mutex_unlock(&ace->tme_ace_mutex);
      
      /* do the callout: */
      rc = (conn_serial != NULL
        ? ((*conn_serial->tme_serial_connection_config)
            (conn_serial,
             &config))
	      : TME_OK);

      /* lock the mutex: */
      tme_mutex_lock(&ace->tme_ace_mutex);

      /* if the callout was unsuccessful, remember that at some
         later time this callout should be attempted again: */
      if (rc != TME_OK) {
        later_callouts |= TME_ACE_CALLOUT_CONFIG;
      }
    }

    /* if this channel's connection is readable: */
    if (callouts & TME_ACE_CALLOUT_READ) {

      /* if the receive buffer is full, remember that at some later
         time this callout should be attempted again: */
      if (tme_serial_buffer_is_full(&ace->tme_ace_buffer_rx)) {
        later_callouts |= TME_ACE_CALLOUT_READ;
      }

      /* otherwise, continue to do the read: */
      else {

      	/* get the minimum of the free space in the receive buffer and
	         the size of our stack buffer: */
      	buffer_input_size
          = tme_serial_buffer_space_free(&ace->tme_ace_buffer_rx);
        buffer_input_size
          = TME_MIN(buffer_input_size, sizeof(buffer_input));

        /* unlock the mutex: */
        tme_mutex_unlock(&ace->tme_ace_mutex);

        /* do the read: */
        rc = (conn_serial != NULL
          ? ((*conn_serial->tme_serial_connection_read)
            (conn_serial,
             buffer_input,
             buffer_input_size,
             &data_flags))
          : 0);
	  
        /* lock the mutex: */
        tme_mutex_lock(&ace->tme_ace_mutex);
	
        /* if the read was successful: */
        if (rc > 0) {

          /* put the the characters into our Rx buffer: */
          (void) tme_serial_buffer_copyin(&ace->tme_ace_buffer_rx,
                                          buffer_input,
                                          rc,
                                          data_flags,
                                          TME_SERIAL_COPY_NORMAL);

          /* if the Rx FIFO is empty, refill it.  */
          if ((ace->tme_ace_reg_lsr & LSR_DR) == 0) {
            /* (it'll tell us if we need to process interrupts.) */
            callouts |= _tme_ace_rx_fifo_refill(ace);
          }

          /* mark that we need to loop to callout to read more data: */
          ace->tme_ace_callout_flags |= TME_ACE_CALLOUT_READ;          
        }

        /* otherwise, the read failed.  convention dictates that we
           forget that the connection was readable, which we already
           have done by clearing the CALLOUT_READ flag: */
      }
    }

    /* if we need to call out a possible change to our interrupt signal: */
    if (callouts & TME_ACE_CALLOUT_INT) {

      /* compute the highest-priority interrupt reason and determine if
         it should interrupt the CPU. */
      int_asserted = _tme_ace_int_pending(ace);
    }
  }

  /* if we checked the interrupt signal and we need to call out a
     change: */
  if (int_asserted != -1
      && int_asserted != ace->tme_ace_int_asserted) {

    /* unlock our mutex: */
    tme_mutex_unlock(&ace->tme_ace_mutex);
    
    /* get our bus connection: */
    conn_bus
        = tme_memory_atomic_pointer_read(struct tme_bus_connection *,
            ace->tme_ace_device.tme_bus_device_connection,
            &ace->tme_ace_device.tme_bus_device_connection_rwlock);
    
    /* call out the bus interrupt signal edge: */
    rc = (*conn_bus->tme_bus_signal)
      (conn_bus,
       TME_BUS_SIGNAL_INT_UNSPEC
       | TME_BUS_SIGNAL_EDGE
       | (int_asserted
          ? TME_BUS_SIGNAL_LEVEL_ASSERTED
          : TME_BUS_SIGNAL_LEVEL_NEGATED));

    /* lock our mutex: */
    tme_mutex_lock(&ace->tme_ace_mutex);
    
    /* if this callout was successful, note the new state of the
       interrupt signal: */
    if (rc == TME_OK) {
      ace->tme_ace_int_asserted = int_asserted;
    }
    
    /* otherwise, remember that at some later time this callout
       should be attempted again: */
    else {
      later_callouts |= TME_ACE_CALLOUT_INT;
    }
  }

  /* put in any later callouts, and clear that callouts are running: */
  ace->tme_ace_callout_flags = later_callouts;
}      

/* the tl16c550 bus cycle handler: */
static int
_tme_ace_bus_cycle(void *_ace, struct tme_bus_cycle *cycle_init)
{
  struct tme_ace *ace;
  tme_bus_addr32_t address, ace_address_last;
  tme_uint8_t buffer, value;
  struct tme_bus_cycle cycle_resp;
  int new_callouts;
  int clear_rx_fifo, clear_tx_fifo;
  int reg;

  /* recover our data structure: */
  ace = (struct tme_ace *) _ace;

  /* the requested cycle must be within range: */
  ace_address_last = ace->tme_ace_device.tme_bus_device_address_last;
  address = cycle_init->tme_bus_cycle_address;
  assert(address <= ace_address_last);
  assert(cycle_init->tme_bus_cycle_size <= (ace_address_last - address) + 1);
  (void)ace_address_last;

  /* get the register being accessed: */
  reg = address >> ace->tme_ace_addr_shift;

  /* lock the mutex: */
  tme_mutex_lock(&ace->tme_ace_mutex);

  /* assume we won't need any new callouts: */
  new_callouts = 0;

  clear_rx_fifo = clear_tx_fifo = 0;

  /* if this is a write: */
  if (cycle_init->tme_bus_cycle_type == TME_BUS_CYCLE_WRITE) {

    /* run the bus cycle: */
    cycle_resp.tme_bus_cycle_buffer = &buffer;
    cycle_resp.tme_bus_cycle_lane_routing = tme_ace_router;
    cycle_resp.tme_bus_cycle_address = 0;
    cycle_resp.tme_bus_cycle_buffer_increment = 1;
    cycle_resp.tme_bus_cycle_type = TME_BUS_CYCLE_READ;
    cycle_resp.tme_bus_cycle_size = sizeof(buffer);
    cycle_resp.tme_bus_cycle_port = 
      TME_BUS_CYCLE_PORT(ace->tme_ace_port_least_lane,
			 TME_BUS8_LOG2);
    tme_bus_cycle_xfer(cycle_init, &cycle_resp);
    value = buffer;

    /* log this write: */
    tme_log(TME_ACE_LOG_HANDLE(ace), 100000, TME_OK,
	    (TME_ACE_LOG_HANDLE(ace),
       "REG %d <- 0x%02x", reg, value));

    switch (reg) {

    case ACE_REG_THR:
      if (ace->tme_ace_reg_lcr & LCR_DLAB) {
        ace->tme_ace_reg_div_lsb = value;
      } else {
        /*
         * XXX We do not properly emulate the THR clobber behavior when THRE
         * XXX is not set.  On a real 16550:
         * XXX
         * XXX - In 16450 mode (FIFO disabled), the new character overwrites
         * XXX   the previous (possibly being transmitted) character.
         * XXX
         * XXX - In 16550 mode (FIFO enabled), the new character goes into
         * XXX   the Tx FIFO, unless the FIFO is full in which case it
         * XXX   overwrites the most-recently added character.
         * XXX
         * XXX Our behavior here is that the new character is dropped.
         */
        if ((ace->tme_ace_reg_fcr & FCR_FIFOEN) != 0 /* 16550 mode */
            || tme_serial_buffer_is_empty(&ace->tme_ace_buffer_tx)) {
          /* copy in the new character: */
          tme_serial_buffer_copyin(&ace->tme_ace_buffer_tx,
                                   &buffer, 1,
                                   TME_SERIAL_DATA_NORMAL,
                                   TME_SERIAL_COPY_NORMAL);
        }

        /* transmiter is no longer empty. */
        ace->tme_ace_reg_lsr &= ~(LSR_THRE | LSR_TEMT);
        ace->tme_ace_internal_flags &= ~ACE_F_TXRDY_PENDING;

        /* must re-compute the interrupt output and callout the new control
           so that the connection can receive the newly-written data. */
        new_callouts |= TME_ACE_CALLOUT_INT | TME_ACE_CALLOUT_CTRL;
      }
      break;

    case ACE_REG_IER:
      if (ace->tme_ace_reg_lcr & LCR_DLAB) {
        ace->tme_ace_reg_div_msb = value;
      } else {
        value &= IER_mask;
        if (ace->tme_ace_reg_ier != value) {
          new_callouts |= TME_ACE_CALLOUT_INT;
          ace->tme_ace_reg_ier = value;
        }
      }
      break;

    case ACE_REG_FCR:
      value &= FCR_mask;

      /* "Bit 0 must be set when other FCR bits are written to or they
         are not programmed."  I'll take that to mean that their
         previously-set values do not change.
         XXX Should verify against a real 16550. */
      if ((value & FCR_FIFOEN) == 0) {
        value = ace->tme_ace_reg_fcr & ~FCR_FIFOEN;
      }

      /* "Changing this bit clears the FIFOs." */
      if ((value ^ ace->tme_ace_reg_fcr) & FCR_FIFOEN) {
        clear_tx_fifo = clear_rx_fifo = 1;
      }

      if (value & FCR_RFR) {
        clear_rx_fifo = 1;
      }
      if (value & FCR_TFR) {
        clear_tx_fifo = 1;
      }

      /* XXX Don't really do anything with the FIFO trigger; we always
         interrupt as soon as a character becomes available. */

      if (clear_rx_fifo) {
        new_callouts |= _tme_ace_reset_rx_fifo(ace);
      }

      if (clear_tx_fifo) {
        new_callouts |= _tme_ace_reset_tx_fifo(ace);
      }

      /* RFR and TFR are self-clearing. */
      value &= ~(FCR_RFR | FCR_TFR);

      ace->tme_ace_reg_fcr = value;
      break;

    case ACE_REG_LCR:
      /* if the data, stop, or parity config changed, we need a
         config callout. */
      if ((ace->tme_ace_reg_lcr ^ value)
          & ~(LCR_BREAK | LCR_DLAB)) {
        new_callouts |= TME_ACE_CALLOUT_CONFIG;
      }

      /* if the BREAK status changed, we need a control callout. */
      if ((ace->tme_ace_reg_lcr ^ value) & LCR_BREAK) {
        new_callouts |= TME_ACE_CALLOUT_CTRL;
      }

      ace->tme_ace_reg_lcr = value;
      break;

    case ACE_REG_MCR:
      if ((ace->tme_ace_reg_mcr ^ value) & (MCR_DTR | MCR_RTS)) {
        new_callouts |= TME_ACE_CALLOUT_CTRL;
      }
      ace->tme_ace_reg_mcr = value;
      break;

    case ACE_REG_LSR:
      /* writes to the LSR are ignored. */
      break;

    case ACE_REG_MSR:
      /* writes to the MSR are ignored. */
      break;

    case ACE_REG_SCR:
      ace->tme_ace_reg_scr = value;
      break;

    default:
      abort();
    }
  }

  /* otherwise, this is a read: */
  else {
    assert(cycle_init->tme_bus_cycle_type == TME_BUS_CYCLE_READ);

    switch (reg) {

    case ACE_REG_RBR:
      if (ace->tme_ace_reg_lcr & LCR_DLAB) {
        value = ace->tme_ace_reg_div_lsb;
      } else {
        /* return whatever's in the Rx FIFO right now and refill it: */
        value = ace->tme_ace_reg_rbr;
        new_callouts |= _tme_ace_rx_fifo_refill(ace);
     }
     break;

    case ACE_REG_IER:
      if (ace->tme_ace_reg_lcr & LCR_DLAB) {
        value = ace->tme_ace_reg_div_msb;
      } else {
        value = ace->tme_ace_reg_ier;
      }
      break;

    case ACE_REG_IIR:
      value = ace->tme_ace_reg_iir;

      /* If the IIR indicates TXRDY, then reading the IIR clears that
         condition.  */
      if (value == IIR_TXRDY) {
        ace->tme_ace_internal_flags &= ~ACE_F_TXRDY_PENDING;
        new_callouts |= TME_ACE_CALLOUT_INT;
      }

      if (ace->tme_ace_reg_fcr & FCR_FIFOEN) {
        value |= IIR_FIFOEN;
      }
      break;

    case ACE_REG_LCR:
      value = ace->tme_ace_reg_lcr;
      break;

    case ACE_REG_MCR:
      value = ace->tme_ace_reg_mcr;
      break;

    case ACE_REG_LSR:
      value = ace->tme_ace_reg_lsr;
 
      /* OE, PE, FE, and BI are cleared on every LSR read.
         XXX don't handle FIFO_ERR here, and we don't really emulate it.  */
      if (value & (LSR_OE | LSR_PE | LSR_FE | LSR_BI)) {
        ace->tme_ace_reg_lsr &= ~(LSR_OE | LSR_PE | LSR_FE | LSR_BI);
        new_callouts |= TME_ACE_CALLOUT_INT;
      }
      break;

    case ACE_REG_MSR:
      value = ace->tme_ace_reg_msr;
      /* DCTS, DDSR, TERI, and DDCD are cleared on every MSR read. */
      if (value & (MSR_DCTS | MSR_DDSR | MSR_TERI | MSR_DDCD)) {
        ace->tme_ace_reg_msr &= ~(MSR_DCTS | MSR_DDSR | MSR_TERI | MSR_DDCD);
        new_callouts |= TME_ACE_CALLOUT_INT;
      }
      break;

    case ACE_REG_SCR:
      value = ace->tme_ace_reg_scr;
      break;

    default:
      abort();
    }

#ifndef TME_NO_LOG
    /* log this read: */
    if (ace->tme_ace_last_read_reg != reg
        || ace->tme_ace_last_read_value != value) {
      ace->tme_ace_last_read_reg = reg;
      ace->tme_ace_last_read_value = value;
      tme_log(TME_ACE_LOG_HANDLE(ace), 100000, TME_OK,
	      (TME_ACE_LOG_HANDLE(ace),
         "REG %d -> 0x%02x", reg, value));
    }
#endif /* !TME_NO_LOG */

    /* run the bus cycle: */
    buffer = value;
    cycle_resp.tme_bus_cycle_buffer = &buffer;
    cycle_resp.tme_bus_cycle_lane_routing = tme_ace_router;
    cycle_resp.tme_bus_cycle_address = 0;
    cycle_resp.tme_bus_cycle_buffer_increment = 1;
    cycle_resp.tme_bus_cycle_type = TME_BUS_CYCLE_WRITE;
    cycle_resp.tme_bus_cycle_size = sizeof(buffer);
    cycle_resp.tme_bus_cycle_port = 
      TME_BUS_CYCLE_PORT(ace->tme_ace_port_least_lane,
			 TME_BUS8_LOG2);
    tme_bus_cycle_xfer(cycle_init, &cycle_resp);
  }
    
  /* make any needed callouts: */
  _tme_ace_callout(ace, new_callouts);

  /* unlock the mutex: */
  tme_mutex_unlock(&ace->tme_ace_mutex);

  /* no faults: */
  return (TME_OK);
}

/* this is called when the serial configuration changes: */
static int
_tme_ace_config(struct tme_serial_connection *conn_serial, 
                struct tme_serial_config *config)
{
  /* do nothing: */
  return (TME_OK);
}

/* this is called when control lines change: */
static int
_tme_ace_ctrl(struct tme_serial_connection *conn_serial, 
              unsigned int ctrl)
{
  struct tme_ace *ace;
  int new_callouts;
  tme_uint8_t new_msr;
  tme_uint8_t new_lsr_bi_bit;

  /* recover our data structures: */
  ace = conn_serial->tme_serial_connection.tme_connection_element->tme_element_private;

  /* lock the mutex: */
  tme_mutex_lock(&ace->tme_ace_mutex);

  /* assume that we won't need to make any callouts: */
  new_callouts = 0;

  new_msr = 0;
  new_lsr_bi_bit = 0;

  if (ctrl & TME_SERIAL_CTRL_DCD) {
    new_msr |= MSR_DCD;
  }

  if (ctrl & TME_SERIAL_CTRL_CTS) {
    new_msr |= MSR_CTS;
  }

  /* XXX this doesn't emulate BREAK quite correctly.  on a real 16550,
     LSR_BI has a slot in the FIFO just like all of the other LSR
     error bits (because it's really just a sort of framing error that
     persists for ~2 character times) and no additional data can enter
     the FIFO until the BREAK condition is cleared.

     Fixing this would require changing the TME serial emulation later
     to push BREAK conditions into the UART receive stream rather than
     treating them as a control signal.  */
  if (ctrl & TME_SERIAL_CTRL_BREAK) {
    new_lsr_bi_bit |= LSR_BI;
  }

  if (ctrl & TME_SERIAL_CTRL_RI) {
    new_msr |= MSR_RI;
  }

  /* note CTS and DCD changes. */
  new_msr |= ((ace->tme_ace_reg_msr ^ new_msr) & (MSR_CTS | MSR_DCD)) >> 4;

  /* TERI is only set on a rising edge. */
  if ((new_msr & MSR_RI) != 0
      && (ace->tme_ace_reg_msr & MSR_RI) == 0) {
    new_msr |= MSR_TERI;
  }

  if ((new_msr & (MSR_DCTS | MSR_DDSR | MSR_TERI | MSR_DDCD)) != 0
      || new_lsr_bi_bit != 0) {
    new_callouts |= TME_ACE_CALLOUT_INT;
  }

  /* if this channel is readable, call out to read data: */
  if (ctrl & TME_SERIAL_CTRL_OK_READ) {
    new_callouts |= TME_ACE_CALLOUT_READ;
  }

  /* if needed, make callouts: */
  _tme_ace_callout(ace, new_callouts);

  /* unlock the mutex: */
  tme_mutex_unlock(&ace->tme_ace_mutex);

  return (TME_OK);
}

/* this is called to read serial data (from the tl16c550 perspective,
   to transmit it): */
static int
_tme_ace_read(struct tme_serial_connection *conn_serial,
              tme_uint8_t *data,
              unsigned int count,
              tme_serial_data_flags_t *_data_flags)
{
  struct tme_ace *ace;
  int new_callouts;
  int rc;

  /* assume that we won't need to make any callouts: */
  new_callouts = 0;

  /* recover our data structures: */
  ace = conn_serial->tme_serial_connection.tme_connection_element->tme_element_private;

  /* lock our mutex: */
  tme_mutex_lock(&ace->tme_ace_mutex);

  /* if the Tx FIFO has been reset, just return 0 here; the serial engine
     will automatically drop the OK_READ indication, and we don't want
     to erroneously signal a TXRDY interrupt below. */
  if (tme_serial_buffer_is_empty(&ace->tme_ace_buffer_tx)) {
    tme_mutex_unlock(&ace->tme_ace_mutex);
    return (0);
  }

  /* copy out data from the Tx FIFO: */
  rc = tme_serial_buffer_copyout(&ace->tme_ace_buffer_tx,
                                 data,
                                 count,
                                 _data_flags,
                                 TME_SERIAL_COPY_NORMAL);

  /* if the Tx buffer is now empty: */
  if (tme_serial_buffer_is_empty(&ace->tme_ace_buffer_tx)) {

    /* update the LSR and note the rising edge of THRE: */
    ace->tme_ace_reg_lsr |= (LSR_THRE | LSR_TEMT);
    ace->tme_ace_internal_flags |= ACE_F_TXRDY_PENDING;

    /* callouts to clear OK_READ and signal TXRDY interrupt. */
    new_callouts |= TME_ACE_CALLOUT_INT | TME_ACE_CALLOUT_CTRL;
  }

  /* make any needed callouts: */
  _tme_ace_callout(ace, new_callouts);

  /* unlock our mutex: */
  tme_mutex_unlock(&ace->tme_ace_mutex);

  /* done: */
  return (rc);
}

/* the tl16c550 TLB filler: */
static int
_tme_ace_tlb_fill(void *_ace,
                  struct tme_bus_tlb *tlb,
                  tme_bus_addr_t address,
                  unsigned int cycles)
{
  struct tme_ace *ace;
  tme_bus_addr32_t address_last;

  /* recover our data structure: */
  ace = (struct tme_ace *) _ace;

  /* the address must be within range: */
  address_last = ace->tme_ace_device.tme_bus_device_address_last;
  assert(address <= address_last);

  /* initialize the TLB entry: */
  tme_bus_tlb_initialize(tlb);

  /* this TLB entry can cover the whole device: */
  tlb->tme_bus_tlb_addr_first = 0;
  tlb->tme_bus_tlb_addr_last = address_last;

  /* allow reading and writing: */
  tlb->tme_bus_tlb_cycles_ok = TME_BUS_CYCLE_READ | TME_BUS_CYCLE_WRITE;

  /* our bus cycle handler: */
  tlb->tme_bus_tlb_cycle_private = ace;
  tlb->tme_bus_tlb_cycle = _tme_ace_bus_cycle;

  return (TME_OK);
}

/* the tl16c550 bus signal handler: */
static int
_tme_ace_signal(void *_ace,
                unsigned int signal)
{
  struct tme_ace *ace;
  int new_callouts;
  unsigned int level;

  /* recover our data structure: */
  ace = (struct tme_ace *) _ace;

  /* assume we won't need any new callouts: */
  new_callouts = 0;

  /* lock the mutex: */
  tme_mutex_lock(&ace->tme_ace_mutex);

  /* take out the signal level: */
  level = signal & TME_BUS_SIGNAL_LEVEL_MASK;
  signal = TME_BUS_SIGNAL_WHICH(signal);

  /* dispatch on the generic bus signals: */
  switch (signal) {

  case TME_BUS_SIGNAL_RESET:
    if (level == TME_BUS_SIGNAL_LEVEL_ASSERTED) {
      new_callouts |= _tme_ace_reset(ace);
    }
    break;

  default:
    break;
  }

  /* make any new callouts: */
  _tme_ace_callout(ace, new_callouts);

  /* unlock the mutex: */
  tme_mutex_unlock(&ace->tme_ace_mutex);

  /* no faults: */
  return (TME_OK);
}

/* this scores a serial connection: */
static int
_tme_ace_connection_score(struct tme_connection *conn,
                          unsigned int *_score)
{
  struct tme_ace *ace;

  /* recover our data structures: */
  ace = conn->tme_connection_element->tme_element_private;

  /* both sides must be serial connections: */
  assert(conn->tme_connection_type == TME_CONNECTION_SERIAL);
  assert(conn->tme_connection_other->tme_connection_type == TME_CONNECTION_SERIAL);

  /* this channel must be free: */
  assert(ace->tme_ace_connection == NULL);
  (void)ace;

  /* we're lax on checking the members of the serial connection, 
     and just assume this connection is fine: */
  *_score = 1;
  return (TME_OK);
}

/* this makes a new serial connection: */
static int
_tme_ace_connection_make(struct tme_connection *conn, unsigned int state)
{
  struct tme_ace *ace;
  struct tme_serial_connection *conn_serial_other;

  /* recover our data structures: */
  ace = conn->tme_connection_element->tme_element_private;
  conn_serial_other = (struct tme_serial_connection *) conn->tme_connection_other;

  /* both sides must be serial connections: */
  assert(conn->tme_connection_type == TME_CONNECTION_SERIAL);
  assert(conn->tme_connection_other->tme_connection_type == TME_CONNECTION_SERIAL);

  /* we're always set up to answer calls across the connection, so we
     only have to do work when the connection has gone full, namely
     taking the other side of the connection: */
  if (state == TME_CONNECTION_FULL) {

    /* save our connection: */
    ace->tme_ace_connection = conn_serial_other;
  }

  return (TME_OK);
}

/* this breaks a connection: */
static int
_tme_ace_connection_break(struct tme_connection *conn,
                          unsigned int state)
{
  abort();
}

/* this makes a new connection side for a tl16c550: */
static int
_tme_ace_connections_new(struct tme_element *element,
                         const char * const *args,
                         struct tme_connection **_conns,
                         char **_output)
{
  struct tme_ace *ace;
  struct tme_serial_connection *conn_serial;
  struct tme_connection *conn;
  int rc;

  /* recover our data structure: */
  ace = (struct tme_ace *) element->tme_element_private;

  /* make the generic bus device connection side: */
  rc = tme_bus_device_connections_new(element, args, _conns, _output);
  if (rc != TME_OK) {
    return (rc);
  }

  /* if we don't have a serial connection, make one. */
  if (ace->tme_ace_connection == NULL) {

    /* allocate the new serial connection: */
    conn_serial = tme_new0(struct tme_serial_connection, 1);
    conn = &conn_serial->tme_serial_connection;
    
    /* fill in the generic connection: */
    conn->tme_connection_next = *_conns;
    conn->tme_connection_type = TME_CONNECTION_SERIAL;
    conn->tme_connection_score = _tme_ace_connection_score;
    conn->tme_connection_make = _tme_ace_connection_make;
    conn->tme_connection_break = _tme_ace_connection_break;

    /* fill in the serial connection: */
    conn_serial->tme_serial_connection_config = _tme_ace_config;
    conn_serial->tme_serial_connection_ctrl = _tme_ace_ctrl;
    conn_serial->tme_serial_connection_read = _tme_ace_read;

    /* return the connection side possibility: */
    *_conns = conn;
  }

  /* done: */
  return (TME_OK);
}

/* the new tl16c550 function: */
TME_ELEMENT_NEW_DECL(tme_ic_tl16c550) {
  const struct tme_tl16c550_socket *socket;
  struct tme_ace *ace;
  struct tme_tl16c550_socket socket_real;
  tme_bus_addr_t address_mask;

  /* dispatch on our socket version: */
  socket = (const struct tme_tl16c550_socket *) extra;
  if (socket == NULL) {
    tme_output_append_error(_output, _("need an ic socket"));
    return (ENXIO);
  }
  switch (socket->tme_tl16c550_socket_version) {
  case TME_TL16C550_SOCKET_0:
    socket_real = *socket;
    break;
  default: 
    tme_output_append_error(_output, _("socket type"));
    return (EOPNOTSUPP);
  }
    
  /* we take no arguments: */
  if (args[1] != NULL) {
    tme_output_append_error(_output,
			    "%s %s, %s %s",
			    args[1],
			    _("unexpected"),
			    _("usage:"),
			    args[0]);
    return (EINVAL);
  }

  /* start the tl16c550 structure: */
  ace = tme_new0(struct tme_ace, 1);
  ace->tme_ace_socket = socket_real;
  tme_mutex_init(&ace->tme_ace_mutex);

  /* allocate the Tx and Rx FIFOs: */  
  tme_serial_buffer_init(&ace->tme_ace_buffer_tx, 
                         TME_ACE_BUFFER_SIZE_TX);
  tme_serial_buffer_init(&ace->tme_ace_buffer_rx, 
                         TME_ACE_BUFFER_SIZE_RX);

  /* reset the UART registers: */
  _tme_ace_reset_registers(ace);

  /* figure our address mask, up to the nearest power of two: */
  address_mask = (ACE_NUM_REGS << ace->tme_ace_addr_shift) - 1;

  /* initialize our simple bus device descriptor: */
  ace->tme_ace_device.tme_bus_device_element = element;
  ace->tme_ace_device.tme_bus_device_tlb_fill = _tme_ace_tlb_fill;
  ace->tme_ace_device.tme_bus_device_signal = _tme_ace_signal;
  ace->tme_ace_device.tme_bus_device_address_last = address_mask;

  /* fill the element: */
  element->tme_element_private = ace;
  element->tme_element_connections_new = _tme_ace_connections_new;

  return (TME_OK);
}
