/* ic/pcf8584.c - implementation of NXP PCF8584 emulation: */

/*
 * Copyright (c) 2026 Jason R. Thorpe
 * All rights reserved.
 */

#include <tme/common.h>

/* includes: */
#include <tme/generic/bus-device.h>
#include <tme/generic/i2c.h>
#include <tme/ic/pcf8584.h>

/* macros: */

#define	PCF8584_CTRL_ACK	TME_BIT(0)
#define	PCF8584_CTRL_STO	TME_BIT(1)
#define	PCF8584_CTRL_STA	TME_BIT(2)
#define	PCF8584_CTRL_ENI	TME_BIT(3)
#define	PCF8584_CTRL_ES2	TME_BIT(4)
#define	PCF8584_CTRL_ES1	TME_BIT(5)
#define	PCF8584_CTRL_ESO	TME_BIT(6)
#define	PCF8584_CTRL_PIN	TME_BIT(7)

#define	PCF8584_STATUS_BBN	TME_BIT(0)
#define	PCF8584_STATUS_LAB	TME_BIT(1)
#define	PCF8584_STATUS_AAS	TME_BIT(2)
#define	PCF8584_STATUS_LRB	TME_BIT(3)
#define	PCF8584_STATUS_BER	TME_BIT(4)
#define	PCF8584_STATUS_STS	TME_BIT(5)
#define	PCF8584_STATUS_INI	TME_BIT(6)
#define	PCF8584_STATUS_PIN	TME_BIT(7)

#define PCF_NUM_REGS 2

#define TME_PCF_CALLOUT_CHECK   (0)
#define TME_PCF_CALLOUT_RUNNING TME_BIT(0)
#define TME_PCF_CALLOUTS_MASK   -2
#define TME_PCF_CALLOUT_CTRL    TME_BIT(1)
#define TME_PCF_CALLOUT_DOUT    TME_BIT(2)
#define TME_PCF_CALLOUT_DIN     TME_BIT(3)
#define TME_PCF_CALLOUT_INT     TME_BIT(4)

#define TME_PCF_LOG_HANDLE(p)  (&(p)->tme_pcf_element->tme_element_log_handle)

#define PCF_REG_NONE        -1
#define PCF_REG_DATA        0   /* S0 */
#define PCF_REG_OWN_ADDR    1   /* S0' */
#define PCF_REG_CTRL        2   /* S1 */
#define PCF_REG_STAT        3   /* S1 */
#define PCF_REG_CLOCK       4   /* S2 */
#define PCF_REG_IVEC        5   /* S3 */
#define PCF_NREGS           6

static const int _tme_pcf_reg_write_callouts[PCF_NREGS] = {
  TME_PCF_CALLOUT_DOUT,
  0,
  TME_PCF_CALLOUT_CTRL,
  0,
  0,
  0,
};

static const int _tme_pcf_reg_read_callouts[PCF_NREGS] = {
  TME_PCF_CALLOUT_DIN,
  0,
  0,
  0,
  0,
  0,
};

static const char * const _tme_pcf8584_reg_names[PCF_NREGS] = {
  "DATA",
  "OWN-ADDR",
  "CTRL",
  "STAT",
  "CLOCK",
  "IVEC",
};

static const char *
_tme_pcf8584_reg_name(unsigned int r)
{
  if (r < PCF_NREGS) {
    return _tme_pcf8584_reg_names[r];
  }
  return "**junk**";
}

#define PCF_MODE_SLV_REC    0   /* idle mode */
#define PCF_MODE_SLV_TRM    1
#define PCF_MODE_MST_REC    2
#define PCF_MODE_MST_TRM    3

#define PCF_MODE_TRM_P(pcf) ((pcf)->tme_pcf_mode & 1)
#define PCF_MODE_REC_P(pcf) (!PCF_MODE_TRM_P(pcf))
#define PCF_MODE_MST_P(pcf) ((pcf)->tme_pcf_mode & 2)
#define	PCF_MODE_SLV_P(pcf) (!PCF_MODE_MST_P(pcf))

/* structures: */

/* the chip: */
struct tme_pcf {

  /* our simple bus device header: */
  struct tme_bus_device tme_pcf_device;
#define tme_pcf_element tme_pcf_device.tme_bus_device_element

  /* our socket: */
  struct tme_pcf8584_socket tme_pcf_socket;
#define tme_pcf_addr_shift      tme_pcf_socket.tme_pcf8584_socket_addr_shift
#define tme_pcf_port_least_lane tme_pcf_socket.tme_pcf8584_socket_port_least_lane

  /* the mutex protecting the chip: */
  tme_mutex_t tme_pcf_mutex;

  /* our registers */
  tme_uint8_t tme_pcf_registers[PCF_NREGS];

  int tme_pcf_mode;
  int tme_pcf_repeated_start;

  /* our callout flags: */
  int tme_pcf_callout_flags;

  /* our interrupt status: */
  int tme_pcf_int_asserted;

  /* our current connection; may be NULL: */
  struct tme_i2c_connection *tme_pcf_i2c_nexus;

  /* our list of connections: */
  struct tme_i2c_connection *tme_pcf_i2c_connections;

#ifndef TME_NO_LOG
  tme_uint8_t tme_pcf_last_read_reg;
  tme_uint8_t tme_pcf_last_read_value;
#endif /* !TME_NO_LOG */
};

/* the pcf8584 bus router */
static const tme_bus_lane_t tme_pcf_router[TME_BUS_ROUTER_SIZE(TME_BUS8_LOG2)] =
 {

  /* [gen]  initiator port size: 8 bits
     [gen]  initiator port least lane: 0: */
  /* D7-D0 */ TME_BUS_LANE_ROUTE(0),
};

static int
_tme_pcf_which_reg(struct tme_pcf *pcf, int a0, int is_read)
{
  const tme_uint8_t ctrl = pcf->tme_pcf_registers[PCF_REG_CTRL];

  if (a0 & 1) {
    if ((ctrl & PCF8584_CTRL_ESO) != 0 && is_read) {
      return PCF_REG_STAT;
    }
    return PCF_REG_CTRL;
  }

  switch (ctrl & (PCF8584_CTRL_ESO|PCF8584_CTRL_ES1|PCF8584_CTRL_ES2)) {
  case 0:
    return PCF_REG_OWN_ADDR;

  case PCF8584_CTRL_ES2:
    return PCF_REG_IVEC;

  case PCF8584_CTRL_ES1:
    return PCF_REG_CLOCK;

  case PCF8584_CTRL_ESO:
    return PCF_REG_DATA;

  case PCF8584_CTRL_ESO|PCF8584_CTRL_ES1:
  case PCF8584_CTRL_ESO|PCF8584_CTRL_ES1|PCF8584_CTRL_ES2:
    return PCF_REG_DATA;

  case PCF8584_CTRL_ESO|PCF8584_CTRL_ES2:
    return PCF_REG_IVEC;

  default:
    return PCF_REG_NONE;
  }
}

static void
_tme_pcf_send_start(struct tme_pcf *pcf)
{
  struct tme_i2c_connection *conn_i2c = pcf->tme_pcf_i2c_nexus;
  const tme_uint8_t addr = pcf->tme_pcf_registers[PCF_REG_DATA];
  int rc = ESRCH;

  /* if we already have a nexus established, we send only to that one. */
  if (conn_i2c != NULL) {

    tme_mutex_unlock(&pcf->tme_pcf_mutex);
    rc = (*conn_i2c->tme_i2c_connection_start)(conn_i2c, addr);
    tme_mutex_lock(&pcf->tme_pcf_mutex);

  } else {

    /* find a nexus for this transfer. */
    for (conn_i2c = pcf->tme_pcf_i2c_connections;
         conn_i2c != NULL;
         conn_i2c = conn_i2c->tme_i2c_connection_next) {
      tme_mutex_unlock(&pcf->tme_pcf_mutex);
      rc = (*conn_i2c->tme_i2c_connection_start)(conn_i2c, addr);
      tme_mutex_lock(&pcf->tme_pcf_mutex);
      if (rc == TME_OK) {
        pcf->tme_pcf_i2c_nexus = conn_i2c;
        break;
      }
    }
  }

  if (rc != TME_OK) {
    /* NACK */
    pcf->tme_pcf_registers[PCF_REG_STAT] |= PCF8584_STATUS_LRB;
  } else {
    pcf->tme_pcf_mode = (addr & 1) ? PCF_MODE_MST_REC : PCF_MODE_MST_TRM;
  }
  pcf->tme_pcf_registers[PCF_REG_STAT] &= ~PCF8584_STATUS_PIN;
}

static void
_tme_pcf_send_stop(struct tme_pcf *pcf)
{
  struct tme_i2c_connection *conn_i2c = pcf->tme_pcf_i2c_nexus;

  tme_mutex_unlock(&pcf->tme_pcf_mutex);

  if (conn_i2c != NULL) {
    (*conn_i2c->tme_i2c_connection_stop)(conn_i2c);
  }

  tme_mutex_lock(&pcf->tme_pcf_mutex);

  pcf->tme_pcf_i2c_nexus = NULL;
  pcf->tme_pcf_mode = PCF_MODE_SLV_REC;
  pcf->tme_pcf_registers[PCF_REG_STAT] |= PCF8584_STATUS_PIN;
}

static void
_tme_pcf_data_out(struct tme_pcf *pcf)
{
  struct tme_i2c_connection *conn_i2c = pcf->tme_pcf_i2c_nexus;
  const tme_uint8_t data = pcf->tme_pcf_registers[PCF_REG_DATA];
  int rc;

  tme_mutex_unlock(&pcf->tme_pcf_mutex);

  rc = (conn_i2c != NULL
    ? (*conn_i2c->tme_i2c_connection_write)(conn_i2c, data)
    : ESRCH);

  tme_mutex_lock(&pcf->tme_pcf_mutex);

  if (rc == TME_OK) {
    pcf->tme_pcf_registers[PCF_REG_STAT] &= ~PCF8584_STATUS_LRB;
  } else {
    pcf->tme_pcf_registers[PCF_REG_STAT] |= PCF8584_STATUS_LRB;
  }
  pcf->tme_pcf_registers[PCF_REG_STAT] &= ~PCF8584_STATUS_PIN;
}

static void
_tme_pcf_data_in(struct tme_pcf *pcf)
{
  struct tme_i2c_connection *conn_i2c = pcf->tme_pcf_i2c_nexus;
  tme_uint8_t data;
  int rc;
  const int nack
    = (pcf->tme_pcf_registers[PCF_REG_CTRL] & PCF8584_CTRL_ACK) == 0;

  tme_mutex_unlock(&pcf->tme_pcf_mutex);

  rc = (conn_i2c != NULL
    ? (*conn_i2c->tme_i2c_connection_read)(conn_i2c, &data, nack)
    : ESRCH);

  tme_mutex_lock(&pcf->tme_pcf_mutex);

  if (rc == TME_OK) {
    pcf->tme_pcf_registers[PCF_REG_STAT] &= ~PCF8584_STATUS_LRB;
    pcf->tme_pcf_registers[PCF_REG_DATA] = data;
  } else {
    pcf->tme_pcf_registers[PCF_REG_STAT] |= PCF8584_STATUS_LRB;
    pcf->tme_pcf_registers[PCF_REG_DATA] = 0xff;  /* all pulled high */
  }
  pcf->tme_pcf_registers[PCF_REG_STAT] &= ~PCF8584_STATUS_PIN;
}

/* the pcf8584 callout function.  it must be called with the mutex locked: */
static void
_tme_pcf_callout(struct tme_pcf *pcf,
                 int new_callouts)
{
  struct tme_bus_connection *conn_bus;
  int callouts;
  int later_callouts;
  int int_asserted;
  int rc;

  /* add in any new callouts: */
  pcf->tme_pcf_callout_flags |= new_callouts;

  /* if this function is already running in another thread, return
     now.  the other thread will do our work: */
  if (pcf->tme_pcf_callout_flags & TME_PCF_CALLOUT_RUNNING) {
    return;
  }

  /* callouts are now running: */
  pcf->tme_pcf_callout_flags |= TME_PCF_CALLOUT_RUNNING;

  /* assume that we won't need any later callouts: */
  later_callouts = 0;

  /* loop while callouts are needed: */
  for (;;) {

    callouts = pcf->tme_pcf_callout_flags & TME_PCF_CALLOUTS_MASK;
    if (callouts == 0) {
      break;
    }

    /* clear the needed callouts: */
    pcf->tme_pcf_callout_flags &= ~TME_PCF_CALLOUTS_MASK;

    if (callouts & TME_PCF_CALLOUT_CTRL) {

      const tme_uint8_t ctrl = pcf->tme_pcf_registers[PCF_REG_CTRL];

      /* setting PIN resets all status bits. */
      if (ctrl & PCF8584_CTRL_PIN) {
        pcf->tme_pcf_registers[PCF_REG_STAT]
          = PCF8584_STATUS_PIN | PCF8584_STATUS_BBN;
        pcf->tme_pcf_registers[PCF_REG_CTRL] &= ~PCF8584_CTRL_PIN;
      }

      /*
       * The actions to be taken here are described in Table 7
       * of the PCF8584 data sheet.  All other state / bit
       * combinations are a NOP.
       */
      switch (pcf->tme_pcf_mode) {
      case PCF_MODE_SLV_REC:
        if ((ctrl & (PCF8584_CTRL_STA|PCF8584_CTRL_STO)) == PCF8584_CTRL_STA) {
          /* Data register has slave address + rw bit.  Go find
             a nexus.  */
          _tme_pcf_send_start(pcf);
        }
        break;

      case PCF_MODE_MST_TRM:
      case PCF_MODE_MST_REC:
        switch (ctrl & (PCF8584_CTRL_STA|PCF8584_CTRL_STO)) {
        case PCF8584_CTRL_STA:
          /* This is a repeated start condition if we're a MST_TRM.  We
              have to wait for the slave address and R/W bit to be written
              to the data register. */
          if (PCF_MODE_TRM_P(pcf)) {
            pcf->tme_pcf_repeated_start = 1;
            pcf->tme_pcf_registers[PCF_REG_STAT] &= ~PCF8584_STATUS_PIN;
          }
          break;

        case PCF8584_CTRL_STO:
          _tme_pcf_send_stop(pcf);
          break;

        case PCF8584_CTRL_STA|PCF8584_CTRL_STO:
          /* this is a chained transmission.  Send a STOP and then
             immediately with a START.  (Good luck with that DATA
             register contents I guess?) */
          _tme_pcf_send_stop(pcf);
          _tme_pcf_send_start(pcf);
          break;

        default:
          break;
        }
        break;

      default:
        break;
      }
    }

    if (callouts & TME_PCF_CALLOUT_DOUT) {
      if (pcf->tme_pcf_repeated_start) {
        pcf->tme_pcf_repeated_start = 0;
        /* Data register has slave address + rw bit.  Go find
           a nexus.  */
        _tme_pcf_send_start(pcf);
      } else {
        _tme_pcf_data_out(pcf);
      }
      pcf->tme_pcf_registers[PCF_REG_STAT] &= ~PCF8584_STATUS_PIN;
    }

    if (callouts & TME_PCF_CALLOUT_DIN) {
      _tme_pcf_data_in(pcf);
      pcf->tme_pcf_registers[PCF_REG_STAT] &= ~PCF8584_STATUS_PIN;
    }
  }

  /* All callouts require processing interrupts. */
  int_asserted
    = (pcf->tme_pcf_registers[PCF_REG_STAT] & PCF8584_STATUS_PIN) == 0
    && (pcf->tme_pcf_registers[PCF_REG_CTRL] & PCF8584_CTRL_ENI) != 0;

  if (int_asserted != pcf->tme_pcf_int_asserted) {

    /* unlock our mutex: */
    tme_mutex_unlock(&pcf->tme_pcf_mutex);

    /* get our bus connection: */
    conn_bus
        = tme_memory_atomic_pointer_read(struct tme_bus_connection *,
            pcf->tme_pcf_device.tme_bus_device_connection,
            &pcf->tme_pcf_device.tme_bus_device_connection_rwlock);

    /* call out the bus interrupt signal: */
    rc = (*conn_bus->tme_bus_signal)
      (conn_bus,
       TME_BUS_SIGNAL_INT_UNSPEC
       | (int_asserted
          ? TME_BUS_SIGNAL_LEVEL_ASSERTED
          : TME_BUS_SIGNAL_LEVEL_NEGATED));

    /* lock our mutex: */
    tme_mutex_lock(&pcf->tme_pcf_mutex);

    /* if this callout was successful, note the new state of the
       interrupt signal: */
    if (rc == TME_OK) {
      pcf->tme_pcf_int_asserted = int_asserted;
    }

    /* otherwise, remember that at some later time this callout
       should be attempted again: */
    else {
      later_callouts |= TME_PCF_CALLOUT_INT;
    }
  }

  /* put in any later callouts, and clear that callouts are running: */
  pcf->tme_pcf_callout_flags = later_callouts;
}

/* the pcf8584 bus cycle handler: */
static int
_tme_pcf_bus_cycle(void *_pcf,
                   struct tme_bus_cycle *cycle_init)
{
  struct tme_pcf *pcf;
  tme_bus_addr32_t address, pcf_address_last;
  tme_uint8_t buffer, value;
  struct tme_bus_cycle cycle_resp;
  int new_callouts;
  int reg, pcf_reg;

  /* recover our data structure: */
  pcf = (struct tme_pcf *) _pcf;

  /* the requested cycle must be within range: */
  pcf_address_last = pcf->tme_pcf_device.tme_bus_device_address_last;
  address = cycle_init->tme_bus_cycle_address;
  assert(address <= pcf_address_last);
  assert(cycle_init->tme_bus_cycle_size <= (pcf_address_last - address) + 1);
  (void)pcf_address_last;

  /* get the register being accessed: */
  reg = address >> pcf->tme_pcf_addr_shift;

  /* lock the mutex: */
  tme_mutex_lock(&pcf->tme_pcf_mutex);

  /* assume we won't need any new callouts: */
  new_callouts = 0;

  pcf_reg = _tme_pcf_which_reg(pcf, reg,
    cycle_init->tme_bus_cycle_type == TME_BUS_CYCLE_READ);

  /* if this is a write: */
  if (cycle_init->tme_bus_cycle_type == TME_BUS_CYCLE_WRITE) {

    /* run the bus cycle: */
    cycle_resp.tme_bus_cycle_buffer = &buffer;
    cycle_resp.tme_bus_cycle_lane_routing = tme_pcf_router;
    cycle_resp.tme_bus_cycle_address = 0;
    cycle_resp.tme_bus_cycle_buffer_increment = 1;
    cycle_resp.tme_bus_cycle_type = TME_BUS_CYCLE_READ;
    cycle_resp.tme_bus_cycle_size = sizeof(buffer);
    cycle_resp.tme_bus_cycle_port =
      TME_BUS_CYCLE_PORT(pcf->tme_pcf_port_least_lane,
       TME_BUS8_LOG2);
    tme_bus_cycle_xfer(cycle_init, &cycle_resp);
    value = buffer;

    /* log this write: */
    tme_log(TME_PCF_LOG_HANDLE(pcf), 0, TME_OK,
      (TME_PCF_LOG_HANDLE(pcf),
       "[%s] <- 0x%02x", _tme_pcf8584_reg_name(pcf_reg), value));

    switch (pcf_reg) {
    case PCF_REG_STAT:
    case PCF_REG_NONE:
      /* ignore cycle */
      break;

    default:
      pcf->tme_pcf_registers[pcf_reg] = value;
      new_callouts |= _tme_pcf_reg_write_callouts[pcf_reg];
      break;
    }
  }

  /* otherwise, this is a read: */
  else {
    assert(cycle_init->tme_bus_cycle_type == TME_BUS_CYCLE_READ);

    switch (pcf_reg) {
    case PCF_REG_NONE:
      /* junk cycle */
      value = 0xff;
      break;

    default:
      value = pcf->tme_pcf_registers[pcf_reg];
      new_callouts |= _tme_pcf_reg_read_callouts[pcf_reg];
      break;
    }

#ifndef TME_NO_LOG
    /* log this read: */
    if (pcf->tme_pcf_last_read_reg != pcf_reg
        || pcf->tme_pcf_last_read_value != value) {
      pcf->tme_pcf_last_read_reg = pcf_reg;
      pcf->tme_pcf_last_read_value = value;
      tme_log(TME_PCF_LOG_HANDLE(pcf), 0, TME_OK,
        (TME_PCF_LOG_HANDLE(pcf),
         "[%s] -> 0x%02x", _tme_pcf8584_reg_name(pcf_reg), value));
    }
#endif /* !TME_NO_LOG */

    /* run the bus cycle: */
    buffer = value;
    cycle_resp.tme_bus_cycle_buffer = &buffer;
    cycle_resp.tme_bus_cycle_lane_routing = tme_pcf_router;
    cycle_resp.tme_bus_cycle_address = 0;
    cycle_resp.tme_bus_cycle_buffer_increment = 1;
    cycle_resp.tme_bus_cycle_type = TME_BUS_CYCLE_WRITE;
    cycle_resp.tme_bus_cycle_size = sizeof(buffer);
    cycle_resp.tme_bus_cycle_port =
      TME_BUS_CYCLE_PORT(pcf->tme_pcf_port_least_lane,
       TME_BUS8_LOG2);
    tme_bus_cycle_xfer(cycle_init, &cycle_resp);
  }

  /* make any needed callouts */
  _tme_pcf_callout(pcf, new_callouts);

  /* unlock the mutex: */
  tme_mutex_unlock(&pcf->tme_pcf_mutex);

  /* no faults: */
  return (TME_OK);
}

/* the pcf8584 TLB filler: */
static int
_tme_pcf_tlb_fill(void *_pcf,
                  struct tme_bus_tlb *tlb,
                  tme_bus_addr_t address,
                  unsigned int cycles)
{
  struct tme_pcf *pcf;
  tme_bus_addr32_t address_last;

  /* recover our data structure: */
  pcf = (struct tme_pcf *) _pcf;

  /* the address must be within range: */
  address_last = pcf->tme_pcf_device.tme_bus_device_address_last;
  assert(address <= address_last);

  /* initialize the TLB entry: */
  tme_bus_tlb_initialize(tlb);

  /* this TLB entry can cover the whole device: */
  tlb->tme_bus_tlb_addr_first = 0;
  tlb->tme_bus_tlb_addr_last = address_last;

  /* allow reading and writing: */
  tlb->tme_bus_tlb_cycles_ok = TME_BUS_CYCLE_READ | TME_BUS_CYCLE_WRITE;

  /* our bus cycle handler: */
  tlb->tme_bus_tlb_cycle_private = pcf;
  tlb->tme_bus_tlb_cycle = _tme_pcf_bus_cycle;

  return (TME_OK);
}

/* the pcf8584 bus signal handler: */
static int
_tme_pcf_signal(void *_pcf,
                unsigned int signal)
{
  /* XXX */

  /* no faults: */
  return (TME_OK);
}

/* this scores an i2c connection: */
static int
_tme_pcf_connection_score(struct tme_connection *conn,
                          unsigned int *_score)
{
  /* both sides must be i2c connections: */
  assert(conn->tme_connection_type == TME_CONNECTION_I2C);
  assert(conn->tme_connection_other->tme_connection_type == TME_CONNECTION_I2C);

  /* I2C masters are connected to multiple slave devices. */

  *_score = 1;
  return (TME_OK);
}

/* this makes a new i2c connection: */
static int
_tme_pcf_connection_make(struct tme_connection *conn, unsigned int state)
{
  struct tme_pcf *pcf;
  struct tme_i2c_connection *conn_i2c_other;

  pcf = conn->tme_connection_element->tme_element_private;
  conn_i2c_other = (struct tme_i2c_connection *) conn->tme_connection_other;

  /* both sides must be i2c connections: */
  assert(conn->tme_connection_type == TME_CONNECTION_I2C);
  assert(conn->tme_connection_other->tme_connection_type == TME_CONNECTION_I2C);

  if (state == TME_CONNECTION_FULL) {
    /* link it into the list. */
    conn_i2c_other->tme_i2c_connection_next = pcf->tme_pcf_i2c_connections;
    pcf->tme_pcf_i2c_connections = conn_i2c_other;
  }

  return (TME_OK);
}

/* this breaks an i2c connection: */
static int
_tme_pcf_connection_break(struct tme_connection *conn,
                          unsigned int state)
{
  abort();
}

static int
_tme_pcf_slave_start(struct tme_i2c_connection *conn_i2c, tme_uint8_t addr)
{
  return (EOPNOTSUPP);
}

static int
_tme_pcf_slave_stop(struct tme_i2c_connection *conn_i2c)
{
  return (EOPNOTSUPP);
}

static int
_tme_pcf_slave_write(struct tme_i2c_connection *conn_i2c, tme_uint8_t data)
{
  return (EOPNOTSUPP);
}

static int
_tme_pcf_slave_read(struct tme_i2c_connection *conn_i2c, tme_uint8_t *datap,
                    int nack)
{
  return (EOPNOTSUPP);
}

/* this makes a new connection side for a pcf8584: */
static int
_tme_pcf_connections_new(struct tme_element *element,
                         const char * const *args,
                         struct tme_connection **_conns,
                         char **_output)
{
  struct tme_i2c_connection *conn_i2c;
  struct tme_connection *conn;
  int rc;

  /* make the generic bus device connection side: */
  rc = tme_bus_device_connections_new(element, args, _conns, _output);
  if (rc != TME_OK) {
    return (rc);
  }

  /* make our side of an i2c connection: */
  conn_i2c = tme_new0(struct tme_i2c_connection, 1);
  conn = &conn_i2c->tme_i2c_connection;

  /* fill in the generic connection: */
  conn->tme_connection_next = *_conns;
  conn->tme_connection_type = TME_CONNECTION_I2C;
  conn->tme_connection_score = _tme_pcf_connection_score;
  conn->tme_connection_make = _tme_pcf_connection_make;
  conn->tme_connection_break = _tme_pcf_connection_break;

  /* fill in the i2c connection: */
  conn_i2c->tme_i2c_connection_start = _tme_pcf_slave_start;
  conn_i2c->tme_i2c_connection_stop = _tme_pcf_slave_stop;
  conn_i2c->tme_i2c_connection_write = _tme_pcf_slave_write;
  conn_i2c->tme_i2c_connection_read = _tme_pcf_slave_read;

  /* return the connection side possibility: */
  *_conns = conn;

  /* done: */
  return (TME_OK);
}

/* the new pcf8584 function: */
TME_ELEMENT_NEW_DECL(tme_ic_pcf8584) {
  const struct tme_pcf8584_socket *socket;
  struct tme_pcf *pcf;
  struct tme_pcf8584_socket socket_real;
  tme_bus_addr_t address_mask;

  /* dispatch on our socket version: */
  socket = (const struct tme_pcf8584_socket *) extra;
  if (socket == NULL) {
    tme_output_append_error(_output, _("need an ic socket"));
    return (ENXIO);
  }
  switch (socket->tme_pcf8584_socket_version) {
  case TME_PCF8584_SOCKET_0:
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

  /* start of the pcf8584 structure: */
  pcf = tme_new0(struct tme_pcf, 1);
  pcf->tme_pcf_socket = socket_real;
  tme_mutex_init(&pcf->tme_pcf_mutex);

  /* figure our address mask, up to the nearest power of two: */
  address_mask = (PCF_NUM_REGS << pcf->tme_pcf_addr_shift) - 1;

  /* initialize our simple bus device descriptor: */
  pcf->tme_pcf_device.tme_bus_device_element = element;
  pcf->tme_pcf_device.tme_bus_device_tlb_fill = _tme_pcf_tlb_fill;
  pcf->tme_pcf_device.tme_bus_device_signal = _tme_pcf_signal;
  pcf->tme_pcf_device.tme_bus_device_address_last = address_mask;

  /* fill the element: */
  element->tme_element_private = pcf;
  element->tme_element_connections_new = _tme_pcf_connections_new;

  return (TME_OK);
}
