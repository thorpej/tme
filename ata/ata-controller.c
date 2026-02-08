/* ata/ata-controller.c - implementation of ATA / IDE disk
   controller emulation: */

/*
 * Copyright (c) 2026 Jason R. Thorpe
 * All rights reserved.
 */

#include <tme/common.h>

/* includes: */
#include <tme/generic/bus-device.h>
#include <tme/ata/ata-controller.h>

/* macros: */

/* ATA disk controller registers, mostly aligned with the WDC1003.
   It's important to remember that with ATA / IDE, the controller isn't
   down on the board, it's at the other end of a cable, on the drive.

   On a WDC1003, it would have been a real controller, doing all the work
   to control an ST506 or similar.  But we're emulating the post WDC1003
   universe where cheap spinning-rust drives and CompactFlash cards reign
   supreme.

	 Also note that ATA has two chip select signals:

   ==> CS1FX- for the 8 command block registers (at IO 0x1f0 on the
       PC/AT).

	 ==> CS3FX- for the 8 control block registers (at IO 0x3f0 on the
       PC/AT).

	 XXX TME doesn't have an easy way, as far as I can tell, to model
   XXX this.  Should fix this one day, but for the Phaethon 1, where
   XXX the two blocks are directly adjacent, I don't need to worry
   XXX about it.  */

/* basic sector size: */
#define	wd_sector_size	512

/* command block: */
#define wd_data         0   /* data register (R/W - 16 bits) */
#define wd_error        1   /* error register (R) */
#define wd_features     1   /* features (W) */
#define wd_seccnt       2   /* sector count (R/W) */
#define wd_ireason      2   /* interrupt reason (R/W) (for atapi) */
#define wd_sector       3   /* first sector number (R/W) */
#define wd_cyl_lo       4   /* cylinder address, low byte (R/W) */
#define wd_cyl_hi       5   /* cylinder address, high byte (R/W) */
#define wd_sdh          6   /* sector size/drive/head (R/W) */
#define wd_command      7   /* command register (W) */
#define wd_status       7   /* status (R) */

#define wd_command_num_regs 8

/* control block: */
/* 0-5 read back high-Z */
#define wd_aux_altsts   6   /* alternate fixed disk status (R) */
#define wd_aux_control  6   /* device control (W) */
#define wd_aux_drvaddr  7   /* drive address (R) */

#define wd_control_num_regs 8

#define wd_num_regs     (wd_command_num_regs + wd_control_num_regs)

/* wd_aux_control: */
#define WDCTL_HOB       0x80  /* read high order byte */
#define WDCTL_4BIT      0x08  /* use four head bits (wd1003) */
#define WDCTL_RST       0x04  /* reset the controller (self-clearing) */
#define WDCTL_IDS       0x02  /* disable controller interrupts */

/* the controller "chip": */
struct tme_ata {

  /* our simple bus device header: */
  struct tme_bus_device tme_ata_device;
#define tme_ata_element tme_ata_device.tme_bus_device_element

  /* our "socket": */
  struct tme_ata_controller_socket tme_ata_socket;
#define	tme_ata_addr_shift	\
  tme_ata_socket.tme_ata_controller_socket_addr_shift
#define	tme_ata_port_least_lane	\
  tme_ata_socket.tme_ata_controller_socket_port_least_lane

	/* the mutex protecting the controller: */
  tme_mutex_t tme_ata_mutex;

  /* our registers: */
	tme_uint8_t tme_ata_reg_error;
  tme_uint8_t tme_ata_reg_features;
  tme_uint8_t tme_ata_reg_seccnt;
  tme_uint8_t	tme_ata_reg_sector;
  tme_uint8_t tme_ata_reg_cyl_lo;
  tme_uint8_t tme_ata_reg_cyl_hi;
  tme_uint8_t tme_ata_reg_sdh;
  tme_uint8_t tme_ata_reg_command;
  tme_uint8_t tme_ata_reg_status;

	tme_uint8_t tme_ata_reg_aux_control;
  tme_uint8_t tme_ata_reg_aux_drvaddr;

  /* the sector buffer: */
	tme_uint8_t	tme_ata_sector_buffer[wd_sector_size];
	unsigned int tme_ata_sector_buffer_index;
};

/* our callout function.  it must be called with the mutex locked: */
static void
_tme_ata_callout(struct tme_ata *ata,
								 int new_callouts)
{
	/* XXX */
}

/* our internal reset function: */
static int
_tme_ata_reset(struct tme_ata *ata)
{
  /* XXX */
	return (0);
}

/* the standard bus router for the 8-bit registers: */
static const tme_bus_lane_t tme_ata_router8[TME_BUS_ROUTER_SIZE(TME_BUS8_LOG2)] = {

	/* [gen]  initiator port size: 8 bits
     [gen]  initiator port least lane: 0: */
  /* D7-D0 */		TME_BUS_LANE_ROUTE(0),
};

/* the bus cycle handler for the data register: */
static int
_tme_ata_bus_cycle_data(struct tme_ata *ata, struct tme_bus_cycle *cycle_init)
{
  /* XXX */
	return TME_OK;
}

/* the general bus cycle handler: */
static int
_tme_ata_bus_cycle(void *_ata, struct tme_bus_cycle *cycle_init)
{
	struct tme_ata *ata;
  tme_bus_addr32_t address, address_last;
  tme_uint8_t buffer, value;
  struct tme_bus_cycle cycle_resp;
  int new_callouts;
  int reg;

  /* recover our data structure: */
  ata = (struct tme_ata *) _ata;

  /* the requested cycle must be within range: */
  address_last = ata->tme_ata_device.tme_bus_device_address_last;
  address = cycle_init->tme_bus_cycle_address;
  assert(address <= address_last);
  assert(cycle_init->tme_bus_cycle_size <= (address_last - address) + 1);
  (void)address_last;

	/* get the register being accessed: */
  reg = address >> ata->tme_ata_addr_shift;

	/* if it's the data register, go handle that separately; we need
     to deal with different access sizes for that register.  */
  if (reg == wd_data) {
    return _tme_ata_bus_cycle_data(ata, cycle_init);
  }

	/* lock the mutex: */
  tme_mutex_lock(&ata->tme_ata_mutex);

  /* assume we won't need any new callouts: */
  new_callouts = 0;

  /* if this is a write: */
  if (cycle_init->tme_bus_cycle_type == TME_BUS_CYCLE_WRITE) {

		/* run the bus cycle: */
    cycle_resp.tme_bus_cycle_buffer = &buffer;
    cycle_resp.tme_bus_cycle_lane_routing = tme_ata_router8;
    cycle_resp.tme_bus_cycle_address = 0;
    cycle_resp.tme_bus_cycle_buffer_increment = 1;
    cycle_resp.tme_bus_cycle_type = TME_BUS_CYCLE_READ;
    cycle_resp.tme_bus_cycle_size = sizeof(buffer);
    cycle_resp.tme_bus_cycle_port =
      TME_BUS_CYCLE_PORT(ata->tme_ata_port_least_lane,
												 TME_BUS8_LOG2);
    tme_bus_cycle_xfer(cycle_init, &cycle_resp);
    value = buffer;

    /* log this write: */
    tme_log(TME_ATA_LOG_HANDLE(ata), 100000, TME_OK,
            (TME_ATA_LOG_HANDLE(ata),
      "REG %d <- 0x%02x", reg, value));

		switch (reg) {

		case wd_data:
      /* handled above. */
      abort();
      break;

		case wd_features:
      ata->tme_ata_reg_features = value;
      break;

		case wd_seccnt:
      ata->tme_ata_reg_seccnt = value;
      break;

    case wd_sector:
      ata->tme_ata_reg_sector = value;
      break;

    case wd_cyl_lo:
      ata->tme_ata_reg_cyl_lo = value;
      break;

    case wd_cyl_hi:
      ata->tme_ata_reg_cyl_hi = value;
      break;

    case wd_sdh:
      ata->tme_ata_reg_sdh = value;
      break;

		case wd_command:
      ata->tme_ata_reg_command = value;
      new_callouts |= TME_ATA_CALLOUT_COMMAND;
      break;

		case wd_command_num_regs + wd_aux_control:
      ata->tme_ata_reg_aux_control = value;
      new_callouts |= TME_ATA_CALLOUT_CONTROL;
      break;

		default:
      /* all other writes are ignored. */
      break;
    }
  }

  /* otherwise, this is a read: */
	else {
    assert(cycle_init->tme_bus_cycle_type == TME_BUS_CYCLE_READ);

    switch (reg) {

    case wd_data:
      /* handled above. */
      abort();
      break;

    case wd_error:
      value = ata->tme_ata_reg_error;
      /* XXX self-clearing? */
      break;

    case wd_seccnt:
      value = ata->tme_ata_reg_seccnt;
      break;

    case wd_sector:
      value = ata->tme_ata_reg_sector;
      break;

		case wd_cyl_lo:
      value = ata->tme_ata_reg_cyl_lo;
      break;

    case wd_cyl_hi:
      value = ata->tme_ata_reg_cyl_hi;
      break;

    case wd_sdh:
      value = ata->tme_ata_reg_sdh;
      break;

    case wd_status:
      value = ata->tme_ata_reg_status;
      /* XXX self-clearing? */
      break;

		case wd_command_num_regs + wd_aux_altsts:
      value = ata->tme_ata_reg_status;
      break;

    default:
      /* all other regs are high-z, read back as $FF */
      value = 0xff;
      break;
  }

#ifndef TME_NO_LOG
		/* log this read: */
    if (ata->tme_ata_last_read_reg != reg
        || ata->tme_ata_last_read_value != value) {
      ata->tme_ata_last_read_reg = reg;
      ata->tme_ata_last_read_value = value;
      tme_log(TME_ATA_LOG_HANDLE(ata), 100000, TME_OK,
              (TME_ATA_LOG_HANDLE(ata),
        "REG %d -> 0x%02x", reg, value));
    }
#endif /* TME_NO_LOG */

		/* run the bus cycle: */
  	buffer = value;
  	cycle_resp.tme_bus_cycle_buffer = &buffer;
  	cycle_resp.tme_bus_cycle_lane_routing = tme_ata_router8;
  	cycle_resp.tme_bus_cycle_address = 0;
  	cycle_resp.tme_bus_cycle_buffer_increment = 1;
  	cycle_resp.tme_bus_cycle_type = TME_BUS_CYCLE_WRITE;
  	cycle_resp.tme_bus_cycle_size = sizeof(buffer);
  	cycle_resp.tme_bus_cycle_port =
    	TME_BUS_CYCLE_PORT(ata->tme_ata_port_least_lane,
                       	 TME_BUS8_LOG2);
		tme_bus_cycle_xfer(cycle_init, &cycle_resp);
  }

  /* make any needed callouts: */
  _tme_ata_callout(ata, new_callouts);

  /* unlock the mutex: */
  tme_mutex_unlock(&ata->tme_ata_mutex);

  /* no faults: */
  return (TME_OK);
}

/* our TLB filler: */
static int
_tme_ata_tlb_fill(void *_ata,
									struct tme_bus_tlb *tlb,
                  tme_bus_addr_t address,
                  unsigned int cycles)
{
	struct tme_ata *ata;
  tme_bus_addr32_t address_last;

  /* recover our data structure: */
  ata = (struct tme_ata *) _ata;

  /* the address must be within range: */
  address_last = ata->tme_ata_device.tme_bus_device_address_last;
  assert(address <= address_last);

	/* initialize the TLB entry: */
  tme_bus_tlb_initialize(tlb);

  /* this TLB entry can cover the whole device: */
  tlb->tme_bus_tlb_addr_first = 0;
  tlb->tme_bus_tlb_addr_last = address_last;

  /* allow reading and writing: */
  tlb->tme_bus_tlb_cycles_ok = TME_BUS_CYCLE_READ | TME_BUS_CYCLE_WRITE;

  /* our bus cycle handler: */
  tlb->tme_bus_tlb_cycle_private = ata;
  tlb->tme_bus_tlb_cycle = _tme_ata_bus_cycle;

  return (TME_OK);
}

/* our bus signal handler: */
static int
_tme_ata_signal(void *_ata,
                unsigned int signal)
{
	struct tme_ata *ata;
  int new_callouts;
  unsigned int level;

  /* recover our data structure: */
  ata = (struct tme_ata *) _ata;

  /* assume we won't need any new callouts: */
  new_callouts = 0;

  /* lock the mutex: */
  tme_mutex_lock(&ata->tme_ata_mutex);

  /* take out the signal level: */
  level = signal & TME_BUS_SIGNAL_LEVEL_MASK;
  signal = TME_BUS_SIGNAL_WHICH(signal);

  /* dispatch on the generic bus signals: */
  switch (signal) {

  case TME_BUS_SIGNAL_RESET:
    if (level == TME_BUS_SIGNAL_LEVEL_ASSERTED) {
      new_callouts |= _tme_ata_reset(ata);
    }
    break;

  default:
    break;
  }

  /* make any new callouts: */
  _tme_ata_callout(ata, new_callouts);

  /* unlock the mutex: */
  tme_mutex_unlock(&ata->tme_ata_mutex);

  /* no faults: */
  return (TME_OK);
}

/* this makes a new connection side for the ATA controller: */
static int
_tme_ata_connections_new(struct tme_element *element,
												 const char * const *args,
                         struct tme_connection **_conns,
                         char **_output)
{
	struct tme_ata *ata;
  int rc;

  /* make the generic bus device connection side: */
  rc = tme_bus_device_connections_new(element, args, _conns, _output);
  if (rc != TME_OK) {
    return (rc);
  }

	/* XXX MOAR */

  /* done: */
  return (TME_OK);
}

/* the new ata controller function: */
TME_ELEMENT_NEW_DECL(tme_ata_controller) {
	const struct tme_ata_controller_socket *socket;
  struct tme_ata *ata;
  struct tme_ata_controller_socket socket_real;
  tme_bus_addr_t address_mask;

  /* dispatch on our socket version: */
  socket = (const struct tme_ata_controller_socket *) extra;
  if (socket == NULL) {
    tme_output_append_error(_output, _("need an ic socket"));
    return (ENXIO);
	}
  switch (socket->tme_ata_controller_socket_version) {
  case TME_ATA_CONTROLLER_SOCKET_0:
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

  /* start the ata controller structure: */
  ata = tme_new0(struct tme_ata, 1);
  ata->tme_ata_socket = socket_real;
  tme_mute_init(&ata->tme_ata_mutex);

  _tme_ata_reset(ata);

  /* figure our address mask, up to the nearest power of two: */
  address_mask = (wd_num_regs << ata->tme_ata_addr_shift) - 1;

  /* initialize our simple bus device descriptor: */
  ata->tme_ata_device.tme_bus_device_element = element;
  ata->tme_ata_device.tme_bus_device_tlb_fill = _tme_ata_tlb_fill;
  ata->tme_ata_device.tme_bus_device_signal = _tme_ata_signal;
  ata->tme_ata_device.tme_bus_device_address_last = address_mask;

  /* fill the element: */
  element->tme_element_private = ata;
  element->tme_element_connections_new = _tme_ata_connections_new;

  return (TME_OK);
}
