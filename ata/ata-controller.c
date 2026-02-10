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

/* wd_sdh: */
#define WDSD_IBM        0xa0  /* forced to 512 byte sector, ecc */
#define WDSD_DRV1       0x10  /* select drive 1 */
#define WDSD_LBA        0x40  /* logical block addressing */
#define WDSD_FUA        0x80  /* Forced Unit Access (FUA) */

/* wd_error: */
#define WDCE_BBK        0x80  /* bad block detected */
#define WDCE_UNC        0x40  /* uncorrectable data error */
#define WDCE_MC         0x20  /* media changed */
#define WDCE_IDNF       0x10  /* id not found */
#define WDCE_MCR        0x08  /* media change requested */
#define WDCE_ABRT       0x04  /* aborted command */
#define WDCE_TK0NF      0x02  /* track 0 not found */
#define WDCE_AMNF       0x01  /* address mark not found */

#define WDCE_DIAG_NO_ERROR            0x01
#define WDCE_DIAG_FORMATTER_ERROR     0x02
#define WDCE_DIAG_SECTOR_BUFFER_ERROR 0x03
#define WDCE_DIAG_ECC_CIRCUITRY_ERROR 0x04
#define WDCE_DIAG_PROCESSOR_ERROR     0x05
#define WDCE_DIAG_CODE_MASK           0x7f
#define WDCE_DIAG_DRV1                0x80

/* wd_features: */
#define WDSF_8BIT_PIO_EN              0x01
#define WDSF_WRITE_CACHE_EN           0x02
#define WDSF_SET_TRANSFER_MODE        0x03
#define WDSF_RETRY_DIS                0x33
#define WDSF_SET_CACHE_SEGMENTS       0x54
#define WDSF_READ_LOOKAHEAD_DIS       0x55
#define WDSF_POD_REVERT_DIS           0x66
#define WDSF_ECC_DIS                  0x77
#define WDSF_8BIT_PIO_DIS             0x81
#define WDSF_WRITE_CACHE_DIS          0x82
#define WDSF_ECC_EN                   0x88
#define WDSF_RETRY_EN                 0x99
#define WDSF_READ_LOOKAHEAD_EN        0xaa
#define WDSF_SET_MAX_PREFETCH         0xab
#define WDSF_4BYTE_ECC                0xbb
#define WDSF_POD_REVERT_EN            0xcc

/* wd_command: */
#define WDCC_READ       0x20  /* disk read */
#define WDCC_WRITE      0x30  /* disk write */
#define WDCC_READ_VERF  0x40  /* read verify */
#define   WDCC__LONG     0x02 /* modifier -- access ecc bytes */
#define   WDCC__NORETRY  0x01 /* modifier -- no retries */

#define WDCC_NOP        0x00  /* Always fail with "aborted command" */
#define WDCC_IDLE_97    0x97  /* Idle */
#define WDCC_IDLE_e3    0xe3
#define WDCC_IDLE_IMM_95 0x95 /* Idle Immediate */
#define WDCC_IDLE_IMM_e1 0xe1
#define WDCC_DIAGNOSE   0x90  /* controller diagnostic */
#define WDCC_FORMAT     0x50  /* format track */
#define WDCC_IDENTIFY   0xec  /* identify drive */
#define WDCC_IDP        0x91  /* initialize drive parameters */
#define WDCC_RECAL_0    0x10  /* disk recalibrate -- resets cntlr */
#define WDCC_RECAL_1    0x11  /* disk recalibrate -- resets cntlr */
#define WDCC_RECAL_2    0x12  /* disk recalibrate -- resets cntlr */
#define WDCC_RECAL_3    0x13  /* disk recalibrate -- resets cntlr */
#define WDCC_RECAL_4    0x14  /* disk recalibrate -- resets cntlr */
#define WDCC_RECAL_5    0x15  /* disk recalibrate -- resets cntlr */
#define WDCC_RECAL_6    0x16  /* disk recalibrate -- resets cntlr */
#define WDCC_RECAL_7    0x17  /* disk recalibrate -- resets cntlr */
#define WDCC_RECAL_8    0x18  /* disk recalibrate -- resets cntlr */
#define WDCC_RECAL_9    0x19  /* disk recalibrate -- resets cntlr */
#define WDCC_RECAL_a    0x1a  /* disk recalibrate -- resets cntlr */
#define WDCC_RECAL_b    0x1b  /* disk recalibrate -- resets cntlr */
#define WDCC_RECAL_c    0x1c  /* disk recalibrate -- resets cntlr */
#define WDCC_RECAL_d    0x1d  /* disk recalibrate -- resets cntlr */
#define WDCC_RECAL_e    0x1e  /* disk recalibrate -- resets cntlr */
#define WDCC_RECAL_f    0x1f  /* disk recalibrate -- resets cntlr */
#define WDCC_SEEK_0     0x70  /* disk seek */
#define WDCC_SEEK_1     0x71  /* disk seek */
#define WDCC_SEEK_2     0x72  /* disk seek */
#define WDCC_SEEK_3     0x73  /* disk seek */
#define WDCC_SEEK_4     0x74  /* disk seek */
#define WDCC_SEEK_5     0x75  /* disk seek */
#define WDCC_SEEK_6     0x76  /* disk seek */
#define WDCC_SEEK_7     0x77  /* disk seek */
#define WDCC_SEEK_8     0x78  /* disk seek */
#define WDCC_SEEK_9     0x79  /* disk seek */
#define WDCC_SEEK_a     0x7a  /* disk seek */
#define WDCC_SEEK_b     0x7b  /* disk seek */
#define WDCC_SEEK_c     0x7c  /* disk seek */
#define WDCC_SEEK_d     0x7d  /* disk seek */
#define WDCC_SEEK_e     0x7e  /* disk seek */
#define WDCC_SEEK_f     0x7f  /* disk seek */
#define WDCC_SET_FEAT   0xef  /* set features */

/* wd_status: */
#define WDCS_BSY        0x80  /* busy */
#define WDCS_DRDY       0x40  /* drive ready */
#define WDCS_DWF        0x20  /* drive write fault */
#define WDCS_DSC        0x10  /* drive seek complete */
#define WDCS_DRQ        0x08  /* data request */
#define WDCS_CORR       0x04  /* corrected data */
#define WDCS_IDX        0x02  /* index */
#define WDCS_ERR        0x01  /* error */

/* wd_aux_control: */
#define WDCTL_HOB       0x80  /* read high order byte */
#define WDCTL_4BIT      0x08  /* use four head bits (wd1003) */
#define WDCTL_RST       0x04  /* reset the controller (self-clearing) */
#define WDCTL_IDS       0x02  /* disable controller interrupts */

#define TME_ATA_LOG_HANDLE(a) (&(a)->tme_ata_element->tme_element_log_handle)

#define TME_ATA_CALLOUT_CHECK       (0)
#define TME_ATA_CALLOUTS_RUNNING    TME_BIT(0)
#define TME_ATA_CALLOUTS_MASK       (-2)
#define TME_ATA_CALLOUT_COMMAND     TME_BIT(1)
#define TME_ATA_CALLOUT_INT         TME_BIT(2)

#define	TME_ATA_SELECTED_DRIVE(ata) \
  (((ata)->tme_ata_reg_sdh & WDSD_DRV1) ? 1 : 0)

/* if SRST is set in the aux_control register, then we behave as if
   no drives are connected.  The drive internal state gets reset on
   the rising edge of SRST, and then when SRST goes low, the drives
   will respond as though an instantaneous reset has occurred. */
/* XXX placeholder */
#define	TME_ATA_DRIVE_PRESENT(ata, d) \
  ((((ata)->tme_ata_reg_aux_control & WDCTL_RST) == 0) \
   && ((d) == 0))

#define	TME_ATA_ANY_DRIVE_PRESENT(ata) \
  (TME_ATA_DRIVE_PRESENT((ata), 0) || TME_ATA_DRIVE_PRESENT((ata), 1))

/* structures: */

/* drive parameter structure defined by the ATA specification.
   Only ATA fields are covered here; no nod is given to ATAPI. */
struct ata_drive_params {
  tme_uint16_t atap_config;             /* 0: general configuration */

#define ATAP_CFG_HARD_SECTOR    TME_BIT(1)
#define ATAP_CFG_SOFT_SECTOR    TME_BIT(2)
#define ATAP_CFG_FIXED          TME_BIT(6)
#define ATAP_CFG_XFER_10PLUS    TME_BIT(10)

#define ATAP_CFG_value          (ATAP_CFG_HARD_SECTOR | \
                                 ATAP_CFG_FIXED | \
                                 ATAP_CFG_XFER_10PLUS)

  tme_uint16_t atap_cylinders;          /* 1: # of non-removable cylinders */
  tme_uint16_t atap___reserved2;        /* 2: */
  tme_uint16_t atap_heads;              /* 3: # of heads */
  tme_uint16_t atap_unf_bytes_track;    /* 4: # unformatted bytes/track */
  tme_uint16_t atap_unf_bytes_sector;   /* 5: # unformatted bytes/sector */
  tme_uint16_t atap_sectors_track;      /* 6: # of sectors/track */
  tme_uint16_t atap___vendor7[3];       /* 7-9: */
  tme_uint8_t  atap_serial[20];         /* 10-19: serial number */
  tme_uint16_t atap___retired20[2];     /* 20-21: */
  tme_uint16_t atap___obsolete22;       /* 22: */
  tme_uint8_t  atap_revision[8];        /* 23-26: firmware revision */
  tme_uint8_t  atap_model[40];          /* 27-46: model number */
#ifdef WORDS_BIGENDIAN
  tme_uint8_t  atap___vendor47;
  tme_uint8_t  atap_multi;              /* 47: max sectors per irq */
#else
  tme_uint8_t  atap_multi;              /* 47: max sectors per irq */
  tme_uint8_t  atap___vendor47;
#endif
  tme_uint16_t atap___reserved48;       /* 48: */
  tme_uint16_t atap_capabilities1;      /* 49: capability flags */

#define ATAP_CAP1_LBA       TME_BIT(8)
#define ATAP_CAP1_DMA       TME_BIT(9)

  tme_uint16_t atap___reserved50;       /* 50: */
#ifdef WORDS_BIGENDIAN
  tme_uint8_t  atap___vendor51;
  tme_uint8_t  atap_pio_mode;           /* 51: PIO timing mode */
  tme_uint8_t  atap___vendor52;
  tme_uint8_t  atap_dma_mode;           /* 52: DMA timing mode */
#else
  tme_uint8_t  atap_pio_mode;           /* 51: old PIO timing mode */
  tme_uint8_t  atap___vendor51;
  tme_uint8_t  atap_dma_mode;           /* 52: old DMA timing mode */
  tme_uint8_t  atap___vendor52;
#endif
  tme_uint16_t atap_extensions;         /* 53: extensions supported */

#define ATAP_EXT_GEOMETRY   TME_BIT(0)  /* words 54-58 are valid */

  /* words 54-62 are ATA only */
  tme_uint16_t atap_curcylinders;       /* 54: current logical cylinders */
  tme_uint16_t atap_curheads;           /* 55: current logical heads */
  tme_uint16_t atap_cursectors_track;   /* 56: current logical sectors/track */
  tme_uint16_t atap_curcapacity[2];     /* 57-58: current capacity */
#ifdef WORDS_BIGENDIAN
  tme_uint8_t  atap_curmulti_flags;
  tme_uint8_t  atap_curmulti;           /* 59: current multi-sector setting */
#else
  tme_uint8_t  atap_curmulti;           /* 59: current multi-sector setting */
  tme_uint8_t  atap_curmulti_flags;
#endif

#define ATAP_CURMULTI_VALID TME_BIT(0)

  tme_uint16_t atap_lba_capacity[2];    /* 60-61: total capacity (LBA only) */
#ifdef WORDS_BIGENDIAN
  tme_uint8_t  atap_sw_dma_active;
  tme_uint8_t  atap_sw_dma_supported;   /* 62: single-word DMA modes */
  tme_uint8_t  atap_mw_dma_active;
  tme_uint8_t  atap_mw_dma_supported;   /* 63: multi-word DMA modes */
#else
  tme_uint8_t  atap_sw_dma_supported;   /* 62: single-word DMA modes */
  tme_uint8_t  atap_sw_dma_active;
  tme_uint8_t  atap_mw_dma_supported;   /* 63: multi-word DMA modes */
  tme_uint8_t  atap_mw_dma_active;
#endif

  tme_uint16_t atap___reserved64[64];   /* 64-127: reserved */
  tme_uint16_t atap___vendor128[32];    /* 128-159: vendor unique */
  tme_uint16_t atap___reserved160[96];  /* 160-255: reserved */
};

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

  /* our registers.  registers that each individual drive updates are
     kept separately, registers that are modified only by the host are
     shared.  for host-writable drive-specific registers, writes go to
     *both*, but reads come from the drive selected in the wd_sdh register. */

#define	drv_reg_error       0   /* r/o */
#define	drv_reg_seccnt      1
#define	drv_reg_sector      2
#define	drv_reg_cyl_lo      3
#define	drv_reg_cyl_hi      4
#define	drv_reg_status      5   /* r/o */
#define	drv_reg_aux_drvaddr 6   /* r/o */
#define	drv_num_regs        7

  tme_uint8_t tme_ata_drv_regs[2][drv_num_regs];
  tme_uint8_t tme_ata_reg_features;
  tme_uint8_t tme_ata_reg_sdh;
  tme_uint8_t tme_ata_reg_command;
  tme_uint8_t tme_ata_reg_aux_control;

  /* the sector buffer: */
  struct tme_ata_sector_buffer {
    unsigned int tme_ata_sector_buffer_index;
    union {
      tme_uint8_t sector_buffer_data[wd_sector_size];
      struct ata_drive_params sector_buffer_params;
    } tme_ata_sector_buffer_union;
  } tme_ata_sector_buffer[2];

  /* I/O flags for each drive. */
  unsigned int tme_ata_io_flags[2];

#define TME_ATA_IO_8BIT       TME_BIT(0)
#define TME_ATA_IO_INPROG     TME_BIT(1)
#define TME_ATA_IO_CANCELLED  TME_BIT(2)

  /* our callout flags. */
  int tme_ata_callout_flags;
};

/* maps a command block register to the per-drive instance. */
static int
_tme_ata_command_reg_to_drv_reg(int reg)
{
  static const unsigned int regmap[wd_command_num_regs] = {
    -1,                     /* wd_data, handled separately */
    drv_reg_error,          /* wd_error */
    drv_reg_seccnt,         /* wd_seccnt */
    drv_reg_sector,         /* wd_sector */
    drv_reg_cyl_lo,         /* wd_cyl_lo */
    drv_reg_cyl_hi,         /* wd_cyl_hi */
    -1,                     /* wd_sdh */
    drv_reg_status,         /* wd_status */
  };
  int rv;

  if (reg < 0 || reg >= wd_command_num_regs) {
    rv = -1;
  } else {
    rv = regmap[reg];
  }

  return (rv);
}

/* our internal reset function: */
static int
_tme_ata_reset(struct tme_ata *ata)
{
  /* reset is always processed by both drives. */
  memset(ata->tme_ata_drv_regs, 0, sizeof(ata->tme_ata_drv_regs));
  ata->tme_ata_drv_regs[0][drv_reg_status] = 
    ata->tme_ata_drv_regs[1][drv_reg_status] = WDCS_DRDY;

  /* the ATA-1 specification gives these as the non-zero register default
     values:  */
  ata->tme_ata_drv_regs[0][drv_reg_error] =
    ata->tme_ata_drv_regs[1][drv_reg_error] = 0x01;
  ata->tme_ata_drv_regs[0][drv_reg_seccnt] =
    ata->tme_ata_drv_regs[1][drv_reg_seccnt] = 0x01;
  ata->tme_ata_drv_regs[0][drv_reg_sector] =
    ata->tme_ata_drv_regs[1][drv_reg_sector] = 0x01;

  ata->tme_ata_reg_features = 0;
  ata->tme_ata_reg_sdh = WDSD_IBM;
  ata->tme_ata_reg_command = 0;

  /* aux_control not affected by reset. */

  /* interrupt status may have changed. */
  return (TME_ATA_CALLOUT_INT);
}

/* abort a command: */
static void
_tme_ata_abort_command(struct tme_ata *ata, int drive)
{
  ata->tme_ata_drv_regs[drive][drv_reg_error] = WDCE_ABRT;
  ata->tme_ata_drv_regs[drive][drv_reg_status] |= WDCS_ERR;
}

/* SET FEATURES command: */
static void
_tme_ata_command_set_features(struct tme_ata *ata, int drive)
{
  /*  dispatch on the contents of the features register: */
  switch (ata->tme_ata_drv_regs[drive][drv_reg_features]) {
  case WDSF_8BIT_PIO_EN:
    ata->tme_ata_io_flags[drive] |= TME_ATA_IO_8BIT;
    break;

  default:
    _tme_ata_abort_command(ata, drive);
    break;
  }
}

/* process a command: */
static void
_tme_ata_command(struct tme_ata *ata)
{
  int drive = TME_ATA_SELECTED_DRIVE(ata);
  int assert_interrupt;

  /* assume the command will not generate an error. */
  ata->tme_ata_drv_regs[drive][drv_reg_status] &= ~WDCS_ERR;

  /* assume an interrupt at the end of the command. */
  assert_interrupt = TRUE;

  /* dispatch on the contents of the command register: */
  switch (ata->tme_ata_reg_command) {

  case WDCC_SET_FEAT:
    _tme_ata_command_set_features(ata);
    break;

  default:
    _tme_ata_abort_command(ata);
  }

  if (assert_interrupt) {
    ata->tme_ata_int_pending = TRUE;
    ata->tme_ata_callout_flags |= TME_ATA_CALLOUT_INTERRUPT;
  }
}

/* our callout function.  it must be called with the mutex locked: */
static void
_tme_ata_callout(struct tme_ata *ata,
                 int new_callouts)
{
  int callouts;
  int later_callouts;

  /* add in any new callouts: */
  ata->tme_ata_callout_flags |= new_callouts;

  /* if this function is already running in another thread, return
     now.  the other thread will do our work: */
  if (ata->tme_ata_callout_flags & TME_ATA_CALLOUTS_RUNNING) {
    return;
  }

  /* callouts are now running: */
  ata->tme_ata_callout_flags |= TME_ATA_CALLOUTS_RUNNING;

  /* assume we won't need any later callouts: */
  later_callouts = 0;

  /* loop while callouts are needed: */
  for (;;) {

    callouts = ata->tme_ata_callout_flags & TME_ATA_CALLOUTS_MASK;
    if (callouts == 0) {
      break;
    }

    /* clear the needed callouts: */
    ata->tme_ata_callout_flags &= ~TME_ATA_CALLOUTS_MASK;

    if (callouts & TME_ATA_CALLOUT_COMMAND) {
      _tme_ata_command(ata);
    }
  }

  /* put in any later callouts, and clear that callouts are running: */
  ata->tme_ata_callout_flags = later_callouts;
}

/* the standard bus router for the 8-bit registers: */
static const tme_bus_lane_t tme_ata_router8[TME_BUS_ROUTER_SIZE(TME_BUS8_LOG2)] = {

  /* [gen]  initiator port size: 8 bits
     [gen]  initiator port least lane: 0: */
  /* D7-D0 */   TME_BUS_LANE_ROUTE(0),
};

/* the bus cycle handler for the data register: */
static int
_tme_ata_bus_cycle_data(struct tme_ata *ata,
                        int drive,
                        struct tme_bus_cycle *cycle_init)
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
  int drv_reg;
  int drive;

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

  /* figure out which drive we are accessing. */
  drive = TME_ATA_SELECTED_DRIVE(ata);

  /* if it's the data register, go handle that separately; we need
     to deal with different access sizes for that register.  */
  if (reg == wd_data) {
    return _tme_ata_bus_cycle_data(ata, drive, cycle_init);
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
      if (TME_ATA_ANY_DRIVE_PRESENT(ata)) {
        ata->tme_ata_reg_features = value;
      }
      break;

    case wd_seccnt:
    case wd_sector:
    case wd_cyl_lo:
    case wd_cyl_hi:
      drv_reg = _tme_ata_command_reg_to_drv_reg(reg);
      assert(drv_reg != -1);
      if (TME_ATA_DRIVE_PRESENT(ata, 0)) {
        ata->tme_ata_drv_regs[0][drv_reg] = value;
      }
      if (TME_ATA_DRIVE_PRESENT(ata, 1)) {
        ata->tme_ata_drv_regs[1][drv_reg] = value;
      }
      break;

    case wd_sdh:
      if (TME_ATA_ANY_DRIVE_PRESENT(ata)) {
        ata->tme_ata_reg_sdh = value | WDSD_IBM;
      }
      break;

    case wd_command:
      if (TME_ATA_ANY_DRIVE_PRESENT(ata)) {
        ata->tme_ata_reg_command = value;
        new_callouts |= TME_ATA_CALLOUT_COMMAND;
      }
      break;

    case wd_command_num_regs + wd_aux_control:
      /* if the new value sets SRST, first we must cancel any
         pending I/O.  This will prevent the drive from reporting
         DRDY=1 until the cancellation is acknowledged.  */
      if (value & WDCTL_RST) {
        if (ata->tme_ata_io_flags[0] & TME_ATA_IO_INPROG) {
          ata->tme_ata_io_flags[0] |= TME_ATA_IO_CANCELLED;
        }
        if (ata->tme_ata_io_flags[1] & TME_ATA_IO_INPROG) {
          ata->tme_ata_io_flags[1] |= TME_ATA_IO_CANCELLED;
        }
        if (ata->tme_ata_int_pending) {
          ata->tme_ata_int_pending = FALSE;
          new_callouts |= TME_ATA_CALLOUT_INT;
        }
      }
      if ((ata->tme_ata_reg_aux_control ^ value) & WDCTL_IDS) {
        /* interrupt enable status changed. */
        new_callouts |= TME_ATA_CALLOUT_INT;
      }
      ata->tme_ata_reg_aux_control = value;
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
    case wd_seccnt:
    case wd_sector:
    case wd_cyl_lo:
    case wd_cyl_hi:
    case wd_status:
    case wd_command_num_regs + wd_aux_altsts:
      drv_reg =
        ((reg == (wd_command_num_regs + wd_aux_altsts)
         ? drv_reg_status
         : _tme_ata_command_reg_to_drv_reg(reg)));
      assert(drv_reg != -1);
      if (TME_ATA_DRIVE_PRESENT(ata, drive)) {
        value = ata->tme_ata_drv_regs[drive][drv_reg];
        if (reg == wd_status) {
          /* reading the status register clears any pending interrupt: */
          ata->tme_ata_int_pending = FALSE;
          new_callouts |= TME_ATA_CALLOUT_INT;
        }
      } else if (ata->tme_ata_reg_aux_control & WDCTL_RST) {
        /* N.B. in the description of the reset response in the ATA-1
           specification, it says that BSY shall be set in the status
           register when SRST is asserted.  returning the high-z value
           here satisfies that requirement. */
        value = 0xff;
      } else if (drv_reg == drv_reg_status) {
        /* if the drive isn't present and it's not because of a reset, the
           status register is handled specially; the ATA-1 specification
           says that drive 0 will respond with 0x00 if drive 1 is not present
           in order to ensure that DRDY=0. */
        if (drive == 1 && TME_ATA_DRIVE_PRESENT(ata, 0)) {
          value = 0x00;
        } else {
          value = 0xff;
        }
      }
      break;

    case wd_sdh:
      if (TME_ATA_ANY_DRIVE_PRESENT(ata)) {
        value = ata->tme_ata_reg_sdh;
      } else {
        value = 0xff;
      }
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
  //struct tme_ata *ata;
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
TME_ELEMENT_SUB_NEW_DECL(tme_ata,controller) {
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
  tme_mutex_init(&ata->tme_ata_mutex);

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
