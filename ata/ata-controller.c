/* ata/ata-controller.c - implementation of ATA / IDE disk
   controller emulation: */

/*
 * Copyright (c) 2026 Jason R. Thorpe
 * All rights reserved.
 */

#include <tme/common.h>

/* includes: */
#include <tme/generic/bus-device.h>
#include <tme/generic/disk.h>
#include <tme/ata/ata-controller.h>
#ifdef HAVE_STDARG_H
#include <stdarg.h>
#else  /* HAVE_STDARG_H */
#include <varargs.h>
#endif /* HAVE_STDARG_H */

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
#define WDCC_RECAL_0    0x10  /* disk recalibrate */
#define WDCC_RECAL_1    0x11  /* disk recalibrate */
#define WDCC_RECAL_2    0x12  /* disk recalibrate */
#define WDCC_RECAL_3    0x13  /* disk recalibrate */
#define WDCC_RECAL_4    0x14  /* disk recalibrate */
#define WDCC_RECAL_5    0x15  /* disk recalibrate */
#define WDCC_RECAL_6    0x16  /* disk recalibrate */
#define WDCC_RECAL_7    0x17  /* disk recalibrate */
#define WDCC_RECAL_8    0x18  /* disk recalibrate */
#define WDCC_RECAL_9    0x19  /* disk recalibrate */
#define WDCC_RECAL_a    0x1a  /* disk recalibrate */
#define WDCC_RECAL_b    0x1b  /* disk recalibrate */
#define WDCC_RECAL_c    0x1c  /* disk recalibrate */
#define WDCC_RECAL_d    0x1d  /* disk recalibrate */
#define WDCC_RECAL_e    0x1e  /* disk recalibrate */
#define WDCC_RECAL_f    0x1f  /* disk recalibrate */
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
#define WDCTL_RST       0x04  /* reset the controller */
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
#define	TME_ATA_DRIVE_PRESENT(ata, d) \
  ((((ata)->tme_ata_reg_aux_control & WDCTL_RST) == 0) \
   && ((ata)->tme_ata_disk_connections[(d)] != NULL))

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

#define ATAP_CAP1_DMA       TME_BIT(8)
#define ATAP_CAP1_LBA       TME_BIT(9)

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

/* an ATA disk connection: */
struct tme_ata_disk_connection {

  /* the regular disk connection: */
  struct tme_disk_connection tme_ata_disk_connection;

  /* the drive number of this disk: */
  int tme_ata_disk_connection_drive;

  /* the block size for this disk.  this is always wd_sector_size,
     but we need the tme_value64 for arithmetic later. */
  union tme_value64 tme_ata_disk_connection_block_size;

  /* C/H/S geometry for this drive.  We compute a fake fixed geometry
     based on 1MB cylinders.  This will truncate the device to the
     geometry when doing CHS-addressed I/O.  *shrug*  */
  tme_uint16_t tme_ata_disk_connection_num_cylinders;
  tme_uint16_t tme_ata_disk_connection_num_heads;
  tme_uint16_t tme_ata_disk_connection_sectors_per_track;

  /* this caches the last addressable LBA for LBA-addressed I/O. */
  tme_uint32_t tme_ata_disk_connection_last_lba;
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

  /* the sector buffer.  this contains all of the state related to
     a data transfer to/from the host. */
  struct tme_ata_sector_buffer {
    tme_uint8_t *sector_buffer_disk_buffer;
    unsigned int sector_buffer_disk_buffer_index;
    unsigned int sector_buffer_index;
    tme_uint8_t sector_buffer_data[wd_sector_size];
    unsigned int sector_buffer_resid;
  } tme_ata_sector_buffers[2];

  /* cursor used for each drive during I/O */
  tme_uint32_t tme_ata_io_cursor[2];

  /* I/O flags for each drive. */
  unsigned int tme_ata_io_flags[2];

#define TME_ATA_IO_8BIT       TME_BIT(0)
#define TME_ATA_IO_READ       TME_BIT(1)  /* drive -> host */
#define TME_ATA_IO_WRITE      TME_BIT(1)  /* host -> drive */
#define TME_ATA_IO_LBA        TME_BIT(2)  /* command is LBA mode */
#define TME_ATA_IO_ADVANCE    TME_BIT(3)  /* need to advance the cursor */

  /* the disk connections: */
  struct tme_ata_disk_connection *tme_ata_disk_connections[2];

  /* our callout flags. */
  int tme_ata_callout_flags;

  /* our interrupt state. */
  int tme_ata_int_pending;
  int tme_ata_int_asserted;
};

/* maps a register to the per-drive instance. */
static int
_tme_ata_reg_to_drv_reg(int reg, int for_read)
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

  if (for_read
       && reg == (wd_command_num_regs + wd_aux_altsts)) {
    return (drv_reg_status);
  }

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

  /* clears any features that were set, cancells I/O. */
  ata->tme_ata_io_flags[0] = 0;
  ata->tme_ata_io_flags[1] = 0;

  /* no pending interrupts */
  ata->tme_ata_int_pending = FALSE;

  /* interrupt status may have changed. */
  return (TME_ATA_CALLOUT_INT);
}

/* set internal logic to indicate a host data transfer is ready */
static int
_tme_ata_data_transfer_init(struct tme_ata *ata,
                            int drive,
                            unsigned int sector_count,
                            unsigned int flags)
{
  struct tme_ata_sector_buffer *sb;

  sb = &ata->tme_ata_sector_buffers[drive];
  sb->sector_buffer_index = 0;
  sb->sector_buffer_resid = sector_count;

  ata->tme_ata_io_flags[drive] |= flags;

  ata->tme_ata_drv_regs[drive][drv_reg_status] |= WDCS_DRQ;

  return ((flags & TME_ATA_IO_READ) ? TRUE : FALSE);
}

/* clear in-flight I/O state. */
static void
_tme_ata_data_transfer_fini(struct tme_ata *ata,
                            int drive)
{
  ata->tme_ata_io_flags[drive]
    &= ~(TME_ATA_IO_READ | TME_ATA_IO_WRITE
         | TME_ATA_IO_LBA | TME_ATA_IO_ADVANCE);
  ata->tme_ata_drv_regs[drive][drv_reg_status] &= ~WDCS_DRQ;

  ata->tme_ata_sector_buffers[drive].sector_buffer_disk_buffer = NULL;
}

/* advance the drive cursor during I/O */
static void
_tme_ata_advance_cursor(struct tme_ata *ata,
                        int drive,
                        int count)
{
  struct tme_ata_disk_connection *conn_ata_disk;
  tme_uint32_t cursor;
  tme_uint32_t cylinder;
  tme_uint32_t head;
  tme_uint32_t sector;

  assert((ata->tme_ata_io_flags[drive] & TME_ATA_IO_ADVANCE) != 0);

  cursor = (ata->tme_ata_io_cursor[drive] += count);

  if (ata->tme_ata_io_flags[drive] & TME_ATA_IO_LBA) {
    ata->tme_ata_drv_regs[drive][drv_reg_sector] = (tme_uint8_t)cursor;
    cursor >>= 8;
    ata->tme_ata_drv_regs[drive][drv_reg_cyl_lo] = (tme_uint8_t)cursor;
    cursor >>= 8;
    ata->tme_ata_drv_regs[drive][drv_reg_cyl_hi] = (tme_uint8_t)cursor;
    cursor >>= 8;
    ata->tme_ata_reg_sdh
      = (ata->tme_ata_reg_sdh & ~0x0f) | (cursor & 0x0f);
  }
  else {
    conn_ata_disk = ata->tme_ata_disk_connections[drive];

    sector
      = (cursor % conn_ata_disk->tme_ata_disk_connection_sectors_per_track)
        + 1;
    head
      = (cursor / conn_ata_disk->tme_ata_disk_connection_sectors_per_track)
        % conn_ata_disk->tme_ata_disk_connection_num_heads;
    cylinder
      = cursor / (conn_ata_disk->tme_ata_disk_connection_num_heads
                  * conn_ata_disk->tme_ata_disk_connection_sectors_per_track);

    ata->tme_ata_drv_regs[drive][drv_reg_sector] = (tme_uint8_t)sector;
    ata->tme_ata_drv_regs[drive][drv_reg_cyl_lo] = (tme_uint8_t)cylinder;
    cylinder >>= 8;
    ata->tme_ata_drv_regs[drive][drv_reg_cyl_hi] = (tme_uint8_t)cylinder;
    ata->tme_ata_reg_sdh
      = (ata->tme_ata_reg_sdh & ~0x0f) | (head & 0x0f);
  }
}

/* copy a string into an IDENTIFY buffer field, padding with spaces. */
static void
_tme_ata_copy_identify_string(tme_uint8_t *data,
                              const char *string,
                              unsigned int size)
{
  tme_uint8_t c;

  for (; size-- > 0; ) {
    c = *(string++); 
    if (c == '\0') {
      c = ' ';
      string--;
    }
    *(data++) = c;
  }
}

/* abort a command: */
static void
_tme_ata_abort_command(struct tme_ata *ata, int drive)
{
  ata->tme_ata_drv_regs[drive][drv_reg_error] = WDCE_ABRT;
  ata->tme_ata_drv_regs[drive][drv_reg_status] |= WDCS_ERR;
}

#define TME_ATA_DISK_MODEL    "TME ATA DISK"
#define TME_ATA_DISK_REVISION "021025"

#define TME_ATA_MAX_SECTORS_IRQ   1

/* get the LBA and sector count for an I/O operation.  returns TRUE
   if the command was aborted due to invalid parameters. */
static int
_tme_ata_get_io_params(struct tme_ata *ata,
                       int drive,
                       tme_uint32_t *lba_out,
                       tme_uint32_t *sectors_out)
{
  struct tme_ata_disk_connection *conn_ata_disk;
  tme_uint32_t num_cylinders;
  tme_uint32_t num_heads;
  tme_uint32_t num_sectors;
  tme_uint32_t cylinder;
  tme_uint32_t head;
  tme_uint32_t sector;
  tme_uint32_t lba;
  tme_uint32_t sector_count;

  conn_ata_disk = ata->tme_ata_disk_connections[drive];

  if (ata->tme_ata_reg_sdh & WDSD_LBA) {
    lba =              ata->tme_ata_reg_sdh & 0xf;
    lba = (lba << 8) | ata->tme_ata_drv_regs[drive][drv_reg_cyl_hi];
    lba = (lba << 8) | ata->tme_ata_drv_regs[drive][drv_reg_cyl_lo];
    lba = (lba << 8) | ata->tme_ata_drv_regs[drive][drv_reg_sector];
  }
  else {
    num_cylinders = conn_ata_disk->tme_ata_disk_connection_num_cylinders;
    num_heads = conn_ata_disk->tme_ata_disk_connection_num_heads;
    num_sectors = conn_ata_disk->tme_ata_disk_connection_sectors_per_track;

    cylinder =                   ata->tme_ata_drv_regs[drive][drv_reg_cyl_lo];
    cylinder = (cylinder << 8) | ata->tme_ata_drv_regs[drive][drv_reg_cyl_hi];

    head = ata->tme_ata_reg_sdh & 0xf;

    sector = ata->tme_ata_drv_regs[drive][drv_reg_sector] - 1;

    if (cylinder >= num_cylinders
        || head >= num_heads
        || sector >= num_sectors) {
      /* illegal address */
      _tme_ata_abort_command(ata, drive);
      return (TRUE);
    }

    lba = (((cylinder * num_heads) + head) * num_sectors) + sector;
  }

  sector_count = ata->tme_ata_drv_regs[drive][drv_reg_seccnt];
  if (sector_count == 0) {
    sector_count = 256;
  }

  if (lba > conn_ata_disk->tme_ata_disk_connection_last_lba
      || (lba + sector_count
          > conn_ata_disk->tme_ata_disk_connection_last_lba + 1)
      || (lba + sector_count < lba)) {
    _tme_ata_abort_command(ata, drive);
    return (TRUE);
  }

  *lba_out = lba;
  *sectors_out = sector_count;

  return (FALSE);
}

/* IDENTIFY command: */
static int
_tme_ata_command_identify(struct tme_ata *ata, int drive)
{
  struct ata_drive_params *atap;
  struct tme_ata_disk_connection *conn_ata_disk;
  tme_uint32_t blocks;

  conn_ata_disk = ata->tme_ata_disk_connections[drive];
  assert(conn_ata_disk != NULL);

  atap = (struct ata_drive_params *)
    &ata->tme_ata_sector_buffers[drive].sector_buffer_data;

  memset(atap, 0, sizeof(*atap));
  atap->atap_config = tme_htole_u16(ATAP_CFG_value);
  atap->atap_cylinders
    = tme_htole_u16(conn_ata_disk->tme_ata_disk_connection_num_cylinders);
  atap->atap_heads
    = tme_htole_u16(conn_ata_disk->tme_ata_disk_connection_num_heads);
  atap->atap_sectors_track
    = tme_htole_u16(conn_ata_disk->tme_ata_disk_connection_sectors_per_track);

  _tme_ata_copy_identify_string(atap->atap_serial,
                                "",
                                sizeof(atap->atap_serial));
  _tme_ata_copy_identify_string(atap->atap_revision,
                                TME_ATA_DISK_REVISION,
                                sizeof(atap->atap_revision));
  _tme_ata_copy_identify_string(atap->atap_model,
                                TME_ATA_DISK_MODEL,
                                sizeof(atap->atap_model));

  atap->atap_multi = TME_ATA_MAX_SECTORS_IRQ;
  atap->atap_capabilities1
    = tme_htole_u16(ATAP_CAP1_LBA);

  blocks = conn_ata_disk->tme_ata_disk_connection_last_lba + 1;
  assert(blocks != 0);

  atap->atap_lba_capacity[0] = tme_htole_u16(blocks & 0xffff);
  atap->atap_lba_capacity[1] = tme_htole_u16(blocks >> 16);

  /* data is now available for the host. */
  return _tme_ata_data_transfer_init(ata, drive, 1, TME_ATA_IO_READ);
}

/* READ VERIFY command: */
static void
_tme_ata_command_read_verify(struct tme_ata *ata, int drive)
{
  tme_uint32_t lba;
  tme_uint32_t sector_count;

  if (_tme_ata_get_io_params(ata, drive, &lba, &sector_count)) {
    /* command was aborted */
    return;
  }

  /* the cursor must point to the last sector verified. */
  ata->tme_ata_io_cursor[drive] = lba;
  ata->tme_ata_io_flags[drive] |= TME_ATA_IO_ADVANCE;
  _tme_ata_advance_cursor(ata, drive, sector_count - 1);
  ata->tme_ata_io_flags[drive] &= ~TME_ATA_IO_ADVANCE;
}

/* READ and WRITE commands: */
static int
_tme_ata_command_io(struct tme_ata *ata, int drive, tme_uint8_t rw)
{
  struct tme_ata_disk_connection *conn_ata_disk;
  struct tme_disk_connection *other_conn_disk;
  struct tme_ata_sector_buffer *sb;
  tme_uint32_t lba;
  tme_uint32_t sector_count;
  union tme_value64 off;
  unsigned long len;
  int rc;

  conn_ata_disk = ata->tme_ata_disk_connections[drive];
  other_conn_disk
    = ((struct tme_disk_connection *)
       conn_ata_disk->tme_ata_disk_connection.tme_disk_connection.tme_connection_other);

  sb = &ata->tme_ata_sector_buffers[drive];

  if (_tme_ata_get_io_params(ata, drive, &lba, &sector_count)) {
    /* command was aborted, need to interrupt. */
    return (TRUE);
  }

  /* set the 64-bit offset and the long size: */
  (void) tme_value64_set(&off, lba);
  (void) tme_value64_mul(&off,
                         &conn_ata_disk->tme_ata_disk_connection_block_size);
  len = sector_count;
  len
    *= conn_ata_disk->tme_ata_disk_connection_block_size.tme_value64_uint32_lo;

  /* get the disk buffer. */
  if (rw == WDCC_READ) {
    rc = ((*other_conn_disk->tme_disk_connection_read)
          (other_conn_disk,
           &off,
           len,
           (const tme_uint8_t **)&sb->sector_buffer_disk_buffer));
  }
  else {
    /* if this disk is read-only: */
    if (other_conn_disk->tme_disk_connection_write == NULL) {
      _tme_ata_abort_command(ata, drive);
      return (TRUE);
    }

    rc = ((*other_conn_disk->tme_disk_connection_write)
          (other_conn_disk,
          &off,
          len,
          &sb->sector_buffer_disk_buffer));
  }

  /* if we couldn't get the disk buffer: */
  if (rc != TME_OK) {
    _tme_ata_abort_command(ata, drive);
    return (TRUE);
  }
  sb->sector_buffer_disk_buffer_index = 0;

  /* initialize the cursor that represents the currently relevant
     sector.  this will get advanced as the I/O moves along and
     at the end will point to the last sector read or written. */
  ata->tme_ata_io_cursor[drive] = lba;

  /* if we're reading, we need to do the initial fill of the sector
     buffer. */
  if (rw == WDCC_READ) {
    memcpy(sb->sector_buffer_data,
           sb->sector_buffer_disk_buffer,
           sizeof(sb->sector_buffer_data));

    /* for the firsts sector buffer fill, the cursor already points to
       the correct place.  it needs to be advanced to point to current
       sector at each subsequent fill. */
    ata->tme_ata_io_flags[drive] |= TME_ATA_IO_ADVANCE;
  }

  return _tme_ata_data_transfer_init(ata,
                                     drive,
                                     sector_count,
                                     ((rw == WDCC_READ)
                                      ? TME_ATA_IO_READ : TME_ATA_IO_WRITE));
}

/* SET FEATURES command: */
static void
_tme_ata_command_set_features(struct tme_ata *ata, int drive)
{
  /* dispatch on the contents of the features register: */
  switch (ata->tme_ata_reg_features) {
  case WDSF_8BIT_PIO_EN:
    ata->tme_ata_io_flags[drive] |= TME_ATA_IO_8BIT;
    break;

  case WDSF_8BIT_PIO_DIS:
    ata->tme_ata_io_flags[drive] &= ~TME_ATA_IO_8BIT;
    break;

  /* ignore these. */
  case WDSF_WRITE_CACHE_EN:
  case WDSF_RETRY_DIS:
  case WDSF_SET_CACHE_SEGMENTS:
  case WDSF_READ_LOOKAHEAD_DIS:
  case WDSF_POD_REVERT_DIS:
  case WDSF_ECC_DIS:
  case WDSF_WRITE_CACHE_DIS:
  case WDSF_ECC_EN:
  case WDSF_RETRY_EN:
  case WDSF_READ_LOOKAHEAD_EN:
  case WDSF_SET_MAX_PREFETCH:
  case WDSF_4BYTE_ECC:
  case WDSF_POD_REVERT_EN:
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

  /* N.B. We don't bother with BSY much here, since the commands
     complete instantanously from the perspective of the guest and
     thus there is no opportunity to observe it. */

  /* clear out previous command status bits. */
  ata->tme_ata_drv_regs[drive][drv_reg_status]
    &= ~(WDCS_ERR | WDCS_CORR | WDCS_DRQ | WDCS_DSC | WDCS_DWF);

  /* cancel any in-flight I/O. */
  _tme_ata_data_transfer_fini(ata, drive);

  /* assume an interrupt at the end of the command. */
  assert_interrupt = TRUE;

  /* dispatch on the contents of the command register: */
  switch (ata->tme_ata_reg_command) {

  case WDCC_SET_FEAT:
    _tme_ata_command_set_features(ata, drive);
    break;

  case WDCC_IDLE_97:
  case WDCC_IDLE_e3:
  case WDCC_IDLE_IMM_95:
  case WDCC_IDLE_IMM_e1:
    /* instantly successful */
    break;

  case WDCC_DIAGNOSE:
    /* these drives always work perfectly */
    ata->tme_ata_drv_regs[0][drv_reg_error] =
      ata->tme_ata_drv_regs[1][drv_reg_error] = WDCE_DIAG_NO_ERROR;
    break;

  case WDCC_FORMAT:
    _tme_ata_abort_command(ata, drive);
    break;

  case WDCC_IDENTIFY:
    assert_interrupt = _tme_ata_command_identify(ata, drive);
    break;

  case WDCC_IDP:
    _tme_ata_abort_command(ata, drive);
    break;

  case WDCC_READ_VERF:
    _tme_ata_command_read_verify(ata, drive);
    break;

  case WDCC_READ:
  case WDCC_READ | WDCC__NORETRY:
    assert_interrupt = _tme_ata_command_io(ata, drive, WDCC_READ);
    break;

  case WDCC_WRITE:
  case WDCC_WRITE | WDCC__NORETRY:
    assert_interrupt = _tme_ata_command_io(ata, drive, WDCC_WRITE);
    break;

  default:
    if (ata->tme_ata_reg_command >= WDCC_RECAL_0
        && ata->tme_ata_reg_command <= WDCC_RECAL_f) {
      /* recalibrate always succeeds */
    }
    else if (ata->tme_ata_reg_command >= WDCC_SEEK_0
             && ata->tme_ata_reg_command <= WDCC_SEEK_f) {
      /* seek also always succeeds, but also sets DSC */
      ata->tme_ata_drv_regs[drive][drv_reg_status] |= WDCS_DSC;
    }
    else {
      _tme_ata_abort_command(ata, drive);
    }
  }

  if (assert_interrupt) {
    ata->tme_ata_int_pending = TRUE;
    ata->tme_ata_callout_flags |= TME_ATA_CALLOUT_INT;
  }
}

/* our callout function.  it must be called with the mutex locked: */
static void
_tme_ata_callout(struct tme_ata *ata,
                 int new_callouts)
{
  struct tme_bus_connection *conn_bus;
  int callouts;
  int later_callouts;
  int new_int_asserted;
  int rc;

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

  /* interrupt defaults to unchanged: */
  new_int_asserted = ata->tme_ata_int_asserted;

  /* loop while callouts are needed: */
  for (;;) {

    callouts = ata->tme_ata_callout_flags & TME_ATA_CALLOUTS_MASK;
    if (callouts == 0) {
      break;
    }

    /* clear the needed callouts: */
    ata->tme_ata_callout_flags &= ~TME_ATA_CALLOUTS_MASK;

    if (callouts & TME_ATA_CALLOUT_COMMAND) {
      /* go process the command.  this may schedule additional callouts. */
      _tme_ata_command(ata);
    }

    if (callouts & TME_ATA_CALLOUT_INT) {
      new_int_asserted = ata->tme_ata_int_pending
        && ((ata->tme_ata_reg_aux_control & WDCTL_IDS) == 0);
    }
  }

  /* if the interrupt change, we need to go call it out: */
  if (new_int_asserted != ata->tme_ata_int_asserted) {

    /* unlock our mutex: */
    tme_mutex_unlock(&ata->tme_ata_mutex);

    /* get our bus connection: */
    conn_bus
        = tme_memory_atomic_pointer_read(struct tme_bus_connection *,
            ata->tme_ata_device.tme_bus_device_connection,
            &ata->tme_ata_device.tme_bus_device_connection_rwlock);

    /* call out the bus interrupt signal edge: */
    rc = (*conn_bus->tme_bus_signal)
      (conn_bus,
       TME_BUS_SIGNAL_INT_UNSPEC
       | TME_BUS_SIGNAL_EDGE
       | (new_int_asserted
          ? TME_BUS_SIGNAL_LEVEL_ASSERTED
          : TME_BUS_SIGNAL_LEVEL_NEGATED));

    /* lock our mutex: */
    tme_mutex_lock(&ata->tme_ata_mutex);

    /* if this callout was successful, note the new state of the
       interrupt signal: */
    if (rc == TME_OK) {
      ata->tme_ata_int_asserted = new_int_asserted;
    }

    /* otherwise, remember that at some later time this callout
       should be attempted again: */
    else {
      later_callouts |= TME_ATA_CALLOUT_INT;
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
  struct tme_bus_cycle cycle_resp;
  struct tme_ata_sector_buffer *sb;
  tme_uint8_t buffer[2];
  int assert_interrupt;

  assert_interrupt = FALSE;

  memset(buffer, 0, sizeof(buffer));

  sb = &ata->tme_ata_sector_buffers[drive];

  /* lock the mutex: */
  tme_mutex_lock(&ata->tme_ata_mutex);

  /* some sanity checks before we perform the bus cycle: */
  if (ata->tme_ata_io_flags[drive] & (TME_ATA_IO_READ | TME_ATA_IO_WRITE)) {

    /* if we're not doing 8-bit I/O, the sector buffer index must
       be even. */
    assert((ata->tme_ata_io_flags[drive] & TME_ATA_IO_8BIT) != 0
           || (sb->sector_buffer_index & 1) == 0);

    /* the cycle must fit within the sector buffer.  note that the
       test performed here is sufficient given the assertion above. */
    assert(sb->sector_buffer_index < sizeof(sb->sector_buffer_data));

    /* if we're writing, there must be an associated disk buffer. */
    assert((ata->tme_ata_io_flags[drive] & TME_ATA_IO_WRITE) != 0
           || sb->sector_buffer_disk_buffer != NULL);

    /* there must be something left to transfer. */
    assert(sb->sector_buffer_resid != 0);
  }

  /* if this is a write: */
  if (cycle_init->tme_bus_cycle_type == TME_BUS_CYCLE_WRITE) {

    /* run the bus cycle: */
    cycle_resp.tme_bus_cycle_buffer = buffer;
    cycle_resp.tme_bus_cycle_lane_routing = tme_ata_router8; /* XXX */
    cycle_resp.tme_bus_cycle_address = 0;
    cycle_resp.tme_bus_cycle_buffer_increment = 1;
    cycle_resp.tme_bus_cycle_type = TME_BUS_CYCLE_READ;
    cycle_resp.tme_bus_cycle_size = 1;  /* XXX */
    cycle_resp.tme_bus_cycle_port =
      TME_BUS_CYCLE_PORT(ata->tme_ata_port_least_lane,
                         TME_BUS8_LOG2);
    tme_bus_cycle_xfer(cycle_init, &cycle_resp);

    /* ignore the cycle if a write command is not in progress. */
    if (ata->tme_ata_io_flags[drive] & TME_ATA_IO_WRITE) {
      sb->sector_buffer_data[sb->sector_buffer_index++] = buffer[0];
      if ((ata->tme_ata_io_flags[drive] & TME_ATA_IO_8BIT) == 0) {
        sb->sector_buffer_data[sb->sector_buffer_index++] = buffer[1];
      }

      /* check to see if we've finished the sector. */
      if (sb->sector_buffer_index == sizeof(sb->sector_buffer_data)) {
        /* copy the sector to the disk buffer. */
        memcpy(&sb->sector_buffer_disk_buffer[sb->sector_buffer_disk_buffer_index],
               sb->sector_buffer_data,
               sizeof(sb->sector_buffer_data));
        sb->sector_buffer_disk_buffer_index += sizeof(sb->sector_buffer_data);

        /* advance the cursor to point at the sector just written.  this
           is not done for the first sector because the cursor already
           points to it. */
        if (ata->tme_ata_io_flags[drive] & TME_ATA_IO_ADVANCE) {
          _tme_ata_advance_cursor(ata, drive, 1);
        }
        ata->tme_ata_io_flags[drive] |= TME_ATA_IO_ADVANCE;

        /* reset the sector buffer index. */
        sb->sector_buffer_index = 0;

        /* sector complete, raise an interrupt. */
        assert_interrupt = TRUE;

        /* decrement the residual count and update I/O status. */
        sb->sector_buffer_resid--;
        ata->tme_ata_drv_regs[drive][drv_reg_seccnt] = sb->sector_buffer_resid;
        if (sb->sector_buffer_resid == 0) {
          _tme_ata_data_transfer_fini(ata, drive);
        }
      }
    }
  }

  /* otherwise, this is a read: */
  else {
    assert(cycle_init->tme_bus_cycle_type == TME_BUS_CYCLE_READ);

    /* we always return what the sector buffer index currently points to. */
    buffer[0] = sb->sector_buffer_data[sb->sector_buffer_index];
    if ((ata->tme_ata_io_flags[drive] & TME_ATA_IO_8BIT) == 0) {
      buffer[1] = sb->sector_buffer_data[sb->sector_buffer_index + 1];
    }

    /* If we're doing an I/O, advance the sector buffer. */
    if (ata->tme_ata_io_flags[drive] & TME_ATA_IO_READ) {
      sb->sector_buffer_index +=
        (ata->tme_ata_io_flags[drive] & TME_ATA_IO_8BIT) ? 1 : 2;

      /* check to see if we've finished the sector. */
      if (sb->sector_buffer_index == sizeof(sb->sector_buffer_data)) {
        /* reset the sector buffer index. */
        sb->sector_buffer_index = 0;

        /* sector complete, raise an interrupt. */
        assert_interrupt = TRUE;

        /* decrement the residual count, maybe re-fill the sector buffer,
           and update I/O status. */
        sb->sector_buffer_resid--;
        ata->tme_ata_drv_regs[drive][drv_reg_seccnt] = sb->sector_buffer_resid;
        if (sb->sector_buffer_resid == 0) {
          _tme_ata_data_transfer_fini(ata, drive);
        } else if (sb->sector_buffer_disk_buffer != NULL) {

          /* advance the cursor to indicate where we re-filling the
             sector buffer from. */
          _tme_ata_advance_cursor(ata, drive, 1);

          sb->sector_buffer_disk_buffer_index
            += sizeof(sb->sector_buffer_data);
          memcpy(sb->sector_buffer_data,
                 &sb->sector_buffer_disk_buffer[sb->sector_buffer_disk_buffer_index],
                 sizeof(sb->sector_buffer_data));
        }
      }
    }

    /* run the bus cycle: */
    cycle_resp.tme_bus_cycle_buffer = buffer;
    cycle_resp.tme_bus_cycle_lane_routing = tme_ata_router8;  /* XXX */
    cycle_resp.tme_bus_cycle_address = 0;
    cycle_resp.tme_bus_cycle_buffer_increment = 1;
    cycle_resp.tme_bus_cycle_type = TME_BUS_CYCLE_WRITE;
    cycle_resp.tme_bus_cycle_size = 1;      /* XXX */
    cycle_resp.tme_bus_cycle_port =
      TME_BUS_CYCLE_PORT(ata->tme_ata_port_least_lane,
                         TME_BUS8_LOG2);
    tme_bus_cycle_xfer(cycle_init, &cycle_resp);
  }

  /* run the callouts if we need to assert the interrupt. */
  if (assert_interrupt) {
    _tme_ata_callout(ata, TME_ATA_CALLOUT_INT);
  }

  /* unlock the mutex: */
  tme_mutex_unlock(&ata->tme_ata_mutex);

  /* done: */
  return (TME_OK);
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
  drv_reg
    = _tme_ata_reg_to_drv_reg(reg,
                              (cycle_init->tme_bus_cycle_type
                               == TME_BUS_CYCLE_READ));

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

#ifndef TME_NO_LOG
    /* log this write: */
    tme_log(TME_ATA_LOG_HANDLE(ata), 0, TME_OK,
            (TME_ATA_LOG_HANDLE(ata),
             "DRIVE %d REG %d (drv_reg %d) <- 0x%02x",
             drive, reg, drv_reg, value));
#endif /* TME_NO_LOG */

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
      if (value & WDCTL_RST) {
        new_callouts |= _tme_ata_reset(ata);
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

    value = 0xff;  /* XXX gcc -Werror=maybe-uninitialized confusion */

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
      assert(drv_reg != -1);
      if (TME_ATA_DRIVE_PRESENT(ata, drive)) {
        value = ata->tme_ata_drv_regs[drive][drv_reg];
        if (reg == wd_status) {
          /* reading the status register clears any pending interrupt: */
          ata->tme_ata_int_pending = FALSE;
          new_callouts |= TME_ATA_CALLOUT_INT;
        }
      }
      else if (ata->tme_ata_reg_aux_control & WDCTL_RST) {
        /* N.B. in the description of the reset response in the ATA-1
           specification, it says that BSY shall be set in the status
           register when SRST is asserted.  returning the high-z value
           here satisfies that requirement. */
        value = 0xff;
      }
      /* if the drive isn't present and it's not because of a reset, the
         status register is handled specially; the ATA-1 specification
         says that drive 0 will respond with 0x00 if drive 1 is not present
         in order to ensure that DRDY=0. */
      else if (drv_reg == drv_reg_status) {
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
    if (1) {
      tme_log(TME_ATA_LOG_HANDLE(ata), 0, TME_OK,
              (TME_ATA_LOG_HANDLE(ata),
               "DRIVE %d REG %d (drv_reg %d) -> 0x%02x",
               drive, reg, drv_reg, value));
    }
#endif /* ! TME_NO_LOG */

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

/* the disk control handler: */
#ifdef HAVE_STDARG_H
static int
_tme_ata_disk_control(struct tme_disk_connection *conn_disk,
                      unsigned int control,
                      ...)
#else /* HAVE_STDARG_H */
static int
_tme_ata_disk_control(conn_disk, control, va_alist)
    struct tme_disk_connection *conn_disk;
    unsigned int control;
    va_dcl
#endif /* HAVE_STDARG_H */
{
  struct tme_ata *ata;

  /* recover our device: */
  ata = (struct tme_ata *) conn_disk->tme_disk_connection.tme_connection_element->tme_element_private;

  /* lock the mutex: */
  tme_mutex_lock(&ata->tme_ata_mutex);

  /* unlock the mutex: */
  tme_mutex_unlock(&ata->tme_ata_mutex);

  return (TME_OK);
}

/* this computes the geometry of the disk based on the size and
   block size. */
static void
_tme_ata_compute_disk_geometry(struct tme_ata_disk_connection *conn_ata_disk)
{
  struct tme_disk_connection *other_conn_disk;
  union tme_value64 blocks64;
  tme_uint32_t blocks32;
  tme_uint32_t num_cylinders;

  other_conn_disk
    = ((struct tme_disk_connection *)
       conn_ata_disk->tme_ata_disk_connection.tme_disk_connection.tme_connection_other);

  /* compute total blocks. */
  blocks64 = other_conn_disk->tme_disk_connection_size;
  (void) tme_value64_div(&blocks64,
                         &conn_ata_disk->tme_ata_disk_connection_block_size);
  blocks32 = blocks64.tme_value64_uint32_lo;

  /* Compute a fake geometry loosely based on the "standard"
     Adaptec fictitious geometry.  Adaptec uses 64 heads and
     32 sectors/track, but we only have 4 head bits.  */
  conn_ata_disk->tme_ata_disk_connection_num_heads = 16;
  conn_ata_disk->tme_ata_disk_connection_sectors_per_track = 128;
  num_cylinders = blocks32 / (16 * 128);
  if (num_cylinders > 0xffff) {
    num_cylinders = 0xffff;
  }
  conn_ata_disk->tme_ata_disk_connection_num_cylinders
    = (tme_uint16_t) num_cylinders;

  /* cache the last LBA */
  conn_ata_disk->tme_ata_disk_connection_last_lba = blocks32 - 1;
}

/* this makes a new disk connection: */
static int
_tme_ata_disk_connection_make(struct tme_connection *conn,
                              unsigned int state)
{
  struct tme_ata *ata;
  struct tme_ata_disk_connection *conn_ata_disk;
  int drive;

  /* both sides must be disk connections: */
  assert (conn->tme_connection_type == TME_CONNECTION_DISK);
  assert (conn->tme_connection_other->tme_connection_type == TME_CONNECTION_DISK);

  /* recover our data structures: */
  ata = conn->tme_connection_element->tme_element_private;
  conn_ata_disk = (struct tme_ata_disk_connection *) conn;

  /* we're always set up to answer calls across the connection,
     so we only have to do work when the connection has gone full,
     namely taking the other side of the connection: */
  if (state == TME_CONNECTION_FULL) {

    /* lock the mutex: */
    tme_mutex_lock(&ata->tme_ata_mutex);

    /* make this disk connection: */
    drive = conn_ata_disk->tme_ata_disk_connection_drive;
    assert(ata->tme_ata_disk_connections[drive] == NULL);
    ata->tme_ata_disk_connections[drive] = conn_ata_disk;

    /* now that we have the disk connected, pre-compute the fixed
       geometry. */
    _tme_ata_compute_disk_geometry(conn_ata_disk);

    /* unlock the mutex: */
    tme_mutex_unlock(&ata->tme_ata_mutex);
  }

  return (TME_OK);
}

/* this breaks a connection: */
static int
_tme_ata_disk_connection_break(struct tme_connection *conn,
                               unsigned int state)
{
  abort();
}

/* this parses an ATA drive number: */
static int
tme_ata_drive_parse(const char *drive_string)
{
  unsigned long val;
  char *p1;

  /* catch a NULL string: */
  if (drive_string == NULL) {
    return (-1);
  }

  /* convert the string: */
  val = strtoul(drive_string, &p1, 0);
  if (p1 == drive_string
      || *p1 != '\0') {
    return (-1);
  }
  return (val);
}

/* this makes a new connection side for the ATA controller: */
static int
_tme_ata_connections_new(struct tme_element *element,
                         const char * const *args,
                         struct tme_connection **_conns,
                         char **_output)
{
  struct tme_ata_disk_connection *conn_ata_disk;
  struct tme_disk_connection *conn_disk;
  struct tme_connection *conn;
  struct tme_ata *ata;
  int drive;
  int arg_i;
  int usage;
  int rc;

  /* recover our device: */
  ata = (struct tme_ata *) element->tme_element_private;

  /* check our arguments: */
  drive = -1;
  arg_i = 1;
  usage = FALSE;

  /* loop reading our arguments: */
  for (;;) {

    /* the drive to attach for */
    if (TME_ARG_IS(args[arg_i + 0], "drive")
        && drive < 0
        && (drive = tme_ata_drive_parse(args[arg_i + 1])) >= 0
        && drive < 2
        && ata->tme_ata_disk_connections[drive] == NULL) {
      arg_i += 2;
    }

    /* if we've run out of arguments: */
    else if (args[arg_i + 0] == NULL) {
      break;
    }

    /* this is a bad argument: */
    else {
      tme_output_append_error(_output,
                              "%s %s, ",
                              args[arg_i],
                              _("unexpected"));
      usage = TRUE;
      break;
    }
  }

  if (usage) {
    tme_output_append_error(_output,
                            "%s %s [ drive %s ]",
                            _("usage:"),
                            args[0],
                            _("DRIVE-NUMBER"));
    return (EINVAL);
  }

  /* make the generic bus device connection side: */
  rc = tme_bus_device_connections_new(element, args, _conns, _output);
  if (rc != TME_OK) {
    return (rc);
  }

  /* if we don't have a particular drive numnber, find the first free
     one and use that.  */
  if (drive < 0) {
    for (drive = 0;
         drive < 2;
         drive++) {
      if (ata->tme_ata_disk_connections[drive] == NULL) {
        break;
      }
    }
    if (drive == 2) {
      return (TME_OK);
    }
  }

  /* create our side of a disk connection: */
  conn_ata_disk = tme_new0(struct tme_ata_disk_connection, 1);
  conn_disk = &conn_ata_disk->tme_ata_disk_connection;
  conn = &conn_disk->tme_disk_connection;

  /* fill in the generic connection: */
  conn->tme_connection_next = *_conns;
  conn->tme_connection_type = TME_CONNECTION_DISK;
  conn->tme_connection_score = tme_disk_connection_score;
  conn->tme_connection_make = _tme_ata_disk_connection_make;
  conn->tme_connection_break = _tme_ata_disk_connection_break;

  /* fill in the disk connection: */
  conn_disk->tme_disk_connection_control = _tme_ata_disk_control;

  /* fill in the internal disk connection: */
  conn_ata_disk->tme_ata_disk_connection_drive = drive;
  (void) tme_value64_set(&conn_ata_disk->tme_ata_disk_connection_block_size,
                         wd_sector_size);

  /* return the connection side possibility: */
  *_conns = conn;

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
  ata->tme_ata_reg_aux_control = WDCTL_IDS;

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
