/* ic/dsrtc.c - implementation of Dallas / Maxim RTC emulation: */

/*  
 * Copyright (c) 2026 Jason R. Thorpe
 * All rights reserved.
 */

#include <tme/tme.h>

/* includes: */
#include <tme/generic/i2c.h>

#define TME_DSRTC_LOG_HANDLE(p) \
  (&(p)->tme_dsrtc_element->tme_element_log_handle)

#define DSRTC_I2C_ADDR  0x68

#define DSRTC_SECONDS   0x00
#define DSRTC_MINUTES   0x01
#define DSRTC_HOURS     0x02
#define DSRTC_DAY       0x03
#define DSRTC_DATE      0x04
#define DSRTC_MONTH     0x05
#define DSRTC_YEAR      0x06
#define DSRTC_SIZE      7

#define DSRTC_TYPE_DS1307     0
#define DSRTC_TYPE_DS3231     1
#define DSRTC_TYPE_DS3232     2

#define	DSRTC_MAXSIZE         256

#define	DSRTC_STATE_IDLE      0
#define DSRTC_STATE_READ      1
#define DSRTC_STATE_CURSOR    2
#define DSRTC_STATE_WRITE     3

static const tme_uint8_t _tme_dsrtc_cursor_max[] = {
  [DSRTC_TYPE_DS1307] = 0x3f,
  [DSRTC_TYPE_DS3231] = 0x12,
  [DSRTC_TYPE_DS3232] = 0xff,
};

/* the chip: */
struct tme_dsrtc {

  /* backpointer to the device's element: */
  struct tme_element *tme_dsrtc_element;

  /* our i2c connection: */
  struct tme_i2c_connection *tme_dsrtc_i2c_connection;

  /* the mutex protecting the chip: */
  tme_mutex_t tme_dsrtc_mutex;

  /* the chip type: */
  int tme_dsrtc_type;

  /* cursor for transfers: */
  unsigned int tme_dsrtc_cursor;
  int tme_dsrtc_state;

  /* eopch year: */
  unsigned int tme_dsrtc_epoch_year;

  /* dsrtc NVRAM: */
  tme_uint8_t tme_dsrtc_nvram[DSRTC_MAXSIZE];
};

static inline tme_uint8_t
_tme_dsrtc_bcd_out(unsigned int value)
{
  return ((value % 10)
	  + ((value / 10) * 16));
}

static void
_tme_dsrtc_update_rtc(struct tme_dsrtc *dsrtc)
{
  struct timeval now;
  time_t _now;
  struct tm *now_tm, now_tm_buffer;
  unsigned int year;

  gettimeofday(&now, NULL);
  _now = now.tv_sec;
  now_tm = gmtime_r(&_now, &now_tm_buffer);

  year = (1900 + now_tm->tm_year) - dsrtc->tme_dsrtc_epoch_year;

  /* put the time-of-day into the registers: */
  dsrtc->tme_dsrtc_nvram[DSRTC_SECONDS] = _tme_dsrtc_bcd_out(now_tm->tm_sec);
  dsrtc->tme_dsrtc_nvram[DSRTC_MINUTES] = _tme_dsrtc_bcd_out(now_tm->tm_min);
  dsrtc->tme_dsrtc_nvram[DSRTC_HOURS] = _tme_dsrtc_bcd_out(now_tm->tm_hour);
  dsrtc->tme_dsrtc_nvram[DSRTC_DAY] = _tme_dsrtc_bcd_out(now_tm->tm_wday + 1);
  dsrtc->tme_dsrtc_nvram[DSRTC_DATE] = _tme_dsrtc_bcd_out(now_tm->tm_mday);
  dsrtc->tme_dsrtc_nvram[DSRTC_MONTH] = _tme_dsrtc_bcd_out(now_tm->tm_mon + 1);
  dsrtc->tme_dsrtc_nvram[DSRTC_YEAR] = _tme_dsrtc_bcd_out(year % 99);
  if (year > 99) {
    dsrtc->tme_dsrtc_nvram[DSRTC_MONTH] |= TME_BIT(7); /* century */
  }
}

static void
_tme_dsrtc_advance_cursor(struct tme_dsrtc *dsrtc)
{
  dsrtc->tme_dsrtc_cursor++;
  if (dsrtc->tme_dsrtc_cursor > _tme_dsrtc_cursor_max[dsrtc->tme_dsrtc_type]) {
    dsrtc->tme_dsrtc_cursor = 0;
  }
}

static void
_tme_dsrtc_set_cursor(struct tme_dsrtc *dsrtc, tme_uint8_t val)
{
  if (val <= _tme_dsrtc_cursor_max[dsrtc->tme_dsrtc_type]) {
    dsrtc->tme_dsrtc_cursor = val;
  } else {
    dsrtc->tme_dsrtc_cursor = val
       % (_tme_dsrtc_cursor_max[dsrtc->tme_dsrtc_type] + 1);
  }
}

static void
_tme_dsrtc_write(struct tme_dsrtc *dsrtc, tme_uint8_t val)
{
  dsrtc->tme_dsrtc_nvram[dsrtc->tme_dsrtc_cursor] = val;
}

static tme_uint8_t
_tme_dsrtc_read(struct tme_dsrtc *dsrtc)
{
  return dsrtc->tme_dsrtc_nvram[dsrtc->tme_dsrtc_cursor];
}

static int
_tme_dsrtc_slave_start(struct tme_i2c_connection *conn_i2c, tme_uint8_t addr)
{
  struct tme_dsrtc *dsrtc =
    conn_i2c->tme_i2c_connection.tme_connection_element->tme_element_private;

  if (addr >> 1 != DSRTC_I2C_ADDR) {
    return (ESRCH);
  }

  tme_mutex_lock(&dsrtc->tme_dsrtc_mutex);

  if (addr & 1) {
    if (dsrtc->tme_dsrtc_state != DSRTC_STATE_READ) {
      _tme_dsrtc_update_rtc(dsrtc);
    }
    dsrtc->tme_dsrtc_state = DSRTC_STATE_READ;
  } else {
    dsrtc->tme_dsrtc_state = DSRTC_STATE_CURSOR;
  }

  tme_mutex_unlock(&dsrtc->tme_dsrtc_mutex);

  return (TME_OK);
}

static int
_tme_dsrtc_slave_stop(struct tme_i2c_connection *conn_i2c)
{
  struct tme_dsrtc *dsrtc =
    conn_i2c->tme_i2c_connection.tme_connection_element->tme_element_private;

  tme_mutex_lock(&dsrtc->tme_dsrtc_mutex);
  dsrtc->tme_dsrtc_state = DSRTC_STATE_IDLE;
  tme_mutex_unlock(&dsrtc->tme_dsrtc_mutex);

  return (TME_OK);
}

static int
_tme_dsrtc_slave_write(struct tme_i2c_connection *conn_i2c, tme_uint8_t data)
{
  struct tme_dsrtc *dsrtc =
    conn_i2c->tme_i2c_connection.tme_connection_element->tme_element_private;

  tme_mutex_lock(&dsrtc->tme_dsrtc_mutex);
  if (dsrtc->tme_dsrtc_state == DSRTC_STATE_CURSOR) {
    _tme_dsrtc_set_cursor(dsrtc, data);
    dsrtc->tme_dsrtc_state = DSRTC_STATE_WRITE;
  } else if (dsrtc->tme_dsrtc_state == DSRTC_STATE_WRITE) {
    _tme_dsrtc_write(dsrtc, data);
    _tme_dsrtc_advance_cursor(dsrtc);
  }
  tme_mutex_unlock(&dsrtc->tme_dsrtc_mutex);

  return (TME_OK);
}

static int
_tme_dsrtc_slave_read(struct tme_i2c_connection *conn_i2c, tme_uint8_t *datap,
                      int nack)
{
  struct tme_dsrtc *dsrtc =
    conn_i2c->tme_i2c_connection.tme_connection_element->tme_element_private;

  tme_mutex_lock(&dsrtc->tme_dsrtc_mutex);
  if (dsrtc->tme_dsrtc_state == DSRTC_STATE_READ) {
    *datap = _tme_dsrtc_read(dsrtc);
    _tme_dsrtc_advance_cursor(dsrtc);
  } else {
    *datap = 0xff;
  }
  tme_mutex_unlock(&dsrtc->tme_dsrtc_mutex);

  return (TME_OK);
}

/* this scores a dsrtc connection: */
static int
_tme_dsrtc_connection_score(struct tme_connection *conn,
                            unsigned int *_score)
{
  struct tme_dsrtc *dsrtc = conn->tme_connection_element->tme_element_private;

  /* both sides must be i2c connections: */
  assert(conn->tme_connection_type == TME_CONNECTION_I2C);
  assert(conn->tme_connection_other->tme_connection_type == TME_CONNECTION_I2C);

  /* this device must be free: */
  assert(dsrtc->tme_dsrtc_i2c_connection == NULL);
  (void)dsrtc;

  *_score = 1;
  return (TME_OK);
}

/* this makes a new i2c connection: */
static int
_tme_dsrtc_connection_make(struct tme_connection *conn, unsigned int state)
{
  struct tme_dsrtc *dsrtc = conn->tme_connection_element->tme_element_private;
  struct tme_i2c_connection *conn_i2c_other;

  conn_i2c_other = (struct tme_i2c_connection *) conn->tme_connection_other;

  /* both sides must be serial connections: */
  assert(conn->tme_connection_type == TME_CONNECTION_I2C);
  assert(conn->tme_connection_other->tme_connection_type == TME_CONNECTION_I2C);

  if (state == TME_CONNECTION_FULL) {

    /* save our connection: */
    dsrtc->tme_dsrtc_i2c_connection = conn_i2c_other;
  }

  return (TME_OK);
}

/* this breaks a connection: */
static int
_tme_dsrtc_connection_break(struct tme_connection *conn,
                            unsigned int state)
{
  abort();
}

/* this makes a new connection side for dsrtc: */
static int
_tme_dsrtc_connections_new(struct tme_element *element,
                           const char * const *args,
                           struct tme_connection **_conns,
                           char **_output)
{
  struct tme_dsrtc *dsrtc = element->tme_element_private;
  struct tme_i2c_connection *conn_i2c;
  struct tme_connection *conn;

  /* if we don't already have an i2c connection, make one: */
  if (dsrtc->tme_dsrtc_i2c_connection == NULL) {

    conn_i2c = tme_new0(struct tme_i2c_connection, 1);
    conn = &conn_i2c->tme_i2c_connection;

    /* fill in the generic connection: */
    conn->tme_connection_next = *_conns;
    conn->tme_connection_type = TME_CONNECTION_I2C;
    conn->tme_connection_score = _tme_dsrtc_connection_score;
    conn->tme_connection_make = _tme_dsrtc_connection_make;
    conn->tme_connection_break = _tme_dsrtc_connection_break;

    /* fill in the i2c connection: */
    conn_i2c->tme_i2c_connection_start = _tme_dsrtc_slave_start;
    conn_i2c->tme_i2c_connection_stop = _tme_dsrtc_slave_stop;
    conn_i2c->tme_i2c_connection_write = _tme_dsrtc_slave_write;
    conn_i2c->tme_i2c_connection_read = _tme_dsrtc_slave_read;

    /* return the connection side possibility: */
    *_conns = conn;
  }

  /* done: */
  return (TME_OK);
}

/* the new dsrtc function: */
TME_ELEMENT_NEW_DECL(tme_ic_dsrtc) {
  struct tme_dsrtc *dsrtc;
  int type;
  int arg_i;
  int usage;
  unsigned int epoch = 1970;	/* default to unix eopch */

  /* check our arguments: */
  usage = 0;
  arg_i = 1;
  type = -1;
  for (;;) {

    if (TME_ARG_IS(args[arg_i + 0], "type")) {
      if (args[arg_i + 1] == NULL) {
        tme_output_append_error(_output,
                                "%s, ",
                                _("missing type"));
        usage = 1;
        break;
      }
      else if (TME_ARG_IS(args[arg_i + 1], "ds1307")) {
        type = DSRTC_TYPE_DS1307;
      }
      else if (TME_ARG_IS(args[arg_i + 1], "ds3231")) {
        type = DSRTC_TYPE_DS3231;
      }
      else if (TME_ARG_IS(args[arg_i + 1], "ds3232")) {
        type = DSRTC_TYPE_DS3232;
      }
      else {
        tme_output_append_error(_output,
                                "%s %s, ",
                                _("bad type"),
                                args[arg_i + 1]);
        usage = 1;
        break;
      }
      arg_i += 2;
    }

    /* if we ran out of arguments: */
    else if (args[arg_i] == NULL) {

      break;
    }

    /* otherwise this is a bad argument: */
    else {
      tme_output_append_error(_output,
                              "%s %s, ",
                              args[arg_i],
                              _("unexpected"));
      usage = 1;
      break;
    }
  }

  if (type == -1) {
    tme_output_append_error(_output,
                            "%s, ",
                            _("missing type"));
    usage = 1;
  }

  if (usage) {
    tme_output_append_error(_output,
                            "%s %s type { ds1307 | ds3231 | ds3232 }",
                            _("usage:"),
                            args[0]);
    return (EINVAL);
  }

  /* start of a new dsrtc structure. */
  dsrtc = tme_new0(struct tme_dsrtc, 1);
  dsrtc->tme_dsrtc_type = type;
  dsrtc->tme_dsrtc_element = element;
  tme_mutex_init(&dsrtc->tme_dsrtc_mutex);

  dsrtc->tme_dsrtc_epoch_year = epoch;

  /* fill the element: */
  element->tme_element_private = dsrtc;
  element->tme_element_connections_new = _tme_dsrtc_connections_new;

  return (TME_OK);
}
