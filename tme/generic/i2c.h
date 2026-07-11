/* tme/generic/i2c.h - header file for a generic i2c support: */

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

#ifndef _TME_GENERIC_I2C_H
#define _TME_GENERIC_I2C_H

#include <tme/common.h>

/* includes: */
#include <tme/element.h>

/* macros: */

/* a serial connection: */
struct tme_i2c_connection {

  /* the generic connection side: */
  struct tme_connection tme_i2c_connection;

  /* our link on a linked list of all i2c connections.  the slave
     side of the connection is linked into the master's list.  */
  struct tme_i2c_connection *tme_i2c_connection_next;

  /* this is called when the master sends a START condition.  we are
     given a chance to ACK (TME_OK) or NACK (ESRCH) the address.
     the address that's passed to us is how it would appear on the wire:
     (addr << 1) | rw (r=1, w=0).  */
  int (*tme_i2c_connection_start) _TME_P((struct tme_i2c_connection *,
                                          tme_uint8_t));

  /* this is called when the master sends a STOP condition.  this is
     mainly to inform us that the transfer is complete.  */
  int (*tme_i2c_connection_stop) _TME_P((struct tme_i2c_connection *));

  /* this is called when the other side wants to write a byte ot data
     to us.  we respond with ACK (TME_OK) or NACK (! TME_OK).  */
  int (*tme_i2c_connection_write) _TME_P((struct tme_i2c_connection *,
                                          tme_uint8_t));

  /* this is called when the other side wants to read a byte of data
     from us.  we respond with ACK (TME_OK) or NACK (! TME_OK).  we
     are told in the nack argument if this is the last byte of a read
     transfer.  */
  int (*tme_i2c_connection_read) _TME_P((struct tme_i2c_connection *,
                                         tme_uint8_t *, int));
};

#endif /* !_TME_GENERIC_I2C_H */
