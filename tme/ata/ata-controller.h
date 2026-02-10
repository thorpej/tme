/* tme/ata/ata-controller.h - public header file for ATA controller emulation */

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

#ifndef _TME_ATA_ATA_CONTROLLER_H
#define _TME_ATA_ATA_CONTROLLER_H

#include <tme/common.h>

/* includes: */
#include <tme/element.h>
#include <tme/generic/bus.h>

/* macros: */
#define TME_ATA_CONTROLLER_SOCKET_0 (0)

/* structures: */
struct tme_ata_controller_socket {
  
  /* the version number of this structure: */
  unsigned int tme_ata_controller_socket_version;

  /* the bus address shift: */
  unsigned int tme_ata_controller_socket_addr_shift;

  /* the system bus byte lane the chip is wired to: */
  unsigned int tme_ata_controller_socket_port_least_lane;
};

#endif /* !_TME_ATA_ATA_CONTROLLER_H */
