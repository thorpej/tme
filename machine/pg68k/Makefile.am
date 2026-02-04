## Process this file with automake to produce Makefile.in
# Makefile.am for The Machine Emulator machine/pg68k:

AUTOMAKE_OPTIONS = 1.4 gnu

AM_CPPFLAGS = -I$(top_srcdir) -I$(srcdir) -I. -D_TME_IMPL

pkglib_LTLIBRARIES = libtme-machine-pg68k.la
libtme_machine_pg68k_la_SOURCES = pg68k-mmu.c pg68k-timer.c
libtme_machine_pg68k_la_LDFLAGS = -version-info 0:0:0
libtme_machine_pg68k_la_LIBADD = $(top_builddir)/libtme/libtme.la \
	$(top_builddir)/generic/libtme-generic.la
