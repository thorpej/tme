## Process this file with automake to produce Makefile.in
# Makefile.am for The Machine Emulator machine/phaethon1:

AUTOMAKE_OPTIONS = 1.4 gnu

AM_CPPFLAGS = -I$(top_srcdir) -I$(srcdir) -I. -D_TME_IMPL

pkglib_LTLIBRARIES = tme_machine_phaethon1.la
tme_machine_phaethon1_la_SOURCES =	\
	phaethon1-mainbus.c		\
	phaethon1-mmu.c			\
	phaethon1-control.c		\
	phaethon1-impl.h
tme_machine_phaethon1_la_LDFLAGS = -module -version-info 0:0:0
tme_machine_phaethon1_la_LIBADD = $(top_builddir)/libtme/libtme.la \
	../pg68k/libtme-machine-pg68k.la

examplesdir = $(datadir)/examples/tme
examples_DATA = PHAETHON1

EXTRA_DIST = $(examples_DATA)

include $(top_srcdir)/modules
