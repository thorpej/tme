## Process this file with automake to produce Makefile.in
# Makefile.am for The Machine Emulator ata/:

AUTOMAKE_OPTIONS = 1.4 gnu

AM_CPPFLAGS = -I$(top_srcdir) -I$(srcdir) -I. -D_TME_IMPL

##SUBDIRS = @TME_ATA_SUBDIRS@
##DIST_SUBDIRS =

pkglib_LTLIBRARIES = tme_ata.la

tme_ata_la_SOURCES = \
	ata-controller.c
tme_ata_la_LDFLAGS = -rpath $(pkglibdir) -module -version-info 0:0:0
tme_ata_la_LIBADD = $(top_builddir)/libtme/libtme.la \
	$(top_builddir)/generic/libtme-generic.la

include $(top_srcdir)/modules
