# -*- Makefile -*-
#
#

AM_CPPFLAGS = \
	$(BOOST_CPPFLAGS) \
	$(MPI_CPPFLAGS) \
	$(HARPLIB_CPPFLAGS)

compdir = $(pkglibdir)
comp_LTLIBRARIES = libpokesim.la
libpokesim_la_SOURCES = \
	pokesim.h \
	pokesim.cc

EXTRA_DIST = README

libpokesim_la_LDFLAGS = -module -avoid-version $(HARPLIB_LDFLAGS)
libpokesim_la_LIBADD = -lharplib
