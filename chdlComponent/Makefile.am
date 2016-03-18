# -*- Makefile -*-
#
#

AM_CPPFLAGS = \
	$(BOOST_CPPFLAGS) \
	$(MPI_CPPFLAGS) \
	$(CHDL_CPPFLAGS)

compdir = $(pkglibdir)
comp_LTLIBRARIES = libchdlComponent.la
libchdlComponent_la_SOURCES = \
	chdlComponent.h \
	chdlComponent.cc
	atlsim.cpp

EXTRA_DIST = 

libchdlComponent_la_LDFLAGS = -module -avoid-version $(CHDL_LDFLAGS)
