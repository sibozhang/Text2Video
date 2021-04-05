# ----------------------------------------------------------- 
#                                                             
#                          ___                                
#                       |_| | |_/   SPEECH                    
#                       | | | | \   RECOGNITION               
#                       =========   SOFTWARE                  
#                                                             
#                                                             
# ----------------------------------------------------------- 
#         Copyright: Cambridge University
#          1995-2006 Engineering Department
#                    http://htk.eng.cam.ac.uk
#                    http://mi.eng.cam.ac.uk
#                 
#   Use of this software is governed by a License Agreement   
#    ** See the file License for the Conditions of Use  **    
#    **     This banner notice must not be removed      **    
#                                                             
# ----------------------------------------------------------- 
# File: Makefile.  Generated from Makefile.in by configure.
# ----------------------------------------------------------- 

SHELL = /bin/sh
srcdir = .
top_srcdir = .

prefix = /usr/local
exec_prefix = ${prefix}
bindir = ${exec_prefix}/bin
sbindir = ${exec_prefix}/sbin
libexecdir = ${exec_prefix}/libexec
datadir = ${prefix}/share
sysconfdir = ${prefix}/etc
sharedstatedir = ${prefix}/com
localstatedir = ${prefix}/var
libdir = ${exec_prefix}/lib
infodir = ${prefix}/share/info
mandir = ${prefix}/share/man
includedir = ${prefix}/include
oldincludedir = /usr/include

CC = gcc
CPPFLAGS = 
CFLAGS = $(CPPFLAGS) -ansi -D_SVID_SOURCE -DOSS_AUDIO -D'ARCH="x86_64"' -Wall -Wno-switch -g -O2
LDFLAGS = -L/usr/X11R6/lib 
LIBS = -lm 
INSTALL = /usr/bin/install -c

DESTDIR =

pkgdatadir = $(datadir)/@PACKAGE@
pkglibdir = $(libdir)/@PACKAGE@
pkgincludedir = $(includedir)/@PACKAGE@

top_builddir = .

ACLOCAL = @ACLOCAL@
AUTOCONF = @AUTOCONF@
AUTOMAKE = @AUTOMAKE@
AUTOHEADER = @AUTOHEADER@

INSTALL = /usr/bin/install -c
INSTALL_PROGRAM = ${INSTALL} $(AM_INSTALL_PROGRAM_FLAGS)
INSTALL_DATA = ${INSTALL} -m 644
INSTALL_SCRIPT = ${INSTALL}
transform = s,x,x,
MAKEINFO = @MAKEINFO@
PACKAGE = @PACKAGE@
VERSION = @VERSION@

HTKLIB  = HTKLib
HLMLIB  = HLMLib
HTKTOOLS = HTKTools
HLMTOOLS = HLMTools
LVREC = HTKLVRec
HTKBOOK = HTKBook
SUBDIRS = $(HTKLIB) $(HLMLIB) $(HTKTOOLS) $(HLMTOOLS) $(LVREC) $(BOOK) 
ACLOCAL_M4 = $(top_srcdir)/aclocal.m4
mkinstalldirs = $(SHELL) $(top_srcdir)/mkinstalldirs
CONFIG_CLEAN_FILES = 
DIST_COMMON =  README ChangeLog Makefile.am \
Makefile.in aclocal.m4 configure configure.ac install-sh missing \
mkinstalldirs


DISTFILES = $(DIST_COMMON) $(SOURCES) $(HEADERS) $(TEXINFOS) $(EXTRA_DIST)

TAR = gtar
GZIP_ENV = --best

.SUFFIXES:

# build rules
all: htktools hlmtools
$(HTKLIB)/HTKLib.a:
	(cd $(HTKLIB) && $(MAKE) HTKLib.a) \
	  || case "$(MFLAGS)" in *k*) fail=yes;; *) exit 1;; esac;
htklib: $(HTKLIB)/HTKLib.a
$(HTKLIB)/HTKLiblv.a:
	(cd $(HTKLIB) && $(MAKE) HTKLiblv.a) \
	  || case "$(MFLAGS)" in *k*) fail=yes;; *) exit 1;; esac;
htkliblv: $(HTKLIB)/HTKLiblv.a
$(HLMLIB)/HLMLib.a:
	(cd $(HLMLIB) && $(MAKE) all) \
	  || case "$(MFLAGS)" in *k*) fail=yes;; *) exit 1;; esac;
hlmlib: $(HLMLIB)/HLMLib.a
htktools: $(HTKLIB)/HTKLib.a
	(cd $(HTKTOOLS) && $(MAKE) all) \
	  || case "$(MFLAGS)" in *k*) fail=yes;; *) exit 1;; esac;
hlmtools: $(HLMLIB)/HLMLib.a
	(cd $(HLMTOOLS) && $(MAKE) all) \
	  || case "$(MFLAGS)" in *k*) fail=yes;; *) exit 1;; esac;
hdecode: $(HTKLIB)/HTKLiblv.a
	(cd $(LVREC) && $(MAKE) all) \
	  || case "$(MFLAGS)" in *k*) fail=yes;; *) exit 1;; esac;
book: 
	(cd $(HTKBOOK) && $(MAKE) all) \
	  || case "$(MFLAGS)" in *k*) fail=yes;; *) exit 1;; esac;

# installation
install-htktools: htktools
	(cd $(HTKTOOLS) && $(MAKE) install) \
	|| case "$(MFLAGS)" in *k*) fail=yes;; *) exit 1;; esac;
install-hlmtools: hlmtools
	(cd $(HLMTOOLS) && $(MAKE) install) \
	|| case "$(MFLAGS)" in *k*) fail=yes;; *) exit 1;; esac;
install-hdecode:
	(cd $(LVREC) && $(MAKE) install) \
	|| case "$(MFLAGS)" in *k*) fail=yes;; *) exit 1;; esac;
install-book: book
	(cd $(HTKBOOK) && $(MAKE) install) \
	|| case "$(MFLAGS)" in *k*) fail=yes;; *) exit 1;; esac;

clean:
	/bin/rm -f *~
	@for dir in $(SUBDIRS); do \
	  (cd $$dir && $(MAKE) clean) \
	  || case "$(MFLAGS)" in *k*) fail=yes;; *) exit 1;; esac; \
	done && test -z "$$fail"
distclean: clean
	/bin/rm -f Makefile config.h config.status config.cache config.log
	@for dir in $(SUBDIRS); do \
	  (cd $$dir && $(MAKE) distclean) \
	  || case "$(MFLAGS)" in *k*) fail=yes;; *) exit 1;; esac; \
	done && test -z "$$fail"

install: install-htktools install-hlmtools
docs: book

.PHONY: all doc install clean distclean htklib-decode \
	htktools hlmtools hdecode docs book \
	install-htktools install-hlmtools install-hdecode install-book

