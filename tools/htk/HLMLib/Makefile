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
#          1995-2005 Engineering Department
#                    http://htk.eng.cam.ac.uk
#                    http://mi.eng.cam.ac.uk
#                 
#   Use of this software is governed by a License Agreement   
#    ** See the file License for the Conditions of Use  **    
#    **     This banner notice must not be removed      **    
#                                                             
# ----------------------------------------------------------- 
# File: HLMLib/Makefile.  Generated from Makefile.in by configure.
# ----------------------------------------------------------- 

SHELL = /bin/sh
srcdir = .
top_srcdir = ..

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
CFLAGS = -ansi -D_SVID_SOURCE -DOSS_AUDIO -D'ARCH="x86_64"' -Wall -Wno-switch -g -O2 -DSANITY -I. -I../HTKLib/
RANLIB = ranlib
INSTALL = /usr/bin/install -c
target  = HLMLib.a
modules = LModel.o LPMerge.o LPCalc.o LUtil.o LWMap.o LCMap.o LGBase.o

all: $(target)

$(target): $(modules)
	if [ -f $@ ] ; then  /bin/rm $@ ; fi
	ar rv $@ $^
	$(RANLIB) $@

.PHONY: clean cleanup depend mkinstalldir install

clean:
	-rm -f $(modules) $(target)

cleanup:
	-rm -f $(modules)

depend:
	-makedepend -Y *.c

distclean: clean
	-rm -f Makefile

install: $(target) mkinstalldir 
	$(INSTALL) -m 755 $< $(libdir)

mkinstalldir:
	-mkdir -p $(libdir)
