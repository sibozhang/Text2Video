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
# File: HTKTools/Makefile.  Generated from Makefile.in by configure.
# ----------------------------------------------------------- 

SHELL =	/bin/sh
inc = ../HTKLib
HTKLIB = $(inc)/HTKLib.a 
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

CC      = 	gcc
CFLAGS  = 	-ansi -D_SVID_SOURCE -DOSS_AUDIO -D'ARCH="x86_64"' -Wall -Wno-switch -g -O2 -I$(inc) -DPHNALG
LDFLAGS = 	-L/usr/X11R6/lib  -lm
INSTALL = 	/usr/bin/install -c
PROGS   = 	 HBuild HCompV HCopy HDMan \
		HERest HHEd HInit HLEd 	HList \
		HLRescore HLStats HMMIRest HParse \
		HQuant HRest HResults HSGen HSmooth \
		HVite 
all: $(PROGS)

%: %.c $(HTKLIB) 
	if [ ! -d $(bindir) -a X_ = X_yes ] ; then mkdir -p $(bindir) ; fi
	if [ x$@ = xHSLab ] ; then \
		$(CC) -o $@ $(CFLAGS) $^ $(LDFLAGS) -lX11 ; \
		else \
		$(CC) -o $@ $(CFLAGS) $^ $(LDFLAGS) ; fi
	if [ X_ = X_yes ] ; then $(INSTALL) -m 755 $@ $(bindir)  ; fi

strip: $(PROGS)
	-strip $(PROGS)

clean:
	-rm -f *.o 

cleanup:
	-rm -f *.o $(PROGS) *.exe

distclean:
	-rm -f *.o $(PROGS) *.exe Makefile

install: mkinstalldir $(PROGS)
	for program in $(PROGS) ; do $(INSTALL) -m 755 $${program} $(bindir) ; done

mkinstalldir:
	if [ ! -d $(bindir) ] ; then mkdir $(bindir) ; fi


.PHONY: all strip clean cleanup distclean install mkinstalldir
