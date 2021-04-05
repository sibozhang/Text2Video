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
# File: HTKLib/Makefile.  Generated from Makefile.in by configure.
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

hlib = .
HLIBS = ${exec_prefix}/lib/HTKLib.a

CC     = gcc
CFLAGS := $(CFLAGS) -ansi -D_SVID_SOURCE -DOSS_AUDIO -D'ARCH="x86_64"' -Wall -Wno-switch -g -O2 -I. -DPHNALG
RANLIB = ranlib
INSTALL= /usr/bin/install -c
target = HTKLib.a
objects = HGraf.null.o esig_asc.o \
	esig_edr.o \
	esignal.o \
	esig_nat.o \
	HAdapt.o \
	HArc.o \
	HAudio.o \
	HDict.o \
	HExactMPE.o \
	HFB.o \
	HFBLat.o \
	HLabel.o \
	HLat.o \
	HLM.o \
	HMap.o \
	HMath.o \
	HMem.o \
	HModel.o \
	HNet.o \
	HParm.o \
	HRec.o \
	HShell.o \
	HSigP.o \
	HTrain.o \
	HUtil.o \
	HVQ.o \
	HWave.o \
	strarr.o 

lvobjects = $(basename HGraf.null.o).lv.o esig_asc.lv.o \
	esig_edr.lv.o \
	esignal.lv.o \
	esig_nat.lv.o \
	HAdapt.lv.o \
	HArc.lv.o \
	HAudio.lv.o \
	HDict.lv.o \
	HExactMPE.lv.o \
	HFB.lv.o \
	HFBLat.lv.o \
	HLabel.lv.o \
	HLat.lv.o \
	HLM.lv.o \
	HMap.lv.o \
	HMath.lv.o \
	HMem.lv.o \
	HModel.lv.o \
	HNet.lv.o \
	HParm.lv.o \
	HRec.lv.o \
	HShell.lv.o \
	HSigP.lv.o \
	HTrain.lv.o \
	HUtil.lv.o \
	HVQ.lv.o \
	HWave.lv.o \
	strarr.lv.o 

all:    HTKLib.a HTKLiblv.a

# This is the library used by HTKTools and HLMTools
HTKLib.a: $(objects)
	if [ -f $@ ] ; then  /bin/rm $@ ; fi
	ar rv $@ $^
	$(RANLIB) $@

# This version is used by HDecode (HTKLVRec)
HTKLiblv.a: $(lvobjects)
	if [ -f $@ ] ; then  /bin/rm $@ ; fi
	ar rv $@ $^
	$(RANLIB) $@

%.lv.o: %.c
	$(CC) -DNO_LAT_LM $(CFLAGS) -c -o $@ $<

.PHONY: clean cleanup depend mkinstalldir install

clean:
	-rm -f $(objects) $(lvobjects) HTKLib.a HTKLiblv.a

cleanup:
	-rm -f $(objects) $(lvobjects)

depend:
	-makedepend -Y *.c

distclean: clean
	-rm -f Makefile

install: HTKLib.a HTKLiblv.a  mkinstalldir 
	$(INSTALL) -m 755 HTKLib.a HTKLiblv.a $(libdir)

mkinstalldir:
	-mkdir -p $(libdir)
