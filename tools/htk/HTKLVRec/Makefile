# HTKLVRec/Makefile.  Generated from Makefile.in by configure.

############################################################
# Makefile for HTKLVRec
# 
# N.B. you must make HTKLiblv.a (in the HTKLib directory first)
#
# "make" or "make all": to build all all binaries
# "make install": install all binaries in ${exec_prefix}/bin
# "make clean": remove object files
# "make cleanup": remove object files and binaries
############################################################

SHELL   = 	/bin/sh
inc     = 	../HTKLib
HTKLIB  = 	$(inc)/HTKLib.a 
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
CFLAGS  := 	-DNO_LAT_LM -m32 -ansi -D_SVID_SOURCE -DOSS_AUDIO -D'ARCH="x86_64"' -Wall -Wno-switch -g -O2 -I$(inc)
LDFLAGS = 	-L/usr/X11R6/lib  -lm
INSTALL = 	/usr/bin/install -c
HTKLIB = $(inc)/HTKLiblv.a
HEADER = HLVLM.h  HLVModel.h  HLVNet.h  HLVRec.h config.h

all: HDecode HDecode.mod


# binaries
HDecode: HDecode.orig.o HLVNet.orig.o HLVRec.orig.o HLVLM.orig.o HLVModel.orig.o $(HTKLIB)
	$(CC) $(CFLAGS)	 -o HDecode HDecode.orig.o HLVNet.orig.o HLVRec.orig.o HLVLM.orig.o \
	HLVModel.orig.o $(HTKLIB) $(LDFLAGS)

HDecode.mod: HDecode.mod.o HLVNet.mod.o HLVRec.mod.o HLVLM.mod.o HLVModel.mod.o $(HTKLIB)
	$(CC) $(CFLAGS)	 -o HDecode.mod HDecode.mod.o HLVNet.mod.o HLVRec.mod.o HLVLM.mod.o \
	HLVModel.mod.o $(HTKLIB) $(LDFLAGS)

# normal version object files
HLVModel.orig.o: HLVModel.c $(HEADER)
	$(CC) -c $(CFLAGS) $<
	mv HLVModel.o $@

HLVNet.orig.o: HLVNet.c $(HEADER)
	$(CC) -c $(CFLAGS) $<
	mv HLVNet.o $@

HLVLM.orig.o: HLVLM.c $(HEADER)
	$(CC) -c $(CFLAGS) $<
	mv HLVLM.o $@

HLVRec.orig.o: HLVRec.c HLVRec-GC.c	HLVRec-LM.c	 HLVRec-misc.c	HLVRec-outP.c \
		  HLVRec-propagate.c  HLVRec-traceback.c $(HEADER)
	$(CC) -c $(CFLAGS) HLVRec.c 
	mv HLVRec.o $@

HDecode.orig.o: HDecode.c $(HEADER)
	$(CC) -c $(CFLAGS) $<
	mv HDecode.o $@



# .mod version object files
HLVModel.mod.o: HLVModel.c $(HEADER)
	$(CC) -c -DHDECODE_MOD $(CFLAGS) $<
	mv HLVModel.o $@

HLVNet.mod.o: HLVNet.c $(HEADER)
	$(CC) -c -DHDECODE_MOD $(CFLAGS) $<
	mv HLVNet.o $@

HLVLM.mod.o: HLVLM.c $(HEADER)
	$(CC) -c -DHDECODE_MOD $(CFLAGS) $<
	mv HLVLM.o $@

HLVRec.mod.o: HLVRec.c HLVRec-GC.c	HLVRec-LM.c	 HLVRec-misc.c	HLVRec-outP.c \
		  HLVRec-propagate.c  HLVRec-traceback.c $(HEADER)
	$(CC) -c -DHDECODE_MOD $(CFLAGS) HLVRec.c 
	mv HLVRec.o $@

HDecode.mod.o: HDecode.c $(HEADER)
	$(CC) -c -DHDECODE_MOD $(CFLAGS) $<
	mv HDecode.o $@


# housekeeping rules
strip: HDecode
	-strip HDecode HDecode.mod

clean:
	-rm -f *.o 

cleanup:
	-rm -f *.o HDecode HDecode.mod

distclean:
	-rm -f *.o HDecode HDecode.mod Makefile

install: mkinstalldir
	$(INSTALL) -m 755 HDecode $(bindir)
	$(INSTALL) -m 755 HDecode.mod $(bindir)

mkinstalldir:
	-mkdir -p $(bindir)

.PHONY: std mod all strip clean cleanup distclean install mkinstalldir


