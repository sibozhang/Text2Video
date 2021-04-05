		Hidden Markov Toolkit (HTK) 3.4.1

Use of this software is governed by a license agreement, the terms and
conditions of which are set forth in the file LICENSE in the
top-level HTK installation directory.  Please read this file carefully
as use of this software implies acceptance of the conditions described
therein.

Introduction
============

HTK is a toolkit for use in research into automatic speech recognition
and has been developed by the Machine Intelligence Laboratory
(formerly know as the Speech Vision Robotics Group) at the
Cambridge University Engineering Department (http://mi.eng.cam.ac.uk) 
and Entropic Ltd (http://www.entropic.com).

Please visit the HTK homepage at the following address for more
information about HTK:

    http://htk.eng.cam.ac.uk/

A number of mailing lists have been established to help users build
and understand HTK, for details see

    http://htk.eng.cam.ac.uk/mailing/subscribe_mail.shtml


License
=======

HTK is made available free of charge and can be downloaded from the
website mentioned above. However it may not be redistributed,
i.e. you must register at the website and download it from
there. Details about the terms under which HTK is made available can
be found in the LICENSE file.


Compiling & Installing HTK under UNIX/Linux, OS X or Cygwin
===========================================================

After unpacking the sources, cd to the htk directory.  

There are now two ways to install HTK, the "traditional" and the
"new".  Up to now HTK has always installed its tools as they were
built, and installed them to a directory such as "bin.linux" so that
binaries for different architectures can be installed in a home
directory say.  If you want to install in this way, please add the
option "--enable-trad-htk" when you run configure.

The "new" method installs by default into /usr/local/bin (equivalent
to a configure option of "--prefix=/usr/local").

1. decide which of the above methods you wish to use
2. cd to htk, then run ./configure (with appropriate options, run
   "./configure --help" if unsure).
   If you don't want to build the programs in HLMTools add the 
   --disable-hlmtools option.
3. make all
4. make install

Running "make install" will install them.  This step may need to be
done as root, if you are not installing them in your home directory.

Notes for particular Unix variants:
Solaris: if "make" isn't installed you may need to add /opt/sfw/bin
and /usr/ccs/bin to your path and run "./configure MAKE=gmake" with
any other options you require.  Then run "gmake" instead of "make",
alternatively you can create a symbolic link called "make" somewhere
it your path to /opt/sfw/bin/gmake


HDecode
=======

If you are also building HDecode (available from the HTK website, under a
different licence from HTK), you will firstly need to unpack the HDecode
source code (in the same directory in which you unpacked the HTK
sources). Follow the steps above for building HTK first, then add
these steps to the build process:

5. make hdecode
6. make install-hdecode


Compiling & Installing HTK under Windows
========================================
Prerequisites:
    * HTK has been verified to compile using Microsoft Visual Studio.
    * For testing, you will require a Perl interpreter such as
      ActivePerl.  
    * You will need a tool such as 7-zip or winzip (commercial) for unpacking
      the HTK source code archive.
    * It is helpful if you have some familiarity with using the DOS
      command line interface, as you will need to interact with it in
      order to compile, install and run HTK.
    * Ensure that your PATH contains 
      C:\Program Files\Microsoft Visual Studio .NET 2003\Vc7\bin
      Or if you are using older versions:
      C:\Program Files\Microsoft Visual Studio\VC98\bin
      

Compilation:
   1. Unpack the HTK sources using 7-zip.
   2. Open a DOS command window: Click Start, select Run type cmd at
      the prompt and click OK.
   3. cd into the directory in which you unpacked the sources.
   4. cd into the htk directory. Type:

      cd htk

   5. Create a directory for the library and tools. Type:

      mkdir bin.win32

   6. Run VCVARS32 (it should be in your path, see prerequisites above)
   7. Build the HTK Library, which provides the common functionality
      used by the HTK Tools. Enter the following commands:

	  cd HTKLib
	  nmake /f htk_htklib_nt.mkf all
	  cd ..

   8. Build the HTK Tools

	  cd HTKTools
	  nmake /f htk_htktools_nt.mkf all
	  cd ..
	  cd HLMLib
	  nmake /f htk_hlmlib_nt.mkf all
	  cd ..
	  cd HLMTools
	  nmake /f htk_hlmtools_nt.mkf all
	  cd ..

Installation:
The HTK tools have now been built and are in the bin.win32
directory. You should add this directory to your PATH, so that you can
run them easily from the command line in future.


Testing the Installation
========================

Among the samples on the HTK website you'll find the HTKDemo package
that can be used to test your installation. See
http://htk.eng.cam.ac.uk/download.shtml for download instructions.

As an initial test of the installation please run the HTK
demonstration using the configuration file
HTKDemo/configs/monPlainM1S1.dcf. There is a README file in the
HTKDemo directory explaining the operation of the demonstration in
detail but, in short, you need to run the demonstration script passing
it the configuration file configs/monPlainM1S1.dcf as input. 
To test the language modelling tools you should follow the tutorial
in the HTK book, using the files in the LMTutorial/ directory.

Before running the demo make sure you have compiled all the HTK tools
and the executables are in your PATH, i.e. just typing 'HInit' at the
commandline prints a short usage summary. To run the demonstration
type:

$ cd HTKDemo
$ ./runDemo configs/monPlainM1S1.dcf

The recognition results obtained should match the following.

On the training set:
------------------------ Overall Results --------------------------
SENT: %Correct=0.00 [H=0, S=7, N=7]
WORD: %Corr=77.63, Acc=74.89 [H=170, D=37, S=12, I=6, N=219]
===================================================================

On the test set:
------------------------ Overall Results --------------------------
SENT: %Correct=0.00 [H=0, S=3, N=3]
WORD: %Corr=63.91, Acc=59.40 [H=85, D=35, S=13, I=6, N=133]
===================================================================

NB to run this demo under Windows you must have perl installed and you
need to invoke perl explicitly. See http://www.perl.org/ to download
the perl distribution. The script runDemo.pl should be used in place
of runDemo i.e. to run the test above type

 > perl runDemo.pl configs\monPlainM1S1.dcf

