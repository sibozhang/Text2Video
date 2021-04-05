/* ----------------------------------------------------------- */
/*                                                             */
/*                          ___                                */
/*                       |_| | |_/   SPEECH                    */
/*                       | | | | \   RECOGNITION               */
/*                       =========   SOFTWARE                  */ 
/*                                                             */
/*                                                             */
/* ----------------------------------------------------------- */
/* developed at:                                               */
/*                                                             */
/*      Speech Vision and Robotics group                       */
/*      Cambridge University Engineering Department            */
/*      http://svr-www.eng.cam.ac.uk/                          */
/*                                                             */
/*      Entropic Cambridge Research Laboratory                 */
/*      (now part of Microsoft)                                */
/*                                                             */
/* ----------------------------------------------------------- */
/*         Copyright: Microsoft Corporation                    */
/*          1995-2000 Redmond, Washington USA                  */
/*                    http://www.microsoft.com                 */
/*                                                             */
/*              2001  Cambridge University                     */
/*                    Engineering Department                   */
/*                                                             */
/*   Use of this software is governed by a License Agreement   */
/*    ** See the file License for the Conditions of Use  **    */
/*    **     This banner notice must not be removed      **    */
/*                                                             */
/* ----------------------------------------------------------- */
/*         File: HWave.c:   Speech Wave File Input/Output      */
/* ----------------------------------------------------------- */

char *hwave_version = "!HVER!HWave:   3.4.1 [CUED 12/03/09]";
char *hwave_vc_id = "$Id: HWave.c,v 1.1.1.1 2006/10/11 09:54:59 jal58 Exp $";

#include "HShell.h"
#include "HMem.h"
#include "HMath.h"
#include "HWave.h"
#include "esignal.h"
#include "HAudio.h"
#include "HParm.h"

#define MAX_INT 536870911
#define WAVEFORM 0
/* Added for Microsoft WAVE format */
#ifndef WAVE_FORMAT_PCM
#define WAVE_FORMAT_PCM                 (0x0001)
#endif
#define WAVE_FORMAT_ALAW                (0x0006)
#define WAVE_FORMAT_MULAW               (0x0007)
/*
This module provides an interface to speech wave files in a variety of
formats.  It is assumed that all files consist of a (possibly null)
header followed by a sequence of 2 byte speech samples.

The type FileFormat lists the currently supported input formats 
which are:

   NOHEAD      - no head at all
   ALIEN       - alien ie unknown
   HTK         - the preferred HMM Toolkit format
   TIMIT       - the TIMIT CD-ROM database
   NIST        - the NIST Resource Management database
   SCRIBE      - the UK SCRIBE CD-ROM database
   AIFF        - Apple audio interchange format
   SDES1       - Sound Designer I format
   SUNAU8      - Sun 8 bit MU law .au format
   OGI         - Oregon Institute format (similar to TIMIT)
   WAV         - Microsoft WAV format
   ESPS        - Entropic format
   ESIG        - Entropic format

If the format is ALIEN, the configuration parameter HEADERSIZE
must be set to specify the size of the header in bytes.
The header of an alien file is simply skipped.  The number of 
samples is computed from the file size or if input is via a pipe
then the number of samples must be specified by the configuration 
parameter NSAMPLES.

The NOHEAD format is treated the same as ALIEN except that a header
size of zero is assumed. 

When a data file is opened with OpenWaveInput, the entire contents of the 
file are read into memory, byte swapped if necessary, and the file
itself is then closed.   Byte swapping may be forced by setting
the configuration parameter BYTEORDER to VAX to indicate that the source
data is in VAX order or any other non-null string to indicate non-VAX
order.

In systems which support it, input can be read from a pipe instead
of a file by setting the environment variable HWAVEFILTER - see the
definition of FOpen in HShell for details.
*/


/* ------------------------ Trace Flags --------------------- */

static int trace = 0;

#define T_OPEN    0002     /* Report Wave Open Operations */

/* --------------------- Global Variables ------------------- */

static Boolean natReadOrder = FALSE;    /* Preserve natural read byte order*/
static Boolean natWriteOrder = FALSE;   /* Preserve natural write byte order*/
extern Boolean vaxOrder;                /* True if byteswapping needed to
                                           preserve SUNSO */

/* --------------------- Abstract Wave Type --------------------- */

typedef struct _Wave{   /* Internal wave file representation */
   MemHeap *mem;        /* memory heap for this wave rec */
   FileFormat fmt;      /* Format of associated source file */
   Boolean isPipe;      /* Source is a pipe */
   HTime sampPeriod;    /* Sample period in 100ns units */
   int  hdrSize;        /* Header size in bytes */
   long nSamples;       /* No of samples in data */
   long nAvail;         /* Num samples allocated for data */
   short *data;         /* Actual data (always short once loaded) */
   int frSize;          /* Num samples per frame */
   int frRate;          /* Frame rate */
   int frIdx;           /* Start of next frame */
}WaveRec;

static ConfParam *cParm[MAXGLOBS];       /* config parameters */
static int numParm = 0;

/* ---------------------- Initialisation ------------------------ */

/* EXPORT->InitWave: Initialise module */
void InitWave(void)
{
   int i;
   Boolean b;

   Register(hwave_version,hwave_vc_id);
   numParm = GetConfig("HWAVE", TRUE, cParm, MAXGLOBS);
   if (numParm>0){
      if (GetConfInt(cParm,numParm,"TRACE",&i)) trace = i;
      if (GetConfBool(cParm,numParm,"NATURALREADORDER",&b)) natReadOrder = b;
      if (GetConfBool(cParm,numParm,"NATURALWRITEORDER",&b)) natWriteOrder = b;
   }
}

/* --------------- Wave Record Memory Management ---------------- */

/* CreateWave: create a wave object with given parameters */
static Wave CreateWave(MemHeap *x, FileFormat fmt)
{
   Wave w;
   char buf[32];
   
   if (x->type != MSTAK) 
      HError(6216,"CreateWave: memory must be an MSTAK");
   w = (Wave)New(x,sizeof(WaveRec));
   w->mem = x;
   w->data = NULL; w->fmt = fmt;
   w->frSize = w->frRate = 0;
   w->nSamples = w->frIdx = 0;
   if (w->fmt == UNDEFF){
      if (GetConfStr(cParm,numParm,"SOURCEFORMAT",buf))
         w->fmt = Str2Format(buf);
      else
         w->fmt = HTK;
   }
   return w;
}

/* ShowWaveInfo: print info for given wave - for tracing */
static void ShowWaveInfo(Wave w)
{
   printf(" src format: %s\n",Format2Str(w->fmt));
   printf(" sampPeriod: %f\n",w->sampPeriod);
   printf(" hdrSize   : %d\n",w->hdrSize);
   printf(" nSamples  : %ld\n",w->nSamples);
   printf(" nAvail    : %ld\n",w->nAvail);
   printf(" frSize    : %d\n",w->frSize);
   printf(" frRate    : %d\n",w->frRate);
   printf(" frIdx     : %d\n",w->frIdx);
}

/* --------------- Detect and Change Byte Ordering -------------- */

typedef enum _SrcOrder{
   VAXSO,      /* little-endian ie low byte first */
   SUNSO,      /* big-endian ie hi byte first */
   UNKNOWNSO   /* unknown source byte order */
}SrcOrder;

   
/* MustSwap: true if reqd format must be byte swapped on this machine */
static Boolean MustSwap(SrcOrder so)
{
   char bos[MAXSTRLEN];
   
   if (GetConfStr(cParm,numParm,"BYTEORDER",bos)) { /* force required order */
      return (strcmp(bos,"VAX") == 0) ? !vaxOrder : vaxOrder;
   } else
      switch(so) {
      case VAXSO:     return !vaxOrder;
      case SUNSO:     return vaxOrder;
      case UNKNOWNSO: return FALSE;
      }
   return FALSE;
}

/* ByteSwap: byte swap the given waveform */ 
void ByteSwap(Wave w)
{
   short *p;
   long i;  

   p = w->data;
   for (i=1; i<=w->nSamples; i++)
      SwapShort(p++); 
}

/* -------------------  File Format Handling ------------------------- */

static char *fmtmap[] = {
   "NOHEAD", "HAUDIO", "HTK", "TIMIT", "NIST", 
   "SCRIBE", "AIFF", "SDES1", "SUNAU8", "OGI", 
   "ESPS", "ESIG", "WAV", "UNUSED", "ALIEN", "UNDEFF"
};

/* EXPORT-> Format2Str: convert given file format to string */
char *Format2Str(FileFormat format)
{
   if (format<NOHEAD || format > UNDEFF)
      HError(6270,"Format2Str: bad format [%d]",format);
   return fmtmap[format];
}

/* EXPORT->Str2Format: Convert string representation to enum FileFormat */
FileFormat Str2Format(char *fmt)
{
   FileFormat i = (FileFormat) -1;
   char *s;
 
   do {
      i = (FileFormat) (i+1);
      s=fmtmap[i];
      if (strcmp(fmt,s) == 0) break;
   } while (strcmp("ALIEN",s)!=0);
   if (strcmp("WAVE",fmt)==0) i=WAV;
   if (strcmp("ESIGNAL",fmt)==0) i=ESIG;
   return i;
}

/* ------------ General purpose file input routines ------------- */

typedef enum { DoCVT    = 1,     /* input conversion needed */
               DoBSWAP  = 2,     /* byte swap needed */
               DoSPACK  = 4,     /* SHORT PACK decompression needed */
               DoSHORT  = 8,     /* SHORTEN decompression needed */
               DoMULAW  = 16,    /* 8 bit Mu-Law expansion needed */
               DoALAW  = 32,    /* 8 bit A-Law expansion needed */
               Do8_16  = 64,    /* 8 bit PCM expansion needed */
               DoSTEREO = 128     /* Convert stereo to mono*/ 
}InputAction;

/* NIST mu-to-linear conversion, also valid for wav (I think?)*/
static short int NISTmutab[256] = {
   -32124, -31100, -30076, -29052,
   -28028, -27004, -25980, -24956, -23932, -22908, -21884, -20860,
   -19836, -18812, -17788, -16764, -15996, -15484, -14972, -14460,
   -13948, -13436, -12924, -12412, -11900, -11388, -10876, -10364,
   -9852, -9340, -8828, -8316, -7932, -7676, -7420, -7164, -6908,
   -6652, -6396, -6140, -5884, -5628, -5372, -5116, -4860, -4604,
   -4348, -4092, -3900, -3772, -3644, -3516, -3388, -3260, -3132,
   -3004, -2876, -2748, -2620, -2492, -2364, -2236, -2108, -1980,
   -1884, -1820, -1756, -1692, -1628, -1564, -1500, -1436, -1372,
   -1308, -1244, -1180, -1116, -1052, -988, -924, -876, -844, -812,
   -780, -748, -716, -684, -652, -620, -588, -556, -524, -492, -460,
   -428, -396, -372, -356, -340, -324, -308, -292, -276, -260, -244,
   -228, -212, -196, -180, -164, -148, -132, -120, -112, -104, -96,
   -88, -80, -72, -64, -56, -48, -40, -32, -24, -16, -8, 0, 32124,
   31100, 30076, 29052, 28028, 27004, 25980, 24956, 23932, 22908,
   21884, 20860, 19836, 18812, 17788, 16764, 15996, 15484, 14972,
   14460, 13948, 13436, 12924, 12412, 11900, 11388, 10876, 10364,
   9852, 9340, 8828, 8316, 7932, 7676, 7420, 7164, 6908, 6652, 6396,
   6140, 5884, 5628, 5372, 5116, 4860, 4604, 4348, 4092, 3900, 3772,
   3644, 3516, 3388, 3260, 3132, 3004, 2876, 2748, 2620, 2492, 2364,
   2236, 2108, 1980, 1884, 1820, 1756, 1692, 1628, 1564, 1500, 1436,
   1372, 1308, 1244, 1180, 1116, 1052, 988, 924, 876, 844, 812,
   780, 748, 716, 684, 652, 620, 588, 556, 524, 492, 460, 428, 396,
   372, 356, 340, 324, 308, 292, 276, 260, 244, 228, 212, 196, 180,
   164, 148, 132, 120, 112, 104, 96, 88, 80, 72, 64, 56, 48, 40,
   32, 24, 16, 8, 0};

static short a2l[]={
   -5504, -5248, -6016, -5760, -4480, -4224, -4992, -4736,
   -7552, -7296, -8064, -7808, -6528, -6272, -7040, -6784,
   -2752, -2624, -3008, -2880, -2240, -2112, -2496, -2368,
   -3776, -3648, -4032, -3904, -3264, -3136, -3520, -3392,
   -22016,-20992,-24064,-23040,-17920,-16896,-19968,-18944,
   -30208,-29184,-32256,-31232,-26112,-25088,-28160,-27136,
   -11008,-10496,-12032,-11520, -8960, -8448, -9984, -9472,
   -15104,-14592,-16128,-15616,-13056,-12544,-14080,-13568,
   -344,  -328,  -376,  -360,  -280,  -264,  -312,  -296,
   -472,  -456,  -504,  -488,  -408,  -392,  -440,  -424,
   -88,   -72,   -120,  -104,  -24,   -8,    -56,   -40,
   -216,  -200,  -248,  -232,  -152,  -136,  -184,  -168,
   -1376, -1312, -1504, -1440, -1120, -1056, -1248, -1184,
   -1888, -1824, -2016, -1952, -1632, -1568, -1760, -1696,
   -688,  -656,  -752,  -720,  -560,  -528,  -624,  -592,
   -944,  -912,  -1008, -976,  -816,  -784,  -880,  -848,
   5504,  5248,  6016,  5760,  4480,  4224,  4992,  4736,
   7552,  7296,  8064,  7808,  6528,  6272,  7040,  6784,
   2752,  2624,  3008,  2880,  2240,  2112,  2496,  2368,
   3776,  3648,  4032,  3904,  3264,  3136,  3520,  3392,
   22016, 20992, 24064, 23040, 17920, 16896, 19968, 18944,
   30208, 29184, 32256, 31232, 26112, 25088, 28160, 27136,
   11008, 10496, 12032, 11520, 8960,  8448,  9984,  9472,
   15104, 14592, 16128, 15616, 13056, 12544, 14080, 13568,
   344,   328,   376,   360,   280,   264,   312,   296,
   472,   456,   504,   488,   408,   392,   440,   424,
   88,    72,    120,   104,   24,    8,     56,    40,
   216,   200,   248,   232,   152,   136,   184,   168,
   1376,  1312,  1504,  1440,  1120,  1056,  1248,  1184,
   1888,  1824,  2016,  1952,  1632,  1568,  1760,  1696,
   688,   656,   752,   720,   560,   528,   624,   592,
   944,   912,  1008,   976,   816,   784,   880,   848 };


/* NumberBytes: Returns number of bytes in file excluding header, if input
   is from a pipe then INT_MAX is returned */
static long NumberBytes(FILE *f, int hSize, Boolean isPipe)
{
   long fileLen,pos;

   if (isPipe) return INT_MAX;
   if ((pos = ftell(f)) == -1L)
      HError(6320,"NumberBytes: Cannot read data file current position");
   if (fseek(f,0,SEEK_END))
      HError(6320,"NumberBytes: Cannot seek to end of data file");
   if ((fileLen = ftell(f)) == -1L)
      HError(6320,"NumberBytes: Cannot read data file end position");
   if (fseek(f,pos,SEEK_SET))
      HError(6320,"NumberBytes: Cannot return to current position");
   return fileLen - hSize;
}

/* FileBytes: Returns number of bytes in file excluding header, if input
   is from a pipe then INT_MAX is returned */
static long FileBytes(FILE *f, Wave w)
{
   long fileLen,pos;

   if (w->isPipe) return INT_MAX;
   if ((pos = ftell(f)) == -1L)
      HError(6220,"FileBytes: Cannot read data file current position");
   if (fseek(f,0,SEEK_END))
      HError(6220,"FileBytes: Cannot seek to end of data file");
   if ((fileLen = ftell(f)) == -1L)
      HError(6220,"FileBytes: Cannot read data file end position");
   if (fseek(f,pos,SEEK_SET))
      HError(6220,"FileBytes: Cannot return to current position");
   return fileLen - w->hdrSize;
}

/* ConsumeHeader: read rest of the header.  This call avoids the
                  use of fseek to allow input from a pipe */
static void ConsumeHeader(FILE *f, int bytesRead, int headSize)
{
   int i;

   for (i=bytesRead; i<headSize; i++)
      if (fgetc(f) == EOF)
         HError(6250,"ConsumeHeader: reading %d of header size %d",i,headSize);
}

/* LoadData: Loads samples from stream into Wave Record */
static ReturnStatus LoadData(FILE *f, Wave w, long fBytes)
{
   long bufSize;
   long nRead;

   if ( fBytes==INT_MAX || fBytes<0 ){
      HRError(6221,"LoadData: File size appears to be infinite");
      return(FAIL);
   }
   bufSize = w->nSamples*2;
   if (bufSize < fBytes) bufSize = fBytes;
   w->nAvail = bufSize / 2;
   w->data = (short *)New(w->mem,bufSize);
   if ((nRead=fread(w->data, 1, fBytes, f)) != fBytes  && !w->isPipe) {
      HRError(6253,"LoadData: Cannot read data into memory");
      return(FAIL);
   }
   if (w->isPipe && nRead!=fBytes) {
      w->nSamples = nRead/2;
   }
   return(SUCCESS);
}


/* --------------- ALIEN/NOHEAD Format Interface Routines -------------- */

/* MakeHeaderInfo: set up the header info in supplied Wave 
   with nSamples computed from file size or from confparm (NSAMPLES) 
   if input is a pipe */
static ReturnStatus MakeHeaderInfo(FILE *f, int hdrSize, Wave w)
{
   int nsamp;
   
   w->hdrSize = hdrSize;
   w->sampPeriod = 0.0;
   if (!w->isPipe)
      w->nSamples = FileBytes(f,w) / 2;
   else{
      if (!GetConfInt(cParm,numParm,"NSAMPLES",&nsamp)){
         HRError(6230,"MakeHeaderInfo: NSAMPLES not set in config");
         return FAIL;
      }
      w->nSamples = nsamp;
   }
   return(SUCCESS);
}

/* GetALIENHeaderInfo: get header of alien file.  In addition to info
   required by MakeHeaderInfo, header size must be given via conf-
   iguration parameter (HEADERSIZE) */
static long GetALIENHeaderInfo(FILE *f, Wave w, InputAction *ia)
{
   int hdSize;

   if (!GetConfInt(cParm,numParm,"HEADERSIZE",&hdSize)){
      HRError(6230,"GetALIENHeaderInfo: HEADERSIZE not set in config");
      return -1;
   }
   if(MakeHeaderInfo(f, hdSize, w)<SUCCESS){
      return -1;
   }
   if (MustSwap(UNKNOWNSO))
      *ia = (InputAction) (*ia | DoBSWAP);
   ConsumeHeader(f,0,hdSize);
   return w->nSamples*2;
}

/* GetNOHEADHeaderInfo: create header for headerless file */
static long GetNOHEADHeaderInfo(FILE *f, Wave w, InputAction *ia)
{
   if(MakeHeaderInfo(f, 0, w)<SUCCESS){
      return -1;
   }
   if (MustSwap(UNKNOWNSO))
      *ia = (InputAction) (*ia | DoBSWAP);
   return w->nSamples*2;
}

/* ---------------------- TIMIT Format Interface Routines --------------------- */

typedef struct {              /* TIMIT File Header */
   short hdrSize;
   short version;
   short numChannels;
   short sampRate;
   int32  nSamples;
} TIMIThdr;

/* GetTIMITHeaderInfo: get fixed size binary 12 byte TIMIT header */
static long GetTIMITHeaderInfo(FILE *f, Wave w, InputAction *ia)
{
   TIMIThdr hdr;
   int n = sizeof hdr;
   Boolean bSwap;
   
   if ((bSwap = MustSwap(VAXSO)))        /* TIMIT is VAX ordered */
      *ia = (InputAction) (*ia | DoBSWAP);
   if (fread(&hdr, 1, n, f) != n){
      HRError(6250,"GetTIMITHeaderInfo: Cannot read TIMIT format header");
      return -1;
   }
   if (bSwap) {
      SwapShort(&hdr.hdrSize);
      SwapShort(&hdr.version);
      SwapShort(&hdr.numChannels);
      SwapShort(&hdr.sampRate);
      SwapInt32(&hdr.nSamples);
   }
   if (hdr.nSamples < 0 || hdr.sampRate < 0 || hdr.numChannels < 0 ||
       hdr.numChannels > 8 ){
      HRError(6251,"GetTIMITHeaderInfo: Bad Numbers in TIMIT format header");
      return -1;
   }
   w->nSamples = hdr.nSamples;
   w->sampPeriod = hdr.sampRate*2.5;
   w->hdrSize = n;
   return w->nSamples*2;
}

/* ---------------------- OGI Format Interface Routines --------------------- */

typedef struct {              /* OGI File Header */
   short hdrSize;
   short version;
   short numChannels;
   short sampRate;
   int32  nSamples;
   int32  lendian;
} OGIhdr;
           
/* GetOGIHeaderInfo: get fixed size binary 16 byte OGI header */
static long GetOGIHeaderInfo(FILE *f, Wave w, InputAction *ia)
{
   OGIhdr hdr;
   int n = sizeof hdr;
   Boolean bSwap;
   
   if((bSwap = MustSwap(SUNSO)))        /* OGI is SUN ordered */
      *ia = (InputAction) (*ia | DoBSWAP);
   if (fread(&hdr, 1, n, f) != n){
      HRError(6250,"GetOGIHeaderInfo: Cannot read OGI format header");
      return -1;
   }
   if (bSwap) {
      SwapShort(&hdr.hdrSize);
      SwapShort(&hdr.version);
      SwapShort(&hdr.numChannels);
      SwapShort(&hdr.sampRate);
      SwapInt32(&hdr.nSamples);
      SwapInt32(&hdr.lendian);
   }
   if (hdr.nSamples < 0 || hdr.sampRate < 0 || hdr.numChannels < 0 ||
       hdr.numChannels > 8 ){
      HRError(6251,"GetOGIHeaderInfo: Bad Numbers in OGI format header");
      return -1;
   }
   w->nSamples = hdr.nSamples;
   w->sampPeriod = hdr.sampRate*2.5;
   w->hdrSize = n;
   return w->nSamples*2;
}

/* ---------------------- NIST Format Interface Routines --------------------- */

static int cNIST;    /* current input char */
static int cCount;   /* num bytes read */

enum _CompressType{
   SHORTPACK,   /* MIT shortpack-v0 */
   SHORTEN,     /* CUED Shorten */
   IMULAW       /* Interleaved 8 bit u-law */
};
typedef enum _CompressType CompressType;

/* GetNISTToken: get next token delimited by white space from f */
static char * GetNISTToken(FILE *f,char *buf)
{
   int i=0;
   
   while (isspace(cNIST)) {
      cNIST=fgetc(f); ++cCount;
   }
   do {
      if (cNIST == EOF)
         HError(6250,"GetNISTToken: Unexpected end of file");
      buf[i++] = cNIST; cNIST=fgetc(f); ++cCount;
   } while(!isspace(cNIST) && i<99);
   buf[i] = '\0';
   return buf;
}

/* NISTSkipLine: skip to next input line of f */
static void NISTSkipLine(FILE *f)
{
   while (cNIST != '\012'){   /* new line is line feed on NIST ROM */
      cNIST=fgetc(f); ++cCount;
      if (cNIST == EOF)
         HError(6250,"NISTSkipLine: Unexpected end of file");
   }  
   cNIST=fgetc(f); ++cCount;
}

/* GetNISTIVal: get int val from f (indicated by -i) */
static int GetNISTIVal(FILE *f)
{
   char buf[100];
   
   if (strcmp(GetNISTToken(f,buf),"-i") != 0)
      HError(6251,"GetNISTIVal: NIST type indicator -i expected");
   return atoi(GetNISTToken(f,buf));
}

/* GetNISTSVal: get string of lenth n into s (indicated by -sn) */
static void GetNISTSVal(FILE *f, char *s)
{
   char buf[100];

   GetNISTToken(f,buf);
   if (buf[0] != '-' || buf[1] != 's')
      HError(6251,"GetNISTSVal: NIST type indicator -s expected");
   GetNISTToken(f,s);
   if (atoi(buf+2) != strlen(s))
      HError(6251,"GetNISTSVal: bad string length");
}

/* GetNISTHeaderInfo: get the NIST Header info */
static long GetNISTHeaderInfo(FILE *f, Wave w, InputAction *ia)
{
   char token[100],*lab,byteFormat[100],sampCoding[100],buf[100];
   Boolean interleaved = FALSE;
   long nS,sR,sS, cC;
   long dataBytes;
   
   cNIST = ' '; cCount = 0;
   nS=sR=sS=-1; 
   byteFormat[0]='\0'; sampCoding[0]='\0';
   lab = GetNISTToken(f,token);           /* Check NIST label */
   if (strlen(lab)>4) *(lab+4) = '\0';
   if (strcmp(lab,"NIST") !=0){
      HRError(6251,"GetNISTHeaderInfo: NIST header label missing");
      return -1;
   }
   NISTSkipLine(f);
   w->hdrSize = atoi(GetNISTToken(f,token)); /* header #bytes */
   NISTSkipLine(f);
   while (strcmp(GetNISTToken(f,token),"end_head")!=0){
      if (strcmp(token,"sample_count") == 0)    /* objects */
         nS = GetNISTIVal(f);
      else if (strcmp(token,"sample_rate") == 0)
         sR = GetNISTIVal(f);
      else if (strcmp(token,"sample_n_bytes") == 0)
         sS = GetNISTIVal(f);
      else if (strcmp(token,"sample_byte_format") == 0)
         GetNISTSVal(f,byteFormat);
      else if (strcmp(token,"sample_coding") == 0)
         GetNISTSVal(f,sampCoding);
      else if (strcmp(token,"channels_interleaved") == 0){
         GetNISTSVal(f,buf);
         if (strcmp(buf,"TRUE") == 0)
            interleaved = TRUE;
      }
      else if (strcmp (token, "channel_count") == 0) {
         cC = GetNISTIVal(f);
         if (cC==2)
            interleaved = TRUE;
         else if (cC!=1)
            HError(6251,"GetNISTHeaderInfo: channel count = %d in NIST header",cC);
      }
      NISTSkipLine(f);
   }
   if (sS < 1 || sS > 2){
      HRError(6251,"GetNISTHeaderInfo: Sample size = %d in NIST header",sS);
      return -1;
   }
   if (nS == -1){
      HRError(6252,"GetNISTHeaderInfo: Num samples undefined in NIST header");
      return -1;
   }
   if (sR == -1){
      HRError(6252,"GetNISTHeaderInfo: Sample Rate undefined in NIST header");
      return -1;
   }
   if (strcmp(byteFormat, "") == 0) {
      HRError(-6252,"GetNISTHeaderInfo: Byte Format undefined in NIST header");
   }
   if (strcmp(sampCoding,"ulaw")==0)
      strcpy(sampCoding,"mu-law");
   if (interleaved)
      strcat (sampCoding, "-interleaved");

   w->nSamples = nS;
   w->sampPeriod = 1.0E7 / (float)sR;
   dataBytes = w->nSamples * 2;
   /* Fix for bug in original WSJ0 shortpack headers */
   if (strcmp(byteFormat,"shortpack-v0") == 0) {
      strcpy(sampCoding,byteFormat);
      strcpy(byteFormat,"01");
   }
   /* standard 16 bit linear formats */
   if (strlen(sampCoding)==0 || strcmp(sampCoding,"pcm")==0){
      if (strcmp(byteFormat,"01") == 0){
         if(MustSwap(VAXSO))
            *ia = (InputAction) (*ia | DoBSWAP);
      }else if (strcmp(byteFormat,"10") == 0)
         if(MustSwap(SUNSO))
            *ia = (InputAction) (*ia | DoBSWAP);
   }
   /* ShortPack compression format */
   else if (DoMatch(sampCoding,"*shortpack*")) {
      *ia = (InputAction) (*ia | DoSPACK);
      *ia = (InputAction) (*ia | DoCVT);
      dataBytes = FileBytes(f, w);
   }
   /* Shorten compression format */
   else if (DoMatch(sampCoding,"*embedded-shorten*")) {
      *ia = (InputAction) (*ia | DoSHORT);
      *ia = (InputAction) (*ia | DoCVT);
      dataBytes = FileBytes(f, w);
   }
   /* Interleaved Mu-Law */
   else if (DoMatch(sampCoding,"*mu-law-interleaved*")) {
      /*      w->nSamples /= 2; */ 
      *ia = (InputAction) (*ia | DoMULAW);
      *ia = (InputAction) (*ia | DoCVT);
      dataBytes = FileBytes(f, w);
   }
   else{
      HRError(6251,"GetNISTHeaderInfo: unknown byte format in NIST header");
      return -1;
   }
   ConsumeHeader(f,cCount,w->hdrSize);
   return dataBytes;
}

/* for use in bit-twiddling operations in GetShortPackBlock */
static const unsigned char bitValue[8] = {1,2,4,8,16,32,64,128};

/* GetShortPackBlock: decode the block of data starting at *inData and
   store at *outData return number of samples in block and update
   *inData and *outData. Data format defined by Mike Phillips at MIT */
static int GetShortPackBlock(char **inData, short **outData)
{
   char *in;   /* dereferenced forms of the arguments */
   short *out;
   unsigned char nSamp, nBits;
   unsigned char buf = '\0';
   int i,k;
   Boolean negative;
   int charBits=0;  
   int numChar=0;
   
   in = *inData; out = *outData;   
   nSamp = *(in++);  /* number of samples in block */
   nBits = *(in++);  /* number of bits / sample excluding sign bit */
   if (nBits > 15)
      HError(6254,"GetShortPackBlock: Incorrect number of bits/sample");
   for (i=0; i<nSamp; i++,out++) {
      *out = 0;
      if (charBits == 0) {
         buf = *(in++);
         numChar++;
      }
      negative = buf & bitValue[7-charBits];
      charBits = (charBits+1)%8;
      k = nBits;
      while (k > 0) {
         if (charBits == 0)  {        
            buf = *(in++);
            numChar++;
         }
         if (charBits == 0 && k >= 8) {     /* do the whole byte at once */
            *out |= buf << (k-8);
            k -= 8;
         }  
         else if (charBits == 0 && k >= 4) { /* most significant nibble */
            *out |= ((buf & '\360') >> 4) << (k-4);
            k -= 4; charBits = 4;
         }
         else if (charBits == 4 && k >= 4) { /* least significant nibble */
            *out |= (buf & '\017') << (k-4);
            k -= 4; charBits = 0;           
         }
         else {  /* do a single bit */
            *out  |= ((buf & bitValue[7-charBits]) >> (7-charBits)) << (k-1);
            k--; charBits = (charBits+1)%8;
         }
      }
      if (negative) /* shortpack data is signed binary */
         *out=(*out==0)?-32768:-(*out); 
   }
   if ((numChar%2) != 0) in++;    /* MIT code uses shorts to read/write */
   *outData = out; *inData = in;
   return (int)nSamp;
}

/* DecompressShortPack: decode short packed data into new data array */
void DecompressShortPack(Wave w)
{
   short* decomp; /* the decompressed data */
   long outSampCount=0;
   char *inData;
   short *outData;
    
   decomp = (short *)New(w->mem,w->nSamples*2);
   inData = (char *)w->data;
   outData = decomp;
   while (outSampCount < w->nSamples) 
      outSampCount += GetShortPackBlock(&inData, &outData);
   if (outSampCount != w->nSamples)
      HError(6254,"DecompressShortPack: Incorrect number of decompressed samples");
   w->data = decomp;
}

/* DecodeIMuLaw: convert interleaved 8bit MU law to 16 bit linear */
static void DecodeIMuLaw(Wave w)
{
   unsigned char *src;
   short *tgt;
   int i,sample,lchan,rchan;
   char smode[MAXSTRLEN];
   enum {imLeft,imRight,imSum} mode;

   /* Set Conversion Mode - left channel, right channel or sum */
   mode = imSum;
   if (GetConfStr(cParm,numParm,"STEREOMODE",smode)){
      if (strcmp(smode,"LEFT") == 0)  mode = imLeft; else
         if (strcmp(smode,"RIGHT") == 0) mode = imRight;
   }

   /* Convert data */
   src = (unsigned char *) w->data;
   tgt = w->data;  
   sample = 0;
   for (i=1; i<=w->nSamples; i++,tgt++) {
      lchan = NISTmutab[*src++];
      rchan = NISTmutab[*src++];
      switch(mode){
      case imLeft:  sample = lchan; break;
      case imRight: sample = rchan; break;
      case imSum:   sample = (lchan+rchan)/2; break;
      }
      *tgt = sample;
   }
}

/* ConvertNISTData: decompress the Wave */ 
ReturnStatus ConvertNISTData(Wave w, InputAction ia)
{
   if (ia&DoSPACK)  
      DecompressShortPack(w); 
   else if (ia&DoSHORT){  
      HRError(6201,"ConvertNISTData: NIST Shorten Compression not implemented"); 
      return(FAIL);
   }
   else if (ia&DoMULAW)  
      DecodeIMuLaw(w);
   return(SUCCESS);
}

/* ---------------------- SDES1 Format Interface Routines --------------------- */

typedef struct {     /* skeleton SDES1 format header */
   short hdrSize;    /* should be 1336 */
   char fill1[182];
   int32 fileSize;    /* num samples * 2 */
   char fill[832];
   int32 sampRate;    /* sample rate in Hertz */
   int32 sampPeriod;  /* sample period in usecs */
   short sampSize;   /* sample size in bits (16) */
} SDes1Header;

/* GetSDES1HeaderInfo: get header for Sound Designer 1 format file */
static long GetSDES1HeaderInfo(FILE *f, Wave w, InputAction *ia)
{
   SDes1Header hdr;
   int n = sizeof hdr;
   Boolean bSwap;
   
   if ((bSwap = MustSwap(UNKNOWNSO)))       /* User might know byte order */
      *ia = (InputAction) (*ia | DoBSWAP);
   if (fread(&hdr, 1, n, f) != n){
      HRError(6250,"GetSDES1HeaderInfo: Cannot read SDES1 format header");
      return -1;
   }
   if (hdr.hdrSize != 1336){
      HRError(6251,"GetSDES1HeaderInfo: Bad HeaderSize in SDES1 format header");
      return -1;
   }
   if (bSwap) {
      SwapShort(&hdr.hdrSize);
      SwapInt32(&hdr.fileSize); 
      SwapInt32(&hdr.sampRate); 
      SwapInt32(&hdr.sampPeriod);
      SwapShort(&hdr.sampSize);
   }
   w->nSamples = hdr.fileSize / 2;
   w->sampPeriod = 1.0E7 / (float)hdr.sampRate;
   w->hdrSize = 1336;
   return w->nSamples*2;
}

/* ---------------------- SCRIBE Format Interface Routines --------------------- */

/* GetSCRIBEHeaderInfo: create a header for a SCRIBE waveform file */
static long GetSCRIBEHeaderInfo(FILE *f, Wave w, InputAction *ia)
{
   int nsamp;
   
   if (MustSwap(UNKNOWNSO))       /* User might know byte order */
      *ia = (InputAction) (*ia | DoBSWAP);
   w->hdrSize    = 0;
   w->sampPeriod = 0.0;           /* User must specify this */
   if (!w->isPipe)
      w->nSamples = FileBytes(f,w) / 2;
   else{
      if (!GetConfInt(cParm,numParm,"NSAMPLES",&nsamp)){
         HRError(6230,"GetSCRIBEHeaderInfo: NSAMPLES not set in config");
         return -1;
      }
      w->nSamples = nsamp;
   }
   return w->nSamples*2;
}

/* ---------------------- SUNAU8 Format Interface Routines --------------------- */

/* Note this is a special case of the NeXt SNDSoundStruct header */

typedef struct {     /* SUNAU8 format header */
   int32 magic;          /* magic number 0x2e736e64 */
   int32 dataLocation;   /* offset to start of data */
   int32 dataSize;       /* number of bytes of data */
   int32 dataFormat;     /* format code */
   int32 sampRate;       /* sample rate code */
   int32 numChan;        /* number of channels */
   char info[4];  
} SunAU8Header;

/* GetSUNAU8HeaderInfo: get header for SUN 8bit mulaw sound file */
static long GetSUNAU8HeaderInfo(FILE *f, Wave w, InputAction *ia)
{
   int n;
   Boolean bSwap;  
   SunAU8Header hdr;
   
   n = sizeof(SunAU8Header);
   if (fread(&hdr,1,n,f) != n){
      HRError(6250,"GetSunAU8HeaderInfo: Cannot read SunAU8 format header");
      return -1;
   }
   if ((bSwap = MustSwap(UNKNOWNSO))) {       /* User might know byte order */
      SwapInt32(&hdr.magic);
      SwapInt32(&hdr.dataLocation); 
      SwapInt32(&hdr.dataSize); 
      SwapInt32(&hdr.dataFormat);
      SwapInt32(&hdr.sampRate);
      SwapInt32(&hdr.numChan);
   }
   if (hdr.magic !=  0x2e736e64 || hdr.dataFormat != 1 /* 8 bit mulaw */
       || hdr.numChan != 1 /* mono */ ){
      HRError(6251,"GetSUNAU8HeaderInfo: Bad Numbers in SUNAU8 format header");
      return -1;
   }
   w->nSamples   = hdr.dataSize;
   w->sampPeriod = 1248;      /* 8012.821 Hz codec input rate */
   w->hdrSize = hdr.dataLocation;
   ConsumeHeader(f,n,w->hdrSize);
   *ia = (InputAction) (*ia | DoCVT);   /* convert 8bit mulaw to 16bit linear */
   return w->nSamples;        /* data size = 2 * file size */
}

/* ConvertSUNAU8Data: convert 8bit MU law to 16 bit linear */
static ReturnStatus ConvertSUNAU8Data(Wave w)
{
   unsigned char *src, ulawbyte;
   short *tgt;
   static int exp_lut[8]={0,132,396,924,1980,4092,8316,16764};
   int i, sign, exponent, mantissa, sample;
   
   /* Convert data */
   src = ((unsigned char *) w->data) + w->nSamples-1;
   tgt = ((short *)  w->data) + w->nSamples-1;  
   for (i=1; i<=w->nSamples; i++,src--,tgt--) {
      ulawbyte = ~(*src);
      sign = (ulawbyte & 0x80);
      exponent = (ulawbyte >> 4) & 0x07;
      mantissa = ulawbyte & 0x0F;
      sample = exp_lut[exponent] + (mantissa << (exponent+3));
      *tgt = (sign != 0) ? -sample : sample;
   }
   return(SUCCESS);
}

/* ---------------------- AIFF Format Interface Routines --------------------- */

typedef struct {     /* skeleton chunk record */
   int32 id;          /* must be 'FORM' */
   int32 size;        /* size of rest of chunk */
   int32 formType;    /* must be 'AIFF' */
} FormChunk;

typedef struct {
   int32 id;
   int32 size;
   short numChannels;
} CommonChunk1;

typedef struct {
   unsigned int nSamples;
   short sampSize;
   /* extended sampRate;  Apples non-standard fp format!! */         
} CommonChunk2;   

/* GetAIFFHeaderInfo: get AIFF format header */
static long GetAIFFHeaderInfo(FILE *f, Wave w, InputAction *ia)
{
   FormChunk fchunk;
   int fn = sizeof fchunk;
   long sndStart = 0;    /* start of sound chunk */
   long fPtr;
   CommonChunk1 ch1, commchunk1;
   CommonChunk2 ch2, commchunk2;   
   int cn1 = 10; /* sizeof(long) + sizeof(long) + sizeof(short); */
   int cn2 = 6;  /* sizeof(long) + sizeof(short); */   
   Boolean hasCC=FALSE, hasSC=FALSE;
   const long fcid = 0x464f524d;  /* 'FORM' */
   const long ccid = 0x434f4d4d;  /* 'COMM' */
   const long scid = 0x53534e44;  /* 'SSND' */
   
   if (w->isPipe){
      HRError(6201,"GetAIFFHeaderInfo: cannot pipe an AIFF file");
      return -1;
   }
   *ia = (InputAction) 0;
   if (MustSwap(SUNSO)){
      HRError(6201,"GetAIFFHeaderInfo: Cannot byte swap AIFF format");
      return -1;
   }
   rewind(f);
   if (fread(&fchunk, 1, fn, f) != fn){
      HRError(6250,"GetAIFFHeaderInfo: Cannot read AIFF form chunk");
      return -1;
   }
   if (fchunk.id != fcid){
      HRError(6251,"GetAIFFHeaderInfo: Not an AIFF file!");
      return -1;
   }
   fPtr = 12;
   while (!(hasCC && hasSC)) {
      if (fseek(f,fPtr,SEEK_SET) != 0){
         HRError(6220,"GetAIFFHeaderInfo: Seek error searching for AIFF chunks");
         return -1;
      }
      if ((fread(&ch1, 1, cn1, f) != cn1) || (fread(&ch2, 1, cn2, f)!= cn2)){ 
         HRError(6251,"GetAIFFHeaderInfo: Cannot read AIFF common chunk");
         return -1;
      }
      if (ch1.id == ccid){  /* common chunk found */
         hasCC=TRUE; commchunk1=ch1;commchunk2=ch2;
      }
      if (ch1.id == scid){  /* sound chunk found */
         hasSC=TRUE; sndStart=fPtr;
      }
      fPtr += ch1.size + 8;
   }
   w->nSamples   = commchunk2.nSamples;
   w->sampPeriod = 625;    /* fudge to avoid decoding Apples
                              10 byte floating point format - assume 16 kHz */
   w->hdrSize = sndStart+16;
   return w->nSamples * 2;
}

/* --------------------- WAV Format Interface Routines --------------------- */

/* GetWAVHeaderInfo: get header for Microsoft WAVE format sound file */
static long GetWAVHeaderInfo(FILE *f, Wave w, InputAction *ia)
{
   char magic[4];
   int32 len,lng,numBytes;
   char c;
   short sht,sampSize,type,chans;
   
   if (MustSwap(VAXSO))
      *ia = (InputAction) (*ia | DoBSWAP);
   
   fread(magic, 4, 1, f);
   if (strncmp("RIFF", magic, 4)){
      HRError(6251,"Input file is not in RIFF format");
      return -1;
   }
   
   fread(&lng, 4, 1, f);
   fread(magic, 4, 1, f);
   if (strncmp("WAVE", magic, 4)){
      HRError(6251,"Input file is not in WAVE format");
      return -1;
   }

   /* Look for "fmt " before end of file */
   while(1) {
      if (feof(f)){
         HRError(6251,"No data portion in WAVE file");
         return -1;
      }
      fread(magic, 4, 1, f);
      fread(&len, 4, 1, f);
      if (*ia & DoBSWAP) SwapInt32(&len);
      /* Check for data chunk */
      if (strncmp("data", magic, 4)==0) break;
      if (strncmp("fmt ", magic, 4)==0) {
         fread(&type, 2, 1, f);
         if (*ia & DoBSWAP) SwapShort(&type);
         if (type != WAVE_FORMAT_PCM && type!=WAVE_FORMAT_MULAW && type!=WAVE_FORMAT_ALAW){
            HRError(6251,"Only standard PCM, mu-law & a-law supported");
            return -1;
         }
         if(type==WAVE_FORMAT_MULAW)
            *ia = (InputAction) (*ia | (DoMULAW|DoCVT));
         else if(type==WAVE_FORMAT_ALAW)
            *ia = (InputAction) (*ia | (DoALAW|DoCVT));
         fread(&chans, 2, 1, f);                /* Number of Channels */
         if (*ia & DoBSWAP) SwapShort(&chans);
         if (chans!=1 && chans!=2){
            HRError(6251,"Neither mono nor stereo!");
            return -1;
         }
         if(chans==2)
            *ia = (InputAction) (*ia | (DoSTEREO|DoCVT));
         fread(&lng, 4, 1, f);                /* Sample Rate */
         if (*ia & DoBSWAP) SwapInt32(&lng);
         w->sampPeriod = 1.0E7 / (float)lng;
         fread(&lng, 4, 1, f);                  /* Average bytes/second */
         fread(&sht, 2, 1, f);                  /* Block align */
         fread(&sampSize, 2, 1, f);             /* Data size */
         if (*ia & DoBSWAP) SwapShort(&sampSize);
         if (sampSize != 16 && sampSize!=8){
            HRError(6251,"Only 8/16 bit audio supported");
            return -1;
         }
         if((type==WAVE_FORMAT_MULAW||type==WAVE_FORMAT_ALAW) && sampSize!=8){
            HRError(6251,"Only 8-bit mu-law/a-law supported");
            return -1;
         }
         if(type==WAVE_FORMAT_PCM && sampSize==8)
            *ia = (InputAction) (*ia | (Do8_16|DoCVT));
         len -= 16;
      }
      /* Skip chunk */
      for (; len>0; len--) fread(&c,1,1,f);
   }
   numBytes=len;
   w->nSamples = numBytes / (sampSize/8);
   /*If stereo: w->nSamples is the stereo value; changed in convertWAV*/
   return numBytes;
}

/* ConvertWAVData*/ 
ReturnStatus ConvertWAVData(Wave w, InputAction *ia)
{
   unsigned char *srcc;
   short *tgt,*srcs;
   int i,sample,lchan,rchan;
   char smode[MAXSTRLEN];
   enum {imLeft,imRight,imSum} mode;

   if(*ia&DoMULAW){
      srcc = ((unsigned char *) w->data) + w->nSamples-1;
      tgt = ((short *)  w->data) + w->nSamples-1;  
      for (i=1; i<=w->nSamples; i++,srcc--,tgt--) {
         *tgt = NISTmutab[*srcc];
      }
      *ia = (InputAction) (*ia & (~DoBSWAP));  /* Must not byte-swap now */
   }

   if(*ia&DoALAW){
      srcc = ((unsigned char *) w->data) + w->nSamples-1;
      tgt = ((short *)  w->data) + w->nSamples-1;  
      for (i=1; i<=w->nSamples; i++,srcc--,tgt--) {
         *tgt = a2l[*srcc];
      }
      *ia = (InputAction) (*ia & (~DoBSWAP));  /* Must not byte-swap now */
   }

   if(*ia&Do8_16){
      srcc = ((unsigned char *) w->data) + w->nSamples-1;
      tgt = ((short *)  w->data) + w->nSamples-1;  
      for (i=1; i<=w->nSamples; i++,srcc--,tgt--) {
         *tgt = (*srcc * 256)-32768;
      }
      *ia = (InputAction) (*ia & (~DoBSWAP));
   }

   if(*ia&DoSTEREO){
      w->nSamples/=2;/*Final mono number*/
      mode = imSum;
      if (GetConfStr(cParm,numParm,"STEREOMODE",smode)){
         if (strcmp(smode,"LEFT") == 0)  mode = imLeft; else
            if (strcmp(smode,"RIGHT") == 0) mode = imRight;
      }
      srcs = (short *) w->data;
      tgt = w->data;  
	  sample = 0;
      for (i=1; i<=w->nSamples; i++,tgt++) {
         lchan = *srcs++;
         rchan = *srcs++;
         switch(mode){
         case imLeft:  sample = lchan; break;
         case imRight: sample = rchan; break;
         case imSum:   sample = (lchan+rchan)/2; break;
         }
         *tgt = sample;
      }
   }
   return(SUCCESS);
}

/*variable to hold fieldlist of an ESIG input file */
static FieldList  ESIGFieldList;  

/* EXPORT->StoreESIGFieldList: store the field list of an ESIG input file */
void StoreESIGFieldList(HFieldList fList)
{
   ESIGFieldList = fList;
}
/* EXPORT->RetrieveESIGFieldList: store the field list of an ESIG input file */
void RetrieveESIGFieldList(HFieldList *fList)
{
   *fList = ESIGFieldList ;
}

/* EXPORT->ReadEsignalHeader: Get header from Esignal file; return FALSE
   in case of failure. */
Boolean ReadEsignalHeader(FILE *f, long *nSamp, long *sampP, short *sampS,
                          short *kind, Boolean *bSwap, long *hdrS,
                          Boolean isPipe)
{
   FieldList       list, list1;
   FieldSpec       *field;
   char            *version;
   int             inarch = UNKNOWN;   /* format variant in input */
   long            pre_size = -9999;   /* arbitrary invalid value for
                                        * uninitialized variable */
   long            hdr_size = -9999;
   long            rec_size = -9999;

   if (!(list =
         ReadHeader(&version, &inarch, &pre_size, &hdr_size, &rec_size, f)))
      {
         HError(6250,"ReadEsignalHeader: cannot read Esignal Header");
         return FALSE;
      }

   StoreESIGFieldList(list);

   if ((field=FindField(list, "parmKind")))
      *kind = Str2ParmKind ((char *) field->data);
   else
      {
         list1 = FieldOrder(list); /* obtain list with record fields only */

         if (list1 == NULL || list1[0] == NULL)
            {
               HError(6252, "ReadEsignalHeader: "
                      "no record fields defined in Esignal file");
               return FALSE;
            }
         if (list1[1] != NULL) {
            HError(6252, "ReadEsignalHeader: "
                   "extraneous field or missing item parmKind in Esignal file");
            return FALSE;
         }
         field = list1[0];
         if (field->occurrence != REQUIRED) {
            HError(6251, "ReadEsignalHeader: "
                   "field %s in Esignal file is OPTIONAL; should be REQUIRED",
                   field->name);
            return FALSE;
         }
         if (FieldLength(field) != 1) {
            HError(6251, "ReadEsignalHeader: "
                   "field %s in Esignal file has length %ld; 1 expected",
                   field->name, FieldLength(field));
            return FALSE;
         }
   
         *kind = WAVEFORM;
      }

   if ((field=FindField(list, "recordFreq")))
      *sampP = (long) (1.0E7/((double*)field->data)[0]);
   else {
      HError(6252,
             "ReadEsignalHeader: item recordFreq missing from Esignal file");
      return FALSE;
   }

   *sampS=rec_size;
   if(isPipe) *nSamp=INT_MAX;
   else *nSamp=NumberBytes(f, hdr_size, isPipe)/rec_size;
   *hdrS = hdr_size;

   switch (inarch)
      {
      case EDR1:        /* fall through */
      case EDR2:
         *bSwap = vaxOrder;
         break;
      case NATIVE:
         *bSwap = FALSE;
         break;
      case ASCII:
         HError(6251, "ReadEsignalHeader: "
                "Esignal ASCII format not supported");
         return FALSE;
      default:
         HError(6251, "ReadEsignalHeader: "
                "unrecognized architecture type in Esignal file");
         return FALSE;
      }

   return TRUE;
}

/* GetESIGHeaderInfo: get header info and check format is 1 2-byte item per record */
static long GetESIGHeaderInfo(FILE *f, Wave w, InputAction *ia)
{
   int             nsamp;
   long            nSamp, sampP;
   short           sampSize, kind;
   long            hdrS;
   Boolean         bSwap;

   if (!ReadEsignalHeader(f, &nSamp, &sampP, &sampSize,
                          &kind, &bSwap, &hdrS, w->isPipe))
      HError(6250,"GetESIGHeaderInfo: cannot read Esignal Header");

   if (kind != WAVEFORM)
      HError(6251,"GetESIGHeaderInfo: sample kind is not WAVEFORM");
   if (sampSize != 2)
      HError(6251,"GetESIGHeaderInfo: samples are not shorts");

   w->sampPeriod = sampP;
   w->hdrSize = hdrS;

   w->nSamples = nSamp; /*Will be INT_MAX if pipe*/

   if (bSwap)
      *ia = (InputAction) (*ia | DoBSWAP);

   if(w->isPipe){
      if (!GetConfInt(cParm,numParm,"NSAMPLES",&nsamp)){
         HRError(6230,"GetESIGHeaderInfo: NSAMPLES not set in config");
         return -1;
      }
      w->nSamples = nsamp;
   }
   
   return w->nSamples*2;
}


/* EXPORT->PutESIGHeader: Write header info to ESIG file f */
void PutESIGHeaderInfo(FILE *f, Wave w)
{
   FieldSpec *field, *field1;
   FieldList inList, outList=NULL;
   int len, i;

   /* Create header items first */
   field = NewFieldSpec( CHAR, 1 );
   field->occurrence=GLOBAL;
   field->name="commandLine";
   field->dim[0]=strlen(RetrieveCommandLine())+1;
   field->data=malloc(field->dim[0]);
   strcpy((char *) field->data, RetrieveCommandLine());
   AddField( &outList, field );

   field = NewFieldSpec( DOUBLE, 0 );
   field->occurrence=GLOBAL;
   field->name="recordFreq";
   field->data=malloc(sizeof(double));
   *((double*)field->data)=1.0E07/w->sampPeriod;
   AddField( &outList, field );

   field = NewFieldSpec( DOUBLE, 0 );
   field->occurrence=GLOBAL;
   field->name="startTime";
   field->data=malloc(sizeof(double));
   if (w->fmt == ESIG) {
      RetrieveESIGFieldList(&inList);
      field1=FindField(inList, "startTime");
      *((double*)field->data)=*((double*)field1->data);
   }
   else *((double*)field->data)=0.0;
   AddField( &outList, field );

   if (w->fmt == ESIG) {
      field = NewFieldSpec( NO_TYPE, 0 );
      field->occurrence=VIRTUAL;
      field->name="source_1";
      len=FieldListLength(inList);
      for (i=0;i<len;i++)
         AddSubfield(field,inList[i]);
      AddField( &outList, field );
   }

   /* Then create actual record descriptor */
   field = NewFieldSpec( SHORT, 1 );
   field->dim[0]=1;
   field->occurrence=REQUIRED;
   field->name="samples";
   AddField( &outList, field );

   WriteHeader(outList,
               (natWriteOrder && vaxOrder) ? NATIVE : EDR1,
               f, NULL);
}


/* ---------------------- HTK Format Interface Routines --------------------- */

#define WAVEFORM 0            /* HTK sample kind */

typedef struct {              /* HTK File Header */
   int32 nSamples;
   int32 sampPeriod;
   short sampSize;
   short sampKind;
} HTKhdr;

/* EXPORT ReadHTKHeader: get header from HTK file, return false not HTK */
Boolean ReadHTKHeader(FILE *f, long *nSamp, long *sampP, short *sampS, 
                      short *kind, Boolean *bSwap)
{
   HTKhdr hdr;
   int n = sizeof hdr;
   
   if (fread(&hdr, 1, n, f) != n)
      return FALSE;
   if (!natReadOrder && vaxOrder){
      *bSwap = TRUE;
   }else{
      *bSwap=MustSwap(UNKNOWNSO);
   }
   if (*bSwap){
      SwapInt32(&hdr.nSamples); 
      SwapInt32(&hdr.sampPeriod);
      SwapShort(&hdr.sampSize);
      SwapShort(&hdr.sampKind); 
   }
   if (hdr.sampSize <= 0 || hdr.sampSize > 5000 || hdr.nSamples <= 0 ||
       hdr.sampPeriod <= 0 || hdr.sampPeriod > 1000000)
      return FALSE;
   *nSamp = hdr.nSamples; *sampP = hdr.sampPeriod; 
   *sampS = hdr.sampSize; *kind = hdr.sampKind;
   return TRUE;
}

/* GetHTKHeaderInfo: get HTK format header and check its a WAVEFORM file */
static long GetHTKHeaderInfo(FILE *f, Wave w, InputAction *ia)
{
   Boolean bSwap;
   short kind,size;
   long sp;
   
   if (!ReadHTKHeader(f,&(w->nSamples),&sp, &size, &kind, &bSwap)){
      HRError(6250,"GetHTKHeaderInfo: cannot read HTK Header");
      return -1;
   }
   w->sampPeriod = sp;
   if (bSwap)  
      *ia = (InputAction) (*ia | DoBSWAP);
   if (kind != WAVEFORM){
      HRError(6251,"GetHTKHeaderInfo: sample kind is not WAVEFORM");
      return -1;
   }
   if (size != 2){
      HRError(6251,"GetHTKHeaderInfo: samples are not shorts");
      return -1;
   }
   w->hdrSize = sizeof(HTKhdr);
   return w->nSamples*2;
}

/* EXPORT->WriteHTKHeader: Write header info to HTK file f */
void WriteHTKHeader(FILE *f, long nSamp, long sampP, short sampS, 
                    short kind, Boolean *bSwap)
{
   HTKhdr hdr;
   int n = sizeof hdr;
   
   hdr.nSamples   = nSamp;
   hdr.sampSize   = sampS;
   hdr.sampKind   = kind;
   hdr.sampPeriod = sampP;
   if (!natWriteOrder && vaxOrder){
      SwapInt32(&hdr.nSamples); 
      SwapInt32(&hdr.sampPeriod);
      SwapShort(&hdr.sampSize);
      SwapShort(&hdr.sampKind);
      if (bSwap!=NULL) *bSwap=TRUE;
   }
   else
      if (bSwap!=NULL) *bSwap=FALSE;
   if (fwrite(&hdr, 1, n, f) != n)
      HError(6214,"WriteHTKHeader: Cannot write HTK format header"); 
}


/* PutHTKHeaderInfo: write a HTK (WAVEFORM) file header */
static void PutHTKHeaderInfo(FILE *f, Wave w)
{
   WriteHTKHeader (f, w->nSamples, (long) w->sampPeriod, 2, WAVEFORM,NULL);
}
   
/* ---------------------- Wave Interface Routines --------------------- */
   
/* EXPORT-> OpenWaveInput: open input waveform file */
Wave OpenWaveInput(MemHeap *x, char *fname, FileFormat fmt, HTime winDur, 
                   HTime frPeriod, HTime *sampPeriod)
{
   Wave w;                /* New Wave Object */
   FILE *f;               /* Input data file */
   InputAction ia=(InputAction)0;      /* flags to enable conversions etc */
   long fBytes=0;         /* Num data bytes in file to load */
   HTime t;
   Boolean isEXF;	  /* File name is extended */
   char actfile[MAXFNAMELEN]; /* actual file name */
   long stindex,enindex;  /* segment indices */
   int sampSize = 2;       

   /* Create Wave Object and open external file */
   w = CreateWave(x,fmt);

   /* extended filename handling */
   strcpy(actfile,fname);
   isEXF = GetFileNameExt(fname,actfile,&stindex,&enindex);
   if ((f = FOpen(actfile, WaveFilter, &(w->isPipe))) == NULL) {
      HError(6210,"OpenWaveInput: Cannot open waveform file %s",fname);
      return NULL;
   }
   if (isEXF && w->isPipe && stindex >= 0) {
      HError(6210,"OpenWaveInput: cannot segment piped input");
      return NULL;
   }
   
   /* Get Header  */
   switch(w->fmt) {
   case ALIEN:    fBytes = GetALIENHeaderInfo (f, w, &ia);  break;
   case NOHEAD:   fBytes = GetNOHEADHeaderInfo(f, w, &ia);  break;
   case HTK:      fBytes = GetHTKHeaderInfo   (f, w, &ia);  break;
   case TIMIT:    fBytes = GetTIMITHeaderInfo (f, w, &ia);  break; 
   case OGI:      fBytes = GetOGIHeaderInfo   (f, w, &ia);  break; 
   case NIST:     fBytes = GetNISTHeaderInfo  (f, w, &ia);  break; 
   case SCRIBE:   fBytes = GetSCRIBEHeaderInfo(f, w, &ia);  break; 
   case SDES1:    fBytes = GetSDES1HeaderInfo (f, w, &ia);  break; 
   case AIFF:     fBytes = GetAIFFHeaderInfo  (f, w, &ia);  break;
   case SUNAU8:   fBytes = GetSUNAU8HeaderInfo(f, w, &ia);
      sampSize = 1;
      break;
   case WAV:      fBytes = GetWAVHeaderInfo(f, w, &ia);     break;
   case ESIG:     fBytes = GetESIGHeaderInfo(f, w, &ia);    break;
   default:       
      FClose(f,w->isPipe);  
      HRError(6270,"OpenWaveInput: Unknown format %s",fmtmap[w->fmt]);
      return(NULL);
      break;
   }

   /* If extended indexed file, modify header and skip to segment start */
   if (isEXF && stindex >= 0) {
      if (enindex < 0) enindex = w->nSamples-1;
      if (w->nSamples < enindex-stindex+1)
         HError(6210,"OpenWaveInput: EXF segment bigger than file");
      w->nSamples = enindex-stindex+1;
      fseek(f,stindex*sampSize,SEEK_CUR);
      fBytes = w->nSamples*sampSize;
   }

   if(fBytes<0){
      FClose(f,w->isPipe);  
      HRError(6213,"OpenWaveInput: Get[format]HeaderInfo failed");
      return(NULL);
   }
   
   /* Check for user override of sample period and set frame size/rate */
   if (*sampPeriod > 0.0)  
      w->sampPeriod = *sampPeriod;
   else {
      if (GetConfFlt(cParm,numParm,"SOURCERATE",&t))
         w->sampPeriod = t;
   }
   *sampPeriod = w->sampPeriod;
   if (w->sampPeriod <= 0.0){ 
      FClose(f,w->isPipe);  
      HRError(6230,"OpenWaveInput: cannot determine sample period for %s",fname);
      return(NULL);
   }
   w->frSize = (int) (winDur / w->sampPeriod);
   w->frRate = (int) (frPeriod / w->sampPeriod);
   
   
   /* Copy Data from file into memory */
   if(LoadData(f, w, fBytes)<SUCCESS){
      FClose(f,w->isPipe);  
      HRError(6213,"OpenWaveInput: LoadData failed");
      return(NULL);
   }
   
   /* If necessary call format dependent conversion routine */
   if (ia&DoCVT)
      switch(w->fmt) {
      case NIST:     
         if(ConvertNISTData(w,ia)<SUCCESS){ 
            FClose(f,w->isPipe);  
            HRError(6270,"OpenWaveInput: No Convertor for format %s",fmtmap[w->fmt]);
            return(NULL);
         }
         break;
      case SUNAU8:   
         if(ConvertSUNAU8Data(w)<SUCCESS){   
            FClose(f,w->isPipe);  
            HRError(6270,"OpenWaveInput: No Convertor for format %s",fmtmap[w->fmt]);
            return(NULL);
         }
         break;
      case WAV:     
         if(ConvertWAVData(w,&ia)<SUCCESS){ 
            FClose(f,w->isPipe);  
            HRError(6270,"OpenWaveInput: No Convertor for format %s",fmtmap[w->fmt]);
            return(NULL);
         }
         break;
      default: 
         FClose(f,w->isPipe);  
         HRError(6270,"OpenWaveInput: No Convertor for format %s",fmtmap[w->fmt]);
         return(NULL);
         break;
      }
          
   /* If necessary byte swap the waveform data */
   if (ia&DoBSWAP)
      ByteSwap(w);
   
   FClose(f,w->isPipe);  
   if (trace&T_OPEN){
      printf("HWave: Input Wave Opened from %s\n",fname);
      ShowWaveInfo(w);
   }
   return w;
}

/* EXPORT->CloseWaveInput: close the given data file and free storage */
void CloseWaveInput(Wave w)
{
   Dispose(w->mem, w);
}

/* EXPORT->ZeroMeanWave: remove mean of wave w */
void ZeroMeanWave(Wave w)
{
   int i;
   short *x;
   double sum=0.0,y,mean;

   x = (short *)w->data;
   for (i=0; i<w->nSamples; i++,x++)
      sum += *x;
   mean = sum / (float)w->nSamples;

   x = (short *)w->data;
   for (i=0; i<w->nSamples; i++,x++){
      y = (float)*x - mean;
      if (y<-32767){
         y = -32767;
         HError(-6255,"ZeroMeanWave: sample too -ve\n");
      }
      if (y>32767){
         y = 32767;
         HError(-6255,"ZeroMeanWave: sample too +ve\n");
      }
      *x = (short) ((y>0.0) ? y+0.5 : y-0.5);
   }
}

/* EXPORT->FramesInWave: return number of frames in given wave */
int FramesInWave(Wave w)
{
   if (w->frSize + w->frIdx > w->nSamples )
      return 0;
   return (w->nSamples - w->frSize - w->frIdx) / w->frRate + 1;
}

/* EXPORT->SampsInWaveFrame: return number of samples per frame */
int SampsInWaveFrame(Wave w)
{
   return w->frSize;
}

/* EXPORT->WaveFormat: return format of given wave */
FileFormat WaveFormat(Wave w)
{
   return w->fmt;
}

/* EXPORT->GetWave: Get next nFrames from w and store in buf */
void GetWave(Wave w, int nFrames, float *buf)
{
   int i,k;
   
   for (i=0; i<nFrames; i++){
      if (w->frIdx+w->frSize > w->nSamples)
         HError(6271,"GetWave: attempt to read past end of buffer");
      for (k=0; k<w->frSize; k++)
         *buf++ = w->data[w->frIdx+k];
      w->frIdx += w->frRate;
   }
}

/* EXPORT->GetWaveDirect: return a pointer to the wave in w */
short *GetWaveDirect(Wave w, long *nSamples)
{
   *nSamples = w->nSamples;
   return w->data;
}

/* EXPORT->OpenWaveOutput: create wave object for output with given bufSize */
Wave OpenWaveOutput(MemHeap *x, HTime *sampPeriod, long bufSize)
{
   Wave w;
   HTime t;
   
   w = CreateWave(x,HTK);
   if (*sampPeriod>0.0)
      w->sampPeriod = *sampPeriod;
   else {
      if (GetConfFlt(cParm,numParm,"TARGETRATE",&t))
         w->sampPeriod = *sampPeriod = t;
      else
         w->sampPeriod = 0.0;
   }
   w->nAvail = bufSize;
   w->data = (short *)New(w->mem,bufSize*2);
   if (trace&T_OPEN){
      printf("HWave: Output Wave Opened\n");
      ShowWaveInfo(w);
   }
   return w;
}

/* EXPORT->PutWaveSample: Append given nSamples in buf to wave w */
void PutWaveSample(Wave w, long nSamples, short *buf)
{
   long i,newSize,needed;
   short *p,*tmp;
   
   needed = w->nSamples+nSamples;
   if (needed > w->nAvail){
      /* increase size of data buffer - assume that only
         w->data has been allocated on w->mem stack */
      newSize = w->nAvail + w->nAvail / 2;
      if (newSize < needed)
         newSize = needed;
      tmp = (short *)New(&gstack,w->nSamples*2);
      memcpy(tmp,w->data,w->nSamples*2);
      Dispose(w->mem,w->data);
      p = (short *)New(w->mem,newSize*2);
      memcpy(p,tmp,w->nSamples*2);
      Dispose(&gstack,tmp);
      w->nAvail = newSize;
      w->data = p;
   }
   for (i=0,p=w->data+w->nSamples; i<nSamples; i++)      
      *p++ = buf[i];
   w->nSamples += nSamples;
}

/* EXPORT->CloseWaveOutput: Output w to fname and free mem */
ReturnStatus CloseWaveOutput(Wave w, FileFormat fmt, char *fname)
{
   FILE *f;
   Boolean isPipe;
   char buf[MAXSTRLEN];

   if ((f = FOpen(fname, WaveOFilter, &isPipe)) == NULL){   /* Open file */
      HRError(6211,"CloseWaveOutput: Cannot create file %s",fname);
      return(FAIL);
   }
   /* Output Header in appropriate format */
   if (fmt == UNDEFF){
      if (GetConfStr(cParm,numParm,"TARGETFORMAT",buf))
         fmt = Str2Format(buf);
      else
         fmt = HTK;
   }
   switch(fmt) {
   case HTK:    PutHTKHeaderInfo(f,w);     break;
   case ESIG:   PutESIGHeaderInfo(f,w);    break;
   case NOHEAD: /* do nothing */           break;
   default: 
      FClose(f,isPipe);
      HRError(6270,"CloseWaveOutput: Cannot save data as %s.", 
              fmtmap[fmt]);  
      return(FAIL);
      break;
   }
   /* If necessary byte swap the waveform data */
   if (!natWriteOrder && vaxOrder)
      ByteSwap(w);
   /* Write Body of file */
   if (fwrite(w->data, 2, w->nSamples, f) != w->nSamples){
      FClose(f,isPipe);
      HRError(6214,"CloseWaveOutput: Cannot write data to %s",fname);
      return(FAIL);
   }
   /* Close file  and free memory */
   FClose(f,isPipe);
   Dispose(w->mem, w);
   return(SUCCESS);
}

/* --------------------------------  HWave.c ------------------------------- */
