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
/*         File: HParm.h:   Speech Parameter Input/Output      */
/* ----------------------------------------------------------- */

/* !HVER!HParm:   3.4.1 [CUED 12/03/09] */

#ifndef _HPARM_H_
#define _HPARM_H_

#ifdef __cplusplus
extern "C" {
#endif

enum _BaseParmKind{
      WAVEFORM,            /* Raw speech waveform (handled by HWave) */
      LPC,LPREFC,LPCEPSTRA,LPDELCEP,   /* LP-based Coefficients */
      IREFC,                           /* Ref Coef in 16 bit form */
      MFCC,                            /* Mel-Freq Cepstra */
      FBANK,                           /* Log Filter Bank */
      MELSPEC,                         /* Mel-Freq Spectrum (Linear) */
      USER,                            /* Arbitrary user specified data */
      DISCRETE,                        /* Discrete VQ symbols (shorts) */
      PLP,                             /* Standard PLP coefficients */
      ANON};
      
typedef short ParmKind;          /* BaseParmKind + Qualifiers */
                                 
#define HASENERGY  0100       /* _E log energy included */
#define HASNULLE   0200       /* _N absolute energy suppressed */
#define HASDELTA   0400       /* _D delta coef appended */
#define HASACCS   01000       /* _A acceleration coefs appended */
#define HASCOMPX  02000       /* _C is compressed */
#define HASZEROM  04000       /* _Z zero meaned */
#define HASCRCC  010000       /* _K has CRC check */
#define HASZEROC 020000       /* _0 0'th Cepstra included */
#define HASVQ    040000       /* _V has VQ index attached */
#define HASTHIRD 0100000       /* _T has Delta-Delta-Delta index attached */

#define BASEMASK  077         /* Mask to remove qualifiers */

/*
   An observation contains one or more stream values each of which 
   is either a vector of continuous values and/or a single
   discrete symbol.  The discrete vq symbol is included if the
   target kind is DISCRETE or the continuous parameter has the
   HASVQ qualifier. Observations are input via buffers or tables.  A
   buffer is a FIFO structure of potentially infinite length and it is
   always sourced via HAudio.  A table is a random access array of
   observations and it is sourced from a file possibly via HWave.
   Buffers are input only, a table can be input and output.
   Too allow discrete systems to be used directly from continuous
   data the observation also holds a separate parm kind for the
   parm buffer and routines which supply observations use this to
   determine stream widths when the observation kind is DISCRETE.
*/

typedef enum {
   FALSE_dup=FALSE, /*  0 */
   TRUE_dup=TRUE,   /*  1 */
   TRI_UNDEF=-1     /* -1 */
}
TriState;

typedef struct {
   Boolean eSep;         /* Energy is in separate stream */
   short swidth[SMAX];   /* [0]=num streams,[i]=width of stream i */
   ParmKind bk;          /* parm kind of the parm buffer */
   ParmKind pk;          /* parm kind of this obs (bk or DISCRETE) */
   short vq[SMAX];       /* array[1..swidth[0]] of VQ index */
   Vector fv[SMAX];      /* array[1..swidth[0]] of Vector */
} Observation;

/*
   A ParmBuf holds either a static table of parameter frames
   loaded from a file or a potentially infinite sequence
   of frames from an audio source. The key information relating 
   to the speech data in a buffer or table can be obtained via 
   a BufferInfo Record.  A static table behaves like a stopped
   buffer.
*/

typedef enum { 
   PB_INIT,     /* Buffer is initialised and empty */
   PB_WAITING,  /* Buffer is waiting for speech */
   PB_STOPPING, /* Buffer is waiting for silence */
   PB_FILLING,  /* Buffer is filling */
   PB_STOPPED,  /* Buffer has stopped but not yet empty */
   PB_CLEARED   /* Buffer has been emptied */
} PBStatus;

typedef struct _ParmBuf  *ParmBuf;

typedef struct {
   ParmKind srcPK;            /* Source ParmKind */ 
   FileFormat srcFF;          /* Source File format */ 
   HTime srcSampRate;         /* Source Sample Rate */ 
   int srcVecSize;            /* Size of source vector */
   long nSamples;             /* Number of source samples */
   int frSize;                /* Number of source samples in each frame */
   int frRate;                /* Number of source samples forward each frame */
   int nObs;                  /* Number of table observations */
   ParmKind tgtPK;            /* Target ParmKind */ 
   FileFormat tgtFF;          /* Target File format */ 
   HTime tgtSampRate;         /* Target Sample Rate */ 
   int tgtVecSize;            /* Size of target vector */
   AudioIn a;                 /* the audio source - if any */
   Wave w;                    /* the wave input - if any */
   Ptr i;                     /* the other input - if any */
   Boolean useSilDet;         /* Use Silence Detector */
   int audSignal;             /* Signal Number for Audio Control */
   char *vqTabFN;             /* Name of VQ Table Defn File */
   Boolean saveCompressed;    /* Save in compressed format */
   Boolean saveWithCRC;       /* Save with CRC check added */
   Boolean spDetParmsSet;     /* Parameters set for sp/sil detector */
   float spDetSil;            /* Silence level for channel */
   float chPeak;              /* Peak-to-peak input level for channel */
   float spDetSp;             /* Speech level for channel */
   float spDetSNR;            /* Speech/noise ratio for channel */
   float spDetThresh;         /* Silence/speech level threshold */
   float curVol;              /* Volume level of last frame (0.0-100.0dB) */
   int spDetSt;               /* Frame number of first frame of buffer */
   int spDetEn;               /* Frame number of last frame of buffer */
   char *matTranFN;           /* Matrix transformation name */
   Ptr xform;                 /* Used for input xform associated with this buffer */
}BufferInfo;

/*
   External source definition structure
*/

typedef struct hparmsrcdef *HParmSrcDef;

/* -------------------- Initialisation ------------------ */

ReturnStatus InitParm(void);
/*
   Initialise the module
*/

/* -------------------- Channel functions ------------------ */

ReturnStatus SetChannel(char *chanName);
/* 
   Set the current channel to use config parameters from chanName.
*/

void ResetChannelSession(char *chanName);
/* 
   Reset the session for the specified channel (NULL indicates default)
*/

/* 
   The next two functions have been kept to allow for backwards 
   compatibility.
*/
void SetNewConfig(char * libmod);
void ResetCurCepMean(void);

/* ---------------- Buffer Input Routines ------------------ */

ParmBuf OpenBuffer(MemHeap *x, char *fn, int maxObs, FileFormat ff, 
		   TriState enSpeechDet, TriState silMeasure);
/*
   Open and return a ParmBuf object connected to the current channel.
   If maxObs==0 blocks and reads whole of file/audio into memory and
   returns with status==PB_STOPPED ready for table access.  All 
   parameters, associated with the loading and conversion of the
   source are defined using configuration parameters.
   If maxObs!=0 buffer may be read as a stream.  In this case reading
   should be via ReadAsBuffer calls which should continue until either
   ReadAsBuffer returns FALSE or buffer status >= PB_STOPPED.  Note 
   that for some types of input (eg pipes) end of data can only be 
   determined by a failed attempt to read the final frame.
   If the speech detector is enabled (either by configuration or
   by explicit parameter in call) then silence measurement can be
   forced/prevented by setting silMeasure to TRUE/FALSE (if UNDEF
   will perform measurement if it is needed by config).
*/

PBStatus BufferStatus(ParmBuf pbuf);
/* 
   Return current status of buffer.
    PB_INIT - buffer ready for StartBuffer call. No observations available.
    PB_FILLING - buffer is currently reading from source.
    PB_STOPPED - source has closed and buffer can be used as a table.
    PB_CLEARED - same as PB_STOPPED but ReadAsBuffer has read final frame.
   Does not block.
*/

int ObsInBuffer(ParmBuf pbuf);
/* 
   Return number of observations available to ReadAsBuffer without blocking.
   This will be zero once the buffer is COMPLETE although ReadAsTable can
   still be used to access whole buffer (use GetBufferInfo to find
   range of allowable indexes).
   Note that final frame may not be read and the source closed until a
   ReadAsBuffer call has to read the final frame (this is only a problem
   for HParm/HWave file descriptors which normally get read immediately in
   full).  This is non-ideal but cannot otherwise guarantee ObsInBuffer 
   is non-blocking.
*/

void StartBuffer(ParmBuf pbuf);
/*
   Start and filling the buffer.  If signals have been enabled
   then effect is delayed until first signal is sent.  If
   silence/speech detection is enabled then frames will 
   accumulate when speech starts and buffer will stop filling
   when silence is detected.  If silence/speech detection is
   not enabled but signals are, then a second signal will stop
   the filling.  This operation will fail if pbuf status is not
   PB_INIT.
   This operation should now be non-blocking.
*/
   
void StopBuffer(ParmBuf pbuf);
/*   
   Filling the buffer is stopped regardless of whether signals
   and/or silence/speech detection is enabled.  After making 
   this call, the pbuf status will change to PB_STOPPED.  
   Only when the buffer has been emptied will the status change
   to PB_CLEARED.
*/

void CloseBuffer(ParmBuf pbuf);
/*
   Close the given buffer, close the associated audio stream if
   any and release any associated memory.
*/

Boolean ReadAsBuffer(ParmBuf pbuf, Observation *o);
/*
   Get next observation from buffer.  Buffer status must be PB_FILLING 
   or PB_STOPPED.  If no observation is available the function
   will block until one is available or the input is closed.
   Will returns FALSE if blocked but could not read new Observation.
*/

void ReadAsTable (ParmBuf pbuf, int index, Observation *o);
/* 
   Get the index'th observation from buffer.  Buffer status
   must be PB_STOPPED.  Index runs 0,1,2,....
   By definition this operation is non-blocking.
*/


void GetBufferInfo(ParmBuf pbuf, BufferInfo *info);
/*
   Get info associated with pbuf.
   Does not block.
*/

/* ---------------- External Data Source Handling---------------- */

HParmSrcDef CreateSrcExt(Ptr xInfo, ParmKind pk, int size, HTime sampPeriod,
                         Ptr (*fOpen)(Ptr xInfo,char *fn,BufferInfo *info),
                         void (*fClose)(Ptr xInfo,Ptr bInfo),
                         void (*fStart)(Ptr xInfo,Ptr bInfo),
                         void (*fStop)(Ptr xInfo,Ptr bInfo),
                         int (*fNumSamp)(Ptr xInfo,Ptr bInfo),
                         int (*fGetData)(Ptr xInfo,Ptr bInfo,int n,Ptr data));
/*
  Create and return a HParmSrcDef object handling an external data source.
  size: 1 for 8 bit u-law, 0x101 for 8 bit a-law or 2 for 16 bit linear
  Semantics of functions fClose() etc. described in HParm.c.
*/

ParmBuf OpenExtBuffer(MemHeap *x, char *fn, int maxObs,
                      FileFormat ff, HParmSrcDef ext,
                      TriState enSpeechDet, TriState silMeasure);

/*
  Open and return input buffer using an external source
*/

/* ----------------- New Buffer Creation Routines -------------- */

ParmBuf EmptyBuffer(MemHeap *x, int size, Observation o, BufferInfo info);
/*
   Create and return an empty ParmBuf object set-up as a table
   with initially size free observation slots.  Observation o is 
   used for sizing and info supplies associated configuration
   parameters.  The latter will typically be copied from a
   buffer created by an OpenBuffer call.
*/

ReturnStatus SaveBuffer(ParmBuf pbuf, char *fname, FileFormat ff);
/*
   Write contents of given buffer to fname.  If SAVEWITHCRC is set in
   config then a cyclic redundancy check code is added.  If
   SAVECOMPRESSED is set then the data in the table is compressed
   before writing out.  If ff is not UNDEFF then ff overrides
   target file format set in buffer.
*/

void AddToBuffer(ParmBuf pbuf, Observation o);
/*
   Append the given observation to the table.
*/

/* ----------------- Observation Handling Routines -------------- */

Observation MakeObservation(MemHeap *x, short *swidth, 
                            ParmKind pkind, Boolean forceDisc, Boolean eSep);
/*
   Create observation using info in swidth, eSep and pkind
   If forceDisc is true the observation will be DISCRETE but can
   read from a continuous parameter parmbuffer.
*/

void ExplainObservation(Observation *o, int itemsPerLine);
/* 
   Explain the structure of given observation by printing
   a template showing component structure
*/

void PrintObservation(int i, Observation *o, int itemsPerLine);
/*
   Print the given observation. If i>0 then print with an index.
*/

void ZeroStreamWidths(int numS, short *swidth);
/*
   Stores numS in swidth[0] and sets remaining components of
   swidth to zero
*/

void  SetStreamWidths(ParmKind pk, int size, short *swidth, Boolean *eSep);
/*
   If swidth has been 'zeroed' by ZeroStreamWidths, then this function
   sets up stream widths in swidth[1] to swidth[S] for number of streams
   S specified in swidth[0]. If eSep then energy is extracted as a 
   separate stream.  If swidth[n] is non-zero, then it only eSep is
   set.
*/

/* EXPORT->SyncBuffers: if matrix transformations are used this syncs the two buffers */
Boolean SyncBuffers(ParmBuf pbuf,ParmBuf pbuf2);

void SetParmHMMSet(Ptr hset);
/* 
   The prototype  for this should really be 
   void SetParmHMMSet(HMMSet *hset);
   However the .h files have an issue with this. A 
   cast is performed in the first line of the function.
*/


/* ------------------- Parameter Kind Conversions --------------- */

char *ParmKind2Str(ParmKind kind, char *buf);
ParmKind Str2ParmKind(char *str);
/*
   Convert between ParmKind type & string form.
*/

ParmKind BaseParmKind(ParmKind kind);
Boolean HasEnergy(ParmKind kind);
Boolean HasDelta(ParmKind kind);
Boolean HasNulle(ParmKind kind);
Boolean HasAccs(ParmKind kind);
Boolean HasThird(ParmKind kind);
Boolean HasCompx(ParmKind kind);
Boolean HasCrcc(ParmKind kind);
Boolean HasZerom(ParmKind kind);
Boolean HasZeroc(ParmKind kind);
Boolean HasVQ(ParmKind kind);
/* 
   Functions to separate base param kind from qualifiers 
*/

Boolean ValidConversion(ParmKind src, ParmKind tgt);
/* 
   Checks that src -> tgt conversion is possible 
*/

#ifdef __cplusplus
}
#endif

#endif  /* _HPARM_H_ */
/* ------------------------ End of HParm.h ------------------------- */
