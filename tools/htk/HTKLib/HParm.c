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
/*          2001-2002 Cambridge University                     */
/*                    Engineering Department                   */
/*                                                             */
/*   Use of this software is governed by a License Agreement   */
/*    ** See the file License for the Conditions of Use  **    */
/*    **     This banner notice must not be removed      **    */
/*                                                             */
/* ----------------------------------------------------------- */
/*       File: HParm.c:  Speech Parameter File Input/Output    */
/* ----------------------------------------------------------- */

char *hparm_version = "!HVER!HParm:   3.4.1 [CUED 12/03/09]";
char *hparm_vc_id = "$Id: HParm.c,v 1.1.1.1 2006/10/11 09:54:58 jal58 Exp $";

#include "HShell.h"
#include "HMem.h"
#include "HMath.h"
#include "HSigP.h"
#include "HAudio.h"
#include "HWave.h"
#include "HVQ.h"
#include "HParm.h"
#include "HLabel.h"
#include "HModel.h"
#include "esignal.h"
#ifdef UNIX
#include <sys/ioctl.h>
#endif

/* ----------------------------- Trace Flags ------------------------- */

static int trace = 0;
#define T_TOP  0001     /* Top Level tracing */
#define T_BUF  0002     /* Buffer operations */
#define T_CPX  0004     /* Compression/Decompression */
#define T_PBS  0010     /* Buffer status */
#define T_QUA  0020     /* Qualifier operations */
#define T_OBS  0040     /* Observation extraction */
#define T_DET  0100     /* Silence detector operation */
#define T_MAT  0200     /* Matrix operations */

/* --------------------- Global Variables ------------------- */

static Boolean natWriteOrder = FALSE; /* Preserve natural write byte order*/
extern Boolean vaxOrder;              /* true if byteswapping needed to 
                                               preserve SUNSO */
/* varScale stuff: acts as a cache to stop the scaling file being re-read 
   on each file opening */

static float varScale[100];
static int varScaleDim=0;
static char varScaleFN[MAXFNAMELEN] = "\0";

static Boolean highDiff = FALSE;   /* compute higher oder differentials, only up to fourth */
static Boolean UseOldXFormCVN = FALSE;  /* this allows us to go back to the old version with broken CVN */
static ParmKind ForcePKind = ANON; /* force to output a customized parm kind to make older versions
                                    happy for all the parm kind types supported here */

static HMMSet *hset = NULL;        /* hmmset to be used for frontend */

/* ------------------------------------------------------------------- */
/* 
   Parameter layout in tables/buffers is
   
      Static [C0] [E]  Deltas Accs
      
   _N option is ignored everywhere except when copying from buffer
   or table into an observation (ie in ExtractObservation) and in
   GetBufferInfo() which returns the observation vector size in 
   tgtvecSize taking into account _N. 
   When _0 is used alone it behaves exactly like _E.  When _0_E,
   C0 is placed immediately before energy and in this case deltas
   are not allowed.
*/

/* ----------------- Configuration Information ----------------- */

/* 
   An IOConfig record specifies the mapping from the source 
   to the target parameterisation.  Its built in defaults
   can be overridden using configuration parameters.
*/

typedef enum { FFTbased, LPCbased, VQbased} CodeStyle;

typedef struct {
   /* ------- Overrideable parameters ------- */
   ParmKind srcPK;            /* Source ParmKind */ 
   FileFormat srcFF;          /* Source File format */ 
   HTime srcSampRate;         /* Source Sample Rate */ 
   Boolean zMeanSrc;          /* Zero Mean the Source */
   ParmKind tgtPK;            /* Target ParmKind */ 
   FileFormat tgtFF;          /* Target File format */ 
   HTime tgtSampRate;         /* Target Sample Rate */ 
   Boolean saveCompressed;    /* If LPREFC save as IREFC else _C */
   Boolean saveWithCRC;       /* Append check sum on save */
   HTime winDur;              /* Source window duration */
   Boolean useHam;            /* Use Hamming Window */
   float preEmph;             /* PreEmphasis Coef */
   Boolean usePower;          /* Use power instead of Magnitude */
   int numChans;              /* Number of filter bank channels */
   float loFBankFreq;         /* Fbank lo frequency cut-off */
   float hiFBankFreq;         /* Fbank hi frequency cut-off */
   float warpFreq;            /* Warp freq axis for vocal tract normalisation */
   float warpLowerCutOff;     /* lower and upper threshold frequencies */
   float warpUpperCutOff;     /*   for linear frequency warping */  
   int lpcOrder;              /* Order of lpc analysis */
   float compressFact;        /* Compression factor for PLP */  
   int cepLifter;             /* Cepstral liftering coef */
   int numCepCoef;            /* Number of cepstral coef */
   float cepScale;            /* Scaling factor to avoid arithmetic problems */
   Boolean rawEnergy;         /* Use raw energy before preEmp and ham */
   Boolean eNormalise;        /* Normalise log energy */
   float eScale;              /* Energy scale factor */
   float silFloor;            /* Silence floor in dBs */
   int delWin;                /* Delta window halfsize */
   int accWin;                /* Accel window halfsize */
   Boolean simpleDiffs;       /* Use simple differences for delta calcs */
   /* Silence detector parameters */
   Boolean useSilDet;         /* Use Silence Detector */
   int selfCalSilDet;         /* Self calibrating silence detection */
   float spThresh;            /* Speech Threshold (in dB above sil level) */
   float silDiscard;          /* Calibrate discard level */
   float silMean;             /* Mean silence energy (in dB) */
   int spcSeqCount;           /* Number of frames for speech window */
   int spcGlchCount;          /*   of spc in sil acceptable as glitches */
   int silGlchCount;          /*   of sil in spc acceptable as glitches */
   int silSeqCount;           /*   of silence before stopping */
   int marginCount;           /*   of sil around speech to process */
   Boolean measureSil;        /* Measure Silence */
   Boolean outSilWarn;        /* Give Warning when SilMeas */
   /* Misc */
   int audSignal;             /* Signal Number for Audio Control */
   Boolean v1Compat;          /* V1 compatibility mode */
   char *vqTabFN;             /* Name of VQ Table Defn File */
   float addDither;           /* Additional dither added to file */
   Boolean doubleFFT;         /* use twice the required FFT size */
  /* side based normalisation */
   char *varScaleFN;          /* var scale file name */          
   char* cMeanDN;             /* dir to find cepstral mean files */
   char* cMeanMask;           /* cepstral mean selection mask */
   char* cMeanPathMask;       /* cepstral mean path selection mask */
   char* varScaleDN ;         /* dir to find variance estimate files */
   char* varScaleMask;        /* variance estimate file selection mask */
   char* varScalePathMask;    /* variance estimate file path selection mask */
   char* sideXFormMask;       /* side XForm mask */
   char* sideXFormExt;       /* side XForm mask */

   VQTable vqTab;             /* VQ table */
   Matrix MatTran;            /* Stores transformation matrix */ 
   char *MatTranFN;           /* points to the file name string */
   int thirdWin;              /* Accel window halfsize */
   int fourthWin;             /* Fourth order differential halfsize */

   /* ------- Internally derived parameters ------- */
   /*  These values are allocated in the IOConfigRec but are really */
   /*  specific to each pbuf and do not rely on any kind of initialisation */
   /* Following 3 variables always reflect the actual state of */
   /* the associated data which may be intermediate between src and tgt */
   ParmKind curPK;    /* Used to track conversion from srcPK to tgtPK */
   ParmKind unqPK;    /* Used to track conversion from srcPK to tgtPK */
   int nUsed;         /* num columns used in each row of the parm block */
   /* The next two are static buffer sizes */
   int nCols;         /* num columns in each row of the parameter block */
   int nCvrt;         /* num columns produced from coding */
   /* sizes of source and target */
   long nSamples;     /* num samples in original (WAVEFORM only) */
   int srcUsed;       /* num columns which was used in source */
   int tgtUsed;       /* num columns which will be used once converted */
   /* Working storage needed for conversions, etc */
   CodeStyle style;   /* style encoding */
   int frSize;        /* Total number of waveform samples in frame */
   int frRate;        /* Number of waveform samples advanced each frame */
   Vector s;          /* speech vector */
   ShortVec r;        /* raw speech vector */
   char *rawBuffer;   /* buffer for external data */
   float curVol;      /* current volume dB (0.0-100.0) */
   Vector a,k;        /* lpc and refc vectors */
   Vector fbank;      /* filterbank vector */
   Vector c;          /* cepstral vector */
   Vector as, ac, lp; /* Auditory, autocorrelation an lp vectors for PLP */ 
   Vector eql;        /* Equal loundness curve */
   DMatrix cm;        /* Cosine matrix for IDFT */ 
   FBankInfo fbInfo;  /* FBank info used for filterbank analysis */
   Vector mean;       /* Running mean shared by this config */
   /* Running stuff */
   Source src;        /* Source to read HParm file from */
   Boolean bSwap;     /* TRUE if source needs byte swapping */
   unsigned short crcc; /* Running CRCC */
   Vector A;          /* Parameters for decompressing */
   Vector B;          /*  HTK parameterised files */
   Vector varScale;   /* var scaling vector  */
   Vector cMeanVector;   /* vector loaded from cmean dir */
   Vector varScaleVector; /* vector loaded from varscale dir */
   ParmKind matPK;
   int preFrames;
   int postFrames;
   Boolean preQual;
   InputXForm *xform;
   AdaptXForm *sideXForm;
}IOConfigRec;

typedef IOConfigRec *IOConfig;

typedef enum {
   /* Source characteristics */
   SOURCEKIND,    /* ParmKind */ 
   SOURCEFORMAT,  /* FileFormat */
   SOURCERATE,    /* Source sample rate in 100ns */
   ZMEANSOURCE,   /* Zero Mean (Wave only) */
   /* Target characteristics */
   TARGETKIND,    /* ParmKind */
   TARGETFORMAT,  /* FileFormat */
   TARGETRATE,    /* Target sample rate in 100ns */
   SAVECOMPRESSED,/* Save output files in compressed form */
   SAVEWITHCRC,   /* Add crc check to output files */
   /* Waveform Analysis */
   WINDOWSIZE,    /* Window size in 100ns */ 
   USEHAMMING,    /* Apply Hamming Window */
   PREEMCOEF,     /* Preemphasis Coefficient */
   /* Filterbank Analysis */
   USEPOWER,      /* Use power instead of magnitude */
   NUMCHANS,      /* Num filterbank channels */
   LOFREQ,        /* Lo Fbank frequency */
   HIFREQ,        /* Hi Fbank frequency */
   WARPFREQ,      /* Vocal tract length compensation by frequency warping */
   WARPLCUTOFF,   /* VTL warping cutoff frequencies for smoothing */
   WARPUCUTOFF,
   /* LPC Analysis and Conversion */
   LPCORDER,      /* LPC order */      
   COMPRESSFACT,  /* Compression Factor fo PLP */
   /* Cepstral Conversion */
   CEPLIFTER,     /* Cepstral liftering coefficient */
   NUMCEPS,       /* Num cepstral coefficients */
   CEPSCALE,      /* Scale factor to prevent arithmetic errors */
   /* Energy Computation */
   RAWENERGY,     /* Use raw energy */
   ENORMALISE,    /* Normalise log energy */
   ESCALE,        /* Log energy scale factor */
   SILFLOOR,      /* Silence floor in dBs */
   /* Regression Coefficients */
   DELTAWINDOW,   /* Window size for 1st diffs */
   ACCWINDOW,     /* Window size for 2nd diffs */
   SIMPLEDIFFS,   /* Use simple differences */
   /* Silence Detector */
   USESILDET,     /* Enable speech/silence detection */
   SELFCALSILDET, /* Self calibrating silence detection on each utterance */
   SPEECHTHRESH,  /* Speech detector threshold */
   SILDISCARD,    /* Energy below which frames discarded when calibrating */
   SILENERGY,     /* Silence detector threshold */
   SPCSEQCOUNT,   /* Speech sequence count */
   SPCGLCHCOUNT,  /* Speech glitch count */
   SILGLCHCOUNT,  /* Silence glitch count */
   SILSEQCOUNT,   /* Silence sequence count */
   SILMARGIN,     /* Margin of silence around speech */
   MEASURESIL,    /* Measure Background Silence */
   OUTSILWARN,    /* Output Warning before Measure Sil */
   /* Audio Input */
   AUDIOSIG,      /* Signal for audio control */
   V1COMPAT,      /* Set Version 1 compatibility mode */
   /* Vector Quantisation */
   VQTABLE,       /* Name of file holding VQ table */
   ADDDITHER,     /* Amount of additional dither added to file */
   DOUBLEFFT,     /* Use twice the required FFT size */

   /* side based normalisation */
   /* variance scaling */
   VARSCALEFN,
   /* cepstral mean subtraction */
   CMEANDIR,     /* dir to find the means */
   CMEANMASK,    /* label mask to idenitfy mean file */
   CMEANPATHMASK,/* label mask to idenitfy the path of mean file */
   VARSCALEDIR,  /* dir to find the variance estimate files */
   VARSCALEMASK, /* label mask to idenitfy the variance estimate files */
   VARSCALEPATHMASK, /* label mask to idenitfy the path of the variance estimate files */
   SIDEXFORMMASK,/* mask for use with side-based xforms */
   SIDEXFORMEXT, /* extension for use with side-based xforms */

   /* MatTran file */
   MATTRANFN,     /* File name for MatTran file */
   MATTRAN,

   /* Extended Deltas */
   THIRDWINDOW,
   FOURTHWINDOW,
   CFGSIZE
}IOConfParm;

static char * ioConfName[CFGSIZE] = {
   "SOURCEKIND", "SOURCEFORMAT", "SOURCERATE", "ZMEANSOURCE", 
   "TARGETKIND", "TARGETFORMAT", "TARGETRATE", 
   "SAVECOMPRESSED", "SAVEWITHCRC",
   "WINDOWSIZE", "USEHAMMING", "PREEMCOEF", 
   "USEPOWER", "NUMCHANS", "LOFREQ", "HIFREQ",
   "WARPFREQ", "WARPLCUTOFF", "WARPUCUTOFF",
   "LPCORDER",  "COMPRESSFACT",
   "CEPLIFTER", "NUMCEPS", "CEPSCALE",
   "RAWENERGY","ENORMALISE", "ESCALE", "SILFLOOR",
   "DELTAWINDOW", "ACCWINDOW", "SIMPLEDIFFS",
   "USESILDET", "SELFCALSILDET", "SPEECHTHRESH", "SILDISCARD", "SILENERGY", 
   "SPCSEQCOUNT", "SPCGLCHCOUNT", "SILGLCHCOUNT", "SILSEQCOUNT", "SILMARGIN", 
   "MEASURESIL", "OUTSILWARN"
   ,"AUDIOSIG", "V1COMPAT", "VQTABLE"
   ,"ADDDITHER",
   "DOUBLEFFT",
   "VARSCALEFN", 
   "CMEANDIR" , "CMEANMASK", "CMEANPATHMASK",
   "VARSCALEDIR", "VARSCALEMASK" , "VARSCALEPATHMASK" , "SIDEXFORMMASK", "SIDEXFORMEXT",
   "MATTRANFN", "MATTRAN", "THIRDWINDOW", "FOURTHWINDOW"
};

/* -------------------  Default Configuration Values ---------------------- */

static const IOConfigRec defConf = {
   ANON, HTK, 0.0, FALSE, /* SOURCEKIND SOURCEFORMAT SOURCERATE ZMEANSOURCE */
   ANON, HTK, 0.0,        /* TARGETKIND TARGETFORMAT TARGETRATE */
   FALSE, TRUE,           /* SAVECOMPRESSED SAVEWITHCRC */
   256000.0, TRUE, 0.97,  /* WINDOWSIZE USEHAMMING PREEMCOEF */
   FALSE, 20, -1.0, -1.0, /* USEPOWER NUMCHANS LOFREQ HIFREQ */
   1.0,                   /* WARPFREQ */
   0.0, 0.0,              /* WARPLCUTOFF WARPUCUTOFF */
   12, 0.33,              /* LPCORDER COMPRESSFACT */
   22, 12, 1.0,           /* CEPLIFTER NUMCEPS CEPSCALE */
   TRUE, TRUE, 0.1, 50.0, /* RAWENERGY ENORMALISE ESCALE SILFLOOR */
   2, 2, FALSE,           /* DELTAWINDOW ACCWINDOW SIMPLEDIFFS */
   FALSE,0,               /* USESILDET SELFCALSILDET */
   9.0,0.0,0.0,           /* SPEECHTHRESH SILDISCARD SILENERGY */
   10,0,2,                /* SPCSEQCOUNT SPCGLCHCOUNT SILGLCHCOUNT */
   100,40,                /* SILSEQCOUNT SILMARGIN */
   TRUE,TRUE,             /* MEASURESIL OUTSILWARN */
   NULLSIG,               /* AUDIOSIG */
   FALSE,NULL,            /* V1COMPAT VQTABLE */
   0.0,                   /* ADDDITHER */
   FALSE,                 /* DOUBLEFFT */
   /* side based normalisation */
   NULL,                  /* VARSCALEFN */
   NULL,NULL,NULL,        /* CMEANDIR CMEANMASK CMEANPATHMASK */
   NULL,NULL,NULL,        /* VARSCALEDIR VARSCALEMASK VARSCALEPATHMASK */

   NULL,NULL,             /* SIDEXFORMMASK SIDEXFORMEXT*/

   NULL,                  /* vqTab */
   NULL, NULL, 2, 2      /* MATTRANFN, MATTRAN THIRDWIN FOURTHWIN */
};

/* ------------------------- Buffer Definition  ------------------------*/

/* Cepstral Mean Record for running average */
typedef struct meanrec 
{
   int frames;            /* Number of frames processed in session */
   Vector defMeanVec;     /* Default mean vector for reset */
   Vector curMeanVec;     /* Current mean */
}
MeanRec;

/* HParm can deal with multiple channels (eg Audio*N/Files/RFE) */
/*  Each channel can have its own setup and preserved information */
typedef struct channelinfo {
   char *confName;        /* Configuration name associated with mean */
   int fCnt;              /* Number of files processed for this channel */
   int sCnt;              /* Number of files processed in current session */
   int oCnt;              /* Number of observations processed in session */
   Boolean spDetParmsSet; /* Speech detector parameters set */
   float frMin;           /* Measured minimum frame energy for channel (dB) */
   float spDetSil;        /* Measured/set silence level for channel (dB) */
   float spDetThresh;     /* Measured/set speech/silence threshold (dB) */
   float spDetSp;         /* Measured/set speech level for channel (dB) */
   float frMax;           /* Measured maximum frame energy for channel (dB) */
   float chPeak;          /* Scaled peak-to-peak range 0.0-1.0 */
   float chOffset;        /* Average sample offset (-32768..32767) */
   float spDetSNR;        /* Measured/set silence/speech ratio (dB) */
   IOConfigRec cf;        /* Channel configuration */
   struct channelinfo *next;  /* Next channel record */
}
ChannelInfo;

typedef struct hparmsrcdef {
   Ptr xInfo;         /* Application data */
   ParmKind pk;       /* Type of source - split into parmKind and */
   int size;          /* Sample size fields */
   HTime sampPeriod;  /* Either 0.0 or the fixed sample rate of source */

   Ptr (*fOpen)(Ptr xInfo,char *fn,BufferInfo *info);  /* Open new buffer */
   /* 
      Return: Pointer to buffer specific data

      Connect to source and allocate necessary structures.
      Each buffer is associated with a specific pointer that is assigned
      to the return value of this function.  All other buffer operations
      are passed this pointer.  Typically it will be used to access a
      source specific data structure containing the necessary information
      for operating the source.
   */
   
   void (*fClose)(Ptr xInfo,Ptr bInfo);  /* Close buffer and free resources */
   /* 
      Ptr bInfo: Pointer returned by fOpen for this buffer

      Free all the resources associated with the buffer (including if 
      necessary the info block itself).
   */

   void (*fStart)(Ptr xInfo,Ptr bInfo);  /* Start data capture for real-time sources */
   /* 
      Ptr bInfo: Pointer returned by fOpen for this buffer

      Start data capture.  Offline sources can ignore this call.
   */

   void (*fStop)(Ptr xInfo,Ptr bInfo);  /* Stop data capture for real-time sources */
   /* 
      Ptr bInfo: Pointer returned by fOpen for this buffer

      Stop data capture.  Offline sources can ignore this call.
   */

   int (*fNumSamp)(Ptr xInfo,Ptr bInfo);  /* Query samples readable without blocking */
   /* 
      Ptr bInfo: Pointer returned by fOpen for this buffer
      Return:   Samples readable without blocking

      Used to determine size of next read.  Offline sources can specify the
      whole utterance whereas real-time sources should return the number of
      buffered data samples once data capture has finished or -1 minus the
      number of samples that can be read without blocking.
   */

   int (*fGetData)(Ptr xInfo,Ptr bInfo,int n,Ptr data);  /* Read samples */
   /* 
      Ptr bInfo: Pointer returned by fOpen for this buffer
      int n:    Number of samples required
      Ptr data: Buffer for returned samples
      Return:   Samples read correctly

      Read samples from the source.
      In general will only read one frame at a time (either frSize samples
      for the first frame or frRate samples for the rest).
      Will only request a frame that fNumSamp indicates will block when the
      next thing to do is process the frame.  Normally only non-blocking
      data will be requested (unless the decoder is keeping up with the
      source).
   */
} HParmSrcDefRec;

typedef enum channeltype {
   /*
     table=1,
     buffer=2,
   */
   ch_haudio=4,   /* The HAudio interface */
   ch_hwave,      /* A waveform file */
   ch_hparm,      /* A parmeterised file */
   ch_hrfe,       /* The RFE is not yet reimplemented */
   ch_ext_wave,   /* Externally defined waveform source */
   ch_ext_parm    /* Externally defined parameterised source */
}
ChannelType;

#define MIN_PB_SIZE 64
#define MAX_PB_SIZE 2048
#define MAX_INT 536870911 /* Don't use INT_MAX cos get numeric overflow */

typedef struct pblock {
   int stRow;        /* absolute number of first row in this block */
   int nRows;        /* number of rows used in this block */
   int maxRows;      /* total number of rows in this block */
   void *data;       /* parameterised data for this block */
   struct pblock *next; /* Next block */
}
PBlock;

typedef struct _ParmBuf {
   MemHeap *mem;       /* Memory heap for this parm buf */
   PBStatus status;    /* status of this buffer */
   ChannelInfo *chan;  /* input channel for this buffer */
   IOConfig cf;        /* configuration for this channel */
   Boolean noTable;    /* no need for table access */
   ChannelType chType; /* type of input channel */
   Boolean chClear;    /* End of channel reached */
   Boolean dShort;     /* data is array of shorts not floats (DISCRETE) */
   Boolean fShort;     /* file is array of shorts (DISCRETE, COMPX or IREFC) */

   /* New parameters for channel type buffer */
   HParmSrcDef ext;     /* external source functions */
   union {
      AudioIn a;        /* the audio source */
      Wave w;           /* the waveform file */
      Ptr i;            /* data for external source */
   }
   in;
   unsigned short crcc;/* Put crcc here when we read it !! */

   /*  Channel buffer consists of a main active (for inwards reading, sil */
   /*  detection and qualification) block plus preceding blocks that form */
   /*  an infinitely extensible read only buffer */
   PBlock main;        /* Main block of data (next points to first block) */
   int inRow;          /* Absolute row number of next to read (nRows+stRow) */
   int outRow;         /*   of next row to return (may be in any block) */
   int lastRow;        /*   of final row (if we know it) */
   /* Main buffer parameters */
   int qst;            /* next row in main block to qualify (qst>qwin) */
   int qen;            /* final row in main block qualified (last valid row) */
   int qwin;           /* Width of qualify window (needed on each side) */
   /* Silence detector parameters and results */
   int minRows;        /* min rows to keep in main block */
   float *spVal;       /* Array of speech/silence levels */
   int spDetLst;       /* Last frame of speech seen */
   int spDetCur;       /* Current speech detector frame */
   int spDetCnt;       /* Number of speech frames in window */
   int silDetCnt;      /* Number of silence frames in window */
   int spDetSt;        /* first row to return (MAX_INT == waiting) */
   int spDetEn;        /* row after last to return (MAX_INT == waiting) */
   int spDetFin;       /* final row allowed to return (normally qen) */
}ParmBufRec;

/* ----------------------------- Local Memory  --------------------------*/
   
static ConfParam *cParm[MAXGLOBS];      /* config parameters */
static int nParm = 0;

static MemHeap parmHeap;                /* HParm no longer uses gstack */

static Boolean hparmBin=TRUE; /* HTK format files are binary */

static ChannelInfo *defChan=NULL;
static ChannelInfo *curChan=NULL;

/* ----------------------- IO Configuration Handling ------------------ */

/* Load the global variance vector for side based CVN */
static void LoadVarScale (MemHeap *x, IOConfig cf)
{
   Source varsrc;
   char buf[MAXSTRLEN];
   Boolean vbinary=FALSE;
   int dim,i;

   Matrix GlobalVar,NewGlobalVar;
   int NewFDim,FDim;

   if (strcmp (cf->varScaleFN, varScaleFN) == 0) { /* already cached */
      cf->varScale = CreateVector (x, varScaleDim);
      for (i=1; i<=varScaleDim; i++)
         cf->varScale[i] = varScale[i];
   }
   else {  /* read it in */
      if (InitSource (cf->varScaleFN, &varsrc, NoFilter) < SUCCESS)
         HError (6310, "LoadVarScale: Can't open varscale file %s", cf->varScaleFN);
      SkipComment (&varsrc);
      ReadString (&varsrc,buf);
      if  (strcmp (buf, "<VARSCALE>") != 0)
         HError (6376, "LoadVarScale: <VARSCALE> missing, read: %s", buf);
      ReadInt (&varsrc, &dim, 1, vbinary);      
      cf->varScale = CreateVector (x, dim);
      if (!ReadVector (&varsrc, cf->varScale, vbinary))
         HError(6376 ,"LoadVarScale: Couldn't read var scale vector from file");
      CloseSource (&varsrc);
      
      /* Apply a linear transform to the global variance */
      if ((cf->MatTran != NULL) &&  (UseOldXFormCVN)) {

         FDim = NumCols(cf->MatTran);
         NewFDim = NumRows(cf->MatTran);

         NewGlobalVar = CreateMatrix(x,NewFDim,NewFDim);
         ZeroMatrix(NewGlobalVar);
         GlobalVar = CreateMatrix(x,FDim,FDim);
         ZeroMatrix(GlobalVar);

         for (i=1;i<=NumRows(GlobalVar);i++){
            GlobalVar[i][i] = cf->varScale[i];
         }         

         LinTranQuaProd(NewGlobalVar,cf->MatTran,GlobalVar);

         cf->varScale = CreateVector(x,NewFDim);
         ZeroVector(cf->varScale); 

         for (i=1;i<=NewFDim;i++){
            cf->varScale[i] = NewGlobalVar[i][i];
         }
         
         dim = NewFDim;             
      }
  
      for (i=1; i<=dim; i++){                    /* cache the vector */
         varScale[i] = cf->varScale[i];
      }
      varScaleDim = dim;
      strcpy (varScaleFN, cf->varScaleFN);
   }
}


/*  
   After the global feature transform is loaded as a macro via HModel, if the 
   channel feature transform config is empty then this function is invoked in 
   LoadMat to pass on all the channel config setup from the loaded input 
   linear transform data structure.
*/
static void SetInputXFormConfig(IOConfig cf, InputXForm *xf)
{
   LinXForm *xform;

   xform = xf->xform;
   cf->xform = xf;
   if (IntVecSize(xform->blockSize) != 1)
      HError(999,"Only full linear transforms currently supported");
   cf->matPK = xf->pkind;
   cf->preQual = xf->preQual;
   /* Currently hard-wired and ignored */
   cf->preFrames = 0;
   cf->postFrames = 0;
   cf->MatTran = xform->xform[1];
   if (cf->MatTranFN == NULL) /* set to non-NULL */
      cf->MatTranFN = xf->xformName;
   if (((cf->preFrames>0) || (cf->postFrames>0)) && (HasZerom(cf->tgtPK))) 
      HError(-1,"Mismatch possible for ZeroMean due to end truncations.\n All static parameters floored (including Energy) as using matrix transformation.\n For post transformation dynamic parameters also floored.");
}

/* EXPORT->SetParmHMMSet: specifies the HMMSet to be used with the frontend */
void SetParmHMMSet(Ptr aset)
{
   char buf[MAXSTRLEN];
   InputXForm *cfg_xf, *hmm_xf;
   LabId id;

   hset = (HMMSet *)aset;
   hmm_xf = hset->xf;
   if (defChan != NULL) { /* xforms may already be set using config files */
      cfg_xf = defChan->cf.xform;
      if (cfg_xf != NULL) { /* is there a transform currently set */
         if (hmm_xf != NULL) { 
            /* 
               need to check that the transforms are the same. This may be achieved
               by ensuring that the transforms have the same macroname.
            */
            if (strcmp(hmm_xf->xformName,cfg_xf->xformName))
               HError(6396,"Incompatible XForm macros in MMF and config file %s and %s",
                      hmm_xf->xformName,cfg_xf->xformName);     
            else if (cfg_xf != hmm_xf)
               HRError(6396,"Assumed compatible XForm macro %s in files %s and %s",
                       hmm_xf->xformName,hmm_xf->fname,cfg_xf->fname);  
         } else {
            /* 
               there is a config transform specified, so check that it is compatble 
               with the model set MMFId
            */
            if ((hset->hmmSetId != NULL) && (!MaskMatch(cfg_xf->mmfIdMask,buf,hset->hmmSetId)))
               HError(6396,"HMM Set %s is not compatible with InputXForm",hset->hmmSetId);       
            /*
              also need to ensure that the transform from the cofig option is converted
              into a macro.
            */
            id = GetLabId(cfg_xf->xformName,TRUE);
            NewMacro(hset,-2,'j',id,(Ptr)cfg_xf);
            cfg_xf->nUse++;
            hset->xf = cfg_xf;
         }
      } else { 
         /* 
            transform needs to be set-up from the model set. 
            Also need to reload the variance floor.
         */
         if (hmm_xf != NULL) {
            SetInputXFormConfig(&(defChan->cf),hmm_xf);
            if ((hset->hmmSetId != NULL) && (!MaskMatch(hmm_xf->mmfIdMask,buf,hset->hmmSetId)))
               HError(6396,"HMM Set %s is not compatible with InputXForm",hset->hmmSetId);       
         }
         if (defChan->cf.varScaleFN != NULL){
            strcpy (varScaleFN,"\0"); 
            LoadVarScale(&gcheap,&(defChan->cf)); 
         }    
      }
   } else { /* somehow this is happening prior to InitParm ... */
      HError(6396,"Calling SetParmHMMSet prior to InitParm");
   }
}

/* 
   Load a global feature transform given in the channel config setup via a function
   defined in HModel since the transform is also treated as a macro. Afterwards check 
   the consistency between the transform's model id. If the channel global feature 
   transform setup is empty then pass on all the information from the loaded transform
*/
static void LoadMat (MemHeap *x, IOConfig cf)  /*static??*/
{
   InputXForm *xf;
   char macroname[MAXSTRLEN];

   xf = LoadInputXForm(hset,NameOf(cf->MatTranFN,macroname),cf->MatTranFN);
   if (xf == NULL)
      HError(999,"Cannot correctly load input transform from file %s",cf->MatTranFN);
   if (cf->xform == NULL) { /* No transform from model set */
      SetInputXFormConfig(cf,xf);
   } else { /* check that transform is the same as the model one */
      if (strcmp(xf->xformName,cf->xform->xformName)) {
         HRError(999,"Possibly incompatible XForm macros in MMF and config file %s and %s (SPR?)",
                 xf->xformName,cf->xform->xformName);   
         SetInputXFormConfig(cf,xf);
      }    
   }
}

static AdaptXForm *LoadSideXForm(IOConfig cf, char *fname) 
{
   AdaptXForm *xf;
   char macroname[MAXSTRLEN];
   char side[MAXSTRLEN];
   Boolean maskMatch;

   maskMatch = MaskMatch(cf->sideXFormMask,side, fname);
   if ((!maskMatch) && (fname != NULL))
      HError(999,"Side xform mask %s does not match filename %s",cf->sideXFormMask,fname);
   MakeFN(side,NULL,cf->sideXFormExt,macroname);
   xf = LoadOneXForm(hset,macroname,NULL);
   if (xf == NULL)
      HError(999,"Cannot correctly load side transform %s",macroname);

   /* Check that this is a valid side XForm */
   if (xf->bclass->numClasses != 1) HError(999,"Can only use global bseclasses for sideXforms");
   if (xf->parentXForm != NULL) HError(999,"Cannot have parent xforms with sideXforms");
   if (xf->xformSet->xkind != CMLLR) HError(999,"Can only use CMLLR as sideXforms");

   return xf;
}

static void ApplyXForm2Vector(LinXForm *linXForm, Vector mean)
{  
   Vector vec, bias;
   int size,b,bsize;
   Matrix A;
   float tmp;
   int i,j;
   int cnt,cnti,cntj;

   /* Check dimensions */
   size = linXForm->vecSize;
   if (size != VectorSize(mean))
      HError(999,"Transform dimension (%d) does not match mean dimension (%d)",
             size,VectorSize(mean));
   vec = CreateVector(&gstack,size);
   CopyVector(mean,vec); ZeroVector(mean);
   /* Transform mean */
   for (b=1,cnti=1,cnt=1;b<=IntVecSize(linXForm->blockSize);b++) {
      bsize = linXForm->blockSize[b];
      A = linXForm->xform[b];
      for (i=1;i<=bsize;i++,cnti++) {
         tmp = 0;
         for (j=1,cntj=cnt;j<=bsize;j++,cntj++)
            tmp += A[i][j] * vec[cntj];
         mean[cnti] = tmp;
      }
      cnt += bsize;
   }
   /* Apply bias if required */
   bias = linXForm->bias;
   if (bias != NULL) {
      for (i=1;i<=size;i++)
         mean[i] += bias[i];
   }
   FreeVector(&gstack,vec);
}

/* 
   Rather than put this all in InitParm and in InitChannel
   we abstract it into a separate function.
*/

char *GS(char *s){static char b[MAXFNAMELEN]; GetConfStr(cParm,nParm,s,b); return b;}
int     GI(char *s){int i;     GetConfInt(cParm,nParm,s,&i); return i;}
double  GF(char *s){double d;  GetConfFlt(cParm,nParm,s,&d); return d;}
Boolean GB(char *s){Boolean b; GetConfBool(cParm,nParm,s,&b); return b;}

/* ReadIOConfig: Create an IOConfig object.  Initial values are copied
   from defCon and then updated from configuration parameters. */
static IOConfig ReadIOConfig(IOConfig p)
{
   IOConfParm i;
   char *s;
   
   for (i=SOURCEKIND; i<CFGSIZE; i=(IOConfParm) (i+1)){
      s = ioConfName[i];
      if (HasConfParm(cParm,nParm,s))
         switch (i) {
         case SOURCEKIND:     p->srcPK = Str2ParmKind(GS(s)); break;          
         case SOURCEFORMAT:   p->srcFF = Str2Format(GS(s)); break;
         case SOURCERATE:     p->srcSampRate = GF(s); break;
         case ZMEANSOURCE:    p->zMeanSrc = GB(s); break;
         case TARGETKIND:     p->tgtPK = Str2ParmKind(GS(s)); break;
         case TARGETFORMAT:   p->tgtFF = Str2Format(GS(s)); break;
         case TARGETRATE:     p->tgtSampRate = GF(s); break;
         case SAVECOMPRESSED: p->saveCompressed = GB(s); break;
         case SAVEWITHCRC:    p->saveWithCRC = GB(s); break;
         case WINDOWSIZE:     p->winDur = GF(s); break;
         case USEHAMMING:     p->useHam = GB(s); break;
         case PREEMCOEF:      p->preEmph = GF(s); break;
         case USEPOWER:       p->usePower = GB(s); break;
         case NUMCHANS:       p->numChans = GI(s); break;
         case CEPSCALE:       p->cepScale = GF(s); break;
         case LOFREQ:         p->loFBankFreq = GF(s); break;
         case HIFREQ:         p->hiFBankFreq = GF(s); break;
         case WARPFREQ:       p->warpFreq = GF(s); break;
         case WARPLCUTOFF:    p->warpLowerCutOff = GF(s); break;
         case WARPUCUTOFF:    p->warpUpperCutOff = GF(s); break;
         case LPCORDER:       p->lpcOrder = GI(s); break;
         case COMPRESSFACT:   p->compressFact = GF(s); break;
         case CEPLIFTER:      p->cepLifter= GI(s); break;
         case NUMCEPS:        p->numCepCoef = GI(s); break;
         case RAWENERGY:      p->rawEnergy = GB(s); break;
         case ENORMALISE:     p->eNormalise = GB(s); break;
         case ESCALE:         p->eScale = GF(s); break;
         case SILFLOOR:       p->silFloor = GF(s); break;
         case DELTAWINDOW:    p->delWin = GI(s); break;
         case ACCWINDOW:      p->accWin = GI(s); break;
         case SIMPLEDIFFS:    p->simpleDiffs = GB(s); break;
         case USESILDET:      p->useSilDet = GB(s); break;
         case SELFCALSILDET:  p->selfCalSilDet = GI(s); break;
         case SPEECHTHRESH:   p->spThresh = GF(s); break;
         case SILDISCARD:     p->silDiscard = GF(s); break;
         case SILENERGY:      p->silMean = GF(s); break;
         case SPCSEQCOUNT:    p->spcSeqCount = GI(s); break;
         case SPCGLCHCOUNT:   p->spcGlchCount = GI(s); break;
         case SILGLCHCOUNT:   p->silGlchCount = GI(s); break;
         case SILSEQCOUNT:    p->silSeqCount = GI(s); break;
         case SILMARGIN:      p->marginCount = GI(s); break;
         case MEASURESIL:     p->measureSil = GB(s); break;
         case OUTSILWARN:     p->outSilWarn = GB(s); break;
         case AUDIOSIG:       p->audSignal = GI(s); break;
         case V1COMPAT:       p->v1Compat = GB(s); break;
         case VQTABLE:        p->vqTabFN = CopyString(&gcheap,GS(s)); break;
         case ADDDITHER:      p->addDither = GF(s); break;
         case DOUBLEFFT:      p->doubleFFT = GB(s); break;
           /* side based normalisation */
         case VARSCALEFN:     p->varScaleFN= CopyString(&gcheap, GS(s)); 
                              break;
         case VARSCALEDIR:    p->varScaleDN = CopyString(&gcheap,GS(s)); break;
         case VARSCALEMASK:   p->varScaleMask = CopyString(&gcheap,GS(s)); break;
         case VARSCALEPATHMASK:   p->varScalePathMask = CopyString(&gcheap,GS(s)); break;
         case CMEANDIR:       p->cMeanDN = CopyString(&gcheap,GS(s)); break;
         case CMEANMASK:      p->cMeanMask = CopyString(&gcheap,GS(s)); break;
         case CMEANPATHMASK:  p->cMeanPathMask = CopyString(&gcheap,GS(s)); break;
         case SIDEXFORMMASK:  p->sideXFormMask = CopyString(&gcheap,GS(s)); break;
         case SIDEXFORMEXT:   p->sideXFormExt = CopyString(&gcheap,GS(s)); break;
         case MATTRANFN:      p->MatTranFN= CopyString(&gcheap, GS(s)); break;

         case THIRDWINDOW:    p->thirdWin = GI(s); break;
         case FOURTHWINDOW:   p->fourthWin = GI(s); break;
         }
   }
   
   if (p->MatTranFN != NULL){
      LoadMat (&gcheap,p);
   }
   
   if (p->varScaleFN != NULL){
      LoadVarScale(&gcheap,p);
   }    

   return p;
}


/* Read channel files once only */
static ReturnStatus ReadChanFiles(ChannelInfo *chan)
{
   /* Load VQ table if needed */
   if (chan->cf.tgtPK&HASVQ) {
      if (chan->cf.vqTabFN==NULL){
         HRError(6350,"ReadChanFiles: No VQ Table Given in Configuration");
         return(FAIL);
      }
      if (trace&T_TOP) 
         printf("HParm: Loading VQ table %s\n",chan->cf.vqTabFN);
      chan->cf.vqTab = LoadVQTab(chan->cf.vqTabFN, chan->cf.tgtPK&(~HASVQ));
   }
   else chan->cf.vqTabFN=NULL,chan->cf.vqTab=NULL;
   return(SUCCESS);
}
   
/* EXPORT->InitParm: initialise memory and configuration parameters */
ReturnStatus InitParm(void)
{
   Boolean b;
   int i;
   char buf[MAXSTRLEN];

   CreateHeap(&parmHeap, "HPARM C Heap",  MSTAK, 1, 1.0, 20000, 80000 );

   Register(hparm_version,hparm_vc_id);
   nParm = GetConfig("HPARM", TRUE, cParm, MAXGLOBS);
   if (nParm>0){
      if (GetConfInt(cParm,nParm,"TRACE",&i)) trace = i;
      if (GetConfBool(cParm,nParm,"NATURALWRITEORDER",&b)) natWriteOrder = b;
      if (GetConfBool(cParm,nParm,"HIGHDIFF",&b)) highDiff = b;
      if (GetConfBool(cParm,nParm,"USEOLDXFORMCVN",&b)) UseOldXFormCVN = b;
      if (GetConfStr(cParm,nParm,"FORCEPKIND",buf))
         ForcePKind = Str2ParmKind(buf);      
   }

   defChan=curChan= (ChannelInfo *) New(&gcheap,sizeof(ChannelInfo));
   defChan->confName=CopyString(&gcheap,"HPARM");
   defChan->fCnt=defChan->sCnt=defChan->oCnt=0;
   defChan->chOffset=defChan->chPeak=-1.0;
   defChan->spDetThresh=defChan->spDetSil=defChan->spDetSNR=-1.0;
   defChan->spDetSp=0.0;
   defChan->spDetParmsSet=FALSE;
   defChan->next=NULL;
   /* Set up configuration parameters - once only now */
   defChan->cf=defConf;
   ReadIOConfig(&defChan->cf);
   if(ReadChanFiles(defChan)<SUCCESS){
      HRError(6350,"InitParm: ReadChanFiles failed");
      return(FAIL);
   }
   return(SUCCESS);
}
 
/* EXPORT->SetNewConfig: Sets config parms for libmod */
ReturnStatus SetChannel(char *confName)
{ 
   char buf[MAXSTRLEN],*in,*out;
   Boolean b;

   if (confName==NULL) {
      curChan=defChan;
       buf[0] = '\0';
   } else {
      for (in=confName,out=buf;*in!=0;in++,out++) *out=toupper(*in);
      *out=0;
      for(curChan=defChan;curChan!=NULL;curChan=curChan->next) 
         if (strcmp(curChan->confName,buf)==0) break;
   }

   if (curChan==NULL) {
      curChan= (ChannelInfo *) New(&gcheap,sizeof(ChannelInfo));
      curChan->confName=CopyString(&gcheap,buf);
      curChan->fCnt=curChan->sCnt=0;
      curChan->chOffset=curChan->chPeak=-1.0;
      curChan->spDetThresh=curChan->spDetSil=curChan->spDetSNR=-1.0;
      curChan->spDetParmsSet=FALSE;
      curChan->next=NULL;
      /* Set up configuration parameters */
      nParm = GetConfig(curChan->confName, FALSE, cParm, MAXGLOBS);
      if (nParm>0){
         if (GetConfBool(cParm,nParm,"HIGHDIFF",&b)) highDiff = b;
      }
      /* Default are the standard HPARM parameters */
      curChan->cf=defChan->cf;
      /* Need to reset the transforms so that the alignment channel assumes nothing */
      (curChan->cf).MatTranFN = NULL;
      ReadIOConfig(&curChan->cf);
      if (((curChan->cf).MatTranFN == NULL) &&  ((curChan->cf).xform != NULL))
         (curChan->cf).MatTranFN = (defChan->cf).MatTranFN;
      /* This should be after setting the model up. Set input xform if HPARM1 is being used */
      if ((hset->xf == NULL) && (strcmp("HPARM1",buf)==0)) {
         hset->xf = (curChan->cf).xform;
      } else {
         /* commented out so that stored in header, rather than separately */
         /*  if (hset->xf != NULL) hset->xf->nUse++; */
      }
      if(ReadChanFiles(curChan)<SUCCESS){
         HRError(6350,"SetChannel: ReadChanFiles for new channel failed");
         return(FAIL);
      }
      /* Link new channel into list */
      curChan->next=defChan->next;
      defChan->next=curChan;
      /* Revert to normal config parameters */
      nParm = GetConfig("HPARM", TRUE, cParm, MAXGLOBS);
    }
   return(SUCCESS);
}


/* EXPORT->ResetChannelSession: Force reinitialisation of running cepMean */
void ResetChannelSession(char *confName)
{
   ChannelInfo *chan;

   if (confName==NULL) chan=defChan;
   else
      for (chan=defChan;chan!=NULL;chan=chan->next) 
         if (strcmp(chan->confName,confName)==0) break;
   if (chan==NULL) chan=defChan;
   chan->sCnt=chan->oCnt=0;
   /* Recalibrate selfCalSilDet every session */
   if (chan->cf.selfCalSilDet!=0) chan->spDetParmsSet=FALSE;
}

/* Keep the next two functions solely for compatibility */
void SetNewConfig(char *confName)
{
   SetChannel(confName);
}


/* ------------------- Buffer Status Operations --------------- */

static void FillBufFromChannel(ParmBuf pbuf,int minRows);

static char * pbStatMap[] = { 
   "PB_INIT","PB_WAITING","PB_STOPPING","PB_FILLING","PB_STOPPED","PB_CLEARED" 
};

/* ChangeState: change state of buffer and trace if enabled */
static void ChangeState(ParmBuf pbuf, PBStatus newState)
{
   if (trace&T_PBS)
      printf("HParm:  %s -> %s\n",pbStatMap[pbuf->status],pbStatMap[newState]);
   pbuf->status = newState;
}

/* CheckBuffer: update status of given buffer */
static void CheckBuffer(ParmBuf pbuf)
{
   Boolean started=FALSE;  /* Speech detected */
   Boolean finished=FALSE; /* Silence detected */
   Boolean cleared=FALSE;  /* Source of data is clear */
   Boolean empty=FALSE;    /* ParmBuf is empty */
   
   if (pbuf->outRow==pbuf->spDetEn) empty=TRUE;
   if (pbuf->status>PB_INIT && pbuf->chClear) cleared=TRUE;
   if (pbuf->spDetEn<=pbuf->inRow) finished=TRUE;
   if (pbuf->spDetSt<=pbuf->inRow) started=TRUE;

   switch(pbuf->status){
   case PB_INIT:
      break;
   case PB_WAITING:
      if (started) ChangeState(pbuf,PB_STOPPING);
      else break;
   case PB_STOPPING:
      if (finished) ChangeState(pbuf,PB_STOPPED);
      break;
   case PB_FILLING:
      if (cleared) ChangeState(pbuf,PB_STOPPED);
      break;
   case PB_STOPPED:
      if (empty) {
         ChangeState(pbuf,PB_CLEARED);
         pbuf->lastRow=pbuf->outRow;
      }
      break;
   case PB_CLEARED:
      break;
   }
}

/* CheckAndFillBuffer: update status of given buffer and fill from channel */
static void CheckAndFillBuffer(ParmBuf pbuf)
{
   if (pbuf->status>PB_INIT && (pbuf->status < PB_STOPPED || 
                                pbuf->qen+pbuf->main.stRow<pbuf->spDetEn-1))
      FillBufFromChannel(pbuf,0);
   CheckBuffer(pbuf);
}

/* ------------------- Parameter Kind Conversions --------------- */

static char *pmkmap[] = {"WAVEFORM", "LPC", "LPREFC", "LPCEPSTRA", 
                         "LPDELCEP", "IREFC", 
                         "MFCC", "FBANK", "MELSPEC",
                         "USER", "DISCRETE", "PLP",
                         "ANON"};

/* EXPORT-> ParmKind2Str: convert given parm kind to string */
char *ParmKind2Str(ParmKind kind, char *buf)
{
   strcpy(buf,pmkmap[BaseParmKind(kind)]);
   if (HasEnergy(kind))    strcat(buf,"_E");
   if (HasDelta(kind))     strcat(buf,"_D");
   if (HasNulle(kind))     strcat(buf,"_N");
   if (HasAccs(kind))      strcat(buf,"_A");
   if (HasThird(kind))     strcat(buf,"_T");
   if (HasCompx(kind))     strcat(buf,"_C");
   if (HasCrcc(kind))      strcat(buf,"_K");
   if (HasZerom(kind))     strcat(buf,"_Z");
   if (HasZeroc(kind))     strcat(buf,"_0");
   if (HasVQ(kind))        strcat(buf,"_V");
   return buf;
}

/* EXPORT->Str2ParmKind: Convert string representation to ParmKind */
ParmKind Str2ParmKind(char *str)
{
   ParmKind i = -1;
   char *s,buf[255];
   Boolean hasE,hasD,hasN,hasA,hasT,hasF,hasC,hasK,hasZ,has0,hasV,found;
   int len;
   
   hasV=hasE=hasD=hasN=hasA=hasT=hasF=hasC=hasK=hasZ=has0=FALSE;
   strcpy(buf,str); len=strlen(buf);
   s=buf+len-2;
   while (len>2 && *s=='_') {
      switch(*(s+1)){
      case 'E': hasE = TRUE; break;
      case 'D': hasD = TRUE; break;
      case 'N': hasN = TRUE; break;
      case 'A': hasA = TRUE; break;
      case 'C': hasC = TRUE; break;
      case 'T': hasT = TRUE; break;
      case 'F': hasF = TRUE; break;
      case 'K': hasK = TRUE; break;
      case 'Z': hasZ = TRUE; break;
      case '0': has0 = TRUE; break;
      case 'V': hasV = TRUE; break;
      default: HError(6370,"Str2ParmKind: unknown ParmKind qualifier %s",str);
      }
      *s = '\0'; len -= 2; s -= 2;
   }
   found = FALSE;
   do {
      s=pmkmap[++i]; 
      if (strcmp(buf,s) == 0) {
         found = TRUE;
         break;
      }
   } while (strcmp("ANON",s)!=0);
   if (!found)
      return ANON;
   if (i == LPDELCEP)         /* for backward compatibility with V1.2 */
      i = LPCEPSTRA | HASDELTA;
   if (hasE) i |= HASENERGY;
   if (hasD) i |= HASDELTA;
   if (hasN) i |= HASNULLE;
   if (hasA) i |= HASACCS;
   if (hasT) i |= HASTHIRD;
   if (hasK) i |= HASCRCC;
   if (hasC) i |= HASCOMPX;
   if (hasZ) i |= HASZEROM;
   if (has0) i |= HASZEROC;
   if (hasV) i |= HASVQ;
   if(trace&T_BUF){
      fprintf(stdout,"ParmKind is %d\n",i);
      fflush(stdout);
   }
   return i;
}

/* EXPORT->BaseParmKind: return the basic sample kind without qualifiers */
ParmKind BaseParmKind(ParmKind kind) { return kind & BASEMASK; }

/* EXPORT->HasXXXX: returns true if XXXX included in ParmKind */
Boolean HasEnergy(ParmKind kind){return (kind & HASENERGY) != 0;}
Boolean HasDelta(ParmKind kind) {return (kind & HASDELTA) != 0;}
Boolean HasAccs(ParmKind kind)  {return (kind & HASACCS) != 0;}
Boolean HasThird(ParmKind kind) {return (kind & HASTHIRD) != 0;}
Boolean HasNulle(ParmKind kind) {return (kind & HASNULLE) != 0;}
Boolean HasCompx(ParmKind kind) {return (kind & HASCOMPX) != 0;}
Boolean HasCrcc(ParmKind kind)  {return (kind & HASCRCC) != 0;}
Boolean HasZerom(ParmKind kind) {return (kind & HASZEROM) != 0;}
Boolean HasZeroc(ParmKind kind) {return (kind & HASZEROC) != 0;}
Boolean HasVQ(ParmKind kind)    {return (kind & HASVQ) != 0;}

/* EXPORT->SyncBuffers: if matrix transformations are used this syncs the two buffers */
Boolean SyncBuffers(ParmBuf pbuf,ParmBuf pbuf2)
{
   int pref1=0, pref2=0, postf1=0, postf2=0;
   int preshift, postshift;
   float *fptr;

   if ((pbuf->cf->MatTranFN == NULL) && (pbuf2->cf->MatTranFN == NULL))
      return(TRUE);

   if (pbuf->cf->MatTranFN != NULL) {
      pref1 = pbuf->cf->preFrames; 
      postf1 = pbuf->cf->postFrames; 
   }
   if (pbuf2->cf->MatTranFN != NULL) {
      pref2 = pbuf2->cf->preFrames; 
      postf2 = pbuf2->cf->postFrames; 
   }
   preshift = pref1-pref2;
   postshift = postf1-postf2;

   if ((preshift == 0) && (postshift == 0))
      return(TRUE);

   if (preshift>0) { /* need to offset the start of buffer2 */
      fptr = pbuf2->main.data;
      fptr += (preshift * pbuf2->cf->nCols);
      pbuf2->main.data = fptr;
      pbuf2->main.nRows -= preshift;
      if (trace&T_MAT) 
         printf("HParm: Removing first %d frames from Buffer2 (%d floats)\n",preshift,(preshift * pbuf2->cf->nCols));
   } else {
      fptr = pbuf->main.data;
      fptr -= (preshift * pbuf->cf->nCols);
      pbuf->main.data = fptr;
      pbuf->main.nRows += preshift;
      if (trace&T_MAT) 
         printf("HParm: Removing first %d frames from Buffer1 (%d floats)\n",-preshift,-(preshift * pbuf->cf->nCols));
   }

   if (postshift>0) {
      pbuf2->main.nRows -= postshift;
      if (trace&T_MAT) 
         printf("HParm: Removing last %d frames from Buffer2\n",postshift);
   } else {
      pbuf->main.nRows += postshift;
      if (trace&T_MAT) 
         printf("HParm: Removing last %d frames from Buffer1\n",-postshift);
   }

   return(TRUE);
}


/* Apply the global feature transform */
static void ApplyStaticMat(IOConfig cf, float *data, Matrix trans, int vSize, int n, int step, int offset)
{
   float *fp,*fp1;
   int i,j,k,l,mrows,mcols,nframes,fsize,m;
   Vector *odata,tmp;

   mrows = NumRows(trans); mcols = NumCols(trans);
   nframes = 1 + cf->preFrames + cf->postFrames;
   fsize = cf->nUsed;
   odata = New(&gstack,nframes*sizeof(Vector));
   odata--;
   fp = data-1;
   for (i=1;i<=nframes;i++)
      odata[i] = CreateVector(&gstack,fsize);
   for (i=2;i<=nframes;i++) {
      for (j=1;j<=fsize;j++)
         odata[i][j] = fp[j];
      fp += vSize;
   }
   fp1 = data-1;

   if ((fsize*nframes) != mcols)
      HError(-1,"Incorrect number of elements (%d %d)",cf->nUsed ,mcols);
   for (i=1;i<=n-nframes+1;i++){
      tmp = odata[1];
      for (j=1;j<=nframes-1;j++)
         odata[j] = odata[j+1];
      odata[nframes] = tmp;
      for (j=1;j<=fsize;j++){
         tmp[j]=fp[j];
      }
      for (j=1;j<=mrows;j++) {
         fp++; fp1++; *fp1=0; m=0;
         for(l=1;l<=nframes;l++) {
            for (k=1;k<=fsize;k++) {
               m++;
               *fp1 += trans[j][m]*odata[l][k];
            }
         }
      }
      fp += vSize-mrows;
      fp1 += vSize-mrows;
   }
   Dispose(&gstack,odata+1);
   cf->nUsed = mrows;
}

/* ---------------- Data Sizing and Memory allocation --------------- */

/* MakeIOConfig: Create an IOConfig object.  Initial values are copied
   from defCon and then updated from configuration parameters */
static IOConfig MakeIOConfig(MemHeap *x,ChannelInfo *chan)
{
   IOConfig p;
   
   p = (IOConfig)New(x,sizeof(IOConfigRec));
   *p = chan->cf;
   return p;
}

/* SetCodeStyle: set the coding style in given cf */
static void SetCodeStyle(IOConfig cf)
{
   ParmKind tgt = cf->tgtPK&BASEMASK;
   char buf[MAXSTRLEN];
   
   switch (tgt) {
   case LPC: case LPREFC: case LPCEPSTRA:
      cf->style = LPCbased;
      break;
   case MELSPEC: case FBANK: case MFCC: case PLP:
      cf->style = FFTbased;
      break;
   case DISCRETE:
      cf->style = VQbased;
      break;
   default:
      HError(6321,"SetCodeStyle: Unknown style %s",ParmKind2Str(tgt,buf));
   }
}

/* ValidCodeParms: check to ensure reasonable wave->parm code params */
static void ValidCodeParms(IOConfig cf)
{
   int order=0;
   ParmKind btgt = cf->tgtPK&BASEMASK;
   
   if (cf->srcSampRate<=0.0 || cf->srcSampRate>10000000.0)
      HError(6371,"ValidCodeParms: src frame rate %f unlikely",cf->srcSampRate);
   if (cf->tgtSampRate<=cf->srcSampRate || cf->tgtSampRate>10000000.0)
      HError(6371,"ValidCodeParms: parm frame rate %f unlikely",cf->tgtSampRate);
   if (cf->winDur<cf->tgtSampRate || cf->winDur>cf->tgtSampRate*100.0)
      HError(6371,"ValidCodeParms: window duration %f unlikely",cf->winDur);
   if (cf->preEmph<0.0 || cf->preEmph>1.0)
      HError(6371,"ValidCodeParms: preEmph %f illegal",cf->preEmph);
   SetCodeStyle(cf);  /* in case not set yet */
   switch (cf->style){
   case LPCbased:
      order = cf->lpcOrder;
      if (order<2 || order>1000)
         HError(6371,"ValidCodeParms: unlikely lpc order %d",cf->lpcOrder);
      if (cf->tgtPK&HASZEROC)
         HError(6321,"ValidCodeParms: cannot have C0 with lpc");
      break;
   case FFTbased:
      order = cf->numChans;
      if (order<2 || order>1000)
         HError(6371,"ValidCodeParms: unlikely num channels %d",cf->numChans);
      if (cf->loFBankFreq > cf->hiFBankFreq || 
          cf->hiFBankFreq > 0.5E7/cf->srcSampRate)
         HError(6371,"ValidCodeParms: bad band-pass filter freqs %.1f .. %.1f",
                cf->loFBankFreq,cf->hiFBankFreq);
      if (btgt == PLP) {
         order = cf->lpcOrder;
         if (order < 2 || order > 1000)
            HError(6371,"ValidCodeParms: unlikely lpc order %d",cf->lpcOrder);
         if (!cf->usePower)
            HError(-6371,"ValidCodeParms: Using linear spectrum with PLP");
         if (cf->compressFact >= 1.0 || cf->compressFact <= 0.0)
            HError(6371,"ValidCodeParms: Compression factor (%f) should have a value between 0 and 1\n",cf->compressFact);
      }
      break;
   default: break;
   }
   if (btgt == LPCEPSTRA || btgt == MFCC || btgt == PLP){
      if (cf->numCepCoef < 2 || cf->numCepCoef > order)
         HError(6371,"ValidCodeParms: unlikely num cep coef %d",cf->numCepCoef);
      if (cf->cepLifter < 0 || cf->cepLifter > 1000)
         HError(6371,"ValidCodeParms: unlikely cep lifter %d",cf->cepLifter);
   }

   if (cf->warpFreq < 0.5 || cf->warpFreq > 2.0)
      HError (6371, "ValidCodeParms: unlikely warping factor %s\n", cf->warpFreq);
   if (cf->warpFreq != 1.0) {
      if (cf->warpLowerCutOff == 0.0 || cf->warpUpperCutOff == 0.0 ||
          cf->warpLowerCutOff > cf->warpUpperCutOff)
         HError (6371, "ValidCodeParms: invalid warping cut-off frequencies %f %f \n",
                 cf->warpLowerCutOff , cf->warpUpperCutOff);
      if (cf->warpUpperCutOff == 0.0 && cf->warpLowerCutOff != 0.0) {
         cf->warpUpperCutOff = cf->warpLowerCutOff;
         HError (-6371, "ValidCodeParms: setting warp cut-off frequencies to %f %f\n",
                 cf->warpLowerCutOff, cf->warpUpperCutOff);
      }
      if (cf->warpLowerCutOff == 0.0 && cf->warpUpperCutOff != 0.0) {
         cf->warpUpperCutOff = cf->warpLowerCutOff ;
         HError (-6371, "ValidCodeParms: setting warp cut-off frequencies to %f %f\n",
                 cf->warpLowerCutOff, cf->warpUpperCutOff);
      }
   }
}

/* EXPORT->ValidConversion: checks that src -> tgt conversion is possible */
Boolean ValidConversion (ParmKind src, ParmKind tgt)
{
   static short xmap[13][13] = {
      { 0, 1, 1, 1, 0, 0, 1, 1, 1, 0, 0, 0, 0},    /* src = WAVEFORM */
      { 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0},    /* src = LPC */
      { 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0},    /* src = LPREFC */
      { 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0},    /* src = LPCEPSTRA */
      { 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0},    /* src = LPDELCEP */
      { 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0},    /* src = IREFC */
      { 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0},    /* src = MFCC */
      { 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0},    /* src = FBANK */
      { 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0},    /* src = MELSPEC */
      { 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0},    /* src = USER */
      { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0},    /* src = DISCRETE */
      { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0},    /* src = PLP */
      { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},    /* src = ANON */
   };
   if (src == tgt) return TRUE;
   if (xmap[src&BASEMASK][tgt&BASEMASK] == 0 ) return FALSE;
   if ((tgt&BASEMASK) == DISCRETE){
      if ((tgt&~(BASEMASK|HASCRCC)) != 0) return FALSE;
   } else {
      if ((tgt&HASENERGY) && !(src&HASENERGY) ) 
         return FALSE;
      if ((tgt&HASZEROC)  && !(src&HASZEROC) ) 
         return FALSE;
      if ((tgt&HASENERGY) && (tgt&HASZEROC) && (tgt&HASDELTA)) 
         return FALSE;
      if ((tgt&HASENERGY) && (tgt&HASZEROC) && (tgt&HASNULLE)) 
         return FALSE;
      if (!(tgt&HASDELTA) && (tgt&HASACCS)) return FALSE;
      if ((tgt&HASNULLE) && !((tgt&HASENERGY) || (tgt&HASZEROC)) ) 
         return FALSE;
      if ((tgt&HASNULLE) && !(tgt&HASDELTA) ) return FALSE;
   }
   return TRUE;
}

/* FindSpans: Finds the positions of the subcomponents of a parameter 
   vector with size components as follows (span values are -ve if 
   subcomponent does not exist):
   span: [0] .. [1]  [2]    [3]  [4] .. [5]  [6] .. [7] [8] .. [9]
           statics   cep0 energy   deltas       accs      third   */
static void FindSpans(short span[12], ParmKind k, int size)
{
   int i,stat,en=0,del=0,acc=0,c0=0,third=0,fourth=0;
   
   for (i=0; i<10; i++) span[i] = -1;
   /* 
      if having higher order differentials and the precondition 
      is there is third oder differentials 
   */
   if (k&HASTHIRD && highDiff == TRUE)
      fourth = third = acc = del = size/5;
   else if (k&HASTHIRD)
      third = acc = del = size/4;
   else if (k&HASACCS)
      acc = del = size/3; 
   else if (k&HASDELTA) 
      del = size/2;
   if (k&HASENERGY) ++en;  if (k&HASZEROC) ++c0;
   stat = size - c0 - en - del - acc - third - fourth;
   if (stat>0) { span[0] = 0;              span[1] = stat-1;}
   if (c0>0)     span[2] = stat; if (en>0) span[3] = stat+c0;
   if (del>0)  { span[4] = stat+c0+en;     span[5] = stat+c0+en+del-1;}
   if (acc>0)  { span[6] = stat+c0+en+del; span[7] = stat+c0+en+del+acc-1;}
   if (third>0){ span[8] = stat+c0+en+del+acc; span[9] = stat+c0+en+del+acc+third-1;}
   if (fourth>0){ span[10] = stat+c0+en+del+acc+third; span[11] = stat+c0+en+del+acc+third+fourth-1;}
}

/* TotalComps: return the total number of components in a parameter vector
   with nStatic components and ParmKind pk */
static int TotalComps(int nStatic, ParmKind pk)
{
   int n,x;
   
   n = nStatic;
   if (pk&HASENERGY) ++n;
   if (pk&HASZEROC)  ++n;
   x = n;
   if (pk&HASDELTA){
      n += x;
      if (pk&HASACCS) {
        n += x;
        if (pk&HASTHIRD) {
           n += x;
           if (highDiff == TRUE){
              n += x;
           }
        }
      }
   }
   return n;   
}

/* NumStatic: return the number of static components in a parameter vector
   with nTotal components and ParmKind pk */
static int NumStatic(int nTotal, ParmKind pk)
{
   short span[12];
   
   FindSpans(span,pk,nTotal);
   return span[1]-span[0]+1;  
}

/* NumEnergy: return the number of energy components in a parameter vector
   with nTotal components and ParmKind pk */
static int NumEnergy(ParmKind pk)
{
   int e;
   
   if (!(pk&(HASENERGY|HASZEROC))) return 0;
   e = 1;
   if (pk&HASDELTA){
      ++e;
      if (pk&HASACCS) {
        ++e;
        if (pk&HASTHIRD){
           ++e;
           if (highDiff == TRUE){
              ++e;
           }
        }
      }
   }
   return e;   
}

/* EqualKind: return true if kinds are internally compatible */
static Boolean EqualKind(ParmKind a, ParmKind b)
{
   /* Energy suppression only occurs at observation level */
   a = a&(~HASNULLE); b = b&(~HASNULLE);
   return a==b;
}

/* --------------- Parameter Conversion Routines ----------------- */

/* All of the routines in this section operate on a table of floats.  
   Typically the same operation is applied to each row.  Source slice
   is indexed by si relative to start of that row, target slice is
   indexed by ti relative to start of that row


   data
   v
   ................................... ^
   si              ti             |
   .....xxxxxxxxxxx.....xxxxxxxxxxx... nRows
   <--- d --->     <--- d --->    |
   ................................... v
   <--------------nCols-------------->
*/

/* AddDiffs: target slice of each row are regression coeffs corresponding
   to source slice.  Regression is calculated over rows
   -winSize to +winSize. This assumes that hdMargin valid rows are stored
   before the parameter block, and tlMargin valid rows are stored after
   the parameter block.  When applied to tables these will be both zero.
   For buffers, they may be positive. */
      
/* Note that unlike before this function really will process nRows of data */
/* And will not automagically do extra (which was a necessary and probably */
/* unintended side effect of the way the function worked previously) */

static void AddDiffs(float *data, int nRows, int nCols, int si, int ti, int d, 
                     int winSize, int hdMargin, int tlMargin, Boolean v1Compat, Boolean simpDiffs)
{
   float *p;
   int n,offset = ti-si;
   int head,tail;

   if (hdMargin<0) hdMargin=0;
   if (hdMargin>winSize) head = 0;
   else head = winSize - hdMargin;
   if (tlMargin<0) tlMargin=0;
   if (tlMargin>winSize) tail = 0;
   else tail = winSize - (tlMargin>0?tlMargin:0);

   p = data+si; n=nRows-(head+tail);
   if (n<=0) {
      if (head>0 && tail>0) {
         /* Special case to cope with ultra short buffers */
         AddRegression(p,d,nRows,nCols,offset,winSize,
                       hdMargin,tlMargin,simpDiffs);
         head=tail=n=0; return;
      }
      else if (tail==0) head=nRows,n=0; /* Just do head rows */
      else if (head==0) tail=nRows,n=0; /* Just do tail rows */
   }

   /* Make sure have winSize before and after columns to qualify */
   if (head>0) {
      if (v1Compat)
         AddHeadRegress(p,d,head,nCols,offset,0,simpDiffs);
      else
         AddRegression(p,d,head,nCols,offset,winSize,
                       hdMargin,winSize,simpDiffs);
      p += head*nCols;
   }
   if (n>0) {
      AddRegression(p,d,n,nCols,offset,winSize,winSize,winSize,simpDiffs);
      p += n*nCols;
   }
   if (tail>0) {
      if (v1Compat)
         AddTailRegress(p,d,tail,nCols,offset,0,simpDiffs);
      else
         AddRegression(p,d,tail,nCols,offset,winSize,
                       winSize,tlMargin,simpDiffs);
   }
}

/* DeleteColumns: delete source slice (NB: nCols is not changed ) */
static void DeleteColumn(float *data, int nUsed, int si, int d)
{
   int rest;
   char *p1,*p2;
   
   p1 = (char *)data + si*sizeof(float);
   p2 = p1 + d*sizeof(float);
   rest = (nUsed - (si+d))*sizeof(float);
   memmove(p1,p2,rest);
}


/* AddQualifiers: add quals needed to get from cf->curPK to cf->tgtPK. */
/*  Ensures that nRows of data are valid and fully qualified. */
/*  This means that delta coefs may be calculated beyond this range */
/*  Values of hd/tlValid indicate that margin of static data exists at */
/*  start/end of nRows */
static void AddQualifiers(ParmBuf pbuf,float *data, int nRows, IOConfig cf, 
                          int hdValid, int tlValid)
{
   char buf[100],buff1[256],buff2[256];
   int si,ti,d=0,ds,de, i, j, step, size;
   short span[12];
   float *fp, mean, scale;
   ParmKind tgtBase;
   Vector tmp;
   LinXForm *xf;

   if (highDiff && ((cf->curPK&HASDELTA) || (cf->curPK&HASACCS) || (cf->curPK&HASTHIRD)))
       HError (6371, "AddQualifiers: HIGHDIFF=T not supported with source features that contain derivatives already");


   if ((cf->curPK == cf->tgtPK) && (cf->MatTranFN == NULL)) return;
   if (trace&T_QUA)
      printf("HParm:  adding Qualifiers to %s ...",ParmKind2Str(cf->curPK,buf));
   if (cf->MatTranFN != NULL) { /* Do the generic checks that the matrix is appropriate */
      if (((cf->matPK&BASEMASK)&(cf->curPK&BASEMASK)) != (cf->matPK&BASEMASK))
         HError(6371, "AddQualifiers: Incorrect source parameter type (%s %s)",ParmKind2Str(cf->curPK,buff1),
                ParmKind2Str(cf->matPK,buff2));
      if ((HasEnergy(cf->matPK) && !(HasEnergy(cf->curPK))) || (!(HasEnergy(cf->matPK)) && (HasEnergy(cf->curPK))) ||
          (HasZeroc(cf->matPK) && !(HasZeroc(cf->curPK))) || (HasZeroc(cf->curPK) && !(HasZeroc(cf->matPK))))
         HError(6371,"AddQualifiers: Incorrect qualifiers in parameter type (%s %s)",ParmKind2Str(cf->curPK,buff1),ParmKind2Str(cf->matPK,buff2));
   }

   if ((cf->MatTranFN != NULL) && (cf->preQual)) {
      if ((HasZerom(cf->matPK) && !(HasZerom(cf->curPK))) || (HasZerom(cf->curPK) && !(HasZerom(cf->matPK))))
         HError(6371, "AddQualifiers: Incorrect qualifiers in parameter type (%s %s)",ParmKind2Str(cf->curPK,buff1),ParmKind2Str(cf->matPK,buff2));
      ApplyStaticMat(cf,data,cf->MatTran,cf->nCols,nRows,0,0);
      pbuf->main.nRows -= (cf->postFrames + cf->preFrames);
      nRows = pbuf->main.nRows;
      cf->nSamples = pbuf->main.nRows;
   }
   if ((cf->MatTranFN != NULL) && (!cf->preQual)) { 
      tgtBase = cf->tgtPK&BASEMASK;
      if ((cf->srcPK&BASEMASK)!=tgtBase && (tgtBase==LPCEPSTRA || tgtBase==MFCC || tgtBase==PLP))
         size = TotalComps(cf->numCepCoef,cf->tgtPK);
      else 
	size = TotalComps(NumStatic(cf->srcUsed,cf->srcPK),cf->tgtPK);
      FindSpans(span,cf->tgtPK,size);    
   } else 
      FindSpans(span,cf->tgtPK,cf->tgtUsed);    

   /* Add any required difference coefficients */
   if ((cf->tgtPK&HASDELTA) && !(cf->curPK&HASDELTA)){   
      d = span[5]-span[4]+1; si = span[0]; ti = span[4];
      if (trace&T_QUA)
         printf("\nHParm:  adding %d deltas to %d rows",d,nRows);

      /* Deltas need to preceed everything a little when they can */
      if (hdValid>0) ds=pbuf->qwin-cf->delWin; else ds=0;
      if (tlValid>0) de=nRows+pbuf->qwin-cf->delWin-1; else de=nRows-1;
      AddDiffs(data+cf->nCols*ds,de-ds+1,cf->nCols,si,ti,d,cf->delWin,
               hdValid+ds,tlValid-ds,cf->v1Compat,cf->simpleDiffs);
      cf->curPK |= HASDELTA; cf->nUsed += d;
   }
   if ((cf->tgtPK&HASACCS) && !(cf->curPK&HASACCS)) {
      d = span[7]-span[6]+1; si = span[4]; ti = span[6];
      if (trace&T_QUA)
         printf("\nHParm:  adding %d accs to %d rows",d,nRows);
      /* Treatment of deltas ensures that whole margin is valid */
      /* the Accs now have to precede the thirds, as you need delts  */
      /* to calc the thirds */
      if (hdValid>0) ds=pbuf->qwin-cf->delWin-cf->accWin; else ds=0;
      if (tlValid>0) de=nRows+pbuf->qwin-cf->delWin-cf->accWin-1; else de=nRows-1;
      
      AddDiffs(data+cf->nCols*ds,de-ds+1,cf->nCols,si,ti,d,cf->accWin,
	       hdValid+ds,tlValid-ds,cf->v1Compat,cf->simpleDiffs);
      cf->curPK |= HASACCS;  cf->nUsed += d;
   }
   if ((cf->tgtPK&HASTHIRD) && !(cf->curPK&HASTHIRD)) {
     d = span[9]-span[8]+1; si = span[6]; ti = span[8];
     if (trace&T_QUA)
         printf("\nHParm:  adding %d thirds to %d rows",d,nRows);
     AddDiffs(data,nRows,cf->nCols,si,ti,d,cf->thirdWin,
                  hdValid,tlValid,cf->v1Compat,cf->simpleDiffs);
     cf->curPK |= HASTHIRD;  cf->nUsed += d;
     /* Adding fourth order differentials */
     if (highDiff == TRUE) {
       d = span[11]-span[10]+1; si = span[8]; ti = span[10];
       if (trace&T_QUA)
           printf("\nHParm:  adding %d fourths to %d rows\n",d,nRows);
       AddDiffs(data,nRows,cf->nCols,si,ti,d,cf->fourthWin,
                    hdValid,tlValid,cf->v1Compat,cf->simpleDiffs);
       cf->nUsed += d;
     }
   }

   /* Zero Mean the static coefficients if required */
   if ((cf->tgtPK&HASZEROM) && !(cf->curPK&HASZEROM)) {
      /* if a global mean vector is not available  */
      if (cf->cMeanVector ==  0) {
         if (cf->MatTranFN == NULL || (!cf->preQual)) {
            d = span[1]-span[0]+1;
            if (cf->tgtPK&HASZEROC && !(cf->curPK&HASNULLE))  /* zero mean c0 too */
               ++d;  
         } else { /* No idea where the statics are so do everything .... */
            d = span[1]-span[0]+1;
            if (cf->tgtPK&HASZEROC && !(cf->curPK&HASNULLE)) d++;
            if (cf->tgtPK&HASENERGY && !(cf->curPK&HASNULLE)) d++;
         }
         if (trace&T_QUA)
            printf("\nHParm:  zero-meaning first %d cols from %d rows",d,nRows);
         FZeroMean(data,d,nRows,cf->nCols);
         
         cf->curPK |= HASZEROM;
      }
      /* if a global cepstral mean file is available */
      else {
         /* subtract the mean vector from cf */
         d = VectorSize(cf->cMeanVector);
         step = cf->nCols;
         for ( i=0; i<d ; i++){
            /* subtract mean from i'th components */
            fp = data+i;
            mean = cf->cMeanVector[i+1];
            for (j=0;j<nRows;j++){
               *fp -= mean; fp += step;
            }
         }
         cf->curPK |= HASZEROM;
      }
   }

   if (UseOldXFormCVN){
      
      if ((cf->MatTranFN != NULL) && (!cf->preQual)) {      
         if (cf->matPK != cf->curPK) {
            /* Need to check that the correct qualifiers are used */
            HError(999,"Incorrect qualifiers in parameter type (%s %s)",
                   ParmKind2Str(cf->curPK,buff1),ParmKind2Str(cf->matPK,buff2));
         }
         ApplyStaticMat(cf,data,cf->MatTran,cf->nCols,nRows,0,0);
         pbuf->main.nRows -= (cf->postFrames + cf->preFrames);
         cf->nSamples = pbuf->main.nRows;
      }  
      /*  Scale the variances */
      if (cf->varScaleFN) {
         if ((VectorSize (cf->varScale) != cf->tgtUsed) && (highDiff != TRUE))
            HError(6376 ,"AddQualifiers: Mismatch beteen varScale (%d) and target size %d",
                   VectorSize (cf->varScale), cf->tgtUsed);
         if (trace&T_QUA)
            printf("\nHParm:  variance normalisation for %d cols from %d rows",
                   cf->tgtUsed, nRows);
         
         if (cf->varScaleVector == 0) {
            HError (6376, "AddQualifiers: no variance scaling vector found");
         }
         else {
            /* use predefined variance estimate */
            /* !!! this hack is to check if the feature transform is only to append zeros,
               we dont want to do CVN in those appended dimensions with zeros! */
            if ((cf->MatTranFN != NULL) && (NumRows(cf->MatTran) > NumCols(cf->MatTran))){
               d = NumCols(cf->MatTran);
            }
            else {
               d = VectorSize(cf->varScaleVector);
            }
            step = cf->nCols;
            for (i=0; i<d ; i++){
               scale = sqrt(cf->varScale[i+1] / cf->varScaleVector[i+1]);
               fp = data+i;
               for (j=0; j<nRows; j++) {
                  *fp *= scale;
                  fp += step;
               }
            }
         }
      }
   }
   
   else {
      /*  Scale the variances */
      if (cf->varScaleFN) {
         if (cf->varScaleVector == 0) {
            HError (6376, "AddQualifiers: no variance scaling vector found");
         }
         d = VectorSize(cf->varScaleVector);
         if (VectorSize (cf->varScale) != d)
            HError(6376 ,"AddQualifiers: Mismatch beteen varScale (%d) and target size %d",
                   VectorSize (cf->varScale), d);
         if (trace&T_QUA)
            printf("\nHParm:  variance normalisation for %d cols from %d rows",
                   cf->tgtUsed, nRows);
         step = cf->nCols;
         for (i=0; i<d ; i++){
            scale = sqrt(cf->varScale[i+1] / cf->varScaleVector[i+1]);
            fp = data+i;
            for (j=0; j<nRows; j++) {
               *fp *= scale;
               fp += step;
            }
         }
      }

      /* Now apply any side specific xforms */
      if (cf->sideXForm != NULL) {
         xf = cf->sideXForm->xformSet->xforms[1];
         if (cf->varScaleFN) {
            if (xf->vecSize != d)
               HError(999,"Incompatible sizes %d and %d",xf->vecSize,d);
         }
         d = xf->vecSize;
         tmp = CreateVector(&gstack,d);
         step = cf->nCols; fp = data;
         for (j=0;j<nRows;j++) {
            for (i=0;i<d;i++) tmp[i+1]=*(fp+i);
            ApplyXForm2Vector(xf,tmp);
            for (i=0;i<d;i++) *(fp+i)=tmp[i+1];
            fp += step;
         }
         FreeVector(&gstack,tmp);
      }
      
      if ((cf->MatTranFN != NULL) && (!cf->preQual)) {      
         if (cf->matPK != cf->curPK) {
            /* Need to check that the correct qualifiers are used */
            HError(999,"Incorrect qualifiers in parameter type (%s %s)",
                   ParmKind2Str(cf->curPK,buff1),ParmKind2Str(cf->matPK,buff2));
         }
         ApplyStaticMat(cf,data,cf->MatTran,cf->nCols,nRows,0,0);
         pbuf->main.nRows -= (cf->postFrames + cf->preFrames);
         cf->nSamples = pbuf->main.nRows;
      }      
   }

   if (trace&T_QUA)
      printf("\nHParm:  quals added to give %s\n",
             ParmKind2Str (cf->curPK, buf));
}

/* DelQualifiers: delete quals in cf->curPK but not in cf->tgtPK.
    Conversion is applied to the single row pointed to by data. */ 
static void DelQualifiers(float *data, IOConfig cf)
{
   char buf[100];
   int si,d,used=cf->nUsed;
   short span[12];
   Boolean baseX,statX,eX,zX;
   
   statX = (cf->curPK&BASEMASK) != (cf->tgtPK&BASEMASK);
   eX = (cf->curPK&HASENERGY) && !(cf->tgtPK&HASENERGY);
   zX = (cf->curPK&HASZEROC) && !(cf->tgtPK&HASZEROC);
   baseX = statX || eX || zX;
   if (trace&T_TOP)
      printf("HParm:  deleting Qualifiers in %s ...",ParmKind2Str(cf->curPK,buf));
   FindSpans(span,cf->curPK,cf->nUsed);
   /* Remove acc coefs if not required in target or statics will change */
   if ((cf->curPK&HASACCS) && (baseX || !(cf->tgtPK&HASACCS)) ) {
      si = span[6]; d = span[7]-span[6]+1;
      if (si<0) HError(6390,"DelQualifiers: no accs to remove");
      if (trace&T_QUA)
         printf("\nHParm:  removing %d accs at col %d",d,si);
      DeleteColumn(data,cf->nUsed,si,d);
      cf->curPK &= ~HASACCS; cf->nUsed -= d;
   }
   /* Remove del coefs if not required in target or statics will change */
   if ((cf->curPK&HASDELTA) && (baseX || !(cf->tgtPK&HASDELTA)) ) {
      si = span[4]; d = span[5]-span[4]+1;
      if (si<0) HError(6390,"DelQualifiers: no deltas to remove");
      if (trace&T_QUA)
         printf("\nHParm:  removing %d deltas at col %d",d,si);
      DeleteColumn(data,cf->nUsed,si,d);
      cf->curPK &= ~HASDELTA; cf->nUsed -= d;
   }
   /* Remove energy if not required in target  */
   if (eX) {
      si = span[3];
      if (si<0) HError(6390,"DelQualifiers: no energy to remove");
      if (trace&T_QUA)
         printf("\nHParm:  removing energy at col %d",si);
      DeleteColumn(data,cf->nUsed,si,1);
      cf->curPK &= ~HASENERGY; --cf->nUsed;
   }
   /* Remove c0 if not required in target  */
   if (zX) {
      si = span[2];
      if (si<0) HError(6390,"DelQualifiers: no c0 to remove");
      if (trace&T_QUA)
         printf("\nHParm:  removing c0 at col %d",si);
      DeleteColumn(data,cf->nUsed,si,1);
      cf->curPK &= ~HASZEROC; --cf->nUsed;
   }
   if (cf->nUsed!=used && (trace&T_QUA)) printf("\n");
   if (trace&T_TOP)
      printf("HParm:  quals deleted to give %s\n",ParmKind2Str(cf->curPK,buf));
}

/* XformLPC2LPREFC: Convert Static Coefficients LPC -> LPREFC */
static void XformLPC2LPREFC(float *data,int d)
{
   Vector a,k;
   int j;
   float *p;
   
   a = CreateVector(&gstack,d);  k = CreateVector(&gstack,d);
   p = data-1;
   for (j=1; j<=d; j++) a[j] = p[j];
   LPC2RefC(a,k);
   for (j=1; j<=d; j++) p[j] = k[j];
   FreeVector(&gstack,k); FreeVector(&gstack,a); 
}

/* XformLPREFC2LPC: Convert Static Coefficients LPREFC -> LPC */
static void XformLPREFC2LPC(float *data,int d)
{
   Vector a,k;
   int j;
   float *p;
   
   a = CreateVector(&gstack,d);  k = CreateVector(&gstack,d);
   p = data-1;
   for (j=1; j<=d; j++) k[j] = p[j];
   RefC2LPC(k,a);
   for (j=1; j<=d; j++) p[j] = a[j];
   FreeVector(&gstack,k); FreeVector(&gstack,a); 
}

/* XformLPC2LPCEPSTRA: Convert Static Coefficients LPC -> LPCEPSTRA */
static void XformLPC2LPCEPSTRA(float *data,int d,int dnew,int lifter)
{
   Vector a,c;
   int j;
   float *p;
   
   if (dnew>d)
      HError(6322,"XformLPC2LPCEPSTRA: lp cep size cannot exceed lpc vec");
   a = CreateVector(&gstack,d);  c = CreateVector(&gstack,dnew);
   p = data-1;
   for (j=1; j<=d; j++) a[j] = p[j];
   LPC2Cepstrum(a,c);
   if (lifter>0)
      WeightCepstrum(c,1,dnew,lifter);
   for (j=1; j<=d; j++) p[j] = c[j];
   FreeVector(&gstack,c); FreeVector(&gstack,a); 
}

/* XformLPCEPSTRA2LPC: Convert Static Coefficients LPCEPSTRA -> LPC */
static void XformLPCEPSTRA2LPC(float *data,int d,int lifter)
{
   Vector a,c;
   int j;
   float *p;
   
   a = CreateVector(&gstack,d);  c = CreateVector(&gstack,d);
   p = data-1;
   for (j=1; j<=d; j++) c[j] = p[j];   
   if (lifter>0)
      UnWeightCepstrum(c,1,d,lifter);
   Cepstrum2LPC(c,a);
   for (j=1; j<=d; j++) p[j] = a[j];
   FreeVector(&gstack,c); FreeVector(&gstack,a); 
}

/* XformMELSPEC2FBANK: Convert Static Coefficients MELSPEC -> FBANK */
static void XformMELSPEC2FBANK(float *data,int d)
{
   Vector v;
   int j;
   float *p;
   
   v = CreateVector(&gstack,d);
   p = data-1;
   for (j=1; j<=d; j++) v[j] = p[j];
   MelSpec2FBank(v);
   for (j=1; j<=d; j++) p[j] = v[j];
   FreeVector(&gstack,v); 
}

/* XformFBANK2MELSPEC: Convert Static Coefficients FBANK -> MELSPEC */
static void XformFBANK2MELSPEC(float *data,int d)
{
   Vector v;
   int j;
   float *p;
   
   v = CreateVector(&gstack,d);
   p = data-1;
   for (j=1; j<=d; j++) v[j] = p[j];   
   FBank2MelSpec(v);
   for (j=1; j<=d; j++) p[j] = v[j];
   FreeVector(&gstack,v); 
}

/* XformFBANK2MFCC: Convert Static Coefficients FBANK -> MFCC */
static void XformFBANK2MFCC(float *data,int d,int dnew,int lifter)
{
   Vector fbank,c;
   int j;
   float *p;
   
   if (dnew>d)
      HError(6322,"XformFBANK2MFCC: mfcc size cannot exceed fbank size");
   fbank = CreateVector(&gstack,d);  c = CreateVector(&gstack,dnew);
   p = data-1;
   for (j=1; j<=d; j++) fbank[j] = p[j];
   FBank2MFCC(fbank,c,dnew);
   if (lifter>0)
      WeightCepstrum(c,1,dnew,lifter);
   for (j=1; j<=d; j++) p[j] = c[j];
   FreeVector(&gstack,c); FreeVector(&gstack,fbank); 
}

/* XformBase: convert statics to change basekind of cf->curPK to cf->tgtPK.
      Conversion is applied to a single row pointed to by data.  */ 
static void XformBase(float *data, IOConfig cf)
{
   char b1[50],b2[50];
   ParmKind curBase,tgtBase,quals;
   int d, dnew, lifter;
   short span[12];
   
   curBase = cf->curPK&BASEMASK;
   tgtBase = cf->tgtPK&BASEMASK;
   if (curBase == tgtBase) return;
   quals = cf->curPK&~BASEMASK;
   if (trace&T_TOP)
      printf("HParm: Attempting to xform static parms from %s to %s\n",
             ParmKind2Str(curBase,b1),ParmKind2Str(tgtBase,b2));
   FindSpans(span, cf->curPK, cf->nUsed);
   d = span[1]-span[0]+1; dnew = cf->numCepCoef; lifter = cf->cepLifter;
   switch (curBase) {
   case LPC:
      switch(tgtBase){
      case LPREFC:    
         XformLPC2LPREFC(data,d); 
         break;
      case LPCEPSTRA: 
         XformLPC2LPCEPSTRA(data,d,dnew,lifter);
         if (dnew<d) {
            DeleteColumn(data,cf->nUsed,dnew,d-dnew);
            cf->nUsed -= d-dnew;
         }
         break;
      default:
         HError(6322,"XformBase: Bad target %s",ParmKind2Str(tgtBase,b1));
      }
      break;
   case LPREFC:
      switch(tgtBase){
      case LPC:       
         XformLPREFC2LPC(data,d);       
         break;
      case LPCEPSTRA: 
         XformLPREFC2LPC(data,d);       
         XformLPC2LPCEPSTRA(data,d,dnew,lifter);
         if (dnew<d) {
            DeleteColumn(data,cf->nUsed,dnew,d-dnew);
            cf->nUsed -= d-dnew;
         }
         break;
      default:
         HError(6322,"XformBase: Bad target %s",ParmKind2Str(tgtBase,b1));
      }
      break;
   case LPCEPSTRA:
      switch(tgtBase){
      case LPREFC:    
         XformLPCEPSTRA2LPC(data,d,lifter);
         XformLPC2LPREFC(data,d); 
         break;
      case LPC:
         XformLPCEPSTRA2LPC(data,d,lifter);
         break;
      default:
         HError(6322,"XformBase: Bad target %s",ParmKind2Str(tgtBase,b1));
      }
      break;
   case MELSPEC:
      switch(tgtBase){
      case FBANK:    
         XformMELSPEC2FBANK(data,d); 
         break;
      case MFCC:     
         XformMELSPEC2FBANK(data,d); 
         XformFBANK2MFCC(data,d,dnew,lifter);
         if (dnew<d) {
            DeleteColumn(data,cf->nUsed,dnew,d-dnew);
            cf->nUsed -= d-dnew;
         }
         break;
      default:
         HError(6322,"XformBase: Bad target %s",ParmKind2Str(tgtBase,b1));
      }
      break;
   case FBANK:    
      switch(tgtBase){
      case MELSPEC:    
         XformFBANK2MELSPEC(data,d); 
         break;
      case MFCC:     
         XformFBANK2MFCC(data,d,dnew,lifter);
         if (dnew<d) {
            DeleteColumn(data,cf->nUsed,dnew,d-dnew);
            cf->nUsed -= d-dnew;
         }
         break;
      default:
         HError(6322,"XformBase: Bad target %s",ParmKind2Str(tgtBase,b1));
      }
      break;
   default:
      HError(6322,"XformBase: Bad source %s",ParmKind2Str(curBase,b1));
   }
   cf->curPK = tgtBase|quals;
   if (trace&T_TOP)
      printf("HParm: Xform complete,  current is %s\n",ParmKind2Str(cf->curPK,b1));
}

/* ----------------- Parameter Coding Routines -------------------- */

/* ZeroMeanFrame: remove dc offset from given vector */
void ZeroMeanFrame(Vector v)
{
   int size,i;
   float sum=0.0,off;

   size = VectorSize(v);
   for (i=1; i<=size; i++) sum += v[i];
   off = sum / size;
   for (i=1; i<=size; i++) v[i] -= off;
}

/* SetUpForCoding: set style, sizes and  working storage */
static void SetUpForCoding(MemHeap *x, IOConfig cf, int frSize)
{
   char buf[50];
   ParmKind btgt;
  
   cf->s = CreateVector(x,frSize);
   cf->r = CreateShortVec(x,frSize);
   cf->curPK = btgt = cf->tgtPK&BASEMASK;
   cf->a = cf->k = cf->c = cf->fbank = NULL;
   SetCodeStyle(cf);
   switch(cf->style){
   case LPCbased:
      cf->nUsed = (btgt==LPCEPSTRA)?cf->numCepCoef:cf->lpcOrder;
      if (btgt==LPREFC)
         cf->k = CreateVector(x,cf->lpcOrder);
      else
         cf->a = CreateVector(x,cf->lpcOrder);
      if (btgt == LPCEPSTRA) 
         cf->c = CreateVector(x,cf->numCepCoef);
      break;
   case FFTbased:
      cf->nUsed = (btgt==MFCC || btgt == PLP) ? cf->numCepCoef : cf->numChans;
      cf->fbank = CreateVector(x,cf->numChans);
      cf->fbInfo = InitFBank (x, frSize, (long) cf->srcSampRate, cf->numChans, 
                              cf->loFBankFreq, cf->hiFBankFreq, cf->usePower, 
                              (btgt == PLP) ? FALSE : btgt != MELSPEC,
                              cf->doubleFFT,
                              cf->warpFreq, cf->warpLowerCutOff, cf->warpUpperCutOff);
      
      if (btgt != PLP) {
         if (btgt == MFCC) 
            cf->c = CreateVector(x,cf->numCepCoef);
      }
      else {            /* initialisation for PLP */
         cf->c = CreateVector (x, cf->numCepCoef+1);
         cf->as = CreateVector (x, cf->numChans+2);
         cf->eql = CreateVector (x, cf->numChans);
         cf->ac = CreateVector (x, cf->lpcOrder+1);
         cf->lp = CreateVector (x, cf->lpcOrder+1);
         cf->cm = CreateDMatrix (x, cf->lpcOrder+1, cf->numChans+2);
         InitPLP (cf->fbInfo, cf->lpcOrder, cf->eql, cf->cm);
      }
      break;
   default:
      HError(6321,"SetUpForCoding: target %s is not a parameterised form",
             ParmKind2Str(cf->tgtPK,buf));
   }
   if (cf->tgtPK&HASENERGY) {
      cf->curPK |= HASENERGY; ++cf->nUsed;
   }
   if (cf->tgtPK&HASZEROC) {
      cf->curPK |= HASZEROC; ++cf->nUsed;
   }
   if (!ValidConversion(cf->curPK,cf->tgtPK))
      HError(6322,"SetUpForCoding: cannot convert to %s",ParmKind2Str(cf->tgtPK,buf));
   if (cf->MatTranFN == NULL) 
      cf->tgtUsed = TotalComps(NumStatic(cf->nUsed,cf->curPK),cf->tgtPK);
   else {
      if (cf->preQual)
         cf->tgtUsed = NumRows(cf->MatTran)*(1+HasDelta(cf->tgtPK)+HasAccs(cf->tgtPK)+HasThird(cf->tgtPK));
      else  
	cf->tgtUsed = NumRows(cf->MatTran);
   }
   cf->nCols=TotalComps(NumStatic(cf->nUsed,cf->curPK),cf->tgtPK);
   cf->nCols = (cf->nCols>cf->tgtUsed)?cf->nCols:cf->tgtUsed;
   cf->nCvrt = cf->nUsed;
}

/* ConvertFrame: convert frame in cf->s and store in pbuf, return total
   parameters stored in pbuf */
static int ConvertFrame(IOConfig cf, float *pbuf)
{
   ParmKind btgt = cf->tgtPK&BASEMASK;
   float re,rawte=0.0,te,*p, cepScale = 1.0;
   int i,bsize=0;
   Vector v=NULL;
   char buf[50];
   Boolean rawE;
   
   p = pbuf;
   rawE = cf->rawEnergy;
   if (btgt<MFCC && cf->v1Compat)
      rawE = FALSE;

   if (cf->addDither!=0.0)
      for (i=1; i<=VectorSize(cf->s); i++)
         cf->s[i] += (RandomValue()*2.0 - 1.0)*cf->addDither;

   if (cf->zMeanSrc && !cf->v1Compat)
      ZeroMeanFrame(cf->s);
   if ((cf->tgtPK&HASENERGY) && rawE){
      rawte = 0.0;
      for (i=1; i<=VectorSize(cf->s); i++)
         rawte += cf->s[i] * cf->s[i];
   }
   if (cf->preEmph>0.0) 
      PreEmphasise(cf->s,cf->preEmph);
   if (cf->useHam) Ham(cf->s);
   switch(btgt){
   case LPC: 
      Wave2LPC(cf->s,cf->a,cf->k,&re,&te);
      v = cf->a; bsize = cf->lpcOrder;
      break;
   case LPREFC: 
      Wave2LPC(cf->s,cf->a,cf->k,&re,&te);
      v = cf->k; bsize = cf->lpcOrder;
      break;      
   case LPCEPSTRA:
      Wave2LPC(cf->s,cf->a,cf->k,&re,&te);
      LPC2Cepstrum(cf->a,cf->c);
      if (cf->cepLifter > 0)
         WeightCepstrum(cf->c, 1, cf->numCepCoef, cf->cepLifter);
      v = cf->c; bsize = cf->numCepCoef;
      break;
   case MELSPEC:
   case FBANK: 
      Wave2FBank(cf->s, cf->fbank, rawE?NULL:&te, cf->fbInfo);
      v = cf->fbank; bsize = cf->numChans;
      break;
   case MFCC: 
      Wave2FBank(cf->s, cf->fbank, rawE?NULL:&te, cf->fbInfo);
      FBank2MFCC(cf->fbank, cf->c, cf->numCepCoef);
      if (cf->cepLifter > 0)
         WeightCepstrum(cf->c, 1, cf->numCepCoef, cf->cepLifter);
      v = cf->c; bsize = cf->numCepCoef;
      break;
   case PLP:
      Wave2FBank(cf->s, cf->fbank, rawE ? NULL : &te, cf->fbInfo);
      FBank2ASpec(cf->fbank, cf->as, cf->eql, cf->compressFact, cf->fbInfo);
      ASpec2LPCep(cf->as, cf->ac, cf->lp, cf->c, cf->cm);
      if (cf->cepLifter > 0)
         WeightCepstrum(cf->c, 1, cf->numCepCoef, cf->cepLifter);
      v = cf->c; 
      bsize = cf->numCepCoef;
      break;
   default:
      HError(6321,"ConvertFrame: target %s is not a parameterised form",
             ParmKind2Str(cf->tgtPK,buf));
   }

   if (btgt == PLP || btgt == MFCC)
      cepScale = (cf->v1Compat) ? 1.0 : cf->cepScale;
   for (i=1; i<=bsize; i++) 
      *p++ = v[i] * cepScale;

   if (cf->tgtPK&HASZEROC){
      if (btgt == MFCC) {
         *p = FBank2C0(cf->fbank) * cepScale;
         if (cf->v1Compat) *p *= cf->eScale;
         ++p;
      }
      else      /* For PLP include gain as C0 */
         *p++ = v[bsize+1] * cepScale;   
      cf->curPK|=HASZEROC ;
   }
   if (cf->tgtPK&HASENERGY) {
      if (rawE) te = rawte;
      *p++ = (te<MINLARG) ? LZERO : log(te);  
      cf->curPK|=HASENERGY;
   }
   return p - pbuf;
}

/* Get data from external source and convert to 16 bit linear */
static int fGetWaveData(int n,void *data,short *res,
                        HParmSrcDef ext,void *bInfo)
{
   int r,i;

   r=ext->fGetData(ext->xInfo,bInfo,n,data);
   /* Transfer whole frame to processing buffer */
   if (ext->size==1) {
      /* 8 bit mulaw */
      unsigned char *d;
      static short u2l[]={
         -32124,-31100,-30076,-29052,-28028,-27004,-25980,-24956,
         -23932,-22908,-21884,-20860,-19836,-18812,-17788,-16764,
         -15996,-15484,-14972,-14460,-13948,-13436,-12924,-12412,
         -11900,-11388,-10876,-10364,-9852, -9340, -8828, -8316,
         -7932, -7676, -7420, -7164, -6908, -6652, -6396, -6140,
         -5884, -5628, -5372, -5116, -4860, -4604, -4348, -4092,
         -3900, -3772, -3644, -3516, -3388, -3260, -3132, -3004,
         -2876, -2748, -2620, -2492, -2364, -2236, -2108, -1980,
         -1884, -1820, -1756, -1692, -1628, -1564, -1500, -1436,
         -1372, -1308, -1244, -1180, -1116, -1052, -988,  -924,
         -876,  -844,  -812,  -780,  -748,  -716,  -684,  -652,
         -620,  -588,  -556,  -524,  -492,  -460,  -428,  -396,
         -372,  -356,  -340,  -324,  -308,  -292,  -276,  -260,
         -244,  -228,  -212,  -196,  -180,  -164,  -148,  -132,
         -120,  -112,  -104,  -96,   -88,   -80,   -72,   -64,
         -56,   -48,   -40,   -32,   -24,   -16,   -8,    0,
         32124, 31100, 30076, 29052, 28028, 27004, 25980, 24956,
         23932, 22908, 21884, 20860, 19836, 18812, 17788, 16764,
         15996, 15484, 14972, 14460, 13948, 13436, 12924, 12412,
         11900, 11388, 10876, 10364, 9852,  9340,  8828,  8316,
         7932,  7676,  7420,  7164,  6908,  6652,  6396,  6140,
         5884,  5628,  5372,  5116,  4860,  4604,  4348,  4092,
         3900,  3772,  3644,  3516,  3388,  3260,  3132,  3004,
         2876,  2748,  2620,  2492,  2364,  2236,  2108,  1980,
         1884,  1820,  1756,  1692,  1628,  1564,  1500,  1436,
         1372,  1308,  1244,  1180,  1116,  1052,  988,   924,
         876,   844,   812,   780,   748,   716,   684,   652,
         620,   588,   556,   524,   492,   460,   428,   396,
         372,   356,   340,   324,   308,   292,   276,   260,
         244,   228,   212,   196,   180,   164,   148,   132,
         120,   112,   104,   96,    88,    80,    72,    64,
         56,    48,    40,    32,    24,    16,    8,     0    };
      
      for (i=0,d=(unsigned char*)data;i<n;i++) res[i]=u2l[*d++];
   }
   if (ext->size==0x0101) {
      /* 8 bit alaw */
      unsigned char *d;
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
      
      for (i=0,d=(unsigned char*)data;i<n;i++) res[i]=a2l[*d++];
   }
   if (ext->size==2) {
      /* 16 bit linear */
      short *d;
      for (i=0,d=(short*)data;i<n;i++) res[i]=*d++;
   }  
   return(r);
}


/* ---------------- Silence detector code ------------------- */

#define DEF_SNR 30

enum {
   ml_min,     /* Min frame energy */
   ml_sil,     /* Average silence energy */
   ml_thresh,  /* Threshold for speech detector */
   ml_sp,      /* Average speeck energy */
   ml_max,     /* Max frame energy */
   ml_range,   /* Peak to peak sample range (% of max) */
   ml_off,     /* Average sample offset */
   ml_snr,     /* Assumed SNR ratio */
   ml_cnt      /* Total size of ml results array */
};

enum {
   ML_SIL_ST=2,
   ML_SIL_EN=3,
   ML_SP_ST=8,
   ML_SP_EN=9,
   ML_PARTS=10
};

static int fcmp(const void *v1,const void *v2)
{
   float f1,f2;
   f1=*(float*)v1;f2=*(float*)v2;
   if (f1<f2) return(-1);
   else if (f1>f2) return(1);
   else return(0);
}

/* CalSilDetParms: Calculate silence detector parameters */
static void CalcSilDetParms(ParmBuf pbuf, float *res, 
                            Vector eFr, float off, float range)
{
   IOConfig cf = pbuf->cf;
   float sil,sp,snr,thresh,nBl,aBl[ML_PARTS+1];
   int i,s,b,c,e,nFr,ndFr;

   nFr=VectorSize(eFr);

   /* Do partition and decide on levels */
   qsort((void*)(eFr+1),nFr,sizeof(float),fcmp);

   for(i=1,s=0,sil=0.0;i<=nFr;i++) {
      if (eFr[i]<cf->silDiscard) s++,sil+=eFr[i];
   }
   aBl[0]=(s>0?sil/s:0.0); ndFr=nFr-s; 
   for(b=1;b<=ML_PARTS;b++) aBl[b]=0.0;
   if (nFr==0) {
      /* No information uses floor level for silence detector */
      sil=snr=cf->silDiscard;
      sp=eFr[nFr];
   }
   else if (ndFr<ML_PARTS) {
      sil=snr=sil/nFr;  /* Average */
      sp=eFr[nFr];
   }
   else {
      /* Find number of frames in each bin */
      nBl=((float)ndFr)/ML_PARTS; 
      /* Find average energy in each bin */
      for(b=1,i=s+1,thresh=s+nBl;b<=ML_PARTS;b++,thresh+=nBl) {
         for (c=0,e=(int)(thresh+0.5);i<=e;i++,c++) aBl[b]+=eFr[i];
         aBl[b]=(c>0?aBl[b]/c:0.0);
      }

      for (sil=0,i=ML_SIL_ST;i<=ML_SIL_EN;i++) sil+=aBl[i];
      sil=sil/(ML_SIL_EN-ML_SIL_ST+1);
      for (sp=0,i=ML_SP_ST;i<=ML_SP_EN;i++) sp+=aBl[i];
      sp=sp/(ML_SP_EN-ML_SP_ST+1);
      /* If person speaks continuously this sil estimate may be too high */
      /*  so check lowest level is similar to silence level */
      snr=aBl[1];
   }

   if (sil-snr>cf->spThresh) {
      /* Silence level is dodgy replace with lowest estimate */
      sil=snr;
   }
   if (eFr[nFr]-eFr[1]>cf->spThresh && sp-sil>cf->spThresh) {
      /* Have reasonable SNR so try to be slightly more clever */
      snr=sp-sil;
   }
   else {
      /* SNR ratio is abysmal - assume signal was basically silence */
      sp=-1.0;
      snr=3*cf->spThresh;
      thresh=sil+cf->spThresh;
   }
   thresh=sil+cf->spThresh;

   res[ml_min]=eFr[1];
   res[ml_sil]=sil;
   res[ml_thresh]=thresh;
   res[ml_sp]=sp;
   res[ml_max]=eFr[nFr];
   res[ml_range]=range;
   res[ml_off]=off;
   res[ml_snr]=snr;

   if (trace&T_DET) {
      int j;
      printf("Frame levels\n");
      for (j=1;j<=nFr;j++) {
         printf(" %5.2f",eFr[j]);
         if ((j%10)==0) printf("\n");
      }
      printf("\n");
      for (j=0;j<=ML_PARTS;j++)
         printf(" Block %d == %.2f\n",j,aBl[j]);
   }
   if (trace&T_TOP) {
      printf("Levels: Offset %.1f, Range %.1f%%, Min %.1fdB, Max %.1fdB\n",
             res[ml_off],res[ml_range],res[ml_min],res[ml_max]);
      printf("        Silence %.1fdB, Thresh %.1fdB, Speech %.1fdB (SNR %.1f)\n",
             res[ml_sil],res[ml_thresh],res[ml_sp],res[ml_snr]);
      fflush(stdout);
   }
}

/* CalcSilDetParmsAudio: listen to audio input and calculate silence detector 
                         threshold  parameters */ 
static void SetAudioSpDetParms(ParmBuf pbuf, float dur, Boolean warn)
{
   IOConfig cf = pbuf->cf;
   AudioIn a;
   Vector v,eFr;
   float mlRes[ml_cnt],x,m,e,range,off;
   int i,j,n,nFr,xMin=32767,xMax=-32768;

   if ((a = OpenAudioInput(pbuf->mem,&(cf->srcSampRate), 
                           cf->winDur, cf->tgtSampRate)) == NULL)
      HError(6306,"OpenAsChannel: Audio input not supported");

   off = 0.0;
   n=SampsInAudioFrame(a); 
   v=CreateVector(&gstack,n);
   nFr=(int) (dur*1.0E7/cf->tgtSampRate);
   if (nFr<ML_PARTS) nFr=ML_PARTS;
   eFr=CreateVector(&gstack,nFr);

   StartAudioInput(a,NULLSIG);
   if (warn) printf("Please speak sentence - measuring levels\n");

   for (i=1; i<=nFr; i++){
      GetAudio(a,1,v+1);
      for (j=1, m=e=0.0; j<=n; j++) {
         x=v[j]; m += x; e += x*x;
         if (x<xMin) xMin = (int) x;
         if (x>xMax) xMax = (int) x;
      }
      m/=n; e=e/n-m*m;
      eFr[i] = (e>0.0)?10.0*log10(e/0.32768):-0.0;

      off += m;
   }
   if (warn) printf("Level measurement completed\n");
   StopAudioInput(a);
   CloseAudioInput(a);

   off=off/nFr;
   range=(xMax-xMin)/65536.0;

   CalcSilDetParms(pbuf,mlRes,eFr,off,range);

   pbuf->chan->frMin=mlRes[ml_min]; pbuf->chan->spDetSil=mlRes[ml_sil]; 
   pbuf->chan->spDetThresh=mlRes[ml_thresh]; 
   pbuf->chan->spDetSp=mlRes[ml_sp]; pbuf->chan->frMax=mlRes[ml_max]; 
   pbuf->chan->spDetSNR=mlRes[ml_snr]; 
   pbuf->chan->chPeak=mlRes[ml_range]; pbuf->chan->chOffset=mlRes[ml_off]; 
}

/* CalcSilDetParmsWave: Analyse waveform and calculate silence detector 
                        parameters */
static void SetWaveSpDetParms(ParmBuf pbuf)
{
   IOConfig cf = pbuf->cf;
   Vector v,eFr;
   float mlRes[ml_cnt],x,m,e,range,off;
   int i,j,n,nFr,nBl,xMin=32767,xMax=-32768;
   short *data;
   long nSamp;
 
   off = 0.0;
   n=cf->frSize;
   v=CreateVector(&gstack,cf->frSize);
   data = GetWaveDirect(pbuf->in.w,&nSamp);
   nFr = (nSamp-cf->frSize)/cf->frRate + 1;
  
   if (nFr<ML_PARTS) nFr=ML_PARTS;
   eFr=CreateVector(&gstack,nFr);
   
   for(i=1; i<=nFr; i++){
      for (j=1, m=e=0.0; j<=n; j++) {
         x=(float)data[(i-1)*cf->frRate+j-1];
         m += x; e += x*x;
         if (x<xMin) xMin = (int) x;
         if (x>xMax) xMax = (int) x;
      }
      m/=n; e=e/n-m*m;
      eFr[i] = (e>0.0)?10.0*log10(e/0.32768):-0.0;
      
      off += m;
   }   
   off=off/nFr; nBl=nFr/ML_PARTS;
   range=(xMax-xMin)/65536.0;
   
   CalcSilDetParms(pbuf,mlRes,eFr,off,range);

   pbuf->chan->frMin=mlRes[ml_min]; pbuf->chan->spDetSil=mlRes[ml_sil]; 
   pbuf->chan->spDetThresh=mlRes[ml_thresh];
   pbuf->chan->spDetSp=mlRes[ml_sp]; pbuf->chan->frMax=mlRes[ml_max]; 
   pbuf->chan->spDetSNR=mlRes[ml_snr]; 
   pbuf->chan->chPeak=mlRes[ml_range]; pbuf->chan->chOffset=mlRes[ml_off]; 
}

/* CalcSilDetParmsAudio: listen to audio input and calculate silence detector 
                         threshold  parameters */ 
static void SetExtSpDetParms(ParmBuf pbuf, float dur, Boolean warn)
{
   IOConfig cf = pbuf->cf;
   ShortVec v;
   Vector eFr,teFr;
   float mlRes[ml_cnt],x,m,e,range,off;
   int i,j,y,z,n,mFr,nFr,xMin=32767,xMax=-32768;

   off = 0.0;

   n=cf->frSize;
   v=CreateShortVec(&gstack,cf->frSize);
   mFr=(int) (dur*1.0E7/cf->tgtSampRate);
   if (mFr<ML_PARTS) mFr=ML_PARTS;
   teFr=CreateVector(&gstack,mFr);

   if (pbuf->ext->fStart!=NULL)
      pbuf->ext->fStart(pbuf->ext->xInfo,pbuf->in.i);
   if (warn) printf("Please speak sentence - measuring levels\n");

   for (i=1,nFr=0; i<=mFr; i++,nFr++){
      /* Copy overlap */
      if (i>1) {
         z=cf->frRate,y=cf->frSize-cf->frRate;
         for (j=1;j<=y;j++) v[j]=v[j+cf->frRate];
      }
      else z=cf->frSize,y=0;
      /* Read frame worth of data from user routine */
      if (fGetWaveData(z,cf->rawBuffer,v+y+1,pbuf->ext,pbuf->in.i)!=z) break;
      /* if (pbuf->ext->fGetData(pbuf->ext->xInfo,pbuf->in.i,z,v+y+1)!=z) break; */
      
      for (j=1, m=e=0.0; j<=n; j++) {
         x=v[j]; m += x; e += x*x;
         if (x<xMin) xMin = (int) x;
         if (x>xMax) xMax = (int) x;
      }
      m/=n; e=e/n-m*m;
      teFr[i] = (e>0.0)?10.0*log10(e/0.32768):-0.0;

      off += m;
   }
   if (warn) printf("Level measurement completed\n");
   if (pbuf->ext->fStop!=NULL)
      pbuf->ext->fStop(pbuf->ext->xInfo,pbuf->in.i);
   
   if (nFr==0)
      HError(6325,"SetExtSpDetParms: Cannot calibrate detector without data");

   eFr=CreateVector(&gstack,nFr);
   for (i=1; i<=nFr; i++) eFr[i]=teFr[i];
   off=off/nFr;
   range=(xMax-xMin)/65536.0;

   CalcSilDetParms(pbuf,mlRes,eFr,off,range);

   pbuf->chan->frMin=mlRes[ml_min]; pbuf->chan->spDetSil=mlRes[ml_sil]; 
   pbuf->chan->spDetThresh=mlRes[ml_thresh]; 
   pbuf->chan->spDetSp=mlRes[ml_sp]; pbuf->chan->frMax=mlRes[ml_max]; 
   pbuf->chan->spDetSNR=mlRes[ml_snr]; 
   pbuf->chan->chPeak=mlRes[ml_range]; pbuf->chan->chOffset=mlRes[ml_off]; 
}

/* SetSelfCallSpDetParms: set detector parameters on first part of utterance */
static void SetSelfCalSpDetParms(ParmBuf pbuf)
{
   Vector eFr;
   float mlRes[ml_cnt],range,off;
   int i,nFr;
   
   if (!pbuf->chan->spDetParmsSet && pbuf->cf->silMean>0.0) {
      pbuf->chan->spDetSil = pbuf->cf->silMean;
      pbuf->chan->spDetThresh = pbuf->cf->silMean+pbuf->cf->spThresh;
      pbuf->chan->spDetParmsSet = TRUE;
      return;
   }
   nFr=pbuf->main.nRows;
   if (pbuf->cf->selfCalSilDet<0 && nFr>-pbuf->cf->selfCalSilDet)
      nFr=-pbuf->cf->selfCalSilDet;
   eFr=CreateVector(&gstack,nFr);
   
   for (i=1; i<=nFr; i++) eFr[i]=pbuf->spVal[i-1];
   /* These are not actually known */
   off=0.0; range=1.0;
   
   CalcSilDetParms(pbuf,mlRes,eFr,off,range);
   
   pbuf->chan->frMin=mlRes[ml_min]; pbuf->chan->spDetSil=mlRes[ml_sil]; 
   pbuf->chan->spDetThresh=mlRes[ml_thresh]; 
   pbuf->chan->spDetSp=mlRes[ml_sp]; pbuf->chan->frMax=mlRes[ml_max]; 
   pbuf->chan->spDetSNR=mlRes[ml_snr]; 
   pbuf->chan->chPeak=mlRes[ml_range]; pbuf->chan->chOffset=mlRes[ml_off]; 
   pbuf->chan->spDetParmsSet = TRUE;
}

/* SetSilDetParms: Check and if not set, etup silence detector thresholds */
static void SetSilDetParms(ParmBuf pbuf, TriState silMeasure)
{
   /* Don't set now if using self calibration (will be set later) */
   if (pbuf->cf->selfCalSilDet!=0) return;
   if (((pbuf->cf->measureSil==TRUE && silMeasure!=FALSE) || 
        silMeasure==TRUE ||
        (!pbuf->chan->spDetParmsSet && pbuf->cf->silMean<=0.0))) {
      switch(pbuf->chType) {
      case ch_haudio:
         SetAudioSpDetParms(pbuf,4.0,pbuf->cf->outSilWarn);
         break;
      case ch_hwave:
         SetWaveSpDetParms(pbuf);
         break;
      case ch_ext_wave:
      case ch_ext_parm:
         SetExtSpDetParms(pbuf,4.0,pbuf->cf->outSilWarn);
         break;
      }
      pbuf->chan->spDetParmsSet = TRUE;
   } else if (!pbuf->chan->spDetParmsSet) {
      pbuf->chan->spDetSil = pbuf->cf->silMean;
      pbuf->chan->spDetThresh = pbuf->cf->silMean+pbuf->cf->spThresh;
      pbuf->chan->spDetParmsSet = TRUE;
   }
}


static void ExtractObservation(float *fp, Observation *o);

void RunSilDet(ParmBuf pbuf,Boolean cleared)
{
   IOConfig cf = pbuf->cf;
   int i;

   if (!pbuf->chan->spDetParmsSet)
      HError(6325,"RunSilDet: Cannot run sil detector without sil estimate");
   
   for (i=pbuf->spDetCur-pbuf->main.stRow;i<=pbuf->qen;i++,pbuf->spDetCur++){
      /* Choose method of Speech/Silence decision */    
      pbuf->spVal[i] -= pbuf->chan->spDetThresh;
      
      /* Silence detector algorithm - independent of method above */
      if (pbuf->spVal[i]>0.0){
         pbuf->spDetCnt++;
      } else {
         pbuf->silDetCnt++;
      }
      if (pbuf->spDetCur>=cf->spcSeqCount) {
         if (pbuf->spVal[i-cf->spcSeqCount]>0.0) pbuf->spDetCnt--;
         else pbuf->silDetCnt--;
      }
      switch(pbuf->status) {
      case PB_WAITING:
         if (pbuf->spDetCnt>=cf->spcSeqCount-cf->spcGlchCount && 
             pbuf->spDetCur>=cf->spcSeqCount) {
            ChangeState(pbuf,PB_STOPPING); pbuf->spDetLst=pbuf->spDetCur;
            pbuf->spDetSt=pbuf->spDetCur-cf->spcSeqCount-cf->marginCount;
            if (pbuf->spDetSt<0) pbuf->spDetSt=0;
            /* Guaranteed haven't read a row yet */
            pbuf->outRow=pbuf->spDetSt;
            /* Can read to end of current window */
            pbuf->spDetFin=pbuf->spDetCur;
            if (trace&T_DET) 
               printf(" Detector: In speech @ %d\n",pbuf->spDetSt);
            break;
         }
         /* pbuf->spDetFin stays at -1 till we get to speech */
         break;
      case PB_STOPPING:
         if (pbuf->spDetCur-pbuf->spDetLst>=cf->silSeqCount+cf->spcSeqCount){
            StopBuffer(pbuf);
            ChangeState(pbuf,PB_STOPPED);
            pbuf->spDetEn=pbuf->spDetLst+cf->marginCount;
            pbuf->spDetFin=pbuf->spDetEn-1;
            if (trace&T_DET)
               printf(" Detector: Finished speech @ %d\n",pbuf->spDetEn);
            break;
         }
         if (pbuf->spDetCnt>=cf->spcSeqCount-cf->silGlchCount)
            pbuf->spDetLst=pbuf->spDetCur;
         else if (pbuf->spDetLst==pbuf->spDetCur-1 && (trace&T_DET))
            printf(" Detector: Start sil @ %d\n",i);
         if (pbuf->spDetCur < pbuf->spDetLst + cf->marginCount)
            pbuf->spDetFin=pbuf->spDetCur;
         break;
      case PB_STOPPED:
         /* Make sure that spDetFin set okay */
         pbuf->spDetFin=pbuf->spDetEn-1;
         break;
      }
   }
   /* if no speech/silence detected then force buffer status onwards */
   if (cleared) {
      if (pbuf->status==PB_WAITING) {
         pbuf->spDetSt=pbuf->spDetEn=0;pbuf->spDetFin=-1;
         ChangeState(pbuf,PB_STOPPED);
      }
      else if (pbuf->status==PB_STOPPING) {
         pbuf->spDetEn=pbuf->spDetCur;
         pbuf->spDetFin=pbuf->spDetEn-1;
         ChangeState(pbuf,PB_STOPPED);
      }
   }
}

/* ---------------- Read/Write ESignal Format Parameter File --------------- */

/* EXPORT ReadESIGPHeader: get header from Esignal pamameter file;
   return FALSE in case of failure */
Boolean ReadESIGPHeader(FILE *f, long *nSamp, long *sampP, short *sampS,
                        short *kind, Boolean *bSwap, Boolean isPipe)
{
   long hdrS;

   return ReadEsignalHeader(f, nSamp, sampP, sampS,
                            kind, bSwap, &hdrS, isPipe);
}


/* ----------------- Observation Handling Routines -------------- */


/* ExtractObservation: copy vector of floats starting at fp into
   observation, splitting into streams as necessary */
static void ExtractObservation(float *fp, Observation *o)
{
   int i,j,k,n,w1,w2,w,nStatic = 0;
   int numS = o->swidth[0];
   Vector v,ev;
   Boolean wantE,skipE;
   
   if (trace&T_OBS) {
      for (i=1,j=0; i<=numS; i++) j += o->swidth[i];
      printf("HParm: Extracting %d observation components\n",j);
      for (i=0; i<j; i++) printf("%8.4f ",fp[i]); printf("\n");
   }
   if (o->eSep){
      wantE = !(o->pk&HASNULLE);
      if (numS == 2){
         w1 = o->swidth[1]; w2 = NumEnergy(o->pk); 
         w = w1+w2; n = w/w2;
         v = o->fv[1]; ev = o->fv[numS];     
         for (i=j=k=1; i<=w; i++){
            if (i%n == 0 )  {
               if (wantE || i>n) ev[k++] = *fp;
               fp++;
            } else
               v[j++] = *fp++;
         }
      } else {
         ev = o->fv[numS]; 
         for (i=1,k=1; i<numS; i++){
            v = o->fv[i];
            for (j=1; j<=o->swidth[i]; j++)
               v[j] = *fp++;
            if (wantE || i>1) ev[k++] = *fp;
            fp++;
         }        
      }
      if (k-1 != o->swidth[numS])
         HError(6391,"ExtractObservation: %d of %d E vals copied",
                k-1,o->swidth[numS]);
   } else {
      skipE = (o->pk&(HASENERGY|HASZEROC)) && (o->pk&HASNULLE);
      if (skipE) {
         nStatic = o->swidth[1];
         if (numS==1) nStatic = (nStatic+1)/NumEnergy(o->pk) - 1;
      }
      for (i=1,k=0; i<=numS; i++){
         v = o->fv[i];
         for (j=1; j<=o->swidth[i]; j++){
            v[j] = *fp++; k++;
            if (skipE && k==nStatic) ++fp;            
         }
      }
   }
}

/* EXPORT->MakeObservation: Create obs using info in swidth and pkind */
Observation MakeObservation(MemHeap *x, short *swidth, 
                            ParmKind pkind, Boolean forceDisc, Boolean eSep)
{
   Observation ob;
   int i,numS;
   
   ob.pk = pkind; ob.bk = pkind&(~HASNULLE); ob.eSep = eSep;
   for (i=0; i<SMAX; i++) ob.fv[i]=NULL,ob.vq[i]=-1;
   if (forceDisc) {
      if ((pkind&BASEMASK) != DISCRETE && !(pkind&HASVQ))
         HError(6373,"MakeObservation: No way to force discrete observation");
      ob.pk = DISCRETE+(pkind&HASNULLE);
   }
   numS = swidth[0];
   if (numS>=SMAX)
      HError(6372,"MakeObservation: num streams(%d) > MAX(%d)",numS,SMAX-1);
   for (i=0; i<=numS; i++){
      ob.swidth[i] =swidth[i];
      if (i>0 && (pkind&BASEMASK) == DISCRETE && swidth[i] != 1)
         HError(6372,"MakeObservation: discrete stream widths must be 1");
   }
   /* Note that the vectors are created even if ob.pk==DISCRETE as */
   /* these are used in ReadAs????? but should not be accessed elsewhere */
   if ((pkind&BASEMASK) != DISCRETE)
      for (i=1; i<=numS; i++)
         ob.fv[i] = CreateVector(x,swidth[i]);
   return ob;
}

#define OBMARGIN 6         /* margin width for displaying observations */
#define OBFLTFORM "%8.3f"  /* format for displaying component values */
#define OBEXPFORM "%8s"    /* format for displaying component names */

/* EXPORT->ExplainObservation: explain the structure of given observation */
void ExplainObservation(Observation *o, int itemsPerLine)
{
   char idx[10],str[5],buf[20],blank[10];
   int s,j,k,len,n,nTotal,nStatic,numS,nDel;
   Boolean idd = FALSE,isE;
   
   for (j=0; j<OBMARGIN; j++) blank[j] = ' ';
   blank[OBMARGIN] = '\0';
   numS = o->swidth[0];
   for (s=1,nTotal=0; s<=numS; s++)
      nTotal += o->swidth[s];
   if (o->pk&HASNULLE) ++nTotal;
   n=1; 
   if (o->pk&HASDELTA) {
      ++n; if (o->pk&HASACCS) {
        ++n;
        if (o->pk&HASTHIRD){
           ++n;
           if (highDiff == TRUE){
              ++n;
           }
        }
      }
   }
   nStatic = nTotal/n;
   if (o->eSep) --nStatic;
   nDel = nStatic*2;
   if (o->pk&HASNULLE && !o->eSep) { --nDel; --nStatic; --nTotal;}
   if (trace&T_OBS) 
      printf("HParm: ExplainObs: nTotal=%d, nStatic=%d, nDel=%d\n",nTotal,nStatic,nDel);
   if ((o->pk&BASEMASK) == DISCRETE || (o->pk&HASVQ)){
      strcpy(buf,"x:");
      len = strlen(buf);
      while (len++<OBMARGIN) strcat(buf," ");
      printf("%s",buf);
      for (s=1; s<=numS; s++) printf(" VQ[%d]=VQindex ",s);
      printf("\n");
      idd = TRUE;
   }
   if ((o->pk&BASEMASK) == DISCRETE) return;
   for (s=1; s<=o->swidth[0]; s++){
      buf[0] = str[0] = idx[0] = '\0';
      if (!idd || numS>1) {
         strcpy(idx,"x");
         if (numS>1) sprintf(str,".%d",s);
         strcpy(buf,idx); strcat(buf,str); strcat(buf,":");
      }
      len = strlen(buf);
      while (len++<OBMARGIN) strcat(buf," ");
      printf("%s",buf);
      for (j=1,k=0; j<=o->swidth[s]; j++){
         n = j;
         if (o->eSep && s==numS){
            strcpy(str,(o->pk&HASENERGY)?"E":"C0"); isE = TRUE;
            switch((o->pk&HASNULLE)?j+1:j){
            case 1:
               strcpy(buf,str); break;
            case 2: 
               strcpy(buf,"Del"); strcat(buf,str);
               break;
            case 3: 
               strcpy(buf,"Acc"); strcat(buf,str);
               break;
            }
         } else {
            strcpy(str,""); strcpy(buf,""); isE = FALSE;
            if ((o->pk&(HASENERGY|HASZEROC)) && !o->eSep ) {
               if(s==1)
                  isE = (!(o->pk&HASNULLE) && (j==nStatic)) || (j==nDel) || (j==nTotal);
               else
                  isE = j==o->swidth[s];
            }
            if (isE) strcpy(str,(o->pk&HASENERGY)?"E":"C0");
            switch(s){
            case 1: 
               if (j<=nStatic) {
                  if(!isE) {
                     ParmKind2Str(BaseParmKind(o->pk),buf); 
                     buf[5] = '\0';   /* truncate to 5 chars max */
                  }
               } else if (j<=nDel) {
                  strcpy(buf,"Del"); n -= nStatic;
               } else {
                  strcpy(buf,"Acc"); n -= nDel;
               }
               break;
            case 2: 
               strcpy(buf,"Del"); break;
            case 3: 
               strcpy(buf,"Acc"); break;
            }
            strcat(buf,str);
            if (!isE) {
               sprintf(idx,"-%d",n); strcat(buf,idx);
            }
         }
         printf(OBEXPFORM,buf);
         if (++k == itemsPerLine) {
            printf("\n%s",(j<o->swidth[s])?blank:""); 
            k = 0;
         }
      }
      if (k>0)printf("\n");
   }
}

/* EXPORT->PrintObservation: Print o with index i (if i>0) */
void PrintObservation(int i, Observation *o, int itemsPerLine)
{
   char idx[10],str[5],buf[20],blank[10];
   int s,j,k,len;
   Boolean idd = FALSE;
   
   for (j=0; j<OBMARGIN; j++) blank[j] = ' ';
   blank[OBMARGIN] = '\0';

   if ((o->pk&BASEMASK) == DISCRETE || (o->pk&HASVQ)){
      if (i>=0) sprintf(buf,"%d:",i);
      len = strlen(buf);
      while (len++<OBMARGIN) strcat(buf," ");
      printf("%s",buf);
      for (s=1; s<=o->swidth[0]; s++) printf(" VQ[%d]=%d ",s,o->vq[s]);
      printf("\n");
      idd = TRUE;
   }
   if ((o->pk&BASEMASK) == DISCRETE) return;
   for (s=1; s<=o->swidth[0]; s++){
      buf[0] = str[0] = idx[0] = '\0';
      if ((i>=0 && !idd) || (i>=0 && o->swidth[0]>1)) {
         sprintf(idx,"%d",i);
         if (o->swidth[0]>1) sprintf(str,".%d",s);
         strcpy(buf,idx); strcat(buf,str); strcat(buf,":");
      }
      len = strlen(buf);
      while (len++<OBMARGIN) strcat(buf," ");
      printf("%s",buf);
      for (j=1,k=0; j<=o->swidth[s]; j++){
         printf(OBFLTFORM,o->fv[s][j]);
         if (++k == itemsPerLine) {
            printf("\n%s",(j<o->swidth[s])?blank:""); 
            k = 0;
         }
      }
      if (k>0)printf("\n");
   }
   fflush(stdout);
}
 
/* ZeroStreamWidths: store numS in swidth[0] and set rest to 0 */
void ZeroStreamWidths(int numS, short *swidth)
{
   int s;

   if (numS >=SMAX)
      HError(6372,"ZeroStreamWidths: num streams(%d) > MAX(%d)\n",
             numS,SMAX-1);
   swidth[0] = numS;
   for (s=1; s<=numS; s++) swidth[s] = 0;
}

/* EXPORT->SetStreamWidths: if not already set, put stream widths in swidth 
           for desired stream split and set eSep.  Otherwise just set eSep */
void  SetStreamWidths(ParmKind pk, int size, short *swidth, Boolean *eSep)
{ 
   Boolean isSet,ok;
   short span[12], sw[SMAX];
   char buf[50];
   int s,neTab,neObs;
   
   /* adjust the target vector size when _N used */
   if ((pk&(HASENERGY|HASZEROC)) && (pk&HASNULLE))
      size++;

   *eSep = FALSE;  ok = TRUE;
   if ((pk&BASEMASK)==DISCRETE){
      swidth[0] = size;
      for (s=1; s<=swidth[0]; s++) swidth[s]=1;
      return;
   }
   isSet = swidth[1] != 0;
   ZeroStreamWidths(swidth[0],sw);
   FindSpans(span,pk,size);
   neObs = neTab = NumEnergy(pk);
   if (pk&HASNULLE) --neObs;
   switch (sw[0]) {
   case 1:  
      sw[1] = size;
      if (pk&HASNULLE) --sw[1];
      break;
   case 2:
      if (pk&(HASENERGY|HASZEROC)){
         sw[2] = neObs;
         sw[1] = size - neTab;
         *eSep = TRUE;
      } else if (!(pk&HASACCS) && (pk&HASDELTA)){
         sw[2] = span[5]-span[4]+1;
         sw[1] = size - sw[2];
      } else
         ok = FALSE;
      break;
   case 3:
      if (pk&HASACCS){
         sw[2] = span[5]-span[4]+1;
         sw[3] = span[7]-span[6]+1;
         sw[1] = size - sw[2] - sw[3];
         if (pk&HASNULLE) --sw[1];
      } else if ((pk&HASDELTA) && (pk&(HASENERGY|HASZEROC))){
         sw[1] = sw[2] = span[1]-span[0]+1;
         sw[3] = neObs;
         *eSep = TRUE; 
      } else
         ok = FALSE;
      break;
   case 4:
      if ((pk&HASACCS) && (pk&(HASENERGY|HASZEROC))){
         sw[1] = sw[2] = sw[3] = span[1]-span[0]+1;
         sw[4] = neObs;
         *eSep = TRUE;
      } else
         ok = FALSE;
      break;
   }
   if (ok && isSet){   /* see if standard split, if not clear eSep */
      for (s=1; s<=sw[0]; s++)
         if (sw[s] != swidth[s]){
            *eSep = FALSE; break;
         }
   } else if (ok && !isSet) {   /* return standard split */
      for (s=1; s<=sw[0]; s++)
         swidth[s] = sw[s];
   } else if (!ok && isSet){
      *eSep = FALSE;            /* must be non standard split */
   } else
      HError(6372,"SetStreamWidths: cant split %s into %d streams",
             ParmKind2Str(pk,buf),swidth[0]);
}

/* ----- side based normalisation ------------ */

/* load appropriate mean vector into ParmBuf */
static void LoadCMeanVector( MemHeap* x , IOConfig cf , char* fname )
{
   static char mfname_prev[MAXFNAMELEN] = "";
   static Vector meanVector = NULL;

   char mfname[MAXFNAMELEN]; /* actual mean filename */
   char pname[MAXFNAMELEN];  /* actual path name */
   char buf[MAXSTRLEN];    /* temporary working buffer */
   char buf2[MAXSTRLEN];   /* temporary working buffer */
   Source src;
   ParmKind pk,tgtMask;
   int dim;

   /* make filename and open it*/
   if (cf->cMeanDN == 0 || cf->cMeanMask == 0)
      HError(6376 , "LoadCMeanVector: mask or dir missing");
   if (!MaskMatch (cf->cMeanMask, mfname, fname))
      HError (6376, "LoadCMeanVector: non-matching mask %s", cf->cMeanMask);

   if ( cf->cMeanPathMask != 0 ){
      if (!MaskMatch (cf->cMeanPathMask, pname, fname))
         HError (6376, "LoadCMeanVector: non-matching path mask %s", cf->cMeanPathMask);
      MakeFN (pname, cf->cMeanDN, 0, buf2);
      MakeFN (mfname, buf2, 0, buf);
   }
   else
      MakeFN(mfname, cf->cMeanDN, 0, buf);
   
   /* caching of vector */
   if (strcmp(buf, mfname_prev) == 0){ /* names match , old vector must be the same */
      cf->cMeanVector = CreateVector(x,VectorSize(meanVector));
      CopyVector(meanVector, cf->cMeanVector ); 
      return;
   }
   else
      strcpy(mfname_prev, buf);
   
   /* read file header and ParmKind */
   if (InitSource (buf, &src, NoFilter) < SUCCESS)
      HError (6310, "LoadCMeanVector: Can't open cepsmean file %s", buf);
   SkipComment (&src);
   ReadString (&src, buf);
   if (strcmp(buf,"<CEPSNORM>") != 0)
      HError (6376 , "LoadCMeanVector: <CEPSNORM> missing, read: %s", buf);
   ReadString (&src, buf);
   buf[strlen(buf)-1] = 0;
   pk = Str2ParmKind (buf+1);

   /* check if ParmKind matches with target ParmKind */
   tgtMask = ~( cf->tgtPK & ( HASDELTA | HASACCS | HASTHIRD | HASZEROM | HASVQ ) ); /* mask irrelevant target flags */
   if ((pk & tgtMask) != (cf->tgtPK & tgtMask))
      HError(6376, "LoadCMeanVector: ParmKind mismatch %s not a subset of %s",
             ParmKind2Str (pk, mfname), ParmKind2Str (cf->tgtPK, buf));

   /* Load mean vector */
   while ((strcmp (buf, "<MEAN>") != 0) && !feof (src.f)) {
      ReadString (&src,buf);
   }
   if (strcmp(buf, "<MEAN>") != 0)
      HError(6376, "LoadCMeanVector: <MEAN> missing, read: %s", buf);
   ReadInt (&src, &dim, 1, FALSE);
   
   /* caching of vector */
   if (meanVector)
       Dispose (&gcheap, meanVector);
   meanVector = CreateVector (&gcheap, dim);
   if (!ReadVector (&src, meanVector , FALSE))
      HError(6376, "LoadCMeanVector: Couldn't read mean vector from file");

   cf->cMeanVector = CreateVector (x, dim);
   CopyVector (meanVector, cf->cMeanVector);
   CloseSource (&src);
}

/* load appropriate vscale vector into ParmBuf */
static void LoadVarScaleVector(MemHeap* x, IOConfig cf, char *fname)
{
   static char mfname_prev[MAXFNAMELEN] = "";
   static Vector varVector = NULL;
   char mfname[MAXFNAMELEN]; /* actual mean filename */
   char pname[MAXFNAMELEN];  /* actual path name */
   char buf[MAXSTRLEN];    /* temporary working buffer */
   char buf2[MAXSTRLEN];   /* temporary working buffer */
   Source src;
   ParmKind pk;
   int dim,i,NewFDim,FDim;
   Matrix CepstrVarNorm,NewCepstrVarNorm;

   /* make filename and open it*/
   if (cf->varScaleDN == 0 || cf->varScaleMask == 0)
      HError(6376, "LoadVarScaleVector: mask or dir missing");
   if (!MaskMatch (cf->varScaleMask, mfname, fname))
      HError (6376, "LoadVarScaleVector: non-matching mask %s", cf->varScaleMask);

   if ( cf->varScalePathMask != 0 ){
      if (!MaskMatch (cf->varScalePathMask, pname, fname))
         HError (6376, "LoadVarScaleVector: non-matching path mask %s", cf->varScalePathMask);
      MakeFN (pname, cf->varScaleDN, 0, buf2);
      MakeFN (mfname, buf2, 0, buf);
   }
   else
      MakeFN (mfname, cf->varScaleDN, 0, buf);
   
   /* caching of vector: if the same side as the previous one, just use the cached one */
   if (strcmp (buf, mfname_prev) == 0 ){ /* names match , old vector must be the same */
      cf->varScaleVector = CreateVector (x, VectorSize (varVector));
      CopyVector(varVector, cf->varScaleVector);
      return;
   }
   else {
      strcpy(mfname_prev, buf);
   }

   /* read file header and ParmKind */
   if (InitSource (buf, &src, NoFilter) < SUCCESS)
      HError (6310, "LoadVarScaleVector: Can't open varscale file %s", buf);
   SkipComment (&src);
   ReadString(&src,buf);
   if  (strcmp (buf, "<CEPSNORM>") != 0)
      HError (6376 ,"LoadVarScaleVector: <CEPSNORM> missing, read: %s",buf);
   ReadString (&src,buf);
   buf[strlen(buf)-1] = 0;
   pk = Str2ParmKind(buf+1 );

   /* check if ParmKind matches with target ParmKind */

   if ( (cf->tgtPK != pk) && ( cf->tgtPK != (pk | HASVQ)))
      HError (6376 ,"LoadVarScaleVector: ParmKind mismatch %s != %s ", 
              ParmKind2Str (pk, mfname), ParmKind2Str (cf->tgtPK, buf));

   /* Load variance vector */
   while ((strcmp (buf, "<VARIANCE>") != 0) && !feof(src.f)) {
      ReadString (&src, buf);
   }
   if  (strcmp (buf, "<VARIANCE>") != 0)
      HError (6376, "LoadVarScaleVector: <VARIANCE> missing, read: %s", buf);
   ReadInt (&src, &dim, 1, FALSE);

   /* refresh the cache if a new side */
   if (varVector)
      Dispose (&gcheap, varVector);
   varVector = CreateVector (&gcheap, dim);
   if (!ReadVector (&src, varVector, FALSE))
      HError(6376,"LoadVarScaleVector: Couldn't read var scale vector from file");
   
   /* Apply the linear transform to the cepstral variance normalizer if needed */
   if ((cf->MatTranFN != NULL) && (UseOldXFormCVN)){
      
      FDim = NumCols(cf->MatTran);
      NewFDim = NumRows(cf->MatTran);
      NewCepstrVarNorm = CreateMatrix(x,NewFDim,NewFDim);
      ZeroMatrix(NewCepstrVarNorm);
      CepstrVarNorm = CreateMatrix(x,FDim,FDim);
      ZeroMatrix(CepstrVarNorm);

      for (i=1;i<=FDim;i++){
         CepstrVarNorm[i][i] = varVector[i];
      }
      
      LinTranQuaProd(NewCepstrVarNorm,cf->MatTran,CepstrVarNorm);

      if (varVector)
         Dispose (&gcheap, varVector);
      varVector = CreateVector(&gcheap,NewFDim);
      ZeroVector(varVector);
      
      for (i=1;i<=NewFDim;i++){
         varVector[i] = NewCepstrVarNorm[i][i];
      }
      
      dim = NewFDim;
   }

   cf->varScaleVector = CreateVector (x, dim);
   CopyVector (varVector, cf->varScaleVector);

   CloseSource (&src);      
}

/* ---------- Parameter File Channel Operations ----------- */

#define CRCC_NONE 65535
#define CRCC_AT_CLOSE 65534
#define CRCC_STREAM 65533

unsigned int UpdateCRCC(void *data,int n,int s,Boolean bSwap,unsigned int crcc)
{
   char *ptr = (char *) data;
   int i;
   unsigned short s1,s2;

   for (i=0;i<n;i++) {
      switch(s) {
      case sizeof(int32):
         s1=*(short*)ptr; ptr+=sizeof(short);
         s2=*(short*)ptr; ptr+=sizeof(short);
         if (bSwap) crcc=(crcc*65536+s2)%36897,crcc=(crcc*65536+s1)%36897;
         else crcc=(crcc*65536+s1)%36897,crcc=(crcc*65536+s2)%36897;
         break;
      case sizeof(short):
         s1=*(short*)ptr; ptr+=sizeof(short);
         crcc=(crcc*65536+s1)%36897;
         break;
      default:
         break;
      }
   }
   return(crcc);
}

static Boolean GetCRCCFrame(ParmBuf pbuf,void *data,int n,int s,Boolean bSwap)
{
   IOConfig cf=pbuf->cf;
   Boolean lastReadValid=FALSE;
   static unsigned short lastReadShort;
   unsigned int crcc=cf->crcc;
   unsigned short *sp,s1,s2;
   int j = 0;

   sp = (unsigned short *) data;
   if (pbuf->crcc==CRCC_NONE || pbuf->crcc==CRCC_AT_CLOSE) {
      switch(s) {
      case sizeof(int32):
         if (!RawReadFloat(&cf->src, (float *) data, n, hparmBin, cf->bSwap)) 
            return(FALSE);
         cf->crcc=UpdateCRCC(data,n,sizeof(int32),cf->bSwap,cf->crcc);
         break;
      case sizeof(short):
         if (!RawReadShort(&cf->src, (short *) data, n, hparmBin, cf->bSwap))
            return(FALSE);
         cf->crcc=UpdateCRCC(data,n,sizeof(short),cf->bSwap,cf->crcc);
         break;
      }
      return(TRUE);
   }
   else {
      /* Try to read two shorts - if the first reads okay and the */
      /*  second doesn't then we have found a crcc. */
      if (lastReadValid) s1=lastReadShort;
      else
         if (!RawReadShort(&cf->src,(short*)&s1,1,hparmBin,cf->bSwap)) {
            if (n!=1 || s!=sizeof(short))
               HError(6350,"GetCRCCFrame: CRCC missing");
            pbuf->crcc=lastReadShort;
            return(FALSE);
         }
      lastReadValid=FALSE;
      if (!RawReadShort(&cf->src,(short*)&s2,1,hparmBin,cf->bSwap)) {
         pbuf->crcc=s1;
         return(FALSE);
      }
      else if (n==1 && s==sizeof(short)) {
         /* We have read two shorts but only need one */
         /*  so we remember it and see if it is CRCC next time !! */
         lastReadShort=s2;lastReadValid=TRUE;
         j=1; *sp++=s1; crcc=(crcc*65536+s1)%36897;
      }
      else
         switch(s) {
         case sizeof(int32):
            if (bSwap) *sp++=s2,*sp++=s1;
            else *sp++=s1,*sp++=s2;
            crcc=(crcc*65536+s1)%36897; crcc=(crcc*65536+s2)%36897;
            for (j=1;j<n;j++) {
               if (!RawReadShort(&cf->src,(short*)&s1,1,hparmBin,cf->bSwap))
                  break;
               if (!RawReadShort(&cf->src,(short*)&s2,1,hparmBin,cf->bSwap))
                  break;
               if (bSwap) *sp++=s2,*sp++=s1;
               else *sp++=s1,*sp++=s2;
               crcc=(crcc*65536+s1)%36897; crcc=(crcc*65536+s2)%36897;
            }
            break;
         case sizeof(short):
            *sp++=s1,*sp++=s2;  
            crcc=(crcc*65536+s1)%36897; crcc=(crcc*65536+s2)%36897;
            for (j=2;j<n;j++) {
               if (!RawReadShort(&cf->src,(short*)&s1,1,hparmBin,cf->bSwap))
                  break;
               *sp++=s1; crcc=(crcc*65536+s1)%36897;
            }
            break;
         }
      cf->crcc=crcc;
      if (j==n) return(TRUE);
      else {
         HError(-6350,"GetCRCCFrame: Incomplete frame read");
         return(FALSE);
      }
   }
}

static int GetParm(ParmBuf pbuf,int nFrame,void *data)
{
   IOConfig cf=pbuf->cf;
   ShortVec s;
   Vector v;
   float *fp;
   int size,j,r,n;

   /* Don't try to read past known end of file */
   if (pbuf->lastRow>=0 && pbuf->inRow>=pbuf->lastRow) return(0);

   r=n=0;
   size=(cf->srcUsed>cf->tgtUsed)?cf->srcUsed:cf->tgtUsed;
   v=CreateVector(&gstack,size);
   s=CreateShortVec(&gstack,size);
   while (r<nFrame && r==n) {
      n++;
      /* DISCRETE starts as shorts and stays as shorts without changing */
      if ((cf->tgtPK&BASEMASK) == DISCRETE) {
         if (GetCRCCFrame(pbuf,data,cf->srcUsed,sizeof(short),cf->bSwap)) r++;
         data=(short*)data+cf->nCols;
      }
      else {
         /* Everything else ends up as floats and can be transformed */
         fp = (float *) data;
         if (pbuf->fShort) {
            /* Compressed HTK and IREFC stored externally as shorts */
            if (GetCRCCFrame(pbuf,s+1,cf->srcUsed,sizeof(short),cf->bSwap)) {
               for (j=1;j<=cf->srcUsed;j++)
                  v[j] = ((float)s[j] + cf->B[j])/cf->A[j];
               r++;
            }
         }
         else {
            /* Otherwise just a vector of floats */
            if (GetCRCCFrame(pbuf,v+1,cf->srcUsed,sizeof(float),cf->bSwap)) r++;
         }
         /* Reset current config parameters to reflect src */
         cf->nUsed = cf->srcUsed; 
         if ((cf->srcPK&BASEMASK) == IREFC)
            cf->curPK = LPREFC | (cf->curPK&~BASEMASK);
         else
            cf->curPK = cf->srcPK;
         cf->curPK = cf->curPK &(~(HASCRCC|HASCOMPX));
         if (r==n && !EqualKind(cf->tgtPK,cf->curPK)) {
            /* Delete any qualifiers which are not required by target */
            DelQualifiers(v+1, cf);
            /* Transform static parms if necessary */
            XformBase(v+1, cf);
         }
         /* Now copy transformed vector */
         for (j=1;j<=cf->nUsed;j++)
            *fp++=v[j];
         data=(float*)data+cf->nCols;
         /* Update kinds */
         cf->nCvrt = cf->nUsed;
         cf->unqPK = cf->curPK;
      }
   }
   if (pbuf->lastRow<0) {
      if (feof(pbuf->cf->src.f)!=0) pbuf->chClear=TRUE;
   }
   else {
      if (pbuf->inRow+r>=pbuf->lastRow) pbuf->chClear=TRUE;
   }
   FreeShortVec(&gstack,s);
   FreeVector(&gstack,v);
   return(r);
}

static int FramesInParm(ParmBuf pbuf)
{
   int n;

   if (pbuf->lastRow<0) {
#ifdef STREAM_PARM_FILES
      long l;
      /* Automagically determine the end of file */
      ioctl(fileno(pbuf->cf->src.f),FIONREAD,&l);
      if (pbuf->cf->srcPK&HASCRCC) l-=2;
      if (pbuf->fShort)
         n = l / (long) (sizeof(short)*pbuf->cf->srcUsed);
      else
         n = l / (long) (sizeof(float)*pbuf->cf->srcUsed);
      if (n<0) n=0;
#else
      n=1; /* Try to read another row every time */
#endif
   }
   else
      n=pbuf->lastRow-pbuf->inRow;

   return(n);
}

/* Open HTK/ESIG parameter read - returns initRows to read */
/*  with -1 indicating this is actually a waveform file */
static ReturnStatus OpenParmChannel(ParmBuf pbuf,char *fname, int *ret_val)
{
   IOConfig cf = pbuf->cf;
   ParmKind tgtBase;
   Boolean isPipe;
   FILE *f;
   long nSamples,sampPeriod;
   int initRows,i, tmp;
   short sampSize,kind;
   char b1[50],b2[50];
   Boolean isEXF;
   char actfname[MAXFNAMELEN];
   long stIndex, enIndex;
   long preskip;
   
   /* map to logical to actual name */
   strncpy (actfname, fname, MAXFNAMELEN);
   isEXF = GetFileNameExt (fname, actfname, &stIndex, &enIndex);
   
   if ((f = FOpen (actfname, ParmFilter, &isPipe)) == NULL) {
      HRError(6310,"OpenParmChannel: cannot open Parm File %s",fname);
      return(FAIL);
   }
   
   if (isEXF && isPipe && stIndex >= 0) {  /* allows mapping, but no segmentation on pipes*/
      HRError (6313, "OpenParmChannel: cannot segment piped input");
      return (FAIL);
   }
   /* Need to turn off buffering to stream file */
#ifdef STREAM_PARM_FILES
   setbuf(f,NULL);
#endif
   switch (cf->srcFF) {
   case HTK:
      if (!ReadHTKHeader(f,&nSamples,&sampPeriod,&sampSize,&kind, &cf->bSwap)){
         HRError(6313,
                 "OpenParmChannel: cannot read HTK Header in File %s",fname);
         return(FAIL);
      }
      break;
   case ESIG:
      if (!ReadESIGPHeader(f, &nSamples, &sampPeriod, &sampSize, &kind, &cf->bSwap, isPipe)){
         HRError(6313,
                 "OpenParmChannel: cannot read ESIG Header in File %s",fname);
         return(FAIL);
      }
      break;
   default:
      HRError(6270,"OpenParmChannel: Cannot read parameterised %s data",
              Format2Str(cf->srcFF));
      return(FAIL);
      break;
   }
   if (kind == WAVEFORM) {
      if (cf->srcPK != ANON && cf->srcPK != WAVEFORM){
         HRError(6320,"OpenParmChannel: src file kind[WAVEFORM] != IOConfig[%s]",
                 ParmKind2Str(cf->srcPK,b1));
         return(FAIL);
      }
      cf->srcPK = WAVEFORM; FClose(f,isPipe);
      *ret_val=-1;
      return(SUCCESS);
   }
   pbuf->chType = ch_hparm;
   AttachSource(f,&cf->src);cf->src.isPipe=isPipe;

   /* If extended segmented file, modify header and computed skip */
   preskip = 0;
   if (isEXF && stIndex >= 0) {
      if (enIndex < 0) 
         enIndex = nSamples-1;
      if (nSamples < enIndex - stIndex + 1) {
         HRError(6313,"OpenParmChannel: EXF segment bigger than file");
         return (FAIL);
      }
      if (nSamples < enIndex + 1) {
         HRError(6313,"OpenParmChannel: EXF segment bigger than file");
         return (FAIL);
      }
      preskip = stIndex * sampSize; 

      nSamples = enIndex - stIndex + 1;
      if (kind & HASCOMPX)      /* the COMPX code below will subtract 4  for A/B */
         nSamples += 4;         
   }
   
   /* if src kind not set then copy from file, otherwise must be identical */
   if (cf->srcPK == ANON)
      cf->srcPK = kind;
   else if (cf->srcPK != kind){
      HRError(6320,"FillTabFromParm: src file kind[%s] != IOConfig[%s]",
              ParmKind2Str(kind,b1),ParmKind2Str(cf->srcPK,b2));
      return(FAIL);
   }
   /* tgt ANON means tgt == src,   tgt ANON_X means tgt == Base(src)_X */
   if (cf->tgtPK == ANON)
      cf->tgtPK = cf->srcPK &(~(HASCRCC|HASCOMPX));
   else if ((cf->tgtPK&BASEMASK) == ANON)
      cf->tgtPK = (cf->tgtPK&~BASEMASK) | (cf->srcPK&BASEMASK);
   /* tgt IREFC should be converted to LPREFC */
   if ((cf->tgtPK&BASEMASK) == IREFC)
      cf->tgtPK = LPREFC | (cf->tgtPK&~BASEMASK);
   if (!ValidConversion(cf->srcPK,cf->tgtPK)){
      HRError(6322,"FillTabFromParm: cannot convert %s to %s",
              ParmKind2Str(cf->srcPK,b1),ParmKind2Str(cf->tgtPK,b2));
      return(FAIL);
   }
   if (trace&T_BUF)
      printf("HParm: Loading data from %s: src=%s/tgt=%s\n",
             fname,ParmKind2Str(cf->srcPK,b1),ParmKind2Str(cf->tgtPK,b2));
   cf->srcSampRate = cf->tgtSampRate = sampPeriod;
   cf->curPK = cf->srcPK;

   cf->nCvrt = cf->nUsed;
   
   if ((cf->srcPK&BASEMASK) == DISCRETE) {
      cf->nCols = cf->srcUsed = cf->tgtUsed = sampSize/sizeof(short);
      pbuf->fShort=TRUE;
   } else {
      if ((cf->srcPK&HASCOMPX) || (cf->srcPK&BASEMASK) == IREFC) {
         cf->srcUsed = sampSize/sizeof(short);
         pbuf->fShort=TRUE;
         cf->A = CreateVector(pbuf->mem,cf->srcUsed); 
         cf->B = CreateVector(pbuf->mem,cf->srcUsed);
         if (cf->srcPK&HASCOMPX) {
            nSamples-=4;
            if (!ReadFloat(&cf->src,cf->A+1,cf->srcUsed,hparmBin)){
               HRError(6313,"OpenParmChannel: Can't read HTK COMPX vector A");
               return(FAIL);
            }
            if (!ReadFloat(&cf->src,cf->B+1,cf->srcUsed,hparmBin)){
               HRError(6313,"OpenParmChannel: Can't read HTK COMPX vector B");
               return(FAIL);
            }
            cf->crcc=UpdateCRCC(cf->A+1,cf->srcUsed,sizeof(float),
                                cf->bSwap,cf->crcc);
            cf->crcc=UpdateCRCC(cf->B+1,cf->srcUsed,sizeof(float),
                                cf->bSwap,cf->crcc);
            cf->curPK -= HASCOMPX;
         }
         else {
            for(i=1;i<=cf->srcUsed;i++) cf->A[i]=32767.0,cf->B[i]=0.0;
            cf->curPK = LPREFC | (cf->curPK&~BASEMASK);
         }
      } else {
         cf->srcUsed = sampSize/sizeof(float);
         pbuf->fShort=FALSE;
      }
      tgtBase = cf->tgtPK&BASEMASK;
      if ((cf->curPK&BASEMASK)!=tgtBase && (tgtBase==LPCEPSTRA || tgtBase==MFCC))
         tmp = TotalComps(cf->numCepCoef,cf->tgtPK);
      else 
         tmp = TotalComps(NumStatic(cf->srcUsed,cf->srcPK),cf->tgtPK);

      if (cf->MatTranFN != NULL) {
         if (cf->preQual)
            cf->tgtUsed = NumRows(cf->MatTran)*(1+HasDelta(cf->tgtPK)+HasAccs(cf->tgtPK)+HasThird(cf->tgtPK));
         else 
            cf->tgtUsed = NumRows(cf->MatTran);
      } else 
         cf->tgtUsed = tmp;
      if (tmp>cf->tgtUsed) cf->nCols=tmp;
      else cf->nCols = cf->tgtUsed;
   }
   /* Cannot use energy sil det from parameter files */
   cf->useSilDet=FALSE;

   /* for extended files skip here, after the A/B vectors for COMPX have been read */
   if (preskip > 0)
      if (fseek (f, preskip, SEEK_CUR) != 0) {
         HError (6313, "OpenParmChannel: error processinf EXF segment");
         return (FAIL);
      }

   if (nSamples>MAX_INT) {
      /* Used streamed file */
      pbuf->main.maxRows = MAX_PB_SIZE;
      pbuf->lastRow = -1; 
      initRows = 0;
   }
   else {
      /* Read in one go as a table */
      initRows = pbuf->main.maxRows = pbuf->lastRow = nSamples;
   }
   /* pbuf->lastRow = -1;  Force incremental read based on ioctl */
   /* initRows = pbuf->main.maxRows = 50; */

   /* Indicate whether doing crcc */
   if (!(pbuf->cf->srcPK & HASCRCC) || (isEXF && stIndex >= 0))
      pbuf->crcc=CRCC_NONE;
   else if (pbuf->lastRow<0)
      pbuf->crcc=CRCC_STREAM;
   else
      pbuf->crcc=CRCC_AT_CLOSE;

   *ret_val=initRows;
   return(SUCCESS);
}

/* ------------------- Channel Operations ------------------- */

/* Return number of frames that can be read without blocking */
/*       -1 == Done, no more to read. */
/*        0 == May block on reading first frame. */
/*        N == Can read N frames immediately without blocking. */
/*  INT_MAX == Will not block. */
static int FramesInChannel(ParmBuf pbuf,int chType)
{
   AudioInStatus as;
   IOConfig cf = pbuf->cf;
   int r=0,n,x;

   switch(chType) {
   case ch_haudio:
      /* Clear once audio clear */
      as = GetAIStatus(pbuf->in.a);
      if (as == AI_ERROR)
         HError(6323,"FramesInChannel: audio device in error state");
      if (as==AI_CLEARED && pbuf->status>PB_INIT)
         r=-1;
      else
         /* Otherwise find real number of frames */
         r=FramesInAudio(pbuf->in.a);
      break;
   case ch_hwave:
      /* Done as soon as no more available */
      if ((r=FramesInWave(pbuf->in.w))<=0)
         r=-1;
      break;
   case ch_hparm:
      /* Two ways to check pbuf */
      if (pbuf->lastRow<0) {
         /* When no previous idea of file length check EOF */
         if (feof(pbuf->cf->src.f)!=0) r=-1;
         else
            r=FramesInParm(pbuf);
      }
      else {
         /* If we know how many frames in total */
         if (pbuf->inRow>=pbuf->lastRow) r=-1;
         else
            r=FramesInParm(pbuf);
      }
      break;
   case ch_hrfe:
      /* Just need to count available frames */
      break;
   case ch_ext_wave:
   case ch_ext_parm:
      /* Count samples and convert to frames */
      n=pbuf->ext->fNumSamp(pbuf->ext->xInfo,pbuf->in.i);
      /* Find out if we need extra samples because this is the first frame */
      if (pbuf->inRow>0) x=0;
      else x=cf->frSize-cf->frRate;
      /* Convert returned value to number of frames */
      if (n<0) /* Could block so avoid it */
         r=(-n-1-x)/cf->frRate,r=(r<0?0:r);
      else if (n==0) /* Channel is clear */
         r=-1;
      else /* Will not block and we want to find end of file */
         r=((n-x)/cf->frRate)+1;
      break;
   }
   if (r==-1) pbuf->chClear=TRUE;
   return(r);
}

/* Get a single frame from particular channel */
/*  Return value indicates number of frames read okay */
static int GetFrameFromChannel(ParmBuf pbuf,int chType,void *vp)
{
   IOConfig cf = pbuf->cf;
   AudioInStatus as;
   int r=0,i,j,x,n;
   double m,e;

   /* Legacy checks for out of data */
   switch(chType) {
   case ch_haudio:
      as = GetAIStatus(pbuf->in.a);
      if (as==AI_CLEARED && pbuf->status>PB_INIT)
         pbuf->chClear=TRUE;
      break;
   case ch_hwave:
      if (FramesInWave(pbuf->in.w)==0) pbuf->chClear=TRUE;
      break;
   case ch_hparm:
      /* Checked in GetParm */
      break;
   }
   if (pbuf->chClear) return(0);

   switch(chType) {
   case ch_haudio:
   case ch_hwave:
   case ch_ext_wave:
      /* Waveform types first */
      switch(chType) {
         /* First get the waveform */
      case ch_haudio:
         GetAudio(pbuf->in.a,1,cf->s+1); r=1;
         as = GetAIStatus(pbuf->in.a);
         if (as==AI_CLEARED && pbuf->status>PB_INIT)
            pbuf->chClear=TRUE;
         break;
      case ch_hwave:
         GetWave(pbuf->in.w,1,cf->s+1); r=1;
         if (FramesInWave(pbuf->in.w)==0) pbuf->chClear=TRUE;
         break;
      case ch_ext_wave:
         /* Copy overlap */
         if (pbuf->inRow>0) {
            n=cf->frRate,x=cf->frSize-cf->frRate;
            for (i=1;i<=x;i++) 
               cf->r[i]=cf->r[i+cf->frRate];
         }
         else n=cf->frSize,x=0;
         /* Get new data */
         if (fGetWaveData(n,cf->rawBuffer,cf->r+x+1,
                          pbuf->ext,pbuf->in.i)==n) r=1;
         else r=0;
         /* Copy to float buffer */
         for (j=1;j<=cf->frSize;j++) cf->s[j]=cf->r[j];
         break;
      }
      if (r==0) break;
      /* Calc frame energy 0.0-100dB */
      for (j=1,m=e=0.0;j<=cf->frSize;j++) {
         x=(int) cf->s[j];
         m+=x;e+=x*x;
      }
      m=m/cf->frSize;e=e/cf->frSize-m*m;
      if (e>0.0) e=10.0*log10(e/0.32768);
      else e=0.0;
      cf->curVol = e;

      if (pbuf->spVal!=NULL)
         pbuf->spVal[pbuf->main.nRows] = e;

      /* Reset current nUsed/PK to indicate results of conversion */
      cf->nUsed = cf->nCvrt; cf->curPK = cf->tgtPK&BASEMASK;
      /* Then convert it to a frame */
      if (ConvertFrame(cf, (float *) vp) != cf->nCvrt)
         HError(6391,"GetFrameFromChannel: convert count != %d",cf->nCvrt);
      /* Update kinds */
      cf->nCvrt = cf->nUsed; cf->unqPK = cf->curPK;
      r=1;
      break;
   case ch_hparm:
      r=GetParm(pbuf,1,vp);
      break;
   case ch_ext_parm:
      break;
   case ch_hrfe:
      /* Just need to read in the converted frame */
      break;
   }
   return(r);
}

/* ------------ Read and Convert Data from Channel Input ------------ */

/* FillBufFromChannel: fill buffer from channel input  */
/*  if minRows>0 ensures that after the call returns at least minRows */
/*  valid rows are available otherwise just reads and qualifies those */
/*  that can be read immediately (without blocking) */
static void FillBufFromChannel(ParmBuf pbuf,int minRows)
{
   IOConfig cf = pbuf->cf;
   PBlock *pb,*lb;
   Boolean dis,cleared;
   char b1[100];
   int availRows,newRows,space,i,head,tail,nShift;
   short *sp1=NULL, *sp2;
   float *fp1=NULL, *fp2;
   
   if ((trace&T_BUF) && minRows>0) {
      printf("HParm: Filling Parm Buf: max=%d, in=%d, out=%d, qst=%d, min=%d\n",
             pbuf->main.maxRows, pbuf->inRow,pbuf->outRow,pbuf->qst,minRows);
      fflush(stdout);
   }

   /* Fill Buffer with converted static coef vectors */
   newRows=FramesInChannel(pbuf,pbuf->chType);

   /* Number of rows that we can read immediately */
   availRows = pbuf->spDetFin - pbuf->outRow + 1;
   if (minRows>0) {
      /* If minRows is > 0 we must return with a valid (fully qualified) */
      /*  obs available (although this may be discarded by sil detector */
      if (pbuf->main.nRows<pbuf->qwin) minRows+=pbuf->qwin-pbuf->main.nRows;
      if (availRows+newRows<minRows)  /* Try to get minRows qualified frames */
         newRows = minRows-availRows;
   }
   /* Check that there is enough space in the buffer */
   space = pbuf->main.maxRows - ((pbuf->main.nRows<pbuf->minRows) ?
                                 pbuf->main.nRows : pbuf->minRows);
   if (newRows > space ) newRows=space;
   if ((newRows > pbuf->main.maxRows - pbuf->main.nRows) || (
                                                             pbuf->spDetSt>pbuf->main.stRow))
      do {
         /* Move existing rows to extensible buffer */
         /* nShift=newRows-(pbuf->main.maxRows-pbuf->main.nRows);  Min shift */
         
         /* Work out if we need to keep the data */
         if (pbuf->spDetSt>pbuf->main.stRow &&
             pbuf->main.next==NULL) {
            nShift=pbuf->spDetSt-pbuf->main.stRow;
            if (nShift>pbuf->main.nRows-pbuf->minRows)
               nShift=pbuf->main.nRows-pbuf->minRows;
            if (nShift<=0) break;
            dis=TRUE;
         }
         else {
            nShift=pbuf->main.nRows-pbuf->minRows; /* Max shift */
            dis = ((pbuf->outRow-nShift>pbuf->main.stRow) && 
                   pbuf->main.next==NULL && pbuf->noTable);
         }
         if (trace&T_BUF)
            printf("%s %d from %d/%d @%d\n",(dis?"Discarding":"Moving"),
                   nShift,pbuf->main.stRow,pbuf->inRow,pbuf->spDetSt);
         if (!dis) {
            /* If we do - first find some space */
            for (pb=pbuf->main.next;pb!=NULL;pb=pb->next)
               if (pb->nRows<pb->maxRows) break;
            if (pb==NULL) {
               /* Allocate new buffer */
               pb=(PBlock *) New(pbuf->mem,sizeof(PBlock));
               pb->stRow=pbuf->main.stRow; pb->nRows=0;
               pb->maxRows=nShift;
               if (pb->maxRows<MIN_PB_SIZE) pb->maxRows=MIN_PB_SIZE;
               if (pbuf->dShort)
                  pb->data=New(pbuf->mem,sizeof(short)*pb->maxRows*cf->nCols);
               else
                  pb->data=New(pbuf->mem,sizeof(float)*pb->maxRows*cf->nCols);
               /* Add this to end of list */
               pb->next=NULL;
               if (pbuf->main.next==NULL) pbuf->main.next=pb;
               else
                  for (lb=pbuf->main.next;lb!=NULL;lb=lb->next) 
                     if (lb->next==NULL) {
                        lb->next=pb;
                        break;
                     }
            }
            /* Check the space in the PBlock */
            if (nShift>pb->maxRows-pb->nRows) nShift=pb->maxRows-pb->nRows;
            /* Transfer oldest data to new buffer */
            if (pbuf->dShort) {
               sp2 = (short *)pbuf->main.data;  /* From start of main buffer */
               sp1 = (short *)pb->data + pb->nRows*cf->nCols; /* to end of pb */
               memcpy(sp1,sp2,nShift*cf->nCols*sizeof(short));
            }
            else {
               fp2 = (float *)pbuf->main.data;  /* From start of main buffer */
               fp1 = (float *)pb->data + pb->nRows*cf->nCols; /* to end of pb */
               memcpy(fp1,fp2,nShift*cf->nCols*sizeof(float));
            }
            pb->nRows+=nShift;
         }
         /* Shift newer data backwards - note memory may overlap */
         if (pbuf->dShort) {
            sp1 = (short *)pbuf->main.data; sp2 = sp1 + nShift*cf->nCols;
            memmove(sp1,sp2,(pbuf->main.nRows-nShift)*cf->nCols*sizeof(short));
         }
         else {
            fp1 = (float *)pbuf->main.data; fp2 = fp1 + nShift*cf->nCols;
            memmove(fp1,fp2,(pbuf->main.nRows-nShift)*cf->nCols*sizeof(float));
         }

         if (pbuf->spVal!=NULL) {
            fp1 = pbuf->spVal; fp2 = fp1 + nShift;
            memmove(fp1,fp2,(pbuf->main.nRows-nShift)*sizeof(float));
         }
         
         /* Rebase everything so that still points to correct point */
         pbuf->main.stRow+=nShift; pbuf->main.nRows-=nShift;
         pbuf->qst-=nShift; pbuf->qen-=nShift;
      }
      while ( newRows > pbuf->main.maxRows - pbuf->main.nRows);
   
   CheckBuffer(pbuf); /* Make sure update buffer status */
   /* Unless we have just finished don't get zero rows */
   if (newRows<=0 && !pbuf->chClear) return; 
   /* If we need to calibrate on the buffer don't read anything */
   /*  until the required number of calibration frames are available */
   if (!pbuf->chan->spDetParmsSet && pbuf->cf->useSilDet &&
       pbuf->cf->selfCalSilDet!=0 && !pbuf->chClear &&
       (newRows+pbuf->main.nRows)<abs(pbuf->cf->selfCalSilDet)) return;

   if (trace&T_BUF){
      printf("HParm:  coding %d frames into %s parm kind\n",newRows,
             ParmKind2Str(cf->curPK,b1));
      fflush(stdout);
   }


   if (pbuf->dShort) 
      sp1 = (short*) pbuf->main.data + pbuf->main.nRows*cf->nCols;
   else
      fp1 = (float*) pbuf->main.data + pbuf->main.nRows*cf->nCols;

   /* Read the necessary frames */
   for (i=0; i<newRows; i++) {
      /* But have final check on read just in case */
      if (pbuf->dShort) {
         if (GetFrameFromChannel(pbuf,pbuf->chType,sp1)!=1) {
            pbuf->chClear=TRUE;
            break;
         }
         sp1 += cf->nCols; 
      }
      else {
         if (GetFrameFromChannel(pbuf,pbuf->chType,fp1)!=1) {
            pbuf->chClear=TRUE;
            break;
         }
         fp1 += cf->nCols; 
      }
      pbuf->inRow++;pbuf->main.nRows++;
   }

   /* Make sure we mark the buffer if we have consumed all input */
   CheckBuffer(pbuf);

   /* Check if this is these rows include the first few */
   if (pbuf->qst<pbuf->qwin) head=pbuf->qst; else head=pbuf->qwin;
   /* And then check if we have reached the end */
   if (pbuf->status>=PB_STOPPED || pbuf->chClear)
      tail=0,cleared=TRUE; 
   else tail=pbuf->qwin,cleared=FALSE;
   /* Check to make sure can do some qualifiers */
   /*  Note pbuf->in should always remain > pbuf->qwin during block moves */
   if (cleared || pbuf->main.nRows-tail-1>=0) {
      /* Find the right bit of data and qualify the block that we can */
      pbuf->qen=pbuf->main.nRows-tail-1;
      fp1=(float *)pbuf->main.data + pbuf->qst*cf->nCols;

      /* Hack to allow ENORMALISE on all in one wave file !! */
      if (!(cf->srcPK&HASENERGY) && (cf->tgtPK&HASENERGY) && cf->eNormalise) {
         if (head==0 && cleared)
            NormaliseLogEnergy(fp1 + cf->nUsed-1, pbuf->qen-pbuf->qst+1,
                               cf->nCols, cf->silFloor, cf->eScale);
         else
            HError(6320,"FillBufFromChannel: Cannot normalise energy");
      }

      /* Reset current nUsed/PK to indicate results of conversion */
      cf->nUsed = cf->nCvrt; cf->curPK = cf->unqPK;

      AddQualifiers(pbuf,fp1,pbuf->qen-pbuf->qst+1,cf,head,tail);
      /* Assume session adaptation now done */
      pbuf->chan->oCnt+=pbuf->qen-pbuf->qst+1;
      /* Set qst for the next time */
      pbuf->qst=pbuf->qen+1;
   }

   /* Silence detector can now read up to pbuf->qen valid qualified frames */
   /* Silence detector works by setting three values */
   /*  Once speech is detected */
   /*   spDetSt should be set to (absolute) first row to read */
   /*   spDetFin should be maintained as the last row allowed to be read */
   /*  Until silence is detected and */
   /*   spDetEn should be set to last+1 row of speech (spDetFin+1) */
   if (!pbuf->chan->spDetParmsSet && pbuf->cf->useSilDet && 
       pbuf->cf->selfCalSilDet!=0)
      SetSelfCalSpDetParms(pbuf);
   if (cf->useSilDet)
      RunSilDet(pbuf,cleared);
   else {
      pbuf->spDetFin=pbuf->qen+pbuf->main.stRow;
      /* Mark end of utterance */
      if (cleared) pbuf->spDetEn=pbuf->spDetFin+1;
   }
   if (pbuf->status>=PB_STOPPED || pbuf->chClear) cf->curVol=0.0;

   if (trace&T_BUF){
      printf("HParm: Parm Buf Filled: in=%d, out=%d, qen=%d, nused=%d\n",
             pbuf->inRow,pbuf->outRow,pbuf->qen,cf->nUsed);
      fflush(stdout);
   }
}

/* OpenAsChannel: open and create an audio input buffer */
static ReturnStatus OpenAsChannel(ParmBuf pbuf, int maxObs, 
                                  char *fname, FileFormat ff,
                                  TriState silMeasure)
{
   ChannelType chType;
   BufferInfo info;
   int initRows;
   long dBytes;
   char b1[50];
   IOConfig cf = pbuf->cf;

   /* First determine channel type */
   /* Determine source ie wave or parm file & fill table */
   if (pbuf->ext!=NULL)
      chType=ch_ext_wave;
   else if (fname==NULL || ff==HAUDIO || 
            (ff==UNDEFF && pbuf->cf->srcFF==HAUDIO))
      chType = ch_haudio;
   else if ((cf->srcFF == HTK || cf->srcFF == ESIG ) && cf->srcPK != WAVEFORM)
      chType=ch_hparm;
   else 
      chType=ch_hwave;

   /* Set up config to reflect call */
   if (ff != UNDEFF) cf->srcFF = ff;

   if (chType==ch_hwave || chType==ch_haudio) {
      if (cf->srcPK == ANON) cf->srcPK = WAVEFORM;
      /* Waveform checks */
      if (cf->srcPK != WAVEFORM){
         HRError(6320,"OpenAsChannel: IOConfig src not set for waveform input");
         return(FAIL);
      }
      if (cf->tgtPK == ANON || cf->tgtPK == WAVEFORM ){
         HRError(6320,"OpenAsChannel: IOConfig tgt not a parameter kind");
         return(FAIL);
      }
   }

   /* Channel parameters */
   pbuf->noTable=TRUE;
   pbuf->crcc=CRCC_NONE; /* Only HParm files have CRCC */

   pbuf->main.next=NULL;
   pbuf->outRow=pbuf->inRow=pbuf->main.stRow=0;pbuf->main.nRows=0;
   pbuf->qst=0;pbuf->qen=-1;pbuf->lastRow=-1;
   pbuf->qwin=0; /* set qwin wide enough to allow 1 computable frame */
   if (cf->tgtPK&HASDELTA) {
      if (cf->tgtPK&HASACCS) {
      pbuf->qwin += cf->accWin;
      if(cf->tgtPK&HASTHIRD) pbuf->qwin += cf->thirdWin;
      }
    pbuf->qwin += cf->delWin;
   }
   /* Open the input */
   switch(chType) {
   case ch_haudio:
      pbuf->chType = ch_haudio;
      /* If necessary calibrate live audio silence detector */
      if (cf->useSilDet)
         SetSilDetParms(pbuf, silMeasure);
      if ((pbuf->in.a = OpenAudioInput(pbuf->mem,&(cf->srcSampRate), 
                                       cf->winDur, cf->tgtSampRate)) == NULL){
         HRError(6306,"OpenAsChannel: Audio input not supported");
         return(FAIL);
      }
      /* Setup remaining size information in IOConfig record */
      ValidCodeParms(cf);
      cf->frSize = SampsInAudioFrame(pbuf->in.a); 
      cf->frRate = (int) (cf->tgtSampRate/cf->srcSampRate);
      SetUpForCoding(pbuf->mem,cf,cf->frSize);

      initRows=0;
      pbuf->main.maxRows = maxObs;
      break;
   case ch_hparm:
      if(OpenParmChannel(pbuf,fname, &initRows)<SUCCESS){
         HRError(6313,"OpenAsChannel: OpenParmChannel failed");
         return(FAIL);
      }
      /* Fall through if waveform file */
      if (initRows>=0) break;
   case ch_hwave:
      pbuf->chType = ch_hwave;
      /* May fall through from ch_hparm !! */
      if((pbuf->in.w = OpenWaveInput(pbuf->mem,fname,cf->srcFF,cf->winDur,
                                     cf->tgtSampRate,&(cf->srcSampRate)))==NULL){
         HRError(6313,"OpenAsChannel: OpenWaveInput failed");
         return(FAIL);
      }
      /* Setup remaining size information in IOConfig record */
      ValidCodeParms(cf);
      cf->frSize = SampsInWaveFrame(pbuf->in.w); 
      cf->frRate = (int) (cf->tgtSampRate/cf->srcSampRate);
      SetUpForCoding(pbuf->mem,cf,cf->frSize);

      pbuf->main.maxRows = FramesInWave(pbuf->in.w);
      GetWaveDirect(pbuf->in.w,&(cf->nSamples));  /* just to get nSamples */

      /* If necessary calibrate waveform silence detector on 
         first file in list of files to process */
      if (cf->useSilDet)
         SetSilDetParms(pbuf, silMeasure);

      initRows = pbuf->lastRow = pbuf->main.maxRows;
      /* pbuf->lastRow = -1;  Force incremental read based on ioctl */
      /* initRows = pbuf->main.maxRows = 50;  In small blocks */
      break;
   case ch_hrfe:
      break;
   case ch_ext_wave:
      if ((pbuf->ext->pk&BASEMASK)==WAVEFORM) 
         pbuf->chType = ch_ext_wave;
      else
         pbuf->chType = ch_ext_parm;
      /* Set up info first */
      if (pbuf->ext->sampPeriod!=0.0 && cf->srcSampRate!=0.0 &&
          pbuf->ext->sampPeriod!=cf->srcSampRate)
         HRError(-6371,"OpenAsChannel: External sample rate does not match configuration");
      if (cf->srcSampRate==0.0) 
         cf->srcSampRate=pbuf->ext->sampPeriod;
      /* Setup remaining size information in IOConfig record */
      ValidCodeParms(cf);
      cf->frSize = (int) (cf->winDur/cf->srcSampRate);
      cf->frRate = (int) (cf->tgtSampRate/cf->srcSampRate);
      SetUpForCoding(pbuf->mem,cf,cf->frSize);
      cf->rawBuffer=(char *) New(pbuf->mem,cf->frSize*(pbuf->ext->size&0xff));
      /* Call user defined Open routine */
      GetBufferInfo(pbuf,&info);
      pbuf->in.i = pbuf->ext->fOpen(pbuf->ext->xInfo,fname,&info);
      /* May need to calibrate */
      if (cf->useSilDet)
         SetSilDetParms(pbuf, silMeasure);
      /* Treat as buffer */
      initRows=0;
      pbuf->main.maxRows = maxObs;
      break;
   default:
      HRError(6320,"OpenAsChannel: Channel type not yet supported");
      return(FAIL);
   }


   pbuf->spDetFin=-1; /* Cannot return anything yet */
   if (cf->useSilDet) {
      pbuf->spDetSt=MAX_INT;pbuf->spDetEn=MAX_INT;pbuf->spDetLst=MAX_INT;
      pbuf->spDetCur=pbuf->spDetCnt=pbuf->silDetCnt=0;
      pbuf->minRows=cf->spcSeqCount+cf->marginCount;
      if (pbuf->minRows<cf->silSeqCount-cf->marginCount)
         pbuf->minRows=cf->silSeqCount-cf->marginCount;
      pbuf->minRows+=1+2*pbuf->qwin; /* Always need qual window */
   }
   else {
      pbuf->spDetSt=0;pbuf->spDetEn=MAX_INT;  /* No silence detector */
      pbuf->minRows=1+2*pbuf->qwin;     /* Will need more for sil det */
   }
   if (pbuf->main.maxRows<=pbuf->minRows+2*pbuf->qwin) 
      pbuf->main.maxRows=pbuf->minRows+2*pbuf->qwin+1;

   /* Allocate main data buffer */
   if ((cf->tgtPK&BASEMASK) == DISCRETE) {
      pbuf->dShort=TRUE;
      dBytes = cf->nCols * pbuf->main.maxRows * sizeof(short);
   }
   else {
      pbuf->dShort=FALSE;
      dBytes = cf->nCols * pbuf->main.maxRows * sizeof(float);
   }
   pbuf->main.data = New(pbuf->mem,dBytes);

   if (cf->useSilDet) 
      pbuf->spVal = (float *) New(pbuf->mem,sizeof(float)*pbuf->main.maxRows);
   else
      pbuf->spVal = NULL;
   

   if (pbuf->lastRow<0) {
      if (cf->tgtPK&HASZEROM){
         HRError(6320,"OpenAsChannel: cannot zero mean within buffer");
         return(FAIL);
      }
      if ((cf->tgtPK&HASENERGY) && cf->eNormalise){
         HRError(6320,"OpenAsChannel: cannot normalise energy via buffer");
         return(FAIL);
      }
   }
   
   if (trace&T_BUF){
      printf("HParm: Channel Opened: curPK=%s; nCols=%d; nUsed=%d; tgtUsed=%d;\n",
             ParmKind2Str(cf->curPK,b1),cf->nCols,cf->nUsed,cf->tgtUsed);
      printf("HParm:                 mRows=%d; initRows=%d; qwin=%d; lastRow=%d\n",
             pbuf->main.maxRows,initRows,pbuf->qwin,pbuf->lastRow);
      fflush(stdout);
   }

   ChangeState(pbuf,PB_INIT);
   if (maxObs==0) {
      /* maxObs==0 indicates want a table straight away */
      StartBuffer(pbuf);
      while(pbuf->status<PB_STOPPED)
         FillBufFromChannel(pbuf,MAX_INT);
   }
   else if (initRows>0) {
      /* This is called for ParmBuf and HWave input */
      StartBuffer(pbuf);
      /* Otherwise just read what we can get initially */
      FillBufFromChannel(pbuf,initRows);
   }
   return(SUCCESS);
}

/* EXPORT->OpenBuffer: open and return an input buffer */
ParmBuf OpenBuffer(MemHeap *x, char *fn, int maxObs, FileFormat ff, 
                   TriState enSpeechDet, TriState silMeasure)
{
   ParmBuf pbuf;

   if (x->type != MSTAK) {
      HRError(6316,"OpenBuffer: memory must be an MSTAK");   
      return(NULL);
   }
   pbuf = (ParmBuf)New(x,sizeof(ParmBufRec));
   pbuf->mem = x; pbuf->status = PB_INIT;
   pbuf->chan = curChan; pbuf->ext=NULL; pbuf->chClear=FALSE;
   pbuf->cf = MakeIOConfig(pbuf->mem, pbuf->chan);
   if (enSpeechDet!=TRI_UNDEF) pbuf->cf->useSilDet=(Boolean)enSpeechDet;
   if (pbuf->cf->addDither>0.0) RandInit(12345);

   /* side based normalisation -- #### should maybe be in OpenAsChannel? */
   /* Load mean vector into pbuf->cf */
   if (HasZerom (pbuf->cf->tgtPK) && !HasZerom (pbuf->cf->srcPK) && 
       (pbuf->cf->cMeanDN || pbuf->cf->cMeanMask))
      LoadCMeanVector (pbuf->mem, pbuf->cf, fn);
   
   /* Load variance estimate into pbuf->cf */
   if (pbuf->cf->varScaleDN || pbuf->cf->varScaleMask) {
      LoadVarScaleVector (pbuf->mem, pbuf->cf, fn);
   }

   /* Load xform associated with this side if necessary */
   if (pbuf->cf->sideXFormMask != NULL) {
      pbuf->cf->sideXForm = LoadSideXForm(pbuf->cf,fn);
   }

   if(OpenAsChannel(pbuf,maxObs,fn,ff,silMeasure)<SUCCESS){
      Dispose(x, pbuf);
      HRError(6316,"OpenBuffer: OpenAsChannel failed");   
      return(NULL);
   }

   return pbuf;
}

/* EXPORT->OpenExtBuffer: open and return an input buffer */
ParmBuf OpenExtBuffer(MemHeap *x, char *fn, int maxObs, 
                      FileFormat ff, HParmSrcDef ext, 
                      TriState enSpeechDet, TriState silMeasure)
{
   ParmBuf pbuf;
   
   if (x->type != MSTAK) {
      HRError(6316,"OpenBuffer: memory must be an MSTAK");   
      return(NULL);
   }
   pbuf = (ParmBuf)New(x,sizeof(ParmBufRec));
   pbuf->mem = x; pbuf->status = PB_INIT;
   pbuf->chan = curChan; pbuf->ext=ext; pbuf->chClear=FALSE;
   pbuf->cf = MakeIOConfig(pbuf->mem, pbuf->chan);
   if (enSpeechDet!=TRI_UNDEF) pbuf->cf->useSilDet=(Boolean)enSpeechDet;
   if (pbuf->cf->addDither>0.0) RandInit(12345);

   if(OpenAsChannel(pbuf,maxObs,fn,ff,silMeasure)<SUCCESS){
      Dispose(x, pbuf);
      HRError(6316,"OpenBuffer: OpenAsChannel failed");   
      return(NULL);
   }   
   return pbuf;
}

/* EXPORT->CreateSrcExt: open and return input buffer using extended source */
HParmSrcDef CreateSrcExt(Ptr xInfo, ParmKind pk, int size, HTime sampPeriod, 
                         Ptr (*fOpen)(Ptr xInfo,char *fn,BufferInfo *info),
                         void (*fClose)(Ptr xInfo,Ptr bInfo),
                         void (*fStart)(Ptr xInfo,Ptr bInfo),
                         void (*fStop)(Ptr xInfo,Ptr bInfo),
                         int (*fNumSamp)(Ptr xInfo,Ptr bInfo),
                         int (*fGetData)(Ptr xInfo,Ptr bInfo,int n,Ptr data))
{
   HParmSrcDef ext;
   
   ext=(HParmSrcDef) New(&gcheap,sizeof(HParmSrcDefRec));
   ext->xInfo=xInfo; ext->pk=pk; ext->size=size; ext->sampPeriod=sampPeriod; 
   ext->fOpen=fOpen;ext->fClose=fClose;
   ext->fStart=fStart;ext->fStop=fStop;
   ext->fNumSamp=fNumSamp;ext->fGetData=fGetData;
   
   return(ext);
}


/* EXPORT->StartBuffer: start audio and fill the buffer */
void StartBuffer(ParmBuf pbuf)
{
   IOConfig cf = pbuf->cf;

   switch(pbuf->chType) {
   case ch_haudio:
      if (pbuf->status != PB_INIT)
         HError(6324,"StartBuffer: Attempt to start uninitialised buffer");
      StartAudioInput(pbuf->in.a,cf->audSignal);
      break;
   case ch_hwave:
   case ch_hparm:
   case ch_hrfe:
      break;
   case ch_ext_wave:
   case ch_ext_parm:
      /* Call user defined Start routine */
      if (pbuf->ext->fStart!=NULL)
         pbuf->ext->fStart(pbuf->ext->xInfo,pbuf->in.i);
      break;
   }
   if (pbuf->status == PB_INIT) {
      if (pbuf->cf->useSilDet) ChangeState(pbuf,PB_WAITING); 
      else ChangeState(pbuf,PB_FILLING); 
   }
   pbuf->chan->fCnt++;
   pbuf->chan->sCnt++;
}

/* EXPORT->StopBuffer: stop audio and let the buffer empty */
void StopBuffer(ParmBuf pbuf)
{
   switch(pbuf->chType) {
   case ch_haudio:
      StopAudioInput(pbuf->in.a);
      break;
   case ch_hwave:
   case ch_hparm:
   case ch_hrfe:
      break;
   case ch_ext_wave:
   case ch_ext_parm:
      /* Call user defined Stop routine */
      if (pbuf->ext->fStop!=NULL)
         pbuf->ext->fStop(pbuf->ext->xInfo,pbuf->in.i);
      break;
   }
   /* Status will go to PB_STOPPED when source buffer is cleared */
}

/* EXPORT->CloseBuffer: close given ParmBuf object */
void CloseBuffer(ParmBuf pbuf)
{
   unsigned short crcc;

   switch(pbuf->chType) {
   case ch_haudio:
      CloseAudioInput(pbuf->in.a); /* Deletes quite alot of pbuf as well */
      break;
   case ch_hwave:
      CloseWaveInput(pbuf->in.w); /* Deletes quite alot of pbuf as well */
      break;
   case ch_hparm:
      if (pbuf->crcc!=CRCC_NONE) {
         if (pbuf->crcc!=CRCC_AT_CLOSE) crcc=pbuf->crcc;
         else if (!RawReadShort(&pbuf->cf->src,(short*)&crcc,1,
                                hparmBin,pbuf->cf->bSwap))
            crcc=CRCC_NONE; /* Guaranteed crcc error */
         if (crcc!=pbuf->cf->crcc)
            HError(6350,"CloseBuffer: Crc error");
      }
      FClose(pbuf->cf->src.f,pbuf->cf->src.isPipe);
      break;
   case ch_hrfe:
      break;
   case ch_ext_wave:
   case ch_ext_parm:
      pbuf->ext->fClose(pbuf->ext->xInfo,pbuf->in.i);
      break;
   }
   Dispose(pbuf->mem,pbuf);
}

/* EXPORT->ObsInBuffer: Return number of observations currently in buffer */
int ObsInBuffer(ParmBuf pbuf)
{
   if (pbuf->status)
      CheckAndFillBuffer(pbuf);
   /* if speech detector yet to trigger return 0 */
   if (pbuf->outRow<pbuf->spDetSt) return(0); 
   /* This has been altered to allow for the matrix transformation stuff */
   /*
     return (pbuf->spDetFin - pbuf->outRow + 1);
   */
   pbuf->spDetEn = pbuf->main.nRows;
   return (pbuf->main.nRows);
}

/* EXPORT->BufferStatus: Return current status of buffer */
PBStatus BufferStatus(ParmBuf pbuf)
{
   CheckAndFillBuffer(pbuf);
   return pbuf->status;
}

static void ReadObs(ParmBuf pbuf, int outRow,Observation *o)
{
   int i,numS;
   float *fp,*data;
   short *sp;
   PBlock *pb;
   char b1[50],b2[50];

   
   if (outRow>pbuf->spDetFin || outRow<pbuf->spDetSt)
      HError(6375,"ReadObs: Index (%d) out of range (%d..%d)",
             outRow,pbuf->spDetSt,pbuf->spDetFin);
   if (!EqualKind(o->bk,pbuf->cf->tgtPK))
      HError(6373,"ReadObs: Obs kind=%s but buffer kind=%s",
             ParmKind2Str(o->bk,b1),ParmKind2Str(pbuf->cf->curPK,b2));
                  
   numS = o->swidth[0];
   if (outRow>=pbuf->main.stRow) {
      data=(float *) pbuf->main.data;
      i=outRow-pbuf->main.stRow;
   } else {
      for (pb=pbuf->main.next;pb!=NULL;pb=pb->next)
         if (pb->stRow+pb->nRows>outRow) break;
      if (pb==NULL) 
         HError(6395,"ReadObs: Frame discarded from buffer");
      data=(float *) pb->data;
      i=outRow-pb->stRow;
   }
   if ((o->bk&BASEMASK) == DISCRETE){
      sp = (short *)data + i*pbuf->cf->nCols;
      for (i=1; i<=numS; i++)
         o->vq[i] = *sp++;
   } else if ((o->pk&BASEMASK) == DISCRETE) {
      fp = (float *)data + i*pbuf->cf->nCols;
      ExtractObservation(fp,o);
      GetVQ(pbuf->cf->vqTab, numS, o->fv, o->vq);
   } else {
      fp = (float *)data + i*pbuf->cf->nCols;
      ExtractObservation(fp,o);
      if (o->pk&HASVQ) 
         GetVQ(pbuf->cf->vqTab, numS, o->fv, o->vq);
   }
}

/* EXPORT->ReadAsBuffer: Get next observation from buffer */
Boolean ReadAsBuffer(ParmBuf pbuf, Observation *o)
{
   CheckBuffer(pbuf);
   if (pbuf->status<=PB_FILLING) {
      /* Force read when finally necessary */
      do
         FillBufFromChannel(pbuf,pbuf->outRow-pbuf->spDetFin);
      /* Carry on reading until frame is available */
      while(pbuf->status<PB_STOPPED && pbuf->outRow>pbuf->spDetFin);
   }
   /* If forced read failed to obtain observation - indicate failure */
   if (pbuf->status>PB_FILLING && pbuf->outRow>pbuf->spDetFin)
      return(FALSE);
   ReadObs(pbuf,pbuf->outRow,o);
   pbuf->outRow++;
   CheckBuffer(pbuf);
   return(TRUE);
}

/* EXPORT->ReadAsTable: return index'th observation in pbuf */
void ReadAsTable(ParmBuf pbuf, int index, Observation *o)
{
   if (pbuf->status!=PB_STOPPED && pbuf->status!=PB_CLEARED)
      HError(6375,"ReadAsTable: Must let buffer stop before reading");
   ReadObs(pbuf,index+pbuf->spDetSt,o);
   CheckBuffer(pbuf);
}


/* ------------------ Other buffer operations ------------------- */

/* EXPORT->GetBufferInfo: Get info associated with pbuf */
void GetBufferInfo(ParmBuf pbuf, BufferInfo *info)
{
   ChannelInfo *chan;
   IOConfig cf;

   if (pbuf!=NULL) {
      chan=pbuf->chan,cf=pbuf->cf;
      CheckAndFillBuffer(pbuf);
   }
   else chan=curChan, cf=&curChan->cf;
   
   info->srcPK       = cf->srcPK;
   info->srcFF       = cf->srcFF;
   info->srcSampRate = cf->srcSampRate;
   info->nSamples    = cf->nSamples;
   info->frSize      = cf->frSize;
   info->frRate      = cf->frRate;
   info->tgtPK       = cf->tgtPK;
   info->tgtFF       = cf->tgtFF;
   info->tgtSampRate = cf->tgtSampRate;
   info->a=NULL;info->w=NULL;info->nObs=-1;
   info->useSilDet   = cf->useSilDet;
   info->audSignal   = cf->audSignal;
   info->vqTabFN     = cf->vqTabFN;
   info->srcVecSize  = cf->srcUsed;
   info->tgtVecSize  = cf->tgtUsed;
   /* adjust the target vector size when _N used */
   if ((cf->tgtPK&HASNULLE) && (cf->tgtPK&(HASENERGY|HASZEROC)))
      info->tgtVecSize -= 1;
   info->saveCompressed = cf->saveCompressed;
   info->saveWithCRC = cf->saveWithCRC;
   info->matTranFN = cf->MatTranFN;
   info->xform = cf->xform;

   /* Fake spDetParmsSet to make self calibrating appear always set */
   info->spDetParmsSet=(chan->spDetParmsSet||(cf->selfCalSilDet!=0));
   info->spDetSil=chan->spDetSil;
   info->chPeak=chan->chPeak;
   info->spDetSp=chan->spDetSp;
   info->spDetThresh=chan->spDetThresh;
   
   info->curVol=cf->curVol;

   info->a=NULL;info->w=NULL;info->i=NULL;
   if (pbuf!=NULL) {
      if (pbuf->status==PB_STOPPED || pbuf->status==PB_CLEARED)
         info->nObs=pbuf->spDetEn-pbuf->spDetSt;
      switch(pbuf->chType) {
      case ch_haudio: info->a = pbuf->in.a; break;
      case ch_hwave:  info->w = pbuf->in.w; break;
      case ch_hparm:  break;
      case ch_hrfe:   break;
      case ch_ext_wave:
      case ch_ext_parm: info->i = pbuf->in.i; break;
      }
      info->spDetSt=pbuf->spDetSt;
      info->spDetEn=pbuf->spDetEn;
   }
}

/* ---------------- Writing buffer functions ------------------ */

/* EXPORT->EmptyBuffer: Open and return an empty ParmBuf object */
ParmBuf EmptyBuffer(MemHeap *x, int size, Observation o, BufferInfo info)
{
   ParmBuf pbuf;
   long dBytes;
   int s;
   IOConfig cf;
   char b1[50],b2[50];

   if (x->type != MSTAK) 
      HError(6316,"EmptyBuffer: memory must be an MSTAK");
   if (o.pk != info.tgtPK)
      HError(6373,"EmptyBuffer: obs pk[%s] != info tgtpk[%s]",
             ParmKind2Str(o.pk,b1),ParmKind2Str(info.tgtPK,b2));
   if (o.bk & HASNULLE)
      HError(6373,"EmptyBuffer: obs bk[%s] != info tgtpk[%s]",
             ParmKind2Str(o.bk,b1),ParmKind2Str(info.tgtPK,b2));
   
   /* Set up ParmBuf */
   pbuf = (ParmBuf)New(x,sizeof(ParmBufRec));
   pbuf->mem = x;    pbuf->status = PB_STOPPED;
   pbuf->main.nRows = 0; pbuf->main.next=NULL;
   pbuf->inRow = pbuf->outRow = 0; pbuf->lastRow = -1;
   pbuf->qst = pbuf->qwin = 0; pbuf->qen = -1;
   pbuf->spDetSt=0;pbuf->spDetEn=0;pbuf->spDetFin=-1;

   /* Set up IOConfig */
   pbuf->cf = cf = (IOConfig)New(x,sizeof(IOConfigRec));
   *cf = defConf;
   cf->nUsed = 0;
   for (s=1; s<=o.swidth[0]; s++)
      cf->nUsed += o.swidth[s];
   if ((o.pk&BASEMASK)==DISCRETE)
      cf->nCols = o.swidth[0], pbuf->dShort=TRUE;
   else
      cf->nCols = cf->nUsed, pbuf->dShort=FALSE;
   cf->srcPK       = info.srcPK;
   cf->srcFF       = info.srcFF;
   cf->srcSampRate = info.srcSampRate;
   cf->nSamples    = 0;
   cf->tgtPK       = info.tgtPK;
   cf->tgtFF       = info.tgtFF;
   cf->tgtSampRate = info.tgtSampRate;
   cf->useSilDet   = FALSE;
   cf->audSignal   = 0;
   cf->vqTabFN     = info.vqTabFN;
   cf->srcUsed     = info.srcVecSize;
   cf->tgtUsed     = cf->nUsed;
   cf->saveCompressed = info.saveCompressed;
   cf->saveWithCRC = info.saveWithCRC;
   cf->curPK = cf->tgtPK;

   /* Set up PBlock */
   dBytes = cf->nCols * size * (pbuf->dShort?sizeof(short):sizeof(float));
   pbuf->main.data = New(pbuf->mem,dBytes); 
   pbuf->main.stRow=0; pbuf->main.nRows=0; pbuf->main.maxRows=size;
   pbuf->chType = 0;
   return pbuf;
}

/* EXPORT->WriteESIGPHeader: Write header info to ESIG parameter file f */
void WriteESIGPHeader(FILE *f, IOConfig cf, HTime sampPeriod, short sampSize, short pKind)
{
   FieldSpec *field, *field1;
   FieldList inList, outList=NULL;
   int len, i;
   char buf[30];
   short elSize, span[12];

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
   *((double*)field->data)=1.0E07/sampPeriod;
   AddField( &outList, field );

   field = NewFieldSpec( DOUBLE, 0 );
   field->occurrence=GLOBAL;
   field->name="startTime";
   field->data=malloc(sizeof(double));
   if (cf->srcFF == ESIG) {
      RetrieveESIGFieldList(&inList);
      field1=FindField(inList, "startTime");
      *((double*)field->data)=*((double*)field1->data);
   }
   else *((double*)field->data)=0.0;
   AddField( &outList, field );

   if (cf->srcFF == ESIG) {
      field = NewFieldSpec( NO_TYPE, 0 );
      field->occurrence=VIRTUAL;
      field->name="source_1";
      RetrieveESIGFieldList(&inList);
      len=FieldListLength(inList);
      for (i=0;i<len;i++)
         AddSubfield(field,inList[i]);
      AddField( &outList, field );
   }

   ParmKind2Str(pKind, buf);
   field = NewFieldSpec( CHAR, 1 );
   field->occurrence=GLOBAL;
   field->name="parmKind";
   field->dim[0]=strlen(buf)+1;
   field->data=malloc(field->dim[0]);
   strcpy((char *) field->data, buf);
   AddField( &outList, field );

   /* Then create actual record descriptors */
   elSize=sampSize/cf->nCols;
   field = NewFieldSpec( (elSize==2)?SHORT:FLOAT, 1 );
   field->occurrence=REQUIRED;
   ParmKind2Str(pKind&BASEMASK, buf);
   field->name=(char *) malloc(strlen(buf)+1);
   strcpy(field->name, buf);
   FindSpans(span, pKind, cf->nCols);
   field->dim[0]=span[1]-span[0]+1;
   field->data=malloc(field->dim[0]);
   AddField( &outList, field );

   if (pKind & HASZEROC) {
      field = NewFieldSpec( (elSize==2)?SHORT:FLOAT, 0 );
      field->occurrence=REQUIRED;
      field->name="zeroc";
      AddField(&outList, field);
   }

   if ((pKind & HASENERGY) && !(pKind & HASNULLE) )  {
      field = NewFieldSpec( (elSize==2)?SHORT:FLOAT, 0 );
      field->occurrence=REQUIRED;
      field->name="energy";
      AddField(&outList, field);
   }

   if ( pKind & HASDELTA ) {
      field = NewFieldSpec( (elSize==2)?SHORT:FLOAT, 1 );
      field->occurrence=REQUIRED;
      field->dim[0]=span[1]-span[0]+1;
      field->name="delta";
      AddField(&outList, field);

      /*
       * If available add 0'th cepstral
       */
      if ( (pKind & HASZEROC) ) {
         field = NewFieldSpec( (elSize==2)?SHORT:FLOAT, 0 );
         field->occurrence=REQUIRED;
         field->name="delta_zeroc";
         AddField(&outList, field);
      }
      /*
       * if available: the field for the energy
       */
      if ( pKind & HASENERGY ) {
         field = NewFieldSpec( (elSize==2)?SHORT:FLOAT, 0 );
         field->occurrence=REQUIRED;
         field->name="delta_energy";
         AddField(&outList, field);
      }
   }

   if ( pKind & HASACCS ) {
      field = NewFieldSpec( (elSize==2)?SHORT:FLOAT, 1 );
      field->occurrence=REQUIRED;
      field->dim[0]=span[1]-span[0]+1;
      field->name="accs";
      AddField(&outList, field);

      /*
       * If available add 0'th cepstral
       */
      if ( (pKind & HASZEROC) ) {
         field = NewFieldSpec( (elSize==2)?SHORT:FLOAT, 0 );
         field->occurrence=REQUIRED;
         field->name="accs_zeroc";
         AddField(&outList, field);
      }
      /*
       * if available: the field for the energy
       */
      if ( pKind & HASENERGY ) {
         field = NewFieldSpec( (elSize==2)?SHORT:FLOAT, 0 );
         field->occurrence=REQUIRED;
         field->name="accs_energy";
         AddField(&outList, field);
      }
   }
   
   WriteHeader(outList,
               (natWriteOrder && vaxOrder) ? NATIVE : EDR1,
               f, NULL);
}

/* CalcCompress: calculate the scaling factors for compressing data */
static void CalcCompress(ParmBuf pbuf, PBlock *pbInit,int nCols, Boolean irefc)
{
   IOConfig cf = pbuf->cf;
   PBlock *pb;
   int i,nx;
   float *fp,x;
   Vector min,max;

   if (trace&T_CPX)
      printf("HParm: Compressing pbuf: nRows=%d, nCols=%d\n",
             pbuf->main.nRows,nCols);

   cf->A = CreateVector(pbuf->mem,cf->nCols); 
   cf->B = CreateVector(pbuf->mem,cf->nCols);

   if (irefc)
      for(i=1;i<=cf->srcUsed;i++) cf->A[i]=32767.0,cf->B[i]=0.0;
   else {
      min = CreateVector(&gstack,nCols); ZeroVector(min); 
      max = CreateVector(&gstack,nCols); ZeroVector(max); 
      /* Find max and min of each vector component */
      /* Initial value from first block */
      if (pbInit->nRows>0) {
         fp = (float *)pbInit->data;
         for (nx=1; nx<=nCols; nx++) min[nx]=max[nx]=*fp++;
      }
      for (pb=pbInit;pb!=NULL;pb=pb->next) {
         fp = (float *)pb->data;
         for (i=0;i<pb->nRows;i++)
            for (nx=1; nx<=nCols; nx++) {
               x = *fp++;
               if (x > max[nx]) max[nx] = x;
               if (x < min[nx]) min[nx] = x;
            }
      }
      /* Compute A and B coefficients */
      for (nx=1; nx<=nCols; nx++) {
         if( (max[nx]-min[nx]) == 0.0 ) {
            cf->A[nx] = 1.0; cf->B[nx] = max[nx];
         } else {
            cf->A[nx] = 2.0 * 32767.0 / (max[nx]-min[nx]);
            cf->B[nx] = (max[nx]+min[nx]) * 32767.0 / (max[nx]-min[nx]);
         }
      }
      FreeVector(&gstack,max); FreeVector(&gstack,min);
   }
}

/* CompressPBlock: convert all floats in PBlock to short */
static void CompressPBlock(ParmBuf pbuf, PBlock *pb, short *sp, int nCols)
{
   IOConfig cf = pbuf->cf;
   int i,nx,ix,count;
   float *fp,x;

   if (trace&T_CPX)
      printf("HParm: Compressing pblock: nRows=%d, nCols=%d\n",
             pb->nRows,nCols);
   count = pb->nRows * cf->nCols;
   /* Convert floats to shorts */
   fp = (float *)pb->data;
   for (i=0;i<pb->nRows;i++)
      for (nx=1; nx<=nCols; nx++) {
         x = ((*fp++) * cf->A[nx] - cf->B[nx]);
         ix = (int) ((x<0.0) ? x-0.5 : x+0.5);
         if (ix<-32767 || ix>32767)
            HError(6393,"CompressPBlock: short out of range %d",ix);
         *sp++ = ix;
      }
}

/* EXPORT->SaveBuffer: write out pbuf to fname */
ReturnStatus SaveBuffer(ParmBuf pbuf, char *fname, FileFormat ff)
{
   PBlock *pb,*pbInit,*pbFin;
   FILE *f;
   IOConfig cf = pbuf->cf;
   Boolean bSwap,isPipe;
   short sampSize,kind,*sp;
   long nSamples,sampPeriod;
   char buf[50];
   
   /*
     if one needs to fake a target parm kind for the buffer to be saved, 
      then the channel config setup is switched here. This is a hack but 
      unfortunately various old versions, mainly decoders, are not happy 
      with all the parm kind types supported in this module, unless they 
      are cheated in such a way.
   */
   if (ForcePKind != ANON) {
      cf->tgtPK = ForcePKind;
   }
   
   if (ff != UNDEFF) cf->tgtFF = ff;
   if (pbuf->spDetEn>=0 && pbuf->main.nRows+pbuf->main.stRow>pbuf->spDetEn)
      pbuf->main.nRows=pbuf->spDetEn-pbuf->main.stRow;
   for (pb=&pbuf->main,nSamples=0;pb!=NULL;pb=pb->next) 
      nSamples += pb->nRows;
   if (nSamples<=0){
      HRError(6352,"SaveBuffer: Cannot save empty buffer to file %s",fname);
      return(FAIL);
   }
   sampPeriod = (long) cf->tgtSampRate;
   kind = cf->tgtPK & ~(HASNULLE|HASVQ);
   if (cf->saveWithCRC) kind |= HASCRCC;
   if ((kind&BASEMASK)==DISCRETE)
      sampSize = cf->nCols * sizeof(short);
   else if (cf->saveCompressed) {
      sampSize = cf->nCols * sizeof(short);
      if ((kind&BASEMASK) == LPREFC)
         kind = IREFC | (kind&~BASEMASK);
      else {
         kind |= HASCOMPX;
         /* Need space for A/B vectors */
         nSamples += 4;
      }
   }
   else
      sampSize = cf->nCols * sizeof(float);

   if ( (f = FOpen(fname,ParmOFilter,&isPipe)) == NULL){ /* Binary file */
      HRError(6311,"SaveBuffer: cannot create file %s",fname);
      return(FAIL);
   }
   bSwap=FALSE;
   switch (cf->tgtFF) {
   case HTK:
      WriteHTKHeader(f,nSamples,sampPeriod,sampSize,kind,&bSwap);
      break;
   case ESIG:
      WriteESIGPHeader(f, cf, sampPeriod, sampSize, kind); 
      bSwap = vaxOrder && !natWriteOrder;
      break;
   default:
      HRError(6270,"SaveBuffer: Cannot save data as %s.",
              Format2Str(cf->tgtFF));
      FClose(f,isPipe);
      return(FAIL);
      break;
   }

   pbInit=pbuf->main.next;
   if (pbInit==NULL) {
      pbInit=&pbuf->main;
      pbFin=NULL;
   }
   else {
      for (pbFin=pbInit;pbFin->next!=NULL;pbFin=pbFin->next);
      pbFin->next=&pbuf->main;
      pbuf->main.next=NULL;
   }
   cf->crcc=0;

   if (cf->saveCompressed)
      CalcCompress(pbuf,pbInit,cf->nCols,((kind&BASEMASK) == LPREFC));
        
   if (cf->saveCompressed && (kind&BASEMASK) != LPREFC) {
      WriteFloat(f,cf->A+1,cf->nCols,hparmBin);
      WriteFloat(f,cf->B+1,cf->nCols,hparmBin);
      cf->crcc=UpdateCRCC(cf->A+1,cf->nCols,sizeof(float),bSwap,cf->crcc);
      cf->crcc=UpdateCRCC(cf->B+1,cf->nCols,sizeof(float),bSwap,cf->crcc);
   }

   for (pb=pbInit;pb!=NULL;pb=pb->next) {
      if ((kind&BASEMASK)==DISCRETE) {
         WriteShort(f, (short *) pb->data, pb->nRows*cf->nCols, hparmBin);
         cf->crcc=UpdateCRCC(pb->data,pb->nRows*cf->nCols,
                             sizeof(short),bSwap,cf->crcc);
      }
      else if (cf->saveCompressed) {
         sp=(short *) New(&gstack,sizeof(short)*pb->nRows*cf->nCols);
         CompressPBlock(pbuf,pb,sp,cf->nCols);
         WriteShort(f,sp,pb->nRows*cf->nCols,hparmBin);
         cf->crcc=UpdateCRCC(sp,pb->nRows*cf->nCols,
                             sizeof(short),bSwap,cf->crcc);
         Dispose(&gstack,sp);
      }
      else {
         WriteFloat(f, (float *) pb->data, pb->nRows*cf->nCols, hparmBin);
         cf->crcc=UpdateCRCC(pb->data,pb->nRows*cf->nCols,
                             sizeof(float),bSwap,cf->crcc);
      }
   }
   if (cf->saveWithCRC)
      WriteShort(f,(short*)&cf->crcc,1,hparmBin);

   if (trace&T_TOP){
      printf("HParm: Parm tab type %s saved to %s [sampSize=%d,nSamples=%ld]", 
             ParmKind2Str(kind,buf),fname,sampSize,nSamples);
      if (cf->saveCompressed) printf(" compressed");
      if (cf->saveWithCRC) printf(" with CRC"); printf("\n");
   }
   FClose(f,isPipe);

   if (pbFin) {
      pbFin->next=NULL;
      pbuf->main.next=pbInit;
   }
   return(SUCCESS);
}

/* EXPORT->AddToBuffer: append observation to pbuf */
void AddToBuffer(ParmBuf pbuf, Observation o)
{
   IOConfig cf = pbuf->cf;
   PBlock *pb, *tp;
   long dBytes;
   float *fp;
   short *sp;
   int s,j;
   Vector v;
   
   /* Make sure there is room */
   if (pbuf->main.nRows>=pbuf->main.maxRows) {
      /* Set up PBlock */
      dBytes = cf->nCols * pbuf->main.maxRows * 
         (pbuf->dShort?sizeof(short):sizeof(float));
      pb = (PBlock *) New(pbuf->mem,sizeof(PBlock));
      pb->data = New(pbuf->mem,dBytes); 
      pb->stRow=pbuf->main.stRow; pb->nRows=pb->maxRows=pbuf->main.maxRows;

      memcpy(pb->data,pbuf->main.data,dBytes);
      pbuf->main.stRow+=pbuf->main.nRows; 
        
      pb->next=NULL;  /* Add into linked list */

      if (pbuf->main.next==NULL) pbuf->main.next=pb;
      else
         for (tp=pbuf->main.next;tp!=NULL;tp=tp->next)
            if (tp->next==NULL) {
               tp->next=pb;
               break;  
            }

      if (trace&T_BUF)
         printf("HParm:  output table extended by %d elems\n",
                pbuf->main.nRows);
      pbuf->main.nRows=0;
   }
   /* copy observation to next free row in pbuf */
   if ((o.pk&BASEMASK) == DISCRETE){
      sp = (short *)pbuf->main.data + pbuf->main.nRows * cf->nCols;
      for (s=1; s<=o.swidth[0]; s++) 
         *sp++ = o.vq[s];
   } else {
      fp = (float *)pbuf->main.data + pbuf->main.nRows * cf->nCols;
      for (s=1; s<=o.swidth[0]; s++) {
         v =  o.fv[s];
         for (j=1; j<=o.swidth[s]; j++)
            *fp++ = v[j];
      }
   }
   pbuf->main.nRows++; pbuf->lastRow++;
   pbuf->spDetEn++; pbuf->spDetFin++;
   pbuf->qst++; pbuf->qen++; 
}

/* --------------------------  HParm.c ------------------------- */
