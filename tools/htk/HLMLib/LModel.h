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
/* main authors: Valtcho Valtchev, Steve Young,                */
/*               Julian Odell, Gareth Moore                    */
/* ----------------------------------------------------------- */
/*         Copyright:                                          */
/*                                                             */
/*          1994-2002 Cambridge University                     */
/*                    Engineering Department                   */
/*                                                             */
/*   Use of this software is governed by a License Agreement   */
/*    ** See the file License for the Conditions of Use  **    */
/*    **     This banner notice must not be removed      **    */
/*                                                             */
/* ----------------------------------------------------------- */
/*      File: LModel:    ARPA style LM handling                */
/* ----------------------------------------------------------- */

/* !HVER!LModel:   3.4.1 [CUED 12/03/09] */

/*
   Language model handling
*/

#include "HLM.h"

#ifndef _LMODEL_H
#define _LMODEL_H

#ifdef __cplusplus
extern "C" {
#endif

#define LM_NSIZE      16

#define L10ZERO       -99.9900
#define L10MINARG     +1.0E-20
#define EXP10MINARG   -20.0
#define LN10          +2.30258509299404568   /* Defined to save recalculating it */

#define LOG10_TO_FLT(x) \
  (((x) < EXP10MINARG) ? 0.0 : exp(LN10*(x)))

#define FLT_TO_LOG10(x) \
  (((x) < L10MINARG) ? L10ZERO : log10(x))

/* Calculate word|class log probability value */
#define LOG_NATURAL(x) (((x) < 1.0E-20) ? -99.9900 : log(x))

#define EMINARG -46.05
#define UNLOG_NATURAL(x) (((x) < EMINARG) ? 0.0 : exp(x))

#define HAS_BOWT    01
#define INT_LMID    02
#define MIN_BOWT    +1.0E-06

#define LM_INDEX(x) (x->aux)

#define DEF_STARTWORD   "<s>"
#define DEF_ENDWORD     "</s>"

typedef struct _AccessInfo  AccessInfo; /* abstract type for access stats structure */

typedef enum {       /* external file format definitions */
  LMF_TEXT, LMF_BINARY, LMF_ULTRA, LMF_OTHER
} LMFileFmt;
/* What text is used by the relevant tools to select these models? */
#define LM_TXT_TEXT "TEXT"
#define LM_TXT_BINARY "BIN"
#define LM_TXT_ULTRA "ULTRA"
#define LM_TXT_OTHER "OTHER"

typedef enum {       /* probability type */
  LMP_LOG   = 001,
  LMP_FLOAT = 002,
  LMP_COUNT = 004,
  LMP_OTHER = 010
} LMProbType;

#ifndef LM_TYPES_DEFINED
typedef unsigned short UShort;
typedef unsigned int   UInt;
typedef unsigned char  Byte;
#define LM_TYPES_DEFINED
#endif

#ifdef LM_ID_SHORT
typedef UShort LM_Id;       /* 2-byte ID for normal models */
#else
typedef UInt   LM_Id;       /* 4-byte ID for large models */
#endif

#ifdef LM_COMPACT
typedef UShort LM_Prob;     /* 2-byte compressed probability */
#else
typedef float  LM_Prob;     /* 4-byte probability/count */
#endif

typedef struct _SMEntry {
   LM_Id   ndx;             /* word index */
   LM_Prob prob;            /* probability */
} SMEntry;

typedef struct _FLEntry {   /* Full LM entry, single level of context */
   float bowt;              /* LOG10 backoff weight */
   LM_Id ndx;               /* word index */
   LM_Id nse;               /* number of SMEntry */
   LM_Id nfe;               /* number of FLEntry */
   struct _SMEntry *sea;    /* sorted array [0..nse-1] of SMEntry */
   struct _FLEntry *fea;    /* sorted array [0..nfe-1] of FLEntry */
   struct _FLEntry *parent; /* parent FLEntry - used when reconstructing context */
} FLEntry;

typedef enum {
   DC_KATZ, DC_ABSOLUTE, DC_LINEAR, DC_LAST
} DiscountType;

typedef struct {            /* Good-Turing discounting parameters */
   int kRange;              /* discounting range */
   float *coef;             /* discounting coefs */
} TuringGoodInfo;

typedef union {
   double bCoef;            /* absolute discounting b coef */
   TuringGoodInfo tgInfo;   /* Turing-Good discounting */
} DiscountInfo;

typedef struct {            /* Backing-off parameters */
   DiscountType dcType;     /* discounting type */
   int cutOff;              /* cut-off */
   float wdThresh;          /* Threshold for wd pruning */
   DiscountInfo dcInfo;     /* discounting coefficients, etc */
} BackOffInfo;

typedef struct {
   int nEntry;              /* number of entries in model */
   LMFileFmt fmt;           /* indicates LM section stored in binary */
   AccessInfo *aInfo;       /* access statistics information */
   BackOffInfo *boInfo;     /* discounting type */
} NGramInfo;

/* Structure for storing word|class probabilities */
typedef struct
{
  float  prob;               /* log probability of word: word|class */
  NameId class;              /* pointer to class this word is in */
  int    id;                 /* private ID used for dynamic count lookups */
} WordProb;

typedef struct {
   int nSize;                /* model order 1=unigram, 2=bigram, etc */
   int vocSize;              /* vocabulary size */
   char  *name;              /* textual description and header */
   LMProbType probType;      /* probability type */
   MemHeap *heap;            /* heap for storage */
   HashTab *htab;            /* hash table for names */
   NameId  *binMap;          /* NameId array for decoding binary LMs */
   FLEntry root;             /* the actual LM tree */
   float   gScale;           /* grammar scale factor */
   NGramInfo gInfo[LM_NSIZE];/* information for each n-gram component */
   Boolean encrypt;          /* flag for encryption */
   /* class components */
   Boolean classLM;          /* TRUE if a class-based LM (toggle on backoff) */
   HashTab *classH;          /* If a class-based LM then use this hash table */
   NameId *classBM;          /* Hash table (similar to binMap) */
   int classW;               /* Number of words */
   Boolean classCounts;      /* TRUE if LM file gives class counts not probs */
   int *word;                /* array of word counts */
   int *totals;              /* array of class counts (sum of word counts for each class) */
   /* other components */
   float *lmvec;             /* vector for storing vector n-grams */
   FLEntry *fe_buff;         /* temp buffer */
   SMEntry *se_buff;         /* temp buffer */ /* NB variable size! */
} BackOffLM;

void InitLModel(void);
/*
   Initialise module
*/

BackOffLM *LoadLangModel(char *fn, WordMap *wl, float gramScale,
			 LMProbType tgtPType, MemHeap *heap);
/*
   Read N-gram language model from fn, optionally perform word mappings
   specified in file mapfn, scale probabilities by gramScale
*/

void SaveLangModel(char *lmFn, BackOffLM *lm);
/*
   Write language model lmodel to file lmFn
*/

void StoreFEA(FLEntry *fe, MemHeap *heap);
/* 
   Move FEA array into permanent location
*/

void AttachAccessInfo(BackOffLM *lm);
/*
   Create and initialise access stats info
*/

void ResetAccessInfo(BackOffLM *lm);
/*
   Reset access statistics
*/

void PrintTotalAccessStats(FILE *f, BackOffLM *lmodel);
/*
   Print n-gram access statistics
*/

/*------------------- Search/compare routines -------------------*/

int CmpSE(const void *p1, const void *p2);
/*
   qsort comparison for small entries
*/

int CmpFE(const void *p1, const void *p2);
/*
   qsort comparison for full entries
*/

SMEntry *FindSE(SMEntry *sptr, int lo, int hi, LM_Id key);
/*
   Find SMEntry in a sorted list
*/

FLEntry *FindFE(FLEntry *fptr, int lo, int hi, LM_Id key);
/*
   Find FLEntry in a sorted list
*/

/*------------------- N-gram access routines -------------------*/

float GetNGramProb(BackOffLM *lm, NameId *words, int G);
/*
   Obtain the probability P(words[G-1]|words[0],...,words[G-2])
*/

void *GetNGramAddress(BackOffLM *lm, NameId *words, int G);
/*
   Obtain a unique pointer id for the context (words[0],...,words[G-2])
*/

void GetNGramProbs(BackOffLM *lm, UInt *nId, int nSize, SMEntry *seBuf, int seBufSize);
/*
   GetNGramProbs: probabilities for selected SEs
*/

float *GetNGramProbVecSE(BackOffLM *lm, int *nId, int nSize, FLEntry *sfe);
/*
   GetNGramProbVec: vector LM access
*/


LogFloat LMTrans2(LModel *LM, LMState src, LabId wdid, LMState *dest);


/* -------------------- End of LModel.h ---------------------- */

#ifdef __cplusplus
}
#endif

#endif
