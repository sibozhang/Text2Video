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
/*         File: HLM.h language model handling                 */
/* ----------------------------------------------------------- */

/* !HVER!HLM:   3.4.1 [CUED 12/03/09] */

#ifndef _HLM_H_
#define _HLM_H_

#ifdef __cplusplus
extern "C" {
#endif

typedef enum { boNGram=1, matBigram, hlmModel } LMType;

#define MAX_LMID 65534          /* Max number of words */
typedef unsigned short lmId;    /* Type used by lm to id words  1..MAX_LMID */
typedef unsigned short lmCnt;   /* Type used by lm to count wds 0..MAX_LMID */

#define NSIZE 4                 /* Max length of ngram 2==bigram etc */



typedef struct sentry {         /* HLM NGram probability */
   lmId word;                   /* word id */
   float prob;                  /* probability */
} SEntry;

typedef struct nentry {         /* HLM NGram history */
   lmId word[NSIZE-1];          /* Word history representing this entry */
   lmCnt nse;                   /* Number of ngrams for this entry */
   float bowt;                  /* Back-off weight */
   SEntry *se;                  /* Array[0..nse-1] of ngram probabilities */
   struct nentry *link;         /* Next entry in hash table */
   void *user;                  /* Accumulator or cache storage */
} NEntry;

typedef struct ngramlm {
   int nsize;                   /* Unigram==1, Bigram==2, Trigram==3 */
   unsigned int hashsize;       /* Size of hashtab (adjusted by lm counts) */
   NEntry **hashtab;            /* Hash table for finding NEntries */
   int counts[NSIZE+1];         /* Number of [n]grams */
   int vocSize;                 /* Core LM size */
   Vector unigrams;             /* Unigram probabilities */
   LabId *wdlist;               /* Lookup table for words from lmId */
   MemHeap *heap;               /* Pointer to heap */
} NGramLM;

typedef struct matbilm {
   lmCnt numWords;              /* Number of words for language model */
   Matrix bigMat;               /* Actual probs */
   LabId *wdlist;               /* Lookup table for words from lmId */
   MemHeap *heap;               /* Pointer to heap */
} MatBiLM;

typedef struct lmodel {
   char *name;                  /* Name used for identifying lm */
   LMType type;                 /* LM type */
   LogFloat pen;                /* Word insertion penalty */
   float scale;                 /* Language model scale */
   union {
      MatBiLM *matbi;
      NGramLM *ngram;
      void *hlmModel;
   }
   data;
   MemHeap *heap;               /* Heap for allocating lm structs */
} LModel;


void InitLM(void);
/*
   Initialise the module
*/

/* ---------------- Lower Level Routines ----------------- */

NGramLM *CreateBoNGram(LModel *lm,int vocSize,int counts[NSIZE+1]);
/*
   Create backoff NGram language models with size defined by counts.
    vocSize=number of words in vocabulary
    counts[1]=number of unigrams
    counts[2]=approximate number of bigrams
    counts[3]=approximate number of trigrams
               (approximate sizes are used to determine hash table size)
*/
MatBiLM *CreateMatBigram(LModel *lm,int nw);
/*
   Create matrix bigram language models of specified size.
*/

NEntry *GetNEntry(NGramLM *nglm,lmId ndx[NSIZE],Boolean create);
/*
   Find [create] ngram entry for word histories ...ndx[1] ndx[0].
*/

/* --------------- Higher Level Routines ----------------- */

float GetLMProb(LModel *lm, LabId prid[NSIZE], LabId wdid);
/*
   Calculate probability for wdid following prid
*/

LModel *ReadLModel(MemHeap *heap,char *fn);
void WriteLModel(LModel *lm,char *fn,int flags);
/*
   Read/write language model from/to specified file.
   Flags control format for writing.
*/

void ClearLModel(LModel *lm);
/* 
   Clear LModel before deletion
*/

#ifndef NO_LAT_LM
typedef Ptr LMState;

LogFloat LMTrans (LModel *lm, LMState src, LabId wdid, LMState *dest);
#endif

#ifdef __cplusplus
}
#endif

#endif  /* _HLM_H_ */

/* ---------------------- End of HLM.h ----------------------- */
