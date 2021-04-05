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
/*      File: LPCalc: probability calculation                  */
/* ----------------------------------------------------------- */

/* !HVER!LPCalc:   3.4.1 [CUED 12/03/09] */

#ifndef _LPCALC_H
#define _LPCALC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "LModel.h"

#define DEF_KRANGE    7
#define DEF_CUTOFF    1
#define DEF_UNIFLOOR  1
#define DEF_SAVEFMT   LMF_BINARY
#define DEF_LMPTYPE   LMP_FLOAT
#define DEF_DCTYPE    DC_KATZ

typedef struct {
   int          nSize;                /* model order */
   FoFTab       *ftab;                /* FoF table */
   WordMap      *wmap;                /* word map */
   LMFileFmt    saveFmt;              /* output LM file format */
   LMProbType   ptype;                /* probability type */
   DiscountType dctype;               /* discount type */
   float        uniFloor;             /* unigram floor */
   int          kRange;               /* discounting range for Turing-Good scheme */
   int          cutOff[LM_NSIZE+1];   /* n-gram cut-off array */
   int          wdThresh[LM_NSIZE+1]; /* n-gram wd threshold array */
   NGInputSet   *inSet;               /* input n-gram file set */
} BuildInfo;

void InitPCalc(void);
/* 
    Initialise module 
*/

void InitBuildInfo(BuildInfo *bi);
/* 
    Setup structure to default values
*/

Boolean FilterNGram(NGInputSet *inSet, UInt *gram, float *count, int nSize);
/* 
    Read n-grams with words in word map 
*/

BackOffLM *UpdateModel(BackOffLM *lm, BuildInfo *bi);
/* 
     Increase the order of an existing model from n-gram data
*/

BackOffLM *GenerateModel(MemHeap *heap, BuildInfo *bi);
/* 
     Generate model fron n-gram data files 
*/

void RebuildLM(BackOffLM *lm, int *cutOff, float *wdThresh, LMProbType tgtPType);
/* 
   Convert model to ptype probs
*/

void ComputeFoFTab(FoFTab *ftab, int nSize, NGInputSet *inSet);
/* 
   ComputeFoFTab: scan files and produce FoF table 
*/

/* -------------------- End of LPCalc.h ---------------------- */

#ifdef __cplusplus
}
#endif

#endif
