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
/*      File: LPMerge:    LM interpolation                     */
/* ----------------------------------------------------------- */

/* !HVER!LPMerge:   3.4.1 [CUED 12/03/09] */

/* ------------------- Model interpolation  ----------------- */

#ifndef _LPMERGE_H
#define _LPMERGE_H

#ifdef __cplusplus
extern "C" {
#endif

#define MAX_LMODEL    32

typedef struct {
   char *fn;                /* LM filename */
   BackOffLM *lm;           /* the language model */
   float weight;            /* interpolation weight */
} LMInfo;

void InitPMerge(void);
/* 
   Initialise module 
*/

BackOffLM *MergeModels(MemHeap *heap, LMInfo *lmInfo, int nLModel, 
		       int nSize, WordMap *wList);
/*
   Interpolate models in lmInfo and return resulting model
*/

void NormaliseLM(BackOffLM *lm);
/* 
   Normalise probabilities and calculate back-off weights 
*/

/* -------------------- End of LPMerge.h ---------------------- */

#ifdef __cplusplus
}
#endif

#endif
