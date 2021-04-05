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
/*    File: LMerge:  combine language models                   */
/* ----------------------------------------------------------- */

char *lmerge_version = "!HVER!LMerge:   3.4.1 [CUED 12/03/09]";
char *lmerge_vc_id = "$Id: LMerge.c,v 1.1.1.1 2006/10/11 09:54:44 jal58 Exp $";

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <math.h>
#include <assert.h>

#include "HShell.h"
#include "HMem.h"
#include "HMath.h"
#include "HWave.h"
#include "HLabel.h"
#include "LWMap.h"
#include "LCMap.h"
#include "LGBase.h"
#include "LUtil.h"
#include "LModel.h"
#include "LPCalc.h"
#include "LPMerge.h"


#define T_TOP       0001

/* -------------------- Global variables ----------------------- */

static int trace = 0;                    /* trace level */

static int       nSize = 0;              /* output ngram size */
static WordMap   wList;                  /* the word list */
static int       nLModel;                /* number of loaded LMs */
static LMInfo    lmInfo[MAX_LMODEL];     /* array of loaded LMs */
static BackOffLM *tgtLM;                 /* target lm */
static char      *tgtFN;                 /* output model name */
static MemHeap   langHeap;               /* Stores global stats */
static LMFileFmt saveFmt = DEF_SAVEFMT;  /* LM file format */

/* ---------------- Configuration Parameters --------------------- */

static ConfParam *cParm[MAXGLOBS];
static int nParm = 0;                  /* total num params */


/* ---------------- Function prototypes -------------------------- */

void Initialise(void);


/* ---------------- Process Command Line ------------------------- */

/* SetConfParms: set conf parms relevant to this tool */
void SetConfParms(void)
{
   int i;

   nParm = GetConfig("LMERGE", TRUE, cParm, MAXGLOBS);
   if (nParm>0){
      if (GetConfInt(cParm,nParm,"TRACE",&i)) trace = i;
   }
}

char *ReturnLMName(int fmt)
{
   switch(fmt) {
      case LMF_TEXT:
	 return LM_TXT_TEXT;
      case LMF_BINARY:
	 return LM_TXT_BINARY;
      case LMF_ULTRA:
	 return LM_TXT_ULTRA;
      default:
	 return LM_TXT_OTHER;
   }   
}

void ReportUsage(void)
{
   printf("\nUSAGE: LMerge [options] wordList inModel outModel\n\n");
   printf(" Option                                       Default\n\n");
   printf(" -f s    set output LM format to s            %s\n", ReturnLMName(DEF_SAVEFMT));
   printf(" -i f s  interpolate with model s, weight f   off\n");
   printf(" -n n    produce n-gram model                 max\n");
   PrintStdOpts("GIST");
   printf("\n\n");
}

int main(int argc, char *argv[])
{
   int i;
   char *s,*c;
   char fmt[256];

   InitShell(argc,argv,lmerge_version,lmerge_vc_id);
   InitMem();
   InitMath();
   InitWave();
   InitLabel();
   InitWMap();
   InitLUtil();
   InitLModel();
   InitPCalc();
   InitPMerge();

   if (!InfoPrinted() && NumArgs() == 0)
      ReportUsage();
   if (NumArgs() == 0) Exit(0);

   SetConfParms();

   nLModel = 1;
   CreateHeap(&langHeap,"langHeap",MSTAK,1,0.5,5000,40000);

   while (NextArg() == SWITCHARG) {
      s = GetSwtArg();
      if (strlen(s)!=1) 
         HError(16319,"Bad switch '%s'; must be single letter",s);
      switch(s[0]){
         case 'f':
	   strcpy(fmt,GetStrArg());
	   for (c=fmt; *c!=0; *c=toupper(*c),c++);
	    if (strcmp(fmt, LM_TXT_TEXT)==0)
               saveFmt = LMF_TEXT;
	    else if (strcmp(fmt, LM_TXT_BINARY)==0)
	       saveFmt = LMF_BINARY;
	    else if (strcmp(fmt, LM_TXT_ULTRA)==0)
	       saveFmt = LMF_ULTRA;
	    else
	       HError(16319,"Unrecognised LM format, should be one of [%s, %s, %s]",
		      LM_TXT_TEXT, LM_TXT_BINARY, LM_TXT_ULTRA);
	   break;
	 case 'i':
            if (NextArg()!=FLOATARG)
	       HError(16319,"Interpolation weight expected for -i");
	    lmInfo[nLModel].weight = GetChkedFlt(0.0,1.0,s);
            if (NextArg()!=STRINGARG)
	       HError(16319,"Interpolation LM filename expected for -i");
	    lmInfo[nLModel].fn = GetStrArg();
	    nLModel++;
	    break;
	 case 'n':
	    nSize = GetChkedInt(1,LM_NSIZE,s); break;
	 case 'T':
	    trace = GetChkedInt(0,077, s); break;
         default:
            HError(16319,"Unknown switch '%s'",s);
      }
   }
   if (NextArg()!=STRINGARG)  /* load the word list */
      HError(16319, "Word list filename expected");
   CreateWordList(GetStrArg(),&wList,10);

   if (NextArg()!=STRINGARG)  /* load the language model */
      HError(16319, "Input language model filename expected");
   lmInfo[0].fn = GetStrArg();

   if (NextArg()!=STRINGARG)  /* load the language model */
      HError(16319, "Output language model filename expected");
   tgtFN= GetStrArg();

   Initialise();
   if (nLModel>1) {
      tgtLM = MergeModels(&langHeap,lmInfo,nLModel,nSize,&wList);
   } else {
      tgtLM = lmInfo[0].lm;
   }

   /*printf("size of 1g is %d\n", tgtLM->gInfo[1].nEntry);*/

   for (i=1; i<=nSize; i++) {
      tgtLM->gInfo[i].fmt = (i==1) ? LMF_TEXT : saveFmt;
   }
   SaveLangModel(tgtFN,tgtLM);

   printf("\nFinished\n");
   Exit(EXIT_SUCCESS);
   return EXIT_SUCCESS; /* never reached -- make compiler happy */
}   

/* Initialise: perform global initialisations */
void Initialise(void) 
{
   int i;
   float x;
   LMInfo *li;

   /* normalise weights */
   for (x=0.0, i=1; i<nLModel; i++)
      x += lmInfo[i].weight;
   lmInfo[0].weight = 1.0-x;

   /* load all models */
   for (li=lmInfo, i=0; i<nLModel; i++, li++) {
      if (trace&T_TOP)
	 printf("Loading language model from %s\n",li->fn);
      li->lm = LoadLangModel(li->fn,NULL,1.0,LMP_FLOAT,&langHeap);
   }
   if (trace&T_TOP) {
      printf("Using language model(s): \n");
      for (li=lmInfo,i=0; i<nLModel; i++,li++)
	 printf("  %d-gram %s, weight %.2f\n",li->lm->nSize,li->fn,li->weight);
   }
   if (nSize==0) { /* not set explicitly */
      for (li=lmInfo, i=0; i<nLModel; i++, li++) {
	 if (li->lm->nSize > nSize)
	    nSize = li->lm->nSize;
      }
   }
   if (trace&T_TOP) {
      printf("Generating %d-gram model %s\n",nSize,tgtFN);
      fflush(stdout);
   }
}
