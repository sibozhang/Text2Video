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
/*    File: LNorm: normalise model                             */
/* ----------------------------------------------------------- */

char *lnorm_version = "!HVER!LNorm:   3.4.1 [CUED 12/03/09]";
char *lnorm_vc_id = "$Id: LNorm.c,v 1.1.1.1 2006/10/11 09:54:44 jal58 Exp $";

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
#include "LGBase.h"
#include "LUtil.h"
#include "LModel.h"
#include "LPCalc.h"
#include "LPMerge.h"

#define T_TOP       0001

/* -------------------- Global variables ----------------------- */

static int       trace = 0;              /* trace level */
static int       nSize = 0;              /* output n-gram size */
static char      *srcFN = NULL;          /* source file name */
static char      *tgtFN = NULL;          /* target file name */
static char      *wlistFN = NULL;        /* word list file name */
static WordMap   wlist;                  /* word list */
static BackOffLM *lm;                    /* the language model */
static MemHeap   langHeap;               /* Stores global stats */
static LMFileFmt saveFmt = LMF_BINARY;   /* LM file format */
static int       cutOff[LM_NSIZE+1];     /* new cutoff's */
static float wdThresh[LM_NSIZE+1];  /* new wdThresh for COUNT-models */

/* ---------------- Configuration Parameters --------------------- */

static ConfParam *cParm[MAXGLOBS];
static int nParm = 0;                  /* total num params */

/* ---------------- Process Command Line ------------------------- */

/* SetConfParms: set conf parms relevant to this tool */
void SetConfParms(void)
{
   int i;

   nParm = GetConfig("LNORM", TRUE, cParm, MAXGLOBS);
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
   printf("\nUSAGE: LNorm [options] inModel outModel\n\n");
   printf(" Option                                       Default\n\n");
   printf(" -c n c  set cutoff for n-gram to c           0\n");
   printf(" -d n c  set weighted discount pruning to c   off\n");
   printf(" -f s    set output LM format to s            %s\n", ReturnLMName(DEF_SAVEFMT));
   printf(" -n n    save model as n-gram                 max\n");
   printf(" -w fn   prune model using word list in fn    off\n");
   PrintStdOpts("GIST");
   printf("\n\n");
}

int main(int argc, char *argv[])
{
   int i;
   char *s,*c;
   char fmt[256];

   InitShell(argc,argv,lnorm_version,lnorm_vc_id);
   InitMem();
   InitMath();
   InitWave();
   InitLabel();
   InitWMap();
   InitLUtil();
   InitLModel();
   InitPCalc();

   if (!InfoPrinted() && NumArgs() == 0)
      ReportUsage();
   if (NumArgs() == 0) Exit(EXIT_SUCCESS);

   SetConfParms();

   CreateHeap(&langHeap,"langHeap",MSTAK,1,0.5,5000,40000);

   for (i=1; i<=LM_NSIZE; i++) cutOff[i] = 0, wdThresh[i] = 0.0;
   while (NextArg() == SWITCHARG) {
      s = GetSwtArg();
      if (strlen(s)!=1) 
         HError(16519,"Bad switch %s; must be single letter",s);
      switch(s[0]){
         case 'c':
            i = GetChkedInt(2,LM_NSIZE,s); 
	    cutOff[i] = GetChkedInt(1,1000,s);
	    break;
         case 'd':
            i = GetChkedInt(2,LM_NSIZE,s); 
	    wdThresh[i] = GetChkedFlt(0.0,1E10,s);
	    break;
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
              HError(16519,"Unrecognised LM format, should be one of [%s, %s, %s]",
                     LM_TXT_TEXT, LM_TXT_BINARY, LM_TXT_ULTRA);
	   break;
         case 'n':
            nSize = GetChkedInt(1,LM_NSIZE,s); break;
         case 'w':
	    if (NextArg() != STRINGARG)
	       HError(16519,"LPlex: Word list file name expected");
	    wlistFN = GetStrArg();
	    break;
	 case 'T':
	    trace = GetChkedInt(0,077, s); break;
         default:
            HError(16519,"LMPlex: Unknown switch %s",s);
      }
   }
   if (NextArg()!=STRINGARG)  /* load the language model */
      HError(16519, "Input language model filename expected");
   srcFN = GetStrArg();

   if (NextArg()!=STRINGARG)  /* load the language model */
      HError(16519, "Output language model filename expected");
   tgtFN= GetStrArg();

   if (wlistFN!=NULL) {
      CreateWordList(wlistFN,&wlist,10);
      lm = LoadLangModel(srcFN,&wlist,1.0,LMP_FLOAT|LMP_COUNT,&langHeap);
   } else {
      lm = LoadLangModel(srcFN,NULL,1.0,LMP_FLOAT|LMP_COUNT,&langHeap);
   }
   if (lm->probType==LMP_COUNT) {
      RebuildLM(lm,cutOff,wdThresh,LMP_FLOAT);
   } else {
      NormaliseLM(lm);
   }
   if (nSize>0 && nSize<lm->nSize)
      lm->nSize = nSize;
   for (i=1;i<=lm->nSize;i++)
      lm->gInfo[i].fmt = (i==1) ? LMF_TEXT : saveFmt;
   SaveLangModel(tgtFN,lm);

   Exit(EXIT_SUCCESS);
   return EXIT_SUCCESS; /* never reached -- make compiler happy */
}   

/* --------------------- end of LNorm.c ------------------------- */
