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
/*         File: LBuild.c - generate LM                         */
/* ----------------------------------------------------------- */

char *lbuild_version = "!HVER!LBuild:   3.4.1 [CUED 12/03/09]";
char *lbuild_vc_id = "$Id: LBuild.c,v 1.1.1.1 2006/10/11 09:54:44 jal58 Exp $";

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

/* -------------------------- Trace Flags ------------------------ */

static int trace = 0;
#define T_TOP  0001     /* Top Level tracing */

/* ---------------- Configuration Parameters --------------------- */

static ConfParam *cParm[MAXGLOBS];
static int nParm = 0;            /* total num params */

/* ---------------------- Global Variables ----------------------- */

static MemHeap langHeap;            /* memory for NGBuffers */

static char *srcFN   = NULL;        /* input LM filename */
static char *tgtFN   = NULL;        /* output LM filename */
static char *wmapFN  = "wmap";      /* output word map file name */
static char *fofFN   = NULL;        /* FoF table filename */
static int  freeSlots = 10;

static BackOffLM *tgtLM = NULL;     /* the generated LM */
static BackOffLM *srcLM = NULL;     /* the source model if any */

static WordMap    wmap;
static NGInputSet inSet;
static BuildInfo  binfo;


/* ---------------- Function prototypes -------------------------- */

void Initialise(BuildInfo *bi);


/* ---------------- Process Command Line ------------------------- */

/* SetConfParms: set conf parms relevant to this tool */
void SetConfParms(void)
{
   int i;

   nParm = GetConfig("LBUILD", TRUE, cParm, MAXGLOBS);
   if (nParm>0){
      if (GetConfInt(cParm,nParm, "TRACE",&i))      trace = i;
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
   printf("\nUSAGE: LBuild [options] wordMap langModel gramfile ....\n\n");
   printf(" Option                                       Default\n\n");
   printf(" -c n c  set cutoff for n-gram to c           1\n");
   printf(" -d n c  set weighted discount pruning to c   off\n");
   printf(" -f s    set output LM format to s            %s\n", ReturnLMName(DEF_SAVEFMT));
   printf(" -k n    set discounting range to [1..n]      7\n");
   printf(" -l fn   build from existing LM from fn       off\n");
   printf(" -n n    set model order                      max\n");
   printf(" -t fn   load FoF table from fn               off\n");
   printf(" -u n    set unigram floor count to n         1\n");
   printf(" -x      save model with counts               off\n");
   PrintStdOpts("");
   printf("\n\n");
}

int main(int argc, char *argv[])
{
   int i;
   char *c,*s;
   char fmt[256];

   InitShell(argc,argv,lbuild_version,lbuild_vc_id);
   InitMem();
   InitMath();
   InitWave();
   InitLabel();
   InitLUtil();
   InitWMap();
   InitGBase();
   InitLModel();
   InitPCalc();

   if (!InfoPrinted() && NumArgs() == 0)
      ReportUsage();
   if (NumArgs() == 0) Exit(EXIT_SUCCESS);

   SetConfParms();

   InitBuildInfo(&binfo);
   while (NextArg() == SWITCHARG) {
      s = GetSwtArg();
      if (strlen(s)!=1) 
         HError(16819,"Bad switch %s; must be single letter",s);
      switch(s[0]){
         case 'c':
            i = GetChkedInt(2,LM_NSIZE,s); 
	    binfo.cutOff[i] = GetChkedInt(0,1000,s);
         break;
         case 'd':
            i = GetChkedInt(2,LM_NSIZE,s); 
  	    binfo.wdThresh[i] = GetChkedFlt(0.0,1E10,s);
  	    break;
         case 'f':
	    strcpy(fmt,GetStrArg());
	    for (c=fmt; *c!=0; *c=toupper(*c),c++);
            if (strcmp(fmt, LM_TXT_TEXT)==0)
	      binfo.saveFmt = LMF_TEXT;
	    else if (strcmp(fmt, LM_TXT_BINARY)==0)
	       binfo.saveFmt = LMF_BINARY;
	    else if (strcmp(fmt, LM_TXT_ULTRA)==0)
	       binfo.saveFmt = LMF_ULTRA;
	    else
	       HError(16819,"Unrecognised LM format, should be one of [%s, %s, %s]",
		      LM_TXT_TEXT, LM_TXT_BINARY, LM_TXT_ULTRA);
	    break;
         case 't' :
	    if (NextArg() != STRINGARG)
	       HError(16819,"LBuild: FoF table filename expected");
	    fofFN = GetStrArg(); break;
         case 'k':
            binfo.kRange = GetChkedInt(1,1000,s); break;
         case 'l' :
	    srcFN = GetStrArg(); break;
         case 'n':
            binfo.nSize = GetChkedInt(1, MAXNG, s); break;
         case 'u':
            binfo.uniFloor = (float) GetChkedInt(0,100000,s); break;
         case 'x':
            binfo.ptype = LMP_COUNT; break;
         case 'T':
            trace = GetChkedInt(0,077,s); break;
         default:
            HError(16819,"LBuild: Unknown switch %s",s);
      }
   }
   if (NextArg() != STRINGARG)
      HError(16819,"LBuild: word map filename expected");
   wmapFN = GetStrArg();
   if (NextArg() != STRINGARG)
      HError(16819,"LBuild: output LM filename expected");
   tgtFN = GetStrArg();
   Initialise(&binfo);
   if (srcLM!=NULL)
      tgtLM = UpdateModel(srcLM,&binfo);
   else 
      tgtLM = GenerateModel(&langHeap,&binfo);
   SaveLangModel(tgtFN,tgtLM);

   Exit(EXIT_SUCCESS);
   return EXIT_SUCCESS; /* never reached -- make compiler happy */
}

/* ------------------------ Initialisation ----------------------- */

/* Exists:  return true if given file exists */
Boolean Exists(char *fn)
{
   FILE *f;
   
   if ((f=fopen(fn,"r")) == NULL) return FALSE;
   fclose(f);
   return TRUE;
}


/* Initialise: initialise global data structures */
void Initialise(BuildInfo *bi)
{
   float weight;
   char *fn;

   CreateWordMap(wmapFN,&wmap,freeSlots);
   SortWordMap(&wmap);

   CreateHeap(&langHeap,"LM heap",MSTAK,1,0.5,1000,20000);
   if (fofFN!=NULL)
      bi->ftab = ReadFoFTab(&langHeap,fofFN);
   if (srcFN!=NULL) {
      srcLM = LoadLangModel(srcFN,NULL,1.0,LMP_FLOAT|LMP_COUNT,&langHeap);
      if (srcLM->probType==LMP_FLOAT && bi->ptype==LMP_COUNT)
	 HError(16820,"LBuild: input model incompatible with -x option");
      if (srcLM->probType==LMP_COUNT && bi->ptype==LMP_FLOAT) {
	 if (trace&T_TOP)
	    printf("Normalising input model %s\n",srcFN);
	 RebuildLM(srcLM,NULL,NULL,LMP_FLOAT);
      }
   }

   weight = 1.0;
   CreateInputSet(&gstack,&wmap,&inSet);
   while (NextArg() == STRINGARG || NextArg() == FLOATARG) {
      if (NextArg() == FLOATARG)  
	 weight = GetFltArg();
      if (weight==0.0 || weight<-10000.0 || weight>10000.0)
         HError(-16819,"Unlikely ngram weight[%.4f]",weight);
      if (NextArg()!=STRINGARG)
         HError(16819,"Gram base file name expected");
      fn = GetStrArg();
      AddInputGFile(&inSet,fn,weight);
      if (trace&T_TOP)
         printf("Input file %s added, weight=%.4f\n",fn,weight);
   }
   bi->wmap = &wmap;
   bi->inSet = &inSet;
   if (bi->nSize==0) bi->nSize = inSet.N;
}
