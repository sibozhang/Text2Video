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
/*    File: LPlex:  compute perplexity                         */
/* ----------------------------------------------------------- */

char *lplex_version = "!HVER!LPlex:     3.4.1 [CUED 12/03/09]";
char *lplex_vc_id = "$Id: LPlex.c,v 1.1.1.1 2006/10/11 09:54:44 jal58 Exp $";

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <math.h>
#include <assert.h>

#include "HShell.h"     /* HTK toolkit libraries */
#include "HMem.h"
#include "HMath.h"
#include "HWave.h"
#include "HLabel.h"
#include "LWMap.h"      /* LM toolkit libraries */
#include "LCMap.h"
#include "LGBase.h"
#include "LUtil.h"
#include "LModel.h"
#include "LPCalc.h"
#include "LPMerge.h"

#define VERSION   "3.2"

#define T_TOP       0001
#define T_SENT      0002
#define T_OOV       0004
#define T_PROB      0010
#define T_SEL       0020

#define MAX_OOV     500000
#define MAX_LM      16
#define MAX_TEST    16
#define LBUF_SIZE   2048
#define MAX_FILES   200000

 typedef struct {
   LabId wdid;
   int count;
} OOVEntry;

typedef struct {
   int nOOV;                /* number of OOVs */
   int nTok;                /* total number of tokens */
   int nUtt;                /* number of utterances */
   int nWrd;                /* number of words predicted */
   double logpp;            /* accumulated LM score */
   double logpp2;           /* accumulated logp^2 score */
   int uniqOOV;             /* number of unique oov's */
   OOVEntry oov[MAX_OOV];   /* array of OOVs */
} PStats;

/* -------------------- Global variables ----------------------- */

static int trace = 0;               /* trace level */

static WordMap wList;               /* the word list */
static char   *wlistFN = NULL;
static int     nWords;              /* number of words in list */
static int     nLModel;             /* number of loaded LMs */
static LMInfo  lmInfo[MAX_LMODEL];  /* array of loaded LMs */
static int     numTests;            /* number of tests to perform */
static int     testInfo[MAX_TEST];  /* the array of test records */
static PStats  sent;                /* per utterance accumulators */
static PStats  totl;                /* global accumulator */

static LabId sstId = NULL;          /* sentence start marker */
static LabId senId = NULL;          /* sentence end marker */
static LabId unkId = NULL;          /* sentence end marker */

static LabId pLab[LBUF_SIZE];       /* label array */

static Boolean skipOOV  = TRUE;     /* discard OOV in computation */
static Boolean printOOV = FALSE;    /* print uniqe OOV's and their frequencies */
static Boolean streamMode = FALSE;  /* stream mode */

static FileFormat lff = UNDEFF;     /* label file format */

static char   *nulName = "???";     /* name of null class */
static LabId  nulClass;             /* Id of NULCLASS phone label */
static int    unkEquiv = 0;         /* number of equivalent words outside the word list */

static NameId **l2nId;              /* array of LabId -> NameId lookup tables */
static LabId  *eqId;                /* label equivalence lookup table */
static int cutOff[LM_NSIZE+1];      /* new cutoffs for COUNT-models */
static float wdThresh[LM_NSIZE+1];  /* new wdThresh for COUNT-models */

static char *outStreamFN = NULL;
FILE *outStream;

MemHeap tempHeap;                   /* Stores data valid only for file */
MemHeap permHeap;                   /* Stores global stats */


/* ---------------- Configuration Parameters --------------------- */

static ConfParam *cParm[MAXGLOBS];
static int nParm = 0;            /* total num params */


/* ---------------- Static function prototypes required ---------- */
static void Initialise(void);
static void ProcessFiles(void);
static void AddEquiv(char * cl, char * eq);


/* ---------------- Process Command Line ------------------------- */

/* SetConfParms: set conf parms relevant to this tool */
void SetConfParms(void)
{
   int i;
   static char b[100];

   nParm = GetConfig("LPLEX", TRUE, cParm, MAXGLOBS);
   if (nParm>0){
      if (GetConfInt(cParm,nParm,"TRACE",&i)) trace = i;
     if (GetConfStr(cParm,nParm,"STARTWORD",b))   sstId = GetLabId(b, TRUE);
      if (GetConfStr(cParm,nParm,"ENDWORD",b))     senId = GetLabId(b, TRUE);
      if (GetConfStr(cParm,nParm,"UNKNOWNNAME",b)) unkId = GetLabId(b, TRUE);
   }

   if (!sstId) sstId = GetLabId(DEF_STARTWORD,TRUE);
   if (!senId) senId = GetLabId(DEF_ENDWORD,TRUE);
   if (!unkId) unkId = GetLabId(DEF_UNKNOWNNAME,TRUE);

}

void ReportUsage(void)
{
   printf("\nUSAGE: LPlex [options] langmodel labelFiles...\n\n");
   printf(" Option                                       Default\n\n");
   printf(" -c n c  set pruning for n-gram to c          off\n");
   printf(" -d n c  set weighted discount pruning to c   off\n");
   printf(" -e s t  Label t is equivalent to s           off\n");
   printf(" -i f s  interpolate with model s, weight f   off\n");
   printf(" -n N    calculate N-gram perplexity          max in LM\n");
   printf(" -o      print OOV word statistics            off\n");
   printf(" -s fn   print prob stream to file fn         off\n");
   printf(" -t      text stream mode                     off\n");
   printf(" -u      use OOV words in context             off\n");
   printf(" -w fn   use word list from fn                off\n");
   printf(" -z s    Redefine null class name to s        ???\n");
   PrintStdOpts("GIST");
   printf("\n\n");
}

int main(int argc, char *argv[])
{
   int i;
   char *s,*c,*e;

   InitShell(argc,argv,lplex_version,lplex_vc_id);
   InitMem();
   InitMath();
   InitWave();
   InitLabel();
   InitWMap();
   InitCMap();
   InitLUtil();
   InitLModel();
   InitPCalc();
   InitPMerge();
   SetConfParms();

   if (!InfoPrinted() && NumArgs() == 0)
      ReportUsage();
   if (NumArgs() == 0) Exit(EXIT_SUCCESS);

   nLModel = 1;
   for (i=1; i<=LM_NSIZE; i++) cutOff[i] = 0, wdThresh[i] = 0.0;
   CreateHeap(&permHeap, "permHeap", MSTAK, 1, 1.0, 4000, 20000);
   CreateHeap(&tempHeap, "tempHeap", MSTAK, 1, 1.0, 8000, 40000);
   while (NextArg() == SWITCHARG) {
      s = GetSwtArg();
      if (strlen(s)!=1)
         HError(16619,"Bad switch %s; must be single letter",s);
      switch(s[0]){
         case 'c':
            i = GetChkedInt(2,LM_NSIZE,s);
	    cutOff[i] = GetChkedInt(1,1000,s);
	    break;
         case 'd':
            i = GetChkedInt(2,LM_NSIZE,s);
	    wdThresh[i] = GetChkedFlt(0.0,1E10,s);
	    break;
         case 'e':
	    if (NextArg() != STRINGARG)
	      HError(16619,"LPlex: Eq Class Name Expected");
	    c = GetStrArg();
	    if (NextArg() != STRINGARG)
	      HError(16619,"LPlex: Eq Label Name Expected");
	    e = GetStrArg();
	    AddEquiv(c,e);
	    break;
	 case 'i':
            if (NextArg()!=FLOATARG)
	       HError(16619,"LPlex: Interpolation weight expected");
	    lmInfo[nLModel].weight = GetChkedFlt(0.0,1.0,s);
            if (NextArg()!=STRINGARG)
	       HError(16619,"LPlex: Interpolation LM filename expected");
	    lmInfo[nLModel].fn = GetStrArg();
	    nLModel++;
	    break;
	 case 'n':
	    testInfo[numTests++] = GetChkedInt(1, 10, s); break;
	 case 'o':
	    printOOV = TRUE; break;
          case 's':
	    if (NextArg() != STRINGARG)
	       HError(16619,"LPlex: Prob Stream file name expected");
	    outStreamFN = GetStrArg();
	    break;
	 case 't':
	    streamMode = TRUE; break;
	 case 'u':
	    skipOOV = FALSE; break;
         case 'w':
	    if (NextArg() != STRINGARG)
	       HError(16619,"LPlex: Word list file name expected");
	    wlistFN = GetStrArg();
	    break;
         case 'z':
	    if (NextArg() != STRINGARG)
	       HError(16619,"LPlex: New null class name expected");
	    nulName = GetStrArg();
	    break;
	 case 'G':
	    if (NextArg() != STRINGARG)
	       HError(16619,"Label File format expected");
	    if((lff = Str2Format(GetStrArg())) == ALIEN)
	       HError(16619,"Warning ALIEN Label file format set");
	    break;
	 case 'I':
	    if (NextArg() != STRINGARG)
	       HError(16619,"MLF file name expected");
	    LoadMasterFile(GetStrArg()); break;
	 case 'T':
	    trace = GetChkedInt(0,077, s); break;
         default:
            HError(16619,"LPlex: Unknown switch %s",s);
      }
   }
#ifdef HTK_TRANSCRIBER
   if (trace&T_PROB) trace=trace^T_PROB;
#endif
   if (NextArg()!=STRINGARG)  /* load the language model */
      HError(16619, "Language model filename expected");
   lmInfo[0].fn = GetStrArg();

   Initialise();
   ProcessFiles();

   Exit(EXIT_SUCCESS);
   return EXIT_SUCCESS; /* never reached -- make compiler happy */
}

/* -------------------- Label Equivalences ----------------------- */

typedef struct _Equiv Equiv;          /* list of equivalent labels */
struct _Equiv{
   LabId classId;
   LabId equivId;
   Equiv *next;
};

static Equiv *eqList = NULL;          /* List of equivalent label ids */


/* AddEquiv: Add the equivalent pair (cl,eq) to eqlist */
static void AddEquiv(char * cl, char * eq)
{
   Equiv *p;

   p=(Equiv*) New(&permHeap,sizeof(Equiv));
   p->classId = GetLabId(cl,TRUE);
   p->equivId = GetLabId(eq,TRUE);
   p->next = eqList; eqList = p;
}

/* NumEquiv: return the number of equivalence sets */
static int NumEquiv(void)
{
   Equiv *p;
   int count = 0;

   for (p=eqList; p!=NULL; p=p->next) count++;
   return count;
}

/* NormaliseName: convert all equiv labels to class name and upper case if set */
static void LinkEquiv(void)
{
   Equiv *p;
   LabId cl,eq;

   for (p=eqList; p!=NULL; p=p->next) {
      cl = p->classId; eq = p->equivId;
      if (eq->aux==NULL) {
	 eq->aux = (Ptr) (nWords + unkEquiv);
	 unkEquiv++;
      }
      eqId[(int) eq->aux] = cl;
   }
}

/* Initialise: perform global initialisations */
static void Initialise(void)
{
   int i,j,ndx;
   float x;
   LMInfo *li;
   Boolean inLM;
   LabId *wid,lab;
   NameId *na,nid;
   Boolean isPipe;

   nulClass = GetLabId(nulName,TRUE);

   /* normalise weights */
   for (x=0.0, i=1; i<nLModel; i++)
      x += lmInfo[i].weight;
   lmInfo[0].weight = 1.0-x;

   /* load all models */
   for (li=lmInfo, i=0; i<nLModel; i++, li++) {
      if (trace&T_TOP)
	 printf("Loading language model from %s\n",li->fn);
      li->lm = LoadLangModel(li->fn,NULL,1.0,LMP_LOG|LMP_COUNT,&permHeap);
      if (li->lm->probType==LMP_COUNT)
	 RebuildLM(li->lm,cutOff,wdThresh,LMP_LOG);
      AttachAccessInfo(li->lm);
   }

   if (trace&T_TOP) {
      printf("Using language model(s): \n");
      for (li=lmInfo,i=0; i<nLModel; i++,li++)
	 printf("  %d-gram %s, weight %.2f\n",li->lm->nSize,li->fn,li->weight);
   }
   if (numTests==0) {
      numTests=1; testInfo[0] = lmInfo[0].lm->nSize;
   }

   /* load or create word list */
   if (wlistFN!=NULL) {
      /* load word list from file */
      CreateWordList(wlistFN,&wList,nWords+10);
      nWords = wList.used;
      for (wid=wList.id, i=0; i<nWords; i++,wid++) /* assign lookup indices */
	 (*wid)->aux = (Ptr) (i+1);
   } else {
      /* derive word list from LMs */
      for (nWords=0,li=lmInfo, i=0; i<nLModel; i++, li++)
      {
 	 /* Obtain class-LM word list in a different way */
	 if (li->lm->classLM)
	 {
	   na = li->lm->classBM;

	   for (j=0; j<li->lm->classW; j++)
	   {
	     lab = GetLabId(na[j+1]->name, TRUE);
	     if (lab->aux==NULL)
	       lab->aux = (Ptr) (++nWords);
	   }
	 }
	 else
	 {
	   na = li->lm->binMap;

	   for (j=0; j<li->lm->vocSize; j++)
	   {
	     lab = GetLabId(na[j+1]->name,TRUE);
	     if (lab->aux==NULL)
	       lab->aux = (Ptr) (++nWords);
	   }
	 }
      }
      CreateWordList(NULL,&wList,nWords+10);
      for (li=lmInfo, i=0; i<nLModel; i++, li++) {
	/* Obtain class-LM word list in a different way */
	if (li->lm->classLM)
	{
	  na = li->lm->classBM;

	  for (j=0; j<li->lm->classW; j++)
	  {
	    lab = GetLabId(na[j+1]->name,TRUE);
	    ndx = ((int) lab->aux) - 1;
	    wList.id[ndx] = lab;
	  }
	}
	else
	{
	  na = li->lm->binMap;

	  for (j=0; j<li->lm->vocSize; j++)
	  {
	    lab = GetLabId(na[j+1]->name,TRUE);
	    ndx = ((int) lab->aux) - 1;
	    wList.id[ndx] = lab;
	  }
	}

      }
      wList.used = nWords;
   }
   if (trace&T_TOP) {
      printf("Found %d unique words in %d model(s)\n",nWords,nLModel);
      fflush(stdout);
   }
   if (unkId->aux==NULL && !skipOOV) {
      HError(16620,"LPlex: OOV class symbol %s not in word list",unkId->name);
   }
   if (sstId->aux==NULL) {
      HError(16620,"LPlex: sentence start symbol %s not in word list",sstId->name);
   }
   if (senId->aux==NULL) {
      HError(16620,"LPlex: sentence end symbol %s not in word list",senId->name);
   }

   /* create lookup table */
   l2nId = (NameId **) New(&permHeap,nLModel*sizeof(NameId *));
   /* create LabId -> NameId lookup arrays (one per LM) */
   for (li=lmInfo, i=0; i<nLModel; i++, li++, na++) {
      na = (NameId *) New(&permHeap,(nWords+2)*sizeof(NameId));
      for (wid = wList.id, j=0; j<nWords; j++, wid++) {
	if (li->lm->classLM)
	{
	  nid = na[(int) ((*wid)->aux)] = GetNameId(li->lm->classH, (*wid)->name, FALSE);
	}
	else
	{
	  nid = na[(int) ((*wid)->aux)] = GetNameId(li->lm->htab, (*wid)->name, FALSE);
	}
#ifdef SANITY
	 if (nid==NULL)
	    HError(-16625,"Unable to find word %s in model %s\n",(*wid)->name,li->fn);
#endif
      }
      l2nId[i] = na;
   }

   /* ensure words present at least in one model */
   for (wid = wList.id, j=0; j<nWords; j++, wid++) {
      for (inLM=FALSE,i=0; i<nLModel; i++, li++)
	 if (l2nId[i][(int) ((*wid)->aux)]!=NULL)
	    inLM = TRUE;
      if (!inLM)
	 HError(16625,"Unable to find word %s in any model\n",(*wid)->name);
   }

   /* create equivalence class lookup array */
   eqId = (LabId *) New(&permHeap,(nWords+NumEquiv()+2)*sizeof(NameId));
   for (wid = wList.id, i=0; i<nWords; i++, wid++) {
      eqId[(int) ((*wid)->aux)] = NULL;
   }

   /* link equivalence classes */
   LinkEquiv();

   /* open output stream */
   if (outStreamFN != NULL)
     if ((outStream = FOpen(outStreamFN,NoOFilter,&isPipe)) == NULL)
        HError(16610,"Initialise: unable to open output file %s",outStreamFN);

}

/* -------------------- OOV calculation/statistics ---------------------*/

/* CmpOOVE: qsort comparison for oov entries */
static int CmpOOVE(const void *p1, const void *p2)
{
   return strcmp(((OOVEntry *)p1)->wdid->name, ((OOVEntry *)p2)->wdid->name);
}

/* SortOOV: sort/unique OOV array of ps */
static int SortOOV(PStats *ps)
{
   int i, c;
   OOVEntry *ove;

   if (ps->uniqOOV==0)
     return 0;
   qsort(ps->oov, ps->uniqOOV, sizeof(OOVEntry), CmpOOVE);
   c = 0; ove = ps->oov;
   for (i=c+1; i<ps->uniqOOV; i++)
     if (CmpOOVE(ove+c, ove+i)==0)
       ove[c].count += ove[i].count;
     else {
	c++; ove[c] = ove[i];
     }
   return (ps->uniqOOV = c+1);
}

/* StoreOOV: store OOV wdid/count into ps */
static void StoreOOV(PStats *ps, LabId wdid, int count)
{
   int n;

   ps->oov[ps->uniqOOV].wdid  = wdid;
   ps->oov[ps->uniqOOV].count = count;
   ps->uniqOOV++; ps->nOOV++;
   if (ps->uniqOOV==MAX_OOV) {
     n = SortOOV(ps);
     printf("StoreOOV: sorting OOVs, compacting %d -> %d\n",MAX_OOV,n);
     if (n==MAX_OOV)
       HError(16630,"Maximum number of unique OOV's [%d] reached\n",MAX_OOV);
  }
}

/* ZeroStats: zero counts in PStats */
static void ZeroStats(PStats *ps)
{
   ps->nOOV = 0;
   ps->nUtt = 0;
   ps->nWrd = 0;
   ps->nTok = 0;
   ps->uniqOOV = 0;
   ps->logpp = ps->logpp2 = 0.0;
}

/* AddStats: add statistics from ps1 to ps2 */
static void AddStats(PStats *ps1, PStats *ps2)
{
   int i;

   for (i=0; i<ps1->uniqOOV; i++)
     StoreOOV(ps2, ps1->oov[i].wdid, ps1->oov[i].count);
   ps2->nTok += ps1->nTok;
   ps2->nUtt += ps1->nUtt;
   ps2->nWrd += ps1->nWrd;
   ps2->logpp += ps1->logpp;
   ps2->logpp2 += ps1->logpp2;
}

/* PrintInfo: print statistics from ps */
static void PrintInfo(PStats *ps, Boolean showOOV)
{
   int i;
   float ovr;
   LMInfo *li;
   OOVEntry *ove;
   double a, b, ppl, stdev;

   /* print perplexity measures */
   a = (ps->logpp)/(double) (ps->nWrd); b = (ps->logpp2)/(double) (ps->nWrd);
   ppl = exp(-a); stdev = b - a*a;
   printf("perplexity %.4f, var %.4f, utterances %d, words predicted %d\n",
	  ppl, stdev, ps->nUtt, ps->nWrd);
   fflush(stdout);

   /* calculate OOV rate and statistics */
   ovr = ((float) (ps->nOOV) / (float) (ps->nTok - ps->nUtt))*100.0;
   printf("num tokens %d, OOV %d, OOV rate %.2f%% (excl. %s)\n",
	  ps->nTok, ps->nOOV, ovr, senId->name);
   if (showOOV && (ps->uniqOOV > 0)) {
      SortOOV(ps);
      printf("unique OOVs [%d]\n", ps->uniqOOV);
      for (ove=ps->oov, i=0; i<ps->uniqOOV; i++, ove++)
	printf("%s \t%d\n", ove->wdid->name, ove->count);
      fflush(stdout);
   }
   for (li=lmInfo, i=0; i<nLModel; i++, li++) {
#ifndef HTK_TRANSCRIBER
      printf("\nAccess statistics for %s:\n", li->fn);
      PrintTotalAccessStats(stdout,li->lm);
#endif
      ResetAccessInfo(li->lm);
   }
}


/*-------------------------- Perplexity calculation --------------------------*/

#define IS_UNK(id)  (id==unkId || id->aux==NULL)
#define IS_SST(id)  (id==sstId)
#define IS_SEN(id)  (id==senId)

static LabId GetEQLab(LabId id)
{
   LabId cl;

   if (id->aux==NULL)
      return id;

   if ((cl = (LabId) eqId[(int) (id->aux)])==NULL)
      return id;
   return cl;
}

/* GetProb: return nSize-gram probability for ngram in wlab */
static double GetProb(LabId *wlab, int nSize)
{
   /*
      this routine will return the interpolated nSize-gram probability for
      the words in wlab. Note that the context maybe shortened in the
      case of multiple LMs and words which do not occur in some of them.
   */

   int i,j;
   LMInfo *li;
   Boolean inThisLM,inAnyLM;
   double x,prob,psum;
   NameId nGram[LM_NSIZE];

   if (nLModel==1) {
      inThisLM = TRUE;
      for (j=0; j<nSize; j++) {
	 if ((nGram[j] = l2nId[0][(int) (wlab[j]->aux)])==NULL)
	    inThisLM = FALSE;
      }
      if (inThisLM) {
         prob = GetNGramProb(lmInfo[0].lm, nGram, nSize);
      }
      else if (nSize > 1)
         prob = GetProb(wlab+1,nSize-1);
      else {
         prob = LZERO;
         HError(-16690,"GetProb: assigning zero probability");
      }
   } else {
      psum = 0.0;
      inAnyLM = FALSE;
      for (li=lmInfo, i=0; i<nLModel; i++, li++) {
	 for (inThisLM=TRUE, j=0; j<nSize; j++) {
	    if ((nGram[j] = l2nId[i][(int) (wlab[j]->aux)])==NULL)
	       inThisLM = FALSE;
	 }
	 if (!inThisLM)
	    continue;
         x = GetNGramProb(li->lm, nGram, nSize);

#ifdef INTERPOLATE_MAX
	 if ((x = exp(x)) > psum)
	    psum = x;
#else
	 psum += li->weight*exp(x);
#endif
	 inAnyLM = TRUE;
      }
      if (inAnyLM)
	 prob = log(psum);
      else if (nSize > 1)
	 prob = GetProb(wlab+1,nSize-1);
      else {
	 prob = LZERO;
	 HError(-16690,"GetProb: assigning zero probability");
      }
   }
   return prob;
}


/* CalcPerplexity: compute perplexity and other statistics */
static void CalcPerplexity(PStats *sent, LabId *pLab, int numPLabs, int nSize)
{
   int i,j;
   LabId *p;
   float prob;
   Boolean hasOOV;

   for (p=pLab, i=nSize-1; i<numPLabs; i++, p++)
   {
      if (pLab[i]==unkId)
	 continue;	           /* cannot predict OOVs */
      if (skipOOV)
      {
	 hasOOV = FALSE;
	 for (j=1; j<nSize; j++)
	 {
	    if (pLab[i-j]==unkId)
	    {
	      hasOOV=TRUE;
	      break;
	    }
	 }
	 if (hasOOV) continue; /* skip to next label since context contains OOV */
      }
      prob = GetProb(p, nSize);
      sent->nWrd++; sent->logpp += prob; sent->logpp2 += prob*prob;

      if (outStreamFN != NULL)
         fprintf(outStream,"%e\n",exp(prob));

      if (trace&T_PROB)
      {
	 printf("logP(%s |", pLab[i]->name);
	 for (j=1; j<nSize; j++)
	 {
	   printf(" %s%s", (j==1)?"":",", pLab[i-j]->name);
	 }
	 printf(") = %.4f\n", prob);
	 /* if (trace&T_INST_INFO) PrintInstStats(nSize); */
	 fflush(stdout);
      }
   }
   if (trace&T_SENT)
      PrintInfo(sent,FALSE);
}


/* ProcessLabelFile: compute perplexity and related statistics from labels */
static void ProcessLabelFile(char *fn, int nSize)
{
   LLink ll;
   double ppl;
   LabList *ref;
   LabId lab;
   Transcription *tr;
   int i,numPLabs,nLabel;

   tr = LOpen(&tempHeap, fn, lff);
   if (tr->numLists < 1) {
      HError(-16635,"ProcessLabelFile: transcription file %s is Empty",fn);
      return;
   }
   ref = GetLabelList(tr, 1);
   if (ref->head->succ == ref->tail) {
      HError(-16635,"ProcessLabelFile: transcription file %s is Empty",fn);
      return;
   }
   if (trace>0) {
      printf("Processing label file: %s\n", fn);
      fflush(stdout);
   }

   nLabel = CountLabs(ref);
   ZeroStats(&sent);
   sent.nTok = nLabel + 2; sent.nUtt = 1;

   /* copy labels into pLab, mapping OOVs */
   numPLabs = 0;
   if (sstId!=NULL)             /* add sentence start marker(s) */
      for (i=0; i<(nSize-1); i++) pLab[numPLabs++] = sstId;
   for (i=0,ll=ref->head->succ; i<nLabel; i++,ll=ll->succ) {
      lab = GetEQLab(ll->labid);
      if ((i==0) && IS_SST(lab)) {
	sent.nTok--; continue;
      }
      if ((i==(nLabel-1)) && IS_SEN(lab)) {
	 sent.nTok--; continue;
      }
      if (IS_UNK(lab)) {
	 if (trace&T_OOV)
	    printf("mapping OOV: %s\n", lab->name);
	 StoreOOV(&sent,lab,1); lab = unkId;
      }
      pLab[numPLabs++] = lab;
      if (numPLabs>=LBUF_SIZE) {
         HError(16650, "Maximum utterance length in a label file exceeded (limit is compiled to be %d tokens)",
                LBUF_SIZE);
      }
   }
   if (senId!=NULL)             /* add sentence end marker */
     pLab[numPLabs++] = senId;

   CalcPerplexity(&sent, pLab, numPLabs, nSize);
   AddStats(&sent, &totl);

   if (trace&T_SEL) {     /* compact info for sentence selection */
      ppl = exp(-(sent.logpp)/(double) (sent.nWrd));
      printf("#! %.4f", ppl);
      for (i=0, ll=ref->head->succ; i<nLabel; i++, ll=ll->succ)
	 printf(" %s", ll->labid->name);
      printf("\n"); fflush(stdout);
   }
}

/* PPlexStream: compute perplexity and related statistics */
static void ProcessTextStream(char *fn, int nSize)
{
   int i;
   FILE *f;
   LabId lab=0;
   double ppl;
   int numPLabs;
   Boolean isPipe;
   char word[256];

   if (fn!=NULL) {
      if ((f=FOpen(fn, LMTextFilter, &isPipe))==NULL)
	 HError(16610,"ProcessTextStream: unable to open file %s", fn);
   } else {
      f = stdin;
   }
   if (trace>0) {
      printf("Processing text stream: %s\n", (fn==NULL)?"<stdin>":fn);
      fflush(stdout);
   }
   numPLabs = 0;
   ZeroStats(&sent);
   sent.nUtt = 1; sent.nTok = 0;
   while ((fscanf(f, "%200s", word))==1) {
      if (strlen(word)>=200)
	 HError(-16640, "ProcessTextStream: word too long, will be split: %s\n", word);
      lab = GetEQLab(GetLabId(word, TRUE));

   if (IS_SST(lab)) {
	 numPLabs = 0;
	 for (i=0; i<(nSize-1); i++) pLab[numPLabs++] = sstId;
	 ZeroStats(&sent);
	 sent.nUtt = 1; sent.nTok = 1;
	 continue;
      }
      if (IS_UNK(lab)) {
	 if (trace&T_OOV)
	    printf("mapping OOV: %s\n", lab->name);
	 StoreOOV(&sent,lab,1); lab = unkId;
      }
      pLab[numPLabs++] = lab; sent.nTok++;
      if (numPLabs>=LBUF_SIZE) {
         HError(16645,"ProcessTextStream: word buffer size exceeded - too many words without a sentence end (%d)",LBUF_SIZE);
	 CalcPerplexity(&sent,pLab,numPLabs,nSize);
	 numPLabs = 0;
      }
      if (IS_SEN(lab)) {
	 CalcPerplexity(&sent,pLab,numPLabs,nSize);
	 AddStats(&sent, &totl);

	 if (trace&T_SEL) {     /* compact info for sentence selection */
	   ppl = exp(-(sent.logpp)/(double) (sent.nWrd));
	   printf("#! %.4f", ppl);
	   for (i=nSize-1; i<numPLabs; i++)
	     printf(" %s", pLab[i]->name);
	   printf("\n"); fflush(stdout);
	 }

	 ZeroStats(&sent);
      }
   }
   AddStats(&sent,&totl);

   if (fn!=NULL)
      FClose(f,isPipe);
}

/* ProcessFiles: process label files */
static void ProcessFiles()
{
   int nSize;
   char *labFn;
   MLFEntry *me;
   char *inpfn[MAX_FILES];
   int i,t,numFiles,fidx;

   numFiles = 0;
   while (NumArgs()>0){
      if (NextArg()!=STRINGARG)
	HError(16619,"ProcessFiles: label file (MLF) name expected");
      inpfn[numFiles++] = CopyString(&gstack, GetStrArg());

      if (numFiles == MAX_FILES) {
	 HError(-16619,"Processing only the first %d files",MAX_FILES);
      }
   }

   for (t=0; t<numTests; t++) {
      ZeroStats(&totl);
      nSize = testInfo[t];
      printf("LPlex test #%d: %d-gram\n", t, nSize);
      if (numFiles==0) {
	 ProcessTextStream(NULL, nSize);
	 continue;
      }

      for (i=0; i<numFiles; i++) {
	 labFn = inpfn[i];
	 if (streamMode) {
	    ProcessTextStream(labFn,nSize);
	 } else {
	    if (IsMLFFile(labFn)) {
	       if (trace>0) {
		  printf("Processing MLF: %s\n", labFn);
		  fflush(stdout);
	       }
	       fidx = NumMLFFiles();
	       if ((me=GetMLFTable()) != NULL) {
		  while(me->next != NULL) me=me->next;
		  LoadMasterFile(labFn);
		  me=me->next;
	       }
	       else{
		  LoadMasterFile(labFn);
		  me=GetMLFTable();
	       }
	       while (me != NULL) {
		  if (me->type == MLF_IMMEDIATE && me->def.immed.fidx == fidx) {
		     ProcessLabelFile(me->pattern,nSize);
		  }
		  me = me->next;
	       }
	    } else {
	       ProcessLabelFile(labFn,nSize);
	    }
	 }
      }
      PrintInfo(&totl, printOOV);
   }
}

/* --------------------- End of LPlex.c  ------------------------ */
