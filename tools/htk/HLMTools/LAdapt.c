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
/*         File: LAdapt.c - adapt LM with new text             */
/* ----------------------------------------------------------- */

char *ladapt_version = "!HVER!LAdapt:   3.4.1 [CUED 12/03/09]";
char *ladapt_vc_id = "$Id: LAdapt.c,v 1.1.1.1 2006/10/11 09:54:44 jal58 Exp $";

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

/* 
   This tool processes source texts and updates an existing LM.
   Text passes through a window word by word and each n-gram is recorded.
   The text in the window can also be modified by match and replace rules 
   and in this case the ngrams in the original matched text are stored 
   in a set of 'negative' gram files and the ngrams in the modified text 
   are stored in a set of 'positive' gram files.
*/
   
/* -------------------------- Trace Flags ------------------------ */

static int trace = 0;
#define T_TOP  0001     /* Top Level tracing */
#define T_SAV  0002     /* Monitor Buffer Saving */
#define T_INP  0004     /* Trace word input stream */
#define T_SHR  0010     /* Trace shift register input */

/* ---------------- Configuration Parameters --------------------- */

static ConfParam *cParm[MAXGLOBS];
static int nParm = 0;   /* total num params */


/* ------------------- Word Shift Registers ----------------------- */

typedef struct {
   int used;            /* actual words in register */
   UInt ng[MAXNG+1];    /* ng[0] is oldest word */
   NGBuffer *ngb;       /* output ngram buffer */
} ShiftReg;

/* ---------------------- Global Variables ----------------------- */

static int nSize     = 3;           /* ngram size */
static int ngbSize   = 2000000;     /* ngram buffer size */
static int newWords  =  100000;     /* max new words to accommodate */
static char *rootFN  = "gram";      /* gbase root file name */
static char *outFN   = NULL;        /* output LM filename */
static char *dbsDir  = NULL;        /* directory to store gbase files */
static char *wlistFN = NULL;        /* file containing edit rules */
static char *omapFN  = "wmap";      /* output word map file name */
static char *txtSrc  = NULL;        /* gram file text source descriptor */
static MemHeap langHeap;            /* memory for NGBuffers and LMs*/

static BackOffLM *newLM;            /* the generated LM */
static BackOffLM *adpLM;            /* the adapted final LM */
static WordMap   *tgtVoc = NULL;    /* target vocabulary */          
static WordMap   wlist;             /* restricting the word list */
static ShiftReg  stdBuf;            /* used for normal N-gram processing */

static Boolean pruneWords = FALSE;    /* prune input text according to word list */
static Boolean saveFiles = TRUE;      /* save intermediate files */ 
static Boolean htkEscape = TRUE;      /* string escaping for output word map */
static Boolean mapUpdated;            /* used optimise sort/saving */
static Boolean processText = TRUE;    /* generate model from raw text data */
static char *defMapName = "LAdapt";   /* map name */

static LabId unkId = NULL;                   /* OOV marker */
static char  unkStr[256] = DEF_UNKNOWNNAME;  /* OOV class string */

/* This MAX_NGRAM_FILES limit is arbitrary and can be removed */
#define MAX_NGRAM_FILES  4096

static int     nLModel;                     /* number of loaded LMs */
static LMInfo  lmInfo[MAX_LMODEL];          /* array of loaded LMs */

static WordMap    wmap;              /* word map for this corpus */
static NGInputSet inSet;             /* input set of files */
static BuildInfo  binfo;             /* build parameters */

/* ---------------- Process Command Line ------------------------- */

/* SetConfParms: set conf parms relevant to this tool */
void SetConfParms(void)
{
   int i;
   char s[256];
   
   nParm = GetConfig("LADAPT", TRUE, cParm, MAXGLOBS);
   if (nParm>0){
      if (GetConfInt(cParm,nParm, "TRACE",&i))      trace = i;
      if (GetConfStr(cParm,nParm, "UNKNOWNNAME",s)) strcpy(unkStr,s);
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
   printf("\nUSAGE: LAdapt [options] langModel txtfile ....\n\n");
   printf(" Option                                       Default\n\n");
   printf(" -a n    allow n new words in input text      %d\n", newWords);
   printf(" -b n    set ngram buffer size                %d\n", ngbSize);
   printf(" -c n c  set pruning for n-gram to c          %d\n", DEF_CUTOFF);
   printf(" -d s    set root n-gram data file name       %s\n", rootFN);
   printf(" -f s    set output LM format to s            %s\n", ReturnLMName(DEF_SAVEFMT));
   printf(" -g      use existing n-gram files            off\n");
   printf(" -i f s  interpolate with model s, weight f   off\n");
   printf(" -j n c  set weighted discount pruning to c   off\n");
   printf(" -n n    set n-gram size                      %d\n", nSize);
#ifndef HTK_TRANSCRIBER
   printf(" -s s    store s in gram header source flds   none\n");
   printf(" -t      use Turing-Good discounting          off\n");
#endif
   printf(" -w fn   load word list from fn               none\n");
#ifndef HTK_TRANSCRIBER
   printf(" -x      save model with counts               off\n");
#endif
   PrintStdOpts("");
   printf("\n\n");
}

int main(int argc, char *argv[])
{
   int i;
   char *c,*s,*fn;
   char sBuf[256],fmt[256];

   void       Initialise(void);
   void       ProcessText(char *fn,Boolean lastFile);
   Boolean    Exists(char *fn);
   BackOffLM *CombineModels(MemHeap *heap,LMInfo *lmi,int nLModel,int nSize,WordMap *wl) ;

   InitShell(argc,argv,ladapt_version,ladapt_vc_id);
   InitMem();
   InitMath();
   InitWave();
   InitLabel();
   InitLUtil();
   InitWMap();
   InitGBase();
   InitLModel();
   InitPCalc();
   InitPMerge();

   SetConfParms();

   if (!InfoPrinted() && NumArgs() == 0)
      ReportUsage();
   if (NumArgs() == 0) Exit(EXIT_SUCCESS);

   InitBuildInfo(&binfo); 
   binfo.dctype = DC_ABSOLUTE;
   nLModel = 1;
   while (NextArg() == SWITCHARG) {
      s = GetSwtArg();
      if (strlen(s)!=1) 
         HError(16419,"Bad switch %s; must be single letter",s);
      switch(s[0]){
         case 'a':
            newWords = GetChkedInt(10,10000000,s); break;
         case 'b':
            ngbSize = GetChkedInt(10,10000000,s); break;
         case 'c':
            i = GetChkedInt(2,LM_NSIZE,s); 
	    binfo.cutOff[i] = GetChkedInt(0,1000,s);
	    break;
         case 'd':
            if (NextArg()!=STRINGARG)
               HError(16419,"Gram base root file name expected");
            rootFN = GetStrArg(); 
	    break;
         case 'f':
	    strcpy(fmt, GetStrArg());
	    for (c=fmt; *c; *c=toupper(*c), c++); /* To uppercase */
	    if (strcmp(fmt, LM_TXT_TEXT)==0)
	      binfo.saveFmt = LMF_TEXT;
	    else if (strcmp(fmt, LM_TXT_BINARY)==0)
	       binfo.saveFmt = LMF_BINARY;
	    else if (strcmp(fmt, LM_TXT_ULTRA)==0)
	       binfo.saveFmt = LMF_ULTRA;
	    else
	       HError(16419,"Unrecognised LM format, should be one of [%s, %s, %s]",
		      LM_TXT_TEXT, LM_TXT_BINARY, LM_TXT_ULTRA);
	    break;
         case 'g':
            processText = FALSE; break;
	 case 'i':
            if (NextArg()!=FLOATARG)
	       HError(16419,"Interpolation weight expected");
	    lmInfo[nLModel].weight = GetChkedFlt(0.0,1.0,s);
            if (NextArg()!=STRINGARG)
	       HError(16419,"Interpolation LM filename expected");
	    lmInfo[nLModel].fn = GetStrArg();
	    nLModel++;
	    break;
         case 'j':
            i = GetChkedInt(2,LM_NSIZE,s); 
	    binfo.wdThresh[i] = GetChkedFlt(0.0,1E10,s);
	    break;
         case 'n':
            nSize = GetChkedInt(1, MAXNG, s); break;
#ifdef HTK_TRANSCRIBER
         case 's':
            if (NextArg()!=STRINGARG)
               HError(16419,"Gram file text source descriptor expected");
            txtSrc = GetStrArg(); break;
         case 't':
	    binfo.dctype = DC_KATZ; break;
#endif
         case 'w':
            if (NextArg()!=STRINGARG)
               HError(16419,"Word list file name expected");
            wlistFN = GetStrArg(); break;
#ifndef HTK_TRANSCRIBER
         case 'x':
            binfo.ptype = LMP_COUNT; break;
#endif
         case 'T':
            trace = GetChkedInt(0,077,s); break;
         default:
            HError(16419,"LAdapt: Unknown switch %s",s);
      }
   }
#ifdef HTK_TRANSCRIBER
   if (nLModel==1) {  /* must interpolate with at least one model */
      HError(16419,"LAdapt: at least one model must be specified with -i option");
   }
   if (binfo.saveFmt==LMF_TEXT) { /* save fomat cannot be TEXT */ 
      binfo.saveFmt=LMF_BINARY;
   }
#endif
   if (NextArg() != STRINGARG)
      HError(16419,"LAdapt: language model file name expected");
   outFN = CopyString(&gstack,GetStrArg());

   Initialise();
   if (processText) {
      if (NextArg() != STRINGARG)
	 ProcessText(NULL,TRUE);       /* input from stdin */
      else
	 while (NextArg() == STRINGARG) {
	    /* !! copy string argument since it gets overwritten 
	       by NextArg() when reading from script file */
	    fn = CopyString(&gstack,GetStrArg());
	    ProcessText(fn,NextArg() != STRINGARG);
	 }
      if (NumArgs() != 0)
	 HError(-16419,"LAdapt: unused args left on cmd line");
      for (i=0; i<stdBuf.ngb->fndx; i++) {
	 sprintf(sBuf,"%s.%d",stdBuf.ngb->fn,i);  
	 AddInputGFile(&inSet,sBuf,1.0);
      }
      ResetHeap(&langHeap);
   } else {
      for (i=0; i<MAX_NGRAM_FILES; i++) {
	 sprintf(sBuf,"%s.%d",rootFN,i);
	 if (!Exists(sBuf))
	    break;
	 AddInputGFile(&inSet,sBuf,1.0);
      }
      if (i==MAX_NGRAM_FILES)
      {
	HError(-16419, "LAdapt: Only %d n-gram files read (recompile with different setting\nof MAX_NGRAM_FILES");
      }
   }
   if (nLModel==1) {
      adpLM = GenerateModel(&langHeap,&binfo);
   } else {
      if (binfo.ptype==LMP_COUNT) 
	 binfo.ptype = LMP_FLOAT;
      newLM = GenerateModel(&langHeap,&binfo);
      lmInfo[0].lm = newLM;
      lmInfo[0].fn = "unknown";
      /* combine all models into one */
      adpLM = CombineModels(&langHeap,lmInfo,nLModel,nSize,tgtVoc);
   }
#ifdef HTK_TRANSCRIBER
#ifdef HTK_CRYPT
   adpLM->encrypt = TRUE;     /* force to write encrypted model */
#endif
#endif
   SaveLangModel(outFN,adpLM);

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
void Initialise(void)
{
   int  i;
   char path[256];

   CreateHeap(&langHeap,"LModel mem",MSTAK,1,0.5,1000,20000);

   if (wlistFN!=NULL) {
      tgtVoc = &wlist;
      CreateWordList(wlistFN,tgtVoc,10);
   }

   if (processText) {
      /* init empty buffer */
      CreateWordMap(NULL,&wmap,newWords); 
      wmap.hasCnts = TRUE;
      wmap.name = defMapName;
      wmap.htkEsc = htkEscape;
      ++wmap.seqno;
      mapUpdated = FALSE;
     
      if (tgtVoc!=NULL) {      /* add words from word list to the map */
	 pruneWords = TRUE;
	 for (i=0; i<tgtVoc->used; i++) {
	    AddWordToMap(&wmap,tgtVoc->id[i]);
	 }
	 SortWordMap(&wmap);
	 unkId = GetLabId(unkStr,FALSE);  
      }
      
      /* init ngram buffer */
      MakeFN(rootFN,dbsDir,NULL,path);
      stdBuf.used = 0;
      stdBuf.ng[nSize] = 1;  /* count = 1 */
      stdBuf.ngb = CreateNGBuffer(&langHeap,nSize,ngbSize,path,&wmap);
   } else {
      CreateWordMap(omapFN,&wmap,1);
   }
   
   CreateInputSet(&gstack,&wmap,&inSet);
   binfo.wmap = &wmap;
   binfo.inSet = &inSet;
   binfo.nSize = nSize;
}

/* ----------------- NGram Counting Routines -------------------- */

/* CompressBuffer: and save if necessary or mustSave is TRUE */
void CompressBuffer(NGBuffer *ngb, Boolean mustSave)
{
   float compx;

   if (ngb->used == 0) return;
   SortNGBuffer(ngb);
   compx = 100.0 * (float)ngb->used / (float)ngb->poolsize;
   if (trace&T_SAV)
      printf(" buffer %s.%d compressed%s to %.1f%%\n",
	     ngb->fn, ngb->fndx, mustSave?"[must save]":"",compx);
   if (compx > 75.0 || mustSave) {
      if (saveFiles && mustSave && mapUpdated) {
	 SaveWordMap(omapFN,&wmap,FALSE);
         mapUpdated = FALSE;
         if (trace&T_TOP) 
            printf(" word map saved to %s\n",omapFN);
      }
      if (trace&T_TOP) {
         printf(" saving %d ngrams to file %s.%d\n",
                 ngb->used, ngb->fn, ngb->fndx);
      }
      WriteNGBuffer(ngb,txtSrc);
   }
}

/* PutShiftRegister: push word into shift register and extract ngram */
void PutShiftRegister(LabId id, ShiftReg *sr)
{
   int i;
   MapEntry *me;
   
   if (trace&T_SHR){
      printf("   %12s --> %s\n",id->name,sr->ngb->fn);
      fflush(stdout);
   }
   AddWordToMap(&wmap,id); 
   mapUpdated = TRUE;
   me = (MapEntry *)id->aux;
   sr->ng[sr->used++] = me->ndx;
   if (sr->used == nSize) {
      /* record ngram */
      StoreNGram(sr->ngb,sr->ng);
      /* shift words */
      sr->used--;
      for (i=0; i<sr->used; i++)
         sr->ng[i] = sr->ng[i+1];
      /* compress buffer if full */
      if (sr->ngb->used == sr->ngb->poolsize)  {
         CompressBuffer(sr->ngb,FALSE);
      }
   }
}

/* ProcessText: read text files line by line and count ngrams */
void ProcessText(char *fn, Boolean lastFile)
{
   FILE *f;
   LabId id;
   Boolean isPipe;
   char word[256];

   if (trace&T_TOP) 
      printf("Reading source text file %s\n",(fn==NULL) ? "<stdin>" : fn);
   if ((fn!=NULL) && (strcmp(fn,"-")!=0)) {
      if ((f = FOpen(fn,LMTextFilter,&isPipe))==NULL)
	 HError(16410,"ProcessText: unable to open text file %s", fn);
   } else {
      f = stdin;
   }
   while (fscanf(f,"%255s",word)==1) {
      if (pruneWords) {
	 if ((id = GetLabId(word,FALSE))==NULL && (id = unkId)==NULL) {
	    stdBuf.used=0;
	    continue;
	 }
      } else {
	 id = GetLabId(word,TRUE);
      }
      if (trace&T_INP) printf("[%s]\n",id->name);
      PutShiftRegister(id,&stdBuf);
   }
   if (fn!=NULL) {
      FClose(f,isPipe);
      if (lastFile)
	 CompressBuffer(stdBuf.ngb,TRUE);
   } else {
      CompressBuffer(stdBuf.ngb,TRUE);
   } 
}

/* CombineModels: load models and combine with the one in memory */
BackOffLM *CombineModels(MemHeap *heap,LMInfo *lmi,int nLModel,int nSize,WordMap *wl) 
{
   int i,j,nw;
   float x;
   LMInfo *li;
   BackOffLM *tgtLM;
   WordMap wordList;
   LabId lab;
   NameId *na;

   /* normalise weights */
   for (x=0.0, i=1; i<nLModel; i++)
      x += lmInfo[i].weight;
   lmInfo[0].weight = 1.0-x;

   /* load all models except the first one*/
   for (li=lmInfo+1, i=1; i<nLModel; i++, li++) {
      if (trace&T_TOP)
	 printf("Loading language model from %s\n",li->fn);
      li->lm = LoadLangModel(li->fn,wl,1.0,LMP_FLOAT,heap);
   }
   if (wl==NULL) {
      wl = &wordList;
      /* derive word list from LMs */
      for (li=lmInfo, i=0; i<nLModel; i++, li++) {  
	 na = li->lm->binMap;
	 for (j=0; j<li->lm->vocSize; j++) {
	    lab = GetLabId(na[j+1]->name,TRUE);
	    lab->aux=NULL; 
	 }
      }
      for (nw=0,li=lmInfo, i=0; i<nLModel; i++, li++) {  
	 na = li->lm->binMap;
	 for (j=0; j<li->lm->vocSize; j++) {
	    lab = GetLabId(na[j+1]->name,FALSE);
	    if (lab->aux==NULL) {
	       nw++; lab->aux = (Ptr) wl;
	    }
	 }
      }
      CreateWordList(NULL,wl,nw+10);
      for (nw=0,li=lmInfo, i=0; i<nLModel; i++, li++) {
	 na = li->lm->binMap;
	 for (j=0; j<li->lm->vocSize; j++) {
	    lab = GetLabId(na[j+1]->name,FALSE);
	    if (lab->aux==(Ptr) wl) {
	       wl->id[nw++]=lab; lab->aux = NULL;
	    }
	 }
      }
      wl->used = nw;
   }
   if (trace&T_TOP) {
      printf("Using language model(s): \n");
      for (li=lmInfo,i=0; i<nLModel; i++,li++)
	 printf("  %d-gram %s, weight %.2f\n",li->lm->nSize,li->fn,li->weight);
   }
   if (trace&T_TOP) {
      printf("Generating %d-gram model %s\n",nSize,outFN);
      fflush(stdout);
   }
   tgtLM = MergeModels(heap,lmInfo,nLModel,nSize,wl);
#ifdef HTK_CRYPT   
   if (tgtLM->encrypt && binfo.saveFmt==LMF_TEXT)
      binfo.saveFmt = LMF_BINARY;
#endif
   for (i=1; i<=nSize; i++) {
      tgtLM->gInfo[i].fmt = (i==1) ? LMF_TEXT : binfo.saveFmt;
   }
   return tgtLM;
}

/* ---------------------- End of LAdapt.c ----------------------- */

