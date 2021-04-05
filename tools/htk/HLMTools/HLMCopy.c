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
/*   File: HLMCopy.c  LM conversion/normalisation utility      */
/* ----------------------------------------------------------- */

char *prog_version = "!HVER!HLMCopy:   3.4.1 [CUED 12/03/09]";
char *prog_vc_id = "$Id: HLMCopy.c,v 1.1.1.1 2006/10/11 09:54:44 jal58 Exp $";

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
#include "HDict.h"
#include "LWMap.h"
#include "LGBase.h"
#include "LUtil.h"
#include "LModel.h"
#include "LPCalc.h"
#include "LPMerge.h"

#define T_TOP       0001

typedef struct dictlist {
   char *fname;
   struct dictlist *next;
}
dictList;

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
static int       cutOff[LM_NSIZE+1];     /* new cutoffs */

static Vocab     vocab;                  /* word list buffer */
static Vocab     *voc   = NULL;          /* the defining word list */
static dictList  *dList = NULL;          /* list of dictionaries */
static char      *outDictFn = NULL;      /* dictionary to write */
static char      *uniFn = NULL;          /* unigram file name */
static Boolean   firstOnly = FALSE;      /* Only use first dictionary with pron */
static Boolean   remDup = TRUE;          /* Don't add duplicate prons */


/* ---------------- Configuration Parameters --------------------- */

static ConfParam *cParm[MAXGLOBS];
static int nParm = 0;                  /* total num params */


/* ---------------- Function Prototypes -------------------------- */

void ReplaceUnigrams(char *fn, BackOffLM *lm);
void MakeDictionary(char *fn, dictList *dicts, Vocab *wlist);


/* ---------------- Process Command Line ------------------------- */

/* SetConfParms: set conf parms relevant to this tool */
void SetConfParms(void)
{
   int i;

   nParm = GetConfig("HLMCOPY", TRUE, cParm, MAXGLOBS);
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
   printf("\nUSAGE: HLMCopy [options] inModel outModel\n\n");
   printf(" Option                                       Default\n\n");
   printf(" -c n c  set pruning for n-gram to c          0\n");
   printf(" -d s    read source dict from s              none\n");
   printf(" -f s    set output LM format to s            %s\n", ReturnLMName(DEF_SAVEFMT));
   printf(" -m      allow multiple identical prons       all\n");
   printf(" -n n    save model as n-gram                 max\n");
   printf(" -o      copy first dictionary pron to output all\n");
   printf(" -u s    read new unigrams from s             none\n");
   printf(" -v s    write pruned dictionary to s         none\n");
   printf(" -w fn   prune model using word list in fn    off\n");
   PrintStdOpts("ST");
   printf("\n\n");
}

int main(int argc, char *argv[])
{
   int i;
   char *s,*c;
   char fmt[256];
   dictList *dEntry,*d;

   InitShell(argc,argv,prog_version,prog_vc_id);
   InitMem();
   InitMath();
   InitWave();
   InitLabel();
   InitDict();
   InitWMap();
   InitLUtil();
   InitLModel();
   InitPCalc();

   if (!InfoPrinted() && NumArgs() == 0)
      ReportUsage();
   if (NumArgs() == 0) Exit(EXIT_SUCCESS);

   SetConfParms();

   CreateHeap(&langHeap,"langHeap",MSTAK,1,0.5,5000,40000);

   for (i=1; i<=LM_NSIZE; i++) cutOff[i] = 0;

   while (NextArg() == SWITCHARG) {
      s = GetSwtArg();
      if (strlen(s)!=1) 
         HError(16919,"Bad switch %s; must be single letter",s);
      switch(s[0]){
         case 'c':
           i = GetChkedInt(2,LM_NSIZE,s); 
	   cutOff[i] = GetChkedInt(1,1000,s);
	   break;
         case 'd':
	   if (NextArg()!=STRINGARG)
	     HError(16919,"LMCopy: Input dictionary file name expected");
	   dEntry=New(&gcheap,sizeof(dictList));
	   dEntry->fname=GetStrArg(); dEntry->next=NULL;
	   if (dList==NULL) dList=dEntry;
	   else {
	     for (d=dList;d->next!=NULL;d=d->next);
	     d->next=dEntry;
	   }
	   break;
         case 'f':
	   strcpy(fmt,GetStrArg());
	   for (c=fmt; *c!=0; *c=toupper(*c), c++);
	    if (strcmp(fmt, LM_TXT_TEXT)==0)
               saveFmt = LMF_TEXT;
	    else if (strcmp(fmt, LM_TXT_BINARY)==0)
               saveFmt = LMF_BINARY;
	    else if (strcmp(fmt, LM_TXT_ULTRA)==0)
               saveFmt = LMF_ULTRA;
	    else
	       HError(16919,"Unrecognised LM format, should be one of [%s, %s, %s]",
		      LM_TXT_TEXT, LM_TXT_BINARY, LM_TXT_ULTRA);
	   break;
	 case 'm':
	   remDup=FALSE;
	   break;
         case 'n':
            nSize = GetChkedInt(1,LM_NSIZE,s); break;
	 case 'o':
	   firstOnly=TRUE;
	   break;
	 case 'u':
	   if (NextArg()!=STRINGARG)
	     HError(16919,"LMCopy: Unigram file name expected");
	   uniFn = GetStrArg();
	   break;
         case 'v':
	   if (NextArg()!=STRINGARG)
	     HError(16919,"LMCopy: Dictionary output file name expected");
	   outDictFn = GetStrArg();
	   break;
         case 'w':
	    if (NextArg() != STRINGARG)
	       HError(16919,"LPlex: Word list file name expected");
	    wlistFN = GetStrArg();
	    break;
	 case 'T':
	    trace = GetChkedInt(0,077, s); break;
         default:
            HError(16919,"LMPlex: Unknown switch %s",s);
      }
   }
   if (NextArg()!=STRINGARG)  /* load the language model */
      HError(16919, "Input language model filename expected");
   srcFN = GetStrArg();

   if (NextArg()!=STRINGARG)  /* load the language model */
      HError(16919, "Output language model filename expected");
   tgtFN= GetStrArg();

   if (wlistFN!=NULL) {
      InitVocab(&vocab);   
      if(ReadDict(wlistFN,&vocab) < SUCCESS) 
	 HError(16913,"Could not read dict in %s", wlistFN);
      if (trace&T_TOP) {
	printf("Loaded %d words from %s\n",vocab.nwords,wlistFN); 
	fflush(stdout);
      }
      voc = &vocab;
      CreateWordList(wlistFN,&wlist,10);
      lm = LoadLangModel(srcFN,&wlist,1.0,LMP_FLOAT|LMP_COUNT,&langHeap);
   } else {
      voc = NULL;
      lm = LoadLangModel(srcFN,NULL,1.0,LMP_FLOAT|LMP_COUNT,&langHeap);
   }
   if (trace&T_TOP) {
     printf("Loaded model from %s\n",srcFN); 
     fflush(stdout);
   }
   if (lm->probType==LMP_COUNT) {
      RebuildLM(lm, cutOff, NULL, LMP_FLOAT); /* GLM there was no threshold before! */
   }
   if (uniFn!=NULL)
      ReplaceUnigrams(uniFn,lm);
   if (nSize>0 && nSize<lm->nSize)
      lm->nSize = nSize;
#ifdef HTK_CRYPT
   if (lm->encrypt && saveFmt==LMF_TEXT)
     saveFmt = LMF_BINARY;
#endif
   for (i=1;i<=lm->nSize;i++)
      lm->gInfo[i].fmt = (i==1) ? LMF_TEXT : saveFmt;
   SaveLangModel(tgtFN,lm);
   if (trace&T_TOP) {
     printf("Wrote model to %s\n",tgtFN); 
     fflush(stdout);
   }
   if (outDictFn) {
      MakeDictionary(outDictFn,dList,voc);
   }

   Exit(EXIT_SUCCESS);
   return EXIT_SUCCESS; /* never reached -- make compiler happy */
}   

/* ReplaceUnigrams: replace unigrams in lm with ones from fn */
void ReplaceUnigrams(char *fn, BackOffLM *lm)
{
   float prob;
   SMEntry *se;
   NameId wdid;
   int ndx,nItem;
   FLEntry *root;
   char buf[MAXSTRLEN];
   Source src;

   nItem = 0;
   root = &lm->root;
   if(InitSource(fn,&src,NoFilter)<SUCCESS)
      HError(16910,"ReplaceUnigrams: Can't open file %s", fn);
   do {
      if (!ReadFloat(&src,&prob,1,FALSE)) break;
      if (!ReadRawString(&src,buf)) break; /* or ReadString if HTK escaped */
      if ((wdid = GetNameId(lm->htab,buf,FALSE))==NULL) {
         printf("skipping '%s'\n", buf);
         continue;
      }
      ndx = LM_INDEX(wdid); se = root->sea+ndx-1;
      if (se->ndx!=ndx && (se=FindSE(root->sea,0,root->nse,ndx))==NULL) {
         printf("ignoring '%s'\n", buf);
         continue;
      }
      se->prob = exp(prob*LN10);
      nItem++;
   } while(SkipLine(&src));
   CloseSource(&src);
   if (trace&T_TOP) {
      printf("Replaced %d unigrams from %s\n",nItem,uniFn); fflush(stdout);
   }
}

/* DumpPhoneTable: copy unseen phones from src to tgt */
static void DumpPhoneTable(Vocab *src, Vocab *tgt) {
#ifdef HTK_HAPI
  int i;

  if (src->nphones==0 || src->ptab==NULL)
    return;
  if (tgt->ptab==NULL) {
    tgt->ptab = (LabId *) New(&gstack,MAXPHONES*sizeof(LabId));
    tgt->nphones = 0;
  }
  for (i=0; i<tgt->nphones; i++)    /* mark seen phones */
    tgt->ptab[i]->aux = (Ptr) tgt;
  for (i=0; i<src->nphones; i++) {  
    if (src->ptab[i]->aux == (Ptr) tgt)
      continue;
    if (tgt->nphones==MAXPHONES)
      HError(16920,"Maximum number of phones reached (%d)",MAXPHONES);
    tgt->ptab[tgt->nphones++] = src->ptab[i];
    src->ptab[i]->aux = (Ptr) tgt;
  }
#endif  
}

/* MakeDictionary: merge one or more dictionaries into a single one */
void MakeDictionary(char *fn,dictList *dicts,Vocab *wlist)
{
   Word word,fnd,cur;
   Pron pron,chk;
   Vocab tDict,dict;
   dictList *d;
   LabId blank=GetLabId("",TRUE);
   int i,l,n,m,p;
   
   /* Read dictionary collection */
   InitVocab(&dict);   
   for (d=dicts;d!=NULL;d=d->next) {
      InitVocab(&tDict);
      if(ReadDict(d->fname,&tDict)<SUCCESS)
	 HError(16913,"Could not read dict in %s", d->fname);
      if (trace&T_TOP) {
	 printf("Loaded %d words from %s\n",tDict.nwords,d->fname);
	 fflush(stdout);
      }
#ifdef HTK_CRYPT
      if (tDict.encrypt)
	 dict.encrypt=TRUE;
#endif
      DumpPhoneTable(&tDict,&dict);
      for (i=0,n=0,m=0,p=0; i<VHASHSIZE; i++)
	 for (word=tDict.wtab[i]; word!=NULL; word=word->next) 
	    if (word!=tDict.nullWord && word!=tDict.subLatWord) {
	       if (wlist==NULL) fnd=word;
	       else fnd=GetWord(wlist,word->wordName,FALSE);
	       cur=GetWord(&dict,word->wordName,FALSE);
	       if (fnd!=NULL && !(firstOnly && cur!=NULL)) {
		  n++;
		  cur=GetWord(&dict,word->wordName,TRUE);
		  if (word->pron==NULL) m++;
		  for (pron=word->pron;pron!=NULL;pron=pron->next) {
		     if (remDup) {
			for (chk=cur->pron;chk!=NULL;chk=chk->next) {
			   if (chk->nphones!=pron->nphones ||
			       chk->prob!=pron->prob) 
			      continue;
			   for(l=0;l<chk->nphones;l++) 
			      if (chk->phones[l]!=pron->phones[l]) break;
			   if (l==chk->nphones) break;
			}
			if (chk!=NULL) continue;
		     }
		     p++;
		     NewPron(&dict,cur,pron->nphones,pron->phones,
			     pron->outSym==NULL?blank:pron->outSym,
			     pron->prob>log(MINPRONPROB)?exp(pron->prob):0.0);
		  }
	       }
	    }
      if (trace&T_TOP) {
	 printf("Copied %d words (%d null,%d prons) from %s\n",n,m,p,d->fname);
	 fflush(stdout);
      }
      ClearVocab(&tDict);
   }
   if (wlist!=NULL) { 
      /* Check dictionary covers word list */
      for (i=0,n=0; i<VHASHSIZE; i++)
	 for (word=wlist->wtab[i]; word!=NULL; word=word->next) {
	    fnd=GetWord(&dict,word->wordName,FALSE);
	    if (fnd==NULL)
	       HError((n++>10)?16930:-16930,
		      "HLMCopy: Cannot find definition for word %s",
		      word->wordName->name);
	 }
      if (n>0)
	 HError(9999,"HLMCopy: Dictionary missing required words");
   }
   /* Write dictionary */
   if(WriteDict(fn,&dict)<SUCCESS)
      HError(3214,"HLMCopy: WriteDict failed");
   if (trace&T_TOP) {
      printf("Wrote dictionary to %s\n",outDictFn); fflush(stdout);
   }
}

/* --------------------- end of HLMCopy.c ------------------------- */
