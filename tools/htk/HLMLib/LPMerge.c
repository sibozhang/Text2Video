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
/*      File: LPMerge.c: probability merging                   */
/* ----------------------------------------------------------- */

char *lpmerge_version = "!HVER!LPMerge:   3.4.1 [CUED 12/03/09]";
char *lpmerge_vc_id = "$Id: LPMerge.c,v 1.1.1.1 2006/10/11 09:54:43 jal58 Exp $";

#include "HShell.h"     /* HMM ToolKit Modules */
#include "HMem.h"
#include "HMath.h"
#include "HWave.h"
#include "HLabel.h"
#include "LWMap.h"      /* LM ToolKit Modules */
#include "LGBase.h"     
#include "LUtil.h"
#include "LModel.h"
#include "LPCalc.h"
#include "LPMerge.h"

#define T_TOP    0001       /* top level tracing */
#define T_PROB   0020       /* probability calculation */

/* -------------------------- Trace Flags ------------------------ */

static int trace = 0;
#define T_TOP  0001     /* Top Level tracing */

/* ---------------- Configuration Parameters --------------------- */

static ConfParam *cParm[MAXGLOBS];
static int nParm = 0;            /* total num params */

/* ---------------------- Global Variables ----------------------- */

/* SetConfParms: set conf parms relevant to this tool */
void InitPMerge(void)
{
   int i;

   Register(lpmerge_version,lpmerge_vc_id);
   nParm = GetConfig("LPMERGE", TRUE, cParm, MAXGLOBS);
   if (nParm>0){
      if (GetConfInt(cParm,nParm, "TRACE",&i))    trace = i;
   }
}

/* ---------------------- Model interpolation  -------------------- */

#define DOT_CHUNK     10000

typedef struct {
   BackOffLM *lm;            /* target model */
   char    *mapFN;           /* LM symbol map file */
   WordMap *wList;           /* the target word list */
   int     nLModel;          /* number of LMs to merge */
   LMInfo  *lmInfo;          /* array of LMs to merge */
   int     **l2nId;          /* array of LabId -> NameId lookup tables */
   int     **n2lId;          /* array of NameId -> LabId lookup tables */
   int     *mId;             /* map array for speedy FE/SE copying */
   int     progress;         /* progress tracking */
} MergeInfo;

/* InitTargetModel: initialise target model structure */
static BackOffLM *InitTargetModel(MemHeap *heap, int nSize, WordMap *wList)
{
   int i,N;
   FLEntry *fe;
   SMEntry *se;
   LabId *srcId;
   BackOffLM *lm;
   NameId wordId;

   N = wList->used;
   lm = (BackOffLM *) New(heap,sizeof(BackOffLM));
   lm->heap = heap;
   lm->htab = CreateHashTable(11731,"Back-off LM hash table");
   lm->gScale = 1.0;
   lm->nSize = nSize;
   lm->vocSize = N;
   lm->probType = LMP_FLOAT;

   /* initialise structures */
   lm->lmvec = (float *) New(lm->heap,N*sizeof(float));
   lm->fe_buff = (FLEntry *) New(lm->heap,(N)*sizeof(FLEntry));
   lm->se_buff = (SMEntry *) New(lm->heap,(N)*sizeof(SMEntry));
   lm->binMap =  (NameId *) New(lm->heap,(N)*sizeof(NameId));
   lm->binMap--;

   lm->root.sea = NULL; lm->root.nse = 0;
   lm->root.fea = NULL; lm->root.nfe = 0;
   lm->root.bowt = 0.0;

   for (i=1;i<=nSize; i++) {
      lm->gInfo[i].fmt  = LMF_TEXT;
      lm->gInfo[i].boInfo = NULL;
      lm->gInfo[i].nEntry = 0;
   }

   for (se=lm->se_buff,i=0; i<N; i++,se++) {
      se->ndx = 0; se->prob = 0.0;
   }

   for (fe=lm->fe_buff,i=0; i<N; i++,fe++) {
      fe->ndx = 0;
      fe->bowt = 0.0;
      fe->nse = fe->nfe = 0;
      fe->sea = NULL; 
      fe->fea = NULL;
   }

   /* process each word in the list in turn */
   for (srcId=wList->id,i=1; i<=N; i++,srcId++) {
      wordId = GetNameId(lm->htab,(*srcId)->name,TRUE);
      LM_INDEX(wordId) = i;
      lm->binMap[i] = wordId;
   }
   return lm;
}

/* InitialiseMerge: create lookup tables */
static void InitialiseMerge(MemHeap *heap, MergeInfo *mi)
{
   int i,j,nw;
   LMInfo *li;
   Boolean inLM;
   LabId  lid, *lip;
   NameId nid, *nip;
   int nWords, nLModel;
   int *na, *la, **l2nId, **n2lId, *mId;   

   nWords = mi->lm->vocSize;
   nLModel = mi->nLModel;
   
   /* assign lookup indeces */
   for (lip = mi->wList->id, j=1; j<=nWords; j++, lip++)
     (*lip)->aux = (Ptr) j;

   /* create LabId.aux -> NameId.aux lookup arrays (one per LM) */
   l2nId = mi->l2nId = (int **) New(heap,nLModel*sizeof(int *));
   for (li=mi->lmInfo, i=0; i<nLModel; i++, li++, na++) {
      na = (int *) New(heap,(nWords+2)*sizeof(NameId));
      for (lip = mi->wList->id, j=1; j<=nWords; j++, lip++) {
	 nid = GetNameId(li->lm->htab, (*lip)->name, FALSE);
	 na[j] = (nid==NULL) ? 0 : (int) nid->aux; 
      }
      l2nId[i] = na;
   }

   /* create NameId.aux -> LabId.aux lookup arrays (one per LM) */
   n2lId = mi->n2lId = (int **) New(heap,nLModel*sizeof(int *));
   for (li=mi->lmInfo, i=0; i<nLModel; i++, li++, na++) {
      nw = li->lm->vocSize;
      la = (int *) New(heap,(nw+2)*sizeof(int));
      for (nip=li->lm->binMap+1,j=1; j<=nw; j++,nip++) {
	 lid = GetLabId((*nip)->name, FALSE);
	 la[j] = (lid==NULL) ? 0 : (int) lid->aux;
      }
      n2lId[i] = la;
   }

   /* create index translation table */
   mId = mi->mId = (int *) New(heap,(nWords+1)*sizeof(int));
   for (i=0; i<=nWords; i++) mId[i] = -1;

   /* ensure words present at least in one model */
   for (lip = mi->wList->id, j=0; j<nWords; j++, lip++) {
      for (inLM=FALSE,i=0; i<nLModel; i++, li++)
	 if (l2nId[i][(int) ((*lip)->aux)]!=0)
	    inLM = TRUE;
      if (!inLM)
	 HError(15620,"InitialiseMerge: Unable to find word %s in any model\n",(*lip)->name);
   }
   mi->progress = 0;
}

/* CalcBackOff: calculate/normalise backoff weights */
static void CalcBackOff(BackOffLM *lm, FLEntry **context,int lev) 
{
   int i,feId[LM_NSIZE];
   SMEntry *se;
   FLEntry *fe,*tgtFE;
   float bosum,umass,*pvec;

   tgtFE = context[lev];
   if (lev > 0) {
      for (i=0; i<lev; i++) feId[i] = context[i+1]->ndx;    /* context[0] is the root FLEntry */
      pvec = GetNGramProbVecSE(lm,feId+1,lev,tgtFE);        /* shorten context */
      bosum = 0.0;
      umass = 0.0;
      for (se = tgtFE->sea, i=0; i<tgtFE->nse; i++, se++) {
	 bosum += pvec[se->ndx];
	 umass += se->prob;
      }
      /* printf("umass = %f, bosum = %f\n",umass,bosum); */
      tgtFE->bowt = (1.0-umass)/(1.0-bosum);
      if (tgtFE->bowt < MIN_BOWT) tgtFE->bowt = MIN_BOWT;
   }

   for (fe=tgtFE->fea,i=0; i<tgtFE->nfe; i++,fe++) {
      context[lev+1] = fe;
      CalcBackOff(lm,context,lev+1);
   }
}

/* MergeFE: recursive LM tree merge */
static int MergeFE(MergeInfo *mi, FLEntry **context, FLEntry **srcFE, int lev) 
{
   /* lev     - indicates current level of recursion (0-unigram, 1-bigram, etc) */
   /* context - ngram context[0..lev] of FLEntry                                */
   /* srcFE   - array[0..nLModel] of FLEntry corresponding to context[lev]      */
   /*           entries can be NULL indicating that nGram doesn't exist         */
   /* mi      - MergeInfo structure                                             */

   LMInfo *li;
   int ndx;
   Boolean inLM;
   BackOffLM *lm;
   UInt feId[LM_NSIZE];
   int i,j,l,nse,nfe,*la,total;
   int nSize,nLModel,**l2nId,**n2lId,*mId;
   FLEntry *srcFE1[MAX_LMODEL];
   SMEntry *sea,*se,*tse,*se_buff;
   FLEntry *fea,*fe,*sfe,*fe_buff,*tgtFE;

   lm = mi->lm;
   if (lm->probType!=LMP_FLOAT)
      HError(15690,"MergeFE: Incompatible probability type");

   nSize = lm->nSize;
   l2nId = mi->l2nId;
   n2lId = mi->n2lId;
   mId = mi->mId;
   nLModel = mi->nLModel;

   tgtFE = context[lev];
   tgtFE->bowt = 0.0;
   tgtFE->nfe = 0;
   tgtFE->fea = NULL;
   
   /*------------------ Process SMEntry first ------------------*/
   nse = 0;
   se_buff = lm->se_buff;   
   for (li=mi->lmInfo,l=0;l<nLModel;l++,li++) {      /* process each model in turn */
      if ((sfe = srcFE[l])==NULL)
	 continue;
      /* count unique SE's across all models */
      la = n2lId[l];                                 /* cache index lookup row */
      for (se=sfe->sea,j=0; j<sfe->nse; j++,se++) {   
	 if ((ndx = la[se->ndx])==0)                 /* obtain actual target LM index */
	   continue;
	 if (mId[ndx] < 0 ) {
	    mId[ndx] = nse;                          /* obtain translated index */
	    se_buff[nse++].ndx = ndx;                /* !! indexed from 0 */
	 }
      }
   }
   for (se=se_buff,j=0; j<nse; j++,se++) se->prob = 0.0;
   /* create SE storage */
   sea = (SMEntry *) New(lm->heap,nse*sizeof(SMEntry));   
   /* process each model */
   for (li=mi->lmInfo, i=0; i<nLModel; i++, li++) {
      /* set up lm-specific lookup indeces */
      la = l2nId[i];
      for (se=se_buff,tse=sea,j=0; j<nse; j++,se++,tse++) {
	 tse->ndx = la[se->ndx];
	 tse->prob = 0.0;
      }
      /* setup context array */
      for (inLM=TRUE, j=0; j<lev; j++) {
	 if ((feId[j] = la[context[j+1]->ndx])==0)
	    inLM = FALSE;
      }
      /* skip if any word in n-gram not in model */
      if (!inLM)
	 continue;

      /* update probabilities */
      GetNGramProbs(li->lm,feId,lev+1,sea,nse);
      for (se=se_buff,tse=sea,j=0; j<nse; j++,se++,tse++) {
	 if (tse->ndx > 0)
#ifdef INTERPOLATE_MAX
	    if ((x = tse->prob) > se->prob) se->prob = x;
#else
	    se->prob += li->weight * tse->prob;
#endif
      }
   }	
   memcpy(sea,se_buff,nse*sizeof(SMEntry));
   qsort(sea,nse,sizeof(SMEntry),CmpSE);  
   tgtFE->nse = nse;
   tgtFE->sea = sea;

   /* reset buffers for next operation */
   for (se=se_buff,i=0; i<nse; i++,se++) {
      mId[se->ndx] = -1;
      se->ndx = 0;
      se->prob = 0.0;
   }

   if (lev==(nSize-1))
      return tgtFE->nse;

   /*------------------ Process FLEntry second ------------------*/
   nfe = 0;
   fe_buff = lm->fe_buff;
   for (li=mi->lmInfo,l=0;l<nLModel;l++,li++) {      /* process each model in turn */
      if ((sfe = srcFE[l])==NULL)
	 continue;
      
      /* merge FLEntry */
      la = n2lId[l];
      for (fe=sfe->fea,j=0; j<sfe->nfe; j++,fe++) {   
	 if (fe->nse==0)                             /* remove useless FEs */ 
	    continue;
	 if ((ndx = la[fe->ndx])==0)
	    continue;
	 if (mId[ndx] < 0 ) {
	    mId[ndx] = nfe;                          /* obtain translated index */
	    fe_buff[nfe++].ndx = ndx;                /* !! indexed from 1 */
	 }
      }
   }
   fea = (FLEntry *) New(lm->heap,nfe*sizeof(FLEntry));  /* move FE to permanent storage */
   memcpy(fea,fe_buff,nfe*sizeof(FLEntry));
   qsort(fea,nfe,sizeof(FLEntry),CmpFE);  
   tgtFE->nfe = nfe;
   tgtFE->fea = fea;

   /* reset buffers for next operation */
   for (fe=fe_buff,i=0; i<nfe; i++,fe++) {
      mId[fe->ndx] = -1;
      fe->ndx = 0;
   }

   /*------------------ Descend down the tree  --------------------*/
   total = 0;
   for (fe=tgtFE->fea,i=0; i<tgtFE->nfe; i++,fe++) {
      context[lev+1] = fe;
      for (l=0; l<nLModel; l++) {
	 if ((sfe = srcFE[l])!=NULL)
	    srcFE1[l] = FindFE(sfe->fea, 0, sfe->nfe, l2nId[l][fe->ndx]);
	 else 
	    srcFE1[l] = NULL;
      }
      nse = MergeFE(mi,context,srcFE1,lev+1);
      {
	 int ii;
	 mi->progress += nse;
	 if (mi->progress >= DOT_CHUNK) {
	    for (ii=0; ii<mi->progress/DOT_CHUNK; ii++) printf(".");
	    fflush(stdout);
	    mi->progress = mi->progress % DOT_CHUNK;
	 }
      }
      total += nse;
   }
   lm->gInfo[lev+2].nEntry += total;
   
   return tgtFE->nse;
}

/* MergeModels: interpolate language models */
BackOffLM *MergeModels(MemHeap *heap, LMInfo *lmInfo, int nLModel, 
		       int nSize, WordMap *wList)
{
   int i;
   LMInfo *li;
   MergeInfo *mi;
   BackOffLM *tgtLM;
   FLEntry *srcFE[MAX_LMODEL];  /* hold corresponding FLEntry */
   FLEntry *context[LM_NSIZE];  /* cache context when descending down the tree */

   mi = (MergeInfo *) New(heap,sizeof(MergeInfo));
   mi->lmInfo = lmInfo;
   mi->nLModel = nLModel;
   mi->wList = wList;
   mi->lm = tgtLM = InitTargetModel(heap,nSize,wList);

   InitialiseMerge(heap,mi);
   context[0] = &(tgtLM->root);
   for (li=lmInfo,i=0; i<nLModel; i++,li++)
      srcFE[i] = &(li->lm->root);
   tgtLM->gInfo[1].nEntry = MergeFE(mi,context,srcFE,0);
   CalcBackOff(tgtLM,context,0);

#ifdef HTK_CRYPT
   tgtLM->encrypt=FALSE;
   for (li=lmInfo,i=0; i<nLModel; i++,li++)
      tgtLM->encrypt = (tgtLM->encrypt || li->lm->encrypt);
#endif
   return tgtLM;
}   

/* ---------------------- Normalisation  -------------------- */

static void NormaliseFE(FLEntry *tgtFE)
{
   int i;
   double psum;
   FLEntry *fe;
   SMEntry *se;

   psum = 0.0;
   for (se=tgtFE->sea,i=0; i<tgtFE->nse; i++,se++)
      psum += se->prob;
   for (se=tgtFE->sea,i=0; i<tgtFE->nse; i++,se++)
      se->prob = se->prob/psum;
   for (fe=tgtFE->fea,i=0; i<tgtFE->nfe; i++,fe++)
      NormaliseFE(fe);
}

/* EXPORT->NormaliseLM: normalise probs and calculate back-off weights */
void NormaliseLM(BackOffLM *lm) 
{
   FLEntry *context[LM_NSIZE];

   /* NormaliseFE(&lm->root); */
   context[0] = &lm->root;
   CalcBackOff(lm,context,0);
}

/* ---------------------- End of LPMerge.c  -------------------- */
