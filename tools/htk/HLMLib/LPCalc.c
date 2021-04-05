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

char *lpcalc_version = "!HVER!LPCalc:   3.4.1 [CUED 12/03/09]";
char *lpcalc_vc_id = "$Id: LPCalc.c,v 1.1.1.1 2006/10/11 09:54:43 jal58 Exp $";

#include "HShell.h"     /* HMM ToolKit Modules */
#include "HMem.h"
#include "HMath.h"
#include "HWave.h"
#include "HLabel.h"
#include "LWMap.h"      
#include "LGBase.h"     /* LM ToolKit Modules */
#include "LUtil.h"
#include "LModel.h"
#include "LPCalc.h"

#define T_TOP    0001       /* top level tracing */
#define T_FOF    0002       /* FoF table tracing */

/* -------------------------- Trace Flags ------------------------ */

static int trace = 0;

/* ---------------- Configuration Parameters --------------------- */

static ConfParam *cParm[MAXGLOBS];
static int nParm = 0;            /* total num params */

/* ---------------------- Global Variables ----------------------- */

static LabId  sstId;                          /* sentence start marker */
static char   sstStr[256] = DEF_STARTWORD;    /* sentence start marker */
static float  uniFloor = 1.0;                 /* unigram floor */

/* SetConfParms: set conf parms relevant to this tool */
void InitPCalc(void)
{
   int i;
   char s[256];

   Register(lpcalc_version,lpcalc_vc_id);
   nParm = GetConfig("LPCALC", TRUE, cParm, MAXGLOBS);
   if (nParm>0){
#ifndef HTK_TRANSCRIBER
      if (GetConfInt(cParm,nParm, "TRACE",&i))    trace = i;
#endif
      if (GetConfStr(cParm,nParm, "STARTWORD",s)) strcpy(sstStr,s);
   }
   sstId = GetLabId(sstStr,TRUE);
}

/* EXPORT->InitBuildInfo: initialise build parameters */
void InitBuildInfo(BuildInfo *bi)
{
   int i, j;    /* Temporary values */
   char s[256]; /* Temporary string */

   bi->nSize = 0;
   bi->ftab  = NULL;
   bi->saveFmt  = DEF_SAVEFMT;
   bi->ptype    = DEF_LMPTYPE;
   bi->uniFloor = DEF_UNIFLOOR;
   bi->kRange   = DEF_KRANGE;
   bi->dctype   = DEF_DCTYPE;
   for (i=1; i<=LM_NSIZE; i++) 
      bi->cutOff[i] = DEF_CUTOFF;
   bi->wmap = NULL;
   bi->inSet = NULL;
   if (GetConfInt(cParm,nParm, "UNIFLOOR",&i)) bi->uniFloor = i;
   if (GetConfInt(cParm,nParm, "KRANGE",&i))   bi->kRange = i;
   if (GetConfStr(cParm,nParm, "DCTYPE",s)) {
     if (!strcmp(s,"TG")) 
       bi->dctype = DC_KATZ;
     else if (!strcmp(s,"ABS")) 
       bi->dctype = DC_ABSOLUTE;
     else if (!strcmp(s,"LIN")) 
       bi->dctype = DC_LINEAR;
   }

   /* See if any config file settings for n-gram cut-offs */
   for (i=2; i<=LM_NSIZE; i++) {
      sprintf(s,"%dG_CUTOFF",i);
      if (GetConfInt(cParm,nParm,s,&j)) bi->cutOff[i] = j;
   }
}

#define LMNDX(wm,i) wm->me[i].sort+1

/* EXPORT->FilterNGram: read n-grams and map them to LM IDs */
Boolean FilterNGram(NGInputSet *inSet, UInt *gram, float *count, int nSize)
{

   int i;
   UInt gbuf[LM_NSIZE];

   if (!GetNextNGram(inSet,gbuf,count,nSize))
      return FALSE;
#ifdef SANITY   
   for (i=0; i<nSize; i++) {
      if (GetMEIndex(inSet->wm,gbuf[i]) < 0) {
	 HError(15590,"FilterNGram: Read n-gram contains out of map words");
      }
   }
#endif
   for (i=0; i<nSize; i++) {
      gram[i] = LMNDX(inSet->wm,GetMEIndex(inSet->wm,gbuf[i]));
   }
   return TRUE;
}

/* EXPORT->CalcUniProbs: calculate unigram */
static int CalcUniProbs(BackOffLM *lm, FLEntry *tgtFE, Boolean rebuild)
{
   NameId nid;
   double tMass;
   SMEntry *se,*unigram;
   int i, numFloored;

   if (rebuild) {
      memcpy(lm->se_buff,tgtFE->sea,tgtFE->nse*sizeof(SMEntry));
      unigram = tgtFE->sea; tgtFE->sea = lm->se_buff;
   } else {
      unigram = (SMEntry *) New(lm->heap,lm->vocSize*sizeof(SMEntry));
   }
   for (se=unigram, i=0; i<lm->vocSize; i++,se++) {    /* initialise array */
      se->prob = 0.0; se->ndx=i+1;
   }
   for (se=tgtFE->sea,i=0; i<tgtFE->nse; i++,se++) {   /* copy all entries across */
      unigram[se->ndx-1].prob = se->prob;
   }
   if (tgtFE->nse!=lm->vocSize) {
      printf("%d distinct unigrams found in data, %d in word list\n",tgtFE->nse,lm->vocSize);
      fflush(stdout);
   }
   tMass = 0.0;
   numFloored = 0;
   for (se=unigram, i=0; i<lm->vocSize; i++,se++) { 
      if (se->prob < uniFloor) {
	 se->prob = uniFloor;
	 numFloored++;
      }
      tMass += se->prob;
   }
   if (numFloored>0) {
      printf("%d unigrams floored to %.1f\n",numFloored,uniFloor);	 
      fflush(stdout);
   }
   if (lm->probType!=LMP_COUNT) {
      /* clamp sentence start symbol prob */
      if ((nid = GetNameId(lm->htab,sstStr,FALSE))!=NULL) {
	 if ((se = FindSE(unigram,0,lm->vocSize,LM_INDEX(nid)))!=NULL) {
	    tMass = tMass - se->prob; 
	    se->prob = 0.0;
	 }
      }
      for (se=unigram, i=0; i<lm->vocSize; i++, se++) {
	 se->prob = se->prob/tMass;
      }
   }
   tgtFE->sea = unigram;
   tgtFE->nse = lm->vocSize;
   tgtFE->ndx = 0;
   tgtFE->bowt = 0.0;
   if (!rebuild) {          /* initialise root FE if building from scratch */
      tgtFE->fea = NULL;       
      tgtFE->nfe = 0;
   }

   return lm->vocSize;
}   

static double ApplyTG(BackOffInfo *boi, FLEntry *tgtFE, double tMass, int nSize)
{
   int i,k,r;
   SMEntry *se;
   double uMass;
   TuringGoodInfo *tgi;
   
   tgi = &boi->dcInfo.tgInfo;
   
   /* apply TG discounting */
   for (se=tgtFE->sea,i=0; i<tgtFE->nse; i++,se++) {
      if ((r = (int) se->prob) <= tgi->kRange) {
         se->prob = tgi->coef[r] * se->prob;
      }
   }
   
   /* accumulate unseen probability mass */
   uMass = 0.0;
   for (se=tgtFE->sea,i=0; i<tgtFE->nse; i++,se++)
     uMass += se->prob;
   uMass = tMass - uMass; 
   
   if (uMass==0.0) {  /* unable to accumulate unseen count, try alternative */
     k = boi->cutOff+1;
     for (se=tgtFE->sea,i=0; i<tgtFE->nse; i++,se++) {
       uMass += (1.0 - tgi->coef[k]) * se->prob;      
       se->prob *= tgi->coef[k];
       if ((k++)==tgi->kRange) break;
     }
   }
   return uMass;
}

static double ApplyABS(BackOffInfo *boi, FLEntry *tgtFE, double tMass) 
{
  int i;
  SMEntry *se;
  double b,uMass;
  
  /* apply Absolute discounting */
  b = boi->dcInfo.bCoef;
  for (se=tgtFE->sea,i=0; i<tgtFE->nse; i++,se++) {
    se->prob = se->prob - b;
    if (se->prob < 0.0) se->prob = 0.0;
  }
  
  /* accumulate unseen probability mass */
  uMass = 0.0;
  for (se=tgtFE->sea,i=0; i<tgtFE->nse; i++,se++)
    uMass += se->prob;
  uMass = tMass - uMass; 

  return uMass;
}

/* 
   EXPORT->CalcNGramProbs: calculate and write n-gram entries

   lm     - target language model (1..nSize-1)-grams should be in place
   feId   - array[0..nSize-2] of LM IDs representing context
   nSize  - n-gram to calculate
   tgtFE  - target FLEntry
   rebuld - TRUE if converting LMP_COUNT -> LMP_FLOAT 
*/
static int CalcNGramProbs(BackOffLM *lm, UInt *feId, int nSize, FLEntry *tgtFE, Boolean rebuild)
{
   int i, j, r;
   int nse, nItem;
   double uMass=0, tMass, boSum, prob;
   LMProbType ptype;
   BackOffInfo *boi;
   SMEntry *se,*bo_se,*se_perm,*tse;
   FLEntry *fe;

   /* se_perm -> permanent SE storage, tgtFE->sea -> lm->se_buff */

   if (nSize==1) {
      return CalcUniProbs(lm,tgtFE,rebuild);
   }
      
   if ((ptype = lm->probType)==LMP_LOG)
      HError(15590,"CalcNGramProbs: Incompatible prob kind (%d)",ptype);
   if ((boi = lm->gInfo[nSize].boInfo)==NULL)
      HError(15590,"CalcNGramProbs: Back-off info not present for %d grams",nSize);
   if (boi->dcType!=DC_KATZ && boi->dcType!=DC_ABSOLUTE)
      HError(15590,"CalcNGramProbs: Unsupported LM type (%d)",boi->dcType);

   if (rebuild) {             /* rebuilding model - no need to allocate storage */
      se_perm = lm->se_buff;
      memcpy(lm->se_buff,tgtFE->sea,tgtFE->nse*sizeof(SMEntry));
      se_perm = tgtFE->sea; tgtFE->sea = lm->se_buff;  /* swap them round */
      tMass = tgtFE->bowt;
   } else {
      se_perm = NULL;
      tMass = 0.0;
   }

   /* first, accumulate total count and apply cutoff */
   nse = 0;
   for (se=tgtFE->sea,i=0; i<tgtFE->nse; i++,se++) { 
      tMass += se->prob;
      if ((r = (int) se->prob) <= boi->cutOff)
	 se->prob = 0.0;
      if (se->prob > 0.0) nse++;
   }
   if (se_perm==NULL) /* allocate permanent SE storage */
      se_perm = (SMEntry *) New(lm->heap,nse*sizeof(SMEntry));

   /* copy entries with non-zero probabilities to se_perm */
   for (tse=se_perm,se=tgtFE->sea,i=0; i<tgtFE->nse; i++,se++)
      if (se->prob>0.0) *tse++=*se;
   /* then copy back to tgtFE->sea */
   memcpy(tgtFE->sea,se_perm,nse*sizeof(SMEntry));
   tgtFE->nse = nse;
   qsort(tgtFE->sea,tgtFE->nse,sizeof(SMEntry),CmpSE);  

   if (ptype==LMP_COUNT) {  /* building COUNT model */

      /* accumulate unseen probability mass */
      uMass = 0.0;
      for (se=tgtFE->sea,i=0; i<tgtFE->nse; i++,se++)
	 uMass += se->prob;
      uMass = tMass - uMass; 
      boSum = 1.0;
      
   } else {    /* building probabilistic model */  

      switch(boi->dcType) {
      case DC_KATZ:
	 uMass = ApplyTG(boi,tgtFE,tMass,nSize);
	 break;
      case DC_ABSOLUTE:
	 uMass = ApplyABS(boi,tgtFE,tMass);
	 break;
      default :
         HError(15590,"CalcNGramProbs: Unsupported LM type (%d)",boi->dcType);
	 break;
      }
      /* calculate sum of (n-1)-gram probs for unseen entries */
      boSum = 0.0;
      GetNGramProbs(lm, feId+1, nSize-1, se_perm, nse);
      if (boi->wdThresh>0) {
         for (bo_se=se_perm,se=tgtFE->sea,fe=tgtFE->fea,i=j=0; i<tgtFE->nse; i++,bo_se++,se++){
            prob=se->prob/tMass;
            if (fabs(se->prob*(log(prob)-log(bo_se->prob)))<boi->wdThresh &&
                (fe==NULL || j>=tgtFE->nfe || fe->ndx!=se->ndx)) {
               uMass+=se->prob,se->prob=0.0; /* oh my goodness - who wrote that?! */
               if (j<tgtFE->nfe && fe->ndx==se->ndx) /* PRUNE FE AS WELL */
                  fe->nse=0,fe->nfe=0;
            }
            if (j<tgtFE->nfe && fe->ndx<=se->ndx) fe++,j++;
         }
      }

      for (bo_se=se_perm,se=tgtFE->sea,i=0; i<tgtFE->nse; i++,bo_se++,se++)
         if (se->prob > 0.0) boSum += bo_se->prob;
      boSum = 1.0 - boSum;
   }
   
   nItem = 0;
   if (uMass!=tMass) {  /* some real n-grams still left after discounting */
      if (ptype == LMP_COUNT) {
	 tMass = 1.0;
      }
      tse = se_perm;
      for (se=tgtFE->sea,i=0; i<tgtFE->nse; i++,se++) {
	 if (se->prob>0.0) {
	    tse->prob = se->prob / tMass;
	    tse->ndx  = se->ndx;
	    tse++; nItem++;
	 }
      }
      tgtFE->sea = se_perm;
      tgtFE->nse = nItem;
      tgtFE->ndx = feId[nSize-2];
      tgtFE->bowt = (boSum <= 0.0) ? MIN_BOWT : (uMass / (tMass * boSum));
      if (!rebuild) {
	 tgtFE->fea = NULL;
	 tgtFE->nfe = 0;
      }
   }
   else {
      tgtFE->sea = se_perm;
      tgtFE->nse = nItem;
      tgtFE->nfe = 0;
   }
   return nItem;
}

#define GRAM2TEXT() { \
   for (s = sbuf, *sbuf='\0', j=0; j<nSize; j++) { \
      sprintf(s," %s",wmap->id[gramKey[j]]->name); s+=strlen(s); \
   } \
}

/* EXPORT CalculateNGram: calculate nSize-grams from gram files in inSet */
static int CalculateNGram(BackOffLM *lm, NGInputSet *inSet, int nSize)
{
   float count;
   WordMap *wmap;
   char *s, sbuf[256];
   int i, j, nse, nfe, nItem;
   SMEntry *se, *se_buff;
   FLEntry *fe, *fe_buff, *feptr;
   UInt *ge, gram[LM_NSIZE];
   UInt gramKey[LM_NSIZE];
   Boolean newCX1, newCX2;

   if ((se = se_buff = lm->se_buff)==NULL)
      HError(15590,"CalculateNGram: se_buff not initialised");
   if ((fe = fe_buff = lm->fe_buff)==NULL)
      HError(15590,"CalculateNGram: fe_buff not initialised");
   if ((wmap = inSet->wm)==NULL)
      HError(15590,"CalculateNGram: Word map not set");
   if (nSize < 1 || nSize > inSet->N)
      HError(15590,"CalculateNGram: Invalid nSize (%d)",nSize);
   nse = 0; nfe = 0; nItem = 0;

   OpenInputSet(inSet);
   if (!FilterNGram(inSet,gram,&count,nSize))
      HError(15513,"CalculateNGram: Unable to read first n-gram");
   memcpy(gramKey,gram,nSize*sizeof(UInt));
   do {
#ifdef SANITY
      for (i=0; i<nSize; i++)
	 if (gram[i] < 1 || gram[i] > lm->vocSize)
	    HError(15590,"CalculateNGram: LM index out of range (%d)",gram[i]);
#endif
      for (newCX1=FALSE, ge=gram, i=0; i<nSize-2; i++, ge++)
	 if (gramKey[i]!=*ge) { 
	    newCX1 = TRUE; break; 
	 }
      newCX2 = (nSize==1) ? newCX1 : newCX1 || (gramKey[nSize-2]!=gram[nSize-2]);
      if (newCX2) {
	 fe->nse = nse;
	 fe->sea = se_buff;
	 if ((nse = CalcNGramProbs(lm,gramKey,nSize,fe,FALSE)) > 0) {
	    fe->fea = NULL; fe->nfe = 0;
	    nItem += nse; fe++; nfe++;
	 }
	 if (newCX1) {
	    if (nfe>0) {
	       for (feptr=&lm->root, i=0; i<nSize-2; i++) {
                  FLEntry *feptr2;
		  if ((feptr2 = FindFE(feptr->fea,0,feptr->nfe,gramKey[i]))==NULL) {
		     GRAM2TEXT();
		     HError(15520,"CalculateNGram: Unable to find FLEntry to attach (%s)",sbuf);
		  }
                  feptr=feptr2;
	       }
	       if (feptr->nfe > 0 || feptr->fea!=NULL) {
		  GRAM2TEXT();
		  HError(15525,"CalculateNGram: Attempt to overwrite entries when attaching (%s)",sbuf);
	       }
	       feptr->fea = fe_buff; feptr->nfe = nfe; StoreFEA(feptr,lm->heap);
	    }
	    fe = fe_buff; nfe = 0;
	    for (ge=gram,i=0; i<nSize-2; i++,ge++) gramKey[i] = *ge;
	 }
	 gramKey[nSize-2] = gram[nSize-2];
	 se = se_buff; nse=0; 
      }
      se->ndx = gram[nSize-1]; se->prob = count;
      se++; nse++;
#ifdef SANITY      
      if (nse>lm->vocSize)
	 HError(15590,"CalculateNGram: SE buffer limit reached (%d)",nse);
#endif
   } while(FilterNGram(inSet,gram,&count,nSize));

   /* finish off the remaining n-grams accumulated */
   if (nSize > 1) {  /* (n>1)-grams */
      fe->nse = nse;
      fe->sea = se_buff;
      if ((nse = CalcNGramProbs(lm,gramKey,nSize,fe,FALSE)) > 0) {
	 fe->fea = NULL; fe->nfe = 0;
	 nItem += nse; fe++; nfe++;
      }
      if (nfe > 0) {
	 for (feptr=&lm->root, i=0; i<nSize-2; i++) {
	    if ((feptr = FindFE(feptr->fea,0,feptr->nfe,gramKey[i]))==NULL) {
	       GRAM2TEXT();
	       HError(15520,"CalculateNGram: Unable to find FLEntry for (%s)",sbuf);
	    }
	 }
	 if (feptr->nfe > 0 || feptr->fea!=NULL) {
	    GRAM2TEXT();
	    HError(15525,"CalculateNGram: Attempt to ovewrite entries when attaching (%s)",sbuf);
	 }
	 feptr->fea = fe_buff; feptr->nfe = nfe; StoreFEA(feptr,lm->heap);
      }
   } else {  /* unigrams */
      lm->root.nse = nse;
      lm->root.sea = se_buff;
      nItem = CalcUniProbs(lm,&lm->root,FALSE);
   }
   CloseInputSet(inSet);

   return nItem;
}

#define DEF_ABS_COEF 0.5

static double CalcABSCoef(int nSize, FoFTab *ftab) 
{
  UInt **fof;
  double coef;

  fof = ftab->fof;
  if (fof[1][nSize]==0 || fof[2][nSize]==0) 
     coef = DEF_ABS_COEF;
  else
     coef = (double) fof[1][nSize] / (double) (fof[1][nSize] + 2.0*fof[2][nSize]);
  if (trace&T_TOP)
    printf("Absolute discounting term %e\n",coef);
  return coef;
}

#define DEF_TG_COEF 0.99

static void CalcTGCoefs(MemHeap *heap, BackOffInfo *boi, int nSize, FoFTab *ftab) 
{
   int r,K;
   UInt **fof;
   double kTerm,gTerm;
   TuringGoodInfo *tgi;
   Boolean ok,allPositive;
   
   fof = ftab->fof; 
   tgi = &boi->dcInfo.tgInfo; 
   K = tgi->kRange;
   tgi->coef = (float *) New(heap,(K+1)*sizeof(float));
   for (r=0; r<=K; r++) tgi->coef[r] = 0.0;
   
   /* check for singularities */
   for (ok = (fof[1][nSize]>0),r=1; ok && r<K; r++) 
      ok = ok && (fof[r][nSize]>0);
   if (ok) {
      do {
         if (K <= 1) {
               HError(-15560, "CalcTGCoefs: Invalid K=%d - setting default K and coefficients", K);
               K = tgi->kRange;
               for (r=1; r <=K; r++)
                  tgi->coef[r] = (r <= boi->cutOff) ? 0.0 : DEF_TG_COEF;
               return;
         }
         kTerm = (double) ((K+1) * fof[K+1][nSize]) / (double) fof[1][nSize];
         /*         if (kTerm>DEF_TG_COEF) {
            kTerm = DEF_TG_COEF;
            if (trace&T_TOP)
               printf("CalcTGCoefs: clamping kTerm to %f\n", DEF_TG_COEF);
         }*/
         /* Further check that kTerm > (r+1).c[r+1]/c[r] for 1<=r<k */
         allPositive = TRUE;
         for (r=(boi->cutOff?boi->cutOff:1); r<K; r++) {
            gTerm = (double) ((r+1) * fof[r+1][nSize])/(double) (r*fof[r][nSize]);
            if (((kTerm<=1.0) && (kTerm>=gTerm)) || ((kTerm>1.0) && (kTerm<gTerm)))
               allPositive = FALSE;
            printf("g[%d]=%f\n", r, gTerm);
         }
         if (allPositive)
            break;
         K--;
         if (trace&T_TOP)
            printf("CalcTGCoefs: lowering K to %d\n", K);
      } while(TRUE);
      for (r=1; r<=K; r++) {
         gTerm = (double) ((r+1) * fof[r+1][nSize])/(double) (r*fof[r][nSize]);
         if (r <= boi->cutOff) {
            tgi->coef[r] = 0.0;
         }
         else {
            tgi->coef[r] = ((gTerm - kTerm) / (1.0 - kTerm));
            if (tgi->coef[r] < 1E-03) { 
               HError(-15560, "CalcTGCoefs: Invalid coefficient detected in Turing-Good discounting (%f) - clamped to 1E-03 [gTerm=%f, kTerm=%f, r=%d, cutoff=%d]", tgi->coef[r], gTerm, kTerm, r, boi->cutOff);
               tgi->coef[r] = 1E-03;
            }
         }
      }
      if (trace&T_TOP)
	 printf("%d-gram coefs:\n",nSize);
   } else {
      for (r=1; r <=K; r++)
	 tgi->coef[r] = (r <= boi->cutOff) ? 0.0 : DEF_TG_COEF;
   }
   for (r=1; r<=K; r++) {
      if (trace&T_TOP) printf("coef[%d]=%e",r,tgi->coef[r]);
      if (tgi->coef[r]>1.0) {
	 tgi->coef[r] = DEF_TG_COEF; 
	 if (trace&T_TOP) printf(", clamped to %.4f",tgi->coef[r]);
      }
      if (trace&T_TOP) printf("\n");
   }
   tgi->kRange = K;
}


/* EXPORT->CalcDiscountCoefs: calculate discount coefs from fof table */
static void CalcDiscountCoefs(BackOffLM *lm, FoFTab *ftab)
{
   int ns;
   BackOffInfo *boi;

   for (ns=2; ns<=lm->nSize; ns++) {
      if ((boi = lm->gInfo[ns].boInfo)==NULL)
	 HError(15590,"CalcDiscountCoefs: Back-off info not available for %d-gram",ns);
      switch (boi->dcType) {
      case DC_KATZ:
	CalcTGCoefs(lm->heap,boi,ns,ftab);
	break;
      case DC_ABSOLUTE:
	boi->dcInfo.bCoef = CalcABSCoef(ns,ftab);
	break;
      default:
	HError(15590,"CalcDiscountCoefs: Unsupported LM type (%d)",boi->dcType);
      }
   }
}

/* CheckCutoffs: check n-gram cutoffs and discounting range */
static void CheckCutoffs(BackOffLM *lm)
{
   BackOffInfo *boi;
   int ns,kRange,lastCutOff;
   
   lastCutOff=0;
   for (ns=2; ns<=lm->nSize; ns++) {
      if ((boi = lm->gInfo[ns].boInfo)==NULL)
	 HError(15590,"CheckCutoffs: Back-off info not available for %d-gram",ns);
      if (boi->cutOff < lastCutOff) {
	 HError(15540,"CheckCutoffs: %d-gram cutoff = %d, %d-gram cutoff = %d",
		ns,boi->cutOff,ns-1,lastCutOff);
      }
      if (boi->dcType!=DC_KATZ && boi->dcType!=DC_ABSOLUTE)
	 HError(15590,"CheckCutoffs: Unsupported LM type (%d)",boi->dcType);
      if (boi->dcType==DC_KATZ) {
	 kRange = boi->dcInfo.tgInfo.kRange;
	 if (boi->cutOff > kRange)
	    HError(-15540,"CheckCutoffs: %d-gram cutoff out of range (%d)",ns,boi->cutOff);
      }
      lastCutOff = boi->cutOff;
   }
}

/* InitTargetModel: initialise target LM structure */
static BackOffLM *InitTargetModel(MemHeap *heap, BuildInfo *bi)
{
   int i,ndx,N;
   NameId nId;
   BackOffLM *lm;
   BackOffInfo *boi;

   if (bi->nSize<1)
      HError(15590,"GenerateLM: Invalid n-gram size (%d)",bi->nSize);
   if (bi->ptype!=LMP_FLOAT && bi->ptype!=LMP_COUNT)
      HError(15590,"GenerateLM: Invalid probability kind (%d)",bi->ptype);

   lm = (BackOffLM *) New(heap,sizeof(BackOffLM));
   lm->heap = heap;
   lm->gScale = 1.0;
   lm->nSize = bi->nSize;
   lm->probType = bi->ptype;
   lm->vocSize = N = bi->wmap->used;
   lm->binMap = (NameId *) New(lm->heap,N*sizeof(NameId)); 
   lm->binMap--;
   lm->htab = CreateHashTable(11731,"Back-off LM hash table");
   for (i=0; i<N; i++) {
      nId = GetNameId(lm->htab,bi->wmap->id[i]->name,TRUE);
      ndx = LMNDX(bi->wmap,i);
      LM_INDEX(nId) = ndx; lm->binMap[ndx] = nId;
   }
   lm->gInfo[1].boInfo = NULL;
   for (i=2; i<=lm->nSize; i++) {    /* initialise discount info, etc */
      boi = (BackOffInfo *) New(lm->heap,sizeof(BackOffInfo));
      boi->cutOff = bi->cutOff[i];
      boi->wdThresh = bi->wdThresh[i];
      boi->dcType = bi->dctype;
      switch (boi->dcType) {
	 case DC_KATZ:
            boi->dcInfo.tgInfo.kRange = bi->kRange;
	    boi->dcInfo.tgInfo.coef   = NULL;
	    break;
	 case DC_ABSOLUTE:
	    boi->dcInfo.bCoef = 0.0;
            break;
         default:
            break;
      }
      lm->gInfo[i].boInfo = boi;
   }
   CheckCutoffs(lm);
#ifdef HTK_CRYPT   
   lm->encrypt = FALSE;
#endif
   lm->fe_buff = (FLEntry *) New(lm->heap,N*sizeof(FLEntry));
   lm->se_buff = (SMEntry *) New(lm->heap,N*sizeof(SMEntry));

   lm->classLM = FALSE;

   return lm;
}

/* CloneInputSet: return a copy of ngs */
static NGInputSet *CloneInputSet(NGInputSet *src,NGInputSet *tgt)
{
   GFLink p;

   CreateInputSet(&gstack,src->wm,tgt);
   for(p=src->head.chain; p!=NULL; p=p->chain)
      AddInputGFile(tgt,p->fn,p->weight);
   return tgt;
}

/* ComputeFoFTab: scan files and produce FoF table */
void ComputeFoFTab(FoFTab *ftab, int nSize, NGInputSet *inSet)
{
   int j,k,oci;
   NGram p, q;
   long **tocMat;
   int pos,fofSize;
   float count,occ[LM_NSIZE];
   UInt gram[LM_NSIZE], fkey[LM_NSIZE];

   if (ftab->N < nSize) 
      HError(15595,"ComputeFoFTab: n-gram size mismatch");
   for (k=0; k<LM_NSIZE; k++) occ[k] = 0.0f;
   
   /* initialise total count matrix */
   fofSize = ftab->size;
   tocMat = (long **) New(&gstack,fofSize*sizeof(long *));
   for (j=0; j<fofSize; j++) {
     tocMat[j] = (long *) New(&gstack,LM_NSIZE*sizeof(long));
     tocMat[j]--; /* indexed from 1 */
     for (k=0; k<LM_NSIZE; k++) tocMat[j][k+1] = 0;
   }

   OpenInputSet(inSet);
   if (!FilterNGram(inSet,gram,&count,nSize))
      HError(15513,"ComputeFoFTab: Unable to read first n-gram");
   memcpy(fkey,gram,nSize*sizeof(UInt));
   do {
      for (p=gram,q=fkey,pos=0; pos<nSize; pos++,p++,q++) {
	 if (*p == *q) {
	    occ[pos] += count; 
	 } else {
	    for (k=pos; k<nSize; k++) {
	       oci = (int) occ[k];
	       if ((oci > 0) && (oci <= fofSize)) {
		  ftab->fof[oci][k+1]++;
	       }
	       for (j=0; j<((fofSize<oci)?fofSize:oci); j++)
		  tocMat[j][k+1]++;
	    }
	    for (k=pos; k<nSize; k++) {
	       fkey[k] = gram[k]; 
	       occ[k] = count;
	    }
	    break;
	 }
      }
   }  while(FilterNGram(inSet,gram,&count,nSize));
   for (k=0; k<nSize; k++) {
     oci = (int) occ[k];
      if ((oci > 0) && (oci <= fofSize)) {
	 ftab->fof[oci][k+1]++;
      }
      for (j=0; j<((fofSize<oci)?fofSize:oci); j++)
	 tocMat[j][k+1]++;
   }
   CloseInputSet(inSet);

   if (trace&T_FOF) {
      /* print total counts if requested */
      printf("\ncutoff ");
      for (j=0; j<nSize; j++) printf("\t%d-g",j+1);
      printf("\n");
      for (k=0; k<fofSize; k++){
	 printf("%d", k);
	 for (j=0; j<nSize; j++) printf("\t%ld ", tocMat[k][j+1]);
	 printf("\n");
      }
   }
   Dispose(&gstack,tocMat);
}

/* EXPORT->UpdateModel: update an existing model */
BackOffLM *UpdateModel(BackOffLM *lm, BuildInfo *bi)
{
   int i,curSize;
   NGInputSet iset;
   BackOffInfo *boi;

   curSize = lm->nSize;
   if (bi->nSize <= curSize)
      HError(15590,"UpdateModel: Current model is already %d-gram",curSize);
   if (bi->ptype != lm->probType)
      HError(15590,"UpdateModel: Incompatible probability kind specified (%d)",bi->ptype);
   for (i = curSize+1; i<=bi->nSize; i++) {
      boi = (BackOffInfo *) New(lm->heap,sizeof(BackOffInfo));
      boi->cutOff = bi->cutOff[i];
      boi->wdThresh = bi->wdThresh[i];
      boi->dcType = bi->dctype;
      switch (boi->dcType) {
	 case DC_KATZ:
	    boi->dcInfo.tgInfo.kRange = bi->kRange;
	    boi->dcInfo.tgInfo.coef   = NULL;
	    break;
	 case DC_ABSOLUTE:
	    boi->dcInfo.bCoef = 0.0;
            break;
         default:
            break;
      }
      lm->gInfo[i].boInfo = boi;
   }
   lm->nSize = bi->nSize;
   if (bi->ftab==NULL) {
      if (trace&T_TOP) {
	 printf("Calculating FoF table\n"); fflush(stdout);
      }
      CloneInputSet(bi->inSet,&iset);
      bi->ftab = CreateFoFTab(&gstack,128,iset.N);
      ComputeFoFTab(bi->ftab,iset.N,&iset);
   }
   if (trace&T_TOP) {
      printf("Calculating discount coefficients\n"); fflush(stdout);
   }
   CheckCutoffs(lm);
   CalcDiscountCoefs(lm,bi->ftab);

   if (lm->se_buff==NULL)
      lm->se_buff = (SMEntry *) New(lm->heap,lm->vocSize*sizeof(SMEntry));
   if (lm->fe_buff==NULL)
      lm->fe_buff = (FLEntry *) New(lm->heap,lm->vocSize*sizeof(FLEntry));
   for (i=curSize+1; i<=bi->nSize; i++) {
      if (trace&T_TOP) {
	 printf("Calculating %d-grams\n",i); fflush(stdout);
      }
      CloneInputSet(bi->inSet,&iset);
      lm->gInfo[i].nEntry = CalculateNGram(lm,&iset,i);
   }
   for (i=1; i<=lm->nSize; i++) {
      lm->gInfo[i].fmt = (i==1) ? LMF_TEXT : bi->saveFmt;
   }
   return lm;
}

/* EXPORT->GenerateModel: generate model fron n-gram data files */
BackOffLM *GenerateModel(MemHeap *heap, BuildInfo *bi)
{
   int i;
   BackOffLM *lm;
   NGInputSet iset;

   lm = InitTargetModel(heap,bi);
   if (bi->nSize > 1) {
      if (bi->ftab==NULL) {
	 if (trace&T_TOP) {
	    printf("Calculating FoF table\n"); fflush(stdout);
	 }
	 CloneInputSet(bi->inSet,&iset);
	 bi->ftab = CreateFoFTab(&gstack,128,iset.N);
	 ComputeFoFTab(bi->ftab,iset.N,&iset);
	 /* WriteFoFTab("foo.fof",bi->ftab,"NULL"); */
      }
      if (trace&T_TOP) {
	 printf("Calculating discount coefficients\n"); fflush(stdout);
      }
      CheckCutoffs(lm);
      CalcDiscountCoefs(lm,bi->ftab);
   }
   uniFloor = bi->uniFloor;           /* !! global */
   for (i=1; i<=bi->nSize; i++) {
      if (trace&T_TOP) {
	 printf("Calculating %d-grams\n",i); fflush(stdout);
      }
      CloneInputSet(bi->inSet,&iset);
      lm->gInfo[i].nEntry = CalculateNGram(lm,&iset,i);
   }
   for (i=1;i<=bi->nSize;i++)
      lm->gInfo[i].fmt = (i==1) ? LMF_TEXT : bi->saveFmt;
   return lm;
}


/* BuildFE: calculate nSize-gram probabilities and backoff weights */
static int RebuildNGrams(BackOffLM *lm,int cxSize,int nSize,FLEntry **context)
{
   int i, nfe, nse, nItem, tFE;
   UInt feId[LM_NSIZE];
   FLEntry *fe, *parent, *tfe;

   nItem = tFE = 0; 
   parent = context[cxSize-1];
   if (cxSize < nSize-1) {
      for (fe=parent->fea,i=0; i<parent->nfe; i++,fe++) {
	 context[cxSize] = fe;
	 nItem += RebuildNGrams(lm,cxSize+1,nSize,context);
      }
   } else {       /* cxSize == nSize-1 */
      for (i=0; i<cxSize; i++)  
	 feId[i] = context[i]->ndx;
      nfe = 0; tfe = lm->fe_buff;
      for (fe=parent->fea,i=0; i<parent->nfe; i++,fe++) {
	 feId[cxSize] = fe->ndx;
	 if ((nse=CalcNGramProbs(lm,feId+1,nSize,fe,TRUE)) > 0) {
	    nItem += nse; *tfe++ = *fe; nfe++;
	 }
      }
      memcpy(parent->fea,lm->fe_buff,nfe*sizeof(FLEntry));
      parent->nfe = nfe; tFE+=nfe;
   }
   lm->gInfo[0].nEntry+=tFE;
   return nItem;
}

/* ConvertToLog: LMP_FLOAT _> LMP_LOG conversion */
static void ConvertToLog(FLEntry *parent)
{
   int i;
   FLEntry *fe;
   SMEntry *se;

   for (se=parent->sea,i=0; i<parent->nse; i++,se++)
      se->prob = LN10*FLT_TO_LOG10(se->prob);
   for (fe=parent->fea,i=0; i<parent->nfe; i++,fe++) {
      fe->bowt = LN10*FLT_TO_LOG10(fe->bowt);
      ConvertToLog(fe);
   }
}

/* EXPORT->RebuildLM: normalise probs and calculate back-off weights */
void RebuildLM(BackOffLM *lm, int *cutOff, float *wdThresh, LMProbType tgtPType) 
{
   int i,k,r,nItem,nNode;
   BackOffInfo *boi;
   TuringGoodInfo *tgi;
   FLEntry *cx[LM_NSIZE];

   if (tgtPType!=LMP_FLOAT && tgtPType!=LMP_LOG && tgtPType !=LMP_COUNT)
      HError(15590,"RebuildLM: Invalid target probability kind (%d)",tgtPType);
      
   if (lm->probType==LMP_COUNT) {
      if (cutOff!=NULL || wdThresh!=NULL) {  /* new cut-offs and coefs */
	 for (i=2; i<=lm->nSize; i++) {
	    if ((boi = lm->gInfo[i].boInfo)==NULL)
	       HError(15590,"RebuildLM: Back-off info not present for %d-grams",i);
	    if (boi->dcType!=DC_KATZ)
	       HError(15590,"RebuildLM: Unsupported LM type (%d)",boi->dcType);
	    if (wdThresh!=NULL) boi->wdThresh = wdThresh[i];
	    if (cutOff==NULL || cutOff[i] < boi->cutOff)
               continue;
	    boi->cutOff = cutOff[i];
	    tgi = &boi->dcInfo.tgInfo;
	    k = (boi->cutOff > tgi->kRange) ? tgi->kRange : boi->cutOff;
	    for (r=1; r<=k; r++) tgi->coef[r] = 0.0;
	 }
	 CheckCutoffs(lm);
      }
      if (lm->se_buff==NULL)
	 lm->se_buff = (SMEntry *) New(lm->heap,lm->vocSize*sizeof(SMEntry));
      if (lm->fe_buff==NULL)
	 lm->fe_buff = (FLEntry *) New(lm->heap,lm->vocSize*sizeof(FLEntry));
      if (tgtPType!=LMP_COUNT)
	 lm->probType = LMP_FLOAT;
      /* convert counts to probs */
      nItem = CalcUniProbs(lm,&lm->root,TRUE);
      if (trace&T_TOP) {
	 printf("  rebuilt %d-grams, %d -> %d\n",1,lm->gInfo[1].nEntry,nItem);
      }
      lm->gInfo[1].nEntry = nItem; 
      nNode=lm->gInfo[0].nEntry; lm->gInfo[0].nEntry=1;
      for (cx[0] = &lm->root,i=2; i<=lm->nSize; i++) {
	 nItem = RebuildNGrams(lm,1,i,cx);
	 if (trace&T_TOP) {
	    printf("  rebuilt %d-grams, %d -> %d\n",i,lm->gInfo[i].nEntry,nItem);
	 }
	 lm->gInfo[i].nEntry = nItem; 
      }
      if (trace&T_TOP) {
	 printf("  rebuilt x-nodes, %d -> %d\n",
		nNode,lm->gInfo[0].nEntry);
      }
   }
   if (tgtPType==LMP_LOG && lm->probType==LMP_FLOAT) {
      ConvertToLog(&lm->root);
      lm->probType=LMP_LOG;
   }
}

/* ---------------------- End of LPCalc.c  ---------------------- */
