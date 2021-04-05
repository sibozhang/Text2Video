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
/*      Entropic Cambridge Research Laboratory                 */
/*      (now part of Microsoft)                                */
/*                                                             */
/* ----------------------------------------------------------- */
/*         Copyright: Microsoft Corporation                    */
/*          1995-2000 Redmond, Washington USA                  */
/*                    http://www.microsoft.com                 */
/*                                                             */
/*          2001-2002 Cambridge University                     */
/*                    Engineering Department                   */
/*                                                             */
/*   Use of this software is governed by a License Agreement   */
/*    ** See the file License for the Conditions of Use  **    */
/*    **     This banner notice must not be removed      **    */
/*                                                             */
/* ----------------------------------------------------------- */
/*            File: HMap.c  - MAP Model Updates               */
/* ----------------------------------------------------------- */

/*
  Calculates the MAP estimate of the new model parameters ASSUMING
  that the stats associated with the updates have been stored
  (ie the forward-backward allignments have been performed).
*/

char *hmap_version = "!HVER!HMap: 3.4.1 [CUED 12/03/09]";
char *hmap_vc_id = "$Id: HMap.c,v 1.1.1.1 2006/10/11 09:54:57 jal58 Exp $";

#include <stdio.h>      /* Standard C Libraries */
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <ctype.h>

#include "HShell.h"     /* HMM ToolKit Modules */
#include "HMem.h"
#include "HMath.h"
#include "HSigP.h"
#include "HWave.h"
#include "HAudio.h"
#include "HParm.h"
#include "HLabel.h"
#include "HModel.h"
#include "HTrain.h"
#include "HUtil.h"
#include "HAdapt.h"
#include "HFB.h"
#include "HMap.h"

#define T_TOP   0001    /* Top level tracing */
#define T_UPD   0002    /* Model updates */

static int trace     = 0;        /* Trace level */
static float minVar  = 0.0;      /* minimum variance (diagonal only) */
static float mixWeightFloor=0.0; /* Floor for mixture weights */
static Vector vFloor[SMAX]; /* variance floor - default is all zero */
static int minEgs    = 0;        /* min examples to train a model */
static float minObs  = 0;        /* min observations  to train a model */
static int maxM;
static int S;
static float mapTau     = 20.0;                /* Guides the MAP process */

static ConfParam *cParm[MAXGLOBS];      /* config parameters */
static int nParm = 0;

/* EXPORT->InitMap: initialise configuration parameters */
void InitMap(void)
{
   int i;
   double f;

   Register(hmap_version,hmap_vc_id);
   nParm = GetConfig("HMAP", TRUE, cParm, MAXGLOBS);
   if (nParm>0){
     if (GetConfInt(cParm,nParm,"TRACE",&i)) trace = i;
     if (GetConfInt(cParm,nParm,"MINEGS",&i)) minEgs = i;
     if (GetConfFlt(cParm,nParm,"MINOBS",&f)) minObs = f;
     if (GetConfFlt(cParm,nParm,"MINVAR",&f)) minVar = f;
     if (GetConfFlt(cParm,nParm,"MAPTAU",&f)) mapTau = f;
     if (GetConfFlt(cParm,nParm,"MIXWEIGHTFLOOR",&f)) mixWeightFloor = MINMIX*f;
   }
}

/* --------------------------- Model Update --------------------- */

static int nFloorVar = 0;     /* # of floored variance comps */
static int nFloorVarMix = 0;  /* # of mix comps with floored vars */

/* FloorMixes: apply floor to given mix set */
static void FloorMixes(MixtureElem *mixes, int M, float floor)
{
   float sum,fsum,scale;
   MixtureElem *me;
   int m;
   
   sum = fsum = 0.0;
   for (m=1,me=mixes; m<=M; m++,me++) {
      if (me->weight>floor)
         sum += me->weight;
      else {
         fsum += floor; me->weight = floor;
      }
   }
   if (fsum>1.0) HError(2328,"FloorMixes: Floor sum too large (%f)",fsum);
   if (fsum == 0.0) return;
   if (sum == 0.0) HError(2328,"FloorMixes: No mixture weights above floor");
   scale = (1.0-fsum)/sum;
   for (m=1,me=mixes; m<=M; m++,me++)
      if (me->weight>floor) me->weight *= scale;
}

/* FloorTMMixes: apply floor to given tied mix set */
static void FloorTMMixes(Vector mixes, int M, float floor)
{
   float sum,fsum,scale,fltWt;
   int m;
   
   sum = fsum = 0.0;
   for (m=1; m<=M; m++) {
      fltWt = mixes[m];
      if (fltWt>floor)
         sum += fltWt;
      else {
         fsum += floor;
	 mixes[m] = floor;
      }
   }
   if (fsum>1.0) HError(2327,"FloorTMMixes: Floor sum too large");
   if (fsum == 0.0) return;
   if (sum == 0.0) HError(2328,"FloorTMMixes: No mixture weights above floor");
   scale = (1.0-fsum)/sum;
   for (m=1; m<=M; m++){
      fltWt = mixes[m];
      if (fltWt>floor)
	mixes[m] = fltWt*scale;
   }
}

/* FloorDProbs: apply floor to given discrete prob set */
static void FloorDProbs(ShortVec mixes, int M, float floor)
{
   float sum,fsum,scale,fltWt;
   int m;
   
   sum = fsum = 0.0;
   for (m=1; m<=M; m++) {
      fltWt = Short2DProb(mixes[m]);
      if (fltWt>floor)
         sum += fltWt;
      else {
         fsum += floor;
	 mixes[m] = DProb2Short(floor);
      }
   }
   if (fsum>1.0) HError(2327,"FloorDProbs: Floor sum too large (%f)",fsum);
   if (fsum == 0.0) return;
   if (sum == 0.0) HError(2328,"FloorDProbs: No probabilities above floor");
   scale = (1.0-fsum)/sum;
   for (m=1; m<=M; m++){
      fltWt = Short2DProb(mixes[m]);
      if (fltWt>floor)
	mixes[m] = DProb2Short(fltWt*scale);
   }
}

static void FloorMixtures(HSetKind hskind, StreamElem *ste, int M, float floor)
{
  switch (hskind){
  case DISCRETEHS:
    FloorDProbs(ste->spdf.dpdf,M,floor);
    break;
  case TIEDHS:
    FloorTMMixes(ste->spdf.tpdf,M,floor);
    break;
  case PLAINHS:
  case SHAREDHS:
    FloorMixes(ste->spdf.cpdf+1,M,floor);
    break;
  }
}


/* UpdateWeights: use acc values to calc new estimate of mix weights */
static void UpdateWeights(HMMSet *hset, int px, HLink hmm)
{
   int i,s,m,M=0,N,vSize;
   float x,occi,denom,tmp;
   WtAcc *wa;
   StateElem *se;
   StreamElem *ste;
   MixtureElem *me;
   
   N = hmm->numStates;
   se = hmm->svec+2; 
   for (i=2; i<N; i++,se++){
      ste = se->info->pdf+1; 
      for (s=1;s<=S; s++,ste++){
	vSize = hset->swidth[s];
	wa = (WtAcc *)ste->hook;
	switch (hset->hsKind){
  	case TIEDHS:
	  M=hset->tmRecs[s].nMix;
	  break;
	case DISCRETEHS:
	case PLAINHS:
	case SHAREDHS:
	  M=ste->nMix;
	  break;
	}
	if (wa != NULL) {
	  occi = wa->occ; 
	  if (occi>0) {
	    me = ste->spdf.cpdf + 1; denom=0;
	    for (m=1; m<=M; m++,me++){
	      tmp = me->weight*vSize*mapTau -1;
	      if (tmp<0) tmp = 0;
	      denom += tmp;
	    }
	    me = ste->spdf.cpdf + 1;
	    for (m=1; m<=M; m++,me++){
	      tmp = me->weight*vSize*mapTau -1;
	      if (tmp<0) tmp = 0;
	      x = (tmp + wa->c[m])/(denom + occi); 
	      if (x>1.0){
		if (x>1.001)
		  HError(2393,"UpdateWeights: Model %d[%s]: mix too big in %d.%d.%d %5.5f",
			 px,HMMPhysName(hset,hmm),i,s,m,x);
		x = 1.0; 
	      }
	      switch (hset->hsKind){
	      case TIEDHS:
		ste->spdf.tpdf[m] = x;
		break;
	      case DISCRETEHS:
		ste->spdf.dpdf[m]=DProb2Short(x);
		break;
	      case PLAINHS:
	      case SHAREDHS:
		me=ste->spdf.cpdf+m;
		me->weight = x;
		break;
	      }
	    }
	    if (mixWeightFloor>0.0){
	      FloorMixtures(hset->hsKind,ste,M,mixWeightFloor);		 
	    }
	    /* Force a normalisation becomes of weird zeroing .... */
	    if ((hset->hsKind == PLAINHS) || (hset->hsKind == SHAREDHS)) {
	      me = ste->spdf.cpdf + 1; x=0;
	      for (m=1; m<=M; m++,me++)
		x += me->weight;
	      if (x>1.001)
		HError(-1,"Updating Weights, sum too large (%f)\n",x);
	      me = ste->spdf.cpdf + 1;
	      for (m=1; m<=M; m++,me++)
		me->weight /= x;	      
	    } 
	  }
	  ste->hook = NULL;
	}
      }
   }
}
      
/* UpdateMeans: use acc values to calc new estimate of means */
static int UpdateMeans(HMMSet *hset, int px, HLink hmm)
{
   int i,s,m,k,M,N,vSize,nmapped;
   float occim;
   MuAcc *ma;
   StateElem *se;
   StreamElem *ste;
   MixtureElem *me;
   Vector mean;
   
   N = hmm->numStates; nmapped=0;
   se = hmm->svec+2; 
   for (i=2; i<N; i++,se++){
      ste = se->info->pdf+1; 
      for (s=1;s<=S;s++,ste++){
         vSize = hset->swidth[s];
         me = ste->spdf.cpdf + 1; M = ste->nMix;
         for (m=1;m<=M;m++,me++)
            if (MixWeight(hset,me->weight) > MINMIX){
               mean = me->mpdf->mean;
               ma = GetHook(mean);
               if (ma != NULL){
                  occim = ma->occ;
                  if (occim > 0.0) {
		    if (occim > minObs) nmapped++;
		    for (k=1; k<=vSize; k++)
		      mean[k] = ( mean[k] * mapTau + (ma->mu[k] + mean[k]*occim) )/( mapTau + occim );
                  } 
                  SetHook(mean,NULL);
               }
            }
      }
   }
   return(nmapped);
}

/* UpdateVars: use acc values to calc new estimate of variances */
static void UpdateVars(HMMSet *hset, int px, HLink hmm)
{
   int i,s,m,k,M,N,vSize;
   float occim,x,muDiffk,dmu;
   Vector minV;
   VaAcc *va;
   MuAcc *ma;
   StateElem *se;
   StreamElem *ste;
   MixtureElem *me;
   Vector mean,var;
   Covariance cov;
   Boolean mixFloored,shared;
   
   N = hmm->numStates;
   se = hmm->svec+2; 
   for (i=2; i<N; i++,se++){
      ste = se->info->pdf+1;
      for (s=1;s<=S;s++,ste++){
         vSize = hset->swidth[s];
         minV = vFloor[s];
         me = ste->spdf.cpdf + 1; M = ste->nMix;
         for (m=1;m<=M;m++,me++)
	   if (MixWeight(hset,me->weight) > MINMIX){
               cov = me->mpdf->cov;
               va = GetHook(cov.var);
               mean = me->mpdf->mean;
               ma = GetHook(mean);
               if (va != NULL){
                  occim = va->occ;
                  mixFloored = FALSE;
                  if (occim > 0.0){
		    shared=(GetUse(cov.var)>1 || ma==NULL || ma->occ<=0.0);
                     if (me->mpdf->ckind==DIAGC) {
		         var = cov.var;
			 for (k=1; k<=vSize; k++){
			   if (shared) muDiffk = 0.0;
			   else {
			     dmu = (ma->mu[k])/(mapTau+occim);
			     muDiffk = 2*dmu*ma->mu[k] - dmu*dmu*occim;
			   }
                           x = (mapTau*var[k]  + va->cov.var[k] - muDiffk) / (mapTau + occim);
			   if (x<minV[k]) {
                             x = minV[k];
                              nFloorVar++;
                              mixFloored = TRUE;
			    }
			   cov.var[k] = x;
			 }
       		     }
                     else { /* FULLC */
		       HError(999,"MAP estimation of full covariance matrices not supported");
		     }
		  }
                  if (mixFloored == TRUE) nFloorVarMix++;
		  SetHook(cov.var,NULL);
               }
            }
      }
   }
}

static int TotMixInSet(HMMSet *hset)
{
   HMMScanState hss;
   HLink hmm;
   int nmix=0;

   NewHMMScan(hset,&hss);
   do {
     hmm = hss.hmm;
     while (GoNextState(&hss,TRUE)) {
       while (GoNextStream(&hss,TRUE)) {
	 if (hss.isCont)                     /* PLAINHS or SHAREDHS */
	   while (GoNextMix(&hss,TRUE)) {
	     if (!IsSeenV(hss.mp->mean)) {
	       nmix++;
	       TouchV(hss.mp->mean);
	     } 
	   }
       }
     }
   } while (GoNextHMM(&hss));
   EndHMMScan(&hss);
   return(nmix);
}

/* -------------------------- MAP code --------------------------------- */

/* EXPORT->MAPUpdateModels: update all models and save them in newDir if set,
   new files have newExt if set */
void MAPUpdateModels(HMMSet *hset, UPDSet uFlags)
{
  HMMScanState hss;
  HLink hmm;
  int px,n,nmapped=0,totM;

  if (hset->logWt == TRUE) HError(999,"HMap: requires linear weights");

  /* Intialise a few global variables */
  SetVFloor( hset, vFloor, minVar);
  maxM = MaxMixInSet(hset);
  totM = TotMixInSet(hset);
  S = hset->swidth[0];

  if (hset->hsKind == TIEDHS){ /* TIEDHS - update mu & var once per HMMSet */
    HError(999,"TIEDHS kind not currently supported in MAP estimation");
  }

  NewHMMScan(hset,&hss);
  px=1;
  do {   
    hmm = hss.hmm;
    n = (int)hmm->hook;
    if (n<minEgs && !(trace&T_UPD))
      HError(-2331,"UpdateModels: %s[%d] copied: only %d egs\n",
	     HMMPhysName(hset,hmm),px,n);
    if (n>=minEgs && n>0) {
      if (uFlags & UPTRANS)
	HError(999,"No support for MAP updating transition probabilities");
      if (maxM>1 && uFlags & UPMIXES)
	UpdateWeights(hset,px,hmm);
      if (hset->hsKind != TIEDHS){
	if (uFlags & UPVARS)
	  UpdateVars(hset,px,hmm);
	if (uFlags & UPMEANS)
	  nmapped += UpdateMeans(hset,px,hmm);
	if (uFlags & (UPMEANS|UPVARS))
	  FixGConsts(hmm);
      }  
    }
    px++;
  } while (GoNextHMM(&hss));
  EndHMMScan(&hss);
  if (trace&T_TOP) {
    printf("Observed components (means) %d of %d: %.2f\n",nmapped,totM,100*(float)nmapped/(float)totM);
    if (nFloorVar > 0)
      printf("Total %d floored variance elements in %d different mixes\n",
	     nFloorVar,nFloorVarMix);
    fflush(stdout);
  }
}
