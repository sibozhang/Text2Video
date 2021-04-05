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
/* ----------------------------------------------------------- */
/*         Copyright:                                          */
/*                                                             */
/*              2002  Cambridge University                     */
/*                    Engineering Department                   */
/*                                                             */
/*   Use of this software is governed by a License Agreement   */
/*    ** See the file License for the Conditions of Use  **    */
/*    **     This banner notice must not be removed      **    */
/*                                                             */
/* ----------------------------------------------------------- */
/*         File: HFBLat.c   Lattice Forward Backward routines  */
/* ----------------------------------------------------------- */

char *hfblat_version = "!HVER!HFBLat:   3.4.1 [CUED 12/03/09]";
char *hfblat_vc_id = "$Id: HFBLat.c,v 1.1.1.1 2006/10/11 09:54:57 jal58 Exp $";

/*
  Performs forward/backward alignment
*/


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
#include "HDict.h"
#include "HNet.h"
#include "HArc.h"
#include "HFB.h"
#include "HFBLat.h"
#include "HExactMPE.h"

#include <math.h>
#include <stdlib.h>

#if 0
float _exp(float x){
   int _errno=errno;
   float ans = exp(x);
   if(errno!=_errno) HError(1, "Exp op failed... ");
   return ans;
}

#define exp(x) _exp(x)
#endif


#define MAX(a,b) ((a)>(b) ? (a) : (b))
#define MIN(a,b) ((a)<(b) ? (a) : (b))

/* Trace Flags */
#define T_TOP   0001    /* Top level tracing */
#define T_TIM   0002    /* Output timings */

/* Global Settings */

static float minFrwdP = 10.0;            /* mix prune threshold */
static int trace     =  1;            /* Trace level */

static Boolean PhoneMEE = TRUE;      /*IMPORTANT*/           /* If true and the MPE routines are called, do MPE, else MWE (word level) */
static Boolean CalcAsError = FALSE;   /* if TRUE, new way of calc'ing error... */ 
static Boolean PhoneMEEUseContext = FALSE;                  /*Compare phones-in-context.  I doubt you would want this true, anyway it makes little difference.*/
static Boolean ExactCorrectness=FALSE; /*IMPORTANT*/         /* Do 'exact' version of MPE/MWE, not using approximation.  This is slightly better for
                                                                e.g. Wall Street Journal and BN, you may have to tune InsCorrectness (e.g, -0.9), but its
                                                                worse for Switchboard.  See also configs in HFBExactMPE, if this is TRUE. */
static Boolean DoingFourthAcc=FALSE;    /* Indicate currently it is doing MPE with MMI prior */
static int add_index = 999;   /* additional index for discriminative training: 3 for MPE with MMI prior */
static float InsCorrectness = -1;                            /* Correctness of an inserted phone.  Can be tuned, it affects recognition insertion rate.
                                                                E.g. InsCorrectness = -0.85 will increase insertions upon testing, relative to default = -1. */
static Boolean NoSilence = FALSE;                      /* If TRUE, then (in non-exact MPE) the silences are omitted from the reference transcription
                                                          when calculating approximated correctness.  It works better with the default setting (with 
                                                          silence included in reference). */
#ifdef SUPPORT_QUINPHONE  /*defined in .h*/
static Boolean Quinphone = FALSE;            /* Set this TRUE if this is a quinphone model set.  The code for quinphones depends on the naming conventions 
                                                used for quinphones, i.e. silence & sp with single models named sil_0 and sp_0, and phone models with 3 
                                                models per phone, named e.g. ax_2_134 ax_3_141 ax_4_102, so the states are numbered 2,3,4.  sil_{2,3,4}_* would work too.  */

#endif

/* configs: */

static float latProbScale = 1.0; /*IMPORTANT*/  /* Scales the lattice-arc and lm probabilities.  Normally set to e.g. 1/12 or 1/15, the inverse
                                                   of the normal language model scale.  Smaller values, e.g. 1/25, 1/50 can be useful too.  */
static float probScale = 1.0;                 /* Scales the state-output and lm probabilities.  Leave this alone for normal usage. */
static float langProbScale = 1.0;             /* Extra scale on lm probabilities.   Leave this alone for normal usage. */


static float phnInsPen = 0.0;                 /* Insertion penalty for each phone, not subject to lm scaling.  Normally zero, but setting it to e.g.
                                                 -0.5 increases test set insertion errors (reducing deletions) and can be helpful where there
                                                 is very strong (small) probability scaling or the LM is scaled down.   */
                                                

/* Misc variables that can be kept at global level. */

static int StartTime=0;     /* This is a value that we use to help calculating the PreComp's of
			       the MOutP's, to make sure not to use previously cached values. 
			       Doesn't matter if shared between different FBLatInfo's.  */
float hfwdbkwd_totalProbScale = 1.0;          /* (not a config.) Product of all scales affecting lm likelihoods.   Also read in HFBExactMPE.c and possibly
                                                 HMMIRest.c */
static FBLatInfo *fbInfo; /* current fbInfo, so don't have to pass it around. */

static ConfParam *cParm[MAXGLOBS];  /* config parameters */
static int nParm = 0;


/*    some macros and definitions..      */

/*--- These defines are also in HFBExactMPE.c ---- */
#define StartOfWord(a) (a->pos==0) 
#define EndOfWord(a) (a->pos == a->parentLarc->nAlign-1)  
#ifdef SUPPORT_QUINPHONE
#define IsSilence(name)  ( Quinphone ? (!strncmp(name,"sil",3)||!strncmp(name,"sp",2)) : (!strcmp(name,"sil")||!strcmp(name,"sp")))   
/*strncmp not strcmp, so as to handle sil_0,sp_0 for quinphone. */
#else
#define IsSilence(name)  (!strcmp(name,"sil")||strcmp(name,"sp"))  
#endif
#define IsStartOrEnd(word) (*word=='!' || *word == '<')
/*-------------------------------------------------- */

#define SET_totalProbScale hfwdbkwd_totalProbScale=langProbScale*latProbScale*probScale
#define translm(lmlike) ((lmlike*hfwdbkwd_totalProbScale)+phnInsPen)
/* #define DEBUG_MPE */



typedef struct{  /* R.E MPE code, these structures store all correct-lattice arcs active at time t */
   int start;
   int end;
   int i_label;
   int is_nonsil;  /* used in SetCorrectnessAsError */
} CorrectArc;

typedef struct _CorrectArcList{
   CorrectArc *h;
   struct _CorrectArcList *t;
} CorrectArcList;


/* -------------------------- Misc routines        ----------------------- */

/* returns phone (with no context) as an identifying int.  Turns a-b+c --> b and makes it into an integer. */

int GetNoContextPhone(LabId phone, int *nStates_quinphone/*actually,number of HMMs per phone*/, int *state_quinphone, HArc *a, int *frame_end){ 
   char _buf[MAXSTRLEN], *buf=_buf,*tmp; int i,len;
   int ans;
   char *lab = phone->name;

   if(PhoneMEEUseContext){
      if(Quinphone) HError(1, "Quinphone not compatible with context phones...");
      return (int) phone;
   }
   strcpy(buf,lab);
#ifdef SUPPORT_QUINPHONE
   if(Quinphone){
      if((tmp=strrchr(buf,'_'))){
         if(tmp[1] == '0' && tmp[2]=='\0'){ /*sil_0,sp_0*/
            *nStates_quinphone = 1; *state_quinphone = 2;
            *tmp = '\0';
            if((tmp=strchr(buf,'_'))) HError(1, ">1 \'_\' in quinphone ending _0");
            goto get_id;/*return GetId(buf);*/
         } else {   /* x_n_nnn */
            int i;
            *tmp='\0';
            if(!(tmp = strchr(buf,'_'))) HError(1, "Unrecognised quinphone %s", phone->name); 
            *tmp='\0';
            *nStates_quinphone = 3;
            if (  (*state_quinphone = i = tmp[1]-(int)'0') < 2 || i > 4) /* 2,3,4 */ HError(1, "Unrecognised quinphone %s [maybe more states than expected? Just change this line].", phone->name); 
            if(i==2 && frame_end!=NULL){ 
               if(!a || !a->follTrans || !a->follTrans->end->follTrans) HError(1, "Problem with arc structure in quinphones...");

               *frame_end=a->follTrans->end->follTrans->end->t_end; /*set frame_end*/  

            }
            goto get_id;/*return GetId(buf);*/
         }
      } else HError(1, "Unrecognised quinphone %s", phone->name); 
   }
#endif
   if((tmp=strchr(buf,'-'))){ buf=tmp+1; }
   if((tmp=strchr(buf,'+'))){ *tmp='\0'; }
 get_id:
   ans=0; tmp=buf;
   for(i=1,len=0;*buf;buf++,i<<=8,len++){
      ans =  ((int)ans + i * *buf);
   }
   /* Want to return an integer identifier for the string.  Put the string into an integer
      if it is <4 chars long, otherwise call GetLabId.  Calling GetLabId too much can be
      inefficient so I use this approach instead for short phones. */
   if(len > sizeof(int)) return (int) GetLabId(tmp, TRUE);
   else return ans;
}


static void SetCorrectness(FBLatInfo *fbInfo, Lattice *numLat){  
   /* Only called in MPE/MWE case, inexact version of code. */
   /* This works out the approximated correctness of each phone arc, based on overlap with arcs in the 
      correct lattice (numLat).  Formula is: if reference arc is z and hyp arc is q, if overlap in time 
      between q and z (relative to the length of z), equals e, then 
      (i)        if z and q are the same phone, correctness = -1 + 2e 
      (ii)       if z and q are not the same phone, correctness = -1 + e
      (i) is a backoff between -1 (insertion) and 1 (correct phone) 
      (ii) is a backoff between -1 (insertion) and 0 (substitution).
      This is based on the fact that the raw accuracy of the string = #correct - #inserted phones.  */
  
   HArc *a;
   int t;
   int larcid;
   CorrectArcList **correctArc;
   correctArc=New(&fbInfo->tempStack, sizeof(CorrectArcList*)*(fbInfo->T+1));
   ResetObsCache();
   if(!numLat) HError(-1, "MEE mode and no numLat provided.  FBLat needs to be given both lattices in this mode.");
   for(t=1;t<=fbInfo->T;t++)    correctArc[t]  = NULL;

   {
      CorrectArc *ca; CorrectArcList *cal;
      for(larcid=0;larcid<numLat->na;larcid++){ /* Set correct labels: */
         int seg;
         float start_time, end_time;  int i_start, i_end,i;

         start_time = numLat->larcs[larcid].start->time;
         if(PhoneMEE){
            for(seg=0;seg<numLat->larcs[larcid].nAlign;seg++){
               int quinphone_nstates,dummy;
               int quinphone_state,quinphone_newstate;
               int i_label;
               char *name = numLat->larcs[larcid].lAlign[seg].label->name;
               if(NoSilence && IsSilence(name)) continue;
               {
                  LabId label = numLat->larcs[larcid].lAlign[seg].label; 
                  if(!PhoneMEEUseContext) i_label = GetNoContextPhone(label,&quinphone_nstates,&quinphone_state,NULL,NULL); 
                  else i_label = (int)label; /* Use address of LabId. */
               }

               end_time = start_time + numLat->larcs[larcid].lAlign[seg].dur;
#ifdef SUPPORT_QUINPHONE
               if(Quinphone){
                  for(quinphone_nstates --; quinphone_nstates > 0;quinphone_nstates--){
                     LabId label;
                     seg++; /* Use one more state.. */
                     label  = numLat->larcs[larcid].lAlign[seg].label; 
                     if(seg>=numLat->larcs[larcid].nAlign){ /*outer loop condition not still true..*/
                        HError(1, "Confused about quinphones...[first quinphone of model was %s", name);
                     }
                     end_time += numLat->larcs[larcid].lAlign[seg].dur; 
                     /* Confirm correct phone + pos: */
                     if(!(i_label == GetNoContextPhone(label,&dummy,&quinphone_newstate,NULL,NULL) && quinphone_newstate == quinphone_state+1))
                        HError(1, "Confused about quinphones...[first quinphone of model was %s", name);
                     quinphone_state = quinphone_newstate;
                  }
               }
#endif	     
	     
               i_start = 1 + TimeToNFrames(start_time, fbInfo->aInfo);
               i_end = TimeToNFrames(end_time, fbInfo->aInfo);
	   
               ca = New(&fbInfo->tempStack, sizeof(CorrectArc));
               ca->start = i_start; ca->end = i_end; ca->i_label=i_label;

	     
#ifdef DEBUG_MPE
               printf("\"%s\" %d %d\n",  numLat->larcs[larcid].lAlign[seg].label->name, i_start,i_end);
#endif
	     
               for(i=i_start;i<=i_end;i++){
                  CorrectArcList *cal = New(&fbInfo->tempStack, sizeof(CorrectArcList));
                  cal->h = ca; cal->t = correctArc[i]; correctArc[i] = cal;
	       
               }
               start_time = end_time;
            }
	 } else { /* WordMEE */
            end_time = numLat->larcs[larcid].end->time;
            i_start = 1 + TimeToNFrames(start_time, fbInfo->aInfo);
            i_end = TimeToNFrames(end_time, fbInfo->aInfo);
#ifdef DEBUG_MPE
            printf("\"%s\" %d %d\n",  numLat->larcs[larcid].end->word->wordName->name, i_start,i_end);
#endif
            ca = New(&fbInfo->tempStack, sizeof(CorrectArc));
            ca->start = i_start; ca->end = i_end; 
            ca->i_label = (int)numLat->larcs[larcid].end->word->wordName;

            for(i=i_start;i<=i_end;i++){
               CorrectArcList *cal = New(&fbInfo->tempStack, sizeof(CorrectArcList));
               cal->h = ca; cal->t = correctArc[i]; correctArc[i] = cal;
            }
	 }
      }
#ifdef SUPPORT_QUINPHONE
      if(PhoneMEEUseContext&&Quinphone) HError(1, "Context and quinphone not compatible.");
#endif
      /*now set the correctness field (for MPE). */
      if(PhoneMEE){
	 for(a=fbInfo->aInfo->start;a;a=a->foll){  /* first do all arcs for which calcArc==NULL 
                                                      [these are the 'prototype' arcs for which acoustic info is calculated] */

            if(a->calcArc) a->mpe->correctness = 0; /* probably redundant. */
            else {  /* do the calculation for all of the 'representative' arcs, all other arcs are the same
                       as one of the 'representative' arcs and the correctness is copied. */ 
               int quinphone_nstates, quinphone_state;
               Boolean ZeroCorrectness=FALSE;
               LabId phone = a->phone;
               int iphone, iotherphone; /*as ints..*/
               int i_start, i_end,i;
               int currBegin=-1,currEnd=-1;
               float currCorrect = -1, tmpCorrect; /*-1 is the min correctness (for wrong phones).*/
               
               /*get phone start&end times.*/
               i_start=a->t_start;i_end=a->t_end;
               if(!PhoneMEEUseContext)  iphone = GetNoContextPhone(phone,&quinphone_nstates,&quinphone_state, a, &i_end);
               else iphone = (int)phone;
#ifdef SUPPORT_QUINPHONE
               if(Quinphone && quinphone_nstates>1 && quinphone_state > 2){ ZeroCorrectness = TRUE; }
#endif
               if(!ZeroCorrectness) /* ZeroCorrectness is true for non-start models in quinphone case */
                  for(i=i_start;i<=i_end;i++){ 
                     /*look for all the ref. phones in this time interval and choose the one giving highest correctness.*/
                     for(cal = correctArc[i];cal;cal=cal->t){
                        float proportion;
                        currBegin = cal->h->start; currEnd=cal->h->end;
                        iotherphone = cal->h->i_label;
                        proportion =
                           (float)(MIN(i_end,currEnd)-MAX(i_start,currBegin)+1) / ((float)(currEnd-currBegin+1));
                       
                        if(iotherphone == iphone){
                           /*Work out how much overlap we have with the correct phone*/
			   /* ref length div by overlap length */
                           tmpCorrect = proportion*(1-InsCorrectness); /*default:InsCorrectness=-1*/
                           tmpCorrect+=InsCorrectness;
                          
                           if(tmpCorrect > currCorrect)  currCorrect = tmpCorrect; 
                        }
                       
                        else{
                           tmpCorrect = proportion * -InsCorrectness;
                           tmpCorrect+= InsCorrectness;
                           if(tmpCorrect > currCorrect)  currCorrect = tmpCorrect; 
                        }
                     }
                  }
               if( IsSilence(phone->name) || ZeroCorrectness) /* If for some reason we arent counting this phone... */
                  currCorrect = 0;
#ifdef DEBUG_MPE
               printf("%s %d %d %.3f\n", phone->name, i_start,i_end, currCorrect);
#endif	    
               a->mpe->correctness = currCorrect;	     
              
            }
	 } 
	 for(a=fbInfo->aInfo->start;a;a=a->foll){ /*Now do the copies of the unique arcs [this is part of
                                                    an optimization. */
            if(a->calcArc){
               a->mpe->correctness = a->calcArc->mpe->correctness;
            }
	 }
      } else { /* MWE */
	 for(a=fbInfo->aInfo->start;a;a=a->foll){
            if(a->calcArc==NULL){
               LArc *la = a->parentLarc; /*for getting time.*/
               if(a->pos!=0) a->mpe->correctness=0; /*not word-start.  We just put the correctness in the beginning-of-word phones.*/
               else {
                  float start_time, end_time;  int i_start, i_end,i; 
                  LabId word = a->word; 
                  int currBegin=-1,currEnd=-1;
                  float currCorrect = -1; /*-1 is the min correctness (for wrong phones).*/
                  if(!word) HError(1, "word is zero...coding error.");
                  start_time = la->start->time; 
                  end_time =  la->end->time;
                  i_start = 1 + TimeToNFrames(start_time, fbInfo->aInfo);
                  i_end = TimeToNFrames(end_time, fbInfo->aInfo);
               
                  for(i=i_start;i<=i_end;i++){ /*look for all the ref. phones in this time interval.*/
                     for(cal = correctArc[i];cal;cal=cal->t){
                        float proportion;
                        int otherWord; 
                        currBegin = cal->h->start; currEnd=cal->h->end;
                        otherWord = cal->h->i_label; /*is actually the (int)LabId in this case.*/
                        proportion =
                           (float)(MIN(i_end,currEnd)-MAX(i_start,currBegin)+1)/ ((float)(currEnd-currBegin+1)); 
                        if(otherWord == (int)word)
                           /*Work out how much overlap we have with the correct phone*/
                           /* ref length div by overlap length */
                           currCorrect = MAX(currCorrect, InsCorrectness + proportion*(-InsCorrectness+1));
                        else
                           currCorrect = MAX(currCorrect, InsCorrectness * ( 1 - proportion));
                     
                     }
                  }
                  if(IsStartOrEnd(word->name)){ /* start/end of file, !SENT_* or <s> or </s>  */
                     currCorrect = 0;
                  }
#ifdef DEBUG_MPE
                  printf("%s %d %d %.3f\n", word->name, i_start,i_end, currCorrect);
#endif
                  a->mpe->correctness = currCorrect;
               }
            } else a->mpe->correctness = 0;
	 }
	 for(a=fbInfo->aInfo->start;a;a=a->foll)
            if(a->calcArc) a->mpe->correctness = a->calcArc->mpe->correctness;
      }
   }    
   if (fbInfo->T != fbInfo->aInfo->T) HError(1, "Lattice and acoustics mismatch in T (%d[lat],%d[acoustic]): may have to set e.g. FRAMEDUR=x (for x!=0.01) in config file", fbInfo->aInfo->T,fbInfo->T);
   Dispose(&fbInfo->tempStack, correctArc);
}

float GetFinalError(float curr_total, float curr_corr, Boolean hyp_is_sil){  /* re the "CALCASERROR" option, other version of approx MPE. */
  float ins=0,sub,del=0;
  if(hyp_is_sil){ del= curr_total; return del; /* All phones [i.e. non-silence phones ] are insertions if hyp is silence. */ }
  else {
    if(curr_total>1){
      del = curr_total-1;
      if(curr_corr > 1) sub=0;
      else sub = 1-curr_corr;
    } else {
      ins = 1-curr_total;
      sub = curr_total - curr_corr;
    }
  }
  return ins*-InsCorrectness + del + sub;
}

/* re the "CALCASERROR" option, other version of approx MPE. */
float GetLowestNegError(int tStart, int tEnd, int tCurr, float curr_corr, float curr_total,  CorrectArcList **correctArc, int iphone, int *compute_count, Boolean hyp_is_sil){
  float best = -100,tmp;
  int currBegin,currEnd;
  float proportion;
  int iotherphone,is_nonsil;
  CorrectArcList *cal;
  if(tCurr > tEnd){
    (*compute_count)--;
    return -GetFinalError(curr_total,curr_corr, hyp_is_sil);  /* Note that this will give "sp" a zero error, but "sp" is set to zero error anyway. */
  }
  else {

    if(tCurr == tStart){  /* If this is the lhs of the interval, can accept partial phones.. */
      for(cal = correctArc[tCurr];cal && *compute_count>0;cal=cal->t){
	currBegin = cal->h->start; currEnd=cal->h->end;
	proportion =
	  (float)(MIN(tEnd,currEnd)-MAX(tStart,currBegin)+1) / ((float)(currEnd-currBegin+1)); /*  overlap as proportion of ref phone length */
	iotherphone = cal->h->i_label;  is_nonsil = cal->h->is_nonsil;
	curr_corr = (iphone==iotherphone)*proportion*is_nonsil;  /* make everything zero if ref is silence. */
	curr_total = proportion *is_nonsil;
	tmp = GetLowestNegError(tStart, tEnd, MIN(currEnd,tEnd)+1, curr_corr, curr_total, correctArc, iphone, compute_count,  hyp_is_sil);
	best = MAX(tmp,best);
      }
      return best;
    } else {  /* Not lhs of interval, so  only use phones starting at this point.. */
      for(cal = correctArc[tCurr];cal && *compute_count>0;cal=cal->t){
	currBegin = cal->h->start; currEnd=cal->h->end;
	if(currBegin==tCurr){
	  proportion =
	    (float)(MIN(tEnd,currEnd)-MAX(tStart,currBegin)+1) / ((float)(currEnd-currBegin+1)); /*  overlap as proportion of ref phone length */
	  iotherphone = cal->h->i_label;  is_nonsil = cal->h->is_nonsil; 
	  /*tmp = GetLowestNegError(tStart, tEnd, MIN(currEnd,tEnd)+1, curr_corr + (iphone==iotherphone)*proportion*is_nonsil, curr_total+proportion*is_nonsil, correctArc, iphone, compute_count, 
				  hyp_is_sil); */
	  tmp = GetLowestNegError(tStart, tEnd, MIN(currEnd,tEnd)+1, MAX(curr_corr, (iphone==iotherphone)*proportion*is_nonsil), curr_total+proportion*is_nonsil, correctArc, iphone, compute_count, 
				  hyp_is_sil); 
	  best = MAX(tmp,best);
	}
      }
      if(best==-100 && *compute_count > 0) HError(-1, "Error in computing approximate error (no matching phone-start found...)");
      return best;
    }
  }
}


static void SetCorrectnessAsError(FBLatInfo *fbInfo, Lattice *numLat){    /* re the "CALCASERROR" option, other version of approx MPE. */
   /* Only called in MPE/MWE case, inexact version of code. */
   /* This works out the approximated error (as opposed to correctness) of each phone arc, based on overlap with arcs in the 
      correct lattice (numLat).  
      The number of errors is #ins + #del + #sub.

   Need approximation for each phone's contribution, it's #sub+#ins+#del$, where we
   use the approximation:


     E(q) =  the minimum (for all reference sentences indexed by n) of :  
                q=non-silence -->
                                             #sub =     max( 1-c(q,n),  0) 
                                           + #ins =     max( t(q,n)-1,  0)   
                                           + #del =     max( 1-t(q,n),  0)

                 q=silence  ->               #ins =     t(q,n)

  where t(q,n) is the approximated total number of non-silence phones in the reference that 
 align with q and c(q,n) is the approximated number of correct phones that align with q.
 The total t(q,n) >=  0 is found by summing, for each reference phone z in the sentence s_r^n that overlaps with q,
 the proportion of each phone z that overlaps with q as a fraction of the length of z.   The 
 number of correct phones 0 \leq c(q,n) \leq 1 is the largest amount of overlap between a phone z in s_r^n
 that is the same phone as q, again as a fraction of the length of z.

   */

  
   HArc *a;
   int t;
   int larcid;
   CorrectArcList **correctArc;
   correctArc=New(&fbInfo->tempStack, sizeof(CorrectArcList*)*(fbInfo->T+1));
   ResetObsCache();
   if(!numLat) HError(-1, "MEE mode and no numLat provided.  FBLat needs to be given both lattices in this mode.");
   for(t=1;t<=fbInfo->T;t++)    correctArc[t]  = NULL;

   {   /* Set up arrays of correct-transcription.. */
      CorrectArc *ca; 
      for(larcid=0;larcid<numLat->na;larcid++){ /* Set correct labels: */
         int seg;
         float start_time, end_time;  int i_start, i_end,i;

         start_time = numLat->larcs[larcid].start->time;
         if(PhoneMEE){
            for(seg=0;seg<numLat->larcs[larcid].nAlign;seg++){
               int quinphone_nstates,dummy;
               int quinphone_state,quinphone_newstate;
               int i_label, is_nonsil;
               char *name = numLat->larcs[larcid].lAlign[seg].label->name;
               if(NoSilence && IsSilence(name)) continue;
               {
                  LabId label = numLat->larcs[larcid].lAlign[seg].label; 
                  if(!PhoneMEEUseContext) i_label = GetNoContextPhone(label,&quinphone_nstates,&quinphone_state,NULL,NULL); 
                  else i_label = (int)label; /* Use address of LabId. */

		  is_nonsil = (int) ! IsSilence(label->name);
               }

               end_time = start_time + numLat->larcs[larcid].lAlign[seg].dur;
#ifdef SUPPORT_QUINPHONE
               if(Quinphone){
                  for(quinphone_nstates --; quinphone_nstates > 0;quinphone_nstates--){
                     LabId label;
                     seg++; /* Use one more state.. */
                     label  = numLat->larcs[larcid].lAlign[seg].label; 
                     if(seg>=numLat->larcs[larcid].nAlign){ /*outer loop condition not still true..*/
                        HError(1, "Confused about quinphones...[first quinphone of model was %s", name);
                     }
                     end_time += numLat->larcs[larcid].lAlign[seg].dur; 
                     /* Confirm correct phone + pos: */
                     if(!(i_label == GetNoContextPhone(label,&dummy,&quinphone_newstate,NULL,NULL) && quinphone_newstate == quinphone_state+1))
                        HError(1, "Confused about quinphones...[first quinphone of model was %s", name);
                     quinphone_state = quinphone_newstate;
                  }
               }
#endif	     
	     
               i_start = 1 + TimeToNFrames(start_time, fbInfo->aInfo);
               i_end = TimeToNFrames(end_time, fbInfo->aInfo);
	   
               ca = New(&fbInfo->tempStack, sizeof(CorrectArc));
               ca->start = i_start; ca->end = i_end; ca->i_label=i_label; ca->is_nonsil=is_nonsil;

	     
#ifdef DEBUG_MPE
               printf("\"%s\" %d %d\n",  numLat->larcs[larcid].lAlign[seg].label->name, i_start,i_end);
#endif
	     
               for(i=i_start;i<=i_end;i++){
                  CorrectArcList *cal = New(&fbInfo->tempStack, sizeof(CorrectArcList));
                  cal->h = ca; cal->t = correctArc[i]; correctArc[i] = cal;
	       
               }
               start_time = end_time;
            }
	 } else { /* WordMEE */
            end_time = numLat->larcs[larcid].end->time;
            i_start = 1 + TimeToNFrames(start_time, fbInfo->aInfo);
            i_end = TimeToNFrames(end_time, fbInfo->aInfo);
#ifdef DEBUG_MPE
            printf("\"%s\" %d %d\n",  numLat->larcs[larcid].end->word->wordName->name, i_start,i_end);
#endif
            ca = New(&fbInfo->tempStack, sizeof(CorrectArc));
            ca->start = i_start; ca->end = i_end; 
            ca->i_label = (int)numLat->larcs[larcid].end->word->wordName;

            for(i=i_start;i<=i_end;i++){
               CorrectArcList *cal = New(&fbInfo->tempStack, sizeof(CorrectArcList));
               cal->h = ca; cal->t = correctArc[i]; correctArc[i] = cal;
            }
	 }
      }
#ifdef SUPPORT_QUINPHONE
      if(PhoneMEEUseContext&&Quinphone) HError(1, "Context and quinphone not compatible.");
#endif
      /*now set the correctness field (for MPE). */
      if(PhoneMEE){
	 for(a=fbInfo->aInfo->start;a;a=a->foll){  /* first do all arcs for which calcArc==NULL 
                                                      [these are the 'prototype' arcs for which acoustic info is calculated] */

            if(a->calcArc) a->mpe->correctness = 0; /* probably redundant. */
            else {  /* do the calculation for all of the 'representative' arcs, all other arcs are the same
                       as one of the 'representative' arcs and the correctness is copied. */ 
               int quinphone_nstates, quinphone_state;
               Boolean ZeroCorrectness=FALSE;
               LabId phone = a->phone;
               int iphone; /*as ints..*/
               int i_start, i_end;
               float currCorrect; /*-1 is the min correctness (for wrong phones).*/
               
               /*get phone start&end times.*/
               i_start=a->t_start;i_end=a->t_end;
               if(!PhoneMEEUseContext)  iphone = GetNoContextPhone(phone,&quinphone_nstates,&quinphone_state, a, &i_end);
               else iphone = (int)phone;
#ifdef SUPPORT_QUINPHONE
               if(Quinphone && quinphone_nstates>1 && quinphone_state > 2){ ZeroCorrectness = TRUE; }
#endif
               if(!ZeroCorrectness){ /* ZeroCorrectness is true for non-start models in quinphone case */
		 int compute_count = 100; /* Limit computation... */
		 currCorrect = GetLowestNegError(i_start, i_end, i_start, 0, 0, correctArc, iphone, &compute_count, IsSilence(phone->name));
	       }
         
               if( /* IsSilence(phone->name) || */ ZeroCorrectness) /* If for some reason we arent counting this phone... relates to quinphones,
       		      I'm not sure if these work with this part of the code. */
		 currCorrect = 0;
#ifdef DEBUG_MPE
               printf("%s %d %d %.3f\n", phone->name, i_start,i_end, currCorrect);
#endif	    
               a->mpe->correctness = currCorrect;	     
              
            }
	 } 
	 for(a=fbInfo->aInfo->start;a;a=a->foll){ /*Now do the copies of the unique arcs [this is part of
                                                    an optimization. */
            if(a->calcArc){
               a->mpe->correctness = a->calcArc->mpe->correctness;
            }
	 }
      } else HError(1, "MWE not supported for approximate error...");

   }  
   if (fbInfo->T != fbInfo->aInfo->T) HError(1, "Lattice and acoustics mismatch in T (%d[lat],%d[acoustic]): may have to set e.g. FRAMEDUR=x (for x!=0.01) in config file", fbInfo->aInfo->T,fbInfo->T);
   Dispose(&fbInfo->tempStack, correctArc);
}




/* ZeroAlpha: zero alpha's of all models */
static void ZeroAlpha(int sq, int eq)
{
   int Nq,j,q;
   DVector aq;
  
   for (q=sq;q<=eq;q++) {
      Acoustic *ac = fbInfo->aInfo->ac+q;
      Nq = ac->hmm->numStates; 
      if(!ac->SP){
         aq = fbInfo->aInfo->ac[q].alphat; 
         for (j=1;j<=Nq;j++)
            aq[j] = LZERO;
      }
   }
}

/* StepAlpha: calculate alphat column for time t */
/* Calculates the forward (alpha) likelihoods given the previous alpha likelihoods, i.e. for t-1 */
static void StepAlpha(int t)
{
   DVector aq,laq,tmp;
   float ***outprob;
   int i,j,q,Nq;
   LogDouble x=0.0,y,a;
   HLink hmm;
   

   for (q = fbInfo->aInfo->qLo[t]; q <= fbInfo->aInfo->qHi[t]; q++) {  /*swap alphat, alphat1*/
      Acoustic *ac = fbInfo->aInfo->ac+q;
      if(!ac->SP){   tmp = ac->alphat1; ac->alphat1=ac->alphat; ac->alphat=tmp; }
   }

   /* Zero any alphas that may be nonzero.*/
   /* not needed. */
   /*  if(t>2)
       ZeroAlpha(fbInfo->aInfo->qLo[t-2],fbInfo->aInfo->qHi[t-2]);   / * Because the alphat vectors are swapped over each time,
       we need to zero the one from t-2. */
   
   for (q = fbInfo->aInfo->qLo[t]; q <= fbInfo->aInfo->qHi[t]; q++) { /*This is just to avoid iterating over all q's.*/
      Acoustic *ac = fbInfo->aInfo->ac+q;
     
      if(t >= ac->t_start && t<=ac->t_end){ /*If it's in the beam...*/
         hmm = ac->hmm; Nq = hmm->numStates; 
         aq = ac->alphat; laq = (t-1>=ac->t_start&&t-1<=ac->t_end)?ac->alphat1:NULL;
         if((outprob = ac->otprob[t]) == NULL)
            HError(2322,"StepAlpha: Outprob NULL at time %d model %d in StepAlpha",t,q);
       


         if(t==ac->t_start) aq[1] = ac->locc - ac->aclike;
         /* This is the forward log likelihood at the start of the phone model.  It is a "cheating" log likelihood
            in the sense that it is calculated to give the model the log occupancy "locc".  */
         else aq[1] = LZERO;  /* no entry to the model unless at its start time. */

         x=LZERO;
         for (j=2;j<Nq;j++) { /*Calculate the alpha probs for the emitting states.*/
            a = hmm->transP[1][j];
            x = (a>LSMALL)?a+aq[1]:LZERO;
            for (i=2;i<=Nq;i++){
               a = hmm->transP[i][j]; y = (laq?laq[i]:LZERO);
               if (a>LSMALL && y>LSMALL)
                  x = LAdd(x,y+a);
            }
            aq[j] = x + outprob[j][0][0];
         }

         x = LZERO;
         for (i=2;i<Nq;i++){
            a = hmm->transP[i][Nq]; y = aq[i];
            if (a>LSMALL && y>LSMALL)
               x = LAdd(x,y+a);
         }
         aq[Nq] = x;
       
         if(t==ac->t_end){ /*Work out the exit prob, just for checking purposes......  */
            double transP;
            hmm = ac->hmm; Nq = hmm->numStates; 
            transP = hmm->transP[1][Nq];
            aq = ac->alphat;
	 
            x = aq[Nq];
            if ( fabs(x - ac->locc) > 0.001 )
               HError(1, "StepAlpha: problem with occs.. (fabs(x-locc)=%d (>0.001))",x-ac->locc); /*x is like an occupancy for that segment.*/
         }
      }
   }
}

/* NewOtprobVec: create prob vector size [0..M] */
static float * NewOtprobVec(MemHeap *x, int M)
{
   float *v;
   int MM,m;

   MM=(M==1)?1:M+1;
   v=(float *)New(x, MM*sizeof(float));
   v[0]=LZERO;
   if (M>1)
      for (m=1;m<=M;m++)
	 v[m] = LZERO;
   return v;
}


/* ShStrP: Stream Outp calculation exploiting sharing */
static float * ShStrP(Vector v, int t, StreamElem *ste, AdaptXForm *xform, MemHeap *amem)
{
   WtAcc *wa;
   MixtureElem *me;
   MixPDF *mp;
   float *outprobjs;
   int m,M;
   PreComp *pMix;
   LogFloat det,x,mixp;
   
   wa = (WtAcc *)ste->hook;
   if (wa->time==t)           /* seen this state before */
      outprobjs = wa->prob;
   else {
      M = ste->nMix;
      outprobjs = NewOtprobVec(amem,M);
      me = ste->spdf.cpdf+1;
      if (M==1){                 /* Single Mix Case */
         mp = me->mpdf;
         pMix = (PreComp *)mp->hook;
         if (pMix->time == t)
            x = pMix->prob;
         else {
            x = MOutP(ApplyCompFXForm(mp,v,xform,&det,t),mp);
            x += det; 
            pMix->prob = x; pMix->time = t; /*dp10006:*/pMix->indx=-1;  /*This relates to the accumulation of the occ.*/
         }
      } else {                   /* Multiple Mixture Case */
         x = LZERO;
         for (m=1;m<=M;m++,me++) {
            if (MixWeight(fbInfo->hset,me->weight)>MINMIX){
               mp = me->mpdf;
               pMix = (PreComp *)mp->hook;
               if (pMix->time==t)
                  mixp = pMix->prob;
               else {
                  mixp = MOutP(ApplyCompFXForm(mp,v,xform,&det,t),mp);
                  mixp += det; 
		  if(isnan(mixp)) HError(1, "mixp zero...");
                  pMix->prob = mixp; pMix->time = t; pMix->indx=-1;
               }
               x = LAdd(x,MixLogWeight(fbInfo->hset,me->weight)+mixp);
	       outprobjs[m] = mixp;
            }
         }
      }
      outprobjs[0] = x;
      wa->prob = outprobjs;
      wa->time = t;
   }
   return outprobjs;
}
   

/* Setotprob: allocate and calculate otprob matrix at time t */
static void Setotprob(int t)
{
   int q,j,Nq,s;
   float ***outprob;
   StreamElem *ste;
   HLink hmm;
   LogFloat sum;
   float local_probscale;
  
   ReadAsTable(fbInfo->al_pbuf,t-1,&fbInfo->al_ot); 
    
   local_probscale = probScale;   /* direct scale on acoustics on state level, usu. 1 */

   if (fbInfo->hsKind == TIEDHS)
      PrecomputeTMix(fbInfo->hset,&(fbInfo->al_ot),minFrwdP,0);
  
  
   for (q=fbInfo->aInfo->qHi[t];q>=fbInfo->aInfo->qLo[t];q--) {
      if(t>=fbInfo->aInfo->ac[q].t_start && t<=fbInfo->aInfo->ac[q].t_end) { /* HMM is active at this time... */
         Acoustic *ac = fbInfo->aInfo->ac+q;
         hmm = ac->hmm; Nq = hmm->numStates;
         outprob = ac->otprob[t];
      
         for (j=2;j<Nq;j++){
            ste=hmm->svec[j].info->pdf+1; sum = 0.0;


            for (s=1;s<=fbInfo->S;s++,ste++){
               switch (fbInfo->hsKind){
               case TIEDHS:	 /* SOutP deals with tied mix calculation */
               case DISCRETEHS:
                  if (fbInfo->S==1) {
                     outprob[j][0] = NewOtprobVec(fbInfo->aInfo->mem,1);
                     outprob[j][0][0] = SOutP(fbInfo->hset,s,&fbInfo->al_ot, ste);
                  } else {
                     outprob[j][s] = NewOtprobVec(fbInfo->aInfo->mem,1);
                     outprob[j][s][0] = SOutP(fbInfo->hset,s,&fbInfo->al_ot, ste);
                  }
		  break;
               case PLAINHS: /* x = SOutP(fbInfo->hset,s,&ot,ste);    break; commented out by dp10006 since
                                sharing is needed in any case for lattices. */
               case SHAREDHS:
		  if (fbInfo->S==1)
		     outprob[j][0] = ShStrP(fbInfo->al_ot.fv[s],t+StartTime,ste,fbInfo->inXForm,fbInfo->aInfo->mem);
		  else
		     outprob[j][s] = ShStrP(fbInfo->al_ot.fv[s],t+StartTime,ste,fbInfo->inXForm,fbInfo->aInfo->mem);
		  break;
               default:       HError(1, "Unknown hset kind.");
               }
               if (fbInfo->S==1)
		  outprob[j][0][0] *= local_probscale;
               else{
		  outprob[j][s][0] *= local_probscale;
		  sum += outprob[j][s][0];
               }
            }
            if (fbInfo->S>1){
               outprob[j][0][0] = sum;
               for (s=1;s<=fbInfo->S;s++)
		  outprob[j][s][0] = sum - outprob[j][s][0];
            }
         }
      }
   }
}

void SetModelBetaPlus(int t, int q){
   double x=LZERO;
   Acoustic *ac = fbInfo->aInfo->ac+q;
   HLink hmm = ac->hmm;
   int Nq = hmm->numStates,i,j;
   DVector bqt = ac->betaPlus[t],bqt1;
   float ***outprob = ac->otprob[t];

   if(t==ac->t_end)  bqt[Nq] = 0;  /* We are calculating the acoustic likelihood for each model separately. */
   else bqt[Nq] = LZERO;
  
   for(i=2;i<Nq;i++){
      x = bqt[Nq] + hmm->transP[i][Nq];
      if(t+1<=ac->t_end){ /*in beam next time frame*/
         bqt1=ac->betaPlus[t+1];
         for(j=2;j<Nq;j++)
            x = LAdd(x, bqt1[j] + hmm->transP[i][j]);
      }
      x += outprob[i][0][0];
      bqt[i] = x;
   }
   x=LZERO;
   for(i=2;i<Nq;i++) x = LAdd(x, bqt[i]+hmm->transP[1][i]);
   bqt[1] = x;
}


/* SetBetaPlus: calculate gamma and otprob matrices */
static void SetBetaPlus()
{
   int t,q; /*,lNq=0,q_at_gMax;*/
   LogDouble x;
  
   
   /* 
      Columns T-1 -> 1.
   */
   ResetObsCache();  
   for (t=fbInfo->T;t>=1;t--) {
      Setotprob(t);
      for (q=fbInfo->aInfo->qHi[t];q>=fbInfo->aInfo->qLo[t];q--) { /*MAX(qHi[t],qLo[t]) because of the case for tee models where qHi[t]=qLo[t]-1 .*/
         Acoustic *ac = fbInfo->aInfo->ac + q;
         if(t>=ac->t_start && t<=ac->t_end){ /*in beam.*/
            SetModelBetaPlus(t,q);
         }
         if(t==ac->t_start){ /* We need to set "aclike", the total accumulated acoustic
                                probability for this frame. */
            if(ac->SP) /* Is a tee model, i.e. zero time, t_end=t_start-1. */
               x = ac->hmm->transP[1][ac->hmm->numStates]; /* beta will not have been set for tee models. */
            else
               x = ac->betaPlus[t][1]; /* state 1 of HMM. */
            ac->aclike = x;
         }	
      }
   }
}




void UpSkipTranParms(int q, int t){
   Acoustic *ac = fbInfo->aInfo->ac+q;
   HLink hmm=ac->hmm;
   double occ = ac->locc;
   float mee_acc_scale = fbInfo->AccScale*(fbInfo->MPE?ac->mpe_occscale:1), abs_mee_acc_scale = fabs(mee_acc_scale); 
   int local_accindx = (mee_acc_scale > 0 ? fbInfo->num_index : fbInfo->den_index);
   TrAcc *ta,*tammi=NULL; int N = hmm->numStates; ta = ((TrAcc*)GetHook(hmm->transP)) + local_accindx;
   if(DoingFourthAcc) tammi = ((TrAcc*)GetHook(hmm->transP)) + add_index;  

   if(occ > MINEARG){
      float occmmi = exp(occ);  
      ta->occ[1] += occmmi * abs_mee_acc_scale;
      ta->tran[1][N] += occmmi * abs_mee_acc_scale;
      if(DoingFourthAcc) {   /* doing 4th acc for MPE with MMI prior */
         tammi->occ[1] += occmmi;
         tammi->tran[1][N] += occmmi;
      }
   }
}

/* UpTranParms: update the transition counters of given hmm */

static void UpTranParms(int t, int q){ 
   TrAcc *ta,*tammi=NULL;   
   Acoustic *ac = fbInfo->aInfo->ac+q;
   HLink hmm = ac->hmm;
   float x=0,mee_acc_scale = fbInfo->AccScale*(fbInfo->MPE?ac->mpe_occscale:1), abs_mee_acc_scale = fabs(mee_acc_scale);   
   DVector aqt = ac->alphat,
      bqtPlus = ac->betaPlus[t],
      bqt1Plus = (t<ac->t_end ? ac->betaPlus[t+1] : NULL);
   int local_accindx = (mee_acc_scale > 0 ? fbInfo->num_index : fbInfo->den_index);
   int i,j,N;

   N = hmm->numStates;    ta = ((TrAcc*)GetHook(hmm->transP)) + local_accindx;
   if(DoingFourthAcc) tammi = ((TrAcc*)GetHook(hmm->transP)) + add_index;   


   for(i=1;i<N;i++){
      float *ti = ta->tran[i], *ai = hmm->transP[i];
      for(j=2;j<=N;j++){
         if (i==1 && j<N) {                  /* entry transition */
            x = aqt[1]+ai[j]+bqtPlus[j]; /* note, aqt[1] lacks prob from this time since state 1 has
                                            no pdf, so OK to use alpha and betaPlus from same time. */
         } else if(i>1&&j<N&&bqt1Plus!=NULL) {   /* internal transition */
            x = aqt[i]+ai[j]+bqt1Plus[j];  /* use alpha from t and beta from t+1 */
         } else if (i>1 && j==N) {               /* exit transition */
            x = aqt[i]+ai[N]+bqtPlus[N]; /* bqtPlus[N] lacks likelihood from this time. */
         }
         /* dont do tee transition in this part of the code. */

         if(x > 0.001) HError(1, "too large transition occ! (%d>0)",x);
         if (x>MINEARG){
            float occmmi,occ;   
            occmmi = exp(x);
            occ = occmmi*abs_mee_acc_scale;
            ti[j] += occ; ta->occ[i] += occ;
            if(DoingFourthAcc) {   /* do 4th acc if MPE with MMI prior */
               tammi->tran[i][j] += occmmi;
               tammi->occ[i] += occmmi;
            }
         }
      }
   }
}



typedef struct{
   MixPDF *mp;
   float occ;
   float scaledOcc; /*for MEE.*/
} MixOcc;

Boolean CachingInitialised = FALSE;
MemHeap cacheMixoccHeap;

/* Following variables relate to caching of mixture occupation probabilities. */


static int nPDFs[SMAX];  /*rely on it being initialised to zeros.*/
static int SavedMixesSize[SMAX]; 
static MixOcc *SavedMixes[SMAX]; /* [1..S][1..nPDFs[s]] */


void DoMixUpdate(MixPDF *mp, int s, float Lr, float meescale, int t){  
   /* Stores the mp for update later...  The updates are performed once every time frame.  Avoids
      accumulating stats more than once for the same Gaussian.  */
  
   int RealT = -(10+t+StartTime); /*now t is a unique identifier; the minus is to distinguish from the use of PreComp for caching of OutPs.
                                    10 is to avoid zero. */
   PreComp *pMix;
   pMix = (PreComp *)mp->hook;

   if(pMix->time != RealT){
      int indx = nPDFs[s]++;
      pMix->indx = indx;
      pMix->time = RealT;
      if(SavedMixesSize[s] <= indx){
         MixOcc *NewArray;
         int NewSize = MAX(100, SavedMixesSize[s]*2), n;
         SavedMixesSize[s] = NewSize;
         NewArray = New(&cacheMixoccHeap, sizeof(MixOcc) * NewSize);
         for(n=0;n<indx;n++){
            NewArray[n] = SavedMixes[s][n];
         }
         if(SavedMixes[s]!=NULL)
            Dispose(&gcheap, SavedMixes[s]);
         SavedMixes[s] = NewArray;
      }
      SavedMixes[s][indx].mp = mp;
      SavedMixes[s][indx].occ = 0;
      SavedMixes[s][indx].scaledOcc = 0;
   }
   SavedMixes[s][pMix->indx].occ += Lr;
   SavedMixes[s][pMix->indx].scaledOcc += Lr*meescale;

}

void DoAllMixUpdates(int t){
   int s,m,k,vSize;
   MixPDF *mp;
   float Lr, unscaledLr, LrWithSign;
   Vector mean,variance,up_otvs,al_otvs,mu_jm;
   LogFloat det;

   float zmean,zmeanlr;
   MuAcc *ma,*mammi;
   VaAcc *va,*vammi;
   int local_accindx;
  
   for(s=1;s<=fbInfo->S;s++){
      float steSumLr = 0.0;
      vSize = fbInfo->hset->swidth[s];
      al_otvs = fbInfo->al_ot.fv[s];
    
      for(m=0;m<nPDFs[s];m++){
         unscaledLr = SavedMixes[s][m].occ;  /*differs in MPE case from Lr*/
         LrWithSign = SavedMixes[s][m].scaledOcc; 

         if(LrWithSign>0.0) local_accindx = fbInfo->num_index; else local_accindx = fbInfo->den_index;
         Lr = fabs(LrWithSign);

         mp = SavedMixes[s][m].mp;
         steSumLr += unscaledLr; /*just a check.*/
         mean = mp->mean; 
         variance = mp->cov.var;
      
         if(mp->ckind != INVDIAGC && mp->ckind != DIAGC){
            HError(-1, "HFwdBkwdLat.c: expecting INVDIAGC or DIAGC");
         }

      /* -------------------- (a) MLLR updates --------------------*/
         if (fbInfo->twoDataFiles){
             up_otvs = ApplyCompFXForm(mp,fbInfo->up_ot.fv[s],fbInfo->paXForm,&det,t);
         } else {
             up_otvs = ApplyCompFXForm(mp,fbInfo->al_ot.fv[s],fbInfo->paXForm,&det,t);
         } 
         if (fbInfo->uFlags&UPXFORM) 
            AccAdaptFrame(fbInfo->hset,Lr, up_otvs, mp, t);   /* note: discriminative transform update needs to be investigated further */

         /* -------------------- (c) Std Mixture updates --------------------*/
         if (fbInfo->uFlags&UPMEANS) { /* cant update vars but not means. */


            ma = ((MuAcc *) GetHook(mean))+local_accindx; mu_jm = ma->mu;
            if(DoingFourthAcc) mammi = ((MuAcc *) GetHook(mean))+add_index;

            if (fbInfo->uFlags&UPVARS){ /* This code is longer than it has to be, to reduce if-statements within loops. */
               switch(mp->ckind){
               case DIAGC:
               case INVDIAGC:
                  va = ((VaAcc *) GetHook(variance))+local_accindx; 
                  va->occ += Lr; 
                  ma->occ += Lr;
                  for (k=1;k<=vSize;k++) {
                     zmean=up_otvs[k]-mean[k]; zmeanlr=zmean*Lr;
                     mu_jm[k] += zmeanlr;
                     va->cov.var[k] += zmean*zmeanlr; 
                  }
                  if(DoingFourthAcc){   
                     vammi = ((VaAcc *) GetHook(variance))+add_index;
                     mammi->occ += unscaledLr;
                     vammi->occ += unscaledLr;
                     for (k=1;k<=vSize;k++) {
                        zmean=up_otvs[k]-mean[k]; zmeanlr=zmean*unscaledLr;
                        mammi->mu[k] += zmeanlr;
                        vammi->cov.var[k] += zmean*zmeanlr; 
                     }
                  }
                  break;
               case FULLC:
                  va = ((VaAcc *) GetHook(variance))+local_accindx; 
                  va->occ += Lr; 
                  ma->occ += Lr;
                  for (k=1;k<=vSize;k++) {
                     int j;
                     zmean=up_otvs[k]-mean[k]; zmeanlr=zmean*Lr;
                     mu_jm[k] += zmeanlr;
                     for(j=1;j<=k;j++) {
                        float zmeanj =up_otvs[j]-mean[j];
                        va->cov.inv[j][k] += zmeanj*zmeanlr; 
                     }
                  } 
                  if(DoingFourthAcc){   
                     vammi = ((VaAcc *) GetHook(variance))+add_index; 
                     vammi->occ += unscaledLr; 
                     mammi->occ += unscaledLr;
                     for (k=1;k<=vSize;k++) {
                        int j;
                        zmean=up_otvs[k]-mean[k]; zmeanlr=zmean*unscaledLr;
                        mammi->mu[k] += zmeanlr;
                        for(j=1;j<=k;j++) {
                           float zmeanj =up_otvs[j]-mean[j];
                           vammi->cov.inv[j][k] += zmeanj*zmeanlr; 
                        }
                     } 
                  }
                  break;
               default: HError(1, "Unknown CKIND.");
               }
            } else {
               ma->occ += Lr;
               for (k=1;k<=vSize;k++) {
                  zmean=up_otvs[k]-mean[k]; zmeanlr=zmean*Lr;
                  mu_jm[k] += zmeanlr;
               }
               if(DoingFourthAcc){   
                  mammi->occ += unscaledLr;
                  for (k=1;k<=vSize;k++) {
                     zmean=up_otvs[k]-mean[k]; zmeanlr=zmean*unscaledLr;
                     mammi->mu[k] += zmeanlr;
                  }
               }
            }
         }
      }
      if(steSumLr > 1.01 || steSumLr < 0.99) HError(-1, "Wrong steSumLr: %f, t=%d, s=%d",steSumLr, t, s);
   }
   for(s=1;s<=fbInfo->S;s++) /*Reset.*/
      nPDFs[s] = 0;
}


/* UpMixParms: update mu/va accs of given hmm  */
static double UpMixParms(int q, HLink hmm, int t, DVector aqt, 
			 DVector aqt1, DVector gqt)
{
   Acoustic *ac = fbInfo->aInfo->ac+q;
   double ans=LZERO;   
   double ans2=LZERO;   
   int mx,s,j,m=0,M=0,N,vSize;
   TMixRec *tmRec = NULL;
   float ***outprob;
   LogFloat c_jm,prob=0;
   LogDouble x,initx = LZERO;
   double Lr,steSumLr;
   float tmp;
   StreamElem *ste;
   MixtureElem *me;
   MixPDF *mp=NULL;
   WtAcc *wa, *wammi=NULL;
   PreComp *pMix;
   Boolean mmix=FALSE;  /* TRUE if multiple mixture */
   float wght=0;
   float mee_acc_scale =   fbInfo->AccScale * (fbInfo->MPE? fbInfo->aInfo->ac[q].mpe_occscale: 1 ),
      abs_mee_acc_scale = fabs(mee_acc_scale); int local_accindx = (mee_acc_scale > 0 ? fbInfo->num_index : fbInfo->den_index);

   float local_probscale;

   local_probscale = probScale;  
   N = hmm->numStates;
   for (j=2;j<N;j++) {
      outprob = ac->otprob[t];
      if(aqt[j]+gqt[j]-outprob[j][0][0]/*-pr*/ > MINEARG){
         ans  = LAdd(ans, aqt[j]+gqt[j]-outprob[j][0][0]); /*the total occ.*/
      }

      if(aqt[j]+gqt[j]-outprob[j][0][0] < -minFrwdP) continue; /*go to the next state.  This added for the tiedhs case,
                                                              to speed things up.*/

      initx = aqt[j]+gqt[j]-(outprob[j][0][0] * (1 + 1 / local_probscale)); 
      /*the outprob[j][0][0]*1 is to
        get the correct occupancy (since both alpha and gamma contain this states outprob); 
        and the  outprob[j][0][0]*1/probScale is to subtract the (unscaled) prob
        so we get the 'initx', which is the x without this timeframe's prob.*/
     
      /*Initx is the occupancy for this mixture, divided by the (unscaled) output prob for this state's PDF.*/
     
      ste = hmm->svec[j].info->pdf+1;

      for (s=1;s<=fbInfo->S;s++,ste++){
         /* Get observation vector for this state/stream */
         vSize = fbInfo->hset->swidth[s];

         switch (fbInfo->hsKind){
         case TIEDHS:		          /* if tied mixtures then we only */
            tmRec = &(fbInfo->hset->tmRecs[s]);   /* want to process the non-pruned */
            M = tmRec->topM;	          /* components */
            mmix = TRUE;
            break;
         case DISCRETEHS:
            M = 1;
            mmix = FALSE;
            break;
         case PLAINHS:
         case SHAREDHS:
            M = ste->nMix;
            mmix = (M>1);
            break;
         }
       
         wa = ((WtAcc*)ste->hook) + local_accindx;
         if(DoingFourthAcc) wammi = ((WtAcc*)ste->hook) + add_index;   
         steSumLr = 0.0;      /*  zero stream occupation count */
       
       
         /* process mixtures */
         for (mx=1;mx<=M;mx++) {
            switch (fbInfo->hsKind){	/* Get wght and mpdf */
            case TIEDHS:
               m=tmRec->probs[mx].index;
               wght=MixWeight(fbInfo->hset,ste->spdf.tpdf[m]);
               mp=tmRec->mixes[m];
               break;
            case DISCRETEHS:
               m=fbInfo->twoDataFiles ? fbInfo->up_ot.vq[s] : fbInfo->al_ot.vq[s];
               wght = 1.0;
               mp=NULL;
               break;
            case PLAINHS:
            case SHAREDHS:
               m = mx;
               me = ste->spdf.cpdf+m;
               wght = MixWeight(fbInfo->hset,me->weight);
               mp=me->mpdf;
               break;
            }
            if (wght>MINMIX){        /* For this mixture m, if the weight is nonzero... */
               /* compute mixture likelihood */
               if (!mmix || (fbInfo->hsKind==DISCRETEHS)){       /*    Don't need the MOutP for 1-mix systems. */
                  x = aqt[j]+gqt[j]-outprob[j][0][0]/*-pr*/;   
                  pMix = (PreComp *)mp->hook;
                  if(pMix->time != t+StartTime){ /* set the indx to -1, this relates to caching of the mixture occupation
                                                    probability on each time frame. */
                     pMix->time = t+StartTime; 
#ifdef MIX_UPDATE_SHARING
                     pMix->indx = -1;
#endif
                  } 
               } else { /*   Multiple-mixture --> need to work out the occupation probabilty of the individual mixture. */
                  c_jm=log(wght);
                  x = initx+c_jm;
                  switch(fbInfo->hsKind){
                  case TIEDHS :
                     tmp = tmRec->probs[m].prob;
                     prob = (tmp>=MINLARG)?(log(tmp)+tmRec->maxP):LZERO; /*maxP is a normalising factor subtracted from all the TMProbs.*/
                     break;                                            /*"prob" now contains the TM prob [not the mixture weight as well.]*/
                  case SHAREDHS : case PLAINHS:
		     if (fbInfo->S==1)
		        prob = outprob[j][0][mx];
		     else
		        prob = outprob[j][s][mx];
		     pMix = (PreComp *)mp->hook;
		     if(pMix->time != t+StartTime){ /* set the indx to -1, this relates to caching of the mixture occupation
						       probability on each time frame. */
		        pMix->time = t+StartTime; 
#ifdef MIX_UPDATE_SHARING
			pMix->indx = -1;
#endif
		     } 
                     break;
                  default:
                     HError(1, "Unknown hsKind.");
                     break;
                  }
                  x += prob;
                  if (fbInfo->S>1)	/* adjust for parallel streams */
                     x += outprob[j][s][0]/probScale; /* all the other streams... */
               }
	   
               /* If transforms are used, x is the prob from the *alignment* transform, not the update transform. */
	   
               if (-x<minFrwdP) {
                  ans2=LAdd(ans2, x);
                  Lr = exp(x);
             
                  /* More diagnostics */
                  if(isnan(Lr) || Lr > 1.001)
                     HError(1, "FwdBkwdLat: UpMixParms: invalid Lr.");
                  if (Lr>1.001)
                     HError(-999,"UpMix: Lr too big %f ", Lr);
	     
                  steSumLr += Lr;
	     
                  DoMixUpdate(mp, s, Lr, mee_acc_scale, t); /* This now does not actually update the mixture, but just notes down
                                                               the probability for later updating with "DoAllMixUpdates", which is called
                                                               once every time frame. */
                  /* ------------------ update mixture weight counts ----------------- */
                  if (fbInfo->uFlags&UPMIXES) {
                     wa->c[m] += Lr * abs_mee_acc_scale;
                     if(DoingFourthAcc) wammi->c[m] += Lr;
                  }
               } 
               /*   printf("q=%d, N=%d,j=%d, M=%d, m=%d, x=%f, prob=%f,stocc=%f\n", q,N,j,M,m,x,prob,aqt[j]+gqt[j]-outprob[j][0][0]); */
            }
         }
   
         wa = ((WtAcc*)ste->hook) + local_accindx;
         wa->occ += steSumLr * abs_mee_acc_scale;
         if(DoingFourthAcc){   /* do 4th acc if MPE with MMI prior */            
            wammi = ((WtAcc*)ste->hook) + add_index;
            wammi->occ += steSumLr;
         }
      }
   }
   return ans;
}



/* -------------------- Top Level of F-B Updating ---------------- */

/* CheckData: check data file consistent with HMM definition */
static void CheckData(char *fn, BufferInfo *info) 
{
   if (info->tgtVecSize!=fbInfo->hset->vecSize)
      HError(2350,"CheckData: Vector size in %s[%d] is incompatible with hset [%d]",
             fn,info->tgtVecSize,fbInfo->hset->vecSize);
   if (!fbInfo->twoDataFiles){
      if (info->tgtPK != fbInfo->hset->pkind)
         HError(2350,"CheckData: Parameterisation in %s is incompatible with hset ",
                fn);
   }
}



/* StepForward: Step from 1 to T calc'ing Alpha columns and updating parms */
static void StepForward()
{
   int q,t,negs;
   DVector aqt,aqt1,bqt,bqt1,tmp;
   double occ, total_occ;
   HLink hmm, up_hmm;
   ResetObsCache();
   ZeroAlpha(1, fbInfo->Q); /*Zero the alphat column,*/
   for(q=1;q<=fbInfo->Q;q++){ /*And switch: now the alphat1 column is zero.*/
      Acoustic *ac = fbInfo->aInfo->ac + q;
      tmp=ac->alphat;ac->alphat=ac->alphat1;ac->alphat1=tmp;
   }
   ZeroAlpha(1, fbInfo->Q); /*Now the alphat column is zero too.*/
  
   for (q=1;q<=fbInfo->Q;q++){  /* inc access counters */
      up_hmm = fbInfo->aInfo->ac[q].hmm;
      negs = (int)up_hmm->hook+1;
      up_hmm->hook = (void *)negs;
   }

   for (t=1;t<=fbInfo->T;t++) {
      /* Get Data */
      ReadAsTable(fbInfo->al_pbuf,t-1,&fbInfo->al_ot);

      if (fbInfo->twoDataFiles)		
         ReadAsTable(fbInfo->up_pbuf,t-1,&fbInfo->up_ot);

      if (fbInfo->hsKind == TIEDHS)  PrecomputeTMix(fbInfo->hset,&fbInfo->al_ot,minFrwdP,0);

      StepAlpha(t); /* Calculate this time's Alpha column. */

      /* Now accumulate statistics. */
      total_occ=LZERO;
      for (q=fbInfo->aInfo->qLo[t];q<=fbInfo->aInfo->qHi[t];q++){ /* inc accs for each active model */
         Acoustic *ac = fbInfo->aInfo->ac+q;
         int tLo = ac->t_start,
            tHi = ac->t_end;
         if(t==tLo && tHi==tLo-1 && fbInfo->uFlags&UPTRANS){ /*In the ExactMatch case, where we have a skip transition.*/
            UpSkipTranParms(q, t);
         }
         if(t>=tLo&&t<=tHi){
            hmm = ac->hmm; 
            aqt = ac->alphat; bqt = ac->betaPlus[t];
	
            bqt1 = (t+1>=tLo&&t+1<=tHi) ? ac->betaPlus[t+1]:NULL;
            aqt1 = (t==1) ? NULL:ac->alphat1; /* alpha from t-1 */

	if (fbInfo->uFlags&(UPMEANS|UPVARS|UPMIXES|UPXFORM|UPMIXES))
	  if((occ=UpMixParms(q,hmm,t,aqt,aqt1,bqt)) > LSMALL){
	    total_occ = LAdd(total_occ, occ);
	  }
	if (fbInfo->uFlags&UPTRANS)
	  UpTranParms(t,q);
      }
    }
    DoAllMixUpdates(t);  /* Iterates over all active mpdf's and actually accumulates stats. */
  
    if(fabs(total_occ) > 0.1)
      HError( 1, "in HFwdBkwdLat.c: Wrong occ: exp(%f)\n",total_occ);
    if(fabs(total_occ) > 1.0e-4)
      HError( -1, "in HFwdBkwdLat.c: Wrong occ: exp(%f)\n",total_occ);
  }
}


    
/* -------------------------- Top Level Forward-Backward Routine ------------------------ */



static  char buf1[255];
static  Boolean eSep;

 
void GetTimes(LArc *larc, int i, int *start, int *end){ /* get start & end times for a lattice arc.  Frame
                                                           duration is afrom the aInfo structure which is usually initialised
                                                           to 0.1 or by config HARC:FRAMEDUR */
   float s = larc->start->time,e; int j;
   float framedur = fbInfo->aInfo->framedur;

   if(!Quinphone){
      for(j=0;j<i;j++) s += larc->lAlign[j].dur;
      e = s + larc->lAlign[i].dur;
   } else { /* assume is a real quinphone, hence 3 states ... */
      for(j=0;j<i;j++) s += larc->lAlign[j].dur;
      e=s;
      for(j=i;j<i+3;j++) e += larc->lAlign[j].dur;    
   }
   *start = (int)(s/(framedur) + 1.5);
   *end = (int)(e/(framedur) + 0.5);
}

void FBLatAddLattice (FBLatInfo *fbInfo, Lattice *lat){  /* add this lattice, 
							    can do this repeatedly. */

   fbInfo->aInfo->lat[ fbInfo->aInfo->nLats ++ ] = lat;
   if(!lat) HError(1, "Zero lattice supplied to FBLat.");
   if(fbInfo->aInfo->nLats > MAXLATS) HError(1,"fbInfo->aInfo->nLats > MAXLATS, increase array size MAXLATS.");


}
void FBLatSetAccScale(FBLatInfo *fbInfo, float AccScale){ /*scale accumulators by this amount. */
   fbInfo->AccScale = AccScale;  
}

int MPE_GetFileLen(Lattice *lat){
   /*  find the true length of the file. (Just for diagnostics)  */
   int len; LNode *node; 
  
   if(PhoneMEE) 
      for(node=lat->lnodes+0,len=0; node->foll; node=node->foll->end){
         int i;
         for(i=0;i<node->foll->nAlign;i++){ 
            LabId lab = node->foll->lAlign[i].label;
            if( !IsSilence(lab->name) ){
#ifdef SUPPORT_QUINPHONE
               int nstates,state;
               if(Quinphone){
                  GetNoContextPhone(lab,&nstates,&state,NULL,NULL);
                  if(nstates==1)  len++;
                  else if(state==2) len++;
               } else len++;
#else
               len++;
#endif
            }
         }
      }
   else
      for(node=lat->lnodes+0,len=0; node->foll; node=node->foll->end) 
         if(node->foll->end->word->wordName && !IsStartOrEnd(node->foll->end->word->wordName->name))
            len++;
   return len;
}




void FBLatClearUp(FBLatInfo *fbInfo); 

void FBLatFirstPass(FBLatInfo *_fbInfo, FileFormat dff, char * datafn, char *datafn2, Lattice *MPECorrLat){
   int q,T2=0; Boolean MPE;
  
   fbInfo = _fbInfo;
   if(fbInfo->InUse) FBLatClearUp(fbInfo); 
   fbInfo->InUse=TRUE; /* will now initialise */
  
   if (trace&T_TOP) {
      printf(" Processing Data: %s\n", NameOf(datafn,buf1));
      fflush(stdout);
   }
   MPE = fbInfo->MPE = (MPECorrLat!=NULL);
  
   ArcFromLat(fbInfo->aInfo, fbInfo->hset);
   if(MPE) AttachMPEInfo(fbInfo->aInfo);
  
   /*[trace:] PrintArcInfo(stdout, &fbInfo->aInfo);*/
   fbInfo->Q = fbInfo->aInfo->Q;
   if (fbInfo->Q==0)
      HError(2325,"FBLat: No arcs in lattice for %s",datafn);
  
   if (fbInfo->twoDataFiles)
      SetNewConfig("HPARM1");
   fbInfo->al_pbuf=OpenBuffer(&fbInfo->al_dataStack,datafn,0,dff,FALSE_dup,FALSE_dup);
   GetBufferInfo(fbInfo->al_pbuf,&fbInfo->al_info);
   if (fbInfo->twoDataFiles){
      if(!datafn2) HError(1, "Need 2 data file names if single pass retraining.");
      SetNewConfig("HPARM2");
      fbInfo->up_pbuf=OpenBuffer(&fbInfo->up_dataStack,datafn2,0,dff,FALSE_dup,FALSE_dup);
      GetBufferInfo(fbInfo->up_pbuf,&fbInfo->up_info);
      CheckData(datafn2,&fbInfo->up_info);
      /*      SyncBuffers(pbuf,pbuf2); */
      T2 = ObsInBuffer(fbInfo->up_pbuf);
   }else
      CheckData(datafn,&fbInfo->al_info);
   fbInfo->T = ObsInBuffer(fbInfo->al_pbuf);
  
   if (fbInfo->twoDataFiles && (fbInfo->T != T2))
      HError(2319,"HERest: Paired training files must be same length for single pass retraining");
  
   if (fbInfo->firstTime){
      SetStreamWidths(fbInfo->al_info.tgtPK,fbInfo->al_info.tgtVecSize,fbInfo->hset->swidth,&eSep);
    
      fbInfo->al_ot = MakeObservation(&fbInfo->miscStack,fbInfo->hset->swidth,fbInfo->al_info.tgtPK,
                                      fbInfo->hsKind==DISCRETEHS,eSep);

      if (fbInfo->twoDataFiles){ /*todo, fix use of this. */
         fbInfo->up_ot = MakeObservation(&fbInfo->miscStack,fbInfo->hset->swidth,fbInfo->up_info.tgtPK,
                                         fbInfo->hsKind==DISCRETEHS,eSep);
      }
      fbInfo->firstTime = FALSE;
   }
  
  
   SetBetaPlus(); /* Step back through file. */
  
   {
      HArc *a; ArcTrans *at; LogFloat lmprob;
      fbInfo->pr = LZERO;
    
      /* Calculate beta [actually betaPlus, like a reversed alpha] */
      for(q=1;q<=fbInfo->Q;q++){  
         Acoustic *ac = fbInfo->aInfo->ac+q;
         ac->locc = LZERO;
         if(ac->aclike == LZERO) HError(1, "Zero acoustic likelihood! (May be due to different model topology than used to create lattice)");
      }
       
      for(a=fbInfo->aInfo->end;a;a=a->prec){ /*Calculate betaPlus .*/
         Acoustic *ac = a->ac;
         LogDouble betaPlus;
         if(!a->follTrans) betaPlus = 0;
         else{	  
            betaPlus=LZERO;
            for(at=a->follTrans;at;at=at->start_foll){
               lmprob = translm(at->lmlike);
               betaPlus = LAdd(betaPlus, at->end->betaPlus + lmprob);
            }
         }
         betaPlus += ac->aclike * latProbScale;
         if(isnan(betaPlus)) HError(1, "betaPlus isnan...");
         a->betaPlus = betaPlus;
         if(!a->precTrans) fbInfo->pr = LAdd(fbInfo->pr, a->betaPlus);
      }
    
      /*Now calculate alpha... The occupancy of an arc is its exp((alpha+betaPlus-outprob)-fbInfo->pr) */
      for(a=fbInfo->aInfo->start;a;a=a->foll){
         Acoustic *ac = a->ac;
         LogDouble alpha, occ;
         if(!a->precTrans) alpha = 0;
         else{	  
            alpha=LZERO;
            for(at=a->precTrans;at;at=at->end_foll){
               lmprob = translm(at->lmlike);
               alpha = LAdd(alpha, at->start->alpha + lmprob);
            }
         }
         alpha += ac->aclike * latProbScale;
         a->alpha = alpha;
         occ = a->alpha + a->betaPlus - ac->aclike*latProbScale- fbInfo->pr; 
      
         /*occ should be log of sth in the region 0.0 ... 1.0 */
         if(occ > 0.0001) HError(1, "occ > 1.0001 (%f)", occ);
      
         /* alpha_startprob[id] = LAdd(alpha_startprob[id], -gamma_startprob[id] + occ); */
         ac->locc = LAdd(ac->locc, occ);
      }

      /* calculate & check overall probability of file */
      { 
         double pr = LZERO;
         for(a=fbInfo->aInfo->start;a;a=a->foll){
            if(!a->follTrans) /* an ending arc. */
               pr = LAdd(pr, a->alpha);
         }
         if(fabs(pr-fbInfo->pr)>1.0e-6) HError(1, "Error in file pr...");
      }
    
     
      if(MPE){  
         int nTrans1=0,nTrans2=0; /*debug*/
         HArc *a; ArcTrans *at; double norm;
         /*We are going to set the "alphaError" and "betaPlusError" fields for each of the arcs."*/


         if(!MPECorrLat) HError(-1, "No num lattice provided for MPE! ");        
         fbInfo->MPEFileLength = MPE_GetFileLen(MPECorrLat);

#ifdef SUPPORT_EXACT_CORRECTNESS   
         if(ExactCorrectness){
            DoExactCorrectness(fbInfo, MPECorrLat);
         } else {
#endif
            /* Inexact version of MPE/MWE. */
	   if(CalcAsError)  SetCorrectnessAsError(fbInfo, MPECorrLat); /* first set 'correctness' field for each arc, a value between -1 and 1. */
	   else  SetCorrectness(fbInfo, MPECorrLat); /* first set 'correctness' field for each arc, a value between -1 and 1. */
         
            /*First set alphaError*/
            for(a=fbInfo->aInfo->start;a;a=a->foll){ 
               double alphaError;
               if(!StartOfWord(a)) a->mpe->alphaError=a->precTrans->start->mpe->alphaError + a->mpe->correctness;
               else{ /*start of word.  Sum previous alphas.*/
                  if(!a->precTrans){ alphaError = 0.0;  } /*start of sentence*/
                  else{
                     norm = LZERO; alphaError = 0.0;
                     for(at=a->precTrans;at;at=at->end_foll){ 
                        nTrans1++;
                        norm = LAdd(norm, at->start->alpha + translm(at->lmlike));
                     }
                     for(at=a->precTrans;at;at=at->end_foll){
                        alphaError += (at->start->alpha+translm(at->lmlike)-norm > MINEARG ? exp(at->start->alpha+translm(at->lmlike)-norm):0.0)*(at->start->mpe->alphaError);
                     }
                  }
                  alphaError +=  a->mpe->correctness; /*-1 .. 1*/
                  a->mpe->alphaError = alphaError;
               }
            }
         
            /*now set betaPlusError (like betaError but includes present phone's contribution). */
            for(a=fbInfo->aInfo->end;a;a=a->prec){
               double betaPlusError;
           
               /* current phone's contribution to correctness:*/
               betaPlusError = a->mpe->correctness; /*if Word MEE this will only be nonzero in the start-of-word case. */
               if(EndOfWord(a)){ /* --> multiple transitions from end of arc. */
                  if(!a->follTrans){ betaPlusError += 0.0; } /*end of sentence*/
                  else{
                     norm = LZERO; 
                     for(at=a->follTrans;at;at=at->start_foll){ /* Get normalising factor = sum following gamma...*/
                        nTrans2++;
                        norm = LAdd(norm, at->end->betaPlus + translm(at->lmlike));
                     }
                     for(at=a->follTrans;at;at=at->start_foll){
                        if(at->end->betaPlus+translm(at->lmlike) -norm > MINEARG)
                           betaPlusError += exp(at->end->betaPlus + translm(at->lmlike) - norm)*(at->end->mpe->betaPlusError);
                     }
                  }
               }  else  betaPlusError += a->follTrans->end->mpe->betaPlusError; 
               a->mpe->betaPlusError = betaPlusError;
            }
         
            if(nTrans1!=nTrans2) HError(-1, "NTrans1!=NTrans2"); /*maybe should be +1 error */

            /*Find the average correctness for the whole file, and do some checking...*/
            {
               double  avgErrBetaPlus=0, avgErrAlpha=0, normBetaPlus=0, normAlpha=0;
               for(a=fbInfo->aInfo->start;a;a=a->foll){
                  if(!a->precTrans){ /*start of file*/
                     if(a->betaPlus-fbInfo->pr > MINEARG){
                        avgErrBetaPlus += exp(a->betaPlus-fbInfo->pr) * a->mpe->betaPlusError;
                        normBetaPlus +=  exp(a->betaPlus-fbInfo->pr);
                     }
                  } 
                  if(!a->follTrans){ /*end of file*/
                     if(a->alpha-fbInfo->pr > MINEARG){
                        avgErrAlpha += exp(a->alpha-fbInfo->pr) * a->mpe->alphaError;
                        normAlpha +=  exp(a->alpha-fbInfo->pr);
                     }
                  }
                  /* printf("Error for %s = %f/%f/%f   ", a->phone->name, a->mpe->alphaError, a->mpe->betaPlusError, a->mpe->alphaError+a->mpe->betaPlusError); */
               }

               if(fabs(1-normBetaPlus)>0.1 || fabs(1-normAlpha)>0.1)    /*Sanity check*/
                  HError(1, "normAlpha/normBetaPlus wrong!");
               if(fabs(avgErrBetaPlus - avgErrAlpha) > 0.0001)         /*Sanity check*/
                  HError(-1, "avgErrBetaPlus and avgErrAlpha disagree (%f,%f)!", avgErrBetaPlus, avgErrAlpha);
               fbInfo->AvgCorr = avgErrBetaPlus;    /* set average correctness of file. */
            }
	   
            { 
               /*  Calculate the scales gamma_q^MPE = (AvgErr(node) - AvgErr(file)) for each arc.  */
               /*  Since the calculation is pooled for identical arcs in pools q=1..Q, this is actually
                   a weighted average of gamma_q^MPE over all the identically timed arcs, weighted by occupation
                   probability gamma_q. */

               for(q=1;q<=fbInfo->Q;q++) fbInfo->aInfo->ac[q].mpe_occscale = 0; 

               for(a=fbInfo->aInfo->start;a;a=a->foll){
                  Acoustic *ac = a->ac;
                  float occ = a->alpha + a->betaPlus - ac->aclike*latProbScale - fbInfo->pr;   /* Occupancy of this cluster of identical arcs which is due to a. */
                  float total_occ = ac->locc;              /* total occupancy of this group of arcs */
                  if(occ-total_occ > MINEARG){
                     ac->mpe_occscale += exp(occ-total_occ) * (a->mpe->alphaError+a->mpe->betaPlusError-fbInfo->AvgCorr - a->mpe->correctness);
                     /* MEEScale is an average error for words including that arc, minus the global average error. */
                     /* - a->mpe->correctness has to be subtracted because alpha and betaPlus both contain it.*/
                  }
               }
            }
         }   /* endif (!ExactCorrectness) */
	    
         {
#ifdef DEBUG_MEE
            printf("\nMEEScale = ");
            for(i=1;i<=fbInfo->Q;i++) printf("%f ", fbInfo->aInfo->ac[i].mpe_occscale);
         
            printf("\nMEEScale*fbscale = ");
            for(i=1;i<=fbInfo->Q;i++) { 
               Acoustic *ac = fbInfo->aInfo->ac+i;
               printf("%f ", ac->mpe_occscale  * exp (ac->locc));
            }
            printf("\n");
#endif
         }

	 if(CalcAsError) fbInfo->AvgCorr += fbInfo->MPEFileLength;       
         if(trace&T_TOP) printf("FLen=%d, AvCor=%f\n", fbInfo->MPEFileLength, fbInfo->AvgCorr); /*normal case.*/

      } /*endif ( MPE ) */
   }
   if(trace&T_TOP) printf("T=%d, pr/fr=%f\n", fbInfo->T, fbInfo->pr/fbInfo->T);

}


void FBLatSecondPass(FBLatInfo *_fbInfo, int num_index, int den_index){
   fbInfo = _fbInfo;
   fbInfo->num_index = num_index; fbInfo->den_index = den_index;


   if(fbInfo->pr == 0) HError(1, "FBLatSecondPass: 1st pass not done!!");
   StepForward();

   FBLatClearUp(fbInfo);
   StartTime += fbInfo->T; /*relates to caching of likelihoods */

}




/* ------------------------------------ Initialisation ------------------------------------ */

/* EXPORT->InitFwdBkwdLat: initialise configuration parameters */
void InitFBLat(void)
{
   int iter,i;
   double f;
   Boolean b;

   Register(hfblat_version,hfblat_vc_id);

   for(iter=0;iter<=1;iter++){
      if(iter) nParm = GetConfig("HFBLAT", TRUE, cParm, MAXGLOBS);
      else nParm = GetConfig("HFWDBKWDBLAT", TRUE, cParm, MAXGLOBS); /* Backward compatibility */

      if (nParm>0){
         if (GetConfInt(cParm,nParm,"TRACE",&i)) trace = i;
         if (GetConfFlt(cParm,nParm,"MINFORPROB",&f))  minFrwdP = f;
         if (GetConfFlt(cParm,nParm,"PROBSCALE",&f))  probScale = f;
         if (GetConfFlt(cParm,nParm,"LANGPROBSCALE",&f))  langProbScale = f;
         if (GetConfFlt(cParm,nParm,"LATPROBSCALE",&f))  latProbScale = f; /* this config also used in HExactMPE.c */
         if (GetConfFlt(cParm,nParm,"PHNINSPEN",&f))  phnInsPen = f; /* this config also used in HExactMPE.c */
         
         if (GetConfBool(cParm,nParm,"NOSILENCE",&b)) NoSilence = b;
#ifdef SUPPORT_QUINPHONE
         if (GetConfBool(cParm,nParm,"QUINPHONE",&b)) Quinphone = b;/* this config also used in HExactMPE.c */
#endif
#ifdef  SUPPORT_EXACT_CORRECTNESS
         if (GetConfBool(cParm,nParm,"EXACTCORRECTNESS",&b)) ExactCorrectness = b;
#endif
         if (GetConfBool(cParm,nParm,"PHONEMEE",&b)) PhoneMEE=b; /* Back-compat. this config also used in HExactMPE.c */
         if (GetConfBool(cParm,nParm,"CALCASERROR",&b)) CalcAsError = b; 
         if (GetConfBool(cParm,nParm,"MWE",&b)) if(b) PhoneMEE=FALSE; /* this config also used in HExactMPE.c */
         if (GetConfBool(cParm,nParm,"MEECONTEXT",&b)) PhoneMEEUseContext=b; /*will prob. want to set
                                                                               this false. */ /*back-compat*/
         if (GetConfBool(cParm,nParm,"USECONTEXT",&b)) PhoneMEEUseContext=b; /*will prob. want to set
                                                                               this false. */
         if (GetConfFlt(cParm,nParm,"INSCORRECTNESS",&f)) InsCorrectness=-fabs(f); /* to make sure negative.*/
         /* this config also used in HFBExactMPE.c */
      }
   }
   SET_totalProbScale;
}



/* EXPORT-> InitialiseFB Sets up heaps etc for Forward-Backwards */

void InitialiseFBInfo(FBLatInfo *fbInfo,
		     HMMSet *hset, 
		      UPDSet uflags, 
                      Boolean twoDataFiles)
{
   int s;

   /* Stacks for global structures requiring memory allocation */
   CreateHeap(&fbInfo->arcStack,    "fbLatArcStore",       MSTAK, 1, 1.0, 1000000,  20000000);
   CreateHeap(&fbInfo->tempStack,   "fbLatTempStore",       MSTAK, 1, 0.5, 1000,  10000);
   CreateHeap(&fbInfo->al_dataStack,    "fbLatDataStore",     MSTAK, 1, 0.5, 1000,  10000);
   CreateHeap(&fbInfo->miscStack,    "fbLatMiscStore",     MSTAK, 1, 0.5, 1000,  10000);
   if (fbInfo->twoDataFiles)
      CreateHeap(&fbInfo->up_dataStack,   "fbLatDataStore2",  MSTAK, 1, 0.5, 1000,  10000);

   fbInfo->hset = hset;
   fbInfo->hsKind = hset->hsKind;      
   fbInfo->S = hset->swidth[0];
   fbInfo->uFlags = uflags;            
   
   fbInfo->firstTime=TRUE;
   fbInfo->twoDataFiles = twoDataFiles;
   fbInfo->AccScale = 1.0; /* A scale on the occupancies; note that it should be
			      +ve otherwise there may be unexpected results in the MMI case. */
   
   fbInfo->aInfo = New(&fbInfo->miscStack, sizeof(ArcInfo));
   fbInfo->aInfo->mem = &(fbInfo->arcStack);
   fbInfo->InUse = FALSE;
   fbInfo->aInfo->nLats = 0;
   if(!CachingInitialised){ /* Initialise the mix occupation-caching  stack. */
      CachingInitialised = TRUE;
      CreateHeap(&cacheMixoccHeap,    "cacheMixocc C heap",       CHEAP, 1, 0.5, 1000,  10000);
      for(s=0;s<SMAX;s++){
         nPDFs[s] = 0; SavedMixesSize[s]=0; SavedMixes[s]=0;
      }
   }
}



void FBLatClearUp(FBLatInfo *fbInfo){  
   ResetHeap(&fbInfo->arcStack);
   fbInfo->aInfo->mem = &fbInfo->arcStack;  
   fbInfo->aInfo->nLats = 0; /* important. */
   CloseBuffer(fbInfo->al_pbuf);  
   ResetHeap(&fbInfo->up_dataStack);
   if (fbInfo->twoDataFiles){
      CloseBuffer(fbInfo->up_pbuf);
      ResetHeap(&fbInfo->up_dataStack);
   }
   ResetHeap(&fbInfo->tempStack);
   fbInfo->InUse=FALSE; 
}


/* Function to support MPE with MMIPrior */
/* EXPORT-> SetDoingFourthAcc: Indicate whether it is currently storing MMI statistics */
void SetDoingFourthAcc(Boolean DO, int indx){
   DoingFourthAcc = DO;
   add_index = indx;
}


/* --------------------------------- End HFBLat.c -------------------------------------- */


