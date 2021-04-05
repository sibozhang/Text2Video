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
/*         File: HExactMPE.c   Discriminative training         */
/* ----------------------------------------------------------- */

char *hexactmpe_version = "!HVER!HExactMPE:   3.4.1 [CUED 12/03/09]";
char *hexactmpe_vc_id = "$Id: HExactMPE.c,v 1.1.1.1 2006/10/11 09:54:57 jal58 Exp $";

/*
    Performs forward/backward alignment
*/


/* Not working: skip transitions just after the first 'sil' are not handled [but don't occur],
  [probably] alignment hmm's. */

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


static ConfParam *cParm[MAXGLOBS];  /* config parameters */
static int nParm = 0;



static int PHONE_BEAM=4; /* phones before & after...*/
static float EXACTCORR_PRUNE= -8.5 /*0.0002*/; 

static float latProbScale = 1.0; /* repeat of config also used in HFBLat.c */
static Boolean PhoneMEE = TRUE;  /* repeat of config also used in HFBLat.c */
#ifdef SUPPORT_QUINPHONE
static Boolean Quinphone = FALSE; /* repeat of config also used in HFBLat.c */
#endif
static float phnInsPen = 0.0;    /* repeat of config also used in HFBLat.c */
static float InsCorrectness = -1;   /* repeat of config also used in HFBLat.c */
static float SubCorrectness = -1;   /* repeat of config also used in HFBLat.c */

extern float hfwdbkwd_totalProbScale;    

#define translm(lmlike) ((lmlike*hfwdbkwd_totalProbScale)+phnInsPen)   /* this define also in HFBLat.c */
       
static int debug_bestcorr = 1000;



#define MIN(x,y)  (x<y?(x):(y))
#define MAX(x,y)  (x>y?(x):(y))

/*--- These defines  are also in HFBLat.c ---- */
#define StartOfWord(a) (a->pos==0) 
#define EndOfWord(a) (a->pos == a->parentLarc->nAlign-1)  
#ifdef SUPPORT_QUINPHONE
#define IsSilence(name)  ( Quinphone ? (!strncmp(name,"sil",3)||!strncmp(name,"sp",2)) : (!strcmp(name,"sil")||!strcmp(name,"sp")))   
   	       /*strncmp not strcmp, so as to handle sil_0,sp_0 for quinphone. */
#else
#define IsSilence(name)  (!strcmp(name,"sil")||strcmp(name,"sp"))  
#endif
#define IsStartOrEnd(word) (*word=='!' || *word == '<')
/*-------------------------------------------- */



#ifdef SUPPORT_EXACT_CORRECTNESS /* if not defined, this whole file is really useless. */



Boolean IsNonSilArc(LArc *larc){ /*returns TRUE if this word is non SENT_START etc. */
  return (larc->nAlign>1 || (larc->nAlign==1 && !IsSilence(larc->lAlign[0].label->name)));
}
int GetNumPhones(LArc *larc){ /*returns num phones in a word...*/
  if(!Quinphone){
    int nAlign = larc->nAlign;
    if(IsSilence(larc->lAlign[nAlign-1].label->name)) return nAlign-1;
    else return nAlign;
  } else {
    int nAlign = larc->nAlign;
    int i = nAlign/3;
    if(nAlign != (i*3)+1) HError(1, "Problem with quinphone code...");
    return i;
  }
}
Boolean NonSil_and_Quinphone_IsStartPhone(LArc *larc, int i){
  if(!Quinphone){ 
    return (i < larc->nAlign-1 || !IsSilence(larc->lAlign[larc->nAlign-1].label->name)); /* not end phone or end phone is non-sil. */
  }
  else{
    return (i % 3 == 0 && i != larc->nAlign-1); /* assuming each has 3 states and a terminal sil/sp [This could be wrong later, but just for simplicity
						   do it this way now.....]*/
    
  }
}



void AddTrans(MemHeap *x, CorrN *start, CorrN *end, float sc_lmlike){
  CorrA *a = New(x, sizeof(CorrA));
  a->start=start;a->end=end;
  a->start_foll = start->follTrans; start->follTrans = a;
  a->end_foll = end->precTrans; end->precTrans = a;
  a->sc_lmlike = sc_lmlike;
}


Boolean GetBestCorrectness /*step correctness by 1 phone.*/
                        (float *_BestCorr, float *_BestCorrPart, int *_bestj, int i,
                        CorrN *cn_prev,
                        short int *minn_of_t,    /* lowest sausage position active at time t. */
                        short int *maxn_of_t,    /* highest ..*/
                        short int *niphones,     /* num alternative sausage positions */
                        int **iphone,            /* phone [0..N-1][0..niphones[n]-1] */
                        unsigned char *nonempty, /* if TRUE then no null transition at that sausage position */
                        int T, int N,   
                         float InsCorrectness){  /* Returns if there *is* some nonzero-likelihood option. */
  int j;
  float corr,corrPart=-10000; /*current contribution to correctness [init probably not needed]*/
  float BestCorr = -10000; float BestCorrPart=0; int bestj=-1;
  int iphone_prev = cn_prev->iphone;
  int start= cn_prev->starti, end =  cn_prev->endi;

  if(!cn_prev->IsSilence){
    Boolean CorrFullPhone = FALSE; /* counted against insertions. */
    Boolean CorrPartialPhone = FALSE; /* not counted against insertions. */
    int nDel = 0;
    
    /* First see if same phone between "end" and i-1, if any.*/
    for(j=end+1;j<=i-1;j++){ /*probably empty loop. */
      int p;
      Boolean Same=FALSE;
      for(p=0;p<niphones[j];p++){
        int this_iphone = iphone[j][p];
        if(this_iphone==iphone_prev) Same = TRUE;
      }
      if(nonempty[j]){
        if(Same) CorrFullPhone = TRUE; 
        nDel++;
      } else {
        if(Same) CorrPartialPhone = TRUE; 
      }
    }
    for(j=MIN(i-1,end);j>=start;j--){ /* substitutions or correct phones. */
      int p;
      Boolean Same=FALSE; 
      for(p=0;p<niphones[j];p++){
        int this_iphone = iphone[j][p];
        if(this_iphone==iphone_prev) Same = TRUE;
      }
      if(nonempty[j]){
        if(Same) CorrFullPhone = TRUE; 
        nDel++;
      } else {
        if(Same)  CorrPartialPhone = TRUE; 
      }
      if(CorrFullPhone)
        corrPart = (1-nDel);
      else{
        if(nDel > 0)
           if(CorrPartialPhone)
              corrPart = MAX(-nDel, (1-nDel) + SubCorrectness);   /* It's either (nDel-1) dels, 1 sub or (nDel) dels, 1 corr. [ins=del=sub=-1,corr=0] */
           else 
              corrPart = (1-nDel) + SubCorrectness;         /*  (nDel-1) dels, 1 sub */ 
        else {
          if(CorrPartialPhone) corrPart = 0; /* correct [with partial phone]. */
          else corrPart = InsCorrectness; /* InsCorrectness is e.g -1 or -0.75, label as insertion [rather than sub with partial phone]
                                             since this gives lower error rate as long as InsCorrectness>-1, which it should be */
        }
      }
      corr = cn_prev->alphaCorr[j] + corrPart;
      if(corr>BestCorr){ BestCorr=corr; bestj=j; BestCorrPart = corrPart; }
    }
    j=i; /*do the case of j=i, so this phone is an insertion. */
    if(j>=start && j<=end){
      corrPart = InsCorrectness;
      corr = cn_prev->alphaCorr[j] + corrPart;
      if(corr>BestCorr){ BestCorr=corr; bestj=j; BestCorrPart = corrPart; }
    }
  } else{ /* If silence, just duplicate previous [with possible deletions in case e-o-f problems...] */
    float corr;
    int nDel=0;
    for(j=end+1;j<=i-1;j++){ /*probably empty loop. */
      if(nonempty[j]) nDel++;
    }
    for(j=MIN(i-1,end);j>=start;j--){    /* Deletions... */
      if(nonempty[j]) nDel++;
      corrPart = -nDel; /*silence does not count as a phone so no subs/ins, just deletions... */
      corr = cn_prev->alphaCorr[j] + corrPart;
      if(corr>BestCorr){ BestCorr=corr; bestj=j; BestCorrPart = corrPart; }
    }
    
    j=i; /* Same time .... */
    if(j>=start&&j<=end){
      corrPart = 0;
      corr= cn_prev->alphaCorr[j] + corrPart;
      if(corr>BestCorr){ BestCorr=corr; bestj=j; BestCorrPart = corrPart; }
    }
  }


  if(_BestCorr) *_BestCorr = BestCorr;             /* final correctness */
  if(_BestCorrPart) *_BestCorrPart = BestCorrPart; /*contribution from this phone. */
  if(_bestj) *_bestj = bestj;                     /* sausage pos of best contribution */
  return(bestj!=-1);
}
float DoCorrectness(FBLatInfo *fbInfo, MemHeap *mem, ArcInfo *ai, float prune, 
                    int beamN/*phones on either side...*/,
		    short int *minn_of_t,    /* lowest sausage position active at time t. */
                    short int *maxn_of_t,    /* highest ..*/
                    short int *niphones,     /* num alternative sausage positions */
                    int **iphone,            /* phone [0..N-1][0..niphones[n]-1] */
		    unsigned char *nonempty, /* if TRUE then no null transition at that sausage position */
                    int T, int N, 
                    float InsCorrectness, Boolean Quinphone, float pr_in){ 
  HArc *a;  
  CorrN  *startNode = NULL, *endNode = NULL; 

  double local_pr=LZERO; double local_pr_beta=LZERO; 
  double avg_correct = 0; double avg_correct_beta = 0;
  CorrN *cn;


  for(a=ai->start;a;a=a->foll) a->mpe->cn = NULL;

  for(a=ai->start;a;a=a->foll){ /* This loop attaches the 'cn' structure to the lattice */
    float locc = a->alpha + a->betaPlus - fbInfo->pr - a->ac->aclike*latProbScale;

    if(locc > prune){   /* ... if above prune threshold then attach the 'cn' structure */
      if(!PhoneMEE && StartOfWord(a)/*expands to a->pos==0*/){  /* This is the MWE case. Create a cn structure for the first phone of the word. */
	LArc *la = a->parentLarc; 
	int iword = (int)/*from LabId*/ la->end->word->wordName;
	int id = (a->calcArc ? a->calcArc->id : a->id);
	HArc *b,*lastArc; int x;

	cn = New(mem, sizeof(CorrN));
	cn->next = NULL; 
	if(endNode){ endNode->next=cn; cn->prev=endNode; endNode=cn;} else {startNode=cn;endNode=cn;cn->prev=NULL;}	
	a->mpe->cn = cn;
	cn->me_start = a;
	cn->iphone = iword;
	cn->IsSilence = IsSilence(a->phone->name); /* First arc of word is sil->silence word. */
	cn->follTrans=cn->precTrans=NULL;
	cn->scaled_aclike = fbInfo->aInfo->ac[id].aclike * latProbScale;
	
	cn->nArcs = la->nAlign; x=1; /*n arcs in cn.*/
	lastArc=a;
	if(a->follTrans)
	  for(b=a->follTrans->end;b->parentLarc==la;b->follTrans&&(b=b->follTrans->end)){
	    HArc *cb =  (b->calcArc ? b->calcArc : b);
	    x++;
	    b->mpe->cn = (CorrN*)(void*)-1;
	    cn->scaled_aclike += cb->ac->aclike * latProbScale 
	      + translm(b->precTrans->lmlike)/*should be zero unless inspen used in a funny way.*/;
	    lastArc=b;
	  }
	if(x!=cn->nArcs) HError(1, "Problem with nArcs [wordMee]...");
	cn->me_end = lastArc;
      } else if(PhoneMEE && !a->mpe->cn){ 
        /* This is the MPE case.   !a->mpe->cn is to rule out silence (see this block 
           of code, in which cn is set to -1. */

	/* Quinphone stuff [only set if quinphone]: */
	int Quinphone_NStates=1; int Quinphone_State=2; /* these defaults correspond to the non-quinphone case. */
	int iphone;
	HArc *ca = (a->calcArc ? a->calcArc : a);
	HArc *b=a; int x;
        
	iphone = GetNoContextPhone(a->phone,&Quinphone_NStates,&Quinphone_State,NULL,NULL);
	
	if(Quinphone_NStates>1 && Quinphone_State != 2){ /*not a start state.*/
	  HError(-1, "Not a [quinphone] start state.  This should happen very rarely if at all. "/*due to pruning, in fact it shouldn't happen at all.*/);
	  continue; /*continue with loop, don't do this one. */
	}
        
	cn = New(mem, sizeof(CorrN));
	
	cn->next = NULL; 
	if(endNode){ endNode->next=cn; cn->prev=endNode; endNode=cn;} else {startNode=cn;endNode=cn;cn->prev=NULL;}	

	a->mpe->cn = cn;
	cn->me_start = a;
	cn->iphone = iphone;
	cn->IsSilence = IsSilence(a->phone->name);
	cn->follTrans=cn->precTrans=NULL;

	/* Following code is the general case, for quinphones as well as triphones. */
	cn->nArcs = Quinphone_NStates; /* number of sequential phone arcs.*/
	cn->scaled_aclike = ca->ac->aclike * latProbScale;
	
	for(x=cn->nArcs;x>1;x--){ /* loop only happens in Quinphone case (when nArcs>1). */
	  if(b){
	    HArc *cb;
	    b=b->follTrans->end; /*so b is last one ... */
	    b->mpe->cn = (CorrN*)(void*)-1; /* set to -1 for all others but the first...*/
	    cb = (b->calcArc ? b->calcArc : b);
	    cn->scaled_aclike += b->ac->aclike * latProbScale + translm(b->precTrans->lmlike)/*should be zero unless inspen used in a funny way.*/;
	  } /* else  will be error . */
	}

	if(b && b->follTrans && !b->follTrans->start_foll && IsSilence(b->follTrans->end->phone->name)){ /*might as well include b->foll as well since it's silence....*/
	  HArc *cb;
	  b = b->follTrans->end;
	  b->mpe->cn = (CorrN*)(void*)-1; /* set to -1 for all others but the first...*/
	  cb = (b->calcArc ? b->calcArc : b);
	  cn->scaled_aclike += cb->ac->aclike * latProbScale + translm(b->precTrans->lmlike)
	    /*should be zero unless inspen used in a funny way.*/;
	  cn->nArcs++;
	}
	
	if(!b) HError(1, "Null arc in DoCorrectness [code or possibly lattice error]...");
	cn->me_end = b;
      }
    }
  }
  
  for(cn=startNode;cn;cn=cn->next){    /* Attach transitions to the cn structure. */
    HArc *a = cn->me_start;
    ArcTrans *at;
    for(at=cn->me_end->follTrans;at;at=at->start_foll){
      if(at->end->mpe->cn){ /* If the next arc is also within the beam... */
	if(at->end->mpe->cn==(CorrN*)(void*)-1) HError(1, "Not expecting -1 for this node..."); /* -1 only for nodes which are not the primary node
                                                                                                   of the arc, i.e. states>2 of quinphone, or end-of-word silence.*/
	AddTrans(mem, a->mpe->cn, at->end->mpe->cn, translm(at->lmlike)); /* add transition. */
      }
    }
  }
  
  /* Now recalculate alphas given new pruning, and get pr.... */
  for(cn=startNode;cn;cn=cn->next){
    CorrA *ca;
    if(!cn->me_start->precTrans) cn->alpha = 0; else cn->alpha = LZERO;

    for(ca=cn->precTrans;ca;ca=ca->end_foll){
      cn->alpha = LAdd(cn->alpha, ca->sc_lmlike + ca->start->alpha);
    }
    cn->alpha += cn->scaled_aclike; /* acoustic likelihood. */
    if(! cn->me_end->follTrans) local_pr = LAdd(local_pr, cn->alpha);
  }

  /* check local_pr:  should be same as normal pr, bar pruning:*/
  if(fabs(local_pr - pr_in) > 0.2) HError(-1, "DoCorrectness: possible problem with pr (%f != %f)...difference shouldnt be too large, decrease EXACTCORRPRUNE.",local_pr,pr_in);


  /* Now set up the arrays attached to the cn structure... */
  for(cn=startNode;cn;cn=cn->next){
    int i,ns,ne;
    int istart = cn->me_start->t_start, iend = cn->me_end->t_end;
    i = (istart+iend)/2; if(i<1||i>T){  HError(1, "istart/iend out of range."); }
    ns = minn_of_t[i];
    ne = maxn_of_t[i];

    /*following may not be needed.*/
    if(!cn->me_start->precTrans) ns = 0; /*start node.*/
    if(!cn->me_end->follTrans) ne = N;   /*end node.*/

    ns = MAX(0, ns - beamN); ne = MIN(N, ne + beamN);  /*A node can start at N although N-1 is the last phone, this may be necessary for silences not consuming any phone.*/
    cn->alphaCorr = New(mem, sizeof(float) * (ne-ns+1)); cn->alphaCorr -= ns;
    cn->betaCorr = New(mem, sizeof(float) * (ne-ns+1)); cn->betaCorr -= ns;
    cn->beta = New(mem, sizeof(double) * (ne-ns+1)); cn->beta -= ns;
    cn->starti = ns; cn->endi = ne;
    for(i=ns;i<=ne;i++){ cn->betaCorr[i]=0; cn->beta[i]=LZERO; }
  }


  /*  Now set cn->alphaCorr[i] for each node cn, which is the average correctness 
      of sentences leading up to reference phone cn where the last hypothesis sausage
      position is i. */

  for(cn=startNode;cn;cn=cn->next){
    int i;
    if(!cn->me_start->precTrans){ /* start node... */
      if(cn->starti > 0) HError(1, "start node but doesn't include zero...");
      cn->alphaCorr[0]=0; 
      for(i=1;i<=cn->endi;i++) cn->alphaCorr[i]=-10000;  /*very negative so wont be used.*/
    } else {   /* Not start node so sum over preceding nodes. */
      CorrA *ca; CorrN *cn_prev;
      for(i=cn->starti;i<=cn->endi;i++){
	cn->alphaCorr[i]=0; 
	if(!cn->precTrans) /* has no preceding nodes-- may be the case due to pruning. */
	  cn->alphaCorr[i]=-10000; 

	for(ca=cn->precTrans;ca;ca=ca->end_foll){  /* recursively calculate the correctness of this cn at this sausage-pos i,
                                                      given that previous cn's will have their correctnesses calculated at all positions. */
	  float BestCorr = -10000;
	  float occ;
	  cn_prev = ca->start;

          if(GetBestCorrectness(&BestCorr, NULL, NULL, i, cn_prev,
                                minn_of_t,maxn_of_t,niphones,iphone, nonempty,T,N,InsCorrectness)){

          
            occ = cn_prev->alpha+ca->sc_lmlike+cn->scaled_aclike - cn->alpha; /* lg(occ as fraction of total occ of cn). */
            if(occ<MINEARG) occ=0.0; else occ=exp(occ); 
            cn->alphaCorr[i] += BestCorr * occ; /* these occs will sum to 1 over all preceding arcs. */
            
            /* Checking: */
            if(BestCorr > 10000 || ((BestCorr < -500) && cn_prev->alpha>LSMALL)){
              if(debug_bestcorr > 0){
                debug_bestcorr--;
                HError(-1, "BestCorr too big (or this is a very long or strange file)... (%f)", BestCorr);
              } else if(!debug_bestcorr){
                HError(-1, "Not warning about this any more, BestCorr too big.");
                debug_bestcorr--;
              }
            }
          }
	}
      }
      if( (!cn->me_end->follTrans)) { /* end node, so get contribution to avg correctness... */
	double alpha = cn->alpha; 
	double occ = alpha - local_pr;
	occ = (occ>MINEARG ? exp(occ) : 0.0);
	
	if(cn->endi < N) HError(1, "Last node of lattice doesn't include N in alphaCorr vector.");
	if(occ > 1.1) HError(1, "Occ too big!");
        
	avg_correct += occ * cn->alphaCorr[N];  /* was MAX(cn->alphaCorr[N], cn->alphaCorr[N-1]); */
        /* This is N+1 the way I've written it in my PhD, I start from 1 not 0 there. */
        /* Only works if last phone = silence or NULL, otherwise technique wont work!! */
      }
    }
  }
  
  /* Now set beta and betaCorrect for all nodes and times. */
  /* This is a traceback of the procedure that sets alpha and alphaCorrect. */
  for(cn=endNode;cn;cn=cn->prev){
    int i;
    
    if(!cn->me_start->precTrans){ /* start node... */
      local_pr_beta = LAdd(local_pr_beta, cn->beta[0] + cn->scaled_aclike);
      avg_correct_beta = avg_correct_beta + cn->betaCorr[0] * (cn->beta[0]+cn->scaled_aclike-local_pr<MINEARG?0.0:exp(cn->beta[0]+cn->scaled_aclike-local_pr));
    } else {   /* Not start node so sum over preceding nodes. */
      CorrA *ca; CorrN *cn_prev;
      if(!cn->me_end->follTrans){ /* end node... */
         /* was: if(cn->alphaCorr[N] >  cn->alphaCorr[N-1])  N is time of this phone. */
         cn->beta[N] = 0.0; /* This is N+1 the way I've written it in my PhD, I start from 1 not 0 there. */
          /* was:   else      cn->beta[N-1] = 0.0;  */
	/* All other betas are previously initialised to LZERO and betaCorr to 0.0. */
      }
      for(i=cn->starti;i<=cn->endi;i++){
	if(cn->beta[i] + cn->alpha - local_pr > 0.001){
	  HError(-1, "Too big pr!");
	}
	if(cn->beta[i] > LZERO+1000){
	  for(ca=cn->precTrans;ca;ca=ca->end_foll){
            float betaCorr,betaCorr_prev;
            float BestCorrPart; int bestj=-1;
            double beta_prev,beta_trans,beta_sum;
	    cn_prev = ca->start;


            if( GetBestCorrectness(NULL, &BestCorrPart,  &bestj, i, cn_prev,
                                   minn_of_t,maxn_of_t,niphones,iphone, nonempty,T,N,InsCorrectness) ){ /* if there is nonzero likelihood to cn_prev.. */
            
              /* Add this contribution of beta to the previous beta, and
                 set the previous betaCorr to a weighted avg of the
                 betaCorrs (weighted by the betas. */
              
              beta_prev = cn_prev->beta[bestj];                              /* previous value of beta [beta is a likelihood] */
              beta_trans = cn->beta[i] + ca->sc_lmlike + cn->scaled_aclike;  /* a likelihood: beta due to this transition. */
              beta_sum = LAdd(beta_prev,beta_trans);                         /* the new value [the sum of old and added] */
	    
              betaCorr = cn->betaCorr[i] + BestCorrPart; /*I.e, contribution from this phone and transition...*/
              betaCorr_prev = cn_prev->betaCorr[bestj];
              {
                double occ,occ_prev;
                occ = beta_trans - beta_sum;        /* lg(occ of new part as fraction of total occ) */
                occ_prev = beta_prev - beta_sum;    /* lg(occ of old part as fraction of total occ) */
                occ=(occ>MINEARG?exp(occ):0.0); occ_prev=(occ_prev>MINEARG?exp(occ_prev):0.0);
                cn_prev->betaCorr[bestj] = betaCorr*occ + betaCorr_prev*occ_prev;
              }
              cn_prev->beta[bestj] = beta_sum;
            }
	  }
	}
      }
    }
  }
 
  /* check local_pr = local_pr_beta: forward and backward same. */
  if(fabs(local_pr - local_pr_beta) > 0.0001) HError(-1, "DoCorrectness: possible problem with pr (forward and backward %f,%f....) ",local_pr,local_pr_beta);

  /* check correctness when calculated forward and backward is the same. */
  if(fabs(avg_correct_beta - avg_correct) > 0.0001)
    HError(-1, "avg_correct{,beta} differ, %f,%f", avg_correct, avg_correct_beta);

  
  for(cn=startNode;cn;cn=cn->next){ /* Now set the "MPE occupancy" gamma_q^MPE =  gamma_q ( corr_q - corr_avg )
                                       actually we set mpe_occscale to corr_q - corr_avg, and get gamma_q^MPE later. */

    int i; float total_diff=0; /* equals the sum of: (corr-avgCorr)*occ. */
    HArc *a;
    /*float arc_occ;*/
    for(i=cn->starti;i<=cn->endi;i++){  /* correctness of node is a sum over preceding transitions... */
      if(cn->beta[i] > LSMALL){         /* only one sausage-position i should have nonzero beta, I think */
	float locc,occ;
	float correctness_diff = cn->betaCorr[i] + cn->alphaCorr[i] - avg_correct; /* difference in correctness for this i.*/
	locc = cn->alpha + cn->beta[i] - local_pr; /* The occupation probability gamma_q due to this transition. */
	occ=(locc>MINEARG?exp(locc):0.0);
	total_diff += occ * correctness_diff;
      }
    }

    a=cn->me_start;
    for(i=1;i<=cn->nArcs;i++){ /* This iterates over arcs a,  see the line "a = a->follTrans->end". */
                                /* In [non-quinphone] MPE, this loop will only have 1 iteration. */
      HArc *ca = (a->calcArc ? a->calcArc : a);
      float total_occ = ca->ac->locc; /*occ of this group of arcs [sharing this start&end&name */
      if(total_occ > MINEARG+5){
	a->ac->mpe_occscale += exp(-total_occ) * total_diff;  /* total_diff is for this arc, summed over preceding arcs. total_occ  is 
                                                                 occupation probability gamma_q for this arc. */
        /* total_diff is  gamma_q ( corr_q - corr_avg ),
           total_occ is gamma_q,
           mpe_occscale = (corr_q - corr_avg) */
      }
      if(i!=cn->nArcs){
	if(!a->follTrans) HError(1, "Problem with quinphone-related code for exact correctness.");
	a = a->follTrans->end;
      }
    }
  }

  return avg_correct;
}



/*  
    
    Criterion is: 

      sum_{last arcs} alphaCorr [N] for last arc * occ of last arc.  

      AvgCorr = sum_{last arcs} alphaCorr[N]   / sum_{last arcs} alpha[N] 

      AvgCorr = sum_{paths p} alpha[p] * corr[p]   / sum_{paths p} alpha[p] 


      d/d(lg alpha[p]) for any path is:     (corr[p] - avg correctness)*alpha[p] / sum_{paths p} alpha[p]    
              
      d/d(lg alpha[p]) for any phone arc q is:   avg(paths):  ( alpha_corr[q]+beta_corr[q] - avg correctness) *  (alpha[q]*beta[q])  / (sum_{paths p} alpha[p]   =   pr)

*/


/* 

   [this comment uses notation now superseded in code]
   
   alpha[q][i] for each arc is the same.

   for starting arcs,  alpha_correctness[q][0?] = 0, else -infinity.
   
   alpha_correctness[q][i] differs over i, & is a sum over previous arcs of alpha_correctness[r][bestj]*alpha[r] + (new contribution to correctness from last arc, if any) / sum_r alpha[r]
   
   beta[Q][N] for the last arc is 1.0  * the acprob of the arc.
   beta[Q][n!=N] for the last arc is 0.0  [* the acprob of the arc]
   
   otherwise,
   
   beta[q][i] 
       equals [acprob *] the sum over following arcs r and time
       i which transition to this arc, of: lmlike(transition prob) * beta[r][i]
 
    beta_correctness[q][N] for last arcs equals 1.0

    otherwise,
    beta_correctness[q][i] equals:
        sum_r,i s.t. there is a transition from [q][i], of:
	beta_correctness[r][bestj] + (contribution to correctness from current phone, if any).  
	  
*/



void DoExactCorrectness(FBLatInfo *fbInfo, Lattice *lat){
  if(!PhoneMEE){ 
    /* Minimum Word Error (exact).  Get the "sausage" of correct words (this is a linear sausage, no alternatives) and call DoCorrectness. */

    LNode *node; LArc *larc;  int a,n,p;
    int i,nWords=0,w; int **iwords;
    short int *niwords; /* just 1's. */
    short int *minn_of_t, *maxn_of_t;
    unsigned char *nonempty;
	 
    if(!lat){ HError(-1, "No extraNumLat for MPE! "); }
	 
    /* Count words. */
    for(node=lat->lnodes+0; node->foll; node=node->foll->end) /*This code appears to rely on first node being silence. */
      if(node->foll->nAlign > 1 ||  (node->foll->nAlign==1 && ! IsSilence(node->foll->lAlign[0].label->name))) /* a word [ not sil. ]...*/
	nWords++; 

    /* for each word arc, number it in list of words. */
    for(larc=lat->larcs,a=0;a<lat->na;larc++,a++){  
      if(larc->start->pred){ n=(int)larc->start->pred->score; }
      else n=0;
      if(larc->start->pred && IsNonSilArc(larc->start->pred))
	n++; /* prev is non-silence so increment.. */
      larc->score = (float) n;
      if((n>=nWords&&IsNonSilArc(larc)) || n<0) HError(1, "n out of range [0...nWords-1], PhoneMEE.");
    }
	 
    niwords = New(&fbInfo->tempStack, sizeof(short int) * (nWords+1));	   
    iwords = New(&fbInfo->tempStack, sizeof(int*) * (nWords+1));
    for(w=0;w<=nWords;w++) niwords[w] = 1;
	 
    nonempty = New(&fbInfo->tempStack, sizeof(char) * (nWords+1));
    minn_of_t = New(&fbInfo->tempStack, sizeof(short int) * fbInfo->T); minn_of_t--;
    maxn_of_t = New(&fbInfo->tempStack, sizeof(short int) * fbInfo->T); maxn_of_t--;
    for(i=1;i<=fbInfo->T;i++){
      minn_of_t[i] = nWords;
      maxn_of_t[i] = 0;
    }
    for(p=0;p<nWords;p++){
      niwords[p] = 1; /*all 1 & will stay that way, for MWE case */
      nonempty[p] = 1;  /*all 1 & will stay that way, for MWE case */
      iwords[p] = New(&fbInfo->tempStack, sizeof(int) * 1);
    }
	 
    /* Get word list. */
    w=0;
    for(node=lat->lnodes+0; node->foll; node=node->foll->end)
      if(node->foll->nAlign > 1 ||  (node->foll->nAlign==1 && ! IsSilence(node->foll->lAlign[0].label->name))) /* a word [ not sil. ]...*/
	iwords[w++][0] = (int)node->foll->end->word->wordName; /* word is at the node at the end of the arc. */
	 
    for(larc=lat->larcs,a=0;a<lat->na;larc++,a++){   
      if(IsNonSilArc(larc)){  /* Is a word [not sil]*/
	int startT, endT;
	int w = (int) larc->score;
	if(w<0 || w>=nWords) HError(-1, "Problem with word numbering [2] (%d,%d)...",w,nWords);
	GetTimes(larc, 0, &startT, &endT); /* get times [of first phone]... */
	if(startT<1){ HError(-1, "Invalid start time..."); startT=1;}
	if(endT>fbInfo->T){ HError(-1, "Invalid end time..."); endT=fbInfo->T; }
	if(startT>fbInfo->T){ HError(-1, "Invalid start time..."); startT=fbInfo->T;}
	if(endT<1){ HError(-1, "Invalid end time..."); endT=1; }
	minn_of_t[endT] = MIN(minn_of_t[endT], w);
	maxn_of_t[startT] = MAX(maxn_of_t[startT], w);
      }
    }
	 
    /* set {min,max}n_of_t */
    { int min=nWords-1; for(i=fbInfo->T;i>=1;i--) if(minn_of_t[i]>min) minn_of_t[i]=min; else min=minn_of_t[i]; } /*make sure increasing.*/
    { int max=0; for(i=1;i<=fbInfo->T;i++) if(maxn_of_t[i]<max) maxn_of_t[i]=max; else max=maxn_of_t[i]; } /*make sure increasing.*/
    if(maxn_of_t[1]<0 || maxn_of_t[fbInfo->T]>=nWords || minn_of_t[1]<0 || minn_of_t[fbInfo->T]>=nWords)
      HError(1, "Problem with minn_of_t or maxn_of_t...");
	 
    for(i=1;i<=fbInfo->Q;i++) fbInfo->aInfo->ac[i].mpe_occscale = 0; 
    fbInfo->AvgCorr = fbInfo->MPEFileLength + 
      DoCorrectness(fbInfo, &fbInfo->tempStack, fbInfo->aInfo, EXACTCORR_PRUNE/*prune*/, PHONE_BEAM, 
		    minn_of_t, maxn_of_t, niwords, iwords, nonempty, fbInfo->T, nWords, InsCorrectness, FALSE/*Quinphone*/, fbInfo->pr);
	 
	 
  } else { 
    /* Minimum Phone Error (Exact).  Get the "sausage" of correct phones and call DoCorrectness. */
    LNode *node; LArc *larc; 
	 
    int nWords=0; short int *maxNPhones, *nArcs, *phoneStart;
    int maxNArcs=0,n,a,w,p,i;
    int **iphone;
    short int *niphones, nPhones; unsigned char *nonempty; /*doesnt contain empty phone...*/
    short int *minn_of_t, *maxn_of_t;
	 
    if(!lat){ HError(-1, "No extraNumLat for MPE! "); }

    for(node=lat->lnodes+0; node->foll; node=node->foll->end){
      if(node->foll->nAlign > 1 ||  (node->foll->nAlign==1 && ! IsSilence(node->foll->lAlign[0].label->name))){ /* a word [ not sil. ]...*/
	nWords++; 
      }
    }
    maxNPhones = New(&fbInfo->tempStack, sizeof(short int) * (nWords+1));
    for(w=0;w<=nWords;w++) maxNPhones[w]=0;
    nArcs = New(&fbInfo->tempStack, sizeof(short int) * (nWords+1));
    for(w=0;w<=nWords;w++) nArcs[w]=0;
    phoneStart = New(&fbInfo->tempStack, sizeof(short int) * (nWords+1)); /* phone no. at which word starts.. */
	   
    for(larc=lat->larcs,a=0;a<lat->na;larc++,a++){  /* for each word arc, number it in list of words, & get maxNPhones[ ].....*/
      int np;
      if(larc->start->pred){ n=(int)larc->start->pred->score; } 
      else n=0;
      if(larc->start->pred && IsNonSilArc(larc->start->pred))
	n++; /* prev is non-silence so increment.. */
      larc->score = (float) n;
      if(IsNonSilArc(larc)){
	np = GetNumPhones(larc); /*works for quinphones too...*/
	if(np > maxNPhones[n]){
	  maxNPhones[n] = np;
	}
	nArcs[n]++;
	if(nArcs[n]>maxNArcs) maxNArcs=nArcs[n];
	if(n >= nWords) HError(-1, "Problem with word numbering...");
      }
    }
    nPhones=0;
    for(w=0;w<nWords;w++){
      phoneStart[w] = nPhones;
      nPhones += maxNPhones[w];
    }
	   
    iphone = New(&fbInfo->tempStack, sizeof(int*) * (nPhones+1));
    niphones = New(&fbInfo->tempStack, sizeof(short int) * (nPhones+1));
    nonempty = New(&fbInfo->tempStack, sizeof(char) * (nPhones+1));
    minn_of_t = New(&fbInfo->tempStack, sizeof(short int) * fbInfo->T); minn_of_t--;
    maxn_of_t = New(&fbInfo->tempStack, sizeof(short int) * fbInfo->T); maxn_of_t--;
    for(i=1;i<=fbInfo->T;i++){
      minn_of_t[i] = nPhones;
      maxn_of_t[i] = 0;
    }

    for(p=0;p<nPhones;p++){
      niphones[p] = 0;
      nonempty[p] = 1;
      iphone[p] = New(&fbInfo->tempStack, sizeof(int) * maxNArcs);
    }


    for(larc=lat->larcs,a=0;a<lat->na;larc++,a++){   /* set up the 'iphone[..]' and 'niphones' and 'nonempty' arrays.*/
      if(IsNonSilArc(larc)){  /* Is a word [not sil]*/
	int startPos,j;
	int w = (int) larc->score;
	if(w<0 || w>=nWords) HError(-1, "Problem with word numbering [2] (%d,%d)...",w,nWords);
	startPos = phoneStart[w];
	       
	p=0; /* indx of "real" phone [start of quinphone]. */
	for(j=0;j<larc->nAlign;j++){ 
	  if(NonSil_and_Quinphone_IsStartPhone(larc,j)){ /* Is a starting phone... ( x_nnn_2 or some such, I think...), and is not silence. */
	    int startT, endT;
	    int nStates_quinphone, state_quinphone=0,x; Boolean Found=FALSE;
	    int local_iphone = GetNoContextPhone(larc->lAlign[j].label,&nStates_quinphone, &state_quinphone,NULL,NULL); 
	    if(Quinphone && state_quinphone != 2) HError(1, "Quinphone problem... check code, may not be compat with this quinphone set.");
	    for(x=0;x<niphones[startPos+p];x++)if(local_iphone==iphone[startPos+p][x]){ Found=TRUE; break; } 
	    if(!Found){ iphone[startPos+p][niphones[startPos+p]++] = local_iphone; }
	    GetTimes(larc, j, &startT, &endT); /* set times... */
	    if(startT<1){ HError(-1, "Invalid start time..."); startT=1;}
	    if(endT>fbInfo->T){ HError(-1, "Invalid end time..."); endT=fbInfo->T; }
	    if(startT>fbInfo->T){ HError(-1, "Invalid start time..."); startT=fbInfo->T;}
	    if(endT<1){ HError(-1, "Invalid end time..."); endT=1; }

	    minn_of_t[endT] = MIN(minn_of_t[endT], startPos+p);
	    maxn_of_t[startT] = MAX(maxn_of_t[startT], startPos+p);
	    p++;
	  }
	}
	for(;p<maxNPhones[w];p++){
	  nonempty[startPos+p] = 0; /* contains an empty phone. */
	}
      }
    }
    for(n=0;n<nPhones;n++){ if(niphones[n]==0 || niphones[n]>maxNArcs) HError(1, "niphones > maxNArcs...."); }
	   
    { int min=nPhones-1; for(i=fbInfo->T;i>=1;i--) if(minn_of_t[i]>min) minn_of_t[i]=min; else min=minn_of_t[i]; } /*make sure increasing.*/
    { int max=0; for(i=1;i<=fbInfo->T;i++) if(maxn_of_t[i]<max) maxn_of_t[i]=max; else max=maxn_of_t[i]; } /*make sure increasing.*/
    if(maxn_of_t[1]<0 || maxn_of_t[fbInfo->T]>=nPhones || minn_of_t[1]<0 || minn_of_t[fbInfo->T]>=nPhones)
      HError(1, "Problem with minn_of_t or maxn_of_t...");
	   
#ifdef DEBUG_MEE
    for(i=0;i<nPhones;i++){ 
      int j; printf(" %d(", i);for(j=0;j<niphones[i];j++){ int phone=iphone[i][j];char *cphone=(char*)(&phone);while(!(*cphone))cphone++; printf("%s:",cphone); }
      printf("%d) ", nonempty[i]);
    } 
    printf("\n");
#endif	   
	   
    for(i=1;i<=fbInfo->Q;i++) fbInfo->aInfo->ac[i].mpe_occscale = 0; 
    fbInfo->AvgCorr = fbInfo->MPEFileLength + 
      DoCorrectness(fbInfo, &fbInfo->tempStack, fbInfo->aInfo, EXACTCORR_PRUNE/*prune*/, PHONE_BEAM, 
		    minn_of_t, maxn_of_t, niphones, iphone, nonempty, fbInfo->T, nPhones, InsCorrectness, Quinphone, fbInfo->pr);
	   
  }
}

#endif /* SUPPORT_EXACT_CORRECTNESS */


/* ------------------------------------ Initialisation ------------------------------------ */

/* EXPORT->InitExactMPE: initialise configuration parameters */ 
void InitExactMPE(void)
{
   int i;
   double f;
   Boolean b;

   Register(hexactmpe_version,hexactmpe_vc_id);
   nParm = GetConfig("HEXACTMPE", TRUE, cParm, MAXGLOBS);
   if (nParm>0){
      if (GetConfInt(cParm,nParm,"PHONEBEAM",&i)) PHONE_BEAM = i;
      if (GetConfFlt(cParm,nParm,"EXACTCORRPRUNE",&f)) EXACTCORR_PRUNE = f; /*default: log(0.0002) = -8.5 */
   }
   nParm = GetConfig("HFBLAT", TRUE, cParm, MAXGLOBS); 
   if (nParm>0){
      if (GetConfFlt(cParm,nParm,"LATPROBSCALE",&f))  latProbScale = f;/* repeat of config also used in HFBLat.c */
      if (GetConfFlt(cParm,nParm,"PHNINSPEN",&f))  phnInsPen = f;     /* repeat of config also used in HFBLat.c */
#ifdef SUPPORT_QUINPHONE
      if (GetConfBool(cParm,nParm,"QUINPHONE",&b)) Quinphone = b;  /* repeat of config also used in HFBLat.c */
#endif
      if (GetConfBool(cParm,nParm,"PHONEMEE",&b)) PhoneMEE=b;     /* repeat of config also used in HFBLat.c */
      if (GetConfBool(cParm,nParm,"MWE",&b)) if(b) PhoneMEE=FALSE; /* this config also used in HFBLat.c */
      if (GetConfFlt(cParm,nParm,"INSCORRECTNESS",&f)) InsCorrectness=-fabs(f); /* to make sure negative.*/
      if (GetConfFlt(cParm,nParm,"SUBCORRECTNESS",&f)) SubCorrectness=-fabs(f); /* to make sure negative.*/
   }

}



