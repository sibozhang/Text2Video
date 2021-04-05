/* ----------------------------------------------------------- */
/*                                                             */
/*                          ___                                */
/*                       |_| | |_/   SPEECH                    */
/*                       | | | | \   RECOGNITION               */
/*                       =========   SOFTWARE                  */ 
/*                                                             */
/*                                                             */
/* ----------------------------------------------------------- */
/*         Copyright: Microsoft Corporation                    */
/*          1995-2000 Redmond, Washington USA                  */
/*                    http://www.microsoft.com                 */
/*                                                             */
/*   Use of this software is governed by a License Agreement   */
/*    ** See the file License for the Conditions of Use  **    */
/*    **     This banner notice must not be removed      **    */
/*                                                             */
/* ----------------------------------------------------------- */
/*         File: HRec.c  Viterbi Recognition Engine Library    */
/* ----------------------------------------------------------- */

char *hrec_version = "!HVER!HRec:   3.4.1 [CUED 12/03/09]";
char *hrec_vc_id = "$Id: HRec.c,v 1.1.1.1 2006/10/11 09:54:58 jal58 Exp $";

#include "HShell.h"
#include "HMem.h"
#include "HMath.h"
#include "HWave.h"
#include "HAudio.h"
#include "HParm.h"
#include "HLabel.h"
#include "HModel.h"
#include "HDict.h"
#include "HNet.h"
#include "HRec.h"
#include "HUtil.h"
#include "HAdapt.h"

/* Trace levels */

#define T_NGEN 1

/* Checks */

#define SANITY

static int trace=0;
static Boolean forceOutput=FALSE;

const Token null_token={LZERO,0.0,NULL,NULL};

/* Define macros for assessing node type */
#define node_hmm(node) ((node)->type & n_hmm)
#define node_word(node) ((node)->type == n_word)
#define node_tr0(node) ((node)->type & n_tr0)
#define node_wd0(node) ((node)->type & n_wd0)

#define SP "sp"

/* Reduced storage requirements for merged paths */
struct nxtpath
{
   Path *prev;          /* Previous word record */
   LogDouble like;      /* Likelihood at boundary */
   LogFloat lm;         /* LM likelihood of current word */
#ifdef PHNALG
  Align *align;
#endif
   NxtPath *chain;      /* Next of NBest Paths */
};

/* Extra alignments information for state/model level traceback */
struct align
{
   short state;         /* State level traceback info */
   NetNode *node;       /* Node for which alignment information present */
   Align *prev;         /* Previous align record */

   LogDouble like;      /* Likelihood upon entering state/model end */
   int frame;           /* Frame number upon entering state/model end */
   
   Boolean used;        /* Reference to struct by current inst */
   int usage;           /* Times struct ref'd (by align or path) */

   Align *link;         /* Next align in list */
   Align *knil;         /* Prev align in list */
};

/* NBest token handling is kept private */
typedef struct reltoken
{
   LogFloat like;       /* Relative Likelihood of token */
   LogFloat lm;         /* LM likelihood of token */
   Path *path;          /* Route (word level) through network */
#ifdef PHNALG
  Align *align;
#endif
}
RelToken;

/* A tokenset is effectively a state instance */
typedef struct tokenset
{
   short n;                  /* Number of rtok valid (0==1-best, 1>==N-best) */
   RelToken *set;            /* Likelihood sorted array[0..nToks] of rtoks */
   Token tok;                /* Most likely Token in state */
}
TokenSet;

/* Need some null RelTokens */
static const RelToken rmax={0.0,0.0,NULL
#ifdef PHNALG
			    ,NULL
#endif
};    /* First rtok same as tok */
static const RelToken rnull={LZERO,0.0,NULL
#ifdef PHNALG
			     ,NULL
#endif
}; /* Rest can be LZERO */

/* The instances actually store tokens and links etc */
/* Instances are stored in creation/token propagation order to allow */
/* null/word/tee instances to be connected together and still do propagation */
/* in one pass.  Only HMMs need extra tokens, others are 1 state */
struct _NetInst
{
   struct _NetInst *link; /* Doubly linked list of instances, forward */
   struct _NetInst *knil; /* Doubly linked list of instances, backward */

   NetNode *node;       /* Position of instance within network */

   int flags;           /* Flags, active ... */
   TokenSet *state;     /* TokenSet[0..N-2] in state [1..N-1] for hmm */
   TokenSet *exit;      /* TokenSet in exit state */

   LogFloat wdlk;       /* Max likelihood of t=0 path to word end node */
   LogFloat max;        /* Likelihood for pruning of instance */

   Boolean pxd;         /* External propagation done this frame */
   Boolean ooo;         /* Instance potentially out of order */

#ifdef SANITY
   int ipos;
#endif
};

/* HMMSet information is some precomputed limits plus the precomps */
typedef struct precomp
{
   int id;                  /* Unique identifier for current frame */
   LogFloat outp;           /* State/mixture output likelihood */
}
PreComp;

struct psetinfo
{
   MemHeap heap;            /* Memory for this set of pre-comps */
   HMMSet *hset;            /* HMM Set for recognition */

   int max;                 /* Max states in HMM set */
   Boolean mixShared;
   int nsp;
   PreComp *sPre;           /* Array[1..nsp] State PreComps */
   int nmp;
   PreComp *mPre;           /* Array[1..nmp] Shared mixture PreComps */
   int ntr;
   short ***seIndexes;      /* Array[1..ntr] of seIndexes */
   Token *tBuf;             /* Buffer Array[2..N-1] of tok for StepHMM1 */
   TokenSet *sBuf;          /* Buffer Array[2..N-1] of tokset for StepHMM1_N */

   short stHeapNum;         /* Number of separate state heaps */
   short *stHeapIdx;        /* Array[1..max] of state to heap index */
};

/* Private recognition information PRecInfo. (Not visible outside HRec) */
/* Contains all status/network/allocation/pruning information for a     */
/*  single network.                                                     */
struct precinfo {
   /* Input parameters - Set once and unseen */

   Observation *obs;         /* Current Observation */

   PSetInfo *psi;           /* HMMSet information */
   Network *net;            /* Recognition network */
   int nToks;               /* Maximum tokens to propagate (0==1) */
   Boolean models;          /* Keep track of model history */
   Boolean states;          /* Keep track of state history */

   float scale;             /* LM (Net probs) scale factor */
   LogFloat wordpen;        /* Word insertion penalty */
   float pscale;            /* Pronunciation probs scale factor */
   /* Private global info */


   int frame;               /* Current frame number */
   int id;                  /* Unique observation identifier */
   int prid;                /* Unique pri identifier */

   NetNode *genMaxNode;     /* Most likely node in network */
   NetNode *wordMaxNode;    /* Most likely word end node in network */

   Token genMaxTok;         /* Most likely token */
   Token wordMaxTok;        /* Most likely word end token */

   LogFloat genThresh;      /* Cutoff from global beam */
   LogFloat wordThresh;     /* Cutoff for word end propagation */
   LogFloat nThresh;        /* Cutoff for non-best tokens */

   LogFloat *qsa;           /* Array form performing qsort */
   int qsn;                 /* Sizeof qsa */

   MemHeap instHeap;        /* Inst heap */
   MemHeap *stHeap;         /* Array[0..stHeapNum-1] of heaps for states */
   MemHeap rTokHeap;        /* RelToken heap */
   MemHeap pathHeap;        /* Path heap */
   MemHeap rPthHeap;        /* NxtPath heap */
   MemHeap alignHeap;       /* Align heap */

   int npth;                /* Current number of path records */
   int cpth;                /* Number of path records after last collection */
   Path pYesRef;            /* Head of PathYesRef linked list */
   Path pNoRef;             /* Head of PathNoRef linked list */
   Path pYesTail;           /* Tail of PathYesRef linked list */
   Path pNoTail;            /* Tail of PathNoRef linked list */

   int nalign;              /* Current number of align records */
   int calign;              /* Number of align records after last collection */
   Align aYesRef;           /* Head of AlignYesRef linked list */
   Align aNoRef;            /* Head of AlignNoRef linked list */
   Align aYesTail;          /* Tail of AlignYesRef linked list */
   Align aNoTail;           /* Tail of AlignNoRef linked list */

   int nact;                /* Number of active instances */
   int tact;                /* Cummulative number of active instances */
   NetInst head;            /* Head (oldest) of Inst linked list */
   NetInst tail;            /* Tail (newest) of Inst linked list */
   NetInst *nxtInst;        /* Inst used to select next in step sequence */
#ifdef SANITY
   NetInst *start_inst;     /* Inst that started a move */
   int ipos;                /* Current inst position */

   int pnlen;               /* Number of PathNoRef list */
   int pylen;               /* Number of PathYesRef list */

   int anlen;               /* Number of AlignNoRef list */
   int aylen;               /* Number of AlignYesRef list */
#endif

};

/* Global variable (so we want to get rid of them) */
static PRecInfo *pri;
static AdaptXForm *inXForm;

/* Module Initialisation */
static ConfParam *cParm[MAXGLOBS];      /* config parameters */
static int nParm = 0;


/* EXPORT->InitRec: register module & set configuration parameters */
void InitRec(void)
{
   int i;
   Boolean b;

   Register(hrec_version,hrec_vc_id);
   nParm = GetConfig("HREC", TRUE, cParm, MAXGLOBS);
   if (nParm>0){
      if (GetConfInt(cParm,nParm,"TRACE",&i)) trace = i;
      if (GetConfBool(cParm,nParm,"FORCEOUT",&b)) forceOutput = b;
   }
}


/* Basic token merging step used during propagation.      */ 
/* Token in cmp plus extra info from src merged into res. */
/*  tokens less likely that info.nThresh ignored.         */
static void TokSetMerge(TokenSet *res,Token *cmp,TokenSet *src)
{
   Path *path;
   TokenSet tmp;
   LogFloat diff,like,limit;
   RelToken *cur,*mch,rtoks[MAX_TOKS];
   NetNode *node, *nodes[MAX_TOKS];
   int i,nw,k,null,aux;

#ifdef SANITY
   if (res->n==0) 
      HError(8590,"TokSetMerge: Not doing NBest");
   if ((res->n!=0 && src->n==0) || (res->n==0 && src->n!=0))
      HError(8590,"TokSetMerge: TokenSet size mismatch");
#ifdef PHNALG
   if (res->tok.path!=res->set->path&&res->tok.like>pri->nThresh) 
      HError(8590,"TokSetMerge: res path mismatch !\n");
   if (res->tok.align!=res->set->align&&res->tok.like>pri->nThresh) 
      HError(8590,"TokSetMerge: res align mismatch !\n");
   if (cmp->path!=src->set->path&&src->tok.like>pri->nThresh) 
      HError(8590,"TokSetMerge: src path mismatch !\n");
   if (cmp->align!=src->set->align&&src->tok.like>pri->nThresh) 
      HError(8590,"TokSetMerge: src align mismatch !\n");
#endif
#endif

   /*  Do >= to ensure that when equal new token will be used every time in */
   /*   order to ensure that when propagation is redone paths are replaced. */
   if (cmp->like>=res->tok.like) {
      if (cmp->like>pri->nThresh) {
         if (res->tok.like>pri->nThresh) {
            /* Need to exchange res and src */

            tmp.tok=res->tok;
            res->tok=*cmp;
            for (k=0;k<res->n;k++) rtoks[k]=res->set[k];
            for (k=0;k<src->n;k++) res->set[k]=src->set[k];
            tmp.n=res->n;
            res->n=src->n;
            tmp.set=rtoks;
         }
         else {
            /* Just need to copy src to res */

            res->tok=*cmp;
            for (k=0;k<src->n;k++) res->set[k]=src->set[k];
            res->n=src->n;
            return;
         }
      }
      else
         return;
   }
   else { 
      /* Don't copy src->set just for comparison */

      if (cmp->like > pri->nThresh) {
         tmp.tok=*cmp;
         tmp.set=src->set;
         tmp.n=src->n;
      }
      else
         return;
   }

#ifdef SANITY
   if (tmp.tok.like>res->tok.like)
      HError(8590,"TokSetMerge: Tokens not exchanged");
#endif

   diff=res->tok.like-tmp.tok.like;
   
   for (i=nw=null=0,cur=res->set;i<res->n;i++,cur++) {
      for (path=cur->path;path!=NULL;path=path->prev)
         if (path->node->info.pron != NULL)
            break;
      if (path==NULL) 
         null=i+1;
      else {
         /* allow for NULL nodes in path */
         node=path->node;
         node->aux = i+1; 
         nodes[nw++] = node;
      }
   }
   
   limit=pri->nThresh-tmp.tok.like;
   for (i=0,cur=tmp.set;i<tmp.n;i++,cur++) {
      if (cur->like<limit) break;
      
      /* allow for NULL nodes in path */
      for (path=cur->path;path!=NULL;path=path->prev)
         if (path->node->info.pron != NULL)
            break;
      if (path==NULL)
         aux=null, node=NULL;
      else {
         node=path->node;
         aux=node->aux;
      }
      like=cur->like-diff;
      mch=NULL;
      /* Find matching tok/path if one (still)exists */
      if (aux!=0) {
         for (k=aux-1;k<res->n;k++) {
            for (path=res->set[k].path;path!=NULL;path=path->prev)
               if (path->node->info.pron != NULL)
                  break;
            if (path==NULL) {
               /* NULL paths match null paths */
               if (!node || !node->info.pron->word) {
                  mch=res->set+k;
                  break;
               }
            }
            else if (node && node->info.pron->word) {
               /* should compare actual net node, not the pron to avoid 
                  incorrectly merging two distinct paths */
               if (path->node==node) {
                  mch=res->set+k;
                  break;
               }
            }
         }
      }
      /* Otherwise match with least likely rtok (creating if possible) */
      if (mch==NULL) {
         if (res->n < pri->nToks) {
            mch=res->set+res->n++;
            *mch=rnull;
         }
         else
            mch=res->set+res->n-1;
      }
      /* When new rtok beats mch need to replace and re-sort */
      if (like > mch->like) {
         for (mch--;like>mch->like;mch--) {
#ifdef SANITY
            if (mch<=res->set)
               HError(8590,"TokSetMerge: Tried to shift max token");
#endif
            mch[1]=mch[0];
         }
         mch++;

         mch->path=cur->path;mch->lm=cur->lm;
#ifdef PHNALG
	 mch->align=cur->align;
#endif
         mch->like=like;
      }
   }

   /* Void lookup information */
   for (i=0;i<nw;i++)
      nodes[i]->aux=0;
}

/* Caching version of SOutP used when mixPDFs shared */
static LogFloat cSOutP(HMMSet *hset, int s, Observation *x, StreamElem *se,
                       int id)
{
   PreComp *pre;
   LogFloat bx,px,wt,det;
   int m,vSize;
   double sum;
   MixtureElem *me;
   TMixRec *tr;
   TMProb *tm;
   Vector v,tv;
   
   switch (hset->hsKind){
   case PLAINHS:
   case SHAREDHS:
      v=x->fv[s];
      me=se->spdf.cpdf+1;
      if (se->nMix==1){     /* Single Mixture Case */
         if (me->mpdf->mIdx>0 && me->mpdf->mIdx<=pri->psi->nmp)
            pre=pri->psi->mPre+me->mpdf->mIdx;
         else pre=NULL;
         if (pre==NULL) {
            bx= MOutP(ApplyCompFXForm(me->mpdf,v,inXForm,&det,id),me->mpdf);
            bx += det;
         } else if (pre->id!=id) {
            bx= MOutP(ApplyCompFXForm(me->mpdf,v,inXForm,&det,id),me->mpdf);
            bx += det;
            pre->id=id;
            pre->outp=bx;
         }
         else
            bx=pre->outp;
      } else {
         bx=LZERO;                   /* Multi Mixture Case */
         for (m=1; m<=se->nMix; m++,me++) {
            wt = MixLogWeight(hset, me->weight);
            if (wt>LMINMIX) {   
               if (me->mpdf->mIdx>0 && me->mpdf->mIdx<=pri->psi->nmp)
                  pre=pri->psi->mPre+me->mpdf->mIdx;
               else pre=NULL;
               if (pre==NULL) {
                  px= MOutP(ApplyCompFXForm(me->mpdf,v,inXForm,&det,id),me->mpdf);
                  px += det;
               } else if (pre->id!=id) {
                  px= MOutP(ApplyCompFXForm(me->mpdf,v,inXForm,&det,id),me->mpdf);
                  px += det;
                  pre->id=id;
                  pre->outp=px;
               }
               else
                  px=pre->outp;
               bx=LAdd(bx,wt+px);
            }
         }
      }
      return bx;
   case TIEDHS:
      v = x->fv[s];
      vSize = VectorSize(v);
      if (vSize != hset->swidth[s])
         HError(7071,"SOutP: incompatible stream widths %d vs %d",
                vSize,hset->swidth[s]);
      sum = 0.0; tr = hset->tmRecs+s;
      tm = tr->probs+1; tv = se->spdf.tpdf;
      for (m=1; m<=tr->topM; m++,tm++)
         sum += tm->prob * tv[tm->index];
      return (sum>=MINLARG)?log(sum)+tr->maxP:LZERO;
   default: HError(7071,"SOutP: bad hsKind %d\n",hset->hsKind);
   }
   return LZERO; /* to keep compiler happy */   
}


/* Version of POutP that caches outp values with frame id */
static LogFloat cPOutP(PSetInfo *psi,Observation *obs,StateInfo *si,int id)
{
   PreComp *pre;
   LogFloat outp;
   StreamElem *se;
   Vector w;
   int s,S;

   if (si->sIdx>0 && si->sIdx<=pri->psi->nsp)
      pre=pri->psi->sPre+si->sIdx;
   else pre=NULL;

#ifdef SANITY
   if (pre==NULL)
      HError(8520,"cPOutP: State has no PreComp attached");
#endif
   
   if (pre->id!=id) { /* bodged at the moment - fix !! */
      if ((FALSE && psi->mixShared==FALSE) || (psi->hset->hsKind == DISCRETEHS)) {
         outp=POutP(psi->hset,obs,si);
      }
      else {
         S=obs->swidth[0];
         if (S==1 && si->weights==NULL){
            outp=cSOutP(psi->hset,1,obs,si->pdf+1,id);
         }
         else {
            outp=0.0;
            se=si->pdf+1;
            w=si->weights;
            for (s=1;s<=S;s++,se++){
               outp+=w[s]*cSOutP(psi->hset,s,obs,se,id);
            }
         }
      }
      pre->outp=outp;
      pre->id=id;
   }
   return(pre->outp);
}

/* Move align record to (head of) YES referenced list */
static void MoveAlignYesRef(Align *align)
{
   align->link->knil=align->knil;
   align->knil->link=align->link;
   align->link=pri->aYesRef.link;
   align->knil=&pri->aYesRef;
   align->link->knil=align->knil->link=align;
}

/* Add reference to align record.                     */
/* Moves record to YES referenced list when necessary */
static void RefAlign(Align *align)
{
   if (align->usage==0) {
      MoveAlignYesRef(align);
#ifdef SANITY
      pri->anlen--;pri->aylen++;
#endif
   }
   align->usage++;
}

/* Remove reference to align record, moving to  */ 
/* (tail of) NO referenced list when necessary. */
static void DeRefAlign(Align *align)
{
#ifdef SANITY
   if (align->usage<0)
      HError(8591,"DeRefAlign: Align not referenced");
#endif
   align->usage--;
   if (align->usage==0) {
      align->link->knil=align->knil;
      align->knil->link=align->link;
      align->knil=pri->aNoTail.knil;
      align->link=&pri->aNoTail;
      align->link->knil=align->knil->link=align;
#ifdef SANITY
      pri->anlen++;pri->aylen--;
#endif
   }
}

/* Allocate new align record and add to NOT referenced list */
static Align *NewNRefAlign(NetNode *node,int state,double like,
                           int frame,Align *prev)
{
   Align *align;

   align=(Align*) New(&pri->alignHeap,0);
   align->link=pri->aNoRef.link;
   align->knil=&pri->aNoRef;
   align->link->knil=align->knil->link=align;
   align->usage=0;
   align->used=FALSE;

   align->node=node;
   align->state=state;
   align->like=like;
   align->frame=frame;
   
   if ((align->prev=prev)!=NULL)
      RefAlign(prev);

   pri->nalign++;
#ifdef SANITY
   pri->anlen++;
#endif

   return(align);
}

/* Remove and free align record from NO referenced list */
static void UnlinkAlign(Align *align)
{
   align->link->knil=align->knil;
   align->knil->link=align->link;
#ifdef SANITY
   if (align->usage!=0)
      HError(8591,"UnlinkAlign: Freeing referenced align record");
   align->link=align->knil=NULL;
   align->node=NULL;align->prev=0;
   align->usage=0;pri->anlen--;
#endif
   Dispose(&pri->alignHeap,align);
   pri->nalign--;
}

static void StepHMM1(NetNode *node) /* Model internal propagation NBEST */
{
   NetInst *inst;
   HMMDef *hmm;
   Token tok,max;
   TokenSet *res,cmp,*cur;
   Align *align;
   int i,j,k,N,endi;
   LogFloat outp;
   Matrix trP;
   short **seIndex;
#ifdef PHNALG
   int n;
#endif
   
   inst=node->inst;
   max=null_token;
   
   hmm=node->info.hmm; 
   N=hmm->numStates;
   trP=hmm->transP;
   seIndex=pri->psi->seIndexes[hmm->tIdx];
   
   for (j=2,res=pri->psi->sBuf+2;j<N;j++,res++) {  /* Emitting states first */
      i=seIndex[j][0]; 
      endi=seIndex[j][1];
      cur=inst->state+i-1;

      res->tok=cur->tok; res->n=cur->n;
      for (k=0;k<cur->n;k++) res->set[k]=cur->set[k];

      res->tok.like+=trP[i][j];

      for (i++,cur++;i<=endi;i++,cur++) {
         cmp.tok=cur->tok;
         cmp.tok.like+=trP[i][j];
         if (res->n==0) {
            if (cmp.tok.like > res->tok.like)
               res->tok=cmp.tok;
         }
         else
            TokSetMerge(res,&cmp.tok,cur);
      }
      if (res->tok.like>pri->genThresh) { /* State pruning */
         outp=cPOutP(pri->psi,pri->obs,hmm->svec[j].info,pri->id);
         res->tok.like+=outp;
   
         if (res->tok.like>max.like)
            max=res->tok;
         if (pri->states) {
            if (res->tok.align==NULL?TRUE:
                res->tok.align->state!=j || res->tok.align->node!=node) {
               align=NewNRefAlign(node,j,
                                  res->tok.like-outp-res->tok.lm*pri->scale,
                                  pri->frame-1,res->tok.align);
               res->tok.align=align;
#ifdef PHNALG
	       if (pri->nToks>1)
		 res->set[0].align=align;
#endif
            }
#ifdef PHNALG
	    for (n=1;n<res->n;n++) {
	      if (res->set[n].align==NULL?TRUE:
		  (res->set[n].align->state!=j||
		   res->set[n].align->node!=node)) {
		align=NewNRefAlign(node,j,
				   res->tok.like-outp-res->tok.lm*pri->scale,
				   pri->frame-1,res->set[n].align);
		res->set[n].align=align;
	      }
	    }
#endif
         }
      } 
      else {
         res->tok=null_token;
         res->n=((pri->nToks>1)?1:0);
      }
   }
   
   /* Null entry state ready for external propagation */
   /*  And copy tokens from buffer to instance */
   for (i=1,res=pri->psi->sBuf+1,cur=inst->state;
        i<N;i++,res++,cur++) {
      cur->n=res->n; cur->tok=res->tok; 
      for (k=0;k<res->n;k++) cur->set[k]=res->set[k];
   }

   /* Set up pruning limits */
   if (max.like>pri->genMaxTok.like) {
      pri->genMaxTok=max;
      pri->genMaxNode=node;
   }
   inst->max=max.like;

   i=seIndex[N][0]; /* Exit state (ignoring tee trP) */
   endi=seIndex[N][1];
   
   res=inst->exit;
   cur=inst->state+i-1;

   res->n=cur->n; 
   res->tok=cur->tok; 
   for (k=0;k<cur->n;k++) res->set[k]=cur->set[k];

   res->tok.like+=trP[i][N];

   for (i++,cur++;i<=endi;i++,cur++) {
      cmp.tok=cur->tok; 
      cmp.tok.like+=trP[i][N];

      if (res->n==0) {
         if (cmp.tok.like > res->tok.like) 
            res->tok=cmp.tok;
      }
      else 
         TokSetMerge(res,&cmp.tok,cur);
   }
   if (res->tok.like>LSMALL){
      tok.like=res->tok.like+inst->wdlk;
      if (tok.like > pri->wordMaxTok.like) {
         pri->wordMaxTok=tok;
         pri->wordMaxNode=node;
      }
      if (!node_tr0(node) && pri->models) {
         align=NewNRefAlign(node,-1,
                            res->tok.like-res->tok.lm*pri->scale,
                            pri->frame,res->tok.align);
         res->tok.align=align;
#ifdef PHNALG
	 if (pri->nToks>1)
           res->set[0].align=align;
         for (n=1;n<res->n;n++) {
           align=NewNRefAlign(node,-1,
                              res->tok.like-res->tok.lm*pri->scale,
                              pri->frame,res->set[n].align);
           res->set[n].align=align;
         }
#endif
      }
   } else {
      inst->exit->tok=null_token;
      inst->exit->n=((pri->nToks>1)?1:0);
   }
}

/* Tee transition propagation - may be repeated */
static void StepHMM2(NetNode *node) 
{
   NetInst *inst;
   HMMDef *hmm;
   TokenSet cmp,*res,*cur;
   Align *align;
   int N;
#ifdef PHNALG
   int n;
#endif

   inst=node->inst;

   hmm=node->info.hmm; 
   N=hmm->numStates;

   cur=inst->state;
   res=inst->exit;

   cmp.tok=cur->tok;
   cmp.tok.like+=hmm->transP[1][N];

   if (res->n==0) {
      if (cmp.tok.like>res->tok.like)
         res->tok=cmp.tok;
   }
   else 
      TokSetMerge(res,&cmp.tok,cur);

   if (pri->models) {
      align=NewNRefAlign(node,-1,
                         res->tok.like-res->tok.lm*pri->scale,
                         pri->frame,res->tok.align);
      res->tok.align=align;
#ifdef PHNALG
      if (pri->nToks>1)
        res->set[0].align=align;
      for (n=1;n<res->n;n++) {
        align=NewNRefAlign(node,-1,
                           res->tok.like-res->tok.lm*pri->scale,
                           pri->frame,res->set[n].align);
        res->set[n].align=align;
      }
#endif
   }
}

static Path *NewNRefPath(void)
{
   Path *path;

   path=(Path*) New(&pri->pathHeap,0);
   path->link=pri->pNoRef.link;
   path->knil=&pri->pNoRef;
   path->link->knil=path->knil->link=path;
   path->chain=NULL;
   path->used=FALSE;
   pri->npth++;
#ifdef SANITY
   pri->pnlen++;
#endif
   return(path);
}

static void MovePathYesRef(Path *path)
{
   path->link->knil=path->knil;
   path->knil->link=path->link;
   path->link=pri->pYesRef.link;
   path->knil=&pri->pYesRef;
   path->link->knil=path->knil->link=path;
}

static void RefPath(Path *path)
{
   if (path->usage==0) {
      MovePathYesRef(path);
#ifdef SANITY
      pri->pnlen--;pri->pylen++;
#endif
   }
   path->usage++;
}
   
static void DeRefPathPrev(Path *path)
{
   Path *pth;
   NxtPath tmp,*cur;
   
   tmp.prev=path->prev;
   tmp.chain=path->chain;
   for (cur=&tmp;cur!=NULL;cur=cur->chain) {
      if ((pth=cur->prev)!=NULL) {
#ifdef SANITY
         if (pth->usage<=0)
            HError(8591,"DeRefPathPrev: Path not referenced");
#endif
         pth->usage--;
         if (pth->usage==0) {
            pth->link->knil=pth->knil;
            pth->knil->link=pth->link;
            pth->link=pri->pNoRef.link;
            pth->knil=&pri->pNoRef;
            pth->link->knil=pth->knil->link=pth;
#ifdef SANITY
            pri->pnlen++;pri->pylen--;
#endif
         }
      }
   }
}
   
static void UnlinkPath(Path *path)
{
   NxtPath *pth,*nth;

   path->link->knil=path->knil;
   path->knil->link=path->link;
   for (pth=path->chain;pth!=NULL;pth=nth) {
      nth=pth->chain;
      Dispose(&pri->rPthHeap,pth);
   }
#ifdef SANITY
   path->link=path->knil=NULL;
   /* path->pron=NULL;*/
   path->prev=0;
   path->usage=0;pri->pnlen--;
   path->chain=NULL;
   path->frame=-1;
#endif
   Dispose(&pri->pathHeap,path);
   pri->npth--;
}

static void CollectPaths(void)
{
   NetInst *inst;
   TokenSet *cur;
   int i,k,n;
   Path *path,*plink;
   Align *align,*alink;

   for (inst=pri->head.link;inst!=NULL;inst=inst->link)
      if (inst->node!=NULL) {
         if (node_hmm(inst->node)) 
            n=inst->node->info.hmm->numStates-1;
         else
            n=1;
         for (i=1,cur=inst->state;i<=n;i++,cur++) {
            path=cur->tok.path;
            if (path && !path->used) {
               if (path->usage!=0) MovePathYesRef(path);
               path->used=TRUE;
            }
#ifdef SANITY
            if (path!=NULL && cur->n>0 && cur->tok.path!=cur->set[0].path)
               HError(8590,"CollectPaths: Top path mismatch in state %d",i);
#endif
            for (k=1;k<cur->n;k++) {
               path=cur->set[k].path;
               if (path && !path->used) {
                  if (path->usage!=0) MovePathYesRef(path);
                  path->used=TRUE;
               }
#ifdef PHNALG
	       align=cur->set[k].align;
               if (align && !align->used) {
                 if (align->usage!=0) MoveAlignYesRef(align);
		 align->used=TRUE;
               }
#endif
            }
            align=cur->tok.align;
            if (align && !align->used) {
               if (align->usage!=0) MoveAlignYesRef(align);
               align->used=TRUE;
            }
         }
         path=inst->exit->tok.path;
         if (path && !path->used) {
            if (path->usage!=0) MovePathYesRef(path);
            path->used=TRUE;
         }
#ifdef SANITY
         if (path!=0 && inst->exit->n>0 && 
             inst->exit->tok.path!=inst->exit->set[0].path)
            HError(8590,"CollectPaths: Top path mismatch in state N");
#endif
         for (k=1;k<inst->exit->n;k++) {
            path=inst->exit->set[k].path;
            if (path && !path->used) {
               if (path->usage!=0) MovePathYesRef(path);
               path->used=TRUE;
            }
#ifdef PHNALG
	    align=inst->exit->set[k].align;
            if (align && !align->used) {
	      if (align->usage!=0) MoveAlignYesRef(align);
	      align->used=TRUE;
            }
#endif
         }
         align=inst->exit->tok.align;
         if (align && !align->used) {
            if (align->usage!=0) MoveAlignYesRef(align);
            align->used=TRUE;
         }
      }

   for (path=pri->pNoRef.link;path->link!=NULL;path=plink) {
      if (!path->used) {
         if (path->align!=NULL)
            DeRefAlign(path->align);
         DeRefPathPrev(path);
         plink=path->link;
         UnlinkPath(path);
      }
      else {
         path->used=FALSE;
         plink=path->link;
      }
   }
   for (path=pri->pYesRef.link;path->link!=NULL;path=path->link) {
      if (!path->used) break;
      path->used=FALSE;
   }
   pri->cpth=pri->npth;

   for (align=pri->aNoRef.link;align->link!=NULL;align=alink) {
      if (!align->used) {
         if (align->prev!=NULL)
            DeRefAlign(align->prev);
         alink=align->link;
         UnlinkAlign(align);
      }
      else {
         align->used=FALSE;
         alink=align->link;
      }
   }
   for (align=pri->aYesRef.link;align->link!=NULL;align=align->link) {
      if (!align->used) break;
      align->used=FALSE;
   }
   pri->calign=pri->nalign;
}

static void StepWord1(NetNode *node) /* Just invalidate the tokens */
{
   node->inst->state->tok=null_token;
   node->inst->state->n=((pri->nToks>1)?1:0);
   node->inst->exit->tok=null_token;
   node->inst->exit->n=((pri->nToks>1)?1:0);
   node->inst->max=LZERO;
}

static void StepWord2(NetNode *node) /* Update the path - may be repeated */
{
   NetInst *inst;
   Path *newpth,*oldpth;
   RelToken *cur;
   NxtPath *rth;
   int i,k;

   inst=node->inst;

   if (node->info.pron==NULL && node->tag==NULL) {
      inst->exit->tok=inst->state->tok;
      inst->exit->n=inst->state->n;
      for (k=0;k<inst->exit->n;k++)
         inst->exit->set[k]=inst->state->set[k];
   }
   else {
      inst->exit->tok=inst->state->tok;
      if (node->info.pron!=NULL) {
         inst->exit->tok.like+=pri->wordpen;
         inst->exit->tok.like+=node->info.pron->prob*pri->pscale;
      }
      newpth=NewNRefPath();
      newpth->node=node;
      newpth->usage=0;
      newpth->frame=pri->frame;
      newpth->like=inst->exit->tok.like;
      newpth->lm=inst->exit->tok.lm;
      if ((newpth->align=inst->exit->tok.align)!=NULL)
         RefAlign(newpth->align);
      inst->exit->tok.path=newpth;
      inst->exit->tok.lm=0.0;
      inst->exit->tok.align=NULL;
      
      oldpth=inst->state->tok.path;
      if ((newpth->prev=oldpth)!=NULL)
         RefPath(oldpth);

      if (pri->nToks>1) {
         inst->exit->n=1;
         inst->exit->set[0].path=newpth;

         cur=inst->state->set+1;
         if (inst->state->n>1) {
            rth=(NxtPath*) New(&pri->rPthHeap,0);
            newpth->chain=rth;
            rth->chain=NULL;
            rth->like=newpth->like+cur->like;
            rth->lm=cur->lm;
            if ((rth->prev=cur->path)!=NULL)
               RefPath(cur->path);
#ifdef PHNALG
	    if ((rth->align=cur->align)!=NULL)
	      RefAlign(cur->align);
#endif
            for (i=2,cur++;i<inst->state->n;i++,cur++) {
               rth->chain=(NxtPath*) New(&pri->rPthHeap,0);
               rth=rth->chain;
               rth->chain=NULL;
               rth->like=newpth->like+cur->like;
               rth->lm=cur->lm;
               if ((rth->prev=cur->path)!=NULL)
                  RefPath(cur->path);
#ifdef PHNALG
	       if ((rth->align=cur->align)!=NULL)
		 RefAlign(cur->align);
#endif
            }
         }
      }
      else {
         inst->exit->n=0;
         newpth->chain=NULL;
      }
   }
}

static void MoveToRecent(NetInst *inst)
{
   if (inst->node==NULL) return;

   /* If we are about to move the instance that is used to determine the   */
   /*  next instance to be stepped (to the most recent end of the list) we */
   /*  must use the previous instance to determine the next one to step !! */
   if (inst==pri->nxtInst)
      pri->nxtInst=inst->knil;

   inst->link->knil=inst->knil;
   inst->knil->link=inst->link;

   inst->link=&pri->tail;
   inst->knil=pri->tail.knil;

   inst->link->knil=inst;
   inst->knil->link=inst;

   inst->pxd=FALSE;
   inst->ooo=TRUE;

#ifdef SANITY
   if (inst==pri->start_inst)
      HError(8521,"MoveToRecent: Loop resulted in circular move");
   inst->ipos=pri->ipos++;
#endif
}

static void ReOrderList(NetNode *node)
{
   NetLink *dest;
   int i;

   if (node->inst!=NULL?!node->inst->ooo:TRUE) return;
   node->inst->ooo=FALSE;
   for (i=0,dest=node->links;i<node->nlinks;i++,dest++) {
      if (!node_tr0(dest->node)) break;
      if (dest->node->inst!=NULL) 
         MoveToRecent(dest->node->inst);
   }
   for (i=0,dest=node->links;i<node->nlinks;i++,dest++) {
      if (!node_tr0(dest->node)) break;
      if (dest->node->inst!=NULL)
         ReOrderList(dest->node);
   }
}

static LogFloat LikeToWord(NetNode *node)
{
   NetLink *dest;
   HMMDef *hmm;
   LogFloat like,best;
   int i,N;

   best=LZERO;
   for (i=0,dest=node->links;i<node->nlinks;i++,dest++) {
      if (!node_tr0(dest->node)) break;
      like=dest->like*pri->scale;
      if (like<=best) continue;
      if (node_word(dest->node)) {
         if (like>best) best=like;
      }
      else if (node_wd0(dest->node)) { /* Must be tee hmm */
         if (dest->node->type!=(n_hmm+n_tr0+n_wd0))
            HError(8521,"LikeToWord: Node should be hmm & tr0 & wd0 (%d)",
                   node->type);
         hmm=dest->node->info.hmm;
         N=hmm->numStates;
         like+=hmm->transP[1][N];
         like+=LikeToWord(dest->node);
         if (like>best) best=like;
      }
   }
   return(best);
}   

static void AttachInst(NetNode *node)
{
   TokenSet *cur;
   NetInst *inst;
   int i,n;

   inst=(NetInst*) New(&pri->instHeap,0);
   if (node_hmm(node))
      n=node->info.hmm->numStates-1;
   else
      n=1;

#ifdef SANITY
   if (pri->psi->stHeapIdx[n]<0)
      HError(8592,"AttachInst: State heap not created for %d states",n);
#endif
   inst->node=node;
   inst->state=(TokenSet*) New(pri->stHeap+pri->psi->stHeapIdx[n],0);
   inst->exit=(TokenSet*) New(pri->stHeap+pri->psi->stHeapIdx[1],0);

   inst->exit->tok=null_token;
   if (pri->nToks>1) {
      inst->exit->set=(RelToken*) New(&pri->rTokHeap,0);
      inst->exit->n=1;
      inst->exit->set[0]=rmax;
   }
   else {
      inst->exit->n=0;
   }
   
   for (i=1,cur=inst->state;i<=n;i++,cur++) {
      cur->tok=null_token;
      if (pri->nToks>1) {
         cur->set=(RelToken*) New(&pri->rTokHeap,0);
         cur->n=1;
         cur->set[0]=rmax;
      }
      else {
         cur->n=0;
      }
   }
   inst->max=LZERO;

   inst->link=&pri->tail;
   inst->knil=pri->tail.knil;

   inst->link->knil=inst;
   inst->knil->link=inst;

   node->inst=inst;

   if (node_wd0(node))
      inst->wdlk=LikeToWord(inst->node);
   else
      inst->wdlk=LZERO;

   pri->nact++;
   /* New node needs any currently alive following insts moved */
   /*  to be more recent than it to ensure tokens propagated in */
   /*  correct order. */
   inst->ooo=TRUE;   /* Need keep list in propagation order */
#ifdef SANITY
   inst->ipos=pri->ipos++;
   pri->start_inst=inst;
#endif
   ReOrderList(node);
}

static void DetachInst(NetNode *node)
{
   TokenSet *cur;
   NetInst *inst;
   int i,n;

   inst=node->inst;
   pri->nact--;
#ifdef SANITY
   if (inst->node!=node)
      HError(8591,"DetachInst: Node/Inst mismatch");
#endif
   inst->link->knil=inst->knil;
   inst->knil->link=inst->link;
   
   if (node_hmm(node))
      n=node->info.hmm->numStates-1;
   else 
      n=1;
   if (pri->nToks>1) {
      for (i=0,cur=inst->state;i<n;i++,cur++) 
         Dispose(&pri->rTokHeap,cur->set);
      Dispose(&pri->rTokHeap,inst->exit->set);
   }
   Dispose(pri->stHeap+pri->psi->stHeapIdx[n],inst->state);
   Dispose(pri->stHeap+pri->psi->stHeapIdx[1],inst->exit);
#ifdef SANITY
   inst->state=NULL;
   inst->exit=NULL;
#endif
   Dispose(&pri->instHeap,inst);

   node->inst=0;
}

static void SetEntryState(NetNode *node,TokenSet *src)
{
   NetInst *inst;
   TokenSet *res;
#ifdef SANITY
#ifdef PHNALG
   int i;
#endif
#endif

   if (node->inst==NULL)
      AttachInst(node);

   inst=node->inst;
   res=inst->state;
#ifdef SANITY
   if ((res->n==0 && src->n!=0) || (res->n!=0 && src->n==0))
      HError(8590,"SetEntryState: TokenSet size mismatch");
   /*
     if (src->tok.like>LSMALL && 
     src->tok.path!=NULL && src->tok.path->node->info.pron==NULL)
     HError(8590,"SetEntryState: NULL word propagated into path"); */
#ifdef PHNALG
   if (node_word(node)&&node->info.pron&&(pri->models||pri->states)) {
     for(i=0;i<src->n;i++)
       if (!src->set[i].align&&src->set[i].path)
	 HError(8590,"SetEntryState: NULL align try to enter word node [%s] (word might have nil pronunciation)!",node->info.pron->word->wordName->name);
     for(i=0;i<res->n;i++)
       if (!res->set[i].align&&res->set[i].path)
	 HError(8590,"SetEntryState: NULL align in word node!");
   }
#endif
#endif
   if (res->n==0) {
      if (src->tok.like > res->tok.like)
         res->tok=src->tok;
   }
   else
      TokSetMerge(res,&src->tok,src);
   if (res->tok.like>inst->max)
      inst->max=res->tok.like;
   if (node->type==n_word && (pri->wordMaxNode==NULL || 
                              pri->wordMaxNode->inst==NULL || 
                              res->tok.like > pri->wordMaxNode->inst->max))
      pri->wordMaxNode=node;
}

static void StepInst1(NetNode *node) /* First pass of token propagation (Internal) */
{
   if (node_hmm(node))
      StepHMM1(node);   /* Advance tokens within HMM instance t => t-1 */
                        /* Entry tokens valid for t-1, do states 2..N */
   else
      StepWord1(node);
   node->inst->pxd=FALSE;
}

static void StepInst2(NetNode *node) /* Second pass of token propagation (External) */
     /* Must be able to survive doing this twice !! */
{
   Token tok;
   TokenSet xtok;
   RelToken rtoks[MAX_TOKS];
   NetLink *dest;
   LogFloat lm;
   int i,k;

   if (node_word(node))
      StepWord2(node);  /* Merge tokens and update traceback */
   else if (node_tr0(node) /* && node_hmm(node) */)
      StepHMM2(node);   /* Advance tokens within HMM instance t => t-1 */
                        /* Entry token valid for t, only do state N */
   tok=node->inst->exit->tok;
   xtok.tok=tok;
   xtok.n=node->inst->exit->n;
   xtok.set=rtoks;
   for (k=0;k<xtok.n;k++)
      xtok.set[k]=node->inst->exit->set[k];

   if (node_word(node))
      if (tok.like<pri->wordThresh)
         tok=null_token;

   if (tok.like>pri->genThresh) {
      for(i=0,dest=node->links;i<node->nlinks;i++,dest++) {
         lm=dest->like;
         xtok.tok.like=tok.like+lm*pri->scale;
         xtok.tok.lm=tok.lm+lm;
         for (k=0;k<xtok.n;k++)
            xtok.set[k].lm=node->inst->exit->set[k].lm+lm;
         if (xtok.tok.like>pri->genThresh) {
            SetEntryState(dest->node,&xtok);
            /* Transfer set of tokens to node, activating when necessary */
            /* choosing N most likely after adding transition likelihood */
         }
      }
   }
   node->inst->pxd=TRUE;
}

static void CreateSEIndex(PSetInfo *psi,HLink hmm)
{
   SMatrix trP;
   short **se; /* Actually (*se)[2] */
   int j,min,max,N;

   trP=hmm->transP; N=hmm->numStates;
   se=psi->seIndexes[hmm->tIdx];
   if (se==NULL) {
      se=(short**) New(&psi->heap,(N-1)*sizeof(short*));
      se-=2;
      for (j=2;j<=N;j++) {
         se[j]=(short*) New(&psi->heap,2*sizeof(short));
         for (min=(j==N)?2:1;min<N;min++) /* Don't want tee transitions */
            if (trP[min][j]>LSMALL) break;
         for (max=N-1;max>1;max--)
            if (trP[max][j]>LSMALL) break;
#ifdef SANITY
         if (min>max) {
            HError(-8520,"CreateSEIndex: No transitions to state %d",j);
            min=(j==N)?2:1;
            max=N-1;
         }
#endif
         se[j][0]=min;
         se[j][1]=max;
      }
      psi->seIndexes[hmm->tIdx]=se;
   }
}

/* Prepare HMMSet for recognition.  Allocates seIndex and preComp from */
/*  hmmset heap.*/
PSetInfo *InitPSetInfo(HMMSet *hset)
{
   PSetInfo *psi;
   RelToken *rtoks;
   int n,h,i;
   HLink hmm;
   MLink q;
   PreComp *pre;
   char name[80];
   static int psid=0;

   psi=(PSetInfo*) New(&gcheap,sizeof(PSetInfo));
   psi->hset=hset;
   sprintf(name,"PRI-%d Heap",psid++);
   CreateHeap(&psi->heap,name,MSTAK,1,1.0,1000,8000);

   psi->max=MaxStatesInSet(hset)-1;

   psi->tBuf=(Token*) New(&psi->heap,(psi->max-1)*sizeof(Token));
   psi->tBuf-=2;

   psi->sBuf=(TokenSet*) New(&psi->heap,psi->max*sizeof(TokenSet));
   rtoks=(RelToken*) New(&psi->heap,psi->max*sizeof(RelToken)*MAX_TOKS);
   psi->sBuf-=1;
   for (i=0; i<psi->max; i++) {
      psi->sBuf[i+1].set=rtoks;rtoks+=MAX_TOKS;
      psi->sBuf[i+1].tok=null_token;
      psi->sBuf[i+1].n=0;
      psi->sBuf[i+1].set[0]=rmax;
   }

   psi->stHeapIdx=(short*) New(&psi->heap,(psi->max+1)*sizeof(short));
   for (i=0; i<=psi->max; i++) psi->stHeapIdx[i]=-1;
   psi->stHeapIdx[1]=0; /* For one state word end models */

   psi->ntr=hset->numTransP;
   psi->seIndexes=(short***) New(&psi->heap, sizeof(short**)*psi->ntr);
   psi->seIndexes--;
   for(i=1;i<=psi->ntr;i++) psi->seIndexes[i]=NULL;
   for (h=0; h<MACHASHSIZE; h++)
      for (q=hset->mtab[h]; q!=NULL; q=q->next) {
         if (q->type=='h') {
            hmm=(HLink)q->structure;
            n=hmm->numStates-1;
            psi->stHeapIdx[n]=0;
            CreateSEIndex(psi,hmm);
         }
      }
   psi->nsp=hset->numStates;
   psi->sPre=(PreComp*) New(&psi->heap, sizeof(PreComp)*psi->nsp);
   psi->sPre--;
   for(i=1,pre=psi->sPre+1;i<=psi->nsp;i++,pre++) pre->id=-1;
   if (hset->numSharedMix>0) {
      psi->mixShared=TRUE;
      psi->nmp=hset->numSharedMix;
      psi->mPre=(PreComp*) New(&psi->heap, sizeof(PreComp)*psi->nmp);
      psi->mPre--;
      for(i=1,pre=psi->mPre+1;i<=psi->nmp;i++,pre++) pre->id=-1;
   }
   else
      psi->mixShared=FALSE,psi->nmp=0,psi->mPre=NULL;

   for (n=1,i=0;n<=psi->max;n++)
      if (psi->stHeapIdx[n]>=0)
         psi->stHeapIdx[n]=i++;
   psi->stHeapNum=i;

   return(psi);
}

void FreePSetInfo(PSetInfo *psi)
{
   DeleteHeap(&psi->heap);
   Dispose(&gcheap,psi);
}

static void LatFromPaths(Path *path,int *ln,Lattice *lat)
{
   LNode *ne,*ns;
   LArc *la;
   Word nullwordId;
   NxtPath tmp,*pth;
   Align *align,*al,*pr;
   MLink ml;
   LabId labid,splabid,labpr = NULL;
   char buf[80];
   int i,frame;
   double prlk,dur,like,wp;

   nullwordId = GetWord(lat->voc,GetLabId("!NULL",FALSE),FALSE);
   splabid = GetLabId(SP,FALSE);

   tmp.prev=path->prev;
   tmp.like=path->like;
   tmp.chain=path->chain;
   tmp.lm=path->lm;
#ifdef PHNALG
   tmp.align=path->align;
#endif

   ne=lat->lnodes-path->usage;
   ne->time=path->frame*lat->framedur;
   if (path->node->info.pron != NULL)
      ne->word=path->node->info.pron->word;
   else
      ne->word=nullwordId;
   ne->tag=path->node->tag;
   if (path->node->info.pron != NULL)
      ne->v=path->node->info.pron->pnum;
   else
      ne->v=1;
   ne->score=path->like;

   align=path->align;

   for(pth=&tmp;pth!=NULL;pth=pth->chain) {
      la=lat->larcs+(*ln)++;
      if (pth->prev){
         ns=lat->lnodes-pth->prev->usage,prlk=pth->prev->like;
      } else {
         ns=lat->lnodes,prlk=0.0;
      }
      la->start=ns;la->end=ne;
      if (ne->word==NULL || ne->word==nullwordId) /* no word or NULL node */
         wp=0.0;                   /* No penalty for current word */
      else wp=pri->wordpen;       /* Inc penalty for current word */
      la->aclike=pth->like-prlk-pth->lm*pri->scale-wp;
      if (path->node->info.pron != NULL) {
         la->aclike-=path->node->info.pron->prob*pri->pscale;
         la->prlike=path->node->info.pron->prob;
      }
      else
         la->prlike=0.0;
      la->lmlike=pth->lm;
      la->score=pth->like;
      la->farc=ns->foll;la->parc=ne->pred;
      ns->foll=ne->pred=la;
      
      if (pth->prev!=NULL && ns->word==NULL)
         LatFromPaths(pth->prev,ln,lat);
#ifdef PHNALG
      align=pth->align;
#endif

      if (align!=NULL) {
         i=0;
         for (al=align;al!=NULL;al=al->prev) 
            i++;
         la->nAlign=i;
         la->lAlign=(LAlign*) New(lat->heap,sizeof(LAlign)*i);
         frame=path->frame;pr=NULL;
         /* Allow for wp diff between path and align */
#ifdef PHNALG
	 like=pth->like-pth->lm*pri->scale-wp;
#else
         like=path->like-pth->lm*pri->scale-wp; 
#endif
         for (al=align;al!=NULL;al=al->prev) {
            ml=FindMacroStruct(pri->psi->hset,'h',al->node->info.hmm);
            if (ml==NULL)
               HError(8520,"LatFromPaths: Cannot find hmm in hset");
            if (al->state<0) {
               if (pr==NULL) {
                  pr=al;
                  labpr=ml->id;
                  continue;
               }
               if (labpr==NULL)
                  HError(8522,"LatFromPaths: Align records out of order");
               dur=(pr->frame-al->frame)*lat->framedur;
               like=pr->like-al->like;
               pr=al;
               labid=labpr;
               labpr=ml->id;
            }
            else {
               if (pri->models)
                  sprintf(buf,"s%d",al->state);
               else
                  sprintf(buf,"%s[%d]",ml->id->name,al->state);
               labid=GetLabId(buf,TRUE);
               dur=(frame-al->frame)*lat->framedur,
                  like=like-al->like;
               frame=al->frame;
            }
            i--;
            la->lAlign[i].state=al->state;
            la->lAlign[i].label=labid;
#ifdef PHNALG
	    /* didn't handle model that allow 0 frame */
            if (dur<0 && labid != splabid) HError(8522,"LatFromPaths: Align  have dur<0");
#endif
            la->lAlign[i].dur=dur;
            la->lAlign[i].like=like;
            like=al->like;
         }
         if (pr!=NULL) {
#ifdef PHNALG
	    if (pth->prev!=NULL)
               dur=(pr->frame-pth->prev->frame)*lat->framedur,
                  like=pr->like-pth->prev->like;
#else
            if (path->prev!=NULL)
               dur=(pr->frame-path->prev->frame)*lat->framedur,
                  like=pr->like-path->prev->like;
#endif
            else
               dur=pr->frame*lat->framedur,
                  like=pr->like;
            i--;
            la->lAlign[i].state=-1;
            la->lAlign[i].label=labpr;
#ifdef PHNALG
	    /* didn't handle model that allow 0 frame */
            if (dur<0 && labid != splabid) HError(8522,"LatFromPaths: Align have dur<0 ");
#endif
            la->lAlign[i].dur=dur;
            la->lAlign[i].like=like;
         }
#ifndef PHNALG
         align=NULL;
#endif
      }
   }
}


/* Number/count nodes (in path->usage field) and count links */
static void MarkPaths(Path *path,int *nn,int *nl)
{
   NxtPath *pth;

   if (path->usage>=0) {
      path->usage=-(*nn)++;
      (*nl)++;
      if (path->prev) MarkPaths(path->prev,nn,nl);
      for (pth=path->chain;pth!=NULL;pth=pth->chain) {
         (*nl)++;
         if (pth->prev) MarkPaths(pth->prev,nn,nl);
      }
   }
}

static Lattice *CreateLattice(MemHeap *heap,TokenSet *res,HTime framedur)
{
   Lattice *lat;
   RelToken *cur;
   Path path;
   WordPron pron;
   NxtPath rth[MAX_TOKS];
   int nn,nl,ln,i;
   NetNode node;

   pron.word=NULL;pron.pnum=0;pron.next=NULL;
   pron.outSym=NULL;pron.phones=NULL;pron.nphones=0;
   pron.prob=0.0;
   
   path.like=res->tok.like;
   path.lm=res->tok.lm;
   path.usage=0;
   path.align=res->tok.align;
   path.node=&node;
   path.node->tag=NULL;
   path.node->info.pron=&pron;
   path.frame=pri->frame;
   path.prev=res->tok.path;
   path.chain=NULL;
   if (res->n>1) {
      path.chain=rth+1;
      for (i=1,cur=res->set+1;i<res->n;i++,cur++) {
         rth[i].like=res->tok.like+cur->like;
         rth[i].lm=cur->lm;
         rth[i].prev=cur->path;
#ifdef PHNALG
	 rth[i].align=cur->align;
#endif
         rth[i].chain=NULL;
         rth[i-1].chain=rth+i;
      }
   }

   nn=1;nl=0;ln=0;
   MarkPaths(&path,&nn,&nl);

   lat=NewLattice(heap,nn,nl);
   lat->voc=pri->net->vocab;
   lat->lmscale=pri->scale;
   lat->wdpenalty=pri->wordpen;
   lat->prscale=pri->pscale;
   lat->framedur=framedur;

   lat->lnodes[0].time=0.0; lat->lnodes[0].word=NULL;
   lat->lnodes[0].tag=NULL;
   lat->lnodes[0].score=0.0;

   LatFromPaths(&path,&ln,lat);

#ifdef SANITY
   if (ln!=nl)
      HError(8522,"CreateLattice: Size mismatch (nl (%d) != ln (%d))",nl,ln);
#endif

   return(lat);
}

static void qcksrtM(float *array,int l,int r,int M)
{
   int i,j;
   float x,tmp;

   if (l>=r || l>M || r<M) return;
   x=array[(l+r)/2];i=l-1;j=r+1;
   do {
      do i++; while (array[i]>x);
      do j--; while (array[j]<x);
      if (i<j) {
         tmp=array[i];array[i]=array[j];array[j]=tmp;
      }
   }
   while(i<j);
   if (j<M) qcksrtM(array,j+1,r,M);
   else qcksrtM(array,l,j,M);
}

/* EXPORT->InitVRecInfo: initialise ready for recognition */
VRecInfo *InitVRecInfo(PSetInfo *psi,int nToks,Boolean models,Boolean states)
{
   VRecInfo *vri;
   PreComp *pre;
   int i,n;
   char name[80];
   static int prid=0;

   vri=(VRecInfo*) New(&gcheap,sizeof(VRecInfo));
   sprintf(name,"VRI-%d Heap",prid++);
   CreateHeap(&vri->heap,name,MSTAK,1,1.0,1000,8000);

   pri=(PRecInfo*) New(&vri->heap,sizeof(PRecInfo));
   vri->pri=pri;
   vri->pri->prid=prid;

#ifdef SANITY
   pri->ipos=0;
   pri->start_inst=NULL;
   pri->pnlen = pri->pylen = 0;
   pri->anlen = pri->aylen = 0;
#endif

   /* Reset readable parameters */
   vri->maxBeam=0;
   vri->genBeam=-LZERO;
   vri->wordBeam=-LZERO;
   vri->nBeam=-LZERO;
   vri->tmBeam=LZERO;
   vri->pCollThresh=1024;
   vri->aCollThresh=1024;

   /* Set up private parameters */
   pri->qsn=0;pri->qsa=NULL;
   pri->psi=NULL;
   pri->net=NULL;
   pri->scale=1.0;
   pri->wordpen=0.0;

   /* Could be in StartNetwork ?? */
   pri->states=states;pri->models=models;
   if (nToks<=1) pri->nToks=0;
   else if (nToks<=MAX_TOKS) pri->nToks=nToks;
   else pri->nToks=MAX_TOKS;

   /* SetUp heaps for recognition */

   /* Model set dependent */
   pri->psi=psi;
   /* pri->psi->sBuf[1].n=((pri->nToks>1)?1:0);  Needed every observation */
   for(i=1,pre=psi->sPre+1;i<=psi->nsp;i++,pre++) pre->id=-1;
   for(i=1,pre=psi->mPre+1;i<=psi->nmp;i++,pre++) pre->id=-1;

   pri->stHeap=(MemHeap *) New(&vri->heap,pri->psi->stHeapNum*sizeof(MemHeap));
   for (n=1;n<=pri->psi->max;n++) {
      if (pri->psi->stHeapIdx[n]>=0) {
         sprintf(name,"State Heap: numStates=%d",n);
         CreateHeap(pri->stHeap+pri->psi->stHeapIdx[n],name,
                    MHEAP,sizeof(TokenSet)*n,1.0,100,1600);
      }
   }

   /* nTok dependent */
   if (pri->nToks>1)
      CreateHeap(&pri->rTokHeap,"RelToken Heap",
                 MHEAP,sizeof(RelToken)*pri->nToks,1.0,200,1600);
   /* Non dependent */
   CreateHeap(&pri->instHeap,"NetInst Heap",
              MHEAP,sizeof(NetInst),1.0,200,1600);
   CreateHeap(&pri->rPthHeap,"NxtPath Heap",
              MHEAP,sizeof(NxtPath),1.0,200,1600);
   CreateHeap(&pri->pathHeap,"Path Heap",
              MHEAP,sizeof(Path),1.0,200,1600);
   CreateHeap(&pri->alignHeap,"Align Heap",
              MHEAP,sizeof(Align),1.0,200,3200);


   /* Now set up instances */

   pri->head.node=pri->tail.node=NULL;
   pri->head.state=pri->tail.state=NULL;
   pri->head.exit=pri->tail.exit=NULL;
   pri->head.wdlk=pri->tail.wdlk=LZERO;
   pri->head.max=pri->tail.max=LZERO;
   pri->head.knil=pri->tail.link=NULL;
   pri->head.link=&pri->tail;pri->tail.knil=&pri->head;

   pri->pYesRef.link=&pri->pYesTail;pri->pYesTail.knil=&pri->pYesRef;
   pri->pYesTail.link=pri->pYesRef.knil=NULL;pri->pYesTail.usage=-2;
   pri->pNoRef.link=&pri->pNoTail;pri->pNoTail.knil=&pri->pNoRef;
   pri->pNoTail.link=pri->pNoRef.knil=NULL;pri->pNoTail.usage=-2;
   pri->npth=pri->cpth=0;
   
   pri->aYesRef.link=&pri->aYesTail;pri->aYesTail.knil=&pri->aYesRef;
   pri->aYesTail.link=pri->aYesRef.knil=NULL;pri->aYesTail.usage=-2;
   pri->aNoRef.link=&pri->aNoTail;pri->aNoTail.knil=&pri->aNoRef;
   pri->aNoTail.link=pri->aNoRef.knil=NULL;pri->aNoTail.usage=-2;
   pri->nalign=pri->calign=0;

   return(vri);
}

/* EXPORT->DeleteVRecInfo: Finished with this recogniser */
void DeleteVRecInfo(VRecInfo *vri)
{
   PRecInfo *pri;
   int i;
   
   pri=vri->pri;

   for (i=0;i<pri->psi->stHeapNum;i++)
      DeleteHeap(pri->stHeap+i);
   if (pri->nToks>1)
      DeleteHeap(&pri->rTokHeap);
   DeleteHeap(&pri->instHeap);
   DeleteHeap(&pri->rPthHeap);
   DeleteHeap(&pri->pathHeap);
   DeleteHeap(&pri->alignHeap);
   DeleteHeap(&vri->heap);
   Dispose(&gcheap,vri);
}

/* EXPORT->BeginRecNet: initialise network ready for recognition */
void StartRecognition(VRecInfo *vri,Network *net,
                      float scale,LogFloat wordpen,float pscale)
{
   NetNode *node;
   NetInst *inst,*next;
   PreComp *pre;
   int i;

   pri=vri->pri;
   if (pri==NULL)
      HError(8570,"StartRecognition: Visible recognition info not initialised");
   /* pri->psi->sBuf[1].n=((pri->nToks>1)?1:0);  Only needed for Step1 */

   vri->noTokenSurvived=TRUE;
   pri->net=net;
   pri->scale=scale;
   pri->wordpen=wordpen;
   pri->pscale=pscale;
   /* Initialise the network and instances ready for first frame */
                       
   for (node=pri->net->chain;node!=NULL;node=node->chain) node->inst=NULL;
   pri->net->final.inst=pri->net->initial.inst=NULL;
   for(i=1,pre=pri->psi->sPre+1;i<=pri->psi->nsp;i++,pre++) pre->id=-1;
   for(i=1,pre=pri->psi->mPre+1;i<=pri->psi->nmp;i++,pre++) pre->id=-1;

   pri->tact=pri->nact=pri->frame=0;

   AttachInst(&pri->net->initial);
   inst=pri->net->initial.inst;
   inst->state->tok.like=inst->max=0.0;
   inst->state->tok.lm=0.0;
   inst->state->tok.path=NULL;
   inst->state->n=((pri->nToks>1)?1:0);

   vri->genMaxNode=vri->wordMaxNode=NULL;
   vri->genMaxTok=vri->wordMaxTok=null_token;
   pri->wordThresh=pri->genThresh=pri->nThresh=LSMALL;
   pri->genMaxNode=pri->wordMaxNode=NULL;
   pri->genMaxTok=pri->wordMaxTok=null_token;
   for (inst=pri->head.link;inst!=NULL && inst->node!=NULL;inst=next)
      if (inst->max<pri->genThresh) {
         next=inst->link;
         DetachInst(inst->node);
      }
      else {
         pri->nxtInst=inst;
         StepInst2(inst->node);
         next=pri->nxtInst->link;
      }
}

void ProcessObservation(VRecInfo *vri,Observation *obs,int id, AdaptXForm *xform)
{
   NetInst *inst,*next;
   int j;
   float thresh;

   pri=vri->pri;
   inXForm = xform; /* sepcifies the transform to use for this observation */
   if (pri==NULL)
      HError(8570,"ProcessObservation: Visible recognition info not initialised");
   if (pri->net==NULL)
      HError(8570,"ProcessObservation: Recognition not started");

   pri->psi->sBuf[1].n=((pri->nToks>1)?1:0); /* Needed every observation */
   pri->frame++;
   pri->obs=obs;
   if (id<0) pri->id=(pri->prid<<20)+pri->frame;
   else pri->id=id;

   if (obs->swidth[0]!=pri->psi->hset->swidth[0])
      HError(8571,"ProcessObservation: incompatible number of streams (%d vs %d)",
             obs->swidth[0],pri->psi->hset->swidth[0]);
   if (pri->psi->mixShared)
      for (j=1;j<=obs->swidth[0];j++)
         if (VectorSize(obs->fv[j])!=pri->psi->hset->swidth[j])
            HError(8571,"ProcessObservatio: incompatible stream widths for %d (%d vs %d)",
                   j,VectorSize(obs->fv[j]),pri->psi->hset->swidth[j]);


   /* Max model pruning is done initially in a separate pass */

   if (vri->maxBeam>0 && pri->nact>vri->maxBeam) {
      if (pri->nact>pri->qsn) {
         if (pri->qsn>0)
            Dispose(&vri->heap,pri->qsa);
         pri->qsn=(pri->nact*3)/2;
         pri->qsa=(LogFloat*) New(&vri->heap,pri->qsn*sizeof(LogFloat));
      }
      for (inst=pri->head.link,j=0;inst!=NULL;inst=inst->link,j++)
         pri->qsa[j]=inst->max;
      if (j>=vri->maxBeam) {
         qcksrtM(pri->qsa,0,j-1,vri->maxBeam);
         thresh=pri->qsa[vri->maxBeam];
         if (thresh>LSMALL) 
            for (inst=pri->head.link;inst->link!=NULL;inst=next) {
               next=inst->link;
               if (inst->max<thresh) 
                  DetachInst(inst->node);
            }
      }
   }   
   if (pri->psi->hset->hsKind==TIEDHS)
      PrecomputeTMix(pri->psi->hset,obs,vri->tmBeam,0);
   /* Pass 1 must calculate top of all beams - inc word end !! */
   pri->genMaxTok=pri->wordMaxTok=null_token;
   pri->genMaxNode=pri->wordMaxNode=NULL;
   for (inst=pri->head.link,j=0;inst!=NULL;inst=inst->link,j++)
      if (inst->node)
         StepInst1(inst->node);
   
   /* Not changing beam width for max model pruning */
   
   pri->wordThresh=pri->wordMaxTok.like-vri->wordBeam;
   if (pri->wordThresh<LSMALL) pri->wordThresh=LSMALL;
   pri->genThresh=pri->genMaxTok.like-vri->genBeam;
   if (pri->genThresh<LSMALL) pri->genThresh=LSMALL;
   if (pri->nToks>1) {
      pri->nThresh=pri->genMaxTok.like-vri->nBeam;
      if (pri->nThresh<LSMALL/2) pri->nThresh=LSMALL/2;
   }
   
   /* Pass 2 Performs external token propagation and pruning */
   for (inst=pri->head.link,j=0;inst!=NULL && inst->node!=NULL;inst=next,j++)
      if (inst->max<pri->genThresh) {
         next=inst->link;
         DetachInst(inst->node);
      }
      else {
         pri->nxtInst=inst;
         StepInst2(inst->node);
         next=pri->nxtInst->link;
      }
   
   if ((pri->npth-pri->cpth) > vri->pCollThresh || 
       (pri->nalign-pri->calign) > vri->aCollThresh)
      CollectPaths();

   pri->tact+=pri->nact;

   vri->frame=pri->frame;
   vri->nact=pri->nact;
   vri->genMaxNode=pri->genMaxNode;
   vri->wordMaxNode=pri->wordMaxNode;
   vri->genMaxTok=pri->genMaxTok;
   vri->wordMaxTok=pri->wordMaxTok;
}

/* EXPORT->TracePath: Summarise word history */
void TracePath(FILE *file,Path *path)
{
   MLink ml;
   Align *align;

   if (path->prev!=NULL)
      TracePath(file,path->prev);
   fprintf(file,"%s ",path->node->info.pron->word->wordName->name);
   if (path->align!=NULL) {
      fprintf(file,"{");
      for (align=path->align;align!=NULL;align=align->prev) {
         ml=FindMacroStruct(pri->psi->hset,'h',align->node->info.hmm);
         if (ml==NULL) fprintf(file," !*!");
         else fprintf(file," %s",ml->id->name);
         if (align->state>0) fprintf(file,"[%d]",align->state);
      }
      fprintf(file," }\n");
   }
}

/* EXPORT->CompleteRecognition: Free unused data and return traceback */
Lattice *CompleteRecognition(VRecInfo *vri,HTime frameDur,MemHeap *heap)
{
   Lattice *lat = NULL;
   NetInst *inst;
   TokenSet dummy;
   RelToken rtok[1];
   int i;

   pri=vri->pri;
   if (pri==NULL)
      HError(8570,"CompleteRecognition: Visible recognition info not initialised");
   if (pri->net==NULL)
      HError(8570,"CompleteRecognition: Recognition not started");
   if (pri->frame==0)
      HError(-8570,"CompleteRecognition: No observations processed");

   vri->frameDur=frameDur;
   
   /* Should delay this until we have freed everything that we can */
   if (heap!=NULL) {
      lat=NULL;vri->noTokenSurvived=TRUE;
      if (pri->net->final.inst!=NULL)
         if (pri->net->final.inst->exit->tok.path!=NULL)
            lat=CreateLattice(heap,pri->net->final.inst->exit,vri->frameDur),
               vri->noTokenSurvived=FALSE;
     
      if (lat==NULL && forceOutput) {
         dummy.n=((pri->nToks>1)?1:0);
         dummy.tok=pri->genMaxTok;
         dummy.set=rtok;
         dummy.set[0].like=0.0;
         dummy.set[0].path=dummy.tok.path;
         dummy.set[0].lm=dummy.tok.lm;
         lat=CreateLattice(heap,&dummy,vri->frameDur);
      }
   }

   /* Now dispose of everything apart from the answer */
   for (inst=pri->head.link;inst!=NULL;inst=inst->link)
      if (inst->node)
         inst->node->inst=NULL;

   /* Remove everything from active lists */

   pri->head.link=&pri->tail;pri->tail.knil=&pri->head;
   pri->npth=pri->cpth=0;
   pri->nalign=pri->calign=0;
   pri->nact=pri->frame=0;

   pri->pYesRef.link=&pri->pYesTail;pri->pYesTail.knil=&pri->pYesRef;
   pri->pNoRef.link=&pri->pNoTail;pri->pNoTail.knil=&pri->pNoRef;
   pri->npth=pri->cpth=0;
   
   pri->aYesRef.link=&pri->aYesTail;pri->aYesTail.knil=&pri->aYesRef;
   pri->aNoRef.link=&pri->aNoTail;pri->aNoTail.knil=&pri->aNoRef;
   pri->nalign=pri->calign=0;

   vri->frame=0;
   vri->nact=0;
   vri->genMaxNode=NULL;
   vri->wordMaxNode=NULL;
   vri->genMaxTok=null_token;
   vri->wordMaxTok=null_token;

   if (pri->nToks>1)
      ResetHeap(&pri->rTokHeap);
   ResetHeap(&pri->instHeap);
   for (i=0;i<pri->psi->stHeapNum;i++)
      ResetHeap(pri->stHeap+i);

   ResetHeap(&pri->alignHeap);
   ResetHeap(&pri->rPthHeap);
   ResetHeap(&pri->pathHeap);
   
   return(lat);
}

/* EXPORT->SetPruningLevels: Set pruning levels for following frames */
void SetPruningLevels(VRecInfo *vri,int maxBeam,LogFloat genBeam,
                      LogFloat wordBeam,LogFloat nBeam,LogFloat tmBeam)
{
   vri->maxBeam=maxBeam;
   vri->genBeam=genBeam;
   vri->wordBeam=wordBeam;
   vri->nBeam=nBeam;
   vri->tmBeam=tmBeam;
}

/* Lattice output routines.  Note lattices need to be sorted before output */

typedef struct nbestentry NBestEntry;

struct nbestentry {
   NBestEntry *link;
   NBestEntry *knil;
   NBestEntry *prev;

   double score;
   double like;
   LNode *lnode;
   LArc *larc;
};

static void MarkBack(LNode *ln,int *nn)
{
   LArc *la;

   ln->n=-2;
   for (la=ln->pred;la!=NULL;la=la->parc)
      if (la->start->n==-1) MarkBack(la->start,nn);
   ln->n=(*nn)++;
}

static Boolean WordMatch(NBestEntry *cmp,NBestEntry *ans)
{
   if (cmp==ans) return(TRUE);
   else if (cmp==NULL || ans==NULL) return(FALSE);
   else if (cmp->larc->end->word!=ans->larc->end->word) return(FALSE);
   else return(WordMatch(cmp->prev,ans->prev));
}

/* EXPORT->TranscriptionFromLattice: Generate NBest labels from lattice */
Transcription *TranscriptionFromLattice(MemHeap *heap,Lattice *lat,int N)
{
   Transcription *trans;
   LabList *ll;
   LLink lab,where;
   LabId model;
   Word word, nullWord;
   Pron pron;
   NBestEntry head,tail,**ans,*best,*newNBE,*pos;
   LArc *la;
   LNode *ln;
   LAlign *lal;
   LogFloat lm,modlk;
   double score,like,start,end;
   Boolean states,models;
   int i,j,n,nAux,*order;
   int nexp=0,nent=0;

   ans=(NBestEntry**) New(&gstack,sizeof(NBestEntry*)*N);ans--;

   for (i=0,ln=lat->lnodes;i<lat->nn;i++,ln++) {
      if (ln->foll==NULL) ln->score=0.0;
      else ln->score=LZERO;
      ln->n=-1;
   }
   n=0;
   for (i=0,ln=lat->lnodes;i<lat->nn;i++,ln++)
      if (ln->n==-1) MarkBack(ln,&n);

   order=(int*) New(&gstack, sizeof(int)*lat->nn);
   for (i=0,ln=lat->lnodes;i<lat->nn;i++,ln++)
      order[ln->n]=i;
   for (i=0,la=lat->larcs;i<lat->na;i++,la++) 
      if (la->start->n>la->end->n)
         HError(8522,"TranscriptionFromLattice: Arcs not properly directed");
   for (i=lat->nn-1;i>0;i--) {
      ln=lat->lnodes+order[i];
      for (la=ln->pred;la!=NULL;la=la->parc) {
         score=ln->score+LArcTotLike(lat,la);
         if (score>la->start->score) la->start->score=score;
      }
   }
   Dispose(&gstack,order);
 
   /* Then do NBest AStar for real answers */

   head.link=&tail;head.knil=NULL;
   tail.link=NULL;tail.knil=&head;
   tail.score=head.score=LZERO;

   for (i=0,ln=lat->lnodes;i<lat->nn;i++,ln++) {
      if (ln->pred!=NULL) continue;
      if (ln->score<LSMALL)
         HError(8522,"TranscriptionFromLattice: No route through lattice");
      for (la=ln->foll;la!=NULL;la=la->farc) {
         like=LArcTotLike(lat,la);
         score=like+la->end->score;
         if (score<LSMALL) continue;
         newNBE=(NBestEntry*) New(&gstack,sizeof(NBestEntry));
         newNBE->score=score;
         newNBE->like=like;
         newNBE->lnode=la->end;
         newNBE->larc=la;
         newNBE->prev=NULL;
         for (pos=head.link;score<pos->score;pos=pos->link);
         newNBE->knil=pos->knil;newNBE->link=pos;
         newNBE->knil->link=newNBE->link->knil=newNBE;
      }
   }
   for (n=0,best=head.link;n<N && best!=&tail;best=head.link) {
      if (head.link==&tail) break;
      best=head.link;
      best->link->knil=best->knil;
      best->knil->link=best->link;
      nent--;

      if (best->lnode->foll!=NULL) {
         nexp++;
         for (la=best->lnode->foll;la!=NULL;la=la->farc) {
            like=best->like+LArcTotLike(lat,la);
            score=like+la->end->score;
            if (score<LSMALL) continue;
            newNBE=(NBestEntry*) New(&gstack,sizeof(NBestEntry));
            newNBE->score=score;
            newNBE->like=like;
            newNBE->lnode=la->end;
            newNBE->larc=la;
            newNBE->prev=best;
            for (pos=head.link;score<pos->score;pos=pos->link);
            newNBE->knil=pos->knil;newNBE->link=pos;
            newNBE->knil->link=newNBE->link->knil=newNBE;
            nent++;
         }
         continue;
      }
      for (i=1;i<=n;i++)
         if (WordMatch(best,ans[i])) {
            best=NULL;
            break;
         }
      if (best!=NULL) {
         ans[++n]=best;
      }
   }

   nullWord=GetWord(lat->voc, GetLabId("!NULL", FALSE), FALSE);
   trans=CreateTranscription(heap);
   for (i=1;i<=n;i++) {
      states=models=FALSE;
      /* Note initial and final nodes are !NULL so ignore these !! */
      for (pos=ans[i]->prev;pos!=NULL;pos=pos->prev) {
         if (pos->larc->end->word==nullWord) continue;
         if (pos->larc->lAlign==NULL) {
            states=models=FALSE;
            break;
         }
         for (j=0,lal=pos->larc->lAlign;j<pos->larc->nAlign;j++,lal++)
            if (lal->state<0)
               models=TRUE;
            else if (lal->state>0)
               states=TRUE;
      }
      nAux=(states?1:0)+(models?1:0);
      ll=CreateLabelList(heap,nAux);
      if (nAux>0) {
         for (pos=ans[i]->prev;pos!=NULL;pos=pos->prev) {
            la=pos->larc;
            lal=la->lAlign;
            word=la->end->word;model=NULL;
            if (word == nullWord) continue;
            lm=LArcTotLMLike(lat,la); modlk=0.0;
            start=la->start->time*1.0E7;
            where=ll->head->succ;
            for (j=0,lal=la->lAlign;j<la->nAlign;j++,lal++) {
               if (lal->state<0 && states) {
                  model=lal->label;
                  modlk=lal->like;
                  lab=NULL;
                  continue;
               }
               lab=CreateLabel(heap,ll->maxAuxLab);
               lab->labid=lal->label;
               lab->score=lal->like;
               end=start+lal->dur*1.0E7;
               lab->start=start;
               lab->end=end;
               lab->pred=where->pred;lab->succ=where;
               lab->succ->pred=lab->pred->succ=lab;
               start=end;

               if (word==NULL)
                  lab->auxLab[nAux]=NULL;
               else
                  lab->auxLab[nAux]=word->wordName;
               word=NULL;
               lab->auxScore[nAux]=lm;lm=0.0;
               if (models && states) {
                  lab->auxLab[1]=model;model=NULL;
                  lab->auxScore[1]=modlk;modlk=0.0;
               }
            }
         }
      }
      else {
         for (pos=ans[i]->prev;pos!=NULL;pos=pos->prev) {
            la=pos->larc;
            for (pron=la->end->word->pron;pron!=NULL;pron=pron->next)
               if (pron->pnum==la->end->v) break;
            if (pron==NULL || pron->outSym==NULL || pron->outSym->name[0]==0)
               continue;
            if (la->end->word == nullWord)
               continue;
            lab=CreateLabel(heap,ll->maxAuxLab);
            lab->labid=pron->outSym;
            lab->score=LArcTotLike(lat,la);
            lab->start=la->start->time*1.0E7;
            lab->end=la->end->time*1.0E7;
            lab->succ=ll->head->succ;lab->pred=ll->head;
            lab->succ->pred=lab->pred->succ=lab;
         }
      }
      AddLabelList(ll,trans);
   }

   ans++;Dispose(&gstack,ans);

   if (trace&T_NGEN)
      printf("HLat:      %d NBest generation %d exp, %d ent\n",N,nexp,nent);

   return(trans);
}

/* EXPORT->FormatTranscription: Format transcription prior to output */
void FormatTranscription(Transcription *trans,HTime frameDur,
                         Boolean states,Boolean models,Boolean triStrip,
                         Boolean normScores,Boolean killScores,
                         Boolean centreTimes,Boolean killTimes,
                         Boolean killWords,Boolean killModels)
{
   LabList *ll;
   LLink lab;
   HTime end;
   char buf[MAXSTRLEN],*p,tail[64];
   int lev,j,frames;
   
   if (killScores) {
      for (lev=1;lev<=trans->numLists;lev++) {
         ll=GetLabelList(trans,lev);
         for(lab=ll->head->succ;lab->succ!=NULL;lab=lab->succ) {
            lab->score=0.0;
            for (j=1;j<=ll->maxAuxLab;j++)
               lab->auxScore[j]=0.0;
         }
      }
   }
   if (triStrip) {
      for (lev=1;lev<=trans->numLists;lev++) {
         ll=GetLabelList(trans,lev);
         for(lab=ll->head->succ;lab->succ!=NULL;lab=lab->succ) {
            if (states && !models) {
               strcpy(buf,lab->labid->name);
               if ((p=strrchr(buf,'['))!=NULL) {
                  strcpy(tail,p);
                  *p=0;
               }
               else
                  *tail=0;
               TriStrip(buf); strcat(buf,tail);
               lab->labid=GetLabId(buf,TRUE);
            }
            else {
               strcpy(buf,lab->labid->name);
               TriStrip(buf); lab->labid=GetLabId(buf,TRUE);
            }
            for (j=1;j<=ll->maxAuxLab;j++) {
               if (lab->auxLab[j]==NULL) continue;
               strcpy(buf,lab->auxLab[j]->name);
               TriStrip(buf); lab->auxLab[j]=GetLabId(buf,TRUE);
            }
         }
      }
   }
   if (normScores) {
      for (lev=1;lev<=trans->numLists;lev++) {
         ll=GetLabelList(trans,lev);
         for(lab=ll->head->succ;lab->succ!=NULL;lab=lab->succ) {
            frames=(int)floor((lab->end-lab->start)/frameDur + 0.4);
            if (frames==0) lab->score=0.0;
            else lab->score=lab->score/frames;
            if (states && models && ll->maxAuxLab>0 && lab->auxLab[1]!=NULL) {
               end=AuxLabEndTime(lab,1);
               frames=(int)floor((end-lab->start)/frameDur + 0.4);
               if (frames==0) lab->auxScore[1]=0.0;
               else lab->auxScore[1]=lab->auxScore[1]/frames;
            }
         }
      }
   }
   if (killTimes) {
      for (lev=1;lev<=trans->numLists;lev++) {
         ll=GetLabelList(trans,lev);
         for(lab=ll->head->succ;lab->succ!=NULL;lab=lab->succ) {
            lab->start=lab->end=-1.0;
         }
      }
   }
   if (centreTimes) {
      for (lev=1;lev<=trans->numLists;lev++) {
         ll=GetLabelList(trans,lev);
         for(lab=ll->head->succ;lab->succ!=NULL;lab=lab->succ) {
            lab->start+=frameDur/2;
            lab->end-=frameDur/2;
         }
      }
   }
   if (killWords) {
      for (lev=1;lev<=trans->numLists;lev++) {
         ll=GetLabelList(trans,lev);
         if (ll->maxAuxLab>0)
            for(lab=ll->head->succ;lab->succ!=NULL;lab=lab->succ)
               lab->auxLab[ll->maxAuxLab]=NULL;
      }
   }
   if (killModels && models && states) {
      for (lev=1;lev<=trans->numLists;lev++) {
         ll=GetLabelList(trans,lev);
         if (ll->maxAuxLab==2)
            for(lab=ll->head->succ;lab->succ!=NULL;lab=lab->succ) {
               lab->auxLab[1]=lab->auxLab[2];
               lab->auxScore[1]=lab->auxScore[2];
               lab->auxLab[2]=NULL;
            }
      }
   }
}

/* ------------------------ End of HRec.c ------------------------- */
