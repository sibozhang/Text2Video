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
/*         File: Arc.h -- Routines used in HFwdBkwdLat.c       */
/*         An alternative kind of lattice format used there.   */
/* ----------------------------------------------------------- */

/* !HVER!HArc:   3.4.1 [CUED 12/03/09] */


/*
   Turns a Lattice into Arc structure.  
*/   

/* start basic stuff: */

typedef struct _Arc HArc;
typedef struct _ArcTrans ArcTrans;




struct _ArcTrans{
  HArc *start;
  HArc *end;

  ArcTrans *start_foll; /*The transition is in a dll w.r.t both the arcs it follows and the ones it precedes.*/
  ArcTrans *start_prec; /*the start_{foll,prec} dll is the list of the transitions attached to the same start node; */
  ArcTrans *end_foll;  /*similarly for the end_{foll,prec}. */
  ArcTrans *end_prec; 
  
  LogFloat lmlike;  /*This is the lmprob + the prlike [not scaled].*/
  LogFloat locc;
};



typedef struct _CorrN CorrN; /* for exact MPE. */
typedef struct _CorrA CorrA;
struct _CorrA{
  CorrN *start;
  CorrN *end;
  CorrA *start_foll;
  CorrA *end_foll;
  LogFloat sc_lmlike;
};

struct _CorrN{

  CorrN *next;
  CorrN *prev;
  CorrA *follTrans;
  CorrA *precTrans;
  short int align; 
  int nArcs; /* normally 1, but e.g.  if quinphone or perhaps MWE, can be more. */
  HArc *me_start; HArc *me_end;/* first & last arc. */
  
  Boolean IsSilence; /* Probably just start & end. */

  float scaled_aclike; /* scaled. */
  double alpha;

  int iphone; /* (int) correct phone. */
  int starti;  /* start & end of search beam. */
  int endi;
  float *alphaCorr; /* [starti..endi]. */
  float *betaCorr; /* [starti..endi]. */
  double *beta; /* [starti..endi]. */
};


typedef struct _Acoustic{
  HArc *myArc; 
 
  int Nq;
  int t_start, t_end;
  LogDouble aclike; /* [unscaled] acoustic likelihood from beginning to end of model. */
  LogFloat locc;
  float mpe_occscale; /* scale on the occupancy [for MPE code], equals
		         accuracy - average accuracy of file. */

  HLink hmm;

  Boolean SP; /*short pause.  If (SP) rest of variables are NULL.*/
  DVector alphat; /* 1..Nq */
  DVector alphat1; /* 1..Nq  [for time t-1] */
  DVector *betaPlus;  /* [myArc->t_start..myArc->t_end][1..Nq] */
  float ****otprob; /* [myArc->t_start..myArc->t_end][0..(S>1?S:0)][2..Nq-1][0..(M>1)?M:0] */

} Acoustic;  /* for calculating acoustic likelihoods... */

typedef struct _MPEStruct{
  float correctness;  /* for inexact MPE. */
  float alphaError, betaPlusError; /*for MPE training.*/
  CorrN *cn; /* if doing dynamic MPE. */
} MPEStruct;

struct _Arc{

  /* For acoustic stuff: */
  HLink hmm; /*For comparison, to see whether the physical hmm is the same.  Only used (at time of writing) for the exact-time
	       (ExactMatch) version.  But is used anyway in HFwdBkwdLat.c */
#define GHOST_ARC -1
  int id;   /*numbering from 1..nArcs for arcs which will be used for forward-backward alignment; 
	      otherwise GHOST_ARC */
  HArc *calcArc; /*An identical arc (in hmm, start, end), which we use for purposes of forward-backward alignment [only if id==0].*/
  Acoustic *ac; 

  /* relating to the phone & word... */
  int pos;       /*the position of this phone in the word arc.*/
  LArc *parentLarc; /*the word arc in the Lattice from which this was derived.*/
  LabId word; 
  LabId phone;         /* phone-in-context. */


  /* start & end frame. */
  int t_start;
  int t_end;

  ArcTrans *follTrans;  /*Arcs which follow.*/
  ArcTrans *precTrans;

  /* phone identifier for phone aligned to-- 
     an int of the kind used in HFwdBkwd.c to test phone equality.  
     ?? Used only in HFwdBkwdLat.c. For DynamicMPE, contains correct phone. */

  HArc *foll;
  HArc *prec;

  /* following are user: */
  
  double alpha; /*alpha and betaPlus are for user's use.  */
  double betaPlus;

  MPEStruct *mpe; /*if doing mpe. */

};


#define MAXLATS 10
/*Will usually be only 1 or 2-- i.e, recognised lat plus aligned correct-transcription lattice.*/

typedef struct ArcInfoStruct{
  int nLats;
  Boolean numLatIncluded; /*if TRUE and this is a denominator lat, means the num lat
			    has been included as the last one.*/
  Lattice *lat[MAXLATS]; /*only needed if we are creating an Arc. Array starts from zero.*/
  MemHeap *mem;
  int nArcs;
  HArc *start; /*The beginning and end of a dll of arcs.*/
  HArc *end;
  float lmScale; /*This functions as a prscale (pronunciation
		   prob scale) as well since we combine the two.*/
  float insPen; 
  float framedur;
  int Q;       /*number of unique arcs.. */
  int T;       /* time is 1..T. */
  Acoustic *ac; /* 1..Q */
  int *qLo;     /* [t], lowest q active at time t */
  int *qHi;     /* [t], highest q active at time t */
}ArcInfo;



void ArcFromLat(ArcInfo *aInfo,  HMMSet *hset); 
/*Takes a ArcInfo with the 'nLats', 'lat' and 'mem' in place, and creates the arcs .*/
/*Note that the lmScale is taken from the first 'lat'.
  If desired it can be changed afterwards.*/


void PrintArcInfo(FILE *f, ArcInfo *aInfo); 


int TimeToNFrames(float time, ArcInfo *aInfo); 
/* 
   divides the time by the frame duration of the lattice and 
   returns it as an int.  Note, the frame duration will currently
   have to be set with the config e.g. FRAMEDUR=0.01, since lats
   do not contain it. 
   To turn a word/phone start time in the lattice to a frame,
   do GetFrameLength and add 1.  For an end time, just call
   GetFrameLength (start/end times in the Arc structure are stored 
   as the first and last frames of the phone).
 */

/* ------------------------- End of Arc.h --------------------------- */

void AttachMPEInfo(ArcInfo *aInfo); /* attaches the "mpe" fields  */

Boolean LatInLat(Lattice *numLat, Lattice *denLat);

#define StartOfWord(a) (a->pos==0)
#define EndOfWord(a) (a->pos == a->parentLarc->nAlign-1)

void InitArc(void);
