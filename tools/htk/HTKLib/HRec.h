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
/*         File: HRec.h  Viterbi Recognition Engine Library    */
/* ----------------------------------------------------------- */

/* !HVER!HREC:   3.4.1 [CUED 12/03/09] */

#ifndef _HREC_H_
#define _HREC_H_

#ifdef __cplusplus
extern "C" {
#endif

/* LIMITS - Only the maximum total number of tokens that can be used */
#define MAX_TOKS 1024


/* Each HMMSet that is used for recognition needs to be    */
/* initialised prior to use.  Initialisation routine adds  */
/* necessary structues to models and returns PSetInfo used */
/* by Viterbi recogniser to access model set.              */
typedef struct psetinfo PSetInfo; /* Private HMMSet information (HRec.c) */


/* Each recognition requires a PRecInfo to maintain status   */
/* information thoughtout the utterance.  The majority of    */
/* the structure is private although some status information */
/* and all pruning parameters are available between frames.  */
typedef struct vrecinfo VRecInfo; /* Visible recognition information */

typedef struct precinfo PRecInfo; /* Private reconition information (HRec.c) */

/* Traceback information takes the form of Path structures */

typedef struct path Path;          /* Traceback route */

/* Alignment and NBest traceback is not visible outside HRec */
typedef struct nxtpath NxtPath;    /* NBest traceback route (HRec.c) */
typedef struct align Align;        /* Alignment route (HRec.c) */

/* Tokens are reasonably standard except for extra Align field */
typedef struct token
{
   LogDouble like;	/* Likelihood of token */
   LogFloat lm;         /* LM likelihood of token */
   Path *path;		/* Route (word level) through network */
   Align *align;        /* Route (state/model level) through network */
}
Token;
extern const Token null_token; /* Null token is part of HRec.c */

struct path
{
   Path *prev;		/* Previous word record */
   LogDouble like;      /* Likelihood at boundary */
   LogFloat lm;         /* LM likelihood of current word */

   NxtPath *chain;      /* Next of NBest Paths */
   
   /* Pron pron;		Word level traceback info */
   NetNode *node;       /* Word level traceback info */
   int frame;           /* Time (frame) of boundary (end of word) */
   Align *align;        /* State/model traceback for this word */

   Boolean used;        /* Reference to struct by current inst */
   int usage;           /* Times struct ref'd (by next path) */

   Path *link;          /* Next path in list */
   Path *knil;          /* Prev path in list */
};

/* Visible recognition information */

struct vrecinfo
{
   MemHeap heap;            /* General storage CHEAP (inc vri) */
   HTime frameDur;          /* Sample rate (to convert frame to time) */
   Boolean noTokenSurvived; /* Set when no valid final token produced */

   /* Pruning thresholds - setable every frame */

   int maxBeam;             /* Maximum model instance beam */
   LogFloat genBeam;        /* Global beam width */
   LogFloat wordBeam;       /* Separte word end beam width */
   LogFloat nBeam;          /* Beam width for non-best tokens */
   LogFloat tmBeam;         /* Beam width for tied mixtures */
   int pCollThresh;         /* Max path records created before collection */
   int aCollThresh;         /* Max align records created before collection */

   /* Status information - readable every frame */

   int frame;               /* Current frame number */
   int nact;                /* Number of active models */

   NetNode *genMaxNode;     /* Most likely node in network */
   NetNode *wordMaxNode;    /* Most likely word end node in network */

   Token genMaxTok;         /* Most likely token */
   Token wordMaxTok;        /* Most likely word end token */

   PRecInfo *pri;           /* Private recognition information */
                            /*  associated with network/hmmset */
};

void InitRec(void);
/* 
   Initialise module
*/

/*
   Functions specific to HMMSet

   Providing calls to ProcessObservation are not made simultaneously
   and that each recogniser's VRecInfo is separately initialised
   multiple recognisers can share HMMSets.  Use of unique observation 
   ids allows correct caching of observation output probabilities.
*/

PSetInfo *InitPSetInfo(HMMSet *hset);
/*
   Attach precomps to HMMSet and return PSetInfo
   describing HMMSet
*/

void FreePSetInfo(PSetInfo *psi);
/*
   Free storage allocated by InitPSetInfo
*/


/*
   Functions specific to each recogniser started
   Each recogniser that needs to be run in parallel must be separately
   initialised
*/

VRecInfo *InitVRecInfo(PSetInfo *psi,int nToks,Boolean models,Boolean states);
/* 
   Initialise a recognition engine attached to a particular HMMSet
   that will use nToks tokens per state and also perform alignment
   at states or models level
*/

void DeleteVRecInfo(VRecInfo *vri);
/*
   Detach recogniser and free storage
*/

/* 
   Each utterance is processed by calling StartRecognition, then
   ProcessObservation for each frame and finally traceback is
   performed by calling EndRecognition.
*/

/* EXPORT->StartRecognition: initialise network ready for recognition */

void StartRecognition(VRecInfo *vri,Network *net,
                      float scale,LogFloat wordpen,float pscale);
/*
   Commence recognition using previously initialised recogniser using
   supplied network and language model scale and word insertion penalty
*/

void ProcessObservation(VRecInfo *vri,Observation *obs,int id, AdaptXForm *xform);
/*
   Process a single observation updating traceback and status
   information in vri.  Each call to ProcessObservation should
   provide an id value unique to a particular observation.
*/

Lattice *CompleteRecognition(VRecInfo *vri,HTime frameDur,MemHeap *heap);
/*
   Create lattice with traceback and then free recognition data
*/

void SetPruningLevels(VRecInfo *vri,int maxBeam,LogFloat genBeam,
		      LogFloat wordBeam,LogFloat nBeam,LogFloat tmBeam);
/*
   At any time after initialisation pruning levels can be set
   using SetPruningLevels or by directly altering the vri values
*/

Transcription *TranscriptionFromLattice(MemHeap *heap,Lattice *lat,int N);
/*
   Convertion of lattice to nbest label file uses simple AStar search.
*/

void FormatTranscription(Transcription *trans,HTime frameDur,
                         Boolean states,Boolean models,Boolean triStrip,
                         Boolean normScores,Boolean killScores,
                         Boolean centreTimes,Boolean killTimes,
                         Boolean killWords,Boolean killModels);
/*
   Format a label transcription prior to calling LSave
*/

void TracePath(FILE *file,Path *path);
/*
   Output to file the sequence of words in path
*/

#ifdef __cplusplus
}
#endif

#endif  /* _HREC_H_ */
