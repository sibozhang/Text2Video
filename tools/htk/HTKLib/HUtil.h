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
/*         File: HUtil.h      HMM utility routines             */
/* ----------------------------------------------------------- */

/* !HVER!HUtil:   3.4.1 [CUED 12/03/09] */

#ifndef _HUTIL_H_
#define _HUTIL_H_

#ifdef __cplusplus
extern "C" {
#endif

typedef struct{   /* HMMSet Scan State */
   HMMSet *hset;     /* HMM set */
   Boolean isCont;   /* true PLAINHS or SHAREDHS */
   int h;            /* current mtab slot */
   MLink mac;        /* current macro */
   HLink hmm;        /* -> current hmm */
   int N;            /* num states in curret hmm */
   int i;            /* current state index 2..N-1 */
   StateElem *se;    /* ->current stateElem */
   StateInfo *si;    /* ->current stateInfo */
   int S;            /* num Streams = hset->swidth[0] */
   int s;            /* current stream index 1..S */
   StreamElem *ste;  /* ->current streamElem */
   int M;            /* num mixtures */
   int m;            /* current mixture index 1..M */
   /* ------ continuous case only ------------ */
   MixtureElem *me;  /* ->current mixtureElem */
   MixPDF *mp;       /* ->current mixPDF */
}HMMScanState;
   
   
void InitUtil(void);
/*
   Initialise the module
*/

/* EXPORT->ResetUtilItemList: frees all the memory from the ItelList heap */
void ResetUtilItemList();

/* EXPORT->SetParsePhysicalHMM: only parse physical HMMs */
void SetParsePhysicalHMM(Boolean parse);

/* ---------------- General Purpose Routines ----------------- */

SVector CloneSVector(MemHeap *hmem, SVector s, Boolean sharing);
SMatrix CloneSMatrix(MemHeap *hmem, SMatrix s, Boolean sharing);
STriMat CloneSTriMat(MemHeap *hmem, STriMat s, Boolean sharing);

MixPDF *CloneMixPDF(HMMSet *hset, MixPDF *s, Boolean sharing);
MixtureVector CloneStream(HMMSet *hset, StreamElem *ste, Boolean sharing);
StateInfo *CloneState(HMMSet *hset, StateInfo *ssi, Boolean sharing);

void CloneHMM(HLink src, HLink tgt, Boolean sharing);
/*
   The src HMM is copied into tgt HMMDef which must already exist.  
   If sharing, then any macros are simply shared, otherwise the complete 
   structure is copied.
*/

void ConvDiagC(HMMSet *hset, Boolean convData);
/*
  Converts all the HMMs in the HMMSet hset to the INVDIAGC from DIAGC
  or vice versa. If convData is TRUE then each variance element is
  replaced by its reciprocal - otherwise only the CovKind in each HMM
  is changed and no data conversions are performed.
*/

void ForceDiagC(HMMSet *hset);
/*
  Converts all the HMMs in the HMMSet hset to DIAGC from INVDIAGC.
*/

void ConvLogWt(HMMSet *hset);
/*
  Converts all the mixture weights into log-weights.
*/

void ConvExpWt(HMMSet *hset);
/*
  Converts all the mixture log-weights into weights.
*/

/* ------------------ HMM Scan Routines -------------------- */

void NewHMMScan(HMMSet *hset, HMMScanState *hss);
/*
   Initialise scan of given HMM set.  The scan record hss is set to
   reference the first mix of the first stream of the first state of
   the first physical HMM definition in the set.
*/

void EndHMMScan(HMMScanState *hss);
/*
   During scanning, the nUse fields are set -ve once seen.  This
   call resets all nUse fields back to their normal state
*/ 

Boolean GoNextHMM(HMMScanState *hss);
/*
   Move to the next physical HMM definition.  On normal completion,
   the scan record is set to reference the first mix of the first
   stream of the first state of the current HMM (whether seen already
   or not).  When all HMMs have been seen, GoNextHMM returns false and
   hss.hmm is NULL.
*/

Boolean GoNextState(HMMScanState *hss, Boolean noSkip);
/*
   Move to the next unseen state and mark it seen.  If current state
   is unseen then nothing happens except that state is marked as seen.
   On normal completion, the scan record is set to reference the first
   mix of the first stream of the current state (whether seen already
   or not).  When all states in all HMMs have been seen, GoNextState
   returns false and hss.se is NULL.  If noSkip then returns false
   as soon as all states in current HMM have been seen.
*/

Boolean GoNextStream(HMMScanState *hss, Boolean noSkip);
/*
   Move to next unseen stream (StreamElem) and mark it seen.  If
   current state is unseen then nothing happens except that stream is
   marked as seen.  On normal completion, the scan record is set to
   reference the first mix of the current stream(whether seen already
   or not).  When all streams of all HMMs have been seen, GoNextStream
   returns false and hss.ste is NULL.  If noSkip then returns false
   as soon as all streams in current state have been seen.
*/

Boolean GoNextMix(HMMScanState *hss, Boolean noSkip);
/*
   Move to next as yet unseen mixture (MixtureElem).  When all
   mixtures of all HMMs have been seen, GoNextMix returns false and
   hss.me is NULL.  If noSkip then returns false as soon
   as all mixture components in current stream have been seen.
*/

/* --------------------- HMM trace routines -------------------- */

char *HMMPhysName(HMMSet *hset,HLink hmm);
/*
   Return name of given hmm from HMMSet.
   Aborts if model does not exist.
*/

/* --------------------- Item List Handling -------------------- */

/* 
   Handling for lists of HMM structures.  All Items are created on
   a separate MHEAP created during InitUtil() and so can be freed in
   any order.
*/

void AddItem(HLink owner, Ptr item, ILink *list);
int NumItems(ILink list);
void FreeItems(ILink *list);
/*
   Functions to add, count and free items in a list.
*/

/* ----------------------- Integer Sets --------------------- */

/*
   Handling for sets of integers.  All sets are created on a
   spearate MSTAK created during InitUtil() and so must be freed
   in the order in which they were created.
*/

typedef struct {		/* Defines a set of integers */
   int nMembers;		/* cardinality of set */
   Boolean *set;		/* array[1..nMembers] of Boolean */
}IntSet;

IntSet CreateSet(int size);
void FreeSet(IntSet s);

/*
   Functions to create and destroy IntSets must be called in
   FILO order.
*/

void AddMember(IntSet s, int x);
Boolean IsMember(IntSet s, int x);
Boolean IsFullSet(IntSet s);
void ClearSet(IntSet s);
void SetSet(IntSet s);

/*
   Functions to set and clear IntSet members flags
*/

/* -------------------- Item List Parser -------------------- */

/*
   Used to convert textual representation of HMM structures 
   into a linked list of items holding pointers to those
   structures
*/

/*
   Item lists are used to define a set of HMM structure components.
   The syntax of an itemlist is 
   
   itemlist =  "{" itemset {"," itemset} "}"
   itemset  =  hname "." ["transP" | "state" state]
   state    =  index ["." statecomp]]
   statecomp=  "dur" | "weights" | ["stream" index "."] "mix" [mix]
   mix      =  index [ "." ("mean" | "cov")]
   hname    =  ident | identlist
   identlist=  "(" ident {"," ident} ")"
   ident    =  <char | metachar>
   metachar =  "?" | "*"
   index    = "[" numset "]"
   numset   =  intrange { "," intrange }
   intrange = integer [ "-" integer]
   
   eg. {(*aa*,*ah*).state[2].stream[3].mix[1-3].cov} denotes the cov 
   components of all mixes of state 2 stream 3 of all 3 mix models 
   with aa or ah in their name.
   
*/

char *PItemList(ILink *ilist, char *type, HMMSet *h,
		Source *s, Boolean itrace);

/* 
   Parse source s and convert into itemlist ilist and type holding
   matching items from HMMSet h and return pattern parsed.
   If itrace is true enable tracing whilst producing this list.
*/

/* ------------------- Generic macro handling ----------------- */

/*
   Functions to provide access to macros independent of type
*/

Ptr GetMacroHook(MLink ml);
void SetMacroHook(MLink ml,Ptr hook);
int GetMacroUse(MLink ml);
void SetMacroUse(MLink ml,int use);

void ResetHooks(HMMSet *hset,char *what);
/* 
   Reset (to NULL) the hook fields of all macros (if what==NULL)
   or just those whose type appears in what (if what!=NULL).
*/

/* ------------------- Load Statistics File  --------------------- */

void LoadStatsFile(char *statfile,HMMSet *hset,Boolean otrace);
/*
   Load the statistics file output by HERest into state hooks
*/

#ifdef __cplusplus
}
#endif

#endif  /* _HUTIL_H_ */

/* ------------------------- End of HUtil.h --------------------------- */
