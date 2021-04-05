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
/*              2002  Cambridge University                     */
/*                    Engineering Department                   */
/*                                                             */
/*   Use of this software is governed by a License Agreement   */
/*    ** See the file License for the Conditions of Use  **    */
/*    **     This banner notice must not be removed      **    */
/*                                                             */
/* ----------------------------------------------------------- */
/*         File: HTrain.h   HMM Training Support Routines      */
/* ----------------------------------------------------------- */

/* !HVER!HTrain:   3.4.1 [CUED 12/03/09] */

#ifndef _HTRAIN_H_
#define _HTRAIN_H_

#ifdef __cplusplus
extern "C" {
#endif

enum _UPDSet{UPMEANS=1,UPVARS=2,UPTRANS=4,UPMIXES=8,UPXFORM=16,UPMAP=32,UPSEMIT=64};
typedef enum _UPDSet UPDSet;

/* Win32 modification */
#ifdef WIN32
#define finite _finite
#endif

void InitTrain(void);
/*
   Initialise the module
*/

/* ------------------ Generic Sequence Type --------------- */

typedef struct _ItemBlock *IBLink;
typedef struct _ItemBlock{
   int used;         /* num items in this block */
   IBLink next;      /* next block in chain */
   Ptr *items;       /* array[0..blkSize-1] of items */
}ItemBlock;

typedef struct {
   MemHeap *mem;     /* memory stack for this sequence */
   int nItems;       /* number of items stored */
   int nFree;        /* number of items slots in total */
   int blkSize;      /* size of an item block */
   IBLink hd;        /* head of list of item blocks */
   IBLink tl;        /* tail of list of item blocks */
}SequenceInfo;

typedef SequenceInfo *Sequence;

Sequence CreateSequence(MemHeap *x, int blkSize);
/* 
   Create and return an empty sequence.  Each sequence is accessed by
   an index in range 1..nItems.  The blkSize affects efficiency, it
   should be large for random access to long sequences and small if
   items are frequently deleted.  The MemHeap must be a stack.  There
   is no Dispose operation, use ResetHeap(x) instead.  Sequences can
   also be used for unordered sets.
*/

void StoreItem(Sequence seq, Ptr item);
/* 
   Add item to end of sequence
*/

void DeleteItem(Sequence seq, int i);
/*
   Delete i'th Item from seq. 
*/

Ptr GetItem(Sequence seq, int i);
/*
   Return ptr to the i'th item in sequence
*/

void SequenceMean(Sequence ss, Vector mean);
/*
   Compute the mean of the set (sequence) of vectors stored in ss.  
*/
   
void SequenceCov(Sequence ss, CovKind ck, Covariance cov, Vector mean);
/*
   Compute the (co)variance of the set (sequence) of vectors stored 
   in ss whose mean is given by mean.  The form of the covariance
   is determined by ck which must be DIAGC,INVDIAGC or FULLC.
*/

/* ------------ Segment (Observation Sequence) Storage ------------ */

typedef struct _SegStoreRec * SegStore;
typedef struct _SegStoreRec{
   MemHeap *mem;        /* memory for this seg store */
   int segLen;          /* blkSize for each segment */
   Observation o;       /* used as an 'i/o channel' to segstore */
   Boolean hasfv;
   Boolean hasvq;
   Sequence fvSegs;     /* each seg is a sequence of fv[SMAX] */
   Sequence vqSegs;     /* each seg is a sequence of vq[SMAX] */
}SegStoreRec;

SegStore CreateSegStore(MemHeap *x, Observation obs, int segLen);
/* 
   Create and return an empty segment store suitable for observations
   of form obs.  Each segment is an ordered sequence of observations with
   blkSize == segLen.  A segment store is a sequence of segments with
   blkSize = 100.
*/

void LoadSegment(SegStore ss, HTime start, HTime end, ParmBuf pbuf);
/*
   Extract the segment of observations in pbuf covering the time span
   start to end then create a sequence in ss containing them.
*/

int NumSegs(SegStore ss);
/*
   Return number of segments in ss
*/

int SegLength(SegStore ss, int i);
/*
   Return the number of observations in the i'th segment
*/

Observation GetSegObs(SegStore ss, int i, int j);
/*
   Return j'th observation from i'th segment.  All indices 1..N
*/

/* --------------------- Vector Clustering -------------------- */

/* Clusters are represented by an array of Cluster records 
   In linear case, each record represents a cluster.
   In tree case, each record represents a node of the tree in
   which cluster 1 is always the root node and the remaining
   nodes are stored in row order.
*/

typedef struct {  /* auxiliary info for each cluster */
   int csize;      /* num items in this cluster */
   Vector vCtr;    /* cluster centre vector */
   Covariance cov; /* covariance of cluster */
   float aveCost;  /* average cost of cluster */
}Cluster;

typedef struct {
   MemHeap *x;     /* memheap holding this set */
   Boolean isTree; /* true if tree clustered */
   int numClust;   /* num cluster nodes */
   CovKind ck;     /* type of covariance if any */
   Cluster *cl;    /* array[1..numClust]of Cluster */
}ClusterSet;

ClusterSet *FlatCluster(MemHeap *x, Sequence vpool, int nc, 
                        CovKind dck, CovKind cck, Covariance dcov);
ClusterSet *TreeCluster(MemHeap *x, Sequence vpool, int nc, 
                        CovKind dck, CovKind cck, Covariance dcov);
/*
   Create and return a set of clusters allocated in x.  vpool holds
   the set of vectors to be split into nc distinct clusters using a
   top-down clustering algorithm.  The cov kind dck determines the
   distance measure used:
       dck==NULLC,  euclidean distance, dcov is ignored
       dck==INVDIAGC, diagonal covariance mahalanobis
       dck==FULLC,    full covariance mahalanobis
   The cov kind cck determines what type of covariance, if any is
   computed at each node.  This choice does not affect the clustering,
   only the subsequent use of the cluster nodes.  The max number of
   iterations per cluster cycle (default 10) can be set by the
   configuration variable MAXCLUSTITER.  MemHeap must be a stack.
*/

void FreeClusterSet(ClusterSet *cs);
/*
   Free the stack storage used by the given cs.
*/

void ShowClusterSet(ClusterSet *cs);
/*
   Print the contents of given cluster set
*/

/* ------------------------- Accumulators ---------------------- */

typedef struct {     /* attached to transP */
   Matrix tran;      /* array[1..N][1..N]of transition count */
   Vector occ;       /* array[1..N] of state occupation */
   int minDur;       /* Min no of frames to get through trans mat */
} TrAcc;

typedef struct {     /* attached to StreamElem */
   Vector c;         /* array[1..M] of mixture weight */
   float occ;        /* occ for states sharing this pdf */
   float *prob;      /* PreComputed mixture Log Probs */
   int   time;       /* time for which prob is valid */
} WtAcc;

typedef struct {     /* attached to mean vector */
   Vector mu;        /* mean vector counts */
   float occ;        /* occ for states sharing this mpdf */
} MuAcc;

typedef struct {     /* attached to covariance vector/matrix */
   Covariance cov;   /* covariance counts */
   float occ;        /* occ for states sharing this mpdf */
} VaAcc;


#define MIX_UPDATE_SHARING

typedef struct {     /* attached to MixPDF */
   LogFloat prob;    /* PreComputed Mixture Log Prob */
   int   time;       /* time for which prob is valid */
#ifdef MIX_UPDATE_SHARING
   int indx;
#endif
} PreComp;


void AttachAccsParallel(HMMSet *hset, MemHeap *x, UPDSet uFlags, int indx);
void AttachAccs(HMMSet *hset, MemHeap *x, UPDSet uFlags);
/*
   Attach zeroed accumulators to given HMM set, also attaches PreComps
   Equals AttachAccsParallel (hset,x,1).
*/

void ZeroAccsParallel(HMMSet *hset, UPDSet uFlags, int indx); /* if indx +ve, does 0.. indx-1; else does -indx. */
void ZeroAccs(HMMSet *hset, UPDSet uFlags);
/*
   Zero all accumulators in given HMM set.
*/

void ShowAccsParallel(HMMSet *hset, UPDSet uFlags, int index);
void ShowAccs(HMMSet *hset, UPDSet uFlags);
/*
   Show all accumulators attached to given HMM set
*/

void AttachPreComps(HMMSet *hset, MemHeap *x);
/*
   Attach reset PreComps to given HMM set
*/

void ResetPreComps(HMMSet *hset);
/*
   Reset all the precomputed prob fields in the
   given HMM set.
*/

void ResetHMMPreComps(HLink hmm, int nStreams);
/*
   Reset all the precomputed prob fields in the
   given HMM.
*/

void ResetHMMWtAccs(HLink hmm, int nStreams);
/*
   Reset all the wt accs associated with a
   given HMM.
*/

FILE * DumpAccsParallel(HMMSet *hset, char *fname, int n, UPDSet uFlags, int index);
FILE * DumpAccs(HMMSet *hset, char *fname, UPDSet uFlags, int n);
/* 
   Dump a copy of the accumulators attached to hset
   in file fname.  Any occurrence of the $ symbol in
   fname is replaced by n. The file is left open 
   and returned to allow extra info to be written.
*/ 

Source LoadAccsParallel(HMMSet *hset, char *fname, UPDSet uFlags, int index);
Source LoadAccs(HMMSet *hset, char *fname, UPDSet uFlags);
/* 
   Increment the accumulators attached to hset by adding
   the values stored in fname.   Accs must be newly 
   created before first call. The file is left open 
   and returned to allow extra info to be read.
*/

void RestoreAccsParallel(HMMSet *hset, int index);
void RestoreAccs(HMMSet *hset);
/* 
   Restore accs by removing the mean offset.
*/

double ScaleAccsParallel(HMMSet *hset, float weight, int index);
double ScaleAccs(HMMSet *hset, float weight);
/*
   Scales all the accumulators.  Returns summed occupancy.
*/

extern Boolean strmProj;
/* 
   Controls whether an  stream projection transform is generated.
   Not elegant, but saves multiple config specifications etc.
*/

#ifdef __cplusplus
}
#endif

#endif  /* _HTRAIN_H_ */

/* ---------------------- End of HTrain.h ---------------------- */
