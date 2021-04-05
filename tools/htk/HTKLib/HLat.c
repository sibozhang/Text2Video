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
/* author: Gunnar Evermann <ge204@eng.cam.ac.uk>               */
/* ----------------------------------------------------------- */
/*         Copyright:                                          */
/*         2001-2004  Cambridge University                     */
/*                    Engineering Department                   */
/*                                                             */
/*   Use of this software is governed by a License Agreement   */
/*    ** See the file License for the Conditions of Use  **    */
/*    **     This banner notice must not be removed      **    */
/*                                                             */
/* ----------------------------------------------------------- */
/*       File: HLat.c:  Lattice Manipulation                   */
/* ----------------------------------------------------------- */

/*#### todo:

     - implement lattice oracle WER calculation
     - allow batch processing?
*/


char *hlat_version = "!HVER!HLat:   3.4.1 [CUED 12/03/09]";
char *hlat_vc_id = "$Id: HLat.c,v 1.2 2006/12/07 11:09:08 mjfg Exp $";


#include "HShell.h"
#include "HMem.h"
#include "HMath.h"
#include "HWave.h"
#include "HAudio.h"
#include "HParm.h"
#include "HLabel.h"
#include "HModel.h"
#include "HUtil.h"
#include "HDict.h"
#include "HNet.h"
#include "HLM.h"
#include "HLat.h"

/* ----------------------------- Trace Flags ------------------------- */

#define T_TOP  00001
#define T_PRUN 00002
#define T_FB   00004
#define T_EXP  00010
#define T_MEM  00020
#define T_TRAN 00040
#define T_LLF  00100
#define T_MRG  00200

static int trace=0;
static ConfParam *cParm[MAXGLOBS];      /* config parameters */
static int nParm = 0;

/* --------------------------- Global Flags -------------------------- */

static LabId startWord;         /* word at start of Lattice (!SENT_START) */
static LabId endWord;           /* word at end of Lattice (!SENT_END) */
static LabId startLMWord;       /* word at start in LM (<s>) */
static LabId endLMWord;         /* word at end in LM (</s>) */
static LabId nullWord;          /* null word in Lattices (!NULL) */
static Boolean beamPruneArcs = TRUE; /* apply beam pruning to arcs (rather than just nodes) */
static Boolean compressMerge = TRUE; /* compressing lattice scores when merging duplicates */

static char *llfExt = "LLF";    /* extension for LLF lattice files */

static MemHeap slaHeap, slnHeap;/* MHEAPs for use in LatExpand() */

/* --------------------------- Prototypes ---------------------------- */


#ifndef NO_LAT_LM
typedef struct _SubLNode SubLNode;
typedef struct _SubLArc SubLArc;

struct _SubLNode {
   union {
      LMState lmstate;
      LNode *newln;
   } data;
   SubLArc *foll;
   SubLNode *next;
};

struct _SubLArc {
   LogFloat lmprob;
   SubLNode *end;
   LArc *la;
   SubLArc *next;
};
#endif

/* --------------------------- LLF processing ---------------------- */

typedef struct _LLFInfo LLFInfo;
struct _LLFInfo {
   LLFInfo *next;
   char name[MAXFNAMELEN];
   Source source;
   int lastAccess;
};


static int numLLFs = 0; 
static int maxLLFs = 5; 
static int numLatsLoaded = 0;
static LLFInfo *llfInfo = NULL;

static MemHeap llfHeap;


void CloseLLF (LLFInfo *llf)
{
   if (trace&T_LLF)
      printf ("Closing LLF %s\n", llf->name);
   CloseSource (&llf->source);
   llf->name[0] = '\0';
   llf->lastAccess = 0;
}


LLFInfo *OpenLLF (char *fn)
{
   LLFInfo *llf;
   Source s;
   char buf[MAXFNAMELEN];

   if (trace&T_LLF)
      printf ("Opening LLF %s\n", fn);

   if (InitSource (fn, &s, NetFilter) < SUCCESS) {
      HError(-8630,"OpenLLF: Cannot open LLF %s", fn);
      return NULL;
   }
   if (!ReadStringWithLen (&s, buf, MAXFNAMELEN) ||
       strcmp (buf, "#!LLF!#")) {   /* no LLF header or read from pipe failed */
      HError(-8630,"OpenLLF: Cannot read from LLF %s", fn);
      return NULL;
   }

   if (numLLFs < maxLLFs) {
      ++numLLFs;
      llf = New (&llfHeap, sizeof (LLFInfo));
      llf->next = llfInfo;
      llfInfo = llf;
   }
   else {
      LLFInfo *l;

      /* find oldest (least recently accessed) LLF */
      llf = llfInfo;
      for (l = llfInfo->next; l; l = l->next)
         if (l->lastAccess < llf->lastAccess)
            llf = l;

      CloseLLF (llf);
   }

   strcpy (llf->name, fn);
   llf->source = s;

   return llf;
}

Boolean ScanLLF (LLFInfo *llf, char *fn, char *ext)
{
   char buf[MAXFNAMELEN];
   char latfn[MAXFNAMELEN];

   llf->lastAccess = numLatsLoaded;
   MakeFN (fn, NULL, ext, latfn);

   while (ReadStringWithLen (&llf->source, buf, MAXFNAMELEN)) {
      if (!strcmp(buf, latfn)) {   /* found name */
         return TRUE;
      }
      if (trace&T_LLF)
         printf ("ScanLLF: skipping '%s'\n", buf);
      ReadUntilLine (&llf->source, ".");   /* skip this lattice */
   }
   HError (-1, "ScanLLF: lattice '%s' not found in LLF '%s'\n", latfn, llf->name);
   return FALSE;
}


Lattice *GetLattice (char *fn, char *path, char *ext,
                     /* arguments of ReadLattice() below */
                     MemHeap *heap, Vocab *voc, 
                     Boolean shortArc, Boolean add2Dict)
{
   Lattice *lat;
   LLFInfo *llf;
   char llfName[MAXFNAMELEN];
   char buf[MAXFNAMELEN];

   MakeFN (path, NULL, llfExt, llfName);

   /* check whether LLF is open already */
   for (llf = llfInfo; llf; llf = llf->next) {
      if (!strcmp (llfName, llf->name))
         break;
   }

   if (!llf) {   /* not found -> try to open LLF */
      llf = OpenLLF (llfName);

      if (!llf) {       /* can't find LLF -> fall back to single file lattices */
         char latfn[MAXFNAMELEN];
         FILE *f;
         Boolean isPipe;

         MakeFN (fn, path, ext, latfn);
         if ((f = FOpen(latfn, NetFilter, &isPipe)) == NULL)
            HError(8632,"GetLattice: Cannot open Lattice file %s", latfn);
         lat = ReadLattice (f, heap, voc, shortArc, add2Dict);
         FClose(f, isPipe);
         return lat;
      }
   }

   /* scan for lattice with requested name in LLF */
   ++numLatsLoaded;
   if (!ScanLLF (llf, fn, ext)) {
      /* this may be because it's missing, or there's an error in the order */
      CloseSource (&llf->source);
      /* LLF must exist open and try again */
      if (InitSource (llfName, &llf->source, NetFilter) < SUCCESS) {   
         HError(8630,"OpenLLF: Cannot open LLF %s", llfName);
      }
      if (!ReadStringWithLen (&llf->source, buf, MAXFNAMELEN) ||
          strcmp (buf, "#!LLF!#")) {   /* no LLF header or read from pipe failed */
         HError(-8630,"OpenLLF: Cannot read from LLF %s", fn);
         return NULL;
      }
      if (!ScanLLF (llf, fn, ext)) {
         /* is not definitely missing */ 
         HError (8632, "ScanLLF: lattice not found in LLF\n");
      }
   }

   /* note we do not support sub lattices here, thus call ReadOneLattice()  directly */
   lat = ReadOneLattice (&llf->source, heap, voc, shortArc, add2Dict);

   return lat;
}


/* --------------------------- Initialisation ---------------------- */

/* EXPORT->InitLat: register module & set configuration parameters */
void InitLat(void)
{
   int i;
   Boolean b;
   char buf[MAXSTRLEN];

   Register(hlat_version,hlat_vc_id);
   nParm = GetConfig("HLAT", TRUE, cParm, MAXGLOBS);
   if (nParm>0){
      if (GetConfInt(cParm,nParm,"TRACE",&i)) trace = i;
      if (GetConfBool(cParm,nParm,"BEAMPRUNEARCS",&b)) 
         beamPruneArcs = b;
      if (GetConfBool(cParm,nParm,"COMPRESSMERGE",&b)) 
         compressMerge = b;
      if (GetConfStr(cParm,nParm,"LLFEXT",buf))
         llfExt = CopyString(&gstack,buf);
      if (GetConfInt(cParm,nParm,"MAXLLFS",&i)) maxLLFs = i;
   }

   CreateHeap (&llfHeap, "LLF stack", MSTAK, 1, 1.0, 1000, 10000);
#ifndef NO_LAT_LM
   CreateHeap (&slaHeap, "LatExpand arc heap", MHEAP, sizeof (SubLArc), 1.0, 1000, 128000);
   CreateHeap (&slnHeap, "LatExpand node heap", MHEAP,sizeof (SubLNode), 1.0, 1000, 32000);
#endif
}


/* --------------------------- Lattice processing ------------------- */

/* LatCheck

     check lattice for consistency: single start & end nodes; no cycles.
*/
void LatCheck (Lattice *lat)
{
   int i, nStart, nEnd;
   LNode *ln;
   LNode **topOrder;

   /* check for unique start and end nodes */
   nStart = nEnd = 0;
   for (i = 0, ln = lat->lnodes; i < lat->nn; ++i, ++ln) {
      if (!ln->pred)
         ++nStart;
      if (!ln->foll)
         ++nEnd;
   }

   if (nStart != 1)
      HError (8622, "HLat: lattice has %d start nodes (should be 1)", nStart);
   if (nEnd != 1)
      HError (8622, "HLat: lattice has %d end nodes (should be 1)", nEnd);

   /* check wheter lat is a DAG ( <=> top order exists). */
   topOrder = (LNode **) New (&gcheap, lat->nn * sizeof(LNode *));
   if (!LatTopSort (lat, topOrder))
      HError (8622, "HLat: lattice contains cylces");

   Dispose (&gcheap, topOrder);
}


/* FixPronProbs

     replace pronunciation probabilities in lattices with values taken from
     the dictionary
*/
void FixPronProbs (Lattice *lat, Vocab *voc)
{
   int i, v;
   LNode *ln;
   Pron pron;
   LArc *la;

   for (i = 0, ln = lat->lnodes; i < lat->nn; ++i, ++ln) {
      if (ln->word != voc->nullWord) {
         if (ln->v > ln->word->nprons)
            HError (8621, "FixPronprbs: lattice refers to non-existing pron");
         pron = ln->word->pron;
         for (v = 2; v <= ln->v; ++v)
            pron = pron->next;
         if (pron->pnum != ln->v)
            HError (8621, "FixPronprbs: dict pron numbering invalid");
         
         for (la = ln->pred; la; la = la->parc)
            la->prlike = pron->prob;
      }
      else {
         for (la = ln->pred; la; la = la->parc)
            la->prlike = 0.0;
      }
   }
}


/* LatStartNode

     return lattice start node

     ###GE: maybe store this in Lattice structure to save time?
*/
LNode *LatStartNode (Lattice *lat)
{
   int i;
   LNode *ln;

   for (i = 0, ln = lat->lnodes; i < lat->nn; ++i, ++ln)
      if (!ln->pred)
         return ln;

   HError (8622, "HLat: lattice has no start node");
   return NULL;         /* make compiler happy */
}

/* LatEndNode

     return lattice end node

     ###GE: maybe store this in Lattice structure to save time?
*/
LNode *LatEndNode (Lattice *lat)
{
   int i;
   LNode *ln;

   for (i = 0, ln = lat->lnodes; i < lat->nn; ++i, ++ln)
      if (!ln->foll)
         return ln;

   HError (8622, "HLat: lattice has no end node");
   return NULL;         /* make compiler happy */
}


/* helper function for LatTopSort()
   number preceeding nodes depth first 
*/
void LatTopSortVisit (LNode *ln, int *time)
{
   LArc *la;

   ln->n = -2;          /* mark node as seen, but not numbered, yet (GRAY in CLR) */
   for (la = ln->pred; la; la = la->parc)
      if (la->start->n == -1)
         LatTopSortVisit (la->start, time);
   
   ++(*time);
   ln->n = *time;
}

/* LatTopSort

     sort lattice nodes in topological order
     returns array of LNode pointers.
     uses depth first traversal (in reverse (pred) direction) 
     based on [CLR:1990], p. 485 
*/
Boolean LatTopSort (Lattice *lat, LNode **topOrder)
{
   int time = -1;       /* new numbering will start at 0 */
   int i;
   LNode *ln;
   LArc *la;
   Boolean isDAG = TRUE;

   for (i = 0, ln = lat->lnodes; i < lat->nn; ++i, ++ln)
      ln->n = -1;        /* mark node as unseen (WHITE in CLR) */

   LatTopSortVisit (LatEndNode (lat), &time);
   
   /* we should have seen all nodes */
   assert (time+1 == lat->nn); 

   /* check topological order */
   for (i = 0, la = lat->larcs; i < lat->na; ++i, ++la)
      if (!(la->start->n < la->end->n)) {
         isDAG = FALSE;
         HError (-8622, "LatTopSort: Lattice contains cycles"); 
         break;
      }

   for (i = 0, ln = lat->lnodes; i < lat->nn; ++i, ++ln)
      topOrder[ln->n] = ln;
   
   /*#### GE: reset ln->n values? */
   for (i = 0, ln = lat->lnodes; i < lat->nn; ++i, ++ln)
      ln->n = i;

   return isDAG;
}

/* LatAttachInfo

     allocate & attach an Info structre for each node
*/
void LatAttachInfo (MemHeap *heap, size_t size, Lattice *lat)
{
   int i;
   LNode *ln;
   
   for (i = 0, ln = lat->lnodes; i < lat->nn; ++i, ++ln)
      ln->hook = (Ptr) New (heap, size);
}

/* LatDetachInfo

     free Info structre for each node
*/
void LatDetachInfo (MemHeap *heap, Lattice *lat)
{
   int i;
   LNode *ln;
   
   for (i = 0, ln = lat->lnodes; i < lat->nn; ++i, ++ln)
      Dispose (heap, ln->hook);
}


/* LatForwBackw

     perform forward-backward algorithm on lattice and store scores in
     FBInfo structre
     choice of using sum (LATFB_SUM) or max (LATFB_MAX) of scores
*/
LogDouble LatForwBackw (Lattice *lat, LatFBType type)
{
   int i;
   LNode *ln;
   LArc *la;
   LNode **topOrder;
   LogDouble score;

   /* We assume that the FBinfo structures are already allocated. */
   /* init */
   for (i = 0, ln = lat->lnodes; i < lat->nn; ++i, ++ln) {
      LNodeFw (ln) = LNodeBw (ln) = LZERO;
   }
   LNodeFw (LatStartNode (lat)) = 0.0;
   LNodeBw (LatEndNode (lat)) = 0.0;

   /* find topological order of nodes */
   topOrder = (LNode **) New (&gcheap, lat->nn * sizeof(LNode *));
   if (!LatTopSort (lat, topOrder))
      HError (8622, "LatForwBackw: cannot calculate forw/backw score on Lattice with cycles"); 
      
   /* the processing in the forward and backward directions are really
      almost the same. if the data structures had been done _slightly_
      nicer this could be done in one loop. The only readable way now 
      would be defining a couple of macros... */

   /* forward direction */
   for (i = 0; i < lat->nn; ++i) {
      ln = topOrder[i];
      for (la = ln->foll; la; la = la->farc) {
         assert (la->start == ln);
         
         score = LNodeFw (ln) + LArcTotLike (lat, la);
         switch (type) {
         case LATFB_SUM:
            LNodeFw (la->end) = LAdd (LNodeFw (la->end), score);
            break;
         case LATFB_MAX:
            if (score > LNodeFw (la->end))
               LNodeFw (la->end) = score;
            break;
         default:
            abort ();
         }
      }
   }

   /* backward direction */
   for (i = lat->nn - 1; i >= 0; --i) {
      ln = topOrder[i];
      for (la = ln->pred; la; la = la->parc) {
         assert (la->end == ln);
         
         score = LNodeBw (ln) + LArcTotLike (lat, la);
         switch (type) {
         case LATFB_SUM:
            LNodeBw (la->start) = LAdd (LNodeBw (la->start), score);
            break;
         case LATFB_MAX:
            if (score > LNodeBw (la->start))
               LNodeBw (la->start) = score;
            break;
         default:
            abort ();
         }
      }
   }

   if (trace & T_FB) {
      printf ("forward prob:  %f\n", LNodeFw (topOrder[lat->nn - 1]));
      printf ("backward prob: %f\n", LNodeBw (topOrder[0]));
   }
   score = LNodeBw (topOrder[0]);
   Dispose (&gcheap, topOrder);

   return score;
}

/* EXPORT->LatFindBest

     find the N-best paths (i.e. lowest sum of LArcTotLike()s) and generate
     Transcription.

     ####GE: currently only N=1 is supported
*/
Transcription *LatFindBest (MemHeap *heap, Lattice *lat, int N)
{
   int i;
   LNode *ln;
   LNode **topOrder;
   LArc *la;
   LogDouble score, ac, lm, pr, tot;
   Word nullWord;
   Pron pron;
   Transcription *trans;
   LabList *ll;
   LLink lab;
   
   if (N != 1)
      HError (8690, "FindBest: only 1-best supported, yet.");

   /* during the search ln->score will hold the score of the best
      path to ln (i.e. lowest score). ln->hook will point to
      the arc leading to the preceeding node in this path */

   /* init fields */
   for (i = 0, ln = lat->lnodes; i < lat->nn; ++i, ++ln) {
      ln->score = LZERO;
      ln->hook = NULL;
   }
   LatStartNode (lat)->score = 0.0;

   /* find topological order of nodes */
   topOrder = (LNode **) New (&gcheap, lat->nn * sizeof(LNode *));
   if (!LatTopSort (lat, topOrder))
      HError (8690, "LatFindBest: cannot find best path in Lattice with cycles");

   assert (topOrder[0] == LatStartNode (lat));

   /* traverse nodes in top order */
   for (i = 0; i < lat->nn; ++i) {
      ln = topOrder[i];
      
      /* for all outgoing arcs: propagate scores forward */
      for (la = ln->foll; la; la = la->farc) {
         assert (la->start == ln);
         score = la->start->score + LArcTotLike (lat, la);
         if (score > la->end->score) {
            la->end->score = score;
            la->end->hook = (Ptr) la;
         }
      }
   }

   /* create traqnscription */
   trans = CreateTranscription (heap);
   ll = CreateLabelList (heap, 0);
   
   nullWord = lat->voc->nullWord;

   ac = lm = pr = tot = 0;
   if (trace & T_TRAN)
      printf ("1best trans (wordPron t ac lm pr tot): ");

   /* backtrack from end node along best path and generate transcription */
   ln = LatEndNode (lat);
   la = (LArc *) ln->hook;
   while (la) {
      LabId outlab;

      if (ln->word != nullWord) {
         for (pron = ln->word->pron; pron; pron = pron->next)
            if (pron->pnum == ln->v) 
               break;
         if (pron)
            outlab = pron->outSym;
         else   /* if we can't find pronvar (e.g. wlist), fall back to word */
            outlab = ln->word->wordName;
         if (outlab) {
            lab = CreateLabel (heap, ll->maxAuxLab);
            lab->labid = outlab;
            lab->start = la->start->time * 1.0e7;
            lab->end = ln->time * 1.0e7;
            lab->score = LArcTotLike (lat, la);
            
            lab->succ = ll->head->succ;
            lab->pred = ll->head;
            lab->succ->pred = lab->pred->succ = lab;
         }
      }

      ac += la->aclike;
      lm += la->lmlike;
      pr += la->prlike;
      tot += LArcTotLike (lat, la);
      if (trace & T_TRAN)
         printf ("(%s%d %.2f %.3f %.3f %.3f %.3f) ", ln->word->wordName->name, ln->v,
                 ln->time, la->aclike, la->lmlike, la->prlike, LArcTotLike (lat, la));


      ln = la->start;
      la = (LArc *) ln->hook;
   }
   AddLabelList (ll, trans);

   if (trace & T_TRAN) {
      printf ("\n");
      printf ("ac lm pr tot: %.3f %.3f %.3f %.3f\n", ac, lm, pr, tot);
   }

   Dispose (&gcheap, topOrder);
   return trans;
}

/* EXPORT->LatSetScores

     set ln->score values to node posterior to make WriteLattice() deterministic
*/
void LatSetScores (Lattice *lat)
{
   LogDouble best;
   LNode *ln;
   int i;

   LatAttachInfo (&gcheap, sizeof (FBinfo), lat);

   best = LatForwBackw (lat, LATFB_MAX);

   for (i = 0, ln = lat->lnodes; i < lat->nn; ++i, ++ln) {
      ln->score = LNodeFw (ln) + LNodeBw (ln);
   }

   LatDetachInfo (&gcheap, lat);
}


/* EXPORT->LatPrune

     prune lattice by removing all paths with score more than 
     thresh lower than the best path.
*/
Lattice *LatPrune (MemHeap *heap, Lattice *lat, LogDouble thresh, float arcsPerSec)
{
   LogDouble best, limit, score;
   LNode *ln, *newln;
   LArc *la, *newla;
   int i, nn, na;
   Lattice *newlat;

   LatAttachInfo (&gcheap, sizeof (FBinfo), lat);

   best = LatForwBackw (lat, LATFB_MAX);
   limit = best - thresh;
   
   /* modify thresh according to arcPerSec limit */
   if (arcsPerSec > 0) {
#define NBIN 1000
      HTime length;
      int nArc, nArcLimit, bin;
      int hist[NBIN];
      float binWidth;

      length = LatEndNode (lat)->time;
      nArcLimit = length * arcsPerSec;

      binWidth = thresh / NBIN;

      for (i = 0; i < NBIN; ++i)
         hist[i] = 0;

      nArc = 0;
      for (i = 0, la = lat->larcs; i < lat->na; ++i, ++la) {
         score = LNodeFw (la->start) + LArcTotLike (lat, la) + LNodeBw (la->end);
         bin = (best - score) / binWidth;
         assert (bin >= 0);
         if (bin < NBIN) {     /* keep */
            ++hist[bin];
            ++nArc;
         }
      }
      
      if (nArc > nArcLimit) {
         nArc = 0;
         for (i = 0; i < NBIN; ++i) {
            nArc += hist[i];
            if (nArc > nArcLimit)
               break;
         }
         thresh = i * binWidth;
         limit = best - thresh;
         if (trace &T_PRUN)
            printf ("length %.2f nArcLimit %d beam adjusted to %f\n", length, nArcLimit, thresh);
      }
   }

   nn = na = 0;

   /* scan nodes, count survivors and verify consistency */
   for (i = 0, ln = lat->lnodes; i < lat->nn; ++i, ++ln) {
      score = LNodeFw (ln) + LNodeBw (ln);
      if (score >= limit) {       /* keep */
         ln->n = nn;
         ++nn;
      }
      else {                    
         ln->n = -1;
      }
   }

   /* scan arcs and count survivors */
   for (i = 0, la = lat->larcs; i < lat->na; ++i, ++la) {
      if (beamPruneArcs) {
         score = LNodeFw (la->start) + LArcTotLike (lat, la) + LNodeBw (la->end);
         if (score >= limit) {     /* keep */
            la->score = (float) na;
            ++na;
         }
         else                   /* remove */
            la->score = -1.0;
      }
      else {                    /* only prune arc if either node is dead */
         if (la->start->n != -1 && la->end->n != -1) {  /* both nodes are alive => keep arc */
            la->score = (float) na;
            ++na;
         }
         else                   /* remove */
            la->score = -1.0;
      }
   }

#if 0
   /*   SANITY check */
   for (i = 0, ln = lat->lnodes; i < lat->nn; ++i, ++ln) {
      if (ln->n == -1) {
         /* check that there are no live arcs attached to dead nodes */
         for (la = ln->foll; la; la = la->farc) {
            if (la->score >= 0.0)
               HError (8691, "LatPrune: arc score (%f) better than node score (%f)\n", 
                       LNodeFw (la->start) + LArcTotLike (lat, la) +
                       LNodeBw (la->end), score);
         }
      }
   }
#endif 
   
   if (trace & T_PRUN)
      printf ("lattice pruned from %d/%d to %d/%d\n", lat->nn, lat->na, nn, na);

   /* build new lattice from remaining nodes/arcs */
   newlat = NewILattice (heap, nn, na, lat);

   /* copy all the nodes we want to keep */
   newln = newlat->lnodes;
   for (i = 0, ln = lat->lnodes; i < lat->nn; ++i, ++ln) {
      if (ln->n != -1) {
         *newln = *ln;
         /* init linnked lists */
         newln->foll = newln->pred = NULL;

         ++newln;
      }
   }
   assert (newln == newlat->lnodes + nn);

   newla = newlat->larcs; 
   for (i = 0, la = lat->larcs; i < lat->na; ++i, ++la) {
      if (la->score >= 0.0) {     /* keep */
         *newla = *la;
         newla->start = newlat->lnodes + ((int) la->start->n);
         newla->end = newlat->lnodes + ((int) la->end->n);
         
         /* insert newla into foll list */
         newla->farc = newla->start->foll;
         newla->start->foll = newla;
         /* ...and into pred list */
         newla->parc = newla->end->pred;
         newla->end->pred = newla;

         ++newla;
      }
   }
   assert (newla == newlat->larcs + na);

   LatDetachInfo (&gcheap, lat);

   return newlat;
}


/* StatsInfo structure attached to all LNodes */
typedef struct _StatsInfo {
   LogDouble nPaths;     /* number of paths from start node */
} StatsInfo;

#define LNodeStats(ln)  (((StatsInfo *) (ln)->hook))


/* CalcStats

     calculate and output some global statistics for a lattice
*/
void CalcStats (Lattice *lat)
{
   LNode **topOrder;
   LNode *ln;
   LArc *la;
   int i, d, max_inDegree, max_outDegree, nWords;
   LogDouble nPaths;
   Boolean isDAG;
   LNode *lnStart, *lnEnd;
   Word word;

   /* attach StatsInfo structre to nodes */
   LatAttachInfo (&gcheap, sizeof (StatsInfo), lat);
   for (i = 0, ln = lat->lnodes; i < lat->nn; ++i, ++ln)
      LNodeStats(ln)->nPaths = 0;

   /* find topological order of nodes */
   topOrder = (LNode **) New (&gcheap, lat->nn * sizeof(LNode *));
   isDAG = LatTopSort (lat, topOrder);

   lnStart = isDAG ? topOrder[0] : LatStartNode (lat);
   lnEnd = isDAG ? topOrder[lat->nn-1] : LatEndNode (lat);

   max_inDegree = max_outDegree = 0;
   LNodeStats(lnStart)->nPaths = 1;

   /* reset word counters */
   for (i = 0; i< VHASHSIZE; i++)
      for (word = lat->voc->wtab[i]; word != NULL; word = word->next)
         word->aux = (Ptr) 0;

   /* iterate over all nodes */
   for (i = 0; i < lat->nn; ++i) {
      ln = topOrder[i];

      /* count words */
      ln->word->aux = (Ptr) (((int)ln->word->aux) + 1);

      /* count incoming and outgoing arcs */
      d = 0;
      for (la = ln->pred; la; la = la->parc)
         ++d;
      if (d > max_inDegree)
         max_inDegree = d;

      d = 0;
      for (la = ln->foll; la; la = la->farc)
         ++d;
      if (d > max_outDegree)
         max_outDegree = d;
      
      if (isDAG) {
         /* propagate nPaths forward */
         nPaths = LNodeStats(ln)->nPaths;
         for (la = ln->foll; la; la = la->farc)
            LNodeStats(la->end)->nPaths += nPaths;
      }
   }

   /* find number of words seen in lattice */
   nWords = 0;
   for (i = 0; i < VHASHSIZE; i++)
      for (word = lat->voc->wtab[i]; word != NULL; word = word->next) {
         if (word->aux)
            ++nWords;
         word->aux = (Ptr) 0;
      }


   nPaths = LNodeStats(topOrder[lat->nn-1])->nPaths;

   printf("length[s]      %.2f\n", lnEnd->time);
   printf("ArcsPerSec     %.2f\n", lat->na/lnEnd->time);
   printf("nodes          %d\n", lat->nn);
   printf("arcs:          %d\n", lat->na);
   printf("max_inDegree   %d\n", max_inDegree);
   printf("max_outDegree  %d\n", max_outDegree);
   printf("nWords         %d\n", nWords);
   if (isDAG)
      printf("nPaths         %e.0\n", nPaths);
   else
      printf("nPaths         inf\n");

   Dispose (&gcheap, topOrder);
}


/* LatSetBoundaryWords

     set start and end words for use in lattices and LM
*/
void LatSetBoundaryWords (char *start, char *end, char  *startLM, char *endLM)
{
   startWord = GetLabId (start ? start : "!SENT_START", TRUE);
   startLMWord = GetLabId (startLM ? startLM : "<s>", TRUE);
   endWord = GetLabId (end ? end : "!SENT_END", TRUE);
   endLMWord = GetLabId (endLM ? endLM : "</s>", TRUE);
   nullWord = GetLabId ("!NULL", TRUE);
}

/* FixBadLat

     if final word is !NULL replace it with endWord. This happens in lattices
     produced with some older decoders.
*/
void FixBadLat (Lattice *lat)
{
   LNode *ln;
   LArc *la;
   Boolean seenSE, seenOther;
   Word end;

   LatCheck (lat);

   end = GetWord (lat->voc, endWord, FALSE);
   if (!end)
      HError (8623, "HLRescore: SentEnd word (%d) not in vocabulary", endWord->name);

   ln = LatEndNode (lat);

   if (ln->word == end)
      return;

   if (ln->word && ln->word != lat->voc->nullWord)
      HError (8624, "HLRescore: final word in lattice (%s) is not !NULL! or sentend", 
              ln->word->wordName->name);

   /* now we know that we have !NULL at the end */

   seenSE = seenOther = FALSE;

   for (la = ln->pred; la; la = la->parc) {
      if (la->start->word == end)
         seenSE = TRUE;
      else
         seenOther = TRUE;
   }

   if (seenSE && seenOther)
      HError (8624, "HLRescore: lattice contains both SentEnd and other words in final position");

   if (!seenOther)
      return;

   ln->word = end;
   ln->v = 1;
}


#ifndef NO_LAT_LM
/* LatLMTrans

     wrapper around HLM:LMTrans() to take start and end words into account
*/
static LogFloat  LatLMTrans (LModel *lm, LMState src, LabId wordId, LMState *dest)
{
   LogFloat prob;
   
   if (wordId == nullWord) {
      dest = NULL;
      return 0.0;
   }
   else if (wordId == startWord && src == NULL) {
      /* always return P(<s>) = 1.0 at start of sentence */
      wordId = startLMWord;
      prob = LMTrans (lm, src, wordId, dest);
      return 0.0;
   }
   else if (wordId == endWord) {
      /* destination state of </s> transition is always NULL */
      wordId = endLMWord;
      prob = LMTrans (lm, src, wordId, dest);
      *dest = NULL;
      return prob;
   }
   else {
      prob = LMTrans (lm, src, wordId, dest);
   }

   return prob;
}

/* FindAddSubLNode

     Search for SubLNode in chain and add it if necessary
*/
static SubLNode *FindAddSubLNode (MemHeap *heap, LNode *ln, LMState lmstate, int *nsln)
{
   SubLNode *subln;
   
   for (subln = (SubLNode *) ln->hook; subln; subln = subln->next) {
      if (subln->data.lmstate == lmstate)
         return subln;
   }
   if (!subln) {
      ++*nsln;
      subln = New (heap, sizeof (SubLNode));
      subln->data.lmstate = lmstate;
      subln->foll = NULL;
      subln->next = (SubLNode *) ln->hook;
      ln->hook = (Ptr) subln;
   }
   return subln;
}


/* EXPORT->LatExpand

     expand lattice using new (typically higher-order) language Model
*/
Lattice *LatExpand (MemHeap *heap, Lattice *lat, LModel *lm)
{
   int i, nsln, nsla;
   LNode *ln, *newln;
   LNode **topOrder;
   LArc *la, *newla;
   SubLNode *startSLN, *endSLN, *sln;
   SubLArc *sla;
   LogFloat lmprob;
   LMState dest;
   Lattice *newlat;

   nsln = nsla = 0;

   /* The idea of this algorithm is that we will split each node and arc
      in the lattice into multiple sub-nodes and sub-arcs as required
      by the new LM. 
      N.B.We never join existing nodes, i.e. Calling LatExpand() with a
      bigram on a fourgram lattice will not reduce the size of the lattice.
   */


   /* for each node in the lattice we keep a linked list (hung of
      ln->hook) of sub-nodes (corresponding to LMStates in the new
      LM). */

   /* init sub-node linked lists */
   for (i = 0, ln = lat->lnodes; i < lat->nn; ++i, ++ln) {
      ln->hook = NULL;
   }

   /* create one sub-node for lattice start node with LMState = NULL */
   FindAddSubLNode (&slnHeap, LatStartNode (lat), NULL, &nsln);
   
   /* find topological order of nodes */
   topOrder = (LNode **) New (&gcheap, lat->nn * sizeof(LNode *));
   LatTopSort (lat, topOrder);

   
   /* create lists of sub-nodes and sub-arcs and count them as we go along */
   for (i = 0; i < lat->nn; ++i) {
      ln = topOrder[i];
      for (startSLN = (SubLNode *) ln->hook; startSLN; startSLN = startSLN->next) {
         /* for each outgoing arc from current subLNode */
         for (la = ln->foll; la; la = la->farc) {
            assert (la->start == ln);
            lmprob = LatLMTrans (lm, startSLN->data.lmstate, la->end->word->wordName, &dest);
            endSLN = FindAddSubLNode (&slnHeap, la->end, dest, &nsln);

            /* add new subLArc */
            ++nsla;
            sla = New (&slaHeap, sizeof (SubLArc));
            sla->lmprob = lmprob;
            sla->end = endSLN;
            sla->la = la;
            /* add to list of arcs leaving startSLN */
            sla->next = startSLN->foll;
            startSLN->foll = sla;
         }
      }
   }

   if (trace & T_EXP)
      printf ("expanded lattice from %d/%d  to %d/%d\n", lat->nn, lat->na, nsln, nsla);
   
   /* build new lattice from sub-node/-arc lists */
   newlat = NewILattice (heap, nsln, nsla, lat);
   newlat->net = CopyString (heap, lm->name);

   /* create one node in new lattice for each sub node in old lattice */
   newln = newlat->lnodes;
   for (i = 0; i < lat->nn; ++i) {
      ln = topOrder[i];
      for (sln = (SubLNode *) ln->hook; sln; sln = sln->next) {
         *newln = *ln;
         newln->foll = newln->pred = NULL;
         newln->n = 0;
         newln->hook = NULL;

         sln->data.newln = newln;
         ++newln;
      }
   }
   assert (newln = newlat->lnodes + newlat->nn);

   /* create arcs in new lattice */
   newla = newlat->larcs;
   for (i = 0; i < lat->nn; ++i) {
      ln = topOrder[i];
      for (sln = (SubLNode *) ln->hook; sln; sln = sln->next) {
         newln = sln->data.newln;
         for (sla = sln->foll; sla; sla = sla->next) {
            *newla = *sla->la;
            newla->start = newln;
            newla->end = sla->end->data.newln;
            newla->lmlike = sla->lmprob;
            
            /* add to start node foll list */
            newla->farc = newla->start->foll;
            newla->start->foll = newla;
            /* add to end node pred list */
            newla->parc = newla->end->pred;
            newla->end->pred = newla;
            
            ++newla;
         }
      }
   }
   assert (newla == newlat->larcs + newlat->na);

   if (trace & T_MEM) {
      printf("Memory State after expanding\n");
      PrintAllHeapStats();
   }

   Dispose (&gcheap, topOrder);
   ResetHeap (&slaHeap);
   ResetHeap (&slnHeap);

   return newlat;
}

#endif


/* 
   MergeArcs : merge two active acrs by deactivating 
   the one with a lower total likelihood
*/
void MergeArcs(Lattice *lat, LArc *la1, LArc *la2)
{
   /* doing nothing if one is deactivated */
   if (la1->score < 0 || la2->score < 0) 
      return;

   /* doing nothing for identical arcs */
   if (la1 == la2) {
      if (trace & T_MRG) {
         fprintf(stdout, "merging identical arcs (%d %s)->(%d %s):%f and (%d %s)->(%d %s):%f ...\n", 
                 la1->start->n, la1->start->word->wordName->name, la1->end->n, 
                 la1->end->word->wordName->name, la1->lmlike,
                 la2->start->n, la2->start->word->wordName->name, la2->end->n, 
                 la2->end->word->wordName->name, la2->lmlike);
         fflush(stdout);         
      }
      return;
   }

   if (trace & T_MRG) {
      fprintf(stdout, "merging arcs (%d %s)->(%d %s):%f and (%d %s)->(%d %s):%f ...\n", 
              la1->start->n, la1->start->word->wordName->name, la1->end->n, 
              la1->end->word->wordName->name, la1->lmlike,
              la2->start->n, la2->start->word->wordName->name, la2->end->n, 
              la2->end->word->wordName->name, la2->lmlike);
      fflush(stdout);   
   }
     
   /* deactivate the arc with a lower total likelihood */
   (LArcTotLike(lat, la1) >= LArcTotLike(lat, la2)) ? 
      (la2->score = -1.0) : (la1->score = -1.0);
   
}

/* 
   MergeArcsForNode: merge all inbound and outbound acrs of 
   a node that have the same LM information
*/
void MergeArcsForNode(Lattice *lat, LNode *ln)
{
   LArc *la1, *la2;

   /* merge following arcs */
   for (la1 = ln->foll; la1; la1 = la1->farc) {
      for (la2 = la1->farc; la2; la2 = la2->farc) {
         if (strcmp(la1->end->word->wordName->name, la2->end->word->wordName->name) == 0
             && la1->lmlike == la2->lmlike) {
            assert(la1->lmlike == la2->lmlike);
            MergeArcs(lat, la1, la2);
         }
      }
   }

   /* merge preceding ars */
   for (la1 = ln->pred; la1; la1 = la1->parc) {
      for (la2 = la1->parc; la2; la2 = la2->parc) {
         if (strcmp(la1->start->word->wordName->name, la2->start->word->wordName->name) == 0
             && la1->lmlike == la2->lmlike) {
            assert(la1->lmlike == la2->lmlike);
            MergeArcs(lat, la1, la2);
         }
      }
   }  
}

/* 
   RecoverArcsForNode: recover inbound and/or outbound acrs of a
   node if no inbound and/or outbound arc is active for this node 
*/
void RecoverArcsForNode(Lattice *lat, LNode *ln)
{
   Boolean in, out; 
   LArc *la1, *la2;

   /* checking inbound arcs */
   in = FALSE;
   for (la1 = ln->pred; la1; la1 = la1->parc) {      
      if (la1->score >= 0) {
         in = TRUE;
         break;
      }
   }

   /* checking outbound arcs */
   out = FALSE;
   for (la1 = ln->foll; la1; la1 = la1->farc) {
      if (la1->score >= 0) {
         out = TRUE;
         break;
      }
   }

   /* doing nothing if the node is fully connected */
   if (in && out) return;
   
   /* recovering inbound arcs */
   if (!in) {
      for (la1 = ln->pred; la1; la1 = la1->parc) { 
         la1->score = 0;
      }
      for (la1 = ln->pred; la1; la1 = la1->parc) { 
         for (la2 = la1->parc; la2; la2 = la2->parc) {
            if (strcmp(la1->start->word->wordName->name, la2->start->word->wordName->name) == 0
               && la1->lmlike == la2->lmlike) {
               la2->score = -1.0;
            }
         }
      }
   }

   /* recovering outbound arcs */
   if (!out) {
      for (la1 = ln->foll; la1; la1 = la1->farc) {
         la1->score = 0;
      }
      for (la1 = ln->foll; la1; la1 = la1->farc) {
         for (la2 = la1->farc; la2; la2 = la2->farc) {
            if (strcmp(la1->end->word->wordName->name, la2->end->word->wordName->name) == 0
                && la1->lmlike == la2->lmlike) {
               la2->score = -1.0;
            }
         }
      }          
   }
}

/*
  MergeNodes: merge two nodes in a lattice
*/
static LNode *MergeNodes(Lattice *lat, LNode *ln1, LNode *ln2)
{
   LArc *la;  
   
   /* doing nothing */
   if (ln1 == ln2) {
      if (trace & T_MRG) {
         fprintf(stdout, "merging identical nodes (%d %s) and (%d %s) ...\n", 
                 ln1->n, ln1->word->wordName->name, ln2->n, ln2->word->wordName->name);
         fflush(stdout);
      }
      return ln1;
   }

   if (trace & T_MRG) {
      fprintf(stdout, "merging nodes (%d %s) and (%d %s) ...\n", 
              ln1->n, ln1->word->wordName->name, ln2->n, ln2->word->wordName->name);
      fflush(stdout);
   }
   
   /* setting start word of follwoing arcs of node ln2 */
   for (la = ln2->foll; la; la = la->farc) {
      la->start = ln1;      
   }
   /* setting end word of preceding arcs of node ln2 */
   for (la = ln2->pred; la; la = la->parc) {
      la->end = ln1;      
   }
   
   /* moving following arcs of node ln2 to ln1 */
   if (ln1->foll) {      
      for (la = ln1->foll; la->farc; la = la->farc);
      la->farc = ln2->foll;
   }
   else {
      ln1->foll = ln2->foll;
   }
   ln2->foll = NULL;

   /* moving preceding arcs of node ln2 to ln1 */
   if (ln1->pred) {      
      for (la = ln1->pred; la->parc; la = la->parc);
      la->parc = ln2->pred;
   }
   else {
      ln1->pred = ln2->pred;
   }
   ln2->pred = NULL;

   /* activate and deactivate */
   ln1->score = 0;   
   ln2->score = -1.0;
   
   return ln1;
}

/*
  ToMergeLatNodesForw: checking if forword merge is needed between 
  a node and other active nodes that are connected to this node
*/
Boolean ToMergeLatNodesForw(Lattice *lat, LNode *ln)
{
   Boolean isFound = FALSE;
   LArc *la1, *la2;
      
   for (la1 = ln->foll; la1; la1 = la1->farc) {
      for (la2 = la1->farc; la2; la2 = la2->farc) {
         if (strcmp(la1->end->word->wordName->name, la2->end->word->wordName->name) == 0
             && la1->score >= 0 && la1->end->score >= 0 && la2->score >= 0 && la2->end->score >= 0) {
            isFound = TRUE;
            return isFound;
         }
      }
   }
   
   return isFound;
}

/* 
   MergeLatNodesForw: recursively merge a node and other 
   forward active nodes that are connected to this node 
*/
void MergeLatNodesForw(Lattice *lat, LNode *ln)
{
   int i, n;
   LNode **ntab;
   LArc *la1, *la2;
   
   for (la1 = ln->foll; la1; la1 = la1->farc) {
      for (la2 = la1->farc; la2; la2 = la2->farc) {
         if (strcmp(la1->end->word->wordName->name, la2->end->word->wordName->name) == 0) {
            /* if lattice contains cycles */
            if ((la1->end == ln || la2->end == ln) && la1->end != la2->end) {
               ln = MergeNodes(lat, la1->end, la2->end);
            }
            /* merge indentical following words */
            else {
               MergeNodes(lat, la1->end, la2->end);
            }
         }         
      }
   }
   
   /* checking all connected and active nodes to merge */ 
   n = 0;
   for (la1 = ln->foll; la1; la1 = la1->farc) {
      if (la1->score >= 0 && la1->end->score >= 0 && ToMergeLatNodesForw(lat, la1->end)) {
         n++;
      }
   }
   if (n == 0) return;

   /* recording all connected and active nodes to merge */ 
   ntab = (LNode **)New(lat->heap, n * sizeof(LNode *));
   
   n = 0;
   for (la1 = ln->foll; la1; la1 = la1->farc) {
      if (la1->score >= 0 && la1->end->score >= 0 && ToMergeLatNodesForw(lat, la1->end)) {
         ntab[n++] = la1->end;
      }
   }
   
   /* merge arcs */
   for(i = 0; i < n; i++) {
      MergeArcsForNode(lat, ntab[i]);
   }
   
   /* recursively merge forward nodes */
   for(i = 0; i < n; i++) {
      MergeLatNodesForw(lat, ntab[i]);
   }

   Dispose(lat->heap, ntab);
}    

/*
  ToMergeLatNodesBackw: checking if backward merge is needed between 
  a node and other active nodes that are connected to this node
*/
Boolean ToMergeLatNodesBackw(Lattice *lat, LNode *ln)
{
   Boolean isFound = FALSE;
   LArc *la1, *la2;
      
   for (la1 = ln->pred; la1; la1 = la1->parc) {
      for (la2 = la1->parc; la2; la2 = la2->parc) {
         if (strcmp(la1->start->word->wordName->name, la2->start->word->wordName->name) == 0
             && la1->score >= 0 && la1->start->score >= 0 && la2->score >= 0 && la2->start->score >= 0) {
            isFound = TRUE;
            return isFound;
         }
      }
   }
   
   return isFound;
}

/* 
   MergeLatNodesBackw: recursively merge a node and other 
   backward active nodes that are connected to this node 
*/
void MergeLatNodesBackw(Lattice *lat, LNode *ln)
{
   int i, n;
   LNode **ntab;
   LArc *la1, *la2;
   
   for (la1 = ln->pred; la1; la1 = la1->parc) {
      for (la2 = la1->parc; la2; la2 = la2->parc) {
         if (strcmp(la1->start->word->wordName->name, la2->start->word->wordName->name) == 0) {
            /* if lattice contains cycles */
            if ((la1->start == ln || la2->start == ln) && la1->start != la2->start) {
               ln = MergeNodes(lat, la1->start, la2->start);
            }
            /* merge indentical following words */
            else {
               MergeNodes(lat, la1->start, la2->start);
            }
         }         
      }
   }
   
   /* checking all connected and active nodes to merge */ 
   n = 0;
   for (la1 = ln->pred; la1; la1 = la1->parc) {
      if (la1->score >= 0 && la1->start->score >= 0 && ToMergeLatNodesBackw(lat, la1->start)) {
         n++;
      }
   }
   if (n == 0) return;

   /* recording all connected and active nodes to merge */ 
   ntab = (LNode **)New(lat->heap, n * sizeof(LNode *));
   
   n = 0;
   for (la1 = ln->pred; la1; la1 = la1->parc) {
      if (la1->score >= 0 && la1->start->score >= 0 && ToMergeLatNodesBackw(lat, la1->start)) {
         ntab[n++] = la1->start;
      }
   }
   
   /* merge arcs */
   for(i = 0; i < n; i++) {
      MergeArcsForNode(lat, ntab[i]);
   }
   
   /* recursively merge forward nodes */
   for(i = 0; i < n; i++) {
      MergeLatNodesBackw(lat, ntab[i]);
   }

   Dispose(lat->heap, ntab);   
}    

/* 
   ShowLattice: show a lattice for debugging purpose
*/
void ShowLattice(Lattice *lat)
{
   int i;
   LNode *ln;
   LArc *la;
   
   fprintf(stdout, "Utterance: %s\n", lat->utterance);
   fprintf(stdout, "N=%-4d L=%-5d\n", lat->nn, lat->na);

   for (i = 0, ln = lat->lnodes; i < lat->nn; i++, ln++) {
      fprintf(stdout, "I=%-4d  t=%-5.2f %-19s\n", ln->n, ln->time, ln->word->wordName->name);
   }

   for (i = 0, la = lat->larcs; i < lat->na; i++, la++) {
      fprintf(stdout, "J=%-4d S=%-4d %-19s E=%-4d %-19s a=%-9.2f l=%-7.3f\n", i,
              la->start->n, la->start->word->wordName->name, la->end->n,
              la->end->word->wordName->name, la->aclike, la->lmlike);
   }

   fflush(stdout);
}

/* EXPORT->MergeLatNodesArcs

     Merge duplicate lattice nodes and arcs caused by different phonetic 
     contexts or pronunciation variants in a lattice for acoutic rescoring.
     The converted word graph retains the original LM information.
*/
Lattice *MergeLatNodesArcs(Lattice *lat, MemHeap *heap, Boolean mergeFwd)
{
   int i, nn, na;
   LNode *ln, *newln;
   LArc *la, *newla;
   Lattice *newlat;

   /* resetting all the score values for nodes and arcs */
   for (i = 0, ln = lat->lnodes; i < lat->nn; i++, ln++) {
      ln->score = 0;
   }
   for (i = 0, la = lat->larcs; i < lat->na; i++, la++) {
      la->score = 0;
   }

   /* merging arcs */
   for (i = 0, ln = lat->lnodes; i < lat->nn; i++, ln++) {
      MergeArcsForNode(lat, ln);
   }

   /* merging nodes */
   if (mergeFwd) {
      for (i = 0, ln = lat->lnodes; i < lat->nn; i++, ln++) {
         MergeLatNodesForw(lat, ln);      
      }
      /* have to merge the remaining active nodes backward 
      for (i = 0, ln = lat->lnodes; i < lat->nn; i++, ln++) {
         if (ln->score >= 0) {
            MergeLatNodesBackw(lat, ln);
         }
      }
      */
   }
   else {
      for (i = 0, ln = lat->lnodes; i < lat->nn; i++, ln++) {
         MergeLatNodesBackw(lat, ln);
      }
      /* have to merge the remaining active nodes forward 
      for (i = 0, ln = lat->lnodes; i < lat->nn; i++, ln++) {
         if (ln->score >= 0) {
            MergeLatNodesForw(lat, ln);
         }
      }
      */
   }
      
   /* getting the number of remaining nodes and acrs */
   nn = 0; na = 0;
   for (i = 0, ln = lat->lnodes; i < lat->nn; i++, ln++) {
      if (ln->score >= 0) {
         nn++;
         /* this might not be needed but just in case */
         RecoverArcsForNode(lat, ln);
      }
   }
   for (i = 0, la = lat->larcs; i < lat->na; i++, la++) {
      if (la->score >= 0) {
         na++;
      }
   }
   
   /* build new lattice from remaining nodes/arcs */
   newlat = NewILattice (heap, nn, na, lat);

   /* copy all the nodes we want to keep */
   nn = 0;
   newln = newlat->lnodes;
   for (i = 0, ln = lat->lnodes; i < lat->nn; i++, ln++) {
      if (ln->score >= 0) {
         /* keep */
         *newln = *ln;
         /* update the ordering index */
         newln->n = ln->n = nn;
         ++newln; 
         nn++;
      }
   }
   assert (newln == newlat->lnodes + nn);

   /* copy all the arcs we want to keep */
   newla = newlat->larcs; 
   for (i = 0, la = lat->larcs; i < lat->na; i++, la++) {
      if (la->score >= 0) {
         /* keep */
         *newla = *la;

         newla->start = &newlat->lnodes[la->start->n];
         newla->end = &newlat->lnodes[la->end->n];
         
         /* insert newla into foll list */
         newla->farc = newla->start->foll;
         newla->start->foll = newla;

         /* ...and into pred list */
         newla->parc = newla->end->pred;
         newla->end->pred = newla;

         if (compressMerge) {
            newla->aclike = 0;
            newla->prlike = 0;
         }
         ++newla;
      }
   }
   assert (newla == newlat->larcs + na);

   if (trace & T_MRG) 
      ShowLattice(newlat);
   
   if (trace & T_TOP) {
      fprintf(stdout, "\nAverage density nodes/sec : %-4.4f -> %-4.4f\n", 
              lat->nn/lat->lnodes[lat->nn - 1].time, newlat->nn/newlat->lnodes[newlat->nn - 1].time); 
      fprintf(stdout, "\nAverage density links/sec : %-4.4f -> %-4.4f\n\n", 
              lat->na/lat->lnodes[lat->nn - 1].time, newlat->na/newlat->lnodes[newlat->nn - 1].time); 
      fflush(stdout);
   }

   return newlat;   
}

/* 
   ApplyWPNetLM2LabLat: apply word pair LM network to a 
   lattice created from a single sequence of word labels 
*/
void ApplyWPNet2LabLat(Lattice *lat, Lattice *wdNet)
{
   int i, j; 
   Boolean isFound;
   LArc *la, *lmla;

   /* label lattice must contain NULL node at the start and end */
   if (lat->lnodes[0].word != lat->voc->nullWord 
       || lat->lnodes[lat->nn - 1].word != lat->voc->nullWord)
      HError(9999, "HLat: Sentence start or end NULL node missing in label lattice!");

   /* for all arcs except [NULL -> SENT_START] and [SENT_END -> NULL] */
   for (i = 1, la = lat->larcs + 1; i < lat->na - 1; i++, la++) {            
      isFound = FALSE;             
      for (j = 0, lmla = wdNet->larcs; j < wdNet->na; j++, lmla++) {
         if (strcmp(la->start->word->wordName->name, lmla->start->word->wordName->name) == 0
             && strcmp(la->end->word->wordName->name, lmla->end->word->wordName->name) == 0) {
            /* this checks if the LM network is deterministic */
            if (isFound && (la->lmlike != (lmla->lmlike * wdNet->lmscale) / lat->lmscale)) {
               HError(8696, "HLat: Word pair LM network is undeterministic: [%s -> %s] : %f and [%s -> %s] : %f !\n", 
                      la->start->word->wordName->name, la->end->word->wordName->name, la->lmlike,
                      la->start->word->wordName->name, la->end->word->wordName->name, 
                      (lmla->lmlike * wdNet->lmscale) / lat->lmscale);
            }
            la->lmlike = (lmla->lmlike * wdNet->lmscale) / lat->lmscale;
            isFound = TRUE;
         }                  
         if (j == wdNet->na - 1 && !isFound) 
            HError(8696, "HLat: LM score for arc [%s -> %s] not found in word pair LM!", 
                   la->start->word->wordName->name, la->end->word->wordName->name);
      }         
   }   
   
   /* for the first and last arcs [NULL -> SENT_START] 
      and [SENT_END -> NULL] */
   lat->larcs[0].lmlike = lat->larcs[lat->na - 1].lmlike = 0;
}


#ifndef NO_LAT_LM
/* 
   ApplyNGram2LabLat: apply N-gram LM to a 
   lattice created from a single sequence of word labels 
*/
void ApplyNGram2LabLat(Lattice *lat, LModel *lm)
{
   int i, j;
   LabId *revlab;
   LNode *ln;
   LArc *la;

   /* label lattice must contain NULL node at the start and end */
   if (lat->lnodes[0].word != lat->voc->nullWord 
       || lat->lnodes[lat->nn - 1].word != lat->voc->nullWord)
      HError(9999, "HLat: Sentence start or end NULL node missing in label lattice!");
      
   /* create label array in revere order for N-gram histories 
      it size is the total number of non NULL words in the label 
      lattice plus maxium N-gram history length */
   revlab = (LabId *)New(lat->heap, (lat->nn - 2 + NSIZE - 1) * sizeof(LabId));   
   for (j = 0; j < lat->nn - 2 + NSIZE - 1; j++) {      
      revlab[j] = NULL;
   }

   /* all the words in the label lattice are store in a reverse order in 
      as [W_n, W_n-1, ..., W_2, W_1] in the array excluding NULL word. 
      LM vocab word Id is also assigned to each word to access LM */ 
   for (i = 0, ln = &lat->lnodes[1]; i < lat->nn - 2; i++, ln++) {
      revlab[lat->nn - 3 - i] = ln->word->wordName;
      for (j = 1; j <= lm->data.ngram->vocSize; j++) {
         if (strcmp(lm->data.ngram->wdlist[j]->name, revlab[lat->nn - 3 - i]->name) == 0) {
            revlab[lat->nn - 3 - i]->aux = (Ptr) lm->data.ngram->wdlist[j]->aux;
         }
      }
   }
   
   if (trace & T_EXP) {
      fprintf(stdout, "\n Word labels: ");
      for (j = 0; j < lat->nn - 2; j++) {
         fprintf(stdout, "%s ", revlab[j]->name);
      }
      fprintf(stdout, "\n");
      fprintf(stdout, "\n Vocab entries: ");
      for (j = 0; j < lat->nn - 2; j++) {
         fprintf(stdout, "%s ", lm->data.ngram->wdlist[(int) revlab[j]->aux]->name);
      }
      fprintf(stdout, "\n\n");
      fflush(stdout);
   }

   /* for all arcs except [NULL -> SENT_START] and [SENT_END -> NULL] */   
   for (i = 1, la = lat->larcs + 1; i < lat->na - 1; i++, la++) {            

      la->lmlike = GetLMProb(lm, &revlab[lat->nn - 2 - i], la->end->word->wordName);

      if (trace & T_EXP) {
         fprintf(stdout, "\n LM likelihood for arc [%s -> %s] from N-gram entry:",
                 la->start->word->wordName->name, la->end->word->wordName->name);
         fprintf(stdout, " ( ");
         for (j = lat->nn - 3; j > lat->nn - 3 - i; j--) {
            fprintf(stdout, "%s ", revlab[j]->name);
         }
         fprintf(stdout, ") -> %s : %f\n", la->end->word->wordName->name, la->lmlike);
         fflush(stdout);
      }
   }   

   /* for the first and last arcs [NULL -> SENT_START] 
      and [SENT_END -> NULL] */
   lat->larcs[0].lmlike = lat->larcs[lat->na - 1].lmlike = 0;

   Dispose(lat->heap, revlab);
}
#endif
