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
/*         File: HNet.h  Network and Lattice Functions         */
/* ----------------------------------------------------------- */

/* !HVER!HNET:   3.4.1 [CUED 12/03/09] */

/*
   Nets come in two forms.
   a)  Lattices are used to represent word level networks.
   b)  Networks used by HRec for viterbi decoding.
     The network reads in a word level network in the form of a lattice.
     To reduce storage requirements for this lattice (which could be very
     large) the lattice uses a compact storage representation which does
     not contain many of the optional lattice fields.
     This lattice is then expanded into the network required for HRec.
     Finally HRec produces a lattice as output in which the lattice arcs
     use the full format and contain additional information rather than 
     just the language model likelihood.
*/

#ifndef _HNET_H_
#define _HNET_H_

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------ Initialisation --------------------------- */

void InitNet(void);
/*
   register module & set configuration parameters
*/

/* -------------------- Lattice Definintions ------------------------- */

/*
   Lattice Types and Routines.

   These are used for all word-level networks 
*/

#define L_VERSION "1.0"     /* Version written to header of all lattices */

typedef int LatFormat;      /* Format of lattice. Formed by oring flags */

#define HLAT_ALABS  0x0001  /* Word labels with arcs (normally with nodes) */
#define HLAT_LBIN   0x0002  /* Binary lattices for speed */
#define HLAT_TIMES  0x0008  /* Node times */
#define HLAT_PRON   0x0010  /* Pronunciation information */
#define HLAT_ACLIKE 0x0020  /* Acoustic likelihoods */
#define HLAT_LMLIKE 0x0040  /* Language model likelihoods (and scale etc) */
#define HLAT_ALIGN  0x0080  /* Output within word alignment (if present) */
#define HLAT_ALDUR  0x0100  /* Output within word alignment durations */
#define HLAT_ALLIKE 0x0200  /* Output within word alignment likelihoods */
#define HLAT_PRLIKE 0x0400  /* Pronunciation likelihoods (and scale etc) */
#define HLAT_TAGS   0x0800  /* Output semantic tags */

#define HLAT_NOSORT 0x1000  /* Do not sort lattice before output */
#define HLAT_NOSUBS 0x2000  /* Do not output sublats */
/* #define HLAT_EXTEN  0x2000   Using extensible versions of everything */
#define HLAT_SHARC  0x4000  /* Using short version of arc data structures */

#define HLAT_DEFAULT 0x03f8 /* Default output format */


typedef struct lnode *NodeId;
typedef struct larc *ArcId;

#define NARC NULL         /* NULL arcid for end of linked lists */
#define NNODE NULL        /* NULL nodeid for end of linked lists */

typedef struct lalign 
{
   int state;         /* State number (-1==model_end) */
   LabId label;       /* Segment label ('phys_hmm[state]' or 'phys_hmm') */
   float dur;         /* Duration of segment in seconds */
   LogFloat like;     /* Total aclike of label (inc trans within + out) */
}
LAlign;

/* Storage of SubLats */
typedef struct sublatdef {
   struct lattice *lat;     /* Lattice this refers to (may be shared) */
   int usage;               /* Number of references to this SubLat */
   struct sublatdef *next;  /* Next sublat at this level */
   /* struct sublatdef *prev;   Previous sublat at this level */
   struct sublatdef *chain; /* Next sublat referring to lat */
} SubLatDef;

/* 
   Note the following is not standard C.

   In order to accomodate compact arcs we need to know how big they are
   so we define larc_s as a type solely to be used in sizeof calculations
   to allow us to find out how big the first five fields of larc are.
*/

typedef struct larc_s
{
   NodeId start;
   NodeId end;
   LogFloat lmlike;
   ArcId farc;
   ArcId parc;
}
LArc_S;

typedef struct larc_e *EArcId;

typedef struct larc_e
{
   NodeId start;       /* Node at start of word */
   NodeId end;         /* Node at end of word */
   LogFloat lmlike;    /* Language model likelihood of word */

   EArcId farc;        /* Next arc following start node */
   EArcId parc;        /* Next arc preceding end node */

   EArcId fcra;        /* List linked in both directions */
   EArcId pcra;        /*  to easy deletetion */
   int n;              /* Arc identity */
   /* Ptr hook;         Hook - For 64 bit machines this is too big */
}
LArc_E;

typedef struct larc
{
   NodeId start;       /* Node at start of word */
   NodeId end;         /* Node at end of word */
   LogFloat lmlike;    /* Language model likelihood of word */

   ArcId farc;         /* Next arc following start node */
   ArcId parc;         /* Next arc preceding end node */

   LogFloat aclike;    /* Acoustic likelihood of word */

   short nAlign;       /* Number of alignment records in word */
   LAlign *lAlign;     /* Array[0..nAlign-1] of alignment records */

   float score;        /* Field used for pruning/sorting */
   LogFloat prlike;    /* Pronunciation likelihood of arc */
}
LArc;

/* Note:  Total arc likelihood == aclike + lmlike*lmscale + wdpenalty */

typedef struct lnode
{
   int n;              /* Sorted order */

   Word word;          /* Word represented by arc (labels may be on nodes) */
   char *tag;          /* Semantic tag for this node */
   short v;            /* Pronunciation variant number */
   SubLatDef *sublat;  /* SubLat for node (if word==lat->voc->subLatWord) */

   HTime time;         /* Time of word boundary at node */

   ArcId foll;         /* Linked list of arcs following node */
   ArcId pred;         /* Linked list of arcs preceding node */

   double score;       /* Field used for pruning */

   Ptr hook;           /* User definable hook */
}
LNode;

typedef struct lattice
{
   MemHeap *heap;               /* Heap lattice uses */
   LatFormat format;	       	/* indicate which fields are valid */
   Vocab *voc;                  /* Dictionary lattice based on */

   int nn;                      /* Number of nodes */
   int na;                      /* Number of arcs */
   LNode *lnodes;               /* Array of lattice nodes */
   LArc *larcs;                 /* Array of lattice arcs */

   LabId subLatId;              /* Lattice Identifier (for SubLats only) */
   SubLatDef *subList;          /* List of sublats in this lattice level */
   SubLatDef *refList;          /* List of all SubLats referring to this lat */
   struct lattice *chain;       /* Linked list used for various jobs */

   char *utterance;		/* Utterance file name (NULL==unknown) */
   char *vocab;			/* Dictionary file name (NULL==unknown) */
   char *hmms;			/* MMF file name (NULL==unknown) */
   char *net;			/* Network file name (NULL==unknown) */

   float acscale;               /* Acoustic scale factor */
   float lmscale;		/* LM scale factor */
   LogFloat wdpenalty;		/* Word insertion penalty */
   float prscale;		/* Pronunciation scale factor */
   HTime framedur;              /* Frame duration in 100ns units */
   float logbase;               /* base of logarithm for likelihoods in lattice files
                                   (1.0 = default (e), 0.0 = no logs) */
   float tscale;                /* time scale factor (default: 1, i.e. seconds) */

   Ptr hook;                    /* User definable hook */
}
Lattice;

/*
   To use both long and short formats for lattice arcs should 
   use the following macros to access the arcs.
*/
/*
#define NumbLArc(lat,n) ((lat)->format&HLAT_EXTEN?(lat)->larcs+(n):\
			 (lat)->format&HLAT_SHARC?\
			 (LArc*)(((LArc_S*)(lat)->larcs)+(n)):(lat)->larcs+(n))
*/

#define NumbLArc(lat,n) ((lat)->format&HLAT_SHARC?\
			 (LArc*)(((LArc_S*)(lat)->larcs)+(n)):(lat)->larcs+(n))

#define LArcNumb(la,lat) ((lat)->format&HLAT_SHARC?\
			 (((LArc_S*)(la))-((LArc_S*)(lat)->larcs)):\
			 (la)-(lat)->larcs)

#define NextLArc(lat,la) (LArc*)((char*)la+((lat->format&HLAT_SHARC)?\
					     sizeof(LArc_S):sizeof(LArc)))

#define LArcTotLMLike(lat,la) ((la)->lmlike*(lat)->lmscale + \
			       (((la)->end->word==NULL || \
				 (la)->end->word==(lat)->voc->nullWord) ? \
				0.0 : (lat)->wdpenalty ))

#define LArcTotLike(lat,la) ((la)->aclike*(lat)->acscale + \
                             (la)->lmlike*(lat)->lmscale + \
			     (la)->prlike*(lat)->prscale + \
			     (((la)->end->word==NULL || \
			       (la)->end->word==(lat)->voc->nullWord) ? \
			      0.0 : (lat)->wdpenalty ))


Lattice *NewLattice(MemHeap *heap, int nn, int na);
/*
   Create a new lattice structure with nn nodes and na arcs.  Arcs
   are full-size.
*/

Lattice *NewILattice(MemHeap *heap,int nn,int na,Lattice *info);
/* 
   Create a new lattice structure with fields initialised by info.
   If nn/na == -1 the nodes/arcs will also be copied otherwise these
   will be set allocated (assuming nn/na!=0) and initialised with 0.
*/

void FreeLattice(Lattice *lat);
/* 
   Free up storage used by lattice
*/

ReturnStatus WriteLattice(Lattice *lat, FILE *file, LatFormat form);
/*
   Write lattice to given file, according to given format  
   specifier.
*/

Lattice *ReadLattice(FILE *file, MemHeap *heap, Vocab *voc, 
		     Boolean shortArc, Boolean add2Dict);

Lattice *ReadOneLattice(Source *src, MemHeap *heap, Vocab *voc, 
                        Boolean shortArc, Boolean add2Dict);
/*
   Read lattice from file and creates a lattice in memory using heap.
   Word names in the lattice are mapped to the internal Word type
   using the Vocab voc.  If shortArc is true, then each arc is stored in
   short form and cannot then support alignment information.
   If add2Dict is TRUE then ReadLattice will add unseen words to voc
   rather than generating an error.
*/



SubLatDef *AdjSubList(Lattice *lat,LabId subLatId,Lattice *subLat,int adj);
/* 
   Adjust list of subLatDefs (and usage counts) for lattice lat to
   reflect usage adjustment adj to subLat/subLatId.
*/

Lattice *SubLatList(Lattice *lat, Lattice *tail, int depth);
/* 
   Create an ordered list of lattices that are sub lattices of lat,
   adding them to tail in dependency order.
*/


Lattice *ExpandMultiLevelLattice(MemHeap *heap, Lattice *lat,Vocab *voc);
/*
   Take the multi-level lattice lat and expand it into a 
   single-level lattice using heap to create the new expanded lattice.
   ExpandMultiLevelLattice returns the expanded lattice.
*/

Lattice *LatticeFromLabels(LabList *ll,LabId bnd,Vocab *voc,MemHeap *heap);
/*
   Convert label list into a lattice under to compute an alignment.  Each
   word in the label list is replaced by a Word using the vocab voc.  If
   bnd is not null, then it is inserted at the start and end of the label list.
   This is typically used to include a silence model.
*/

int NumNodeFoll(NodeId n);
/* 
   Return the number of followers (outgoing arcs) from node n
*/
 
int NumNodePred(NodeId n);
/* 
   Return the number of predecessors (incoming arcs) to node n 
*/
 
NodeId FindLatStart(Lattice *lat);
/* 
   Find and return the lattice start node, there can only be one
*/
 
NodeId FindLatEnd(Lattice *lat);
/* 
   Find and return the lattice end node, there can only be one
*/

/*  ----------------------- Networks ------------------------ */

/* Model level networks - for use with e.g. HRec  */

/* Types of node that can appear in the network */
enum {
   n_unused,            /* Node Instance not yet assigned */
   n_hmm=2,             /* Node Instance represents HMM */
   n_word=4,            /* Node Instance represents word end (or null) */
   n_tr0=4,             /* Entry token reaches exit in t=0 */
   n_wd0=1,             /* Exit token reaches word node in t=0 */
   n_wdstart=8,         /* Temporary wdstart node */
   n_nocontext=15,      /* binary and with this to remove context ids */
   n_lcontext=16,       /* Multiplication factor for context id */
   n_rcontext=16384     /* Multiplication factor for context id */
};
typedef int NetNodeType; 

typedef struct _NetLink NetLink;
typedef struct _NetInst NetInst;
typedef struct _NetNode NetNode;

/* The network nodes themselves just store connectivity info */
struct _NetNode {
   NetNodeType type;    /* Type of this node (includes context) */
   union {
      HLink  hmm;       /* HMM (physical) definition */
      Pron   pron;      /* Word represented (may == null) */
   }
   info;                /* Extra information specific to type of node */
   char    *tag;        /* Semantic tagging information */
   int nlinks;          /* Number of nodes connected to this one */
   NetLink *links;      /* Array[0..nlinks-1] of links to connected nodes */
   NetInst *inst;       /* Model Instance (if one exists, else NULL) */   
   NetNode *chain;
   int aux;
};

struct _NetLink{
   NetNode *node;       /* Node in network */
   LogFloat like;       /* Transition likelihood */
};

typedef struct {
   MemHeap *heap;     /* heap for allocating network */
   Vocab *vocab;      /* Dictionary from which words appear */
   Word nullWord;     /* Word for output when word==NULL */
   Boolean teeWords;  /* True if any tee words are present */
   NetNode initial;   /* Initial (dummy) node */
   NetNode final;     /* Final (dummy) node */
   int numNode;
   int numLink;
   MemHeap nodeHeap;  /* a heap for allocating nodes */
   MemHeap linkHeap;  /* a stack for adding the links as needed */
   NetNode *chain;
} Network;

typedef struct hmmsetcxtinfo {
   HMMSet *hset;   /* HMMSet */
   int nc;         /* Number of contexts */
   int xc;         /* Number of cross word contexts */
   Boolean sLeft;  /* Seen left context dependency */
   Boolean sRight; /* Seen right context dependency */
   LabId *cxs;     /* Sorted array of labids indexed by context */
   int nci;        /* Number of context independent models */
   LabId *cis;     /* Sorted array of context independent labids */
   int ncf;        /* Number of context free models */
   LabId *cfs;     /* Sorted array of context free labids */
}
HMMSetCxtInfo;

Network *ExpandWordNet(MemHeap *heap,Lattice *lat,Vocab *voc,HMMSet *hset);
/*
   ExpandWordNet converts a lattice to a network.

   It uses the dictionary voc to expand each word in lat into a series
   of pronunciation instances.  How this expansion is performed depends 
   upon the hmms that appear in hset and the value of HNet configuration
   parameters, ALLOWCXTEXP, ALLOWXWRDEXP, FORCECXTEXP, FORCELEFTBI and
   FORCERIGHTBI.

   The expansion proceeds in four stages.
   i)   Context definition.
        It is necessary for the expansion routine to determine how model
	names are constructed from the dictionary entries and whether
	cross word context expansion should be performed.
	Phones in the dictionary are classified as either
	 a) Context Free.       Phone is skipped when determining context.
	 b) Context Indpendent. Phone only exists in CI form.
	 c) Context Dependent.  Otherwise phone needs modelsname expansion.
	This classification depends on whether a phone appears in the context
	part of the name (and this defines the context name) and whether
	and context dependent versions of the phone exist in the HMMSet.
	 
   ii)  Determination of network type.
	The default behaviour is to try and produce the simplest network
	possible. So if the dictionary is closed no expansion of phone
	names is used to get model names, otherwise if word internal
	context expansion will find each model this is used otherwise it
	tries full cross word context expansion.
	This behaviour can be modified by the configuration parameters.
	If ALLOWCXTEXP==FALSE no expansion of phone names (from the
	dictionary) is performed and each phone corresponds to the model
	of the same name.
	If ALLOWXWRDEXP==FALSE expansion across word boundaries is blocked
	and although each phone still corresponds to a single model the 
	phone labels can be expanded to produce a context dependent model
	name.
	If FORCECXTEXP==TRUE an error will be generated if no context
	expansion is possible.

   iii) Network expansion.
        For cross word context expansion the initial and final context
	dependent phones (and any preceding/following context independent
	ones) are duplicated several times to allow for different cross
	word contexts.  Each pronunciation instance has a word end node
	for each left context in which it appears.  (!NULL words just
	have these word nodes).
	Otherwise each word in the lattice is expanded into its different
	pronunciations and these expanded into a node for each phone 
	together with a word end node.  (Again !NULL words just have the 
	word end node).
   
   iv)  Linking of models to network nodes.
        Model names are determined from the phone name and the surrounding
	context names.
	 a)  Construct CD name and see if model exists.
	 b)  Construct CI name and see if model exists.
	If ALLOWCXTEXP==FALSE (a) is skipped and if FORCECXTEXP==TRUE 
	(b) is skipped.  When no matching model is found and error is
	generated.
	The name for (a) is either a left biphone (when the right context
	is a boundary or FORCELEFTBI==TRUE), a right biphone (when the left
	context is a boundary or FORCERIGHTBI==TRUE) or a triphone.
	The resulting name is of the [left_context-]phone[+right_context]
	with the phone label coming direct from the dictionary and the 
	context names coming from (i) above.
	Context free phones are skipped in this process so
	  sil aa r sp y uw sp sil
	would be expanded as
	  sil sil-aa+r aa-r+y sp r-y+uw y-uw+sil sp sil
	if sil was context independent and sp was context free.

   [ Stages (iii) and (iv) actually proceed concurrently to allow sharing
     of logical models with the same underlying physical model for the first
     and last phone of context dependent models ].
*/

/* --- Context handling stuff useful for general network building --- */

HMMSetCxtInfo *GetHMMSetCxtInfo(HMMSet *hset, Boolean frcCxtInd);
/* 
   Create HMMSetCxtInfo for hset - possibly forced to be  CI.
*/

int GetHCIContext(HMMSetCxtInfo *hci,LabId labid);
/* 
   Search hci to find appropriate context number for labid
*/

Boolean IsHCIContextInd(HMMSetCxtInfo *hci,LabId labid);
/* 
   Search hci to find if labid represents a context independent model
*/

HLink GetHCIModel(HMMSetCxtInfo *hci,int lc,LabId name,int rc);
/* 
   Find the appropriate model for particular context
*/

int AddHCIContext(HMMSetCxtInfo *hci,LabId labid);
/* 
   Explicitly add phone into context set.
   Return its context (possibly newly added).
*/

#ifdef __cplusplus
}
#endif

#endif  /* _HNET_H_ */
