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
/*     File: HParse.c: HParse based word-network definition    */
/* ----------------------------------------------------------- */

char *hparse_version = "!HVER!HParse:   3.4.1 [CUED 12/03/09]";
char *hparse_vc_id = "$Id: HParse.c,v 1.1.1.1 2006/10/11 09:55:01 jal58 Exp $";

/* The HParse program reads in a set of HTK  HParse rewrite rules
   (as used in HTK V1.x) and writes out an HTK V2 lattice and
   if operating in V1.x compatability mode possibly a dictionary.

   In compatability mode the interpretation of the HParse network
   is that used by the HTK V1.5 HVite program i.e. the reserved 
   node names WD_BEGIN and WD_END are used to delimit word boundaries 
   - nodes between a WD_BEGIN/WD_END pair are called "word-internal"  
   while all other nodes are  "word-external".  All WD_BEGIN/WD_END nodes 
   must have an external name attached that denotes the word. 
   The connectivity of the words is output in an HTK V2 word lattice format 
   and the pronunciation information stored in an HTK V2 dictionary.
   Word-external nodes are treated as words and stored in the lattice
   with corresponding entries in the dictionary. 

   When not operating in compatability mode all nodes are treated as
   words. 

   Note that information regarding the "external name" as
   used in HTK V1.x is ignored when not operating in compatability mode.
   In compatability mode it is only allowed for a particular external
   name to be used for one WD_BEGIN/WD_END pair. For wdExternal nodes
   the external name is ignored.

   The definition of the rewrite rules for the textual definition 
   are as follows:-

      name = char{char}   -- any sequence of characters except the meta
                              chars {}[]<>|=$();*\/   - the latter must be
                              escaped using backslash
      model = name [ "%" ("%" | name)]    -- option for external name
      variable = $ name
      variable = @ name                  

      factor = "(" expr ")" |    -- simple factoring
               "{" expr "}" |    -- 0 or more repetitions
               "<" expr ">" |    -- 1 or more repetitions
               "[" expr "]" |    -- 0 or 1 repetition ie optional
               "<<" expr ">>" |  -- triphone loop
               model |           -- the name of a HMM 
               variable          -- must be already defined
               
      sequence = factor {factor}          -- sequences
      
      expr   = sequence {"|" sequence}    -- alternatives

      subnet = variable "=" expr ";"      -- define a variable

      network = {subnet} "(" expr ")"     -- the network itself

   All variables must be defined by a subnet definition before being
   used in an expression (this prohibits recursion and makes it easier
   to implement). C style comments may be placed anywhere in the text
   
   The network build process proceeds in 3 steps:
   
      1) the parser builds a  set of subnetworks for each 
         defined in the textual representaion.
      2) the subnetworks are (recursively) substituted to fully
         expand the network. Each sub-network (& many components thereof)
         use additional 'glue' nodes
      3) all glue nodes are removed

*/

/* Trace Flags */
#define T_TOP        0001    /* Top Level tracing */
#define T_HPNET      0002    /* print HParse final network */
#define T_HPMEMSTAT  0004    /* print memory stacks after HP net built */
#define T_HPREMGLUE  0010    /* print progress through Remove Glue */

#include "HShell.h" /* HMM ToolKit Modules */
#include "HMem.h"
#include "HMath.h"
#include "HSigP.h"
#include "HAudio.h"
#include "HWave.h"
#include "HVQ.h"
#include "HParm.h" 
#include "HLabel.h"
#include "HModel.h"
#include "HUtil.h" 
#include "HDict.h"
#include "HNet.h"


/* ------------ HParse network types and definitions -------------- */

typedef struct _Node *Link;

typedef struct {
   int nUse;         /* num sharing this LinkSet */
   short numLinks;   /* number of links in set */
   short maxLinks;   /* max number of links */
   Link *links;      /* array[1..numLinks]of Link */
   Ptr  user;       /* for attaching user defined data */
} LinkSet;

typedef struct _Node{
   LabId modelName;  /* name of node */
   LabId extName;    /* external name (used in compatability mode) */
   LinkSet *succ;    /* successors to this node */
   LinkSet *pred;    /* predecessors to this node */
   Link chain;       /* simple linked list of all nodes */  
   Ptr  user;       /* for attaching user defined data */
} Node;

typedef struct {
   Link entryNode;
   Link exitNode;
   Link chain;
} HPNetwork;

typedef struct _SubNetDef{
   LabId netName;    /* variable name (LHS of rule) */
   HPNetwork network;  /* sub-network (RHS of rule) */ 
   struct _SubNetDef *next;
} SubNetDef;

/* ------  Network Node Labelling (used in conversion process) -------- */

typedef enum {unknown, wdInternal, wdExternal, wdBegin, wdEnd, nullNode} NodeType;

typedef struct _NodeInfo{
   NodeType nType;   /* the type of this node */
   Boolean seen;     /* flag used when scanning network */
   Link history;     /* used for word pronunciation expansion */
   int  nodeNum;     /* store node numbers */
}NodeInfo;

static int trace     = 0;         /* Trace flags */

/* -------------------  Global variables -------------------------- */

static LabId enterId;                 /* LabId of the ENTER node name */
static LabId exitId;                  /* LabId of the EXIT node name */
static LabId wdBeginId;               /* LabId of WD_BEGIN nodes  */
static LabId wdEndId;                 /* LabId of WD_END nodes  */
static int numWdBegin=0;              /* number of WORD_BEGIN nodes */
static int numWdEnd=0;                /* number of WORD_END nodes */

static Boolean v1Compat=FALSE;        /* compatability mode? */
static Boolean saveLatLM=FALSE;       /* output lattice probabilities */
static Boolean saveLatBin=FALSE;      /* save lattice in binary */ 

/* ---------------- Configuration Parameters --------------------- */

static ConfParam *cParm[MAXGLOBS];
static int nParm = 0;            /* total num params */

/* ---------------- Process Command Line ------------------------- */

/* SetConfParms: set conf parms relevant to this tool */
void SetConfParms(void)
{
   int i;
   Boolean b;

   nParm = GetConfig("HPARSE", TRUE, cParm, MAXGLOBS);
   if (nParm>0){
      if (GetConfInt(cParm,nParm,"TRACE",&i)) trace = i;
      if (GetConfBool(cParm,nParm,"V1COMPAT",&b)) v1Compat = TRUE;
   }
}

void ReportUsage(void)
{
   printf("\nUSAGE: HParse [options] netFile latFile\n\n");
   printf(" Option                                       Default\n\n");
   printf(" -b      output lattice in binary             ascii\n");   
   printf(" -c      set V1.x compatability mode          off\n");
   printf(" -d s    output dictionary to file s          none\n");
   printf(" -l      include LM log probs in lattice      off\n");
   PrintStdOpts(""); 
   printf("\n\n");
}

static HPNetwork CreateHParseNetwork(char *fname);
static void PrintHParseNetwork(HPNetwork *network);
static void ConvertHParseNetwork(HPNetwork *network,char *latf,char *dictf);

int main(int argc, char *argv[])
{
   char *netFn,*latFn,*dictFn=NULL;
   char  *s;
   HPNetwork theNet;

   if(InitShell(argc,argv,hparse_version,hparse_vc_id)<SUCCESS)
      HError(3100,"HParse: InitShell failed");

   InitMem();   InitLabel();
   InitMath();  InitDict(); 
   InitNet();  

   if (!InfoPrinted() && NumArgs() == 0)
      ReportUsage();
   if (NumArgs() == 0) Exit(0);
   SetConfParms();

   while (NextArg() == SWITCHARG) {
      s = GetSwtArg();
      if (strlen(s)!=1) 
         HError(3119,"HParse: Bad switch %s; must be single letter",s);
      switch(s[0]){
      case 'b':
         saveLatBin=TRUE; break;
      case 'c':
         v1Compat=TRUE; break;
      case 'd':
         if (NextArg()!=STRINGARG)
            HError(3119,"HParse: Output dictionary name expected");
         dictFn = GetStrArg(); break;
      case 'l':
         saveLatLM=TRUE; break;
      case 'T':
         trace = GetChkedInt(0,511,s); break;
      default:
         HError(3119,"HParse: Unknown switch %s",s);
      }
   } 
   if (NextArg()!=STRINGARG)
      HError(3119,"HParse: network file name expected");
   netFn = GetStrArg();
   if (NextArg()!=STRINGARG)
      HError(3119,"HParse: output lattice file name expected");
   latFn = GetStrArg();

   if ((dictFn != NULL) && (!v1Compat))
      HError(3119,"HParse: Dictionary only valid in compatability mode");

   /* Set Up The Network using specification in 'netfn' */
   exitId  = GetLabId("$$$HPARSE_EXIT",TRUE); 
   enterId = GetLabId("$$$HPARSE_ENTER",TRUE);
   wdBeginId = GetLabId("WD_BEGIN",TRUE);
   wdEndId = GetLabId("WD_END",TRUE);
   if (trace > 0)
      printf("Creating HParse net from file %s\n",netFn);
   theNet =  CreateHParseNetwork(netFn); 
   if (trace&T_HPNET) 
      PrintHParseNetwork(&theNet);
   ConvertHParseNetwork(&theNet,latFn,dictFn);
   Exit(0);
   return (0);          /* never reached -- make compiler happy */
}


/* --------------------- HParse network building ----------------------- */


#define LINKCHUNKSIZE 3
#define LINKEXTENTFACTOR 1.5
#define MAXIDENT 80
typedef char Ident[MAXIDENT+1];

#define NODEBLOCK 512
#define LSBLOCK 1024


static long numLinkSets = 0;     /* usage counters */
static long numLinks = 0;
static long numNodes = 0;
static MemHeap nodeHeap;
static MemHeap lsHeap;
static MemHeap lsChunkHeap;
static MemHeap lsLargeHeap;


/* ---------------- Node and LinkSet Manipulation ---------------------- */

/* CreateLinkSet: allocate space for a LinkSet with >= size slots */
LinkSet *CreateLinkSet(int size)
{
   LinkSet *p;
   Link *q;
   
   p = (LinkSet *) New(&lsHeap,sizeof(LinkSet));
   if ( size <= LINKCHUNKSIZE) { 
      size = LINKCHUNKSIZE;
      q = (Link *) New(&lsChunkHeap,LINKCHUNKSIZE*sizeof(Link));
   } else   /* create a new one */
      q = (Link *) New(&lsLargeHeap,size*sizeof(Link));
   p->links = q-1;
   p->user = NULL;
   p->nUse = 1;                        /* set usage counter to 1 */
   p->numLinks = 0; p->maxLinks = size;
   ++numLinkSets; numLinks += size;    /* update statistics */ 
   return p;
}

/* FreeLinkSet: free the given linkset */
static void FreeLinkSet(LinkSet *p)
{
   Link *q;
   
   if (p == NULL) return;
   if (p->nUse >1 ) {
      --(p->nUse); return;
   }
   q = p->links+1;
   if (p->maxLinks == LINKCHUNKSIZE)  /* give back to the block store */
      Dispose(&lsChunkHeap,q);
   else 
      Dispose(&lsLargeHeap,q);
   numLinks -= p->maxLinks; --numLinkSets;
   Dispose(&lsHeap,p);
}

/* ResizeLinkSet: change number of slots in ls to newSize slots */
static void ResizeLinkSet(LinkSet *ls, int newSize)
{
   Link *p;
   int oldSize;
   int oldEntries;
   int i;
   
   oldSize = ls->maxLinks;
   oldEntries = ls->numLinks;
   if (newSize==0){
      numLinks -= ls->maxLinks;
      if (ls->maxLinks == LINKCHUNKSIZE)
         Dispose(&lsChunkHeap,ls->links+1);
      else
         Dispose(&lsLargeHeap,ls->links+1);
      ls->numLinks = 0; ls->maxLinks = 0; 
      ls->links = NULL;    
   }else if (ls->maxLinks != LINKCHUNKSIZE) {
      p = (Link *)New(&lsLargeHeap,newSize*sizeof(Link));
      p--;
      for (i=1; (i<=oldEntries) && (i <= newSize); i++)
         p[i] = ls->links[i];
      Dispose(&lsLargeHeap,ls->links+1);
      ls->links = p;
      ls->maxLinks = newSize;
      numLinks += newSize-oldSize;
   }else if (newSize > ls->maxLinks) { /* ls->maxLinks == LINKCHUNKSIZE */
      p = (Link *)New(&lsLargeHeap,newSize*sizeof(Link));
      p--;
      for (i=1; i<=ls->numLinks; i++)
         p[i] = ls->links[i];
      Dispose(&lsChunkHeap,ls->links+1);
      ls->links = p;
      ls->maxLinks = newSize;
      numLinks += newSize-oldSize;
   }
}

/* PrModelName: print name of model p to stdout */
static void PrModelName(Link p)
{
   Ident name;
   
   if (p->modelName==NULL || p->modelName->name==NULL)
      strcpy(name,"****");
   else if (strlen(p->modelName->name) > MAXIDENT)
      strcpy(name,"????");
   else
      strcpy(name,p->modelName->name);
   printf("%s[%03d] ",name, ((int)p % 4000) / 4 );
}

/* PrintLinkSet: print first n slots of given LinkSet to stdout */
static void PrintLinkSet(int n, LinkSet *p)
{
   int i;
   
   if (p==NULL || n==0)
      printf(" ---");
   else{
      if (n > p->numLinks) n = p->numLinks;
      printf("[U:%d] ",p->nUse);
      for (i=1;i<=n;i++) PrModelName(p->links[i]);
   }
   printf("\n");
}

/* CreateNode: create node and link it into given chain */
Link CreateNode(LabId name, Link *chain, int maxSucc, int maxPred)
{
   Link p;
   
   p = (Link) New(&nodeHeap,sizeof(Node)); 
   p->modelName=name; p->extName = name;
   p->succ = (maxSucc>0)?CreateLinkSet(maxSucc):NULL;
   p->pred = (maxPred>0)?CreateLinkSet(maxPred):NULL;
   p->chain = *chain; *chain=p;
   p->user  = NULL;
   ++numNodes;
   return p;
}

/* FreeNode:  release storage allocated to p */
static void FreeNode(Link p)
{
   FreeLinkSet(p->succ);
   FreeLinkSet(p->pred);
   --numNodes;
   Dispose(&nodeHeap,p);
}

/* ShrinkNode:  reduces sizes of LinkSets to minimum */
static void ShrinkNode(Link p)
{
   LinkSet *ls;
   
   ls = p->succ;
   if (ls->maxLinks > ls->numLinks)
      ResizeLinkSet(ls,ls->numLinks);
   ls = p->pred;
   if (ls->maxLinks > ls->numLinks)
      ResizeLinkSet(ls,ls->numLinks);
}

/* FreeNetwork: free storage allocated for network */
void FreeNetwork(HPNetwork *network)
{
   Link p = network->chain;
   Link q;

   while (p!=NULL) {
      q = p->chain;
      FreeNode(p); 
      p = q;
   }
}

/* NotLinked: returns true if node x is not in link set ls */
static Boolean NotLinked(LinkSet *ls, Link x)
{
   int i;
   Link *p;
   
   p = ls->links+1;
   for (i=1; i<=ls->numLinks;i++,p++)
      if (x == *p) return FALSE;
   return TRUE;
}

/* JoinNodes: join node a -> node b */
void JoinNodes(Link a, Link b)
{
   LinkSet *asucc,*bpred;
   
   asucc = a->succ; bpred = b->pred;
   if (NotLinked(asucc,b)) {
      if (asucc->numLinks == asucc->maxLinks) {
         ResizeLinkSet(asucc,(int)(asucc->maxLinks*LINKEXTENTFACTOR+1));
      }
      asucc->links[++(asucc->numLinks)] = b;
      if (asucc->nUse<0) asucc->nUse = -(asucc->nUse);  /* undo marks */
   }
   if (NotLinked(bpred,a)) {
      if (bpred->numLinks == bpred->maxLinks) {
         ResizeLinkSet(bpred,(int)(bpred->maxLinks*LINKEXTENTFACTOR+1));
      }
      bpred->links[++(bpred->numLinks)] = a; 
      if (bpred->nUse<0) bpred->nUse = -(bpred->nUse);  /* undo marks */
   }
}

/* TailMerge: join a to b by sharing succ link sets */
void TailMerge(Link a, Link b)
{
   Link pNode;
   LinkSet *shared,*bpred;
   
   bpred = b->pred;
   pNode = bpred->links[1];      /* assume all preds share same link set */
   shared = pNode->succ; ++(shared->nUse);
   FreeLinkSet(a->succ); a->succ = shared;
   if (NotLinked(bpred,a)) {
      if (bpred->numLinks == bpred->maxLinks) {
         ResizeLinkSet(bpred,(int)(bpred->maxLinks*LINKEXTENTFACTOR+1));
      }
      bpred->links[++(bpred->numLinks)] = a; 
   }
}

/* HeadMerge: join a to b by sharing pred link sets */
void HeadMerge(Link a, Link b)
{
   Link sNode;
   LinkSet *shared,*asucc;
   
   asucc = a->succ;
   sNode = asucc->links[1];      /* assume all succs share same link set */
   shared = sNode->pred; ++(shared->nUse);
   FreeLinkSet(b->pred); b->pred = shared;
   if (NotLinked(asucc,b)) {
      if (asucc->numLinks == asucc->maxLinks) {
         ResizeLinkSet(asucc,(int)(asucc->maxLinks*LINKEXTENTFACTOR+1));
      }
      asucc->links[++(asucc->numLinks)] = b;
   }
}

/* PrintNode: print node p to stdout */
static void PrintNode(Link p)
{
   PrModelName(p); 
   printf("%s\n",(p->extName==NULL)?"<Blank>":p->extName->name);
   printf("  Pred: "); 
   PrintLinkSet(20,p->pred);
   printf("  Succ: "); 
   PrintLinkSet(20,p->succ);
   fflush(stdout);
}


/* PrintHParseNetwork: print network to stdout */
static void PrintHParseNetwork(HPNetwork *network)
{
   Link p = network->chain;
   
   while (p!=NULL) {
      PrintNode(p);
      p=p->chain;
   }
   printf("\n");
}

 
/* PrintStats: print stats on memory usage */
void PrintStats(void)
{
   printf("Network Memory Usage:\n");
   printf("Nodes: %ld Nodes, %ld Bytes\n",numNodes,numNodes*sizeof(Node));
   printf("Links: %ld Sets, %ld Slots, %ld Bytes\n",numLinkSets,
          numLinks, numLinks*sizeof(Link)+numLinkSets*sizeof(LinkSet));
   fflush(stdout);
}

/* --------------------------- Network Handling ---------------------- */

static Link curChain;       /* current network being built */

/* CloneNetwork: returns a clone of given network.  
      Works by attaching a copy of each node in prototype to 
      user slot, then replacing each link to node p in the copies 
      by a link to p->user  */
HPNetwork CloneNetwork(HPNetwork prototype)
{
   HPNetwork clone;
   Link p = prototype.chain;
   Link *oldL,*newL,q,p2,q2;
   int i,nx,nu;
   
   clone.chain=NULL; 
   while (p!=NULL) {    /* copy all nodes of prototype */
      q = CreateNode(p->modelName,&clone.chain,0,0);
      q->extName = p->extName;
      p->user = (Ptr) q; 
      p=p->chain;
   }
   p = prototype.chain; /* now fix links */
   while (p!=NULL) {
      q=(Link)p->user;
      if (q->succ == NULL && p->succ != NULL) {     /* otherwise its null or */
         q->succ = CreateLinkSet(p->succ->maxLinks); /* already shared */
         q->succ->numLinks = p->succ->numLinks;
         oldL=p->succ->links+1; newL=q->succ->links+1;
         for (i=1; i<=p->succ->numLinks; i++) {
            *newL++ = (Link) (*oldL)->user; oldL++;
         }
         if ((nu=p->succ->nUse) > 1) {    /* shared linkset */
            p2 = prototype.chain; nx = 1;
            while (p2 != NULL){
               if ( p != p2 && p->succ == p2->succ){
                  ++nx;
                  q2 = (Link)p2->user;
                  if (q2->succ != NULL)
                     HError(3190,"CloneNetwork: Overwriting a succ linkset");
                  q2->succ = q->succ;
               }
               p2 = p2->chain;
            }
            if (nu != nx)
               HError(3190,"CloneNetwork: shared succ linkset count %d vs %d",nx,nu);
            q->succ->nUse = nu;
         }
      }
      if (q->pred == NULL && p->pred != NULL) {     /* otherwise its null or */
         q->pred = CreateLinkSet(p->pred->maxLinks); /* already shared */
         q->pred->numLinks = p->pred->numLinks;
         oldL=p->pred->links+1; newL=q->pred->links+1;
         for (i=1; i<=p->pred->numLinks; i++) {
            *newL++ = (Link) (*oldL)->user; oldL++;
         }
         if ((nu=p->pred->nUse) > 1) {    /* shared linkset */
            p2 = prototype.chain; nx = 1;
            while (p2 != NULL){
               if ( p != p2 && p->pred == p2->pred){
                  ++nx;
                  q2 = (Link)p2->user;
                  if (q2->pred != NULL)
                     HError(3190,"CloneNetwork: Overwriting a pred linkset");
                  q2->pred = q->pred;
               }
               p2 = p2->chain;
            }
            if (nu != nx)
               HError(3190,"CloneNetwork: shared pred linkset count %d vs %d",nx,nu);
            q->pred->nUse = nu;
         }
      }
      p=p->chain;
   }
   clone.entryNode = (Link)prototype.entryNode->user;
   clone.exitNode = (Link)prototype.exitNode->user;
   return clone;
}
   
/* ------------------------ Variable (SubNet) Defs ------------------- */

static SubNetDef *defs;
static LabId  subNetId;
static MemHeap subNetHeap;

static void  RemoveGlue(HPNetwork *network);

/* InitSubNetDefs: set the list of subnet defs to empty */
static void InitSubNetDefs(void)
{
   /* SubNetDef * dummy; */

   defs = NULL;
   subNetId = GetLabId("$$$HParse_SubNet",TRUE);
   CreateHeap(&subNetHeap,"HParse SubNet Heap",MHEAP,sizeof(SubNetDef), 1.5, 1000, 10000);
   /* dummy = (SubNetDef *) New(&subNetHeap,sizeof(SubNetDef)); */
   /* Dispose(&subNetHeap,dummy); */
}

/* DefineSubNet: define a subnet with given name */
static void DefineSubNet(LabId name, Link entryNode, Link exitNode, Link chain)
{
   SubNetDef *p;
   
   p=(SubNetDef *)New(&subNetHeap,sizeof(SubNetDef));
   p->netName=name;
   p->network.entryNode = entryNode;
   p->network.exitNode = exitNode;
   p->network.chain = chain;
   RemoveGlue(&(p->network)); 
   p->next = defs; defs = p;
   /* store pointer to sub network in NameCell */ 
   name->aux = (Ptr) &p->network; 
}

/* FreeSubNetDefs: free all storage used by subnet definitions */
static void FreeSubNetDefs(void)
{
   SubNetDef *p;
   
   while (defs!=NULL)
      {
         FreeNetwork(&defs->network);
         defs->netName->aux=NULL;         /*reset aux NameCell field*/
         p=defs->next; defs=p;
      }
   ResetHeap(&subNetHeap);
}

/* ------------ Special Triphone Loop Builder ------------- */

typedef struct{
   LabId l;
   LabId r;
   LabId m;
}SplitName;

MemHeap  joinHeap;
MemHeap  splitNameHeap;
MemHeap  subNetStoreHeap;
MemHeap  jMatHeap;

typedef unsigned char **JoinMatrix; 
static JoinMatrix jmat;        /* binary join matrix */
static int jmRows,jmCols;      /* size of JoinMatrix */

/* SplitTriName: splits name n of form A-B+C in to 3 parts 
   if either context is missing then the corresponding part 
   is null */
static void SplitTriName(LabId n, SplitName *x)
{
   char buf[64],*p;

   x->l = x->r = NULL;
   strcpy(buf,n->name);
   if ((p=strrchr(buf,'+')) != NULL) { /* Right Context found */
      *p++ = '\0';
      x->r = GetLabId(p,TRUE);
   }
   if ((p=strchr(buf,'-')) != NULL) {  /* Left Context found */
      *p++ = '\0';
      x->m = GetLabId(p,TRUE);
      x->l = GetLabId(buf,TRUE);
   } else 
      x->m = GetLabId(buf,TRUE);
}

/* MakeTriList: split all the elements of a triloop into component parts */
/*              stores the result in trilist */
static SplitName* MakeTriList(int numElements, LinkSet *asucc)
{
   int i;
   Link thisL;
   SplitName *trilist;
   
   /* reserve space for numElements plus TLOOP_BEGIN & TLOOP_END if not present */
   CreateHeap(&splitNameHeap,"HParse Split Name Heap",MHEAP,(numElements+2)*sizeof(SplitName),0,1,1);
   trilist = (SplitName *) New(&splitNameHeap,(numElements+2)*sizeof(SplitName));
   for (i=0;i<numElements;i++){   /* Build split name list */
      thisL = asucc->links[i+1];
      SplitTriName(thisL->modelName,trilist+i);
   }
   return trilist;
}

/* FindLoopBegin: find the glue node before the loop begins   */
/*                free the LinkSets of the skipped glue nodes */
Link FindLoopBegin(Link *hd)
{
   Link a;
   LinkSet *asucc;

   a = *hd;
   while (a->succ->links[1]->modelName == NULL) {
      FreeLinkSet(a->pred); a->pred = NULL;
      asucc = a->succ;  a->succ = NULL;
      a = asucc->links[1]; FreeLinkSet(asucc); 
   }
   return a;
}   

/* FindLoopEnd: find the glue node after the loop ends     */
/*              free the LinkSets of the skipped glue nodes */
Link FindLoopEnd(Link *tl)
{
   Link b;
   LinkSet *bpred;

   b = *tl;
   while (b->pred->links[1]->modelName == NULL) {
      FreeLinkSet(b->succ); b->succ = NULL;
      bpred = b->pred;  b->pred = NULL;
      b = bpred->links[1]; FreeLinkSet(bpred); 
   }
   return b;
}   

void SwapSN(SplitName *sn1, SplitName *sn2)
{
   SplitName tmpSN;

   tmpSN = *sn2; *sn2 = *sn1; *sn1 = tmpSN;
}    

void SwapNodeNames(Link l1, Link l2)
{
   LabId tmpId;
    
   tmpId = l2->modelName; 
   l2->modelName = l1->modelName; 
   l1->modelName = tmpId;
   tmpId = l2->extName; 
   l2->extName = l1->extName; 
   l1->extName = tmpId;
}

/* AddTLoopBeginEnd: add TLOOP_BEGIN & TLOOP_END if not present  */
/*                   make them last elements of loop and trilist */
/*                   return (new) number of elements in loop     */
int AddTLoopBeginEnd(int numElements, SplitName* trilist, Link a, Link b)
{
   LabId loopBeginId, loopEndId;
   SplitName *tlb, *tle;
   Boolean beginFound = FALSE;
   Boolean endFound = FALSE;
   int total = numElements;
   int ib,ie;
   LinkSet *asucc;
   Link p;
   
   loopBeginId = GetLabId("TLOOP_BEGIN",TRUE);
   loopEndId = GetLabId("TLOOP_END",TRUE);
   tlb = trilist;
   for (ib = 0; ib < numElements; ib++) {
      tlb = trilist+ib;
      if (tlb->m == loopBeginId) {
         beginFound = TRUE; break;
      }  
   }  
   tle = trilist;
   for (ie = 0; ie < numElements; ie++) {
      tle = trilist+ie;
      if (tle->m == loopEndId) {
         endFound = TRUE; break;
      }  
   }  
   if (!beginFound) {
      p=CreateNode(loopBeginId,&curChain,LINKCHUNKSIZE,LINKCHUNKSIZE);      
      p->extName = loopBeginId;
      HeadMerge(a, p);
      TailMerge(p, b);
      (trilist+total)->m = loopBeginId;
      (trilist+total)->r = NULL;
      tlb = trilist+total; ib = total;
      total++;
   }   
   if (!endFound) {
      p=CreateNode(loopEndId,&curChain,LINKCHUNKSIZE,LINKCHUNKSIZE);      
      p->extName = loopEndId;
      HeadMerge(a, p);
      TailMerge(p, b);
      (trilist+total)->m = loopEndId;
      (trilist+total)->l = NULL;
      tle = trilist+total; ie = total;
      total++;
   }
   asucc=a->succ;
   if (ie != total-1) {  /* need to swap around */
      SwapSN(trilist+total-1,tle);
      SwapNodeNames(asucc->links[ie+1], asucc->links[total]);
      if (ib == total-1) {
         ib = ie; tlb = trilist+ib;
      }
   }
   if (ib != total-2) {  /* need to swap around */
      SwapSN(trilist+total-2,tlb);
      SwapNodeNames(asucc->links[ib+1], asucc->links[total-1]);
   }
   (trilist+total-2)->l = GetLabId("",TRUE);
   (trilist+total-1)->r = GetLabId("",TRUE);
   return total;
}

/* DoesMatch: returns true if name s is equal to snet or is a
              member of subnet snet */
static int DoesMatch(LabId s, LabId snet)
{
   HPNetwork *subnet;
   Link *p;
   LinkSet *asucc;
   int i;

   if (snet == NULL) return TRUE;            /* No context match required */
   if ((subnet = (HPNetwork *) snet->aux) == NULL) 
      return s == snet;            /* No subnet so use simple match */
   else {
      asucc = subnet->entryNode->succ; p = asucc->links+1;
      for (i=1;i<=asucc->numLinks;i++,p++)
         if (s == (*p)->modelName) return TRUE;
      return FALSE;
   }
}

static void CreateJMat(void)
{
   int i,j;
   unsigned char *p;
   int jmatsize;   
 
   jmatsize = jmRows*sizeof(unsigned char *);
   CreateHeap(&jMatHeap,"HParse JMat Heap",MSTAK,1,0.0,jmatsize,jmatsize);
   jmat = (JoinMatrix) New(&jMatHeap,jmRows*sizeof(unsigned char *));
   for (i=0; i<jmRows; i++){
      p = (unsigned char *) New(&jMatHeap,jmCols);
      for (j=0;j<jmCols;j++) p[j] = 0;
      jmat[i] = p;
   }
}

static void ClearJMat(void)
{
   int i,j;
   
   for (i=0; i<jmRows; i++)
      for (j=0; j< jmCols; j++)
         jmat[i][j] = 0;
}

static void FreeJMat(void)
{
   int i;
   
   for (i=jmRows-1; i>=0; i--) 
      Dispose(&jMatHeap,jmat[i]);
   Dispose(&jMatHeap,jmat);
}

/* SameLinks: Check to see if a1 and a2 have the same succs or preds */
static Boolean SameLinks(int a1, int a2)
     /* note: succs or preds checked depending on contents of jmat */
{
   Boolean same = TRUE;
   int j;
   unsigned char *p, *q;
   
   p=jmat[a1]; q=jmat[a2];
   for (j=0; same && j<jmCols; j++)
      if (*p++ != *q++) same = FALSE;
   return same;
}

static Boolean IsJoined(int a, int b)
{
   return (jmat[a][b/8] & (1 <<(b&7))) != 0;
}

static int NumJSuccs(int a)
{
   unsigned char x, *p;
   int j, count =0;

   p = jmat[a];
   for (j=0; j<jmCols; j++)
      for (x=*p++; x!=0; x>>=1)
         if (x&1) ++count;
   return count;
}

static int NumJPreds(int a)
{
   unsigned char x, *p;
   int j, count=0;

   p = jmat[a];
   for (j=0; j<jmCols; j++)
      for (x=*p++; x!=0; x>>=1)
         if (x&1) ++count;
   return count;
}

/* FillSuccJM: find joins with succs as rows of jmat */
static void FillSuccJM(int numElements, SplitName *trilist)
{
   SplitName *tli,*tlj;
   int i,j;
   
   for (i=0;i<numElements-2;i++){ 
      tli = trilist+i;
      for (j=0;j<numElements-2;j++){
         tlj = trilist+j;
         if (DoesMatch(tlj->m,tli->r) && DoesMatch(tli->m,tlj->l))
            jmat[i][j/8] |= 1 <<(j&7);
      }
   }
   /* do the succs that point from TLOOP_BEGIN */
   i = numElements-2;  
   for (j=0,tlj=trilist;j<numElements-2;j++,tlj++)
      if (DoesMatch(tlj->m,(trilist+i)->r))
         jmat[i][j/8] |= 1 << (j&7);
   /* do the succs that point to TLOOP_END */
   j = numElements-1;  
   for (i=0,tli=trilist;i<numElements-2;i++,tli++)
      if (DoesMatch(tli->m,(trilist+j)->l))
         jmat[i][j/8] |= 1 << (j&7);
}

/* FillPredJM: find joins with preds as rows of jmat */
static void FillPredJM(int numElements, SplitName *trilist)
{
   SplitName *tli,*tlj;
   int i,j;

   for (i=0;i<numElements-2;i++){   
      tli = trilist+i;
      for (j=0;j<numElements-2;j++){
         tlj = trilist+j;
         if (DoesMatch(tli->m,tlj->r) && DoesMatch(tlj->m,tli->l))
            jmat[i][j/8] |= 1 <<(j&7);
      }
   }
   /* do the preds that point from TLOOP_END */
   i = numElements-1;  
   for (j=0,tlj=trilist;j<numElements-2;j++,tlj++)
      if (DoesMatch(tlj->m,(trilist+i)->l))
         jmat[i][j/8] |= 1 << (j&7);
   /* do the preds that point to TLOOP_BEGIN */
   j = numElements-2;  
   for (i=0,tli=trilist;i<numElements-2;i++,tli++)
      if (DoesMatch(tli->m,(trilist+j)->r))
         jmat[i][j/8] |= 1 << (j&7);
} 
 
/* MakeTriSubNets: Make up sub net for each variable defn in triloop */
/*                 Make the node user field point to  network        */
static void MakeTriSubNets(LinkSet *asucc, int numElements, 
                           SplitName *trilist)
{
   HPNetwork *proto, *thisNet;
   int i;
   Link thisL,p;
   int numSubNets = 0;

   for (i=0; i<numElements; i++)   
      if ((HPNetwork *) (trilist[i].m)->aux != NULL)
         numSubNets++;
   CreateHeap(&subNetStoreHeap,"HParse SubNetStore Heap",MHEAP,sizeof(HPNetwork),0.0,
              numSubNets,numSubNets);
   if (numSubNets > 0)
      for (i=0; i<numElements; i++) {   
         thisL = asucc->links[i+1];
         proto = (HPNetwork *) (trilist[i].m)->aux;
         if (proto!=NULL) {        /* clone the def */
            thisNet = (HPNetwork *) New(&subNetStoreHeap,sizeof(HPNetwork));
            *thisNet = CloneNetwork(*proto);
            thisL->user = (void*) thisNet;
            p = thisNet->chain;
            while(p->chain!=NULL)p=p->chain;
            p->chain=curChain; curChain = thisNet->chain;
            thisL->modelName = NULL; /* turn into glue */
            /* thisL->extName = NULL; turn into glue */
         }
         else
            thisL->user = NULL;
      }
}

static void MakeSuccLinks(int numElements, LinkSet *asucc)
{
   int i,j,k,n;
   Link thisL, p;
   LinkSet *ls;

   for (i=0; i<numElements; i++) {    
      if (i == numElements-1) {
         asucc->links[i+1]->succ->numLinks = 0; /* clear succs for TLOOP_END */
         continue;                              /* & don't make any new ones */
      }
      p = NULL;
      thisL = asucc->links[i+1];
      for (j=0; j<i; j++)            /* look to see if same set of */
         if (SameLinks(i,j)) {       /* succs already created */
            p = asucc->links[j+1];
            break;
         }
      if (p!=NULL) {                 /* use previous link set */
         ls = (p->user != NULL) ? ((HPNetwork *) p->user)->exitNode->succ 
            : p->succ ;
         FreeLinkSet(thisL->succ); thisL->succ = NULL;
         if (thisL->user != NULL) { /* node has sub-net attached */
            FreeLinkSet(((HPNetwork *)thisL->user)->exitNode->succ); 
            ((HPNetwork *)thisL->user)->exitNode->succ = ls;
         }
         else 
            thisL->succ = ls;
         ++(ls->nUse);
      } else {                      
         FreeLinkSet(thisL->succ);  /* free the old one */
         thisL->succ = NULL;
         n = NumJSuccs(i);         /* and create a new one */
         ls = CreateLinkSet(n);
         for (j=0,k=0; j<jmRows; j++)                
            if (IsJoined(i,j)) {
               if (asucc->links[j+1]->user != NULL)
                  ls->links[++k] = 
                     ((HPNetwork *)asucc->links[j+1]->user)->entryNode;
               else
                  ls->links[++k] = asucc->links[j+1];
            }
         ls->numLinks = k;
         if (thisL->user != NULL) { /* node has sub-net attached */
            FreeLinkSet(((HPNetwork *)thisL->user)->exitNode->succ); 
            ((HPNetwork *)thisL->user)->exitNode->succ = ls;
         }
         else
            thisL->succ = ls; 
      }
   }
}

static void MakePredLinks(int numElements, LinkSet *asucc)
{
   int i,j,k,n;
   Link thisL, p;
   LinkSet *ls;

   for (i=0; i<numElements; i++) {    
      if (i == numElements-2) {
         asucc->links[i+1]->pred->numLinks = 0; /* clear preds for TLOOP_BEGIN */
         continue;                              /* & don't make new preds      */
      }
      p = NULL;
      thisL = asucc->links[i+1];
      for (j=0; j<i; j++)            /* look to see if same set of */
         if (SameLinks(i,j)) {       /* preds already created */
            p = asucc->links[j+1];
            break;
         }
      if (p!=NULL) {                 /* use previous link set */
         ls = (p->user != NULL) ? ((HPNetwork *)p->user)->entryNode->pred 
            : p->pred ;
         FreeLinkSet(thisL->pred); thisL->pred = NULL;
         if (thisL->user != NULL) {    /* node has sub-net attached */
            FreeLinkSet(((HPNetwork *)thisL->user)->entryNode->pred); 
            ((HPNetwork *)thisL->user)->entryNode->pred = ls;
         }
         else 
            thisL->pred = ls;
         ++(ls->nUse);
      } else {                      
         FreeLinkSet(thisL->pred);   /* free old one */
         thisL->pred = NULL;
         n = NumJPreds(i);          /* and create new one */
         ls = CreateLinkSet(n);
         for (j=0,k=0 ; j<jmRows; j++) 
            if (IsJoined(i,j)) {
               if (asucc->links[j+1]->user != NULL)
                  ls->links[++k] = 
                     ((HPNetwork *)asucc->links[j+1]->user)->exitNode;
               else
                  ls->links[++k] = asucc->links[j+1];
            }
         ls->numLinks = k;
         if (thisL->user != NULL) {  /* node has sub-net attached */
            FreeLinkSet(((HPNetwork *)thisL->user)->entryNode->pred);
            ((HPNetwork *)thisL->user)->entryNode->pred = ls;
         }
         else
            thisL->pred = ls;
      }
   }
}
/* FixTLoopEnds: connect TLOOP_BEGIN and TLOOP_END as loop ends */
/*               and clean up initial loop ends                 */
static void FixTLoopEnds(int numElements, Link *hd, Link *tl, Link a, Link b)                 
{
   *hd = a->succ->links[numElements-1];  /* connect the ends */
   *tl = a->succ->links[numElements];
   (*hd)->modelName = NULL;            /* make TLOOP_BEGIN & TLOOP_END */
   (*tl)->modelName = NULL;            /* into glue                    */
   FreeLinkSet(a->succ); a->succ = NULL;
   FreeLinkSet(a->pred); a->pred = NULL;
   FreeLinkSet(b->succ); b->succ = NULL;
   FreeLinkSet(b->pred); b->pred = NULL;
}

/* MakeTriLoop:  builds a context dependent triphone loop around
      given expression which must be a simple list of alternatives.
      Each triphone in list must have a name of the form L-X+R 
      where L and R are either simple names or the names of
      subnetworks which again must be simple lists.  Each triphone 
      L1-X1+R1 is looped back to all phones L2-X2+R2 for which X1 
      is in L2 and X2 is in R1.  Either the left or right context
      may be missing in which case simple right or left context
      dependence is implemented. The special symbols TLOOP_BEGIN
      and TLOOP_END control entry and exit to/from the loop
*/
static void MakeTriLoop(Link *hd, Link *tl)
{
   int numElements;
   SplitName *trilist;
   Link a, b;

   a = FindLoopBegin(hd);   /* skip (and free) nodes at either end */
   b = FindLoopEnd(tl);
   if (a->succ->numLinks != b->pred->numLinks)
      HError(3131,"MakeTriLoop: Incorrectly formed tri-loop detected");
   numElements = a->succ->numLinks;              /* num elements in the tri loop */
   trilist = MakeTriList(numElements,a->succ);   /* split all the triphone names */
   numElements = AddTLoopBeginEnd(numElements,trilist,a,b);
   MakeTriSubNets(a->succ,numElements,trilist);  
   jmRows = numElements; jmCols = (jmRows+7)/8;
   CreateJMat();                               /* create the join matrix */
   ClearJMat();
   FillSuccJM(numElements, trilist);   /* load jmat with succ first info */
   MakeSuccLinks(numElements,a->succ);                   
   ClearJMat();
   FillPredJM(numElements, trilist);   /* load jmat with pred first info */
   MakePredLinks(numElements,a->succ);
   FixTLoopEnds(numElements,hd,tl,a,b);
   FreeJMat();
   DeleteHeap(&splitNameHeap);
   DeleteHeap(&subNetStoreHeap);
}

/*  ------------ Expand Sub Net Defs ------------------   */

void PrintChain(Link chain)
{
   Link p = chain;
   
   printf("\n");
   while (p!=NULL) {
      PrintNode(p);
      p=p->chain;
   }
}

static void SubstituteSubNet(HPNetwork *subNet, Link p)
{
   Link predNode,succNode;
   int i,j;

   /* first do case of  self-loop on p */
   for (i=1; i <= p->pred->numLinks; i++) 
      if (p->pred->links[i] == p)
         p->pred->links[i] = subNet->exitNode;
   for (i=1; i <= p->succ->numLinks; i++) 
      if (p->succ->links[i] == p)
         p->succ->links[i] = subNet->entryNode;

   /* now go through all the pred nodes of p */
   for (i=1; i <= p->pred->numLinks; i++) {
      predNode = p->pred->links[i];
      if (predNode->succ->nUse > 0) {
         predNode->succ->nUse = -predNode->succ->nUse; /*mark as seen*/
         for (j=1; j <= predNode->succ->numLinks; j++)
            if (predNode->succ->links[j] == p) 
               predNode->succ->links[j] = subNet->entryNode;
      }
   }
   for (i=1; i <= p->pred->numLinks; i++) {  /*restore nUse counts*/ 
      predNode = p->pred->links[i];
      if (predNode->succ->nUse < 0) 
         predNode->succ->nUse = -predNode->succ->nUse; 
   }

   /* and all the succ nodes of p */
   for (i=1; i<=p->succ->numLinks; i++) {
      succNode = p->succ->links[i];
      if (succNode->pred->nUse > 0) {
         succNode->pred->nUse = -succNode->pred->nUse; /*mark as seen*/
         for (j=1; j<= succNode->pred->numLinks; j++)
            if (succNode->pred->links[j] == p) 
               succNode->pred->links[j] = subNet->exitNode;
      }
   }
   for (i=1; i <= p->succ->numLinks; i++) {  /*restore nUse counts*/ 
      succNode = p->succ->links[i];
      if (succNode->pred->nUse < 0) 
         succNode->pred->nUse = -succNode->pred->nUse; 
   }
   FreeLinkSet(subNet->entryNode->pred); 
   FreeLinkSet(subNet->exitNode->succ);  
   subNet->entryNode->pred = p->pred;
   subNet->entryNode->pred->nUse++;
   subNet->exitNode->succ = p->succ;
   subNet->exitNode->succ->nUse++;
}

static void ExpandSubNetDefs(Link *chain)
{
   Link p,q;
   HPNetwork *proto;
   HPNetwork subNet;
   
   p = *chain; *chain = NULL;
   while (p != NULL) {
      if (p->extName == subNetId) { 
         if ((HPNetwork *) p->modelName->aux == NULL) 
            HError(3130,"ExpandSubNetDefs: Variable %s is undefined",p->modelName->name);
         proto = (HPNetwork *) p->modelName->aux;
         subNet = CloneNetwork(*proto);
         SubstituteSubNet(&subNet,p);
         q = subNet.chain;
         while (q->chain != NULL) q=q->chain;
         q->chain = p->chain; 
         FreeNode(p);      /* free sub-net node */
         p = subNet.chain; /* process its def instead */
      }          
      else {
         q = p->chain;   /* form new chain in reverse order */
         p->chain = *chain;
         *chain = p;
         p = q;
      }  
   }          
}          

/* ----------------------- Scanner ------------------------ */

#define ESCAPE '\\'
#define SCANBUFMAX  255    /* amount read from input at one time */
#define H_EOF '\0'

enum _Symbol{NAMESYM, VARSYM, VARATSYM, LPARSYM, RPARSYM, LBRACESYM, 
             RBRACESYM, LANGSYM, RANGSYM, LBRAKSYM, RBRAKSYM, 
             LTRISYM,RTRISYM,EQSYM, SEMISYM, BARSYM, PERCENTSYM, 
             EOFSYM};
             
             
static LabId ident;                 /* Current identifier, if any */
static char inlyne[SCANBUFMAX+1];   /* (Portion of) current input line */
static char ch;                     /* Current character */
static enum _Symbol symbol;         /* Current symbol */
static int curpos;                  /* Current position in input line */
static int curlen;                  /* Current length of input line */
static FILE *f;                     /* Input stream */

/* InitScan: initialise scanner to read from fname */
static void InitScan(char *fname)
{
   if ((f=fopen(fname,"r")) == NULL)
      HError(3110,"InitScan: Cannot open Network Defn file %s",fname);
   curlen=curpos=1; ch=' '; inlyne[0]=' '; inlyne[1]='\0';
}

/* PGetCh: get next input character -> ch */
static void PGetCh(void)
{
   if (curpos>=curlen) {
      if (fgets(inlyne,SCANBUFMAX,f)==NULL) {
         ch=H_EOF; return;
      }
      curpos=0; curlen=strlen(inlyne);
      if (inlyne[curlen-1] == '\n')
         inlyne[curlen-1]=' ';   /* change newline,if any to space */
   }
   ch = inlyne[curpos++];
}

/* PGetIdent: get identifer -> ident */
static void PGetIdent(void)
{
   int i=0;
   Ident id;
   
   do {
      if (ch==ESCAPE) PGetCh();
      if (i<MAXIDENT) id[i++]=ch;
      PGetCh();
   } while ( !isspace((int) ch) && ch!='{' && ch!='}' && ch!='[' && ch!=']' &&
             ch!='<' && ch!='>' && ch!='(' && ch!=')' && ch!='=' && 
             ch!=';' && ch!='|' && ch!='/' && ch!='%');
   id[i]='\0';
   ident = GetLabId(id,TRUE);
}

/* PGetSym: get next symbol -> symbol */
static void PGetSym(void)
{
   while (isspace((int) ch) || (ch=='/' && inlyne[curpos]=='*') ) {
      if (isspace((int) ch))  /* skip space */
         PGetCh();
      else {            /* skip comment */
         PGetCh(); PGetCh();
         while (!(ch=='*' && inlyne[curpos]=='/')) PGetCh();
         PGetCh(); PGetCh();        
      }
   }
   switch (ch) {
   case '$': PGetCh(); PGetIdent(); symbol=VARSYM; break;
   case '(': PGetCh(); symbol=LPARSYM; break;
   case ')': PGetCh(); symbol=RPARSYM; break;
   case '<': PGetCh(); 
      if (ch=='<') {symbol=LTRISYM; PGetCh();} else
         symbol=LANGSYM; break;
   case '>': PGetCh();  
      if (ch=='>') {symbol=RTRISYM; PGetCh();} else
         symbol=RANGSYM; break;
   case '{': PGetCh(); symbol=LBRACESYM; break;
   case '}': PGetCh(); symbol=RBRACESYM; break;
   case '[': PGetCh(); symbol=LBRAKSYM; break;
   case ']': PGetCh(); symbol=RBRAKSYM; break;
   case '=': PGetCh(); symbol=EQSYM; break;
   case ';': PGetCh(); symbol=SEMISYM; break;
   case '|': PGetCh(); symbol=BARSYM; break;
   case '%': PGetCh(); symbol=PERCENTSYM; break;
   case H_EOF: symbol=EOFSYM; break;
   default:  PGetIdent(); symbol=NAMESYM; break;
   }
}

/* ------------------------- Errors ------------------------ */

/* ParseError: Print a parser error message then die */
static void ParseError(int errn)
{
   int i;
   
   fprintf(stderr,"%s\n",inlyne);
   for (i=1; i<curpos-1; i++) fputc(' ',stderr);
   fprintf(stderr, "^  *** Error %d\n",errn);
   switch(errn) {
   case 1: HError(3150,"ParseError: Unexpected EOF");
   case 2: HError(3150,"ParseError: Garbage at end of file");
   case 3: HError(3150,"ParseError: Factor expected");
   case 4: HError(3150,"ParseError: variable expected");
   case 5: HError(3150,"ParseError: = expected");
   case 6: HError(3150,"ParseError: ; expected");
   case 7: HError(3150,"ParseError: ) expected");
   case 8: HError(3150,"ParseError: ] expected");
   case 9: HError(3150,"ParseError: } expected");
   case 10: HError(3150,"ParseError: > expected");
   case 11: HError(3150,"ParseError: >> expected");
   case 12: HError(3150,"ParseError: External name expected");
   case 13: HError(3150,"ParseError: ( expected");
   default: HError(3150,"ParseError: Unknown error number!!!");
   }
}

/* --------------------------- Parser -------------------------------- */

/*
  Each parser routine has two parameters: hd and tl.  As each
  routine completes successfully, it uses these parameters to
  return the head and tail of a subnetwork representing the
  structure just parsed.
*/

LabId curId;         /* name of current variable def, if any */

static void PExpr(Link *hd, Link *tl);

/* PModel: parse a model name and create a node for it */
static void PModel(Link *hd, Link *tl)
{
   Link p;
   
   p=CreateNode(ident,&curChain,LINKCHUNKSIZE,LINKCHUNKSIZE);
   *hd = *tl = p;
   PGetSym();
   if (symbol==PERCENTSYM){
      PGetSym();
      if (symbol!=NAMESYM && symbol != PERCENTSYM) 
         ParseError(12);
      if (symbol==PERCENTSYM)
         p->extName = NULL;
      else 
         p->extName = ident;
      PGetSym();
   }
}

/* PVariable: parse a variable name and create a node for variable */
static void PVariable(Link *hd, Link *tl)
{
   Link p;
   HPNetwork *proto;
      
   if((proto = (HPNetwork *)ident->aux) == NULL)
      HError(3130,"PVariable: Variable %s is undefined",ident->name);
   p=CreateNode(ident,&curChain,LINKCHUNKSIZE,LINKCHUNKSIZE);
   p->extName = subNetId;
   *hd = *tl = p;
   PGetSym();
}


/* PGroup: parse ( .. ) */
static void PGroup(Link *hd, Link *tl)
{
   PGetSym();
   PExpr(hd,tl);
   if (symbol!=RPARSYM) ParseError(7);
   PGetSym();
}


/* POption: parse [ .. ] */
static void POption(Link *hd, Link *tl)
{
   Link eTail;
   
   PGetSym();
   PExpr(hd, &eTail);
   *tl = CreateNode(NULL,&curChain,LINKCHUNKSIZE,LINKCHUNKSIZE);
   JoinNodes(*hd,*tl);
   JoinNodes(eTail,*tl);
   if (symbol!=RBRAKSYM) ParseError(8);
   PGetSym();
}

/* PRepetition0: parse { .. } */
static void PRepetition0(Link *hd, Link *tl)
{
   Link eHead, eTail;
   
   PGetSym();
   PExpr(&eHead, &eTail);
   *hd = *tl = CreateNode(NULL,&curChain,LINKCHUNKSIZE,LINKCHUNKSIZE);
   JoinNodes(*tl,eHead);
   JoinNodes(eTail,*tl);
   if (symbol!=RBRACESYM) ParseError(9);
   PGetSym();
}

/* PRepetition1: parse < .. > */
static void PRepetition1(Link *hd, Link *tl)
{
   PGetSym();
   PExpr(hd, tl);
   JoinNodes(*tl,*hd);
   if (symbol!=RANGSYM) ParseError(10);
   PGetSym();
}

/* PTriloop: parse << .. >> */
static void PTriloop(Link *hd, Link *tl)
{
   Link curChainSave;
   Link p;

   PGetSym();
   curChainSave = curChain;
   curChain = NULL;
   PExpr(hd, tl);
   ExpandSubNetDefs(&curChain);
   p = curChain;
   while (p->chain != NULL) p=p->chain;
   p->chain=curChainSave;    /* add on the old curChain */
   MakeTriLoop(hd, tl);
   if (symbol!=RTRISYM) ParseError(11);
   PGetSym();
}

/* PFactor: parse a factor */
static void PFactor(Link *hd, Link *tl)
{
   switch(symbol) {
   case NAMESYM:     PModel(hd,tl);       break;
   case VARSYM:      PVariable(hd,tl);    break;
   case LPARSYM:     PGroup(hd,tl);       break;
   case LBRAKSYM:    POption(hd,tl);      break;
   case LBRACESYM:   PRepetition0(hd,tl); break;
   case LANGSYM:     PRepetition1(hd,tl); break;
   case LTRISYM:     PTriloop(hd,tl);     break;
   default: ParseError(3);
   }
}

/* PSequence: parse a sequence of factors */
static void PSequence(Link *hd, Link *tl)
{
   Link hd2, tl2;
   
   PFactor(hd,tl);
   while (symbol==NAMESYM || symbol==VARSYM || symbol==LBRAKSYM || 
          symbol==LBRACESYM || symbol==LPARSYM || 
          symbol==LTRISYM || symbol==LANGSYM) {
      PFactor(&hd2, &tl2);
      JoinNodes(*tl,hd2);
      *tl=tl2;
   }
   if ((*hd)->pred->numLinks != 0) {
      hd2 = *hd;
      *hd = CreateNode(NULL,&curChain,LINKCHUNKSIZE,LINKCHUNKSIZE);
      JoinNodes(*hd,hd2);
   }
   if ((*tl)->succ->numLinks != 0) {  
      tl2 = *tl;
      *tl=CreateNode(NULL,&curChain,LINKCHUNKSIZE,LINKCHUNKSIZE);
      JoinNodes(tl2,*tl);
   }
}

/* PExpr: parse an expression */
static void PExpr(Link *hd, Link *tl)
{
   Link hd2, tl2;
   
   PSequence(hd,tl);
   hd2 = *hd, tl2 = *tl;
   *hd=CreateNode(NULL,&curChain,LINKCHUNKSIZE,LINKCHUNKSIZE);
   *tl=CreateNode(NULL,&curChain,LINKCHUNKSIZE,LINKCHUNKSIZE);
   JoinNodes(*hd,hd2);
   JoinNodes(tl2,*tl);
   while (symbol==BARSYM) {
      PGetSym();
      PSequence(&hd2, &tl2);
      HeadMerge(*hd,hd2);
      TailMerge(tl2,*tl);
   }
}

/* PSubNet: parse a subnet def and store it */
static void PSubNet(void)
{
   Link hd,tl;
   
   if (symbol != VARSYM) ParseError(4);
   curId=ident;
   PGetSym();
   if (symbol != EQSYM) ParseError(5);
   PGetSym();
   curChain=NULL;
   PExpr(&hd,&tl);
   if (symbol != SEMISYM) ParseError(6);
   PGetSym();
   DefineSubNet(curId,hd,tl,curChain);
}

static LabId  enterExitId;   /* for use in RemoveGlue */

/* PNetwork: parse a complete network definition.  If netOnly then there
   must be a main network expression and all subnet definitions are 
   destroyed once the main net has been built. Otherwise, a main net
   expression is optional.  If skipEpr then the main net is skipped
   even if it is there.  */
static void PNetwork(Link *hd, Link *tl, Boolean netOnly, Boolean skipExpr)
{
   Link entryNode,exitNode;
   
   InitSubNetDefs();
   enterExitId = GetLabId("$$$HPARSE_ENTEREXITNODE",TRUE); /* for use in RemoveGlue */
   while (symbol != LPARSYM && symbol != EOFSYM) {
      PSubNet();
   }
   curId=NULL; curChain=NULL;
   *hd = NULL; *tl = NULL;
   if ((!skipExpr && symbol == LPARSYM)  || netOnly){
      if (symbol != LPARSYM) ParseError(13);
      PGetSym();
      entryNode = CreateNode(enterId,&curChain,LINKCHUNKSIZE,1);
      exitNode = CreateNode(exitId,&curChain,1,LINKCHUNKSIZE); 
      PExpr(hd,tl);
      JoinNodes(entryNode,*hd);
      JoinNodes(*tl,exitNode);
      *hd=entryNode; *tl=exitNode;
      if (symbol != RPARSYM) ParseError(7);
      PGetSym();
   }
   if (symbol!=EOFSYM  && !skipExpr) ParseError(2);
   ExpandSubNetDefs(&curChain);
   if (netOnly) FreeSubNetDefs();
}

/* -------------------- Clean Up HParse Network ---------------------- */

/* ReSizeNodes: scan the network and shrink all of its nodes */
static void ReSizeNodes(HPNetwork *net)
{
   Link p;
   
   p=net->chain;
   while (p!=NULL) {
      ShrinkNode(p);
      p=p->chain;
   }
}

/* DeleteLink: delete a link x from a linkSet ls */
static void DeleteLink(Link x, LinkSet *ls)
     /* note: deletes the link whether ls is shared or not */
{
   Boolean found = FALSE;
   int i,j;

   i = 0;
   for (i=1; i<=ls->numLinks; i++) 
      {
         if (ls->links[i] == x) {
            found = TRUE;
            break;
         }
      }
   if (found) {
      --(ls->numLinks);
      for (j=i;j<=ls->numLinks;j++) 
         ls->links[j] = ls->links[j+1];
   }
}

/* MergeLinks: adds all links from *from to *to */
static LinkSet*  MergeLinks(LinkSet *from, LinkSet *to)
     /* note: makes a new link set if *to is not large enough */
     /*       returns the merged link set */
{
   int i,j;
   LinkSet *newls;

   for (i=1; i<= from->numLinks; i++)     /* Remove any duplicates from to */
      DeleteLink(from->links[i], to);       
   if (from->numLinks > (to->maxLinks - to->numLinks))
      {
         newls = CreateLinkSet((int) ((from->numLinks+to->numLinks)*LINKEXTENTFACTOR));
         for (i=1; i<=to->numLinks; i++)
            newls->links[i] = to->links[i];
         newls->numLinks = to->numLinks;
         to = newls;
      }   
   for (i=to->numLinks+1, j=1; j<= from->numLinks; i++, j++)
      to->links[i] = from->links[j];
   to->numLinks += from->numLinks;
   return to;
}

/* CanCompact: Determine if a Gluenode should be compacted */
static Boolean CanCompact(Link p)
     /* compact if p either all succs are non-glue 
           or all preds are non-glue
           or only 1 succ and one pred (both glue)
           or no succs or no preds
*/
{
   int i;
   Boolean ok =TRUE;    
   Link predNode,succNode;

   for (i=1; (i <= p->succ->numLinks) && ok ; i++) {
      succNode = p -> succ->links[i];
      ok = (succNode -> modelName != NULL);
   }
   if (!ok) {
      ok = TRUE;    
      for (i=1; (i <= p->pred->numLinks) && ok ; i++) {
         predNode = p -> pred->links[i];
         ok = (predNode -> modelName != NULL);
      }
   }
   if (!ok)
      ok = ((p->succ->numLinks == 1) && (p->pred->numLinks == 1));
   return ok;
}

/* CompactGlueNode: move all the links from/to node *p to its neighbours */
static void CompactGlueNode(Link p)
{
   int i,j;
   Link predNode, succNode;
   Link predNode2, succNode2;
   LinkSet *oldls;

   /* first process the successors of of the predessor nodes to p */
   DeleteLink(p, p->succ);  /* delete self loop if present */
   for (i=1; i<= p->pred->numLinks; i++) {
      predNode = p->pred->links[i];
      if (predNode == p) continue;
      if (predNode->succ->numLinks == 1) { /* only pointed to this node */
         FreeLinkSet(predNode ->succ);
         predNode->succ = p->succ;
         predNode->succ->nUse++;
      } else {
         if (predNode->succ->nUse > 1) { /* the succ is shared */
                                         /* and hasn't been processed yet */
            oldls =  predNode->succ;
            DeleteLink(p, predNode->succ);
            predNode->succ  = MergeLinks(p->succ,predNode->succ);
            predNode->succ->nUse = -(oldls->nUse);   /* mark as processed */
            for (j=1; j<= p->pred->numLinks; j++) {  /* point other shared ls*/
               predNode2 = p ->pred->links[j];       /* to new linkset */
               if (predNode2->succ == oldls)
                  predNode2->succ = predNode->succ;
            }
            if (oldls != predNode->succ)    /* did the linkset change? */
               FreeLinkSet(oldls);
         }
         else if (predNode->succ->nUse == 1) {    /* non-shared link sets */
            DeleteLink(p, predNode->succ);
            predNode->succ  = MergeLinks(p->succ,predNode->succ);
         }
      }
   }
   for (i=1; i<= p->pred->numLinks; i++) {   /* clean up nUse markers */
      predNode = p -> pred->links[i];
      if (predNode->succ->nUse < 0)
         predNode->succ->nUse = -(predNode->succ->nUse);
   }

   /* now do the predecessors of the successor nodes to p */
   DeleteLink(p, p->pred);  /* delete self loop if present */
   for (i=1; i<= p->succ->numLinks; i++) {
      succNode = p -> succ->links[i];
      if (succNode ->pred ->numLinks == 1) { /* only pointed to this node */
         FreeLinkSet(succNode ->pred);
         succNode->pred = p->pred;
         succNode->pred->nUse++;
      }
      else {
         if (succNode->pred->nUse > 1) { /* the pred is shared */
                                         /* and hasn't been processed yet */
            oldls =  succNode->pred;
            DeleteLink(p, succNode->pred);
            succNode->pred  = MergeLinks(p->pred,succNode->pred);
            succNode->pred->nUse = -(oldls->nUse);   /* mark as processed */
            for (j=1; j<= p->succ->numLinks; j++) {  /* point other shared ls*/
               succNode2 = p -> succ->links[j];      /* to new linkset */
               if (succNode2->pred == oldls)
                  succNode2->pred = succNode->pred;
            }
            if (oldls != succNode->pred)    /* did the linkset change? */
               FreeLinkSet(oldls);
         }
         else if (succNode->pred->nUse == 1) {    /* non-shared link sets */
            DeleteLink(p, succNode->pred);
            succNode->pred  = MergeLinks(p->pred,succNode->pred);
         }
      }
   }
   for (i=1; i<= p->succ->numLinks; i++) {   /* clean up nUse markers */
      succNode = p -> succ->links[i];
      if (succNode->pred->nUse < 0)
         succNode->pred->nUse = -(succNode->pred->nUse);
   }
}

/* RemoveGlue: removes all the glue nodes in the network */
static void  RemoveGlue(HPNetwork *network)
     /* note: takes care to retain LinkSet sharing (where possible) */
{
   Link p,q;
   int numGlueLeft=0;
   Boolean removedp;
   Boolean changed=FALSE;
   Boolean removeAll=FALSE;
   int iter = 0;
   
   if (network->entryNode->modelName==NULL) network->entryNode->modelName=enterExitId;
   if (network->exitNode->modelName==NULL) network->exitNode->modelName=enterExitId;
   do {   /* compact nodes until no glue left  */
      if ((numGlueLeft > 0) && (!changed))
         removeAll = TRUE;
      numGlueLeft = 0; 
      changed = FALSE;
      if (trace & T_HPREMGLUE) {
         printf("RemoveGlue Iteration: %d\n",iter);
         PrintHParseNetwork(network);
      }
      p=network->chain; network->chain=NULL; 
      while (p!=NULL) {         /* while there are nodes left to process */
         removedp = FALSE;
         if (p->modelName==NULL) {        /* then its a glue node */
            if ( CanCompact(p) || removeAll ) {
               CompactGlueNode(p);
               q=p; p=p -> chain; FreeNode(q);
               removedp = TRUE; changed = TRUE;
            } 
            else
               numGlueLeft++; 
         } 
         if (!removedp) {     
            /* not changed on this pass so add to head of new chain */
            q = p->chain;
            p->chain = network->chain;
            network->chain = p;
            p = q;
         }
      }
      iter++;
   }
   while (numGlueLeft != 0); 
   if (network->entryNode->modelName==enterExitId) network->entryNode->modelName=NULL;
   if (network->exitNode->modelName==enterExitId) network->exitNode->modelName=NULL;
}

/* DisconNode: Is a node disconnected from the network */
static Boolean DisconNode(Link p)
{
   if (p->succ == NULL || p->pred == NULL)
      return TRUE;
   if (p->succ->numLinks == 0 || p->pred->numLinks == 0)
      return TRUE;
   return FALSE;
}

/* RemoveDiscon: remove any nodes (glue or ortherwise) that are */
/*               not properly connected to the network          */
static void RemoveDiscon(HPNetwork *net)
{
   Link p,q;
   Link succNode, predNode;
   Boolean removedp;
   Boolean changed;
   int i;
       
   do {   /* until no changes  */
      changed = FALSE;
      p=net->chain; net->chain=NULL; 
      while (p!=NULL) {        /* while nodes left to process */
         removedp = FALSE;
         if (p != net->entryNode && p != net->exitNode)
            if (DisconNode(p)) { 
               if (p->succ != NULL)
                  for (i=1; i<=p->succ->numLinks; i++) {
                     succNode = p->succ->links[i];
                     DeleteLink(p, succNode->pred);
                  }
               if (p->pred != NULL)
                  for (i=1; i<=p->pred->numLinks; i++) {
                     predNode = p->pred->links[i];
                     DeleteLink(p, predNode->succ);
                  }
               q=p; p=p->chain; FreeNode(q);
               removedp = TRUE; changed = TRUE;
            } 
         if (!removedp) {    /* not changed so add to new chain */   
            q = p->chain;
            p->chain = net->chain;
            net->chain = p;
            p = q;
         }
      }
   }
   while (changed);
}

/* -------------------- Main HParse Call ----------------------------- */

/* CreateHParseNetwork: parse and build a network */
static HPNetwork CreateHParseNetwork(char *fname)
{
   Link hd,tl;
   HPNetwork theNet;
   
   /* Create the memory heaps */
   CreateHeap(&nodeHeap,"HParse Node Heap",MHEAP,sizeof(Node), 
              0.0, NODEBLOCK, NODEBLOCK);
   CreateHeap(&lsHeap,"HParse LinkSet Heap",MHEAP,sizeof(LinkSet), 
              0.0, LSBLOCK,  LSBLOCK);
   CreateHeap(&lsChunkHeap,"HParse Link Chunk Heap",MHEAP, 
              LINKCHUNKSIZE*sizeof(Link),0.0, LSBLOCK,  LSBLOCK);
   CreateHeap(&lsLargeHeap,"HParse Large Links Heap",CHEAP,1,0.0,0,0);
   InitScan(fname);
   PGetSym(); 
   PNetwork(&hd,&tl,TRUE,FALSE);
   fclose(f);
   theNet.entryNode = hd;
   theNet.exitNode = tl;
   theNet.chain = curChain;
   RemoveDiscon(&theNet);
   RemoveGlue(&theNet); 
   ReSizeNodes(&theNet); 
   FreeSubNetDefs();
   if (trace & T_HPMEMSTAT) {
      printf("Memory statistics after generating HParse network:\n");
      PrintAllHeapStats();
   }
   return theNet;
}

/* ----------------- End of HParse Network building  -------------------- */



/* ------------   HParse Network -> Lattice Conversion ------------------- */

static MemHeap nodeInfoHeap;

/* AttachNodeInfos: to each node in theNet */
void AttachNodeInfos(HPNetwork *theNet)
{
   Link p;
   NodeInfo *ni;
   
   p = theNet->chain;
   CreateHeap(&nodeInfoHeap,"HParse Node Info Heap",MHEAP,sizeof(NodeInfo),1.5,500,5000);
   while (p!=NULL) {
      ni = (NodeInfo *) New(&nodeInfoHeap,sizeof(NodeInfo));
      p->user = (Ptr)ni; 
      if ((p->modelName == wdBeginId) && v1Compat)
         ni->nType = wdBegin;
      else if ((p->modelName == wdEndId) && v1Compat)
         ni->nType = wdEnd;
      else if (p->modelName == enterId || p->modelName == exitId)
         ni->nType = nullNode;
      else 
         ni->nType = unknown;
      ni->seen = FALSE;
      ni->history = NULL;
      if (p->succ->nUse > 1) {  /* we will put a NULL node here ... */
         ni = (NodeInfo *) New(&nodeInfoHeap,sizeof(NodeInfo));
         p->succ->user = (Ptr) ni;
         ni->nType  = nullNode;
         ni->seen = FALSE;
         ni->history = NULL;
      }
      p = p->chain;
   }
}


/* LabelInternal: mark wdInternal nodes until reach a non-internal node */
void LabelInternal(Link p)
{
   NodeInfo *ni;
   int i;
   
   ni = (NodeInfo *) p->user;
   if (ni->nType == unknown) {
      ni->nType = wdInternal;
      if (!ni->seen) {
         ni->seen = TRUE;
         for (i=1; i <= p->succ->numLinks; i++)
            LabelInternal(p->succ->links[i]);
      }  
   }
   else if ((ni->nType != wdEnd) && (ni->nType != wdInternal))
      HError(3131,"LabelInternal: incorrect WD_BEGIN/WD_END node connection, node %d is %d",((int)p % 4000) / 4,ni->nType);
}

/* FindNodeTypes: mark each node as wdInternal or wdExternal */
void FindNodeTypes(HPNetwork *theNet)
{
   Link p;
   NodeInfo *ni;
   int i;
   
   if (!v1Compat) {   /* label all nodes as external */
      for (p=theNet->chain; p !=NULL; p=p->chain) {
         ni = (NodeInfo *) p->user;
         if (ni->nType == unknown) ni->nType = wdExternal;
      }
      return;
   }
   for (p=theNet->chain; p != NULL; p=p->chain) {
      if (p->modelName == wdBeginId)
         numWdBegin++;
      if (p->modelName == wdEndId)
         numWdEnd++;
   }
   if (numWdBegin != numWdEnd) 
      HError(3131,"FindNodeTypes: Different num WD_BEGIN (%d) & WD_END nodes (%d)",
             numWdBegin, numWdEnd);
   if (numWdBegin > 0) {
      if (trace > 0) {
         printf("  HParse Net contains %d WD_BEGIN/WD_END pairs\n", numWdBegin);
         fflush(stdout);
      }
      /* label the internal nodes as such */
      for (p=theNet->chain; p !=NULL; p=p->chain)
         if (p->modelName == wdBeginId)
            for (i=1; i <=p->succ->numLinks; i++) 
               LabelInternal(p->succ->links[i]);
      /* reset the seen flags  */
      for (p=theNet->chain; p !=NULL; p=p->chain) {
         ni = (NodeInfo *) p->user;
         ni->seen = FALSE;
      }
      /* check all nodes connected to wdBegin are internal */
      for (p=theNet->chain; p !=NULL; p=p->chain) {
         if (p->modelName == wdBeginId)
            for (i=1; i <=p->succ->numLinks; i++) {
               ni = (NodeInfo *) p->succ->links[i]->user;
               if (ni == NULL || ni->nType != wdInternal)
                  HError(3131,"FindNodeTypes: incorrect WD_BEGIN node connection");
            }
      }
   }        
   /* label all other nodes as external */
   for (p=theNet->chain; p !=NULL; p=p->chain) {
      ni = (NodeInfo *) p->user;
      if (ni->nType == unknown)
         ni->nType = wdExternal;
   }
}

/* AddWordExtern: Add a wdExternal node to the dictionary */
void AddWordExtern(Vocab *voc, Link p)
{
   Word wd;
   LabId outSym;

   wd = GetWord(voc,p->modelName,TRUE);
   outSym = p->extName;
   if (outSym == NULL) outSym = GetLabId("",TRUE);
   NewPron(voc,wd,1,&(p->modelName),outSym,1.0);
}

static LabId phonebuf[MAXPHONES];   /* space to store the current pronunciation */

/* AddWordModel: add a pronunciation to dictionary voc */
void AddWordModel(Vocab *voc, Link p, Link history)
{
   Link h;
   NodeInfo *ni;
   Word wd;
   int nphon = 0;

   wd = GetWord(voc,p->extName,TRUE);
   h = history;
   while (h != NULL) {
      if (nphon < MAXPHONES)
         phonebuf[nphon++] = h->modelName;
      else
         HError(3132,"AddWordModel: Dictionary entry for word %s exceeded max num [%d] phones", 
                p->extName->name,MAXPHONES);
      ni = (NodeInfo *) h->user;
      h = ni->history;
   }
   NewPron(voc,wd,nphon,phonebuf,p->extName,1.0);
}

/* ExpandWordModel: Expand word nodes into linear strings  */
/*                  Keeps a history in the recursive calls */
/*                  for each string. Processes backwards   */
/*                  from wdEnd node -> wdBegin             */
void ExpandWordModel(Vocab *voc, Link p, Link history)
{
   NodeInfo *ni;
   int i;

   ni = (NodeInfo *) p->user;
   if (ni->nType == wdBegin)
      AddWordModel(voc,p,history);
   else {
      ni->history = history;
      for (i = 1; i <= p->pred->numLinks; i++) 
         ExpandWordModel(voc,p->pred->links[i],p);
   }
}

/* GenerateDict: Go thro theNet finding all Word models */
void GenerateDict(HPNetwork *theNet,Vocab *voc) 
{
   Link p;
   NodeInfo *ni;
   int i;
   
   for (p=theNet->chain; p != NULL; p=p->chain) {
      ni = (NodeInfo *) p->user;
      if (ni->nType == wdEnd) {
         if (p->extName->aux != (Ptr) 0)
            HError(-3132,"GenerateDict: Word %s already in dictionary: ignored",
                   p->extName->name);
         else {
            p->extName->aux = (Ptr) 1;
            for (i = 1; i <= p->pred->numLinks; i++) 
               ExpandWordModel(voc,p->pred->links[i],NULL);
         }
      }
   }
   for (p=theNet->chain; p != NULL; p=p->chain) {
      ni = (NodeInfo *) p->user;
      if (ni->nType == wdExternal) {
         if (p->modelName->aux != (Ptr) 0)
            HError(-3132,"GenerateDict: Word %s already in dictionary: ignored",
                   p->modelName->name);
         else 
            if (p->modelName->aux == (Ptr) 0) {
               p->modelName->aux = (Ptr) 2;
               AddWordExtern(voc,p);
            }
      }
   }
}

/* GetWdBeginNum: Find node num of wdBegin from corresponding wdEnd */
int GetWdBeginNum(Link p)
{
   NodeInfo *ni;
   
   ni = (NodeInfo *) p->user;
   if (ni->nType == wdEnd)
      return ni->nodeNum;
   else if (1 <= p->succ->numLinks )
      return GetWdBeginNum(p->succ->links[1]);
   else
      abort();  /* #### ge: sort this out -- the 2.2_ref code was even worse!!!  */
}

/* GenerateLattice: generate lattice */
static Lattice* GenerateLattice(HPNetwork *theNet, Vocab *voc)
{
   Link p;
   NodeInfo *ni,*ni2;
   int nNode,nLink,i,j;
   int fromNode,toNode;
   Lattice *lat;
   Word wd;
   LNode *ln;
   LArc *la;

   /* first count the number of nodes and links and record the node numbers */
   nNode=0; nLink=0;
   for (p=theNet->chain; p != NULL; p=p->chain) {
      ni = (NodeInfo *) p->user;
      if (ni->nType == wdExternal || ni->nType == wdEnd || ni->nType == nullNode) {
         ni->nodeNum = nNode++; 
         if (p->succ != NULL) {
            ni2 = (NodeInfo *) p->succ->user;
            if (ni2 != NULL) {
               nLink++;  /* link from node to null node */
               if (ni2->seen == FALSE) {
                  nLink += p->succ->numLinks;
                  ni2->nodeNum = nNode++; 
                  ni2->seen = TRUE;
               }
            }
            else 
               nLink += p->succ->numLinks;

         }
      }
   }
   /* reset seen flags on shared linksets */
   for (p=theNet->chain; p != NULL; p=p->chain) {
      ni = (NodeInfo *) p->user;
      if (ni->nType == wdExternal || ni->nType == wdEnd) {
         ni2 = (NodeInfo *) p->succ->user;
         if (ni2 != NULL && ni2->seen == TRUE) 
            ni2->seen = FALSE;
      }
   }

   lat = NewLattice(&gstack,nNode,nLink);
   lat->voc = voc;
   lat->lmscale = 1.0; lat->wdpenalty = 0.0;
   lat->prscale = 1.0;
   if (trace > 0)
      printf("Generating Lattice with %d nodes and %d links\n",nNode,nLink);
   /* add the nodes */   
   for (p=theNet->chain; p != NULL; p=p->chain) {
      ni = (NodeInfo *) p->user;
      if (ni->nType == wdExternal || ni->nType == wdEnd || ni->nType == nullNode ) {
         if (ni->nType == wdEnd) 
            wd = GetWord(voc, p->extName,TRUE);
         else if (ni->nType == wdExternal)
            wd = GetWord(voc, p->modelName,TRUE);
         else 
            wd = voc->nullWord;
         ln = &lat->lnodes[ni->nodeNum];
         ln->word=wd; ln->n=0; ln->v=0;
         ni2 = (NodeInfo *) p->succ->user;
         if (ni2 != NULL && ni2->seen == FALSE) {
            ln = &lat->lnodes[ni2->nodeNum];
            ln->word=voc->nullWord; ln->n=0; ln->v=0;
            ni2->seen = TRUE;
         }
      }
   }
   for (p=theNet->chain; p != NULL; p=p->chain) {
      ni = (NodeInfo *) p->user;
      if (ni->nType == wdExternal || ni->nType == wdEnd) {
         ni2 = (NodeInfo *) p->succ->user;
         if (ni2 != NULL && ni2->seen == TRUE) 
            ni2->seen = FALSE;
      }
   }

   /* get the node nums of wdBegin nodes */
   for (p=theNet->chain; p != NULL; p=p->chain) {
      ni = (NodeInfo *) p->user;
      if (ni->nType == wdBegin)
         ni->nodeNum = GetWdBeginNum(p);
   }

   /* make the links */   
   j = 0;
   for (p=theNet->chain; p != NULL; p=p->chain) {
      ni = (NodeInfo *) p->user;
      if (ni->nType == wdExternal || ni->nType == wdEnd || (ni->nType == nullNode && p->succ != NULL)) {
         ni2 = (NodeInfo *) p->succ->user;
         if (ni2 != NULL) {   /* join up null node */
            fromNode = ni->nodeNum; toNode = ni2->nodeNum;
            la = &lat->larcs[j++];
            la->start=&lat->lnodes[fromNode]; la->end=&lat->lnodes[toNode]; 
            la->lmlike = 0.0;
            if (ni2->seen == FALSE) {
               for (i=1; i <= p->succ->numLinks; i++) {
                  fromNode = ni2->nodeNum; 
                  toNode = ((NodeInfo *)p->succ->links[i]->user)->nodeNum;
                  la = &lat->larcs[j++];
                  la->start=&lat->lnodes[fromNode]; la->end=&lat->lnodes[toNode]; 
                  la->lmlike =  -log((double) p->succ->numLinks);
               }
               ni2->seen = TRUE;
            }                  
         }
         else 
            for (i=1; i <= p->succ->numLinks; i++) {
               fromNode = ni->nodeNum; 
               toNode = ((NodeInfo *)p->succ->links[i]->user)->nodeNum;
               la = &lat->larcs[j++];
               la->start=&lat->lnodes[fromNode]; la->end=&lat->lnodes[toNode]; 
               la->lmlike =  -log((double) p->succ->numLinks);
            }
      }
   }
   return lat;
}

static void SaveLattice(Lattice *lat, char *latFn, LatFormat format)
{
   FILE *latf;
   Boolean isPipe;

   if ( (latf = FOpen(latFn,NetOFilter,&isPipe)) == NULL)
      HError(3111,"SaveLattice : Cannot create new lattice file  %s",latFn);
   if(WriteLattice(lat,latf,format)<SUCCESS)
      HError(3111,"SaveLattice : WriteLattice failed");
   FClose(latf,isPipe);
}

/* ConvertHParseNetwork: Convert theNet & print to latFn and possibly dictFn */
static void ConvertHParseNetwork(HPNetwork *theNet, char *latFn, char *dictFn)
{
   Vocab voc;
   Lattice *lat;
   LatFormat format;

   AttachNodeInfos(theNet);
   FindNodeTypes(theNet);
   InitVocab(&voc);
   if (dictFn != NULL) {
      if (numWdBegin > 0) {
         GenerateDict(theNet, &voc);
         if (trace > 0)
            printf("Writing Dictionary to %s\n",dictFn);      
         if(WriteDict(dictFn, &voc)<SUCCESS)
            HError(3114,"SaveLattice : WriteDict failed");
      }
      else
         HError(-3132,"ConvertHParseNetwork: Dict. would be empty: not written");
   }
   lat = GenerateLattice(theNet,&voc);
   if (trace > 0)
      printf("Writing Word Lattice to %s\n",latFn);      
   format = 0;
   if (saveLatLM) format |= HLAT_LMLIKE;
   if (saveLatBin) format |= HLAT_LBIN;   
   SaveLattice(lat,latFn,format);
}

/* ------------------- End of HParse.c --------------------------------- */
