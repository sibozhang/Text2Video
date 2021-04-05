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
/* main authors: Valtcho Valtchev, Steve Young,                */
/*               Julian Odell, Gareth Moore                    */
/* ----------------------------------------------------------- */
/*         Copyright:                                          */
/*                                                             */
/*          1994-2002 Cambridge University                     */
/*                    Engineering Department                   */
/*                                                             */
/*   Use of this software is governed by a License Agreement   */
/*    ** See the file License for the Conditions of Use  **    */
/*    **     This banner notice must not be removed      **    */
/*                                                             */
/* ----------------------------------------------------------- */
/*         File: LGPrep - prepare a sorted GramBase           */
/* ----------------------------------------------------------- */

char *lgprep_version = "!HVER!LGPrep:   3.4.1 [CUED 12/03/09]";
char *lgprep_vc_id = "$Id: LGPrep.c,v 1.1.1.1 2006/10/11 09:54:44 jal58 Exp $";

#include "HShell.h"
#include "HMem.h"
#include "HMath.h"
#include "HWave.h"
#include "HLabel.h"
#include "LUtil.h"
#include "LWMap.h"
#include "LGBase.h"
#include "LModel.h"

/* 
   This tool processes source texts and saves sorted n-gram counts.
   Text passes thru a window word by word and each n-gram is recorded.
   The text in the window can also be modified by match and replace rules 
   and in this case, the n-grams in the original matched text are stored 
   in a set of 'negative' gram files and the n-grams in the modified text 
   are stored in a set of 'positive' gram files.
*/
   
/* -------------------------- Trace Flags ------------------------ */

static int trace = 0;
#define T_TOP  0001     /* Top Level tracing */
#define T_SAV  0002     /* Monitor Buffer Saving */
#define T_INP  0004     /* Trace word input stream */
#define T_SHR  0010     /* Trace shift register input */
#define T_RIN  0020     /* Rule input monitoring */
#define T_RUL  0040     /* Print rule set */
#define T_MEM  0100     /* Print heap stats */

/* ---------------- Configuration Parameters --------------------- */

static ConfParam *cParm[MAXGLOBS];
static int nParm = 0;            /* total num params */

/* ------------------- Edit Rule Definitions --------------------- */

#define MAX_FIELDS   256      /* max number of fields in a rule */
#define MAX_ITEMS    256      /* max number of items in any one set */
#define MAX_SETS     256      /* max number sets */

typedef enum {             /* tags for rule fields */
   f_WORD,                    /* literal word */
   f_WILD,                    /* wildcard */
   f_WSET,                    /* in word set */
   f_NWSET,                   /* not in word set */
   f_FIELD,                   /* slot contents */
   f_NONE
} FieldOp;

typedef union {            /* contents of a field */
   int   flid;                /* field index */
   int   setid;               /* set index */
   LabId wdid;                /* word literal */
} FieldItem;

typedef struct {           /* set definition */
   int nItem;                 /* num words in set */
   LabId item[MAX_ITEMS];     /* list of words */
} SetDef;

typedef struct {           /* field list (match or replace part) */
   int n;                     /* number of fields in list */
   FieldOp   fop[MAX_FIELDS]; /* tag/operation for each field */
   FieldItem fdt[MAX_FIELDS]; /* actual data in each field */
} FieldVec;

typedef struct ruledef{    /* rule definition */
   float pact;                /* % applic factor */
   float psum;                /* accumulator */
   FieldVec src;              /* match part of rule */
   FieldVec tgt;              /* replacement part of rule */
   struct ruledef *next;
} RuleDef;

typedef struct {           /* rule set */
   MemHeap mem;               /* Memory for this ruleset */
   int nRules;                /* number of rules */
   RuleDef * head;            /* head of list of rules */
   RuleDef * tail;            /* tail of list of rules */
   int nSets;                 /* number of word sets actually defined */
   SetDef **setlist;          /* array[0..MAX_SETS-1] of -> SetDef */
} RuleSet;

/* ------------------- Word Shift Registers ----------------------- */

typedef struct {
   int used;                  /* actual words in register */
   UInt ng[MAXNG+1];          /* ng[0] is oldest word */
   NGBuffer *ngb;             /* output ngram buffer */
} ShiftReg;

/* ---------------------- Global Variables ----------------------- */

static int nSize     = 3;           /* ngram size */
static int ngbSize   = 2000000;     /* ngram buffer size */
static int egbSize   =  100000;     /* edited ngram buffer size */
static int newWords  =  100000;     /* max new words to accommodate */
static char *rootFN  = "gram";      /* gbase root filename */
static int  dumpOfs  = 0;           /* initial numeric ext of gbase files */
static char *dbsDir  = NULL;        /* directory to store gbase files */
static char *ruleFN  = NULL;        /* file containing edit rules */
static char *omapFN  = NULL;        /* output word map filename */
static char *imapFN  = NULL;        /* input word map filename */
static char *txtsrc  = NULL;        /* gram file text source descriptor */
static Boolean gbGen = TRUE;        /* flag to enable GBase generation */
static Boolean forceCnts = FALSE;   /* force the output of word counts */
static Boolean htkEscape = TRUE;    /* default escaping */
static Boolean tagSentStart = FALSE;/* tag senetence start words with _ */
static WordMap wmap;                /* word map for this corpus */
static Boolean mapUpdated;          /* used optimise sort/saving */
static RuleSet rset;                /* rule set if any */
static ShiftReg stdBuf;             /* used for normal N-gram processing */
static ShiftReg posBuf;             /* N-grams from edited text */
static ShiftReg negBuf;             /* N-grams from matched source text */
static MemHeap ngbHeap;             /* memory for NGBuffers */

static int editWinSize = 0;         /* size of edit window */
static int editUsed;                /* number of words in editPipe */
static LabId sstId = NULL;          /* sentence start id */
static LabId editBuf[MAX_FIELDS];   /* edit buffer for input text */
static int wordnum = 0;

/* ---------------- Prototype functions -------------------------- */

void Initialise(void);
void ProcessText(char *fn, Boolean lastFile);

/* ---------------- Process Command Line ------------------------- */

/* SetConfParms: set configuration parameters relevant to this tool */
void SetConfParms(void)
{
   int i;
   static char b[100];

   sstId = GetLabId(DEF_STARTWORD,TRUE);

   nParm = GetConfig("LGPREP", TRUE, cParm, MAXGLOBS);
   if (nParm>0) {
      if (GetConfInt(cParm,nParm,"TRACE",&i)) trace = i;
      if (GetConfStr(cParm,nParm,"STARTWORD",b)) sstId = GetLabId(b, TRUE);
   }
}


/* ReportUsage: Tool help */
void ReportUsage(void)
{
   printf("\nUSAGE: LGPrep [options] wmap [txtfile] ....\n\n");
   printf(" Option                                       Default\n\n");
   printf(" -a n    allow n new words in input text      %d\n", newWords);
   printf(" -b n    set gram buffer size                 %d\n", ngbSize);
   printf(" -c      force counts into wordmap on update  same as input\n");
   printf(" -d s    output gram file directory           current directory\n");
   printf(" -e n    edited gram buffer size              %d\n", egbSize);
   printf(" -f s    fix text source using rules in s     off\n");
   printf(" -h      disable HTK escaping on output       %s\n", htkEscape?"off":"on");
   printf(" -i n    set output gram file start index     %d\n", dumpOfs);
   printf(" -n n    set n-gram size                      %d\n", nSize);
   printf(" -q      tag sentence start words with '_'    %s\n", tagSentStart?"on":"off");
   printf(" -r s    set root gram filename               %s\n", rootFN);
   printf(" -s s    store s in gram header source fields none\n");
   printf(" -w s    write output map to s                wmap [or -d dir/wmap]\n");
   printf(" -z      suppress gram file generation        %s\n", gbGen?"off":"on");
   printf(" -Q      print rule summary help              off\n");
   PrintStdOpts("");
   printf("\n\n");
}

void RuleSummary(void)
{
   printf("\nEdit Rule Syntax:\n");
   printf("    <set-def>     = '#'<number> <word1> <word2> ... <wordN>.\n");
   printf("    <rule-def>    = <app-factor> <match-def> : <repl-def>\n");
   printf("    <match-def>   = { <word> | '*' | !<set> | %%<set> }\n");
   printf("    <repl-def>    = { '$'<field> | string }\n");
   printf("where\n");
   printf("      <app-factor>   = float defining %% of matches to change\n");
   printf("      <string>       => exact match of specified word\n");
   printf("      *              => matches anything\n");
   printf("      !<set>         => not in set\n");
   printf("      %%<set>         => in set \n");
   printf("      $<field>       => the value in field <number>\n");
   printf("sets and fields are identified by a zero-based index.\n");
   printf("Each set or rule def must be on a single line. \n\n");
   Exit(0);
}

int main(int argc, char *argv[])
{
   char *s,*fn;

   InitShell(argc,argv,lgprep_version,lgprep_vc_id);
   InitMem();
   InitMath();
   InitWave();
   InitLabel();
   InitLUtil();
   InitWMap();
   InitGBase();

   if (!InfoPrinted() && NumArgs() == 0)
      ReportUsage();
   if (NumArgs() == 0) Exit(EXIT_SUCCESS);

   SetConfParms();

   while (NextArg() == SWITCHARG) {
      s = GetSwtArg();
      if (strlen(s)!=1) 
         HError(16019,"Bad switch '%s' - must be single letter",s);
      switch(s[0]){
         case 'a':
            newWords = GetChkedInt(10, 10000000, s); break;
         case 'b':
            ngbSize = GetChkedInt(10, 100000000, s); break;
         case 'c':
            forceCnts = TRUE; break;
         case 'd':
            if (NextArg() != STRINGARG)
               HError(16019,"Output gram file directory expected with -d");
            dbsDir = GetStrArg(); break;
         case 'e':
            egbSize = GetChkedInt(10, 100000000, s); break;
         case 'f':
            if (NextArg()!=STRINGARG)
               HError(16019,"Rule filename expected with -f");
            ruleFN = GetStrArg(); break;
         case 'h':
            htkEscape = FALSE; break;
         case 'i':
            dumpOfs = GetChkedInt(0, 100000, s); break;
         case 'n':
            nSize = GetChkedInt(1, MAXNG, s); break;
         case 'q':
	    tagSentStart=TRUE; break;
         case 'r':
            if (NextArg()!=STRINGARG)
               HError(16019,"Gram base root filename expected with -r");
            rootFN = GetStrArg(); break;
         case 's':
            if (NextArg()!=STRINGARG)
               HError(16019,"Gram file text source descriptor expected with -s");
            txtsrc = GetStrArg(); break;
         case 'w':
            if (NextArg()!=STRINGARG)
               HError(16019,"Output word map filename expected with -w");
            omapFN = GetStrArg(); break;
         case 'z':
            gbGen = FALSE; break;
         case 'Q':
            RuleSummary(); break;
         case 'T':
            trace = GetChkedInt(0,077,s); break;
         default:
            HError(16019,"LGPrep: Unknown switch '%s'",s);
      }
   }
   if (NextArg() != STRINGARG)
      HError(16019,"LGPrep: word map filename expected");
   imapFN = GetStrArg();

   if (omapFN == NULL) {
     char path[256];
     MakeFN("wmap",dbsDir,NULL,path);
     omapFN = CopyString(&gstack,path);
   }

   Initialise();
   if (NextArg() != STRINGARG)
      ProcessText(NULL,TRUE);       /* input from stdin */
   else
      while (NextArg() == STRINGARG) {
	/* Copy the string argument since it gets overwritten 
	   by NextArg() when reading from script file */
	fn = CopyString(&gstack,GetStrArg());
	ProcessText(fn,NextArg() != STRINGARG);
      }
   if (NumArgs() != 0)
      HError(-16019,"LGPrep: unused arguments left on command line");
   if (trace&T_TOP) {
      printf(" %d words processed\n",wordnum);
   }

   Exit(EXIT_SUCCESS);
   return EXIT_SUCCESS; /* never reached -- make compiler happy */
}

/* ------------------------ Text Processing Routines -------------- */

/* SkipToWord: scan string till first word or end of string */
char *SkipToWord(char *s)
{
   while (isspace((int) *s) && (*s != '\0')) ++s;
   return s;
}

/* NextWord: extract next word from given string, returning ptr to next */
char * NextWord(char *s, char *word)
{
   while (isspace((int) *s) && (*s != '\0')) ++s;
   if (*s == '\0') return NULL;
   while (!isspace((int) *s) && (*s != '\0')) *word++ = *s++;
   *word = '\0';
   return s;
}


/* ----------------- Pattern Match/Replace Data Routines -------------- */

/* InSet: return TRUE if wdid is in wset */
Boolean InSet(SetDef *wset, LabId wdid)
{
   int i;

   for (i=0; i<wset->nItem; i++)
      if (wdid == wset->item[i]) return TRUE;
   return FALSE;
}

/* CreateRuleSet: initialise a rule set */
void CreateRuleSet(RuleSet *rset)
{
   int i;
   
   CreateHeap(&(rset->mem),"ruleHeap",MSTAK,1,0.5,1000,10000);
   rset->nRules = 0;
   rset->head = rset->tail = NULL;
   rset->nSets = 0;
   rset->setlist = (SetDef **)New(&(rset->mem),sizeof(SetDef *)*MAX_SETS);
   for (i=0; i<MAX_SETS; i++) rset->setlist[i] = NULL;
}

/* ReadSetDef: read set definition from s and add it to rule set */
void ReadSetDef(char *s, RuleSet *rset)
{
   SetDef *x;
   char buf[256];
   int n;
   
   s = NextWord(s,buf);
   n = atoi(buf);
   if (s==NULL)
      HError(16020,"ReadSetDef: no item in def for set %d",n);
   if (trace&T_RIN) printf("  reading set %d: ",n);
   if (n<0 || n>=MAX_SETS)
      HError(16020,"ReadSetDef: set index %d out of range 0..%d",n,MAX_SETS);
   if (rset->setlist[n] != NULL)
      HError(16020,"ReadSetDef: set index %d already defined",n);
   rset->setlist[n] = x = (SetDef *)New(&(rset->mem),sizeof(SetDef));
   x->nItem = 0;
   s = NextWord(s,buf);
   while (s != NULL) {
      x->item[x->nItem++] = GetLabId(buf,TRUE);
      s = NextWord(s,buf);
   }
   rset->nSets++;
   if (trace&T_RIN) {
      printf("  %d elements read\n",x->nItem);
      fflush(stdout);
   }
}

/* ReadRuleDef: read rule definition from s and it add to rule set */
void ReadRuleDef(char *s, RuleSet *rset)
{
   RuleDef *x;
   char buf[256];
   float f;
   Boolean inPat = TRUE;
   
   x = (RuleDef *)New(&(rset->mem),sizeof(RuleDef));
   x->next = NULL;
   if (rset->nRules++ == 0)
      rset->head = x;
   else
      rset->tail->next = x;
   rset->tail = x;
   s = NextWord(s,buf);
   f = atof(buf);
   if (f<0.0 || f>1.0)
      HError(16020,"ReadRuleDef: appl. factor %f out of range 0..1",f);
   x->pact = f; x->psum = 0.0;
   x->src.n = 0;
   s = NextWord(s,buf);
   while (s != NULL && inPat) {
      if (x->src.n >= MAX_FIELDS)
         HError(16020,"ReadRuleDef: too many fields in pattern");
      switch(buf[0]) {
         case ':':
            inPat = FALSE;
            break;
         case '*':
            x->src.fop[x->src.n++] = f_WILD;
            break;
         case '%':
            x->src.fop[x->src.n] = f_WSET;
            x->src.fdt[x->src.n++].setid = atoi(buf+1);
            break;
         case '!':
            x->src.fop[x->src.n] = f_NWSET;
            x->src.fdt[x->src.n++].setid = atoi(buf+1);
            break;            
         default:
            x->src.fop[x->src.n] = f_WORD;
            x->src.fdt[x->src.n++].wdid = GetLabId(buf,TRUE);
            break;            
      }
      s = NextWord(s,buf);
   }
   x->tgt.n = 0;
   while (s != NULL) {
      if (x->tgt.n >= MAX_FIELDS)
         HError(16020,"ReadRuleDef: too many fields in replace");
      switch(buf[0]) {
         case '$':
            x->tgt.fop[x->tgt.n] = f_FIELD;
            x->tgt.fdt[x->tgt.n++].setid = atoi(buf+1);
            break;
         default:
            x->tgt.fop[x->tgt.n] = f_WORD;
            x->tgt.fdt[x->tgt.n++].wdid = GetLabId(buf,TRUE);
            break;            
      }
      s = NextWord(s,buf);
   }
   if (x->src.n > editWinSize) editWinSize = x->src.n;
   if (trace&T_RIN)
      printf(" read rule: %5.2f [%d : %d]\n",x->pact,x->src.n,x->tgt.n);
}

/* ReadRuleSet: read rule set from file */
void ReadRuleSet(char *fn, RuleSet *rset)
{
   Source src;
   char buf[1024], *s;
   Boolean infile;
   
   if (InitSource(fn,&src,NoFilter)==FAIL) {
      HError(16010, "ReadRuleSet: Can't read rule set from '%s'", fn);
   }
   do {
      infile = ReadLine(&src,buf);
      s = SkipToWord(buf);
      if (*s != '\0') {
         if (*s == '#') 
            ReadSetDef(s+1,rset);
         else
            ReadRuleDef(s,rset);
      }
   } while (infile);
  if (trace&T_RIN) {
      printf("Loaded %d sets and %d rules from file %s\n",
              rset->nSets,rset->nRules,fn);
      fflush(stdout);
   }
   CloseSource(&src);
}

/* PrintFields: print a list of rule fields */
void PrintFields(FieldVec *fl)
{
   int i;
   
   for (i=0; i<fl->n; i++)
      switch(fl->fop[i]){
         case f_FIELD:  printf(" $%d",fl->fdt[i].flid); break;
         case f_WILD:   printf(" *"); break;
         case f_WORD:   printf(" %s",fl->fdt[i].wdid->name); break;
         case f_WSET:   printf(" %%%d",fl->fdt[i].setid); break;
         case f_NWSET:  printf(" !%d",fl->fdt[i].setid); break;
         default:       printf(" <error; unknown type>"); break;
      }
}

/* PrintRuleSet: print rule set */
void PrintRuleSet(RuleSet *rset)
{
   int i,j;
   SetDef *x;
   RuleDef *r;
   
   printf("Rule Set [%d sets, %d rules]:\n",rset->nSets,rset->nRules);
   for (i=0; i<MAX_SETS; i++) {
      if ((x = rset->setlist[i]) != NULL) {
         printf(" #%3d ",i);
         for (j=0; j<x->nItem; j++) printf(" %s",x->item[j]->name);
         printf("\n");
      }
   }
   for (r=rset->head; r != NULL; r = r->next) {
      printf("%5.2f ",r->pact);
      PrintFields(&r->src); printf(" : "); PrintFields(&r->tgt);
      printf("\n");
   }
}

/* ------------------------ Initialisation ----------------------- */

/* Exists:  return true if given file exists */
Boolean Exists(char *fn)
{
   FILE *f;
   
   if ((f=fopen(fn,"r")) == NULL) return FALSE;
   fclose(f);
   return TRUE;
}

/* InitWordMap: load and initialise wordmap */
void InitWordMap(void)
{
   CreateWordMap(imapFN, &wmap, newWords);
   if (forceCnts) wmap.hasCnts = TRUE;
   if (!htkEscape) wmap.htkEsc = FALSE;   /* default is TRUE */
   ++wmap.seqno;
   mapUpdated = FALSE;
}

/* InitShiftReg: initialise a shift register */
void InitShiftReg(ShiftReg *sr, int size, char *fn)
{
   char path[256];
   
   MakeFN(fn,dbsDir,NULL,path);
   sr->used = 0;
   sr->ng[nSize] = 1;   /* count = 1 */
   sr->ngb = CreateNGBuffer(&ngbHeap,nSize,size,path,&wmap);
   sr->ngb->fndx += dumpOfs;
}

/* Initialise: initialise global data structures */
void Initialise(void)
{
   char buf[256];
   
   if (ruleFN != NULL) {
      if (trace&T_TOP) printf(" creating rule set %s\n",ruleFN);
      CreateRuleSet(&rset);
      ReadRuleSet(ruleFN,&rset);
      if (trace&T_RUL) PrintRuleSet(&rset);
   }
   InitWordMap();
   CreateHeap(&ngbHeap,"NGB mem",MSTAK,1,0.0,1000,1000);
   if (gbGen) InitShiftReg(&stdBuf,ngbSize,rootFN);
   if (ruleFN != NULL) {
      sprintf(buf,"%s_pos",rootFN);
      InitShiftReg(&posBuf,egbSize,buf);
      sprintf(buf,"%s_neg",rootFN);
      InitShiftReg(&negBuf,egbSize,buf);
   }
}

/* ----------------- NGram Counting Routines -------------------- */

/* CompressBuffer: and save if necessary or mustSave is TRUE */
void CompressBuffer(NGBuffer *ngb, Boolean mustSave)
{
   float compx;

   if (ngb->used == 0) return;
   if (trace&T_MEM) {
      printf("** before buffer sort\n");
      PrintAllHeapStats();
   }
   SortNGBuffer(ngb);
   if (trace&T_MEM) {
      printf("** after buffer sort\n");
      PrintAllHeapStats();
   }
   compx = 100.0 * (float)ngb->used / (float)ngb->poolsize;
   if (trace&T_SAV) {
      printf(" buffer %s.%d compressed%s to %.1f%% at word %d\n",
              ngb->fn, ngb->fndx, mustSave?"[must save]":"",compx,wordnum);
   }
   if (compx > 75.0 || mustSave) {
      if (mustSave && mapUpdated) {
         SaveWordMap(omapFN,&wmap,FALSE);
         mapUpdated = FALSE;
         if (trace&T_TOP) 
            printf(" word map saved to %s\n",omapFN);
      }
      if (trace&T_TOP) {
         printf(" saving %d ngrams to file %s.%d\n",
                 ngb->used, ngb->fn, ngb->fndx);
      }
      WriteNGBuffer(ngb,txtsrc);
   }
}

/* PutShiftRegister: push word into shift register and extract ngram */
void PutShiftRegister(LabId id, ShiftReg *sr)
{
   int i;
   MapEntry *me;
   
  if (trace&T_SHR){
      printf("   %12s --> %s\n",id->name,sr->ngb->fn);
      fflush(stdout);
   }
   AddWordToMap(&wmap,id); 
   mapUpdated = TRUE;
   me = (MapEntry *)id->aux;
   sr->ng[sr->used++] = me->ndx;
   if (sr->used == nSize) {
      /* record ngram */
      StoreNGram(sr->ngb,sr->ng);
      /* shift words */
      sr->used--;
      for (i=0; i<sr->used; i++)
         sr->ng[i] = sr->ng[i+1];
      /* compress buffer if full */
      if (sr->ngb->used == sr->ngb->poolsize)  {
         CompressBuffer(sr->ngb,FALSE);
      }
   }
}

/* -------------------- Editing Routines ------------------------- */

/* MatchRule: return true if given rule matches editBuf */
Boolean MatchRule(RuleDef *r)
{
   int i,j;
   
   for (i=0; i<r->src.n; i++) {
      switch(r->src.fop[i]) {
         case f_WORD:   
            if (editBuf[i] != r->src.fdt[i].wdid) return FALSE; 
            break;
         case f_WSET:
            j = r->src.fdt[i].setid;
            if (!InSet(rset.setlist[j],editBuf[i])) return FALSE;
            break;
         case f_NWSET:
            j = r->src.fdt[i].setid;
            if (InSet(rset.setlist[j],editBuf[i])) return FALSE;
            break;
         case f_WILD:   
            break;
         default:
            HError(16090,": bad op [%d] in field %d of replace",
                   r->tgt.fop[i],i);
      }
   }
   return TRUE;
}

/* ApplyRule: put replace part of rule r into buf */
void ApplyRule(RuleDef *r, LabId *buf)
{
   int i;
   
   for (i=0; i<r->tgt.n; i++) {
      switch(r->tgt.fop[i]) {
         case f_WORD:   
            buf[i] = r->tgt.fdt[i].wdid; 
            break;
         case f_FIELD:  
            buf[i] = editBuf[r->tgt.fdt[i].flid]; 
            break;
         default:
            HError(16090,": bad op [%d] in field %d of replace",
                   r->tgt.fop[i],i);
      }
   }
}

/* SendToEditBuffer: insert word into edit buffer and apply rules */
void SendToEditBuffer(LabId id)
{
   RuleDef *r;
   LabId replBuf[MAX_FIELDS];
   int i;

   editBuf[editUsed++] = id;
   if (editUsed == editWinSize) {  /* buffer is filled */
      /* try each rule in turn */
      for (r=rset.head; r != NULL; r = r->next)
         if (MatchRule(r)) {
            r->psum += r->pact;
            if (r->psum>1.0) {
               ApplyRule(r,replBuf);
               r->psum -= 1.0;
               for (i=0; i< r->src.n; i++) 
                  PutShiftRegister(editBuf[i],&negBuf);
               negBuf.used = 0;
               for (i=0; i< r->tgt.n; i++) 
                  PutShiftRegister(replBuf[i],&posBuf);
               posBuf.used = 0;
            }
         }
      /* Shift words */
      editUsed--;
      for (i=0; i<editUsed; i++)
         editBuf[i] = editBuf[i+1];
   }
}

/* -------------------------- Input Text ------------------------- */

/* ProcessText: read text files line by line and count ngrams */
void ProcessText(char *fn, Boolean lastFile)
{
   FILE *f;
   LabId id;
   char sbuf[1024],*word;
   Boolean isPipe,wasSentStart;

   if (trace&T_TOP) 
      printf("Reading source text file %s\n",(fn==NULL) ? "<stdin>" : fn);
   if ((fn!=NULL) && (strcmp(fn,"-")!=0)) {
      if ((f = FOpen(fn,LMTextFilter,&isPipe))==NULL)
	 HError(16010,"ProcessText: unable to open text file %s", fn);
   } else {
      f = stdin;
   }
   wasSentStart = FALSE;
   word = sbuf+1; sbuf[0]='_';
   while (fscanf(f,"%255s",word)==1) {
      wordnum++;
      if (tagSentStart) {
	id = GetLabId(wasSentStart ? sbuf : word,TRUE);
	wasSentStart = (id==sstId);
      } else {
	id = GetLabId(word,TRUE);
      }
      if (trace&T_INP) printf("[%s]\n",id->name);
      if (ruleFN == NULL && !gbGen) {
	AddWordToMap(&wmap,id); 
      }	else {
	id = GetLabId(word,TRUE);
	if (ruleFN != NULL) 
	  SendToEditBuffer(id);
	if (gbGen) 
	  PutShiftRegister(id,&stdBuf);
      }
   }
   if (fn!=NULL)
      FClose(f,isPipe);
   if (lastFile) {
      if (ruleFN == NULL && !gbGen) {
         SortWordMap(&wmap); 
         SaveWordMap(omapFN,&wmap,FALSE);
      } else {
	if (gbGen)
	  CompressBuffer(stdBuf.ngb,TRUE);
	if (ruleFN != NULL){
	  CompressBuffer(negBuf.ngb,TRUE);
	  CompressBuffer(posBuf.ngb,TRUE);
	}
      }
   }
}

/* ---------------------- End of LGPrep.c ----------------------- */
