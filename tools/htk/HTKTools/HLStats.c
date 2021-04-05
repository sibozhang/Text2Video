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
/*    File: HLStats.c: gather statistics from transcriptions   */
/* ----------------------------------------------------------- */

char *hlstats_version = "!HVER!HLStats:   3.4.1 [CUED 12/03/09]";
char *hlstats_vc_id = "$Id: HLStats.c,v 1.1.1.1 2006/10/11 09:55:01 jal58 Exp $";

#include "HShell.h"
#include "HMem.h"
#include "HMath.h"
#include "HSigP.h"
#include "HAudio.h"
#include "HWave.h"
#include "HVQ.h"
#include "HParm.h"
#include "HLabel.h"
#include "HModel.h"
#include "HDict.h"
#include "HLM.h"
#include "HUtil.h"

/*
   This program collects statistics (such as number of occurrences,
   min, max and average duration) from a set of label files.
   It is also able to generate simple backoff and matrix bigram
   language models.
*/

/* -------------------------- Trace Flags & Vars ------------------------ */

#define T_BAS 0x0001                /* Trace basic progress information */
#define T_MEM 0x0002                /* Trace memory usage */
#define T_BIG 0x0004                /* Trace bigram statistics */
#define T_FIL 0x0008                /* Trace each file name */

static int trace = 0;               /* trace level */

/* -------------------------- Global Variables etc ---------------------- */

static Boolean doBigram = FALSE;    /* do what? */
static Boolean doDurs   = FALSE;
static Boolean doList   = FALSE;
static Boolean doPCount = FALSE; 
static Boolean doLCount = FALSE;
static Boolean doBOff   = FALSE;

static char *listFile   = NULL;     /* file for label list */
static char *bigFile    = NULL;     /* file for bigram */

static float uniFloor   = 1.0;      /* min count for unigram probs */
static float bigFloor   = 0.0;      /* floor for matrix bigram probs */
static int bigThresh    = 0;        /* threshold for including bigram probs */
static int pCountLimit  = -1;       /* max occurrences to list for pCount */
static int lCountLimit  = -1;       /* max occurrences to list for lCount */
static int hSize = 0;               /* hash table size, small(0), med(1), large(2)  */

static LabId enterId;               /* id of ENTRY label in ngram */
static LabId exitId;                /* id of EXIT label in ngram */
static LabId nullId;                /* id of !NULL label in ngram */

static FileFormat ff=UNDEFF;        /* Label file format */

static MemHeap tmpHeap;             /* Temporary storage */
static MemHeap statHeap;            /* Permenant stats storage */

/* ---------------- Configuration Parameters --------------------- */

static ConfParam *cParm[MAXGLOBS];
static int nParm = 0;               /* total num params */
static float disCount = 0.5;        /* discount for backoff */

/* ------------------ Process Command Line ------------------------- */

/* SetConfParms: set conf parms relevant to this tool */
void SetConfParms(void)
{
   double d;
   int i;

   nParm = GetConfig("HLSTATS", TRUE, cParm, MAXGLOBS);
   if (nParm>0){
      if (GetConfFlt(cParm,nParm,"DISCOUNT",&d)) disCount = d;
      if (GetConfInt(cParm,nParm,"TRACE",&i)) trace = i;
   }
}

void ReportUsage(void)
{
   printf("\nUSAGE: HLStats [options] hmmList labFile...\n\n");
   printf(" Option                                       Default\n\n");
   printf(" -b fn    output bigram to file fn            off\n");
   printf(" -c N     count num logical occs upto N       none\n");
   printf(" -d       compute duration statistics         off\n");
   printf(" -f f     set matrix bigram floor prob f      0.0\n");
   printf(" -h N     set hashsize: medium(1), large(2)   small(0)\n");
   printf(" -l s     output covering list of models to s off\n");
   printf(" -o       generate wsj style back-off files   matrix\n");
   printf(" -p N     count num physical occs upto N      none\n");
   printf(" -s s1 s2 select start s1 and end s2 labels   !ENTER !EXIT\n");
   printf(" -t n     set threshold for including bigram  0\n");
   printf(" -u f     set back off unigram floor prob f   1.0\n");
   PrintStdOpts("GIX");
   printf("\n\n");
}

int main(int argc, char *argv[])
{
   char * labFn, *listfn, *s;
   int i,fidx;
   MLFEntry *me = NULL;
   Transcription *t;
   void InitStats(char *listfn);
   void GatherStats(Transcription *t);
   void OutputStats(void);

   if(InitShell(argc,argv,hlstats_version,hlstats_vc_id)<SUCCESS)
      HError(1300,"HLStats: InitShell failed");

   InitMem();   InitMath();
   InitWave();  InitLabel();
   InitLM();

   if (!InfoPrinted() && NumArgs() == 0)
      ReportUsage();
   if (NumArgs() == 0) Exit(0);
   SetConfParms();
   
   enterId=GetLabId("!ENTER",TRUE); /* All sentences should or are coerced */
   exitId=GetLabId("!EXIT",TRUE);   /*  to start enterId and end exitId */
   nullId=GetLabId("!NULL",TRUE);  /* Name for words not in list */

   while (NextArg() == SWITCHARG) {
      s = GetSwtArg();
      if (strlen(s)!=1) 
         HError(1319,"HLStats: Bad switch %s; must be single letter",s);
      switch(s[0]){
      case 'b':
         doBigram = TRUE;
         if (NextArg() != STRINGARG)
            HError(1319,"HLStats: Ngram output file name expected");
         bigFile = GetStrArg();
         break;
      case 'c':
         doLCount = TRUE;
         lCountLimit = GetChkedInt(0,100000,s);
         break;
      case 'd':
         doDurs = TRUE; break;
      case 'f':
         bigFloor = GetChkedFlt(0.0,1000.0,s);
         break;
      case 'h':
         hSize =  GetChkedInt(1,2,s);
         break;
      case 'l':
         doList = TRUE; 
         if (NextArg() != STRINGARG)
            HError(1319,"HLStats: Output label list file name expected");
         listFile = GetStrArg();
         break;
      case 'o':
         doBOff = TRUE;
         break;
      case 'p':
         doPCount = TRUE;
         pCountLimit = GetChkedInt(0,100000,s);
         break;
      case 's':
         if (NextArg() != STRINGARG)
            HError(1319,"HLStats: ENTER label name expected");
         enterId=GetLabId(GetStrArg(),TRUE);
         if (NextArg() != STRINGARG)
            HError(1319,"HLStats: EXIT label name expected");
         exitId=GetLabId(GetStrArg(),TRUE);
         break;
      case 't':
         bigThresh = GetChkedInt(0,100,s);
         break;
      case 'u':
         uniFloor = GetChkedFlt(0.0,1000.0,s);
         break;
      case 'G':
         if (NextArg() != STRINGARG)
            HError(1319,"HLStats: Input label File format expected");
         if((ff = Str2Format(GetStrArg())) == ALIEN)
            HError(-1389,"HLStats: Warning ALIEN Label file format set");
         break;
      case 'I':
         if (NextArg() != STRINGARG)
            HError(1319,"HLStats: Input MLF file name expected");
         LoadMasterFile(GetStrArg());
         break;
      case 'T':
         if (NextArg() != INTARG)
            HError(1319,"HLStats: Trace value expected");
         trace = GetChkedInt(0,017,s); break;
      default:
         HError(1319,"HLStats: Unknown switch %s",s);
      }
   }

   if (NextArg()!=STRINGARG)
      HError(1319,"HLStats: Label list file name expected");
   listfn = GetStrArg();
   if (!(doDurs || doBigram || doList || doLCount || doPCount))
      HError(1330,"HLStats: Nothing to do!");
   InitStats(listfn);

   i=0;
   while (NumArgs()>0) {
      if (NextArg()!=STRINGARG)
         HError(1319,"HLStats: Input label file name expected");
      labFn = GetStrArg();
      if (IsMLFFile(labFn)) {
         fidx = NumMLFFiles();
         if ((me=GetMLFTable()) != NULL) {
            while(me->next != NULL) me=me->next;
            LoadMasterFile(labFn);
            me=me->next;
         }
         else {
            LoadMasterFile(labFn);
            me=GetMLFTable();
         }
         while (me != NULL) {
            if (me->type == MLF_IMMEDIATE && me->def.immed.fidx == fidx) {
               if (trace&T_FIL) {
                  printf("  Processing file %s\n",me->pattern); fflush(stdout);
               }
               t = LOpen(&tmpHeap,me->pattern,ff);
               if (t->numLists<1)
                  HError(-1330,"HLStats: Empty file %s",me->pattern);
               else
                  GatherStats(t),i++;

               Dispose(&tmpHeap,t);
            }
            me = me->next;
            if ((trace&T_BAS) && !(trace&T_FIL) &&
                NumMLFEntries()>5000 && i%1000==0) 
               printf(". "),fflush(stdout);
         }
         if ((trace&T_BAS) && !(trace&T_FIL) && NumMLFEntries()>5000)
            printf("\n");
      } else {
         if (trace&T_FIL) {
            printf("  Processing file %s\n",labFn); fflush(stdout);
         }
         t = LOpen(&tmpHeap,labFn,ff);
         if (t->numLists<1)
            HError(-1330,"HLStats: Empty file %s",me->pattern);
         else
            GatherStats(t),i++;
         Dispose(&tmpHeap,t);
      }
   }
   if (trace&T_MEM)
      PrintAllHeapStats();
   OutputStats();

   if (trace&T_MEM)
      PrintAllHeapStats();
   Exit(0);
   return (0);          /* never reached -- make compiler happy */
}

/* PrintSettings: print info on stats requested */
void PrintSettings(void)
{
   if (doLCount || doPCount){
      printf("Computing Label Occurrence Statistics\n");
      if (doPCount)
         printf("  upto %d physical\n",pCountLimit);
      if (doLCount)
         printf("  upto %d logical\n",lCountLimit);
   }
   if (doBigram) {
      printf("Computing Bigram Statistics\n");
      if (doBOff){
         printf("  unifloor = %f\n",uniFloor);
         printf("  bgthresh = %d\n",bigThresh);
         printf("  discount = %f\n",disCount);
      } else 
         printf("  bigfloor = %f\n",bigFloor);
   }
   if (doDurs)
      printf("Computing Label Duration Statistics\n");
   fflush(stdout);
}

/* -------------------- Gather Statistics -------------------- */

typedef struct cntr{             /* Physical Label Occurrence Counters */
   LabId name;                   /* Name */
   int count;                    /* Times seen */
} Cntr;

typedef struct wordinfo{         /* Label Occurrence Counters */
   LabId name;                   /* Name */
   int count;                    /* Times seen */
   Cntr *pCntr;                  /* Physical counter */
   float minDur;                 /* Min duration */
   float maxDur;                 /* Max duration */
   float sumDur;                 /* Total duration */
} WordInfo;

#define ASIZE 2                  /* Need two words to id a bigram */

typedef struct aentry {          /* Storage for counts */
   unsigned short word[ASIZE];   /* Bigram id */
   int count;                    /* Count */
   struct aentry *link;          /* Next entry in hash table */
} AEntry;

static int lSize;                /* Number of logical labels */
static int pSize;                /* Number of physical labels */
static WordInfo *lTab;           /* Table of logical counts/durations */
static Cntr *pTab;               /* Table of physical counts */

static AEntry **aetab;           /* Hash table for bigram accumulators  */
static int aetabsize=0;          /* Size of hash table selected from .. */
static int hashsizes[4]={ 87793, 188281, 715249 };
static int nae=0;                /* Number of accumulators created */

/* wd_cmp: word order relation used to sort lTab */
static int wd_cmp(const void *v1,const void *v2)
{
   WordInfo *w1,*w2;
   
   w1=(WordInfo*)v1;w2=(WordInfo*)v2;
   if (w1->name==enterId) return(-1);
   else if (w2->name==enterId) return(1);
   else if (w1->name==exitId) return(1);
   else if (w2->name==exitId) return(-1);
   return(strcmp(w1->name->name,w2->name->name));
}

/* InitWordInfo: Initialise contents of WordInfo rec */
void InitWordInfo(WordInfo *w, LabId id, Cntr *pCntr)
{
   w->name   = id;
   w->pCntr  = pCntr;
   w->minDur = 1E30;
   w->maxDur = 0.0;
   w->sumDur = 0.0;
   w->count  = 0;
}

/* InitStats: Create and init all necessary global accumulators */
void InitStats(char *listFn)
{
   int h,p,l;
   MLink q,hm;
   HLink hmm;
   HMMSet *hset;

   CreateHeap(&tmpHeap,"TempHeap",MSTAK,1,1.0,8000,80000);
   CreateHeap(&statHeap,"StatHeap",MSTAK,1,1.0,8000,240000);

   hset=(HMMSet*)New(&tmpHeap,sizeof(HMMSet));
   CreateHMMSet(hset,&tmpHeap,FALSE);
   if(MakeHMMSet(hset,listFn)<SUCCESS)
      HError(1328,"Initstats: MakeHMMSet failed");

   /* Make sure we have entries for ENTER / EXIT labels */
   if (FindMacroName(hset,'l',enterId)==NULL) {
      hmm=(HMMDef*)New(&tmpHeap,sizeof(HMMDef));
      NewMacro(hset,0,'l',enterId,hmm);
      NewMacro(hset,0,'h',enterId,hmm);
   }
   if (FindMacroName(hset,'l',exitId)==NULL) {
      hmm=(HMMDef*)New(&tmpHeap,sizeof(HMMDef));
      NewMacro(hset,0,'l',exitId,hmm);
      NewMacro(hset,0,'h',exitId,hmm);
   }

   pSize=hset->numPhyHMM;
   pTab=(Cntr*)New(&statHeap,(pSize+1)*sizeof(Cntr));
   
   p=1;
   pTab[0].name=nullId;
   for (h=0; h<MACHASHSIZE; h++)
      for (q=hset->mtab[h]; q!=NULL; q=q->next) {
         if (q->type=='h') {
            hmm=(HLink) q->structure;
            hmm->hook=(Ptr)p;
            pTab[p].name=q->id;
            pTab[p].count=0;
            p++;
         }
      }

   lSize=hset->numLogHMM;
   lTab=(WordInfo*)New(&statHeap,(lSize+1)*sizeof(WordInfo));

   l=1;
   InitWordInfo(lTab,nullId,pTab);
   for (h=0; h<MACHASHSIZE; h++)
      for (q=hset->mtab[h]; q!=NULL; q=q->next)
         if (q->type=='l') {
            hmm=(HLink) q->structure;
            hm=FindMacroStruct(hset,'h',q->structure);
            if (hm==NULL || hmm->hook==0)
               HError(1390,"InitStats: No physical name found for %s",
                      q->id->name);
            InitWordInfo(lTab+l,q->id,pTab+(int)hmm->hook);
            l++;
         }
   qsort(lTab+1,lSize,sizeof(WordInfo),wd_cmp);
   for (l=1; l<=lSize; l++)
      lTab[l].name->aux=(Ptr)l;
   Dispose(&tmpHeap,hset);

   if (doBigram) {   /* create aetab */
      aetabsize=hashsizes[hSize];
      aetab=(AEntry**)New(&statHeap,aetabsize*sizeof(AEntry*));
      for (l=0;l<aetabsize;l++) aetab[l]=NULL;
   }
   if (trace&T_BAS) {
      PrintSettings();
      printf("\n\nRead Label list - %d/%d labels\n",lSize,pSize);
   }
}

/* GetAEntry: find ngram in in aetab.  If not found and create
   is set, then add new entry */
AEntry *GetAEntry(int in[ASIZE],Boolean create)
{
   AEntry *ae;
   int i;
   unsigned int hash;

   hash=0;
   for (i=0,hash=0;i<ASIZE;i++)
      hash=((hash<<16)+in[i])%aetabsize;

   for (ae=aetab[hash];ae!=NULL;ae=ae->link)
      if (ae->word[0]==in[0] && ae->word[1]==in[1])
         break;

   if (ae==NULL && create) {
      nae++;
      ae=(AEntry*)New(&statHeap,sizeof(AEntry));
      for (i=0;i<ASIZE;i++)
         ae->word[i]=in[i];
      ae->count=0;
      ae->link=aetab[hash];
      aetab[hash]=ae;
   }
   return(ae);
}

/* GatherStats: update stats using given label file */
void GatherStats(Transcription *t)
{
   LLink l;
   LabList *ll;
   WordInfo *lt;
   int i,j,st,en,lab,in[ASIZE];
   float dur;
   AEntry *ae;

   ll=GetLabelList(t,1);
   st=1;  en=CountLabs(ll);

   /* If first label is enterId then we need to skip it */
   l = GetLabN(ll,1);
   if (l->labid==enterId) st++;

   /* If the final label is exitId then it should be skipped */
   l = GetLabN(ll,en);
   if (l->labid==exitId) en--;

   /* Coerce previous labels to be enterId */
   for (i=0; i<ASIZE; i++) in[i]=(int)enterId->aux;
   lt = lTab+(int)enterId->aux; ++lt->count;
   
   /* Process actual labels in list */ 
   for (i=st; i<=en; i++) {
      l = GetLabN(ll,i);
      lab=(int)l->labid->aux;
      dur = (float)(l->end - l->start)/10000.0;
      lt=lTab+lab;
      /* increment stats */
      lt->count++;
      lt->sumDur += dur;
      if (dur < lt->minDur) lt->minDur=dur;
      if (dur > lt->maxDur) lt->maxDur=dur;
      lt->pCntr->count++;
      if (doBigram) {
         /* We ignore all transitions into enterId and exitId */
         /* May wish to warn user about badly formed sentences */
         if (!(lab==(int)enterId->aux || (lab==(int)exitId->aux))) {
            for (j=ASIZE-1;j>0;j--) in[j]=in[j-1];
            in[0]=lab;
            ae = GetAEntry(in,TRUE);
            ae->count++;
         }
      }
   }
   /* Deal with transition into EXIT */
   if (doBigram) {
      for (j=ASIZE-1;j>0;j--) in[j]=in[j-1];
      in[0]=(int)exitId->aux;
      ae = GetAEntry(in,TRUE);
      ae->count++;
   }
   lt = lTab+(int)exitId->aux; ++lt->count;
}

/* ----------------------- Output Results -------------------- */

/* CmpCntr: return sign(c1->count - c2->count) , if equal then
   use same ordering as in lTab */
int CmpCntr(const void *p1, const void *p2)
{
   Cntr *c1, *c2;
   int diff;

   c1=(Cntr *)p1; c2=(Cntr *)p2;
   diff=c1->count-c2->count;
   if (diff==0) return((int)c2->name->aux-(int)c1->name->aux);
   else return(diff);
}


/* CmpWordInfo: return sign(c1->count - c2->count) , if equal then
   use same ordering as in lTab */
int CmpWordInfo(const void *p1, const void *p2)
{
   WordInfo *c1, *c2;
   int diff;
   
   c1=(WordInfo *)p1; c2=(WordInfo *)p2;
   diff=c1->count-c2->count;
   if (diff==0) return((int)c2->name->aux-(int)c1->name->aux);
   else return(diff);
}

/* OutputCounts: output logical/physical counters */
void OutputCounts(void)
{
   int i;
   WordInfo *l;
   Cntr *p;

   if (doLCount){
      qsort(lTab+1,lSize,sizeof(WordInfo),CmpWordInfo);
      printf("\nLogical Model Counts:\n");
      printf("       Label   LCount   PCount\n");
      for (i=0,l=lTab+1; i<lSize; i++,l++){
         if (l->count > lCountLimit) break;
         printf("%12s %8d %8d\n",l->name->name,l->count,l->pCntr->count);
      }
   }
   if (doPCount){                      /* Breaks log->phy relation */
      qsort(pTab+1,pSize,sizeof(Cntr),CmpCntr);
      printf("\nPhysical Model Counts:\n");
      printf("       Label   PCount\n");
      for (i=0,p=pTab+1; i<pSize; i++,p++){
         if (p->count > pCountLimit) break;
         printf("%12s %8d\n",p->name->name,p->count);
      }
   }
   printf("\n");
   fflush(stdout);
}

/* OutputDurs: output duration stats */
void OutputDurs(void)
{
   int i;
   WordInfo *l;
   
   printf("\nDuration Statistics:\n");
   printf("       Label   Count  AveDur  MinDur  MaxDur\n");
   for (i=0,l=lTab+1; i<lSize; i++,l++){
      printf("%12s %7d",l->name->name,l->count);
      if (l->count>0 && l->name != enterId && l->name != exitId) {
         printf("%8.1f",l->sumDur/l->count);
         if (l->minDur < 1E30)
            printf("%8.1f",l->minDur);
         else
            printf("%8s","---");
         printf("%8.1f",l->maxDur);
      }
      printf("\n");
   }
   printf("\n"); fflush(stdout);
}

/* OutputList: output a list of all labels that occurred at least once */
void OutputList(void)
{
   int i;
   FILE *f;
   WordInfo *l;

   if ((f=fopen(listFile,"w"))==NULL)
      HError(1311,"OutputList: Cannot create label list file %s",listFile);   
   for (i=0,l=lTab+1; i<lSize; i++,l++)
      if (l->count>0)
         fprintf(f,"%s\n",l->name->name);
   fclose(f);
}

/* ------------------- Bigram Handling ---------------------- */

#define log2(x) (log(x)/log(2.0))
#define ent2(x) ((x)>0.0?((x)*log2(x)):0.0)

/* RebuildAETab: rebuild the aetab in aelists such that all
   ngrams  (n,x) are stored in the list aelists[n]. */
void RebuildAETab(AEntry **aelists)
{
   AEntry *ae,*nx;
   int h;

   for (h=0; h<aetabsize; h++) {
      for (ae=aetab[h]; ae!=NULL; ae=nx) {
         nx=ae->link;
         if (ae->word[1]==0) continue;
         ae->link=aelists[ae->word[1]];
         aelists[ae->word[1]]=ae;
      }
      aetab[h]=NULL;
   }
}

/* se_cmp: ordering relation for SEntrys based on word id */
int se_cmp(const void *v1,const void *v2)
{
   SEntry *s1,*s2;

   s1=(SEntry*)v1;  s2=(SEntry*)v2;
   return((int)(s1->word-s2->word));
}

/* Simple calculation of backoff weights - 0.5 subtracted from each count */
static float BuildNEntry(NEntry *ne,Vector boff,float bent)
{
   SEntry *cse;
   AEntry *ae;
   double bowt,bsum,cnt,tot,ent,prob;
   
   ne->nse=0;
   tot=cnt=0.0;
   bsum=1.0;
   if (ne->word[0]!=(int)exitId->aux)
      for (ae=(AEntry *) ne->user; ae!=NULL; ae=ae->link) {
         tot+=ae->count;
         if (ae->word[0]!=0 && ae->word[0]!=(int)enterId->aux &&
             ae->count>bigThresh)
            cnt+=(ae->count-disCount),ne->nse++,bsum-=boff[ae->word[0]];
      }
   if (ne->nse==0) {
      ne->se=NULL;
      ne->bowt=0.0;
      ent=bent;
   }
   else {
      ne->se=(SEntry*)New(&statHeap,sizeof(SEntry)*ne->nse);
      bowt = (bsum>0.0) ? (1.0-cnt/tot)/bsum : 0.0;
      ent  = (bowt>0.0) ? bowt*(bent-log2(bowt)) : 0.0;
      for (cse=ne->se,ae=(AEntry *) ne->user; ae!=NULL; ae=ae->link)
         if (ae->word[0]!=0 && ae->word[0]!=(int)enterId->aux &&
             ae->count>bigThresh) {
            prob=((double)ae->count-disCount)/tot;
            cse->word=ae->word[0];
            cse->prob=log(prob);
            ent -= ent2(prob);
            prob = bowt*boff[cse->word];
            ent += ent2(prob);
            cse++;
         }
      if (bowt>0.0) ne->bowt=log(bowt);
      else ne->bowt=LZERO;
      qsort(ne->se,ne->nse,sizeof(SEntry),se_cmp);
   }
   return(ent);
}

/* OutputBoBigram: output ARPA/MIL-LL style back off bigram */
void OutputBoBigram(void)
{
   LModel lm;
   NGramLM *nglm;
   NEntry *ne;
   SEntry *se;
   AEntry **aelists;
   lmId ndx[NSIZE];
   int i,tot,counts[NSIZE+1];
   double uent,ent,bent;

   lm.heap=&statHeap;
   lm.type=boNGram;
   counts[1]=lSize;counts[2]=nae;
   for(i=3;i<NSIZE+1;i++)
      counts[i]=0;
   nglm=CreateBoNGram(&lm,lSize,counts);  /* Give max size at creation */
   for (i=1;i<=lSize;i++)
      nglm->wdlist[i]=lTab[i].name;

   aelists=(AEntry**)New(&tmpHeap,sizeof(AEntry*)*(lSize+1));
   for (i=1;i<=lSize;i++) aelists[i]=NULL;
   RebuildAETab(aelists);          /* Un-hash hashtable */

   for (i=1,tot=0.0;i<=lSize;i++) {    /* Calculate unigrams first */
      if (i==(int)enterId->aux)
         nglm->unigrams[i]=0.0;
      else if (lTab[i].count<uniFloor)
         nglm->unigrams[i]=uniFloor;
      else
         nglm->unigrams[i]=lTab[i].count;
      tot+=nglm->unigrams[i];
   }
   for (i=1,uent=0.0;i<=lSize;i++,se++) {
      nglm->unigrams[i]=nglm->unigrams[i]/tot;
      uent-=ent2(nglm->unigrams[i]);
   }

   nglm->counts[1]=lSize;           /* Calculate real sizes during build */
   nglm->counts[2]=0;
   for (i=0; i<NSIZE; i++) ndx[i]=0;
   if (trace&T_BIG) {
      printf("\n  UNIGRAM NEntry        - %4d foll, ent %.3f [= %.3f]\n\n",
             lSize,uent,pow(2.0,uent));
      printf("  BIGRAMS NEntries\n");
      fflush(stdout);
   }
   for (i=1,bent=0.0;i<=lSize;i++) {
      ndx[0]=i;
      ne=GetNEntry(nglm,ndx,TRUE);
      ne->user=aelists[i];
      ent = BuildNEntry(ne,nglm->unigrams,uent);
      nglm->counts[2]+=ne->nse;
      if (trace&T_BIG) 
         if (i!=(int)exitId->aux){
            if (i==(int)enterId->aux)
               bent+=nglm->unigrams[(int)exitId->aux]*ent;
            else 
               bent+=nglm->unigrams[i]*ent;
            printf("   %-20s - %4d foll, ent %6.3f [= %6.2f]\n",
                   lTab[i].name->name,ne->nse,ent,pow(2.0,ent));
            fflush(stdout);
         }
   }
   Dispose(&tmpHeap,aelists);
   
   if (trace&T_BIG) {
      printf("\n  BIGRAM: training data entropy %.3f (perplexity %.2f)\n",
             bent,pow(2.0,bent));
      fflush(stdout);
   }

   ndx[0]=0;                        /* Set up unigram nentry separately */
   ne=GetNEntry(nglm,ndx,TRUE);
   ne->nse=lSize;
   se=ne->se=(SEntry*)New(nglm->heap,sizeof(SEntry)*lSize);
   for (i=1;i<=lSize;i++,se++) {
      se->word=i;
      if (nglm->unigrams[i]>0)
         se->prob=nglm->unigrams[i]=log(nglm->unigrams[i]);
      else
         se->prob=nglm->unigrams[i]=LZERO;
   }  

   lm.name=CopyString(lm.heap,bigFile); /* Name and write to disk */
   WriteLModel(&lm,bigFile,0);
}

/* OutputMatBigram: output matrix style bigram */
void OutputMatBigram(void)
{
   LModel lm;
   MatBiLM *matbi;
   AEntry **aelists,*ae;
   Vector vec;
   double vsum,fsum,tot,scale;
   double ent,bent,prob,fent;
   int i,j,nf,tf=0,nu,tu=0,np,tp=0,tn=0;

   lm.heap=&statHeap;
   lm.type=matBigram;
   matbi=CreateMatBigram(&lm,lSize);

   for (i=1;i<=lSize;i++)
      matbi->wdlist[i]=lTab[i].name;

   aelists=(AEntry**)New(&tmpHeap,sizeof(AEntry*)*(lSize+1));
   for (i=1;i<=lSize;i++) aelists[i]=NULL;
   RebuildAETab(aelists);          /* Un-hash hashtable */

   if (trace&T_BIG) {
      printf("\n  BIGRAMS from MatBigram\n");
      fflush(stdout);
   }
   bent=0.0;
   fent = ent2(bigFloor);
   for (i=1;i<=lSize;i++) {
      vec=matbi->bigMat[i];
      for (ae=aelists[i],tot=0.0; ae!=NULL; ae=ae->link)
         if (ae->word[0]!=0) tot += ae->count;
      fsum = (lSize-1)*bigFloor; vsum=0.0;
      for (ae=aelists[i];ae!=NULL;ae=ae->link)
         if (ae->count/tot > bigFloor && ae->word[0]!=0)
            fsum -= bigFloor, vsum += ae->count;
         else
            ae->count=0;
      scale = (1.0 - fsum) / vsum;
      for (j=1;j<=lSize;j++) {
         if (j==(int)enterId->aux) vec[j]=0.0;
         else if (tot==0.0) vec[j]=1.0/(lSize-1);
         else vec[j]=bigFloor;
      }
      for (ae=aelists[i];ae!=NULL;ae=ae->link)
         if (ae->count>0)
            vec[ae->word[0]]=ae->count*scale;
      if (trace&T_BIG) {
         nf=nu=np=0;
         if (tot==0.0) 
            ent=-log2(1.0/(lSize-1)),prob=1.0,nu=lSize-1;
         else
            ent=-(lSize-1)*fent,
               prob=bigFloor*(lSize-1),
               nf+=lSize-1;
         for (ae=aelists[i];ae!=NULL;ae=ae->link)
            if (ae->count>0) {
               prob += vec[ae->word[0]]-bigFloor;
               ent -= ent2(vec[ae->word[0]]);
               ent += fent;
               nf--;  np++;
            }
         if (i!=(int)exitId->aux){
            j=lTab[i].count;
            bent+=j*ent;tn+=j;
            if (tot==0.0)
               printf("   %-20s - %4d unis, ent %6.3f [= %6.2f] (P=%7.5f)\n",
                      lTab[i].name->name,nu,ent,pow(2.0,ent),prob);
            else
               printf("   %-20s - %4d foll, ent %6.3f [= %6.2f] (P=%7.5f)\n",
                      lTab[i].name->name,np,ent,pow(2.0,ent),prob);
            fflush(stdout);
         }
         tf+=nf;tu+=nu;tp+=np;
      }
   }
   if (trace&T_BIG) {
      bent/=tn;
      printf("\n  BIGRAM: training data entropy %.3f (perplexity %.2f)\n",
             bent,pow(2.0,bent));
      printf("         Estimated %d, floored %d, unigrammed %d for %d\n",
             tp,tf,tu,lSize);
      fflush(stdout);
   }

   Dispose(&tmpHeap,aelists);

   /* convert probabilities to logs */
   for (i=1;i<=matbi->numWords;i++) {
      vec = matbi->bigMat[i];
      for (j=1; j<=matbi->numWords; j++){
         vec[j] = ((vec[j]<MINLARG)?LZERO:log(vec[j]));
      }
   }
   lm.name=CopyString(lm.heap,bigFile); /* Name and write to disk */
   WriteLModel(&lm,bigFile,0);
}

/* OutputStats: print desired stats on standard output */
void OutputStats(void)
{  
   if (doDurs) OutputDurs();
   if (doBigram) {
      if (doBOff)
         OutputBoBigram();
      else
         OutputMatBigram();
   }
   if (doList) OutputList();
   if (doPCount || doLCount) 
      OutputCounts(); /* Breaks log->phy links */
}

/* ------------------------------------------------------------ */
/*                      END:  HLStats.c                         */
/* ------------------------------------------------------------ */
