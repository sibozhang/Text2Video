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
/*         File: HLabel.c:   Speech Label File Input           */
/* ----------------------------------------------------------- */

char *hlabel_version = "!HVER!HLabel:   3.4.1 [CUED 12/03/09]";
char *hlabel_vc_id = "$Id: HLabel.c,v 1.1.1.1 2006/10/11 09:54:57 jal58 Exp $";

#include "HShell.h"
#include "HMem.h"
#include "HMath.h"
#include "HWave.h"
#include "HLabel.h"

/* ----------------------------- Trace Flags ------------------------- */

static int trace = 0;
#define T_MLF     0001     /* Top Level MLF tracing */
#define T_MHASH   0002     /* MLF Hashing */
#define T_MAT     0004     /* MLF Pattern Matching */
#define T_SUBD    0010     /* MLF Subdir search */
#define T_HTKL    0020     /* HTK Lab file loading */
#define T_HASH    0040     /* GetLabId Hashing */
#define T_SAV     0100     /* Label file saving */

/* -------  Input file formats supported by this module ---------------
 
      TIMIT - text files with each label on a line in the form
                  start-sample end-sample  label   
              (eg 456 7899 sh). 
              
      SCRIBE - SAM format basically a sequence of tagged lines of
              form
                       <header stuff>
                   LBD:
                   LBB:  st, ,en, label
                   LBB:  st, ,en, label
                       ....
                   ELF:
                   

      HTK   - a HTK transcription consists of one or more label lists
              where each label list is separated by 3 / characters
                 <transcription> = <label list> {"///" <label list>}
              each label list is a sequence of lines of text, each defining
              a labelled segment of speech
                 <label list> = <labelseg> { <labelseg> }
              each label seg consists of optional start and end times in
              units of 100ns followed by one or more label names followed 
         by an optional score  
        <label> = [<start> <end>] < <name> [<score>] > [";" <comment>]
              The start and end times are written as ints externally but
              are stored as doubles, the score can be
              int or float.  The name must begin with a non-digit
              character other than the level separator.

      ESPS  - waves label files. Text files with each label on a line 
              in the form
                 time color-code label   (eg 0.138375 121 h#)
              see the ESPS waves manual pages for details

*/

/* ----------------- Master Label File Data Structures ---------

An MLF is a file containing a sequence of patterns, each pattern
either points to a subdirectory to search for a label file or
has a definition immediately following it (terminated by a period
on a single line.  The file must start with the MLF id.
   
#!MLF!#
"pattern1" -> "subdir1"
"pattern2" => "subdir2"
"pattern3"
0   100 sil
101 205 bah
206 300 sil
.
"pattern4"
etc
   
where -> denotes a simple search ie the name of the file matching
pattern1 must be in subdir1; => denotes a full search so that some
part of the file's path matching pattern2 must be in subdir2
*/

static ConfParam *cParm[MAXGLOBS];        /* config parameters */
static int numParm = 0;
static Boolean stripTriPhones = FALSE;   /* Enable triPhone stripping */
static int transLev = 0;           /* if >0 filter all but specified level */
static int transAlt = 0;           /* if >0 filter all but specified alt */
static Boolean compatMode = FALSE;  /* Allow spaces around . or /// */
static char labelQuote = 0;        /* How do we quote label names */
static double htkLabelTimeScale = 1; /* multiply all times in HTK format labels by this on reading */

/* --------------- Global MLF Data Structures  --------- */

#define MLFCHUNKSIZE 500
#define MAXMLFS 200

static int      numMLFs = 0;     /* number of MLF files opened */
static FILE   * mlfile[MAXMLFS]; /* array [0..numMLFs-1] of MLF file */
static int      mlfUsed = 0;     /* number of entries in mlfTab */
static MLFEntry *mlfHead = NULL; /* head of linked list of MLFEntry */
static MLFEntry *mlfTail = NULL; /* tail of linked list of MLFEntry */
static MemHeap mlfHeap;          /* memory heap for MLF stuff */

typedef struct {
   FILE *file;
   LabId name;
} OutMLFEntry;

static FILE *outMLF = NULL;                 /* output MLF file, if any */ 
static int numOutMLF = 0;                   /* number of output MLFs */ 
static OutMLFEntry outMLFSet[MAXMLFS];      /* array of output MLFs */

/* ---------------- Label Name Hashing ----------------- */

#define HASHSIZE 250007                /* size of hash table */
static NameCell *hashtab[HASHSIZE];  /* the actual table */
static MemHeap namecellHeap;         /* heap for name cells */
static long numAccesses = 0;
static long numTests = 0;

/* Hash: return a hash value for given label name */
static unsigned Hash(char *name)
{
   unsigned hashval;

   for (hashval=0; *name != '\0'; name++)
      hashval = *name + 31*hashval;
   return hashval%HASHSIZE;
}

/* NewCell: return a pointer to a new NameCell */
static NameCell *NewCell(char *name)
{
   char *s;
   NameCell *p;
   int len;

   len = strlen(name);
   s = (char *)New(&namecellHeap,len+1);
   strcpy(s,name);
   p = (NameCell *) New(&namecellHeap,sizeof(NameCell));
   p->name = s; p->next = NULL; p->aux = NULL;
   return p;
}

/* EXPORT->InitLabel: initialise module */
void InitLabel(void)
{
   int i;
   Boolean b;
   double d;
   char str[MAXSTRLEN];

   Register(hlabel_version,hlabel_vc_id);
   CreateHeap(&namecellHeap,"namecellHeap",MSTAK,1,0.5,5000,20000);
   for (i=0;i<HASHSIZE;i++)
      hashtab[i] = NULL;
   CreateHeap(&mlfHeap,"mlfHeap",MSTAK,1,0.5,10000,50000);
   numParm = GetConfig("HLABEL", TRUE, cParm, MAXGLOBS);
   if (numParm>0){
      if (GetConfInt(cParm,numParm,"TRACE",&i)) trace = i;
      if (GetConfBool(cParm,numParm,"STRIPTRIPHONES",&b)) 
         stripTriPhones = b;
      if (GetConfBool(cParm,numParm,"V1COMPAT",&b))  compatMode = b;
      if (GetConfStr(cParm,numParm,"LABELSQUOTE",str))
         labelQuote=str[0];
      if (GetConfInt(cParm,numParm,"TRANSALT",&i)) transAlt = i;
      if (GetConfInt(cParm,numParm,"TRANSLEV",&i)) transLev = i;
      if (GetConfFlt(cParm,numParm,"HTKLABELTIMESCALE",&d)) htkLabelTimeScale = d;
   }
}


/* EXPORT->GetLabId: return id of given name */
LabId GetLabId(char *name, Boolean insert)
{
   int h;
   NameCell *p;

   ++numAccesses; ++numTests;
   if ((trace&T_HASH) && numAccesses%100 == 0) 
      PrintNameTabStats();
   h = Hash(name); p = hashtab[h];
   if (p==NULL) {  /* special case - this slot empty */
      if (insert)
         p=hashtab[h]=NewCell(name);
      return p;
   }
   do{             /* general case - look for name */
      if (strcmp(name,p->name) == 0)
         return p; /* found it */
      ++numTests;
      p = p->next;
   } while (p != NULL);
   if (insert){    /* name not stored */
      p = NewCell(name);
      p->next = hashtab[h];
      hashtab[h] = p; 
   }
   return p;
}

/* EXPORT->PrintNameTabStats: print out statistics on hash table usage */
void PrintNameTabStats(void)
{
   printf("Name Table Statistics:\n");
   printf("Total Accesses: %ld\n", numAccesses);
   printf("Ave Search Len: %f\n",(float)numTests/(float)numAccesses); 
   PrintHeapStats(&namecellHeap);
   printf("\n"); fflush(stdout);
}

/* EXPORT->ReadLabel: into buf from file f and return TRUE if ok */
Boolean ReadLabel(FILE *f, char *buf)
{
   int c;
   
   c = fgetc(f);
   while (isspace(c)) c = fgetc(f);
   if (c==EOF || !isgraph(c)) return FALSE;
   do {
      *buf++ = c; c = fgetc(f);
   } while ( !isspace(c) && c != EOF);
   if (c != EOF) ungetc(c,f);
   *buf = '\0';
   return TRUE;
}

/* -------------------- Label List Handling -------------------- */

/* EXPORT->CreateTranscription: create empty transcription */
Transcription *CreateTranscription(MemHeap *x)
{
   Transcription *t;
   
   t = (Transcription *)New(x,sizeof(Transcription));
   t->head = t->tail = NULL;
   t->numLists = 0;
   return t;
}

/* EXPORT->CopyTranscription: create copy of given transcription */
Transcription *CopyTranscription(MemHeap *x, Transcription *t)
{
   Transcription *newt;
   LabList *ll,*newll;
   
   newt = CreateTranscription(x);
   ll = t->head;
   for (ll = t->head; ll != NULL; ll=ll->next){
      newll = CopyLabelList(x,ll);
      AddLabelList(newll,newt);
   }
   return newt;  
}

/* EXPORT->CreateLabelList: create a new label list with sentinels */
LabList* CreateLabelList(MemHeap *x, int maxAuxLab)
{
   LLink st,en;
   LabList *ll;
   
   ll = (LabList *)New(x,sizeof(LabList));
   st = (LLink)New(x,sizeof(Label));
   en = (LLink)New(x,sizeof(Label));
   st->labid = en->labid = NULL; 
   st->pred = NULL; st->succ = en;
   en->succ = NULL; en->pred = st;
   ll->head = st; ll->tail = en; ll->next = NULL;
   ll->maxAuxLab = maxAuxLab;
   return ll;
}

/* EXPORT->AddLabelList: Add given label list to transcription t */
void AddLabelList(LabList *ll, Transcription *t)
{
   if (ll==NULL) return;
   ++t->numLists;
   if (t->tail==NULL)
      t->head = t->tail = ll;
   else {
      t->tail->next = ll; t->tail = ll;
   }
}

/* EXPORT->GetLabelList: get n'th label list (n=1,2,...) */
LabList* GetLabelList(Transcription *t, int n)
{
   LabList* q;
   int i;
   
   if (n>t->numLists)
      HError(6570,"GetLabelList: n[%d] > numLists[%d]",n,t->numLists);
   q = t->head;
   for (i=1; i<n; i++) q = q->next;
   return q;
}

/* EXPORT->CopyLabelList: return a copy of given label list */
LabList* CopyLabelList(MemHeap *x, LabList* ll)
{
   LabList *newll;
   LLink p,q;
   
   newll = CreateLabelList(x,ll->maxAuxLab);
   for (q=ll->head->succ; q->succ != NULL; q = q->succ){
      p = AddLabel(x,newll,q->labid,q->start,q->end,q->score);
      if (ll->maxAuxLab > 0)
         AddAuxLab(p,ll->maxAuxLab,q->auxLab,q->auxScore);
   }
   return newll;
}

/* EXPORT->CreateLabel: create a label with maxAux aux slots */
LLink CreateLabel(MemHeap *x, int maxAux)
{
   LLink p;
   int i;
   LabId *id;
   float *s;
   
   p = (LLink)New(x,sizeof(Label));
   p->labid = NULL; p->score = 0.0; 
   p->auxLab = NULL; p->auxScore = NULL;
   p->start = p->end = 0;
   p->succ = p->pred = NULL;
   if (maxAux > 0) {
      id = (LabId *)New(x,sizeof(LabId)*maxAux);
      s = (float *)New(x,sizeof(float)*maxAux);
      p->auxLab = id - 1; p->auxScore = s - 1;
      for (i=1; i<=maxAux; i++){
         p->auxLab[i] = NULL;
         p->auxScore[i] = 0.0;
      }  
   }
   return p;
}

/* EXPORT->AddLabel: append given label info to given label list */
LLink AddLabel(MemHeap *x, LabList *ll, LabId id,
               HTime st, HTime en, float score)
{
   LLink p,q,newLL;
   
   p = ll->tail->pred; q = ll->tail;
   newLL = CreateLabel(x,ll->maxAuxLab);
   newLL->labid = id; newLL->score = score; newLL->start = st; newLL->end = en;
   q->pred = newLL; newLL->succ = q;
   p->succ = newLL; newLL->pred = p;
   return newLL;
}

/* EXPORT->AddAuxLab: Store n auxiliary label/score in lab */
void AddAuxLab(LLink lab, int n, LabId *auxLab, float *auxScore)
{
   int i;
   
   for (i=1; i<=n; i++){
      lab->auxLab[i] = auxLab[i];
      lab->auxScore[i] = auxScore[i];
   }
}

/* EXPORT->DeleteLabel: unlink given label from a label list */
void DeleteLabel(LLink item)
{
   LLink p,q;
   
   if (item->pred == NULL || item->succ == NULL)
      HError(6571,"DeleteLabel: attempt to delete sentinel");
   p = item->pred; q = item->succ;
   q->pred = p; p->succ = q;
}


/* EXPORT->NumCases: find num cases of primary label in label list */
int NumCases(LabList *ll, LabId id)
{
   int n = 0;
   LLink p;
   
   for (p=ll->head->succ; p->succ!=NULL; p=p->succ) 
      if (p->labid == id) ++n;
   return n;
}

/* EXPORT->GetCase: find nth occ of primary label in label list */
LLink GetCase(LabList *ll, LabId id, int n)
{
   LLink p;
   int k=0;
   
   for (p=ll->head->succ; p->succ!=NULL; p=p->succ) {
      if (p->labid == id) 
         if (++k==n) return p;
   }
   HError(6571,"GetCase: %d case of %s nonexistent",n,id->name);
   return NULL;
}

/* EXPORT->NumAuxCases: find num cases of aux label i in label list */
int NumAuxCases(LabList *ll, LabId id, int i)
{
   int n = 0;
   LLink p;
   
   if (ll->maxAuxLab < i)
      HError(6570,"NumAuxCases: aux idx %d > max[%d]",i,ll->maxAuxLab);
   for (p=ll->head->succ; p->succ!=NULL; p=p->succ) 
      if ((i==0) ? (p->labid==id) : (p->auxLab[i]==id))  ++n;
   return n;
}

/* EXPORT->GetAuxCase: find nth occ of aux label i in label list */
LLink GetAuxCase(LabList *ll, LabId id, int n, int i)
{
   LLink p;
   int k=0;
   
   if (ll->maxAuxLab < i)
      HError(6570,"GetAuxCase: aux idx %d > max[%d]",i,ll->maxAuxLab);
   for (p=ll->head->succ; p->succ!=NULL; p=p->succ) {
      if ((i==0) ? (p->labid==id) : (p->auxLab[i]==id))
         if (++k==n) return p;
   }
   HError(6571,"GetAuxCase: %d case of %s nonexistent",n,id->name);
   return NULL;
}

/* EXPORT->GetLabN: return n'th primary label */
LLink GetLabN(LabList *ll, int n)
{
   int count=0;
   LLink p;

   for (p=ll->head->succ; p->succ!= NULL; p=p->succ) 
      if (++count==n) return p;
   HError(6571,"GetLabN: %d'th label nonexistent",n);
   return NULL;
}

/* EXPORT->GetAuxLabN: return n'th aux i label */
LLink GetAuxLabN(LabList *ll, int n, int i)
{
   int count=0;
   LLink p;

   if (ll->maxAuxLab < i)
      HError(6570,"GetAuxLabN: aux idx %d > max[%d]",i,ll->maxAuxLab);
   for (p=ll->head->succ; p->succ!= NULL; p=p->succ) 
      if (i==0 || p->auxLab[i]!=NULL)      
         if (++count==n) return p;
   HError(6571,"GetAuxLabN: %d'th aux[%d] label nonexistent",n,i);
   return NULL;
}

/* EXPORT->CountLabs: count num primary labels in lab list */
int CountLabs(LabList *ll)
{
   int count = 0;
   LLink p;

   if (ll!=NULL)
      for (p=ll->head->succ; p->succ!= NULL; p=p->succ) 
         ++count;
   return(count);
}

/* EXPORT->CountAuxLabs: count num aux i labels in lab list */
int CountAuxLabs(LabList *ll, int i)
{
   int count = 0;
   LLink p;

   if (ll!=NULL)
      for (p=ll->head->succ; p->succ != NULL; p=p->succ)
         if (i==0 || p->auxLab[i]!=NULL)
            ++count;
   return(count);
}

/* EXPORT->AuxLabEndTime: return the end time for the i'th aux lab */
HTime AuxLabEndTime(LLink p, int i)
{
   LLink q;
   
   q = p->succ;
   while (i!=0 && q->succ != NULL && q->auxLab[i]==NULL){
      p = q; q = q->succ;
   }
   return p->end;
}

/* PrintLabel: print the given label on one line */
static void PrintLabel(LLink p, int maxAux)
{
   int n;
   LabId id;
   
   printf("%8.0f%8.0f",p->start,p->end);
   printf(" %8s %5f",p->labid->name,p->score);
   for (n=1; n<=maxAux; n++){
      id = p->auxLab[n];
      printf(" %8s %5f",(id!=NULL)?id->name:"<null>",p->auxScore[n]);
   }
   printf("\n");
}

/* PrintList: print the given label list */
static void PrintList(LabList *ll)
{
   int i;
   LLink p;
   
   for (p=ll->head->succ,i=1; p->succ!= NULL; p=p->succ,i++) {
      printf("%4d. ",i);
      PrintLabel(p,ll->maxAuxLab);
   }
}

/* EXPORT->PrintTranscription: for diagnostics */
void PrintTranscription(Transcription *t, char *title)
{
   int i;
   LabList *ll;
   
   printf("Transcription: %s [%d lists]\n",title,t->numLists);
   ll = t->head;
   for (i=1; i<=t->numLists; i++) {
      printf(" List %d\n",i);
      PrintList(ll); ll = ll->next;
   }
}

/* FilterLevel: remove all but given level from transcription */
static void FilterLevel(Transcription *t, int lev)
{
   LabList *ll;
   LLink p;   
   
   for (ll = t->head; ll != NULL; ll = ll->next) {
      if (ll->maxAuxLab < lev)
         HError(6570,"FilterLevel: level %d > max[%d]",lev,ll->maxAuxLab);
      if (lev > 0) {
         for (p=ll->head->succ; p->succ!=NULL; p=p->succ) {
            if (p->auxLab[lev] != NULL) {
               p->labid = p->auxLab[lev];
               p->score = p->auxScore[lev];
               p->end = AuxLabEndTime(p,lev);
            } else
               DeleteLabel(p);
         }
      }
      ll->maxAuxLab = 0;
   }
}

/* --------------- Format Specific Label File Input Routines --------------- */

#define LEVELSEP "///"
#define strlen_LEVELSEP 3
#define COMMCHAR  ';'
#define LFEED   '\012'
#define CRETURN '\015'

enum _TrSymbol{TRNULL,TRNUM,TRSTR,TREOL,TRLEV,TRCOMMA,TREOF};
typedef enum _TrSymbol TrSymbol;

static int curch = ' ';
static TrSymbol trSym = TRNULL;
static double trNum;
static char trStr[256];

/* IsNumeric: returns true if given string is a number */
Boolean IsNumeric(char *s)
{
   int i,len;
   
#ifdef WIN32
   if(*s < 0)
      return FALSE;
#endif

   len = strlen(s)-1;
   if (!(isdigit((int) s[0])||s[0]=='+'||s[0]=='-')) return FALSE;
   if (!(isdigit((int) s[len]))) return FALSE;   
   for (i=1; i<len; i++)
      if (!(isdigit((int) s[i]) || s[i]=='.' || s[i]=='-' || s[i]=='+' || s[i]=='e' || s[i]=='E' ))
         return FALSE;
   return TRUE;
}

/* InitTrScan: initialise the scanner */
static void InitTrScan(void)
{
   curch = ' ';
   trSym = TRNULL;
}

/* GetTrSym: get next symbol from f, remember that f might be an MLF
             in which case EOF is a period on its own line   */
static void GetTrSym(Source *src, Boolean htk)
{
   int nxtch;
   Boolean trSOL;

   trNum = 0.0; trStr[0]='\0'; 
   if (trSym==TRNULL) curch = GetCh(src);
   if (trSym==TREOL || trSym==TRNULL)
      trSOL=TRUE;
   else
      trSOL=FALSE;
   while (curch == ' ' || curch == '\t') {
      trSOL=FALSE;
      curch = GetCh(src);
   }
   if (!htk && curch == COMMCHAR)
      SkipLine(src);
   
   switch (curch) {
   case EOF:
      trSym = TREOF;
      break;
   case LFEED:
      curch = GetCh(src);
      trSym = TREOL;
      break;
   case CRETURN:
      curch = GetCh(src);
      if (curch == LFEED) curch = GetCh(src);
      trSym = TREOL;
      break;
   case ',':
      if (!htk) {
         curch = GetCh(src); trSym = TRCOMMA;
         break;
      }
   case '.':
      if (curch=='.' && trSOL==TRUE && mlfUsed>0 && htk) {
         nxtch = GetCh(src);
         if (nxtch == LFEED  || nxtch == CRETURN) {
            trSym = TREOF;
            break;
         }
         UnGetCh(nxtch,src);  /*  Requires more than one character pushback */
      }
   default:
      if (htk) {
         UnGetCh(curch,src);
         if (!ReadString(src,trStr)) {
            trSym=TREOF;
            break;
         }
         curch=GetCh(src);
         if (trSOL && strcmp(LEVELSEP,trStr)==0) {
            if (curch == LFEED  || curch == CRETURN) {
               trSym = TRLEV;
               break;
            }
         }
      }
      else {
         nxtch=0;
         do {
            if (nxtch>=255) break;
            trStr[nxtch++]=curch; curch=GetCh(src);
         }
         while (!isspace(curch) && curch != ',' && curch != EOF);
         trStr[nxtch]='\0';
         src->wasQuoted=FALSE;
      }
      if (!src->wasQuoted && IsNumeric(trStr)){
         sscanf(trStr,"%lf",&trNum);
         trSym = TRNUM;
         break;
      }
      if (htk && compatMode && 
          (strcmp(LEVELSEP,trStr)==0 || strcmp(".",trStr)==0)) {
         src->wasNewline=FALSE;
         SkipWhiteSpace(src);
         if (src->wasNewline) {
            curch = CRETURN;
            trSym=(strcmp(LEVELSEP,trStr)==0?TRLEV:TREOF);
            break;
         }
         curch = GetCh(src);
      }
      if (stripTriPhones) TriStrip(trStr);
      trSym = TRSTR;
      break;
   }
}

/* -------------------- HTK Label Format ------------------ */

/* ExtendAux: extend the aux arrays in given lab list to n elems 
   New elems are set to NULL/0.0 */
static void ExtendAux(MemHeap *x, LabList *ll, int n)
{
   int i,oldn;
   LabId *id;
   float *s;
   LLink p;

   if (n>=99)
      HError(6570, "ExtendAux: Too many auxiliary fields in label file");
   
   oldn = ll->maxAuxLab; ll->maxAuxLab = n;
   for (p=ll->head->succ; p->succ!=NULL; p=p->succ){
      id = (LabId *)New(x,sizeof(LabId)*n) - 1; 
      s = (float *)New(x,sizeof(float)*n) - 1;
      for (i=1; i<=oldn; i++){
         id[i] = p->auxLab[i];
         s[i] = p->auxScore[i];
      }
      for (i=oldn+1; i<=n; i++){
         id[i] = NULL;
         s[i] = 0.0;
      }
      p->auxLab = id; p->auxScore = s;
   }
   if (trace&T_HTKL)
      printf("HLabel:     aux extended from %d to %d\n",oldn,n);
}

/* LoadHTKList: load a single HTK label list - dont create anything if 
                transAlt>0 and alt != transAlt */
static LabList * LoadHTKList(MemHeap *x, Source *src, int alt)
{
   LabList  *ll = NULL;
   LabId labid, auxLab[100];
   LLink p = NULL;
   HTime start,end;
   float score, auxScore[100];
   int n,maxAux = 0;
   Boolean ok;
   
   ok = (transAlt==0) || (transAlt == alt);
   if (ok) ll = CreateLabelList(x,maxAux);  /* assume no aux labels */
   if (trace&T_HTKL)
      printf("HLabel: looking for lab list\n");
   
   while (trSym==TRNUM || trSym==TRSTR){
      start = -1; end = -1; score = 0.0;
      if (trSym==TRNUM) {
         start = trNum; GetTrSym(src,TRUE);
         start *= htkLabelTimeScale;
         if (trSym==TRNUM) {
            end = trNum; GetTrSym(src,TRUE);
            end *= htkLabelTimeScale;
         }
      }
      if (trSym != TRSTR)
         HError(6550,"LoadHTKList: Label Name Expected");
      labid = GetLabId(trStr,TRUE);
      GetTrSym(src,TRUE);
      if (trSym==TRNUM){
         score = trNum;
         GetTrSym(src,TRUE);
      }
      if (trace&T_HTKL)
         printf("HLabel: adding %.0f %.0f %s %f\n",start,end,labid->name,score);
      if (ok) p = AddLabel(x,ll,labid,start,end,score);
      /* Any aux labels ? */
      n = 0;
      while (trSym != TREOL && trSym!=TREOF) {
         n++;
         if (trSym != TRSTR)
            HError(6550,"LoadHTKList: Aux Label Name Expected");
         auxLab[n] = GetLabId(trStr,TRUE);
         if (trace&T_HTKL)
            printf("HLabel:   adding aux lab %d = %s\n",n,auxLab[n]->name);
         GetTrSym(src,TRUE);
         if (trSym==TRNUM){
            auxScore[n] = trNum;
            if (trace&T_HTKL)
               printf("HLabel:   adding aux score %d = %f\n",n,trNum);
            GetTrSym(src,TRUE);
         } else
            auxScore[n] = 0.0;
      }
      if (ok && n>0) { /* need to add aux info */
         if (n>maxAux) {
            ExtendAux(x,ll,n);
            maxAux = n;
         } else while (n<maxAux) {
            ++n;
            auxLab[n] = NULL;
            auxScore[n] = 0.0;
         }
         AddAuxLab(p,n,auxLab,auxScore);
      }
      if (trSym!=TREOF)
         GetTrSym(src,TRUE);
   }
   return ll;
}

/* LoadHTKLabels: load a HTK transcription */
static void LoadHTKLabels(MemHeap *x, Transcription *t, Source *src)
{
   LabList *ll;
   int alt = 0;
   
   InitTrScan();
   GetTrSym(src,TRUE);
   if (trSym==TRNUM || trSym==TRSTR){
      ll = LoadHTKList(x,src,++alt);
      AddLabelList(ll,t);
      while (trSym == TRLEV){
         GetTrSym(src,TRUE);
         if (trSym != TREOL)
            HError(6550,"LoadHTKList: End of Line after /// Expected");       
         GetTrSym(src,TRUE);
         ll = LoadHTKList(x,src,++alt);
         AddLabelList(ll,t);
      }
   }
   while (trSym==TREOL) 
      GetTrSym(src,TRUE);
   if (trSym != TREOF)
      HError(6550,"LoadHTKLabels: Junk at end of HTK transcription");
}

/* --------------- TIMIT Label Format --------------------- */

/* LoadTIMITLabels: load a TIMIT transcription */
static void LoadTIMITLabels(MemHeap *x, Transcription *t, Source *src)
{
   LabList *ll;
   LabId labid;
   HTime start,end;
   float score;
   
   ll = CreateLabelList(x,0);  AddLabelList(ll,t);
   InitTrScan();
   GetTrSym(src,FALSE);
   while (trSym == TRNUM){    
      start = trNum*625;                 /* sample rate is 16KHz */
      GetTrSym(src,FALSE);
      if (trSym != TRNUM) 
         HError(6552,"LoadTIMITLabels: End Time expected in TIMIT Label File");
      end   = trNum*625;
      GetTrSym(src,FALSE);
      if (trSym != TRSTR) 
         HError(6552,"LoadTIMITLabels: Label Name expected in TIMIT Label File");
      labid = GetLabId(trStr,TRUE);
      score = 0.0;
      AddLabel(x,ll,labid,start,end,score);
      GetTrSym(src,FALSE);
      if (trSym == TREOL)
         GetTrSym(src,FALSE);
   }
}

/* --------------- ESPS Label Format --------------------- */

/* LoadESPSLabels: read waves label file */
static void LoadESPSLabels(MemHeap *x, Transcription *t, Source *src)
{
   LabList *ll;
   LabId labid;
   HTime start,end;
   float score;
   
   ll = CreateLabelList(x,0);  AddLabelList(ll,t);
   InitTrScan();
   GetTrSym(src,FALSE);
   while ( trStr[0] != '#' ) {
      GetTrSym(src,FALSE);
      if (trSym == TREOF) 
         HError(6553,"LoadESPSLabels: Unexpected EOF in ESPS Waves Label file.");
      if ( strcmp( "nfields", trStr) == 0 ) {
         GetTrSym(src,FALSE);
         if ( trSym != TRNUM )
            HError(6553,"LoadESPSLabels: Expecting field number");
         if ( trNum != 1 )
            HError(6553,"LoadESPSLabels: Can only read single field label files.");
      }
   }
   end = start = 0.0; score = 0.0;
   GetTrSym(src,FALSE);
   while ( trSym != TRNUM && trSym != TREOF )
      GetTrSym(src,FALSE);
   while ( trSym == TRNUM ) {    
      start = end;   /* Get time stamp */
      end = trNum * 1.0E7; 
      if ( start > end )
         HError(-6553,"LoadESPSLabels: time stamps out of order.");
      GetTrSym(src,FALSE);       /* Ignore color */
      GetTrSym(src,FALSE);       /* Get field label for current level */
      if ( trSym != TRSTR) 
         HError(6553,"LoadESPSLabels: Expecting label string in ESPS Waves Label file.");
      labid = GetLabId( trStr,TRUE); 
      AddLabel(x,ll,labid,start,end,score);  
      GetTrSym(src,FALSE);
      if ( trSym != TREOL )
         HError(6553,"LoadESPSLabels: End-of-line expected in ESPS Waves Label file.");
      GetTrSym(src,FALSE);
   }
}

/* --------------- SCRIBE Label Format --------------------- */

enum _ScribeLab {
   S_LBB, S_LBA, S_UTS, S_EOF
};
typedef enum _ScribeLab ScribeLab;

static char *scribeMap[] = {"LBB:", "LBA:", "UTS:"};

/* GetScribeLab: get next scribe label in input */
static ScribeLab GetScribeLab(Source *src)
{
   char *s;
   int i;
   
   do {
      do {
         GetTrSym(src,FALSE);
         if (trSym == TREOF) return S_EOF;
      } while (trSym != TRSTR);
      i = 0;
      do {
         s=scribeMap[i]; 
         if (strcmp(trStr,s) == 0) 
            return (ScribeLab) i;
         i++;
      } while (i < S_EOF);
   } while(TRUE);
   return ((ScribeLab) 0);
}

/* LoadSCRIBELabels: load a SCRIBE (SAM) label file - searches for
         first occurrence of a label symbol 
               LBA - acoustic label
               LBB - broad class label
               UTS - utterance 
         it loads this symbol and all subsequent labels of the
         same type.  All other SAM label types are ignored */
static void LoadSCRIBELabels(MemHeap *x, Transcription *t, Source *src)
{
   LabList *ll;
   LabId labid;
   HTime start,end;
   float score;
   ScribeLab ltype, lx;
   double sp;
   char buf[256];
   
   if (!GetConfFlt(cParm,numParm,"SOURCERATE",&sp))  
      sp = 500.0;   /* actual SCRIBE rate */
   ll = CreateLabelList(x,0);  AddLabelList(ll,t);
   InitTrScan();
   do {  /* search for first label */
      ltype = GetScribeLab(src);
      if (ltype == S_EOF)
         HError(6554,"LoadSCRIBELabels: Unexpected EOF");
   } while (ltype != S_LBB && ltype != S_LBA && ltype != S_UTS);
   do { /* load this and all subsequent ltype labels */
      GetTrSym(src,FALSE);
      if (trSym != TRNUM)
         HError(6554,"LoadSCRIBELabels: Start Index expected [%d]\n",trSym);
      start = trNum * sp;
      GetTrSym(src,FALSE);
      if (trSym != TRCOMMA)
         HError(6554,"LoadSCRIBELabels: Comma expected [%d]\n",trSym);
      GetTrSym(src,FALSE);
      if (ltype == S_LBA || ltype == S_LBB) {   /* LBB and LBA have a centre field */
         if (trSym != TRCOMMA)
            HError(6554,"LoadSCRIBELabels: Comma expected [%d]\n",trSym);     
         GetTrSym(src,FALSE);
      }
      if (trSym != TRNUM)
         HError(6554,"LoadSCRIBELabels: End Index expected [%d]\n",trSym);
      end = trNum * sp;
      GetTrSym(src,FALSE);
      if (trSym != TRCOMMA)
         HError(6554,"LoadSCRIBELabels: Comma expected [%d]\n",trSym);
      GetTrSym(src,FALSE);
      if (trSym != TRSTR)
         HError(6554,"LoadSCRIBELabels: Label expected [%d]\n",trSym);
      strcpy(buf,trStr);
      GetTrSym(src,FALSE);
      while (trSym == TRSTR){
         strcat(buf,"_"); strcat(buf,trStr);
         GetTrSym(src,FALSE);
      }
      labid = GetLabId(buf,TRUE); 
      score = 0.0;
      AddLabel(x,ll,labid,start,end,score); 
      if (trSym != TREOL)
         HError(6554,"LoadSCRIBELabels: End of Line expected [%d]\n",trSym);
      lx = GetScribeLab(src);
   } while (lx != S_EOF);
}

/* ----------------- TriPhone Stripping ------------------- */

/* EXPORT->TriStrip: Remove contexts of form A- and +B from s */
void TriStrip(char *s)
{
   char buf[100],*p;
   
   if ((p = strchr(s,'-')) == NULL) p = s; else ++p;
   strcpy(buf,p);
   if ((p = strrchr(buf,'+')) != NULL) 
      *p = '\0';
   strcpy(s,buf);
}

/* EXPORT->LTriStrip: enable triphone stripping */
void LTriStrip(Boolean enab)
{
   stripTriPhones = enab;
}

/* ------------------ Master Label File Handling -------------------- */

/* StoreMLFEntry: store the given MLF entry */
static void StoreMLFEntry(MLFEntry *e)
{
   e->next = NULL;
   if (mlfHead == NULL)
      mlfHead = mlfTail = e;
   else {
      mlfTail->next = e; mlfTail = e;
   }
   ++mlfUsed;
}

/* FindMLFStr: find the next quoted string in s */
static Boolean FindMLFStr(char *s, char **st, char **en)
{
   char *p,*q;
   
   if (s==NULL || *s == '\0') return FALSE;
   p = strchr(s,'"');
   if (p==NULL || *(p+1)=='\0') return FALSE;
   q = strchr(p+1,'"');
   if (q==NULL) return FALSE;
   *st = p; *en = q;
   return TRUE;
}

/* FindMLFType: find a -> or => symbol in s if any */
static MLFDefType FindMLFType(char *s, char **en)
{
   char *p;
   
   p=strchr(s+1,'>');
   if (p!=NULL) {
      *en = p;
      if (*(p-1) == '-') return MLF_SIMPLE;
      if (*(p-1) == '=') return MLF_FULL;
   }
   return MLF_IMMEDIATE;
}

/* NoMLFHeader: return true if s doesnt contain #!MLF!# */
static Boolean NoMLFHeader(char *s)
{
   int len;
   char *e;
   
   len = strlen(s);
   while (isspace((int) *s) && len>7) {
      --len; ++s;
   }
   e = s+len-1;
   while (isspace((int) *e) && len>7) {
      --len; --e;
   }
   if (len != 7) return TRUE;
   *(e+1) = '\0';
   return (strcmp(s,"#!MLF!#") != 0);
}

static Boolean incSpaces;

/* IsDotLine: return true if line contains only a single dot */
static Boolean IsDotLine(char *s)
{
   int len;
   char *e;
   Boolean cut;

   len = strlen(s);cut=FALSE;
   if (s[len-1]=='\n') {
      len--; s[len]=0;
      if ((len>0) && (s[len-1]=='\r')) {
         len--; s[len]=0;
      }
   }
   if (compatMode) {
      while (isspace((int) *s) && len>0) {
         cut=TRUE;
         --len; ++s;
      }
      e = s+len-1;
      if (*e=='\n' && len>0) --len,--e;
      while (isspace((int) *e) && len>0) {
         cut=TRUE;
         --len; --e;
      }
   }
   if ((*s == '.') && (len == 1)) {
      if (cut) incSpaces=TRUE;
      return(TRUE);
   }
   if (compatMode && cut && len==strlen_LEVELSEP && 
       strncmp(s,LEVELSEP,strlen_LEVELSEP)==0)
      incSpaces=TRUE;
   return(FALSE);
}

/* ClassifyMLFPattern: classify given pattern string */
MLFPatType ClassifyMLFPattern(char *s)
{
   char *t;
   
   if (strchr(s,'?') != NULL) return PAT_GENERAL;
   if (strchr(s,'*') == NULL) return PAT_FIXED;
   if (strlen(s)<=2) return PAT_GENERAL;
   if (s[0]!='*' || s[1]!=PATHCHAR) return PAT_GENERAL;
   t = s+2;
   if (strchr(t,'*') == NULL ) return PAT_ANYPATH;
   return PAT_GENERAL;
}

/* MLFHash: hash the given string */
static unsigned MLFHash(char *s)
{
   unsigned hashval;

   for (hashval=0; *s != '\0'; s++)
      hashval = *s + 31*hashval;
   return hashval;
}

/* EXPORT->LoadMasterFile: Load the Master Label File stored in fname 
                           and append the entries to the MLF table */
void LoadMasterFile(char *fname)
{
   char buf[1024];
   char *men;        /* end of mode indicator */
   char *pst,*pen;   /* start/end of pattern (inc quotes) */
   char *dst,*den;   /* start/end of subdirectory (inc quotes) */
   Boolean inEntry = FALSE;   /* ignore ".." within an entry */
   MLFEntry *e;
   FILE *f;
   
   if (numMLFs == MAXMLFS)
      HError(6520,"LoadMasterFile: MLF file limit reached [%d]",MAXMLFS);
   if ((f = fopen(fname,"rb")) == NULL)
      HError(6510,"LoadMasterFile: cannot open MLF %s",fname);
   if (fgets(buf,1024,f) == NULL)
      HError(6513,"LoadMasterFile: MLF file is empty");
   if (NoMLFHeader(buf))
      HError(6551,"LoadMasterFile: MLF file header is missing"); 
   incSpaces=FALSE;
   while (fgets(buf,1024,f) != NULL){
      if (!inEntry && FindMLFStr(buf,&pst,&pen)) {
         e = (MLFEntry *)New(&mlfHeap,sizeof(MLFEntry));
         e->type = FindMLFType(pen+1,&men);
         if (e->type == MLF_IMMEDIATE) {
            e->def.immed.fidx = numMLFs;
            e->def.immed.offset = ftell(f);
            if (e->def.immed.offset < 0)
               HError(6521,"LoadMasterFile: cant ftell on MLF file");
            inEntry = TRUE;
         } else {
            if (!FindMLFStr(men+1,&dst,&den))
               HError(6551,"LoadMasterFile: Missing subdir in MLF\n(%s)",buf);
            *den = '\0';
            e->def.subdir = NewString(&mlfHeap,den-dst-1);
            strcpy(e->def.subdir,dst+1);
         }
         *pen = '\0';         /* overwrite trailing pattern quote */
         ++pst;               /* skipover leading pattern quote */
         e->patType = ClassifyMLFPattern(pst);
         if (e->patType == PAT_ANYPATH) 
            pst += 2;         /* skipover leading "* /" */
         e->pattern = NewString(&mlfHeap,pen-pst);
         strcpy(e->pattern,pst);
         e->patHash = (e->patType==PAT_GENERAL)?0:MLFHash(e->pattern);
         StoreMLFEntry(e);
      } else
         if (inEntry && IsDotLine(buf)) inEntry = FALSE;
   }
   if (compatMode && incSpaces)
      HError(-6551,"LoadMasterFile: . or %s on line with spaces in %s",
             LEVELSEP,fname);
   mlfile[numMLFs++] = f;
}

/* EXPORT->NumMLFFiles: return number of loaded MLF files */
int NumMLFFiles(void)
{
   return numMLFs;
}

/* EXPORT->NumMLFEntries: return number of entries in the MLFTab */
int NumMLFEntries(void)
{
   return mlfUsed;
}

/* EXPORT->GetMLFFile: return fidx'th MLF file pointer */
FILE *GetMLFFile(int fidx)
{
   if (fidx<0 || fidx>=numMLFs)
      HError(6520,"GetMLFFile: fidx out of range[%d]",fidx);
   return mlfile[fidx];
}

/*EXPORT->IsMLFFile: return true if fn is an MLF */
Boolean IsMLFFile(char *fn)
{
   FILE *f;
   char buf[1024];
   
   if ((f = fopen(fn,"rb")) == NULL) return FALSE;
   if (fgets(buf,1024,f) == NULL) {
      fclose(f); return FALSE;
   }
   if (NoMLFHeader(buf)) {
      fclose(f); return FALSE;
   }
   fclose(f); return TRUE;
}

/* EXPORT->GetMLFEntry: Return the first entry in the MLF table */
MLFEntry *GetMLFTable(void)
{
   return mlfHead;
}

/* SplitPath: last name in path is removed and prefixed to name, then
              this new name is suffixed to subdir and stored in tryspec */            
static void SplitPath(char *path, char *name, char *subdir, char *tryspec)
{
   char buf1[1024],buf2[1024],*p;
   char pch[2] = " ";
   
   pch[0] = PATHCHAR;
   PathOf(path,buf1); NameOf(path,buf2);
   if (strlen(name)>0 ) strcat(buf2,pch);    /* new name */
   strcat(buf2,name); strcpy(name,buf2);
   p = buf1+strlen(buf1)-1;                  /* new path */
   if (*p == PATHCHAR) *p = '\0';
   strcpy(path,buf1);   
   strcpy(buf1,subdir);                      /* tryspec */
   p = buf1+strlen(buf1)-1;
   if (*p != PATHCHAR) strcat(buf1,pch);
   strcat(buf1,name);
   strcpy(tryspec,buf1);
}

/* OpenLabFile: opens a file corresponding to given fname, the file
                returned may be a real file or simply the MLF seek'ed
                to the start of an immediate file definition, isMLF
                tells you which it is.  Returns NULL if nothing found  */
static FILE * OpenLabFile(char *fname, Boolean *isMLF)
{
   FILE *f;
   MLFEntry *e;
   char path[1024],name[256],tryspec[1024];
   Boolean isMatch = FALSE;
   unsigned fixedHash;     /* hash value for PAT_FIXED */
   unsigned anypathHash;   /* hash value for PAT_ANYPATH */ 
   char *fnStart;          /* start of actual file name */
   static MLFEntry *q=NULL;/* entry after last one accessed - checked first */
   
   *isMLF = FALSE; 
   fixedHash = anypathHash = MLFHash(fname);
   fnStart = strrchr(fname,PATHCHAR);
   if (fnStart != NULL) {
      ++fnStart;
      anypathHash = MLFHash(fnStart);
   } else 
      fnStart = fname;
   if (trace&T_MLF)
      printf("HLabel: Searching for label file %s\n",fname);
   if (trace&T_MHASH) 
      printf("HLabel:  anypath hash = %d;  fixed hash = %d\n",anypathHash,fixedHash);
   for (e=(q==NULL?mlfHead:q); e != NULL; e = (e==NULL?mlfHead:e->next)) {
      switch (e->patType){
      case PAT_GENERAL:
         if (trace&T_MAT) 
            printf("HLabel:  general match against %s\n",e->pattern);
         isMatch = DoMatch(fname,e->pattern);
         break;
      case PAT_ANYPATH:
         if (trace&T_MAT) 
            printf("HLabel:  anypath match against %s[%d]\n",e->pattern,e->patHash);
         if (e->patHash == anypathHash)
            isMatch = strcmp(e->pattern,fnStart) == 0;
         else
            isMatch = FALSE;
         break;
      case PAT_FIXED:
         if (trace&T_MAT) 
            printf("HLabel:  fixed match against %s[%d]\n",e->pattern,e->patHash);
         if (e->patHash == fixedHash)
            isMatch = strcmp(e->pattern,fname) == 0;
         else
            isMatch = FALSE;
         break;
      }
      if ( isMatch ) {
         if (e->type == MLF_IMMEDIATE) {
            f = mlfile[e->def.immed.fidx];
            if (fseek(f,e->def.immed.offset,SEEK_SET) != 0)
               HError(6521,"OpenLabFile: cant seek to label def in MLF");
            *isMLF=TRUE;
            if (trace&T_MLF)
               printf("HLabel: Loading Immediate Def [Pattern %s]\n",
                      e->pattern);
            q=e->next;
            return f;
         } else {
            name[0] = '\0'; strcpy(path,fname);
            SplitPath(path,name,e->def.subdir,tryspec);
            if (trace&T_SUBD)
               printf("HLabel: trying %s\n",tryspec);
            f = fopen(tryspec,"rb");
            while (f==NULL && e->type == MLF_FULL && strlen(path)>0) {
               SplitPath(path,name,e->def.subdir,tryspec);
               if (trace&T_SUBD)
                  printf("HLabel: trying %s\n",tryspec);
               f = fopen(tryspec,"rb");
            }
            if (f != NULL) {
               if (trace&T_MLF)
                  printf("HLabel: Loading Label File %s [Pattern %s]\n",
                         tryspec,e->pattern);
               return f;
            }
         }
      }
      if (q!=NULL) e=NULL;
      q = NULL;
   }
   /* No MLF Match so try direct open */  
   if (trace&T_SUBD)
      printf("HLabel: trying actual file %s\n",fname);
   f = fopen(fname,"rb");
   if (f !=NULL && trace&T_MLF)
      printf("HLabel: Loading Actual Label File %s\n", fname);
   return f;
}

/* ------------------ Label File Opening/Closing -------------------- */

/* EXPORT->LOpen: Load transcription in fname and return it */
Transcription *LOpen(MemHeap *x, char * fname, FileFormat fmt)
{
   FILE *f;
   Source source;
   char buf[MAXSTRLEN];
   Transcription *t;
   Boolean isMLF;

   if (fmt == UNDEFF){
      if (GetConfStr(cParm,numParm,"SOURCELABEL",buf))
         fmt = Str2Format(buf);
      else
         fmt = HTK;
   }
   if ((f=OpenLabFile(fname, &isMLF)) == NULL)
      HError(6510,"LOpen: Unable to open label file %s",fname);
   AttachSource(f,&source);
   strcpy(source.name,fname);
   t = CreateTranscription(x);
   switch (fmt) {
   case TIMIT:    LoadTIMITLabels(x,t,&source); break;
   case HTK:      LoadHTKLabels(x,t,&source); break;
   case SCRIBE:   LoadSCRIBELabels(x,t,&source); break;
   case ESPS:     LoadESPSLabels(x,t,&source); break;
   default:
      HError(6572,"LOpen: Illegal label file format [%d]",fmt);
   }
   if (!isMLF) fclose(f);
   if (transLev > 0) FilterLevel(t,transLev-1);
   return t;
}
   
/* EXPORT->SaveToMasterfile: make all subsequent LSaves go to fname */
ReturnStatus SaveToMasterfile(char *fname)
{
   int i;
   LabId nid;
   OutMLFEntry *omlf;
   char buf[MAXSTRLEN];

   if (fname==NULL || *fname=='\0') {
      outMLF=NULL; 
      return (FAIL);
   }
   sprintf(buf,"#!MLF-%s!#",fname);
   if ((nid=GetLabId(buf,FALSE))!=NULL) {
      for (omlf=outMLFSet, i=0; i<numOutMLF; i++, omlf++)
         if (omlf->name==nid) break;
      if (i<numOutMLF) {
         if ((outMLF = omlf->file)==NULL){
            HRError(6511,"SaveToMasterfile: MLF file %s already closed",fname);
            return(FAIL);
         }
         return(SUCCESS);
      }
   }
   if (numOutMLF==MAXMLFS-1){
      HRError(6511,"SaveToMasterfile: Unable to create MLF file %s",fname);
      return(FAIL);
   }
   if ((outMLF=fopen(fname,"w")) == NULL){
      HRError(6511,"SaveToMasterfile: Unable to create MLF file %s",fname);
      return(FAIL);
   }
   fprintf(outMLF,"#!MLF!#\n");
   nid = GetLabId(buf,TRUE);
   outMLFSet[numOutMLF].file = outMLF;
   outMLFSet[numOutMLF].name = nid;
   numOutMLF++;
   return(SUCCESS);
}

/* EXPORT->CloseMLFSaveFile: Close the MLF output file */
void CloseMLFSaveFile(void)
{
   int i;
   OutMLFEntry *omlf;

   if (outMLF != NULL) {
      fclose(outMLF);
      for (omlf=outMLFSet,i=0; i<numOutMLF; i++,omlf++)
         if (omlf->file==outMLF) {
            omlf->file = NULL;
            break;
         }
      outMLF = NULL;
   }
}

/* SaveESPSLabels: Save transcription in f using ESPSwaves format */
static void SaveESPSLabels( FILE *f, Transcription *t)
{
   LabList *ll;
   LLink p;
   
   if ( t->numLists > 1 )
      HError(6572,"SaveESPSLabels: can't save multiple level transcription.");
   fprintf(f,"#\n");    /* Write waves label header */
   ll = t->head;
   for (p=ll->head->succ; p->succ != NULL; p=p->succ)
      fprintf(f,"%f 121 %s\n", (p->end+0.5)*1E-7, p->labid->name);
}

/* SaveHTKLabels: Save transcription in f using HTK format */
static void SaveHTKLabels( FILE *f, Transcription *t) 
{
   int i,j;
   LabList *ll;
   LLink p,hd;
   LabId id;
   Boolean hasScores[100];

   ll = t->head;
   for (i=1; i<=t->numLists; i++,ll=ll->next){
      hd = ll->head;
      /* See which columns have scores */
      for (j=0; j<=ll->maxAuxLab; j++)
         hasScores[j] = FALSE;
      for (p = hd->succ; p->succ != NULL; p = p->succ){
         if (p->score != 0.0)
            hasScores[0] = TRUE;
         for (j=1; j<=ll->maxAuxLab; j++)
            if (p->auxScore[j] != 0.0)
               hasScores[j] = TRUE;
      }
      for (p = hd->succ; p->succ != NULL; p = p->succ) {
         if (p->start>=0.0) {
            fprintf(f,"%.0f ",p->start / htkLabelTimeScale);
            if (p->end>=0.0) 
               fprintf(f,"%.0f ",p->end / htkLabelTimeScale);
         }
         WriteString(f,p->labid->name,labelQuote);
         if (hasScores[0])
            fprintf(f," %f",p->score);
         for (j=1; j<=ll->maxAuxLab; j++) {
            id = p->auxLab[j];
            if (id !=NULL){
               fputc(' ',f);
               WriteString(f,id->name,labelQuote);
               if (hasScores[j])
                  fprintf(f," %f",p->auxScore[j]);
            }
         }
         fprintf(f,"\n");
      }
      if (i<t->numLists)
         fprintf(f,"%s\n",LEVELSEP);
   }
}

/* EXPORT->LSave: Save transcription in fname */
ReturnStatus LSave(char *fname, Transcription *t, FileFormat fmt)
{
   FILE *f;
   char buf[MAXSTRLEN];

   if (fmt == UNDEFF){
      if (GetConfStr(cParm,numParm,"TARGETLABEL",buf))
         fmt = Str2Format(buf);
      else
         fmt = HTK;
   }
   if (outMLF != NULL) {
      if (fmt != HTK){
         HRError(6572,"LSave: cant save to MLF in %s format",Format2Str(fmt));
         return(FAIL);
      }
      f = outMLF;                      /* save to MLF file */
      fprintf(f,"\"%s\"\n",fname);
   } else                              /* else open new one */
      if ((f=fopen(fname,"wb")) == NULL){
         HRError(6511,"LSave: Unable to create label file %s",fname);
         return(FAIL);
      }
   if (trace&T_SAV)
      printf("HLabel: Saving transcription to %s in format %s\n",
             fname,Format2Str(fmt));
   switch (fmt) {
   case HTK:    SaveHTKLabels( f, t);   break;
   case ESPS:   SaveESPSLabels(f, t);   break;
   default: HRError(6572,"LSave: Illegal label file format."); 
      fclose(f);
      return(FAIL);
      break;
   }
   if (outMLF != NULL && fmt==HTK){
      fprintf(f,".\n");  fflush(f);
   }else
      fclose(f);
   return(SUCCESS);
}

/* ------------------------ End of HLabel.c ------------------------- */
