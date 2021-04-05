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
/*    File: HResults.c: gather statistics on results           */
/* ----------------------------------------------------------- */

char *hresults_version = "!HVER!HResults:   3.4.1 [CUED 12/03/09]";
char *hresults_vc_id = "$Id: HResults.c,v 1.1.1.1 2006/10/11 09:55:01 jal58 Exp $";

#include "HShell.h"
#include "HMem.h"
#include "HMath.h"
#include "HWave.h"
#include "HLabel.h"


/*
   This program reads in a set of HTK Label format files (output
   from a recognition tool) and compares them with the corresponding 
   transcription files.  The basic output is recognition statistics
   for the whole file set.  When the -w option is set then the DP 
   string matching above is replaced by the NIST word spotting algorithm.
*/

/* -------------------------- Trace Flags & Vars ------------------------ */

#define T_BAS     0001           /* basic progress reporting */
#define T_EVN     0002           /* show error rate for each of n lists */
#define T_SPK     0004           /* trace speaker matching */
#define T_NKY     0010           /* warn about non-keywords found */
#define T_SPT     0020           /* show detailed word spotting scores */
#define T_MEM     0040           /* trace memory usage */


static int trace = 0;               /* trace level */
static ConfParam *cParm[MAXGLOBS];   /* configuration parameters */
static int nParm = 0;               /* total num params */

/* ----------------------- HResults configuration ----------------------- */

static Boolean wSpot      = FALSE;    /* true if word spotting */

/* General options */
static int fileLimit = INT_MAX;       /* max num of label files to process */
static char * labDir    = NULL;       /* label file directory */
static char * labExt    = "lab";      /* label file extension */
static FileFormat rff   = UNDEFF;     /* ff of reference transcription files */
static FileFormat tff   = UNDEFF;     /* ff of test transcription files */
static char * nulName = "???";        /* name of null class */
static LabId nulClass;                /* Id of NULCLASS phone label */
static Boolean ignoreCase = FALSE;    /* true converts labels to upper case */
static Boolean stripContexts = FALSE; /* strip triphone contexts */

/* Word spotting only options */
static HTime totalDur = 0.0;          /* total duration of test input */
static float faTimeUnit = 1.0;        /* unit for measuring false alarms */
/* default is 1 hour and this gives NIST std of measuring over 0 to 10 FA/hr */

/* Rec only options */
static Boolean fullResults = FALSE;   /* enable full Results */
static Boolean outTrans   = FALSE;    /* enable transcription output */
static Boolean outPStats  = FALSE;    /* enable phoneme statistics */
static Boolean nistAlign = FALSE;     /* use NIST alignment & penalties */
static Boolean nistFormat = FALSE;    /* use NIST formatting */
static int maxNDepth=1;               /* find best of 1..max lists */
static char * spkrMask = NULL;        /* non-null report on per spkr basis */
static char * phraseStr = "SENT";     /* label for phrase level stats */
static char * phoneStr  = "WORD";     /* label for phone level stats */
static int maxWordLen = 5;

/* ---------------------- Global Variables ----------------------- */

MemHeap tempHeap;                     /* Stores data valid only for file */
MemHeap permHeap;                     /* Stores global stats */

static char *recfn;                   /* rec file name (test) */
static char labfn[255];               /* lab file name (reference) */

static int rlev=0;                    /* Label level to be used as ref */
static int tlev=0;                    /* Label level to be scored */
static LabList *ref,*test;            /* the labels being compared */
static Transcription *ans;            /* the full set of answers */

static char * refid=NULL;             /* identifiers for reference material */
static char recid[5][255];            /* upto 5 identifiers for */
static int recidUsed = 0;             /* number of test identifiers set */


/* ------------------ Process Command Line ------------------------- */

/* SetConfParms: set conf parms relevant to this tool */
void SetConfParms(void)
{
   char s[MAXSTRLEN];
   Boolean b;
   int i;

   nParm = GetConfig("HRESULTS", TRUE, cParm, MAXGLOBS);
   if (nParm>0){
      if (GetConfInt(cParm,nParm,"TRACE",&i)) trace = i;
      if (GetConfInt(cParm,nParm,"REFLEVEL",&i)) rlev = i;
      if (GetConfInt(cParm,nParm,"TESTLEVEL",&i)) tlev = i;
      if (GetConfBool(cParm,nParm,"STRIPCONTEXT",&b)) stripContexts = b;
      if (GetConfBool(cParm,nParm,"IGNORECASE",&b)) ignoreCase = b;
      if (GetConfBool(cParm,nParm,"NISTSCORE",&b))
         nistFormat = nistAlign = b;
      if (GetConfStr(cParm,nParm,"PHRASELABEL",s))
         phraseStr=CopyString(&permHeap,s);
      if (GetConfStr(cParm,nParm,"PHONELABEL",s))
         phoneStr=CopyString(&permHeap,s);
      if (GetConfStr(cParm,nParm,"SPEAKERMASK",s))
         spkrMask=CopyString(&permHeap,s);
      if (GetConfInt(cParm,nParm,"MAXWORDLEN",&i))
	 maxWordLen = i;
   }
}

void ReportUsage(void)
{
   printf("\nUSAGE: HResults [options] labelList recFiles...\n\n");
   printf(" Option                                       Default\n\n");
   printf(" -a s    Redefine string level label          SENT\n");
   printf(" -b s    Redefine unitlevel label             WORD\n");
   printf(" -c      Ignore case differences              off\n");
   printf(" -d N    Find best of N levels                1\n");
   printf(" -e s t  Label t is equivalent to s\n");
   printf(" -f      Enable full results                  off\n");
   printf(" -g fmt  Set test label format to fmt         HTK\n");
   printf(" -h      Enable NIST style formatting         off\n");
   printf(" -k s    Results per spkr using mask s        off\n");
   printf(" -m N    Process only the first N rec files   all\n");
   printf(" -n      Use NIST alignment procedure         off\n");
   printf(" -p      Output phoneme statistics            off\n");
   printf(" -s      Strip triphone contexts              off\n");
   printf(" -t      Output time aligned transcriptions   off\n");
   printf(" -u f    False alarm time units (hours)       1.0\n");
   printf(" -w      Enable word spotting analysis        off\n");
   printf(" -z s    Redefine null class name to s        ???\n");
   PrintStdOpts("GILX");
   printf("\n\n");
}

int main(int argc, char *argv[])
{
   char *s,*e,*c;
   int fidx,count=0;
   MLFEntry *me;
   Boolean tffSet = FALSE;      /* indicates rff set at command line */   

   void Initialise(char * listfn);
   void MatchFiles(void);
   void OutputStats(void);
   void AddEquiv(char * cl, char * eq);
   
   if(InitShell(argc,argv,hresults_version,hresults_vc_id)<SUCCESS)
      HError(3300,"HResults: InitShell failed");

   InitMem();   InitMath();
   InitWave();  InitLabel();

   if (!InfoPrinted() && NumArgs() == 0)
      ReportUsage();
   if (NumArgs() == 0) Exit(0);
   SetConfParms();

   CreateHeap(&permHeap, "permHeap", MSTAK, 1, 1.0, 4000, 20000);
   while (NextArg() == SWITCHARG) {
      s = GetSwtArg();
      if (strlen(s)!=1) 
         HError(3319,"HResults: Bad switch %s; must be single letter",s);
      switch(s[0]){
      case 'a':
         if (NextArg() != STRINGARG)
            HError(3319,"HResults: New PHRASE name expected");
         phraseStr = GetStrArg();
         break;
      case 'b':
         if (NextArg() != STRINGARG)
            HError(3319,"HResults: New PHONE name expected");
         phoneStr = GetStrArg();
         break;
      case 'c':
         ignoreCase = TRUE; break;
      case 'd': 
         maxNDepth = GetChkedInt(1,INT_MAX,s); break;
      case 'e':
         if (NextArg() != STRINGARG)
            HError(3319,"HResults: Eq Class Name Expected");
         c = GetStrArg();
         if (NextArg() != STRINGARG)
            HError(3319,"HResults: Eq Label Name Expected");
         e = GetStrArg();
         AddEquiv(c,e);
         break;
      case 'f':
         fullResults = TRUE; break;
      case 'g':
         if (NextArg() != STRINGARG)
            HError(3319,"HResults: Label File format expected");
         if((tff = Str2Format(GetStrArg())) == ALIEN)
            HError(-3389,"HResults: Warning ALIEN Label file format set");
         tffSet = TRUE;
         break;
      case 'h':
         nistFormat = TRUE; break;
      case 'j': 
         fileLimit = GetChkedInt(1,INT_MAX,s); break;
      case 'k':
         if (NextArg() != STRINGARG)
            HError(3319,"HResults: Speaker mask expected");
         spkrMask = GetStrArg();
         if (strchr(spkrMask,'%')==NULL)
            HError(3319,"HResults: Speaker mask missing speaker");
         break;
      case 'm': 
         fileLimit = GetChkedInt(1,INT_MAX,s); break;
      case 'n':
         nistAlign = TRUE; break;
      case 'p':
         outPStats = TRUE; break;
      case 's':
         stripContexts = TRUE; break;
      case 't':
         outTrans = TRUE; break;
      case 'u':
         faTimeUnit = GetChkedFlt(0.001, 100.0, s); break;
      case 'w':
         wSpot = TRUE; break;
      case 'z':
         if (NextArg() != STRINGARG)
            HError(3319,"HResults: New Null Class Name Expected");
         nulName = GetStrArg();
         break;
      case 'G':
         if (NextArg() != STRINGARG)
            HError(3319,"HResults: Label File format expected");
         if((rff = Str2Format(GetStrArg())) == ALIEN)
            HError(-3389,"HResults: Warning ALIEN Label file format set");
         if (!tffSet)
            tff = rff;
         break;
      case 'I':
         if (NextArg() != STRINGARG)
            HError(3319,"HResults: MLF file name expected");
         refid = GetStrArg();
         LoadMasterFile(refid);
         break;
      case 'L':
         if (NextArg()!=STRINGARG)
            HError(3319,"HResults: Label file directory expected");
         labDir = GetStrArg(); break;
      case 'T': 
         trace = GetChkedInt(0,077,s); break;
      case 'X':
         if (NextArg()!=STRINGARG)
            HError(3319,"HResults: Label file extension expected");
         labExt = GetStrArg(); break;
      default:
         HError(3319,"HResults: Unknown switch %s",s);
      }
   }
   if (wSpot && (outPStats || nistAlign || nistFormat 
                 || outTrans || spkrMask != NULL))
      HError(-3319,"HResults: Trying to use rec options in word spotting mode");
   if (NextArg() != STRINGARG)
      HError(3319,"HResults: Label Id List File expected");
   if (refid==NULL) refid = labDir;
   Initialise(GetStrArg());

   if (trace&T_MEM)
      PrintAllHeapStats();

   while (NumArgs()>0 && count<fileLimit){
      if (NextArg()!=STRINGARG)
         HError(3319,"HResults: recognition output file name expected");
      recfn = GetStrArg();
      if (recidUsed<5) strcpy(recid[recidUsed++],recfn);

      if (IsMLFFile(recfn)){
         fidx = NumMLFFiles();
         if ((me=GetMLFTable()) != NULL) {
            while(me->next != NULL) me=me->next;
            LoadMasterFile(recfn);
            me=me->next;
         }
         else{
            LoadMasterFile(recfn);
            me=GetMLFTable();
         }
         while (me != NULL) {
            if (me->type == MLF_IMMEDIATE && me->def.immed.fidx == fidx) {
               recfn = me->pattern;
               MatchFiles();
               ++count;
               if (count>=fileLimit) break;
            }
            me = me->next;
         }
      } 
      else {
         MatchFiles();
         ++count;
      }
   }
   if (count>=fileLimit)
      printf("\n** HResults terminated after %d files **\n\n",count);
   if (trace&T_MEM)
      PrintAllHeapStats();
   if (count>0)
      OutputStats();
   else 
      printf("No transcriptions found\n");

   Exit(0);
   return (0);          /* never reached -- make compiler happy */
}

/* -------------------- Label Equivalences ----------------------- */

typedef struct _Equiv Equiv;          /* list of equivalent labels */
struct _Equiv{
   LabId classId;
   LabId equivId;
   Equiv *next;
};

static Equiv *eqlist=NULL;            /* List of equivalent label ids */


/* AddEquiv: Add the equivalent pair (cl,eq) to eqlist */
void AddEquiv(char * cl, char * eq)
{
   Equiv *p;

   p=(Equiv*)New(&permHeap,sizeof(Equiv));
   p->classId = GetLabId(cl,TRUE);
   p->equivId = GetLabId(eq,TRUE);
   p->next = eqlist; eqlist = p;
}

/* NormaliseName: convert all equiv labels to class name and upper case if set */
void NormaliseName(LabList *ll,int lev)
{
   LabId cl,eq,id;
   LLink l;
   Equiv *p;
   int i,n,len;
   char buf[256],*ptr;

   n=CountAuxLabs(ll,lev);
   for (p=eqlist; p!=NULL; p=p->next) {
      cl = p->classId; eq = p->equivId;
      for (i=1;i<=n;i++) {
         l = GetAuxLabN(ll,i,lev);
         if (lev==0) {
            if (l->labid==eq) l->labid=cl;
         }
         else {
            if (l->auxLab[lev]==eq) l->auxLab[lev]=cl;
         }
      }
   }
   if (ignoreCase)
      for (i=1;i<=n;i++){
         l = GetAuxLabN(ll,i,lev);
         id = ((lev==0) ? l->labid : l->auxLab[lev]);
         strcpy(buf,id->name);
         len = strlen(buf);
         for (ptr=buf;*ptr!=0;ptr++)
            if (islower((int) *ptr)) break;
         if (*ptr){
            for (;*ptr!=0;ptr++)
               *ptr = toupper(*ptr);
            id = GetLabId(buf,TRUE);
            if (lev==0) l->labid=id;
            else l->auxLab[lev]=id;
         } 
      }
}

/* ------------------ Statistics Recording --------------------- */

typedef struct _Cell *CellPtr;

enum _Direction{DIAG,VERT,HOR,NIL};
typedef enum _Direction Direction;

typedef struct _Cell{           /* used in DP routines below */
   int ins,del,sub,hit;
   int score;
   Direction dir;
} Cell;

typedef struct _Spkr{           /* list of spkr records */
   int ins,del,sub,hit;
   int nsyms,nphr,phrcor;
   struct _Spkr *next;
   char *name;
} Spkr;

/* Global Stats */

static long ins   = 0;     /* Total insertions */
static long del   = 0;     /* Total deletions */
static long sub   = 0;     /* Total substitutions */
static long hits  = 0;     /* Total hits */
static long nsyms = 0;     /* Total symbols */
static long nphr  = 0;     /* Total phrases */
static long phrcor= 0;     /* Phrase correct */

static Spkr *spkrHead = NULL;
static int  numSpkrs  = 0;
static int  htkWidth  = 66;     /* width of output banners */
static int  spkrFails = 0;     /* num time spkr pattern fails to match */

/* SRMatch: recursively match s against pattern p, minplen
   is the min length string that can match p and
   numstars is the number of *'s in p 
           spkr is next character of the spkr name */
static Boolean SpRMatch(char *s,char *p,char *spkr,
                        int slen,int minplen,int numstars)
{
   Boolean match;
   
   if (slen==0 && minplen==0)
      match=TRUE;
   else if ((numstars==0 && minplen!=slen) || minplen>slen)
      match=FALSE;
   else if (*p == '*') {
      match=(SpRMatch(s+1,p,spkr,slen-1,minplen,numstars) ||
             SpRMatch(s,p+1,spkr,slen,minplen,numstars-1) ||
             SpRMatch(s+1,p+1,spkr,slen-1,minplen,numstars-1));
   }
   else if (*p == '%') {
      *spkr=*s,spkr[1]=0;
      match=SpRMatch(s+1,p+1,spkr+1,slen-1,minplen-1,numstars);
      if (!match) *spkr=0;
   }
   else if (*p == *s || *p == '?')
      match=SpRMatch(s+1,p+1,spkr,slen-1,minplen-1,numstars);
   else
      match=FALSE;
   
   return(match);
}

/* SpMatch: return spkr if s matches pattern p */
Boolean SpMatch(char *spkrpat, char *spkr, char *str)
{
   int spkrlen, slen, minplen, numstars;
   char *q,c;
  
   if (spkrpat == NULL || str==NULL) return(FALSE);
   slen = strlen(str);
   spkrlen = minplen = numstars = 0;
   q = spkrpat;
   while ((c=*q++)) {
      if (c == '*') ++numstars; else ++minplen;
      if (c == '%') ++spkrlen;
   }
   if (spkrlen>=MAXSTRLEN)
      HError(3390,"SpMatch: Speaker name too long %d vs %d",spkrlen,MAXSTRLEN);
   spkr[0]=0;
   if (SpRMatch(str,spkrpat,spkr,slen,minplen,numstars))
      return(TRUE);
   else {
      spkr[0]=0;
      return(FALSE);
   }
}

/* GetSpeaker: use spkrMask to extract spkr name from recfn */
Spkr *GetSpeaker(void)
{
   char name[MAXSTRLEN], buf[MAXSTRLEN];
   LabId id;
   Spkr *s, *t;
   Boolean found;

   strcpy(buf,recfn);
   if (trace&T_SPK)
      printf("Finding speaker for %s ",buf);
   found = SpMatch(spkrMask,name,buf);
   if (!found){
      ++spkrFails;
      return NULL;
   }
   if (trace&T_SPK)
      printf(":speaker is %s\n",name);
   id = GetLabId(name,TRUE);
   if (id->aux == 0) {
      s=(Spkr*)New(&permHeap,sizeof(Spkr));
      s->ins = s->del = s->sub = s->hit = 0;
      s->nsyms = s->nphr = s->phrcor = 0;
      s->name = id->name;  s->next = NULL;
      id->aux = (Ptr) s; 
      ++numSpkrs;
      if (spkrHead==NULL || strcmp(s->name,spkrHead->name)<0) {
         s->next=spkrHead;
         spkrHead=s;
      }
      else for (t=spkrHead;t!=NULL;t=t->next) 
         if (t->next == NULL || strcmp(s->name,t->next->name)<0) {
            s->next=t->next;
            t->next=s;
            break;
         }
   } 
   else
      s = (Spkr *) id->aux;
   return s;
}

/* RecordFileStats: record info from current input file */
Boolean RecordFileStats(CellPtr p)
{
   Boolean error;
   Spkr *s;

   del += p->del;    ins += p->ins; /* update global counters */
   sub += p->sub;    hits += p->hit;
   nsyms += p->hit+p->del+p->sub; ++nphr;
   error = !(p->del==0 && p->ins==0 && p->sub==0);
   if (!error) ++phrcor;
   if (spkrMask != NULL){       /* update speaker if reqd */
      s = GetSpeaker();
      if (s != NULL){ 
         s->del += p->del;  s->ins += p->ins;
         s->sub += p->sub;  s->hit += p->hit;
         s->nsyms += p->hit+p->del+p->sub;  ++s->nphr;
         if (!error) ++ s->phrcor;
      }
   }
   return error;
}

/* ---------------------- Print Routines  ------------------------ */

static Boolean headerPrinted = FALSE;  /* delay header as long as poss */
/* to accumulate rec file names */

/* PrintBar: print a bar of given char width with title if any */
void PrintBar(int offset, int width, char c, char *title)
{
   int i,k,tlen;
   
   tlen = strlen(title);
   if (tlen>0) tlen +=2;        /* allow for spaces */
   k = (width - tlen)/2;
   for (i=1; i<=offset; i++) putchar(' ');
   for (i=1; i<=k; i++) putchar(c);
   if (tlen>0) printf(" %s ",title);
   for (i=k+tlen; i<=width; i++) putchar(c);
   printf("\n");
}

/* PrintNBar: print a bar of given width and char, with given border chars */
void PrintNBar(int width, char c, char left, char right)
{
   int i;
   
   putchar(left);
   for (i=1; i<=width; i++) putchar(c);
   putchar(right);
   printf("\n");
}

/* PrintNMargin: print a margin for NIST format tables */
void PrintNMargin(void)
{
   int i;
   
   for (i=1; i<=4; i++) putchar(' ');
}

/* PClip: clip string to given length  */
void PClip(char * instr, char *outstr, int max)
{
   char *p;

   if (instr==NULL){
      outstr[0] = '\0'; return;
   }
   if (strlen(instr) <= max){
      strcpy(outstr,instr);
      return;
   }
   p = instr + strlen(instr)-max;
   strcpy(outstr,p);
   outstr[0] = '>';
}

/* PrintHeader: print title information */
void PrintHeader(void)
{
   char datestr[255];
   time_t clock = time(NULL);
   int i;
   char buf[100];

   strcpy(datestr,ctime(&clock));
   datestr[strlen(datestr)-1] = '\0';
   if (nistFormat){
      PrintNMargin(); PrintNBar(61,'-',',','.');
      PrintNMargin(); 
      printf("| HTK Results Analysis at %s",datestr);
      PrintNBar(35-strlen(datestr),' ',' ','|');
      PrintNMargin(); 
      PClip(refid,buf,54);
      printf("| Ref: %s",buf); PrintNBar(54-strlen(buf),' ',' ','|');
      PrintNMargin(); 
      PClip(recid[0],buf,54);
      printf("| Rec: %s",buf); PrintNBar(54-strlen(buf),' ',' ','|');
      for (i=1; i<recidUsed;i++){
         PrintNMargin(); 
         PClip(recid[i],buf,54);
         printf("|    : %s",buf); 
         PrintNBar(54-strlen(buf),' ',' ','|');
      }
      if (fullResults){
         PrintNMargin(); PrintNBar(61,'-','|','|');
         PrintNMargin(); 
         printf("|      FILE       |  Corr    Sub    Del    Ins    Err         |\n"); 
      }
   } else {
      PrintBar(0,htkWidth,'=',"HTK Results Analysis");
      printf("  Date: %s\n",datestr);
      PClip(refid,buf,70);
      printf("  Ref : %s\n",buf);
      PClip(recid[0],buf,70);
      printf("  Rec : %s\n",buf);
      for (i=1; i<recidUsed;i++){
         PClip(recid[i],buf,70);
         printf("      : %s\n",buf);
      }
   }
   headerPrinted = TRUE;
}

/* PrintFileStats:  output per file recognition statistics */
void PrintFileStats(char *fn, int h, int d, int s, int i)
{
   float accuracy,correct;
   float psub,pdel,pins,perr;
   int nc;
   char buf[100];
   
   if (!headerPrinted) {
      PrintHeader();
      if (!nistFormat) 
         PrintBar(0,htkWidth,'-',"File Results");
   }
   nc = h+d+s;
   accuracy = 100.0 * ((float) (h-i) / (float) nc);
   correct =  100.0 * ((float) h / (float) nc);
   if (nistFormat){
      psub = 100.0 * (float)s / (float)nc;
      pdel = 100.0 * (float)d / (float)nc;
      pins = 100.0 * (float)i / (float)nc;
      perr = 100.0 - accuracy;
      PrintNMargin(); PrintNBar(61,'-','|','|');
      PClip(fn,buf,15);
      PrintNMargin(); printf("| %15s |%7.2f%7.2f%7.2f%7.2f%7.2f        |\n",
                             buf, correct,psub,pdel,pins,perr);
   } else {
      printf("%s:  %6.2f(%6.2f)  [H=%4d, D=%3d, S=%3d, I=%3d, N=%3d]\n",
             fn, correct, accuracy, h,d,s,i,nc);
   }
}

void PrintSpkrStats(void)
{
   float accuracy,correct,phrcorrect;
   float psub,pdel,pins,perr;
   int nc;
   Spkr *s;
   char buf[32];

   if (!headerPrinted) PrintHeader();
   if (nistFormat){
      PrintNMargin(); PrintNBar(61,'-','|','|');
      PrintNMargin(); 
      printf("|    SPKR | # Snt |  Corr    Sub    Del    Ins    Err  S. Err |\n"); 
   }else{
      PrintBar(0,htkWidth,'-',"Speaker Results");
      printf("spkr: %%Corr( %%Acc )  [ Hits, Dels, Subs, Ins, #Words] %%S.Corr [ #Sent ]\n");
      PrintBar(0,htkWidth,'-',"");
   }
   for (s=spkrHead; s!=NULL; s=s->next){
      nc = s->hit + s->del + s->sub;
      accuracy = 100.0 * (float) (s->hit-s->ins) / (float) s->nsyms;
      correct = 100.0 * (float) s->hit / (float) s->nsyms;
      phrcorrect = 100.0 * ((float) s->phrcor / (float) s->nphr);
      if (nistFormat){
         psub = 100.0 * (float)s->sub / (float)nc;
         pdel = 100.0 * (float)s->del / (float)nc;
         pins = 100.0 * (float)s->ins / (float)nc;
         perr = 100.0 - accuracy;
         PrintNMargin(); PrintNBar(61,'-','|','|');
         PClip(s->name,buf,7);
         PrintNMargin(); printf("| %7s |",buf);
         printf("%5d  |%7.2f%7.2f%7.2f%7.2f%7.2f%7.2f |\n",
                s->nphr,correct,psub,pdel,pins,perr,100.0-phrcorrect);
      } else {
         PClip(s->name,buf,8);
         printf("%s: %6.2f(%6.2f)  [H=%4d, D=%3d, S=%3d, I=%3d, N=%4d]",
                buf, correct, accuracy, s->hit,s->del,s->sub,s->ins,nc);
         printf(" %6.2f [N=%3d]\n",
                phrcorrect,s->nphr);
      }
   }
}

/* PrintGlobalStats:  output global recognition statistics */
void PrintGlobalStats(void)
{
   float accuracy,correct,phrcorrect;
   float psub,pdel,pins,perr;
   void OutConMat(void);

   if (!headerPrinted) PrintHeader();
   accuracy = 100.0 * (float) (hits-ins) / (float) nsyms;
   correct = 100.0 * (float) hits / (float) nsyms;
   phrcorrect = 100.0 * ((float) phrcor / (float) nphr);
   if (nistFormat){
      if (spkrMask == NULL) {
         PrintNMargin(); PrintNBar(61,'=','|','|');
         PrintNMargin(); 
         printf("|           # Snt |  Corr    Sub    Del    Ins    Err  S. Err |\n"); 
         PrintNMargin(); PrintNBar(61,'-','|','|');
      } else{
         PrintSpkrStats();
         PrintNMargin(); PrintNBar(61,'=','|','|');
      }
      psub = 100.0 * (float)sub / (float)nsyms;
      pdel = 100.0 * (float)del / (float)nsyms;
      pins = 100.0 * (float)ins / (float)nsyms;
      perr = 100.0 - accuracy;
      PrintNMargin(); printf("| Sum/Avg |");
      printf("%5ld  |%7.2f%7.2f%7.2f%7.2f%7.2f%7.2f |\n",
             nphr,correct,psub,pdel,pins,perr,100.0-phrcorrect);
      PrintNMargin(); PrintNBar(61,'-','`','\'');
      if (outPStats) 
         OutConMat();
   } else {
      if (spkrMask != NULL) PrintSpkrStats();
      PrintBar(0,htkWidth,'-',"Overall Results");
      printf("%s: %%Correct=%.2f [H=%ld, S=%ld, N=%ld]\n",phraseStr,
             phrcorrect,phrcor,nphr-phrcor,nphr);
      printf("%s: %%Corr=%.2f, Acc=%.2f",phoneStr,correct,accuracy);
      printf(" [H=%ld, D=%ld, S=%ld, I=%ld, N=%ld]\n",hits,del,sub,ins,nsyms);
      if (outPStats) 
         OutConMat();
      PrintBar(0,htkWidth,'=',"");
   }
   if (spkrFails>0)
      printf("\n** Speaker Pattern '%s' Failed %d Times **\n\n",
             spkrMask,spkrFails);
}


/* ------------------------ DP Matching ------------------------ */

/* In the DP matching, the recognition output label file is 
   referred to as the test file, the annotated label file is
   referred to as the reference file. Test labels are indexed
   by i and ref labels are indexed by j.  Viewed as a grid,
   the test labels span the horizantal axis and the ref labels
   span the vertical.  The cell in column i and row j represents
   the match between the i'th test label and the j'th ref label
*/

static const int subPen = 10;     /* error penalties */
static const int delPen = 7;
static const int insPen = 7;

static const int subPenNIST = 4;  /* NIST error penalties */
static const int delPenNIST = 3;
static const int insPenNIST = 3;

static CellPtr *grid;     /* matrix of cells */
static LabId *lRef,*lTest;
static int nRef,nTest;

/* DumpGrid: for debugging */
void DumpGrid(void)
{
   int i,j;

   printf("Grid -\n");
   for (j=0; j<=nRef; j++)
      if (j==0) 
         printf("%10s","");
      else
         printf("%5.4s",lRef[j]->name);
   printf("\n");
   
   for (i=0; i<=nTest; i++) {
      if (0==i) 
         printf("%5s","");
      else
         printf("%5.4s",lTest[i]->name);
      for (j=0; j<=nRef; j++) {
         printf("%4d",grid[i][j].score);
         if (grid[i][j].dir==DIAG)
            printf("d");
         else if (grid[i][j].dir==HOR)
            printf("h");
         else if (grid[i][j].dir==VERT)
            printf("v");
         else 
            printf("n");
      }
      printf("\n");
   }
   fflush(stdout);
}

/* CreateGrid: Create a grid of cells of the given dimensions */
void CreateGrid(void)
{
   LLink l;
   LabId labid;
   int i;
   
   nRef = CountAuxLabs(ref,rlev);
   nTest = CountAuxLabs(test,tlev);
   grid=(CellPtr *)New(&tempHeap,(nTest+1)*sizeof(CellPtr));
   for (i=0; i<=nTest;i++)
      grid[i]=(CellPtr) New(&tempHeap,(nRef+1)*sizeof(Cell));
   lRef = (LabId*) New(&tempHeap,sizeof(LabId)*nRef);lRef--;
   lTest = (LabId*) New(&tempHeap,sizeof(LabId)*nTest);lTest--;

   grid[0][0].score = grid[0][0].ins = grid[0][0].del = 0;
   grid[0][0].sub = grid[0][0].hit = 0;
   grid[0][0].dir = NIL;
   for (i=1;i<=nTest;i++) {
      l=GetAuxLabN(test,i,tlev);
      labid = ((tlev==0) ? l->labid : l->auxLab[tlev]);
      lTest[i]=labid;
      grid[i][0] = grid[i-1][0];
      grid[i][0].dir = HOR;
      if (labid != nulClass) {
         grid[i][0].score += nistAlign ? insPenNIST : insPen;
         ++grid[i][0].ins;
      }
   }
   for (i=1;i<=nRef;i++) {
      l=GetAuxLabN(ref,i,rlev);
      labid = ((rlev==0) ? l->labid : l->auxLab[rlev]);
      lRef[i]=labid;
      grid[0][i] = grid[0][i-1];
      grid[0][i].dir = VERT;
      if (labid != nulClass) {
         grid[0][i].score += nistAlign ? delPenNIST : delPen;
         ++grid[0][i].del;
      }
   }
}

/* FreeGrid: free storage allocated to grid and label arrays */
void FreeGrid(void)
{
   Dispose(&tempHeap,grid);
}

/* DoCompare: fill the grid */
void DoCompare(void)
{
   CellPtr gridi,gridi1;
   int h,d,v,i,j;
   Boolean refnull,testnull;

   for (i=1;i<=nTest;i++){
      gridi = grid[i]; gridi1 = grid[i-1];
      testnull = (lTest[i] == nulClass);
      for (j=1;j<=nRef;j++) {
         refnull = (lRef[j] == nulClass);
         if (refnull && testnull) { /* both ref and test are null */
            h = gridi1[j].score; 
            d = gridi1[j-1].score; 
            v = gridi[j-1].score;
            if (d<=v && d<=h) {
               gridi[j] = gridi1[j-1]; 
               gridi[j].dir = DIAG;
            }
            else if (h<v) {
               gridi[j] = gridi1[j]; 
               gridi[j].dir = HOR;
            }
            else {
               gridi[j] = gridi[j-1];
               gridi[j].dir = VERT;
            }
         }
         else if (refnull) {    /* ref is null */
            gridi[j] = gridi[j-1]; 
            gridi[j].dir = VERT;
         }
         else if (testnull) {   /* test is null */
            gridi[j] = gridi1[j];
            gridi[j].dir = HOR;
         }
         else {                 /* normal case */
            h = gridi1[j].score +insPen;
            d = gridi1[j-1].score;
            if (lRef[j] != lTest[i])
               d += subPen;
            v = gridi[j-1].score + delPen;
            if (d<=h && d<=v) { /* DIAG = hit or sub */
               gridi[j] = gridi1[j-1];
               gridi[j].score = d;
               gridi[j].dir = DIAG;
               if (lRef[j] == lTest[i])
                  ++gridi[j].hit;
               else
                  ++gridi[j].sub;
            }
            else if (h<v) {     /* HOR = ins */
               gridi[j] = gridi1[j];
               gridi[j].score = h;
               gridi[j].dir = HOR;
               ++ gridi[j].ins;
            }
            else {              /* VERT = del */
               gridi[j] = gridi[j-1];
               gridi[j].score = v;
               gridi[j].dir = VERT;
               ++gridi[j].del;
            }
         }
      } /* for j */
   } /* for i */
   grid[0][0].dir = NIL;
}

/* DoCompareNIST: fill the grid using NIST alignment rules*/
void DoCompareNIST(void)
{
   CellPtr gridi,gridi1;
   int h,d,v,i,j;
   Boolean refnull,testnull;

   for (i=1;i<=nTest;i++){
      gridi = grid[i]; gridi1 = grid[i-1];
      testnull = (lTest[i] == nulClass);
      for (j=1;j<=nRef;j++) {
         refnull = (lRef[j] == nulClass);
         if (refnull && testnull) { /* both ref and test are null */
            h = gridi1[j].score; 
            d = gridi1[j-1].score; 
            v = gridi[j-1].score;
            if (v <= d && v <= h) {    
               gridi[j] = gridi[j-1];
               gridi[j].dir = VERT;
            }
            else if (d <= h) {
               gridi[j] = gridi1[j-1]; 
               gridi[j].dir = DIAG;
            }
            else {
               gridi[j] = gridi1[j]; 
               gridi[j].dir = HOR;
            }
         }
         else if (refnull) {    /* ref is null */
            gridi[j] = gridi[j-1]; 
            gridi[j].dir = VERT;
         }
         else if (testnull) {   /* test is null */
            gridi[j] = gridi1[j];
            gridi[j].dir = HOR;
         }
         else {                 /* normal case */
            h = gridi1[j].score +insPenNIST;
            d = gridi1[j-1].score;
            if (lRef[j] != lTest[i])
               d += subPenNIST;
            v = gridi[j-1].score + delPenNIST;
            if (v <= d && v <= h) { /* VERT = del */
               gridi[j] = gridi[j-1];
               gridi[j].score = v;
               gridi[j].dir = VERT;
               ++gridi[j].del;
            }
            else if (d <= h) {  /* DIAG = hit or sub */
               gridi[j] = gridi1[j-1];
               gridi[j].score = d;
               gridi[j].dir = DIAG;
               if (lRef[j] == lTest[i])
                  ++gridi[j].hit;
               else
                  ++gridi[j].sub;
            }
            else {              /* HOR = ins */
               gridi[j] = gridi1[j];
               gridi[j].score = h;
               gridi[j].dir = HOR;
               ++ gridi[j].ins;
            }
         }
      } /* for j */
   } /* for i */
   grid[0][0].dir = NIL;
}

/* ------------------- Aligned Transcriptions --------------- */

/* AppendItem: appends item padded to width spaces to s */
void AppendItem(char *s, char *item, int len, int width)
{
   char buf[64];
   int i;

   for (i=0; i<len; i++)
      buf[i] = item[i];
   for (i=len; i<width; i++)
      buf[i] = ' ';
   buf[width] = '\0';
   strcat(s,buf);
}

/* AppendPair: append a to linea and b to lineb */
void AppendPair(char *linea, char *a, char *lineb, char *b)
{
   int lena,lenb,width;
   
   lena = strlen(a); lenb = strlen(b);
   width = ((lena>=lenb) ? lena : lenb)+1;
   AppendItem(linea,a,lena,width);
   AppendItem(lineb,b,lenb,width);
}

/* AppendCell: path upto grid[i][j] to tb and rb (recursive) */
void AppendCell(int i, int j, char *tb, char *rb)
{
   char *rlab,*tlab;
   LabId rid=NULL,tid=NULL;
   char empty[1];

   if (i<0 || j<0) 
      HError(3391,"AppendCell: Trace back failure");
   empty[0] = '\0'; rlab = tlab = empty;
   switch (grid[i][j].dir) {
   case DIAG:
      tid  = lTest[i]; tlab = tid->name;
      rid  = lRef[j]; rlab = rid->name;
      AppendCell(i-1,j-1,tb,rb); break;
   case HOR:
      tid  = lTest[i]; tlab = tid->name;
      rid = NULL; rlab = empty;
      AppendCell(i-1,j,tb,rb); break;
   case VERT:
      tid = NULL; tlab = empty;
      rid  = lRef[j]; rlab = rid->name;
      AppendCell(i,j-1,tb,rb); break;
   case NIL:
      return;
   }
   if (tid != nulClass && rid != nulClass)
      AppendPair(rb,rlab,tb,tlab);
}

/* OutTrans: output aligned transcriptions using best path in grid */
void OutTrans(void)
{
   char refBuf[4096];           /* no checking of output length so */
   char testBuf[4096];          /* these are generous sizes */
   
   strcpy(refBuf," LAB: ");
   strcpy(testBuf," REC: ");
   AppendCell(nTest,nRef,testBuf,refBuf);
   printf("Aligned transcription: %s vs %s\n", labfn, recfn);
   printf("%s\n",refBuf);
   printf("%s\n",testBuf);
   fflush(stdout);
}

/* ----------------- HMMList handling ----------- */

static int nLabs;
static LabId *names;

Boolean ReadWordFromLine(Source *src, char *s)
{
   char *p;
   int ch;

   p=s;
   if (s!=NULL) *p=0;
   SkipWhiteSpace(src);
   while ((ch=GetCh(src))!=EOF) {
      if (isspace(ch)) break;
      if (s!=NULL) *p++=ch;
   }
   if (s!=NULL) *p=0;
   UnGetCh(ch,src);
   SkipLine(src);
   if (ch==-1 && s==p)
      return(FALSE);
   else
      return(TRUE);
}

/* ReadHMMList: Read in List of Labels from file fn */
void ReadHMMList(char *fn)
{
   Source source;
   LabId labid;
   char buf[MAXSTRLEN];
   int i;

   /* Once to find out how many lines */
   if(InitSource(fn,&source,HMMListFilter)<SUCCESS)
      HError(3310,"ReadHMMList: Can't open file %s", fn);
   i=0;
   while(ReadWordFromLine(&source,NULL)) i++;
   CloseSource(&source);

   /* Now do it for real */
   nLabs=i;
   names=(LabId*)New(&permHeap,sizeof(LabId)*nLabs);
   --names;

   if(InitSource(fn,&source,HMMListFilter)<SUCCESS)
      HError(3310,"ReadHMMList: Can't open file %s", fn);
   for (i=1;i<=nLabs;i++) {
      ReadWordFromLine(&source,buf);
      labid=GetLabId(buf,TRUE);
      names[i]=labid;
      labid->aux = (Ptr)i;
   }
   CloseSource(&source);
}

int Index(LabId labid)
{
   int i;
   
   i=(int)labid->aux;
   if (wSpot && i==0) return(0);
   if (i<1 || i>nLabs || names[i]!=labid)
      HError(3331,"Index: Label %s not in list[%d of %d]",
             labid->name,i,nLabs);
   return(i);
}

/* ----------------- Confusion Matrix (Phoneme Stats) ----------- */

#define MAXCONMATSIZE 200

static ShortVec *conMat;  /* confusion matrix, conMat[i][j] is the number of
                             times label i was recognised as label j */
static ShortVec conDel,conIns; /* corresponding deletion and insertion counts */

/* InitConMat:  allocate and initialise confusion matrix */
void InitConMat(void)
{
   int i;

   if (nLabs>MAXCONMATSIZE)
      HError(3332,"InitConMat: Confusion matrix would be too large");
   conMat = (ShortVec *) New(&permHeap, nLabs*sizeof(ShortVec));
   --conMat;                    /* index is 1..nLabs */
   for (i=1;i<=nLabs;i++){
      conMat[i]=CreateShortVec(&permHeap,nLabs);
      ZeroShortVec(conMat[i]);
   }  
   conDel=CreateShortVec(&permHeap,nLabs);
   ZeroShortVec(conDel);
   conIns=CreateShortVec(&permHeap,nLabs);
   ZeroShortVec(conIns);
}

/* OutConMat: output the confusion matrix */
void OutConMat(void)
{
   Boolean *seen;
   int i,j,k,err,rowerr,maxlen;
   char *s,c,buf[64];
   float correct, errprop;

   seen=(Boolean*)New(&tempHeap,sizeof(Boolean)*nLabs);
   seen--;
   maxlen = 0;
   for (i=1;i<=nLabs;i++) {
      k = strlen(names[i]->name);
      if (k > maxlen) maxlen = k;
   }
   if (maxlen>maxWordLen) maxlen = maxWordLen;
   PrintBar(0,htkWidth,'-',"Confusion Matrix");
   for (j=1; j<=nLabs; j++) {
      for (i=1,k=conIns[j];i<=nLabs;i++) k+=conMat[i][j];
      if (k==0) seen[j]=FALSE;
      else seen[j]=TRUE;
   }
   for (j=0;j<maxlen;j++) {
      for (k=0; k<=4 ; k++) printf(" ");
      for (i=1;i<=nLabs;i++) {
         if (!seen[i]) continue;
         s = names[i]->name;
         c = (j<strlen(s))?s[j]:' ';
         printf("  %c ",c);
      }
      if (j==maxlen-1)
         printf(" Del [ %%c / %%e]");
      printf("\n");
   }
   for (i=1;i<=nLabs;i++){
      for (j=1,k=conDel[i];j<=nLabs;j++) k+=conMat[i][j];
      if (k==0) continue;

      strcpy(buf,names[i]->name); buf[4] = '\0';
      printf("%4s ",buf);
      rowerr = 0;
      for (j=1; j<=nLabs; j++){
         if (!seen[j]) continue;
         err = conMat[i][j];
         if (i!=j) rowerr += err;
         if (err<100)
            printf(" %2d ",err);
         else
            printf("%4d",err);
      }
      printf("%4d",conDel[i]);
      if (rowerr>0) {
         correct = 100.0*(float)conMat[i][i]/(float)(conMat[i][i]+rowerr);
         errprop = 100.0*(float)rowerr/(float)nsyms;
         printf(" [%4.1f/%3.1f]\n",correct,errprop);
      } else
         printf("\n");
   }
   printf("Ins ");
   for (j=1; j<=nLabs; j++) {
      if (!seen[j]) continue;
      printf("%4d",conIns[j]);
   }
   printf("\n");
   seen++;
   Dispose(&tempHeap,seen);
}     

/* CollectStats: trace back from grid[i][j] collecting phoneme stats */
void CollectStats(int i,int j)
{
   int ri,ti;
   LabId rlab,tlab;

   do {
      switch(grid[i][j].dir) {
      case NIL:   
         return;
      case DIAG:  
         rlab = lRef[j--];
         tlab = lTest[i--];
         if (rlab==nulClass || tlab==nulClass) 
            break;
         ri=Index(rlab);
         ti=Index(tlab);
         ++conMat[ri][ti];
         break;
      case VERT:
         rlab = lRef[j--];
         if (rlab==nulClass) 
            break;
         ri=Index(rlab);
         ++conDel[ri];
         break;
      case HOR:
         tlab = lTest[i--];
         if (tlab==nulClass)  
            break;
         ti=Index(tlab);
         ++conIns[ti];
         break;
      }
   } while (!(i==0 && j==0));
}

/* ----------------  Recognition Match Routines ---------------- */

/* MatchRecFiles: match sequence in test vs sequence in ref */
void MatchRecFiles(void)
{
   Cell bp,*p;
   int i,n,err,berr,best;
   char buf[255];
   
   n=(ans->numLists>maxNDepth)?maxNDepth:ans->numLists;
   best=0;berr=INT_MAX;
   for (i=1;i<=n;i++) {
      test=GetLabelList(ans,i);
      if (test->head->succ == test->tail) {
         HError(-3330,"MatchRecFiles: Test Output List %s(%d) is Empty",recfn,i);
         break;
      }
      NormaliseName(test,tlev);
      CreateGrid();
      if (nistAlign)
         DoCompareNIST();
      else 
         DoCompare();
      p = &grid[nTest][nRef];
      err = p->del+p->sub+p->ins;
      if (best==0 || err < berr) {
         berr = err; best=i;
         bp = *p;
      }
      FreeGrid();  /* Actually frees much more */
      if (trace & T_EVN) {
         if (i == 1) printf("%s:",NameOf(recfn,buf));
         printf(" %2d",err);fflush(stdout);
      }
   }
   if (best==0) return; /* Empty test labels */

   if (trace & T_EVN) printf("\n"),fflush(stdout);

   err = RecordFileStats(&bp);
   if (fullResults) 
      PrintFileStats(NameOf(recfn,buf),bp.hit,bp.del,bp.sub,bp.ins);

   if ((outTrans && err) || outPStats) {
      test=GetLabelList(ans,best);
      CreateGrid();
      if (nistAlign)
         DoCompareNIST();
      else 
         DoCompare();
      if (outTrans && err) 
         OutTrans();
      if  (outPStats) 
         CollectStats(nTest,nRef);
      FreeGrid();  /* Actually frees much more */
   }
}

/* ------------------ Word Spot Recording --------------------- */

/* Linked list of keyword spots, these are chained to the labels
   in descending score order - nasty but convenient */
typedef struct _SpotRec{
   float score;                 /* score for this hit */
   Boolean hit;                 /* true if real occurrence */
   struct _SpotRec *next;
} SpotRec;

typedef int FAVec[10];

static int spotWidth = 66;
static SpotRec **spots;
static int nonKeys=0;         /* count of nonKeyWords encountered */
static int *keyOccs;          /* array[1..kn] of actual keyword occurrences */
static int fomN;              /* num FA levels needed */
static float fomA;            /* interpolation factor */
static Matrix fomTab;         /* array[1..kn+1,1..fomN+1] of hitRate */
/* kn = #keywords, fomN = int >10T-0.5, T = total duration in hours */
/* fomA = 10T - fomN, kn+1'th key index corresponds to sum over all keys */

/* InitSpotLists: initialise the spot lists.  Every entry has
   a sentinel record attached to avoid the 'empty list' case. */
void InitSpotLists(void)
{
   SpotRec *p;
   int i;
   
   keyOccs=(int *)New(&permHeap,nLabs*sizeof(int));
   keyOccs--;
   spots=(SpotRec**)New(&permHeap,sizeof(SpotRec*)*nLabs);
   spots--;
   for (i=1; i<=nLabs; i++){
      p = (SpotRec *) New(&permHeap,sizeof(SpotRec));
      p->score = LZERO; p->hit = FALSE; p->next = NULL;
      keyOccs[i] = 0;
      spots[i] = p;
   }
}

/* AddSpot: insert given keyword spot into list */
void AddSpot(LabId key, Boolean hit, float score)
{
   SpotRec *newRec,*p,*q;
   int i;
   
   if (score <LZERO)
      HError(3333,"AddSpot: score %f too small to store",score);
   i = Index(key);
   if (i==0){
      ++nonKeys;
      if (trace&T_NKY) {
         printf(" AddSpot: non keyword %s in test file\n",key->name);
         fflush(stdout);
      }
      return;
   }
   newRec = (SpotRec *) New(&permHeap,sizeof(SpotRec));
   newRec->score = score; newRec->hit = hit;
   p = spots[i]; q = NULL;
   while (score < p->score) {
      q = p; p = p->next;
   }
   newRec->next = p;
   if (q==NULL)
      spots[i] = newRec;
   else
      q->next = newRec;
}

/* PrintSpotList: print the entries in the given spot list */
void PrintSpotList(SpotRec *p)
{
   while (p->next != NULL){
      printf("%12f %s\n",p->score,p->hit?"":"FA");
      p = p->next;
   }
}

/* PrintKeySpots: print all recorded key spots */
void PrintKeySpots(void)
{
   SpotRec *p;
   int i;

   printf("Spot Records\n");
   for (i=1; i<=nLabs; i++){
      printf("Key Spot list for %s [%d actual occurrences]\n",
             names[i]->name,keyOccs[i]);
      p = spots[i]; 
      PrintSpotList(p);
      printf("\n");
   }
   fflush(stdout);
}

/* IsHit: true if test label occurs in ref label list */
Boolean IsHit(LLink t)
{
   LLink l;
   HTime mid;
   int i,n;
   
   n=CountAuxLabs(ref,rlev);
   for (i=1; i<=n;  i++) {
      l = GetAuxLabN(ref,i,rlev);
      mid = (l->start + AuxLabEndTime(l,rlev))/2.0;
      if (t->labid == l->labid && 
          (t->start <= mid && t->end >= mid)){
         return TRUE;
      }
   }
   return FALSE;
}

/* CreateFOMTab: calculate size of fomTab and create it */
void CreateFOMTab(void)
{
   float T;
   int kn;
   
   kn = nLabs;
   T = totalDur / (1.0E7*faTimeUnit*3600.0);
   fomN = (int) (10.0*T + 0.49999);
   fomA = 10.0*T - (float)fomN;
   fomTab = CreateMatrix(&permHeap,kn+1,fomN+1);
   ZeroMatrix(fomTab);
   if (trace&T_SPT)
      printf("FOM Table:  kn=%d, fomN=%d, fomA=%f\n",kn,fomN,fomA);
}  

/* CalcKeyFOM: Calculate the fomTab entries for given keyword */
void CalcKeyFOM(int idx)
{
   int count=0,faSeen=0;
   float nOccs;
   SpotRec *p;
   
   nOccs = (float) keyOccs[idx];
   p = spots[idx];
   while (p->next != NULL && faSeen<fomN+1){
      if (p->hit)
         ++count;
      else {
         ++faSeen;
         fomTab[idx][faSeen] = (nOccs>0.0) ? (float) count / nOccs : 0.0;
      }
      p = p->next;
   }
   while (faSeen<fomN+1)
      fomTab[idx][++faSeen] = (nOccs>0.0) ? (float) count / nOccs : 0.0;
}

/* CalcGlobalFOM: Calculate the global fomTab entries */
void CalcGlobalFOM(void)
{
   int kn1,i,k,tOcc;
   float sum;
   
   kn1 = nLabs+1;
   
   for (i=1; i<=fomN+1; i++){
      sum = 0.0; tOcc = 0;
      for (k=1; k<kn1; k++)
         if (keyOccs[k]>0){
            sum += fomTab[k][i] * keyOccs[k];
            tOcc += keyOccs[k];
         }
      if (tOcc==0)
         HError(3333,"CalcGlobalFOM: zero total occurrences for fa %d",i);
      fomTab[kn1][i] = sum/(float)tOcc;
   }
}

/* GetFOM: calculate FOM for given index */
float GetFOM(int idx)
{
   int i;
   float tenT,sum;
   
   tenT = totalDur / (1.0E6*faTimeUnit*3600.0);
   sum = 0.0;
   for (i=1; i<=fomN; i++)
      sum += fomTab[idx][i];
   sum += fomTab[idx][fomN+1] * fomA;
   return (sum / tenT) * 100;
}

/* GetHitRate: calculate % hit rate at given FA/hour */
float GetHitRate(int idx, int FAperHour)
{
   float xN,a,y1,y2,hitr;
   int n1,n2;
   
   xN = ((float)FAperHour * totalDur) / (1.0E7*faTimeUnit*3600.0) + 0.5;
   n1 = (int) xN;
   n2 = n1 + 1;
   if (n2>fomN+1)
      HError(3333,"GetHitRate: n2 [%d] > fomN+1 [%d+1]",n2,fomN+1);
   a = xN - (float) n1;
   if (n1>0)
      y1 = fomTab[idx][n1];
   else{                        /* need to interpolate */
      if (n2<fomN+1)
         y1 = fomTab[idx][n2]*2.0 - fomTab[idx][n2+1];
      else
         y1 = fomTab[idx][n2];
   }
   y2 = fomTab[idx][n2];
   hitr = (y2*a + y1*(1.0-a)) * 100.0;
   return hitr>0.0?hitr:0.0;
}

/* CountHits: count hits/FAs for given word */
void CountHits(int idx, int *nh, int *nf)
{
   SpotRec *p;
   int h=0,f=0;
   
   p = spots[idx];
   while (p->next != NULL) {
      if (p->hit) ++h; else ++f;
      p = p->next;
   }
   *nh = h; *nf = f;
}

/* ------------------ Word Spot Matching Routines ----------------- */

/* MatchSpotFiles: record word spots in test using reference in ref */
void MatchSpotFiles(void)
{
   int i,j,nr,nt;
   Label t,*l;
   HTime t1,t2;

   test=GetLabelList(ans,1);
   NormaliseName(test,tlev);
   if (test->head->succ == test->tail) {
      HError(-3330,"MatchSpotFile: Test Output File %s is Empty",recfn);
      return;
   }
   
   /* increment total duration - note that this assumes that the
      end of the last ref label or test label whichever is greater,
      marks the end of the file */
   t1 = t2 = 0.0;
   if (ref->tail->pred!=ref->head)
      t1=ref->tail->pred->end;
   if (test->tail->pred!=test->head)
      t2=test->tail->pred->end;

   if (t1<0.0 || t2<0.0) {
      HError(-3333,"MatchSpotFiles: Final times missing from label files");
      return;
   }

   totalDur += (t1>t2)?t1:t2;

   /* increment key occ counts */
   nr=CountAuxLabs(ref,rlev);
   for (i=1; i<=nr;  i++) {
      l = GetAuxLabN(ref,i,rlev);
      j=Index((rlev==0)?l->labid:l->auxLab[rlev]);
      if (j!=0) ++keyOccs[j];
   }
   /* record key spots */
   nt=CountAuxLabs(test,tlev);
   for (i=1; i<=nt;  i++) {
      l = GetAuxLabN(test,i,tlev);
      t = *l;
      t.end = AuxLabEndTime(l,tlev);
      AddSpot(l->labid,IsHit(&t),l->score);
   }
}

/* PrintROCInfo: print %hits as function of FA rate */   
void PrintROCInfo(int kn)
{
   int i,fa;
   float hr;
   
   if (!headerPrinted) PrintHeader();
   PrintBar(0,spotWidth,'-',"ROC Information");
   printf("\n%13s:","KeyWord");
   for (fa=0;fa<=10;fa++)
      printf("%5d",fa); printf("\n");
   for (i=1; i<=kn; i++){
      printf("%13s:",names[i]->name);
      for (fa=0; fa<=10; fa++){
         hr = GetHitRate(i,fa);
         if (hr<100.0)
            printf("%5.1f",hr);
         else
            printf(" 100.");
      }
      printf("\n");
   }
   printf("%13s:","Overall");
   for (fa=0; fa<=10; fa++){
      hr = GetHitRate(kn+1,fa);
      if (hr<100.0)
         printf("%5.1f",hr);
      else
         printf(" 100.");
   }
   printf("\n");
}

/* OutputSpotStats: output the word spotting scores */
void OutputSpotStats(void)
{
   int i,kn;
   int totOcc=0,totHit=0,totFAs=0;
   int nHits, nFAs, nActual;
   float fom;
   
   if (trace&T_SPT) PrintKeySpots();
   CreateFOMTab();
   kn = nLabs;
   for (i=1; i<=kn; i++)
      CalcKeyFOM(i);
   CalcGlobalFOM();
   if (fullResults) PrintROCInfo(kn);
   if (!headerPrinted) PrintHeader();
   PrintBar(0,spotWidth,'-',"Figures of Merit");
   printf("\n%13s: %8s %8s %8s %8s\n","KeyWord","#Hits","#FAs","#Actual","FOM");
   for (i=1; i<=kn; i++){
      CountHits(i,&nHits,&nFAs); 
      totHit += nHits; totFAs += nFAs;
      nActual = keyOccs[i]; totOcc += nActual;
      fom = (nActual>0)?GetFOM(i):0.0;
      printf("%13s: %8d %8d %8d %8.2f\n",names[i]->name,
             nHits, nFAs, nActual, fom);
   }
   fom = (totOcc>0)?GetFOM(kn+1):0.0;
   printf("%13s: %8d %8d %8d %8.2f\n","Overall",
          totHit, totFAs, totOcc, fom);
   printf("\n");
   if (nonKeys>0)
      printf(" %d non keywords found in test files",nonKeys);
   PrintBar(0,spotWidth,'-',"");
   fflush(stdout);
}


/* ---------------------- Top Level Control ------------------- */

/* Initialise: load the given 'hmm' list and initialise the program */
void Initialise(char * listfn)
{
   CreateHeap(&tempHeap, "tempHeap", MSTAK, 1, 1.0, 8000, 40000);
   
   nulClass = GetLabId(nulName,TRUE);
   ReadHMMList(listfn);
   if (stripContexts) LTriStrip(TRUE);
   if (outPStats)
      InitConMat();
   if (wSpot)
      InitSpotLists();
   if (fullResults && !wSpot && !nistFormat)
      PrintBar(0,htkWidth,'-',"Sentence Scores");
   if (!nistFormat && spkrMask!=NULL) htkWidth += 11;
}

/* MatchFiles: match recfn (test) against labfn (ref) */
void MatchFiles(void)
{
   Transcription *tr;

   ans = LOpen(&tempHeap,recfn,tff);
   MakeFN(recfn,labDir,labExt,labfn);
   tr = LOpen(&tempHeap,labfn,rff);
   ref = GetLabelList(tr,1);
   if (ref->head->succ == ref->tail) {
      HError(-3330,"MatchFiles: Reference Transcription File %s is Empty",recfn);
      return;
   }
   NormaliseName(ref,rlev);
   if (wSpot)
      MatchSpotFiles();
   else
      MatchRecFiles();
   Dispose(&tempHeap,ans);
}

/* OutputStats:  output global statistics */
void OutputStats(void)
{
   if (wSpot)
      OutputSpotStats();
   else
      PrintGlobalStats();
}

/* ------------------------------------------------------------ */
/*                      END:  HResults.c                        */
/* ------------------------------------------------------------ */

