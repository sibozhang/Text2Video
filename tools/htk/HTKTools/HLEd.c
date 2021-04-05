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
/*      File: HLEd.c: Edit label file(s)                       */
/* ----------------------------------------------------------- */

char *hled_version = "!HVER!HLEd:   3.4.1 [CUED 12/03/09]";
char *hled_vc_id = "$Id: HLEd.c,v 1.1.1.1 2006/10/11 09:55:01 jal58 Exp $";

#include "HShell.h"
#include "HMem.h"
#include "HMath.h"
#include "HWave.h"
#include "HLabel.h"
#include "HDict.h"


/*
   This program reads in a list of 'editing' commands from a
   script file and then makes an edited copy of one or more 
   label files.  Each edit command in the script file must be
   on a separate line.  The commands supported are printed by
   the Summary routine.  Some commands can be 1 or 2 letters.
   The 1 letter form is retained from the original for compatibility.
*/

/* -------------------------- Trace Flags & Vars ------------------------ */

#define T_TOP     0001           /* basic progress reporting */
#define T_RDSC    0002           /* output edit script details */
#define T_CMDX    0004           /* trace general command operation */
#define T_CHOP    0010           /* trace Change operation */
#define T_SLEV    0020           /* trace level split/merge operation */
#define T_DLEV    0040           /* trace delete level operation */
#define T_EDIN    0100           /* trace edit file input */
#define T_MEM     0200           /* trace memory usage */
#define T_DIC     0400           /* trace dictionary substitution */

static int  trace    = 0;           /* trace level */
static ConfParam *cParm[MAXGLOBS];   /* configuration parameters */
static int nParm = 0;               /* total num params */

/* -------------------------- Global Variables etc ---------------------- */

#define MAXARGS  50     /* in any single HLEd command */
#define MAXIW    10     /* max number of interword boundaries */
#define MAXIV    10     /* max number of non-interword boundaries */

static char * newDir    = NULL;     /* dest label file directory */
static char * newExt    = "lab";    /* dest label file extension */

typedef enum { 
   NOCMD=0, 
   REPLACE, CHANGE, FIND,   MERGE,  EDOP_DELETE, DEFCON, TRIST,  
   SBDEF,   EXPAND, IFILL,  SORT,   WBDEF,  VBDEF,  LCTXT,  
   RCTXT,   TCTXT,  SETLEV, DELLEV, SPLLEV, ISIL,
   LASTCMD
} EdOp;

static int  nCmds = LASTCMD-1;
static char *cmdmap[] = {"","RE","CH","FI","ME","DE","DC","IT",
                         "SB","EX","FG","SO","WB","NB","LC",
                         "RC","TC","ML","DL","SP","IS","" };
static int  n1Cmds = 17;
static char oldmap[] = " RCFMDXZBEISWVLGTY ";

typedef struct {
   EdOp op;
   short nArgs;
   LabId args[MAXARGS];
}EditCmd;

typedef struct _ScriptItem{
   EditCmd cmd;
   struct _ScriptItem *next;
}ScriptItem;

static ScriptItem *script;          /* linked list of edit commands */
static LabId asterix;               /* id of an asterix */
static LabId sentMarker=NULL;       /* sentence bndary marker set by SB cmd */
static FILE *newLabs = NULL;        /* list of newly created labels */
static LabId wbnd[MAXIW];           /* interword bndaries set via WB cmd */
static LabId vbnd[MAXIV];           /* non-interwd bndaries set via NB cmd */
static int nWB = 0;                 /* number of interword boundaries */
static int nVB = 0;                 /* number of non-interword boundaries */
static Boolean noBounds = FALSE;    /* suppress boundary times if TRUE */
static Boolean triStrip = FALSE;    /* ignore triphone contexts in matching */
static Boolean levSplit = FALSE;    /* Split levels into lists */
static Boolean sortFirst = FALSE;   /* Sort on loading */
static FileFormat ifmt=UNDEFF;      /* Label input file format */
static FileFormat ofmt=UNDEFF;      /* Label output file format */
static HTime minGap = 50000.0;      /* minimum interword gap */
static char *dictFn = NULL;         /* Dictionary used for EX command */
static Vocab vocab;                 /* And the associated vocab */

static MemHeap tempHeap;            /* Storage for current file */
static MemHeap permHeap;            /* Permanent storage */

/* ------------------ Process Command Line ------------------------- */

/* Summary: print a summary of all HLEd commands */
void Summary(void)
{
   printf("\nHLEd Command Summary\n\n");
   printf("CH/C X A Y B    - replace Y in context of A_B by X\n");
   printf("DC/X A B C ...  - define context A as labels B,C,...\n");
   printf("DE/D A B  ...   - delete labels A,B,...\n");
   printf("DL [N]          - delete all current level [or level N]\n");
   printf("EX/E            - expand labels from dictionary or form A_B_C\n");
   printf("FG/I X          - fill interlabel gaps with label X\n");
   printf("FI/F A Y B      - find all occurrences of pattern AYB\n");
   printf("IS A B          - insert A at start and B at end\n");
   printf("IT/Z            - ignore triphone contexts in CH/FI cmds\n");
   printf("LC/L [X]        - convert phonemes to Left-context dependent\n");
   printf("ML N            - move to level N (1-99)\n");
   printf("ME/M X A B ...  - merge label seq A,B,.. and rename X\n");
   printf("NB/V X          - label X is not an interword boundary\n");
   printf("RC/G [X]        - convert phonemes to right-context dependent\n");
   printf("RE/R X A B ...  - replace labels A,B,... by X\n");
   printf("SB/B X          - define X as a sentence boundary marker\n");
   printf("SO/S            - sort labels into time order\n");
   printf("SP              - split multiple levels into multiple lists\n");
   printf("TC/T [X [Y]]    - convert phonemes to Triphones\n");
   printf("WB/W X          - define X as an interword boundary\n\n");
   Exit(0);
}

void ReportUsage(void)
{
   printf("\nUSAGE: HLEd [options] edCmdFile labFiles...\n\n");
   printf(" Option                                       Default\n\n");
   printf(" -b      suppress boundary times              off\n");
   printf(" -d s    load dictionary from s               off\n");
   printf(" -i s    Output transcriptions to MLF s       off\n"); 
   printf(" -g f    Set min gap for FG command to f      50000.0\n"); 
   printf(" -l s    Dir to store output label file(s)    current\n");
   printf(" -m      Strip to monophones on loading       off\n");
   printf(" -n f    Output list of all new labs to f     off\n");
   PrintStdOpts("GIPQX");
   printf("\n\n");
}

int main(int argc, char *argv[])
{
   char * labFn, *scriptFn, *newFn=NULL, *s;
   int fidx;
   MLFEntry *me;
   void Initialise(void);
   void EditFile(char *labfn);
   void ReadScript(char *scriptFn);
   void ListFinds(void);

   if(InitShell(argc,argv,hled_version,hled_vc_id)<SUCCESS)
      HError(1200,"HLEd: InitShell failed");

   InitMem();   InitMath();
   InitWave();  InitLabel();
   InitDict();

   if (!InfoPrinted() && NumArgs() == 0)
      ReportUsage();
   if (NumArgs() == 0) Exit(0);

   Initialise();
   while (NextArg() == SWITCHARG) {
      s = GetSwtArg();
      if (strlen(s)!=1) 
         HError(1219,"HLEd: Bad switch %s; must be single letter",s);
      switch(s[0]){
      case 'b':
         noBounds = TRUE; break;
      case 'd':
         if (NextArg()!=STRINGARG)
            HError(1219,"HLEd: Dictionary filename expected");
         dictFn = GetStrArg();
         break;
      case 'g':
         if (NextArg() != FLOATARG)
            HError(1219,"HLEd: Min fill gap expected (100ns units)");
         minGap = GetChkedFlt(0.0,100000000.0,s); 
         break;
      case 'i':
         if (NextArg()!=STRINGARG)
            HError(1219,"HLEd: Output MLF file name expected");
         if(SaveToMasterfile(GetStrArg())<SUCCESS)
            HError(1214,"HCopy: Cannot write to MLF");
         break;
      case 'l':
         if (NextArg()!=STRINGARG)
            HError(1219,"HLEd: Output label file directory expected");
         newDir = GetStrArg(); break;
      case 'm':
         LTriStrip(TRUE); break;
      case 'n':
         if (NextArg() != STRINGARG)
            HError(1219,"HLEd: New labels file name expected");
         newFn = GetStrArg(); break;
      case 'G':
         if (NextArg() != STRINGARG)
            HError(1219,"HLEd: Input label file format expected");
         if((ifmt = Str2Format(GetStrArg())) == ALIEN)
            HError(-1289,"HLEd: Warning ALIEN input label file format set");
         break;
      case 'I':
         if (NextArg() != STRINGARG)
            HError(1219,"HLEd: Input MLF file name expected");
         LoadMasterFile(GetStrArg()); break;
      case 'P':
         if (NextArg() != STRINGARG)
            HError(1219,"HLEd: Output label file format expected");
         if((ofmt = Str2Format(GetStrArg())) == ALIEN)
            HError(-1289,"HLEd: Warning ALIEN Label output file format set");
         break;
      case 'Q':
         Summary(); break;
      case 'T':
         trace = GetChkedInt(0,0777,s); break;
      case 'X':
         if (NextArg()!=STRINGARG)
            HError(1219,"HLEd: Output label file extension expected");
         newExt = GetStrArg(); break;
      default:
         HError(1219,"HLEd: Unknown switch %s",s);
      }
   }
   if (NextArg()!=STRINGARG)
      HError(1219,"HLEd: Edit script file name expected");
   scriptFn = GetStrArg();
   ReadScript(scriptFn);
   if (newFn!=NULL)
      if ((newLabs = fopen(newFn,"w")) == NULL)
         HError(1211,"HLEd: Cannot create new label file %s",newFn);
   if (dictFn!=NULL) {
      InitVocab(&vocab);
      if(ReadDict(dictFn,&vocab)<SUCCESS)
         HError(1213,"HLEd: ReadDict failed");
   }

   while (NumArgs()>0){
      if (NextArg()!=STRINGARG)
         HError(1219,"HLEd: Input label file name expected");
      labFn = GetStrArg();
      if (IsMLFFile(labFn)){
         fidx = NumMLFFiles();
         if ((me=GetMLFTable()) != NULL) {
            while(me->next != NULL) me=me->next;
            LoadMasterFile(labFn);
            me=me->next;
         }else{
            LoadMasterFile(labFn);
            me=GetMLFTable();
         }
         while (me != NULL) {
            if (me->type == MLF_IMMEDIATE && me->def.immed.fidx == fidx)
               EditFile(me->pattern);
            me = me->next;
         }
      } else
         EditFile(labFn);
   }
   ListFinds();
   if (newLabs != NULL) fclose(newLabs);
   Exit(0);
   return (0);          /* never reached -- make compiler happy */
}

/* --------------------- Initialisation ----------------------- */

/* SetConfParms: set conf parms relevant to HLEd */
void SetConfParms(void)
{
   int i;
   
   nParm = GetConfig("HLED", TRUE, cParm, MAXGLOBS);
   if (nParm>0) {
      if (GetConfInt(cParm,nParm,"TRACE",&i)) trace = i;
   }
}

/* Initialise: confparms, str->int map and memory */
void Initialise(void)
{
   int i;
   char buf[MAXSTRLEN];
   LabId labid;
   
   SetConfParms();
   asterix = GetLabId("*",TRUE);
   for (i=1;i<=99;i++) {
      sprintf(buf,"%d",i);
      labid=GetLabId(buf,TRUE);
      labid->aux=(void*) i;
   }
   CreateHeap(&permHeap, "permHeap", MSTAK, 1, 1.2, 512, 4096);
   CreateHeap(&tempHeap, "tempHeap", MSTAK, 1, 1.2, 4096, 8192);
}

/* ------------------- Context List Handling ------------------ */

typedef struct _Context{
   short nDefs;
   LabId cname;
   LabId cdef[MAXARGS];
   struct _Context *next;
} Context;

static Context *xlist = NULL;

/* DefineContext:  add given context to list */
void DefineContext(LabId name, LabId *def, int nDefs)
{
   Context *p;

   p = (Context *)New(&permHeap,sizeof(Context));
   p->next = xlist; xlist = p;
   p->cname = name; p->nDefs = nDefs;
   memcpy(p->cdef,def,nDefs*sizeof(LabId));
   p->cdef[nDefs] = NULL;
}

/* PrintContexts: print all contexts */
void PrintContexts(void)
{
   Context *p = xlist;
   void PrintIdList(LabId *i);

   if (p == NULL){
      printf("No Contexts Defined\n");
      return;
   }
   printf("Contexts Defined:\n");
   while (p != NULL) {
      printf("   Context %s == ",p->cname->name);
      PrintIdList(p->cdef);
      printf("\n");
      p = p->next;
   }
}
   
/* GetContext: return list of ids defined by name */
LabId *GetContext(LabId name)
{
   Context *p = xlist;

   while (p != NULL) {
      if (p->cname == name)
         return p->cdef;
      p = p->next;
   }
   HError(1230,"GetContext: Context %s not defined by DC command",name->name);
   return NULL;
}

/* IsWBnd: return true if given id is an Interword boundary */
Boolean IsWBnd(LabId id)
{
   int i;
   
   for (i=0; i<nWB; i++)
      if (wbnd[i] == id) return TRUE;
   return FALSE;
}

/* PrintWBounds: print a list of defined interword boundaries */
void PrintWBounds(void)
{
   int i;
   
   if (nWB==0)
      printf("No inter-word boundaries defined\n");
   else {
      printf("Word Bounds Defined:\n");
      for (i=0;i<nWB;i++) printf("  %s",wbnd[i]->name);
      printf("\n");
   }
}

/* IsVBnd: return true if given id is a non-interword boundary */
Boolean IsVBnd(LabId id)
{
   int i;
   
   for (i=0; i<nVB; i++)
      if (vbnd[i] == id) return TRUE;
   return FALSE;
}

/* PrintVBounds: print a list of defined non-interword boundaries */
void PrintVBounds(void)
{
   int i;
   
   if (nVB==0)
      printf("No non inter-word boundaries defined\n");
   else {
      printf("Non Inter-Word Boundaries Defined:\n");
      for (i=0;i<nVB;i++) printf("  %s",vbnd[i]->name);
      printf("\n");
   }
}

/* ------------------- New Label Recording -------------------- */

/* PutLab: if given label new then output it to newLabs */
void PutLab(LabId id)
{
   if (id->aux == NULL) {  /* not seen this label before */
      id->aux = (void*) -1;
      fprintf(newLabs,"%s\n",id->name);
   }
}

/* ------------------- Find Accumulators ---------------------- */

typedef struct _FindAcc{
   LabId *pattern;      /* pattern to find */
   int acc;             /* num of matches */
   struct _FindAcc *next;
} FindAcc;

static FindAcc *facc = NULL;

/* GetFindAccumulator: return a pointer to acc for given pattern */
int *GetFindAccumulator(LabId *pat)
{
   FindAcc *p = facc;

   while (p != NULL) {
      if (p->pattern == pat)
         return &(p->acc);
      p = p->next;
   }
   HError(1292,"GetFindAccumulator: Acc %s %s %s not defined",
          pat[0]->name,pat[1]->name,pat[2]->name);
   return NULL;
}

/* MakeFindAccumulator: create a new find acc for given pattern */
void MakeFindAccumulator(LabId *pat)
{
   FindAcc *p;

   p = (FindAcc *)New(&permHeap,sizeof(FindAcc));
   p->next = facc; facc = p;
   p->pattern = pat; p->acc = 0;
}

/* ListFinds: list accumulated totals in all find accs */
void ListFinds(void)
{
   FindAcc *p = facc;
   LabId *i;

   if (p != NULL) {
      printf("Find Totals:\n");
      while (p != NULL){
         i = p->pattern;
         printf(" %4d  %s-%s+%s\n",p->acc,
                i[0]->name,i[1]->name,i[2]->name);
         p = p->next;
      }
   }
}

/* ---------------------- Print Script ---------------------- */

/* PrintIdList: print list of ids */
void PrintIdList(LabId *i)
{
   while (*i != NULL) {
      printf(" %s",(*i)->name);
      ++i;
   }
}

/* PrintScript: prints the currently loaded script - for tracing only */
void PrintScript(char *scriptFN)
{
   ScriptItem *i;
   int j;
   LabId src;

   printf("HLEd: script %s\n",scriptFN);
   if (sortFirst) 
      printf("Sort (initial)\n");
   if (levSplit)
      printf("Split levels into lists\n");
   for (i=script, j=1; i != NULL; i = i->next,j++) {
      printf("%2d. ",j);
      src = *(i->cmd.args);
      switch (i->cmd.op) {
      case TRIST:
         printf("Toggle TriStripping\n");
         break;
      case SORT:
         printf("Sort\n"); 
         break;
      case EDOP_DELETE:
         printf("Delete  ["); PrintIdList(i->cmd.args); printf(" ]\n");
         break;
      case REPLACE:
         printf("Replace ["); PrintIdList(i->cmd.args+1);
         printf(" ] by %s\n",src->name);
         break;
      case CHANGE:
         printf("Change  ["); PrintIdList(i->cmd.args+1);
         printf(" ] by %s\n",src->name);
         break;
      case FIND:
         printf("Find  ["); PrintIdList(i->cmd.args); 
         printf(" ]\n");
         break;
      case EXPAND:
         printf("Expand\n");
         break;
      case IFILL:
         printf("FillGap  ["); PrintIdList(i->cmd.args); 
         printf(" ]\n");
         break;
      case ISIL:
         printf("InSil  ["); PrintIdList(i->cmd.args); 
         printf(" ]\n");
         break;
      case MERGE:
         printf("Merge   ["); PrintIdList(i->cmd.args+1);
         printf(" ] to %s\n",src->name);
         break;
      case LCTXT:
         printf("Left Context ["); PrintIdList(i->cmd.args);
         printf(" ]\n");
         break;
      case RCTXT:
         printf("Right Context ["); PrintIdList(i->cmd.args);
         printf(" ]\n");
         break;
      case TCTXT:
         printf("Triphonise ["); PrintIdList(i->cmd.args);
         printf(" ]\n");
         break;
      case SETLEV:
         printf("Set Level to %d\n",(int)i->cmd.args[0]->aux);
         break;
      case DELLEV:
         if (i->cmd.nArgs==1)
            printf("Delete Level %d\n",(int)i->cmd.args[0]->aux);
         else
            printf("Delete Current Level\n");
         break;
      default: break;
      }
   }
}

/* ---------------------- Read Script ---------------------- */

/* CmdIndex: return index 1..N of given command */
int CmdIndex(char *s)
{
   int i;

   if (s[1] == ' ') {
      for (i=1; i<=n1Cmds; i++)
         if (oldmap[i] == s[0]) return i;
   } else {
      for (i=1; i<=nCmds; i++)
         if (strcmp(cmdmap[i],s) == 0) return i;
   }
   return -1;
}

/* ReadCmd: returns next command in src or NOCMD if none */
EdOp ReadCmd(Source *src)
{
   char s[3],buf[MAXSTRLEN];
   int cmdidx;
   
   SkipComment(src);
   SkipWhiteSpace(src);
   if (!ReadString(src,buf)) return(NOCMD);
   s[0]=buf[0]; s[1] = isalnum((int) buf[1])?buf[1]:' '; s[2]='\0';
   if (trace&T_EDIN)
      printf(" input cmd   '%s'\n",s);
   cmdidx = CmdIndex(s);
   if (cmdidx<0)
      HError(1230,"ReadCmd: Unknown command %s at %s",
             s,SrcPosition(*src,buf));
   return (EdOp)cmdidx;
}

/* ReadLabId: Read a label from the current line, return NULL if none */
LabId ReadLabId(Source *src)
{
   char buf[MAXSTRLEN];
   
   SkipWhiteSpace(src);
   if (src->wasNewline) return NULL;
   if (ReadString(src,buf)){
      if (trace&T_EDIN)
         printf(" input label '%s'\n",buf);
      return GetLabId(buf,TRUE);
   } else
      HError(1230,"ReadLabId: string arg expected at %s",
             SrcPosition(*src,buf));
   return(NULL);
}

/* ReadIdList: Read a list of args and store in argList, return num args */
int ReadIdList(Source *src,LabId *argList)
{
   int count = 0;
   LabId id;
   
   id = ReadLabId(src);
   while (id != NULL) {
      *argList++ = id; ++count;
      id=ReadLabId(src);
   }
   *argList = NULL;
   return count;  
}

/* ReadScript: read a file of script commands and store them in script */
void ReadScript(char *scriptFn)
{
   Source src; 
   ScriptItem *i, *tail=NULL;
   EdOp op;
   LabId id,args[MAXARGS];
   int n,nArgs;

   if(InitSource(scriptFn, &src, NoFilter)<SUCCESS)
      HError(1210,"ReadScript: Can't open file %s", scriptFn);
   script = NULL;
   while (TRUE) {
      if ((op = ReadCmd(&src)) == NOCMD) break;
      if (op == DEFCON){
         id = ReadLabId(&src);
         nArgs = ReadIdList(&src,args);
         DefineContext(id,args,nArgs);
         continue;
      }
      if (op == WBDEF) {
         if (nWB == MAXIW)
            HError(1231,"ReadScript: too many W commands");
         id = ReadLabId(&src);
         wbnd[nWB++] = id;
         continue;
      }
      if (op == VBDEF) {
         if (nVB == MAXIV)
            HError(1231,"ReadScript: too many V commands");
         id = ReadLabId(&src);
         vbnd[nVB++] = id;
         continue;
      }
      if (op == SBDEF) {
         sentMarker = ReadLabId(&src);
         if (trace&T_RDSC)
            printf("Sentence marker is '%s'\n",sentMarker->name);
         continue;
      }
      if (op == SPLLEV) {
         levSplit=TRUE;
         if (trace&T_RDSC)
            printf("Splitting levels for output\n");
         continue;
      }
      if (op == SORT && script==NULL) {
         sortFirst=TRUE;
         if (trace&T_RDSC)
            printf("Initial sort enabled\n");
         continue;
      }
      i = (ScriptItem *)New(&permHeap,sizeof(ScriptItem));
      i->cmd.op = op;
      switch (op) {
      case EDOP_DELETE:
      case REPLACE:
      case MERGE:
         i->cmd.nArgs = ReadIdList(&src,i->cmd.args);
         break;
      case CHANGE:
         i->cmd.nArgs = ReadIdList(&src,i->cmd.args);
         if (i->cmd.nArgs != 4)
            HError(1230,"ReadScript: CH must have 4 arguments");
         break;   
      case FIND:
         i->cmd.nArgs = ReadIdList(&src,i->cmd.args);
         if (i->cmd.nArgs != 3)
            HError(1230,"ReadScript: FI must have 3 args");
         MakeFindAccumulator(i->cmd.args);
         break;   
      case LCTXT:
      case RCTXT:
         i->cmd.nArgs = ReadIdList(&src,i->cmd.args);
         if (i->cmd.nArgs > 1)
            HError(1230,"ReadScript: LC/RC command has too many args");
         break;
      case TCTXT:
         i->cmd.nArgs = ReadIdList(&src,i->cmd.args);
         if (i->cmd.nArgs > 2)
            HError(1230,"ReadScript: TC command has too many args");
         break;
      case IFILL:
         i->cmd.nArgs = ReadIdList(&src,i->cmd.args);
         if (i->cmd.nArgs != 1)
            HError(1230,"ReadScript: FG must have 1 arg");
         break;   
      case ISIL:
         i->cmd.nArgs = ReadIdList(&src,i->cmd.args);
         if (i->cmd.nArgs != 2)
            HError(1230,"ReadScript: IS must have 2 args");
         break;   
      case SETLEV:
         i->cmd.nArgs = ReadIdList(&src,i->cmd.args);
         if (i->cmd.nArgs!=1 || 
             (n=(int)i->cmd.args[0]->aux)<1 || n>99 )
            HError(1230,"ReadScript: ML must have 1 arg between 1 and 99");          
         break;   
      case DELLEV:
         i->cmd.nArgs = ReadIdList(&src,i->cmd.args);
         if (i->cmd.nArgs>1)
            HError(1230,"ReadScript: DL can have at most 1 arg");
         if (i->cmd.nArgs==1 && 
             ((n=(int)i->cmd.args[0]->aux)<1 || n>99) )
            HError(1230,"ReadScript: DL arg must be between 1 and 99");           
         break;   
      case SORT:
      case EXPAND:
      case TRIST:
         break;
      default: break;
      }
      i->next = NULL;
      if (tail==NULL)
         script = tail = i;
      else {
         tail->next = i;
         tail = i;
      }
   }
   CloseSource(&src);
   if (trace&T_RDSC){
      PrintContexts();
      PrintWBounds();
      PrintVBounds();
      PrintScript(scriptFn);
   }
}

/* ----------------------- Misc Operations -------------------- */

/* NumParts: return number of parts in a label of form A_B_C  or 
   the number of phones in the dictionary entry as appropriate */
int NumParts(LabId id)
{
   Word word;
   char buf[MAXSTRLEN], *s;
   int count=1;
   
   if (dictFn!=NULL) {
      if ((word = GetWord(&vocab,id,FALSE))==NULL)
         HError(1232,"NumParts: Cannot find word %s in dictionary",id->name);
      if (word->pron==NULL || word->pron->nphones==0)
         HError(1232,"NumParts: Word %s does not have valid pronunciation",id->name);
      count = word->pron->nphones;
      if (trace&T_DIC)
         printf("NumParts: word %s has %d parts\n",id->name,count);
   }
   else {
      strcpy(buf,id->name);
      while ((s = strchr(buf,'_')) != NULL) {
         ++count; *s = ' ';
      }
   }
   return count;
}

/* GetPart: return n'th part of a label of form A_B_C   or 
     the n'th phone in the dictionary entry as appropriate */
LabId GetPart(LabId id, int n)
{
   Word word;
   char buf[MAXSTRLEN], *s, *t;
   int i;
   LabId pid;
   
   if (dictFn!=NULL) {
      if ((word = GetWord(&vocab,id,FALSE))==NULL)
         HError(1290,"GetPart: Cannot find word %s in dictionary",id->name);
      if (word->pron==NULL || word->pron->nphones<n)
         HError(1290,"GetPart: Word %s does not have valid phone %d",id->name,n);
      pid = word->pron->phones[n-1];
      if (trace&T_DIC)
         printf("GetPart: %d'th part of word %s is %s\n",n,id->name,pid->name);
      return(pid);
   }
   else {
      strcpy(buf,id->name);
      i = 1; s= buf;
      while (i<n && (s = strchr(s,'_')) != NULL) {
         ++i; ++s;
      }
      if (s==NULL)
         HError(1290,"GetPart: cant find %d leading _'s in %s",n-1,buf);
      if ((t = strchr(s,'_')) != NULL) 
         *t = '\0';
      return GetLabId(s,TRUE);
   }
}

/* ----------------------- Edit Operations -------------------- */

/* TrOpEntry: trace entry to named operation */
void TrOpEntry(char *name)
{
   printf(" -> %s",name);
}

/* TrOpExit: trace exit from current operation */
void TrOpExit(int count)
{
   if (count >= 0) printf(" %d items",count);
   printf("\n");  fflush(stdout);
}

/* IsSame: return true if given ids are identical */
Boolean IsSame(LabId a, LabId b)
{
   char buf[MAXSTRLEN];

   if (a==NULL || b==NULL) return(FALSE);
   if (triStrip) {
      strcpy(buf,a->name); TriStrip(buf); a = GetLabId(buf,TRUE);
      strcpy(buf,b->name); TriStrip(buf); b = GetLabId(buf,TRUE);
   }
   return a==b;
}

/* IsInIdList: return true if id is in idlist */
Boolean IsInIdList(LabId id, LabId *idlist)
{
   if (id!=NULL)
      while (*idlist != NULL) {
         if (IsSame(id,*idlist)) return TRUE;
         ++idlist;
      }
   return FALSE;
}

/* SeqMatch: returns true if labels i,i+1,i+2,... match idList */
LLink SeqMatch(LLink l,int numIds, LabId *idList)
{
   int j;
   
   for (j=0; j<numIds;j++,l=l->succ)
      if (*idList++ != l->labid || l->succ==NULL) {
         return(NULL);
      }
   return l;
}

/* DeleteOp: scan the current transcription and delete any items
   in args,  returns number of deletions performed */
int DeleteOp(LabList *ll,LabId *args)
{
   LLink l;
   int count=0;
   
   if (trace&T_CMDX) TrOpEntry("DE(lete");
   for (l=ll->head->succ; l->succ!=NULL; l=l->succ)
      if (IsInIdList(l->labid,args)) {
         ++count;
         DeleteLabel(l);
      }
   if (trace&T_CMDX) TrOpExit(count);
   return count;
}

/* ReplaceOp: scan the current transcription and replace any
   items in args[1..] with arg[0],  returns the number of
   replaces performed */
int ReplaceOp(LabList *ll,LabId *args)
{
   LabId item;
   LLink l;
   int count=0;
   
   if (trace&T_CMDX) TrOpEntry("RE(place");
   item = *args++;
   for (l=ll->head->succ; l->succ!=NULL; l=l->succ)
      if(IsInIdList(l->labid,args)) {
         ++count;
         l->labid = item;
      }
   if (trace&T_CMDX) TrOpExit(count);
   return count;
}

/* InContext: Match Context (lc-x+rc) against label l */
Boolean InContext(LLink l, LabId *lc, LabId *rc, LabId item)
{
   if (l->succ==NULL || l->pred==NULL)
      HError(1291,"InContext: trying to match sentinel");
   if (!IsSame(l->labid,item)) return FALSE;
   /* Check left context - 1st label is a special case */
   if (l->pred->labid == NULL ){
      if (lc != NULL && !IsInIdList(sentMarker,lc)) return FALSE;
   } else {
      if (lc != NULL && !IsInIdList(l->pred->labid,lc)) return FALSE;
   }
   /* Check right context - last label is a special case */
   if (l->succ->labid == NULL) {
      if (rc != NULL && !IsInIdList(sentMarker,rc)) return FALSE;
   } else {
      if (rc != NULL && !IsInIdList(l->succ->labid,rc)) return FALSE;
   }
   return TRUE;
}

/* ChangeOp: scan the current transcription and replace any 
   items which match the context given by args[1..3] with arg[0],
   returns number of changes performed.  A sequence of CH cmds is
   executed simultaneously by making changes to a copy.  ChangeOp is
   called with NULL args to terminate and update any pending changes */
int ChangeOp(LabList *ll,LabId *args)
{
   int i,count=0;
   LabId *lc,*rc,newId;
   static LabList *tl;
   LLink l,t;

   if (ll==NULL) {      /* end of change sequence */
      tl=NULL;  return 0;
   } else
      if (tl==NULL)        /* first change so clone transcription */
         tl = CopyLabelList(&tempHeap, ll);
   if (trace&T_CMDX) TrOpEntry("CH(ange");
   newId = *args++;
   lc = (*args==asterix)?NULL:GetContext(*args);
   rc = (*(args+2)==asterix)?NULL:GetContext(*(args+2));
   if (trace&T_CHOP){
      printf("  Left  context="); if (lc!=NULL) PrintIdList(lc);
      printf("\n  Right context="); if (rc!=NULL) PrintIdList(rc);
      printf("\n");
   }
   i=0;
   for (l=ll->head->succ,t=tl->head->succ; l->succ!=NULL;
        l=l->succ,t=t->succ) {
      if (trace&T_CHOP) 
         printf("  %d. checking %s",++i,t->labid->name);
      if(InContext(t,lc,rc,*(args+1))) {
         ++count;
         l->labid = newId;
         if (trace&T_CHOP) printf(" replaced by %s\n",newId->name);
      } else
         if (trace&T_CHOP) printf("\n");
   }
   if (trace&T_CMDX) TrOpExit(count);
   return count;
}

/* FindOp: scan the current transcription and count any
   items which match the context given by args[0..2] */
void FindOp(LabList *ll,LabId *args)
{
   LLink l;
   int *acc;
   LabId *lc,*rc;

   if (trace&T_CMDX) TrOpEntry("FI(nd");
   acc = GetFindAccumulator(args);
   lc = (*args==asterix)?NULL:GetContext(*args);
   rc = (*(args+2)==asterix)?NULL:GetContext(*(args+2));
   
   for (l=ll->head->succ; l->succ!=NULL; l=l->succ)
      if(InContext(l,lc,rc,*(args+1)))
         ++(*acc);
   if (trace&T_CMDX) TrOpExit(-1);
}

/* MergeOp: scan the current transcription and merge any consecutive
   sequence of items matching args[1,2,3,..] with arg[0], returns 
   number of merges performed */
int MergeOp(LabList *ll,int nArgs, LabId *args)
{
   LLink l,n,t;
   int count=0;
   
   if (trace&T_CMDX) TrOpEntry("ME(rge");
   nArgs--;
   for (l=ll->head->succ; l->succ!=NULL; l=l->succ)
      if((n=SeqMatch(l,nArgs,args+1))!=NULL) {
         ++count;
         l->labid = *args;
         for (t=l->succ; t!=n; t=t->succ) t->labid=NULL; /* Deleted */
         l->end = n->pred->end;
         l->succ = n;
         l->succ->pred = l;
      }
   if (trace&T_CMDX) TrOpExit(count);
   return count;
}

/* SortOp: sort the lablist into ascending order of start time */
void SortOp(LabList *rl)
{
   LLink i,j,sw,succ,pred;
   int count = 0;
   
   if (trace&T_CMDX) TrOpEntry("SO(rt");
   for (i=rl->head->succ; i->succ!=NULL; i=sw->succ) {
      sw = i;  /* find soonest label sw */
      for (j=i->succ; j->succ!=NULL; j=j->succ) {
         if (j->start<sw->start || (j->start==sw->start && j->end<sw->end))
            sw=j;
      }
      if (sw!=i) {  /* swap sw with first elem in scan list */
         ++count;
         pred = i->pred; succ = sw->succ;
         i->pred->succ = i->succ;      /* Delete i from list */
         i->succ->pred = i->pred;
         sw->pred->succ = sw->succ;    /* Delete sw from list */
         sw->succ->pred = sw->pred;
         i->succ = succ; i->pred=succ->pred;   /* Insert i */
         i->succ->pred = i->pred->succ = i;
         sw->pred = pred; sw->succ=pred->succ; /* Insert sw */
         sw->succ->pred = sw->pred->succ = sw;
      }  
   }
   if (trace&T_CMDX) TrOpExit(count);
}

/* IFillOp: fill inter-label gaps with arg1, return number added */
int IFillOp(LabList *ll,LabId *args)
{
   int count=0;
   LabId fId;
   LLink l,n;

   if (trace&T_CMDX) TrOpEntry("FG(fillgaps");
   if (ll->head->succ->succ != NULL) {
      fId = *args;
      for (l=ll->head->succ; l->succ->succ!=NULL; l=l->succ) {
         if ((l->succ->start - l->end) > minGap) {
            count++; 
            n = CreateLabel(&tempHeap,ll->maxAuxLab);
            n->labid = fId;
            n->start = l->end;
            n->end = l->succ->start;   
            n->succ = l->succ; n->pred = l;
            n->pred->succ = n; n->succ->pred = n;
         }
      }
   }
   if (trace&T_CMDX) TrOpExit(count);
   return count;
}

/* ISilOp: insert arg1 at start and arg2 at end */
int ISilOp(LabList *ll,LabId *args)
{
   LLink l,n;
   int count=0;

   if (trace&T_CMDX) TrOpEntry("IS(insert silence");
   if (ll->head->succ->succ != NULL) {
      count += 2;
      /* do start */
      l=ll->head;
      n = CreateLabel(&tempHeap,ll->maxAuxLab);
      n->labid = args[0];
      n->start = n->end = l->succ->start;
      n->succ = l->succ; n->pred = l;
      l->succ = n; n->succ->pred = n;
      /* do end */
      l=ll->tail->pred;
      n = CreateLabel(&tempHeap,ll->maxAuxLab);
      n->labid = args[1];
      n->start = n->end = l->end;
      n->succ = l->succ; n->pred = l;
      l->succ = n; n->succ->pred = n;
   }
   if (trace&T_CMDX) TrOpExit(count);
   return count;
}

/* ExpandOp: expand labels of form A_B_C_... A B C */
int ExpandOp(LabList *ll)
{
   int k,count=0,nparts;
   LLink l,n;
   LabId id;
   HTime partDur;

   if (trace&T_CMDX) TrOpEntry("EX(pand");
   for (l=ll->head->succ; l->succ!=NULL; l=l->succ) {
      id = l->labid;
      nparts = NumParts(id);
      if (nparts > 1 || dictFn != NULL) {
         partDur = (l->end - l->start) / nparts;
         l->labid = GetPart(id,1); /* Aux label stays with first */
         l->end = l->start+partDur;
         for (k=2; k<=nparts; k++){
            n = CreateLabel(&tempHeap,ll->maxAuxLab);
            n->labid = GetPart(id,k);
            n->start = l->end;
            n->end = n->start+partDur;          
            n->succ=l->succ;n->pred=l;
            l->succ=n;n->succ->pred=n;          
            l=n; count++;
         }
      }
   }
   if (trace&T_CMDX) TrOpExit(count);  
   return count;
}


/* MakeTriId:  concatenate args separated by - and +'s and return its id */
LabId MakeTriId(LabId l, LabId c, LabId r)
{
   char buf[MAXSTRLEN];
   LabId item;
   
   if ( l!= NULL){
      strcpy(buf,l->name); strcat(buf,"-");
      strcat(buf,c->name);
   } else
      strcpy(buf,c->name);
   if (r != NULL){
      strcat(buf,"+"); strcat(buf,r->name);
   }
   item = GetLabId(buf,TRUE);
   return item;
}

/* LeftTriCxt: return id of left context from idx */
LabId LeftTriCxt(LLink l)
{
   for (l=l->pred; l->pred; l=l->pred) if (!IsVBnd(l->labid)) break;
   return (l->pred!=NULL? l->labid: NULL);
}

/* RightTriCxt: return id of right context from idx */
LabId RightTriCxt(LLink l)
{
   for (l=l->succ; l->succ; l=l->succ) if (!IsVBnd(l->labid)) break;
   return (l->succ!=NULL? l->labid: NULL);
}

/* TriPhonise: convert phone labels in transcription t to either
      left, right or triphone (ie both) contexts */
int TriPhonise(LabList *ll,Boolean left, Boolean right, LabId stId, 
               LabId enId, char *cmd)
{
   int count=0;
   LabId leftId, centreId, rightId; 
   LLink l,t;
   LabList *tmp;
   
   if (trace&T_CMDX) TrOpEntry(cmd);
   tmp = CopyLabelList(&tempHeap, ll); 
   for (l=ll->head->succ,t=tmp->head->succ; l->succ!=NULL;
        l=l->succ,t=t->succ) {
      centreId = t->labid;
      if (l->pred->pred==NULL) {         /* First Label */
         leftId =left?stId:NULL;
         rightId = (right && stId != NULL)?RightTriCxt(t):NULL;
      }
      else if (t->succ->succ==NULL) {    /* Last Label */
         leftId = (left && enId != NULL)?LeftTriCxt(t):NULL;
         rightId = right?enId:NULL;
      }
      else {
         leftId =left?LeftTriCxt(t):NULL;
         rightId = right?RightTriCxt(t):NULL;
      }
      if (IsWBnd(leftId)  || IsWBnd(centreId)) leftId  = NULL;
      if (IsWBnd(rightId) || IsWBnd(centreId)) rightId = NULL;
      if ((l->labid = MakeTriId(leftId,centreId,rightId))!= centreId)
         ++count;
   }
   if (trace&T_CMDX) TrOpExit(count);  
   return count;
}

/* DoLCtxt: Left context command */
int DoLCtxt(LabList *ll,int nArgs,LabId *args)
{
   LabId x;
   
   x = nArgs==1?*args:NULL;
   return TriPhonise(ll,TRUE,FALSE,x,x,"LC(leftcontext");
}

/* DoRCtxt: Right context command */
int DoRCtxt(LabList *ll,int nArgs,LabId *args)
{
   LabId x;
   
   x = nArgs==1?*args:NULL;
   return TriPhonise(ll,FALSE,TRUE,x,x,"RC(rightcontext");
}

/* DoTCtxt: Left and Right context command */
int DoTCtxt(LabList *ll,int nArgs,LabId *args)
{
   LabId l,r;
   
   if (nArgs == 1)
      l = r = *args;
   else if (nArgs == 2){
      l = *args; r = *(args+1);
   } else
      l = r = NULL;
   return TriPhonise(ll,TRUE,TRUE,l,r,"TC(tricontext");
}

/* KillTimes: set all times to -1 */
void KillTimes(LabList *ll)
{
   LLink l;

   for (l=ll->head->succ; l->succ!=NULL; l=l->succ) {
      l->start = -1.0;
      l->end   = -1.0;
   }
}

/* ------------------ Multiple Level Handling -------------------- */

static int top=1;    /* virtual top level of current label list */

/* VSetLabelList: set top level to n */
void VSetLabelList(int n)
{
   top = n;
}

/* VGetLabelList: ensure attempts to get replacement top level 
                  are redirected */
LabList* VGetLabelList(Transcription *t, int n)
{
   if (n==top) n=1;
   else if (n==1) n=top;
   return GetLabelList(t, n);
}

/* SplitLevels: into separate LabLists (with pointers to parent) */
Transcription *SplitLevels(LabList *rl)
{
   Transcription *ltr;
   LabList *ll,*nl;
   LLink l,c;
   int lev;

   ltr = CreateTranscription(&tempHeap);
   nl=CopyLabelList(&tempHeap, rl);
   AddLabelList(nl, ltr);
   for (lev=1;lev<=rl->maxAuxLab;lev++) {
      ll=CreateLabelList(&tempHeap,1);
      AddLabelList(ll, ltr);
      for (l=nl->head->succ,c=NULL;l->succ!=NULL;l=l->succ) {
         if (l->auxLab[lev]!=NULL) {
            c=AddLabel(&tempHeap, ll, l->auxLab[lev], 
                       l->start, l->end, l->auxScore[lev]);
            c->auxLab[1]=(LabId) l;
            l->auxLab[lev]=NULL;
         }
         else if (c!=NULL)
            c->end=l->end;
      }
   }
   if (trace&T_SLEV) PrintTranscription(ltr,"Split levels");
   return ltr;
}

/* MergeLevels: merge separate LabLists back into levels */
LabList *MergeLevels(Transcription *ltr)
{
   LabList *ll;
   LLink l,c;
   int lev,idx;
   
   for (lev=idx=1;lev<ltr->numLists;lev++) {
      ll=GetLabelList(ltr,lev+1);
      if (ll->head->succ->succ==NULL)
         continue;
      for (c=ll->head->succ;c->succ!=NULL;c=c->succ) {
         l = (LLink) c->auxLab[1];
         l->auxLab[idx] = c->labid;
         l->auxScore[idx] = c->score;
      }
      idx++;
   }
   if (trace&T_SLEV) PrintTranscription(ltr,"Merged levels");
   return GetLabelList(ltr,1);
}

/* DeleteLevel: delete level lev from ltr */
void DeleteLevel(Transcription *ltr,int lev)
{
   LLink l,c;
   LabList *top,*ll;

   if (trace&T_CMDX) TrOpEntry("DL(deletelevel");
   if (lev==1) {
      /* Can't delete level 1 so do some swapping */

      if (ltr->numLists==1)
         HError(1231,"DeleteLevel: Cannot delete last levels");
      for (lev=2,ll=NULL;lev<=ltr->numLists;lev++) {
         ll=GetLabelList(ltr, lev);
         if (ll->head->succ->succ!=NULL)
            break;
         else if (lev==ltr->numLists)
            HError(1231,"DeleteLevel: Cannot delete final level");
      }

      top=GetLabelList(ltr, 1);

      /* First delete all labels in level 1 */

      top->head->succ=top->tail;
      top->tail->pred=top->head;

      /* Rebuild level lev using LLinks from level 1 */

      for (c=ll->head->succ;c->succ!=NULL;c=c->succ) {
         l = (LLink) c->auxLab[1];
         l->start = c->start;
         l->end = c->end;
         l->labid = c->labid; /* Will be redone later */
         l->score = c->score;

         l->succ = top->tail; l->pred = top->tail->pred;
         l->pred->succ = l->succ->pred = l;
      }
      /* Don't delete level because we will probably edit it */
      
      VSetLabelList(lev);
   }
   else
      ll=GetLabelList(ltr, lev);
   /* Delete level */

   ll->head->succ=ll->tail;
   ll->tail->pred=ll->head;
   if (trace&T_CMDX) TrOpExit(-1);
}

/* --------------------- Top Level Control -------------------- */

/* EditFile: Load label file and apply each script command to it. 
   Multiple alts within transcriptions are edited independently. */
void EditFile(char *labfn)
{
   ScriptItem *i;
   char outfn[255];
   int m,d,r,c,a,clev,nlev,list;
   Transcription *ct,*levs,*at;
   LabList *ll,*rl;
   LLink l;

   if (trace&T_TOP) {
      printf("Editing file: %s\n",labfn); fflush(stdout);
   }
   ct = LOpen(&tempHeap,labfn,ifmt);
   at = CreateTranscription(&tempHeap);
   triStrip = FALSE; /* reset to default value */
   
   a=c=m=d=r=0;
   for (list=1;list<=ct->numLists;list++) {
      rl = GetLabelList(ct, list);
      if (sortFirst) SortOp(rl);
      /* put each level as separate alt in levs */
      levs = SplitLevels(rl);
      nlev = levs->numLists; clev = 1;

      VSetLabelList(1); 
      ll = VGetLabelList(levs, clev);

      for (i = script; i != NULL; i = i->next) {
         if (i->cmd.op==SORT && nlev>1)
            HError(1231,"EditFile: Cannot perform SO on multilevel");
         else if ((i->cmd.op==EXPAND || i->cmd.op==IFILL || i->cmd.op==ISIL) 
                  && clev>1)
            HError(1231,"EditFile: Cannot perform EX/FG/IS on level>1");
         if (i->cmd.op != CHANGE || i==script) /* terminate any pending */
            ChangeOp(NULL, NULL);     /* sequence of CH(ange operations */
         if (i->cmd.op == TRIST)
            triStrip = !triStrip;
         if (ll!=NULL)
            switch (i->cmd.op) {
            case FIND:
               FindOp(ll,i->cmd.args); break;               
            case CHANGE:
               c += ChangeOp(ll,i->cmd.args); break;
            case REPLACE:
               r += ReplaceOp(ll,i->cmd.args); break;
            case MERGE:
               m += MergeOp(ll,i->cmd.nArgs,i->cmd.args); break;
            case LCTXT:
               c += DoLCtxt(ll,i->cmd.nArgs,i->cmd.args); break;
            case RCTXT:
               c += DoRCtxt(ll,i->cmd.nArgs,i->cmd.args); break;
            case TCTXT:
               c += DoTCtxt(ll,i->cmd.nArgs,i->cmd.args); break;
            case EDOP_DELETE:
               d += DeleteOp(ll,i->cmd.args); break;
            case DELLEV:
               if (i->cmd.nArgs==1)
                  DeleteLevel(levs,(int)i->cmd.args[0]->aux);
               else
                  DeleteLevel(levs,clev);
               break;
            case EXPAND:
               a += ExpandOp(ll); break;
            case IFILL:
               a += IFillOp(ll,i->cmd.args); break;
            case ISIL:
               a += ISilOp(ll,i->cmd.args); break;
            case SETLEV:
               clev=(int)i->cmd.args[0]->aux;
               if (clev>nlev) {
                  ll = NULL;
                  HError(-1231,"EditLevel: Level %d does not exist",clev);
               }else
                  ll = VGetLabelList(levs, clev);
               break;
            default: break;
            }
      }
      if (levSplit) {
         for (clev=1;clev<=nlev;clev++) {
            ll = GetLabelList(levs, clev);
            if (ll->head->succ->succ!=NULL) {
               ll->maxAuxLab = 0;      /* Don't want (dummy) auxlabs */
               ll = CopyLabelList(&tempHeap, ll);
               AddLabelList(ll, at);
            }
         }
      }else {
         ll = MergeLevels(levs);
         ll = CopyLabelList(&tempHeap, ll);
         AddLabelList(ll, at);
      }
   }
   MakeFN(labfn,newDir,newExt,outfn);
   if (newLabs != NULL)
      for (clev=1;clev<=at->numLists;clev++) {
         ll=GetLabelList(at, clev);
         for (l=ll->head->succ; l->succ!=NULL; l=l->succ)
            PutLab(l->labid);
      }
   if (noBounds)
      for (clev=1;clev<=at->numLists;clev++) {
         ll=GetLabelList(at, clev);
         KillTimes(ll);
      }
   if(LSave(outfn,at,ofmt)<SUCCESS)
      HError(1214,"EditFile: Cannot save file %s", outfn);
   if (trace&T_TOP) {
      printf("   %d adds; %d deletes; %d replaces; %d changes; %d merges\n",
             a,d,r,c,m);
      fflush(stdout);
   }
   if(trace & T_MEM) PrintAllHeapStats();
   ResetHeap(&tempHeap);
}


/* ------------------------------------------------------------ */
/*                         END:  HLEd.c                         */
/* ------------------------------------------------------------ */
