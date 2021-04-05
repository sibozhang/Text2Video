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
/*              2001  Cambridge University                     */
/*                    Engineering Department                   */
/*                                                             */
/*   Use of this software is governed by a License Agreement   */
/*    ** See the file License for the Conditions of Use  **    */
/*    **     This banner notice must not be removed      **    */
/*                                                             */
/* ----------------------------------------------------------- */
/*    File: HDMan:   pronunciation dictionary manager          */
/* ----------------------------------------------------------- */

char *hdman_version = "!HVER!HDMan:   3.4.1 [CUED 12/03/09]";
char *hdman_vc_id = "$Id: HDMan.c,v 1.2 2006/12/07 11:09:08 mjfg Exp $";

#include "HShell.h"
#include "HMem.h"
#include "HMath.h"
#include "HWave.h"
#include "HLabel.h"
#include "HDict.h"

/* -------------------------- Trace Flags & Vars ------------------------ */

#define T_TOP     00001           /* basic progress reporting */         
#define T_WBUF    00002           /* word buffer operations */           
#define T_VALI    00004           /* show valid inputs */                
#define T_EDW0    00010           /* word level editing */               
#define T_EDW1    00020           /* word level editing in detail */     
#define T_SCPT    00040           /* print edit scripts */               
#define T_NPHN    00100           /* new phone recording */              
#define T_DSOP    00200           /* pron deletions */                   
#define T_DWOP    00400           /* word deletions */                   

static int  trace    = 0;           /* trace level */
static ConfParam *cParm[MAXGLOBS];   /* configuration parameters */
static int nParm = 0;               /* total num params */

#define MAXARGS  100    /* max args in any command */
/* MAXPHONES (max phones in any pronunciation) is defined in HDict.h */
#define MAXPRONS 400     /* max number of pronunciations per word */
#define MAXDICTS 100     /* max number of source dictionaries */
#define MAXCONS  20     /* max number of contexts per script */
#define MAXPVOC  500    /* max num distinct phones */

/* ---------------------- Global Data Structures ------------------------------ */

typedef enum {UCWORD, UCPHONE, LCWORD, LCPHONE, DELETEW, DELDEF, FUNCW, DEFCON,
              REPLACEP, CONREPLACE, MERGEP, SPLITP, DELETEP, DELSOURCE, 
              LCTXT, RCTXT,
              TCTXT, APPSIL, REMSTRESS, REPLACEW, RAWMODE, 
              NOCMD} EdOp;

static char *cmdMap[] = { "UW","UP","LW","LP","DW","DD","FW","DC",
                          "RP","CR","MP","SP","DP","DS",
                          "LC","RC",
                          "TC","AS","RS","RW","IR"};
static int  nCmds = 21;

typedef struct{                 /* a single edit command */
   EdOp op;
   short nArgs;
   LabId args[MAXARGS];
}EditCmd;

typedef struct _ScriptItem{     /* internal rep of a complete edit script */
   EditCmd cmd;
   struct _ScriptItem *next;
}ScriptItem;

typedef struct {                /* a single pronunciation */
   short nPhone;
   float prob;
   LabId phone[MAXPHONES];
   LabId source;                /* name of source dict */
}Pronunciation;

typedef struct {
   LabId word;                  /* a word + its pronunciations */
   LabId outsym;                /* name of output symbol if any */
   short nPron;
   Pronunciation pron[MAXPRONS];
}WordBuf;

typedef struct {
   Boolean rawMode;             /* Raw input mode */
   EdOp wop;                    /* Used to apply UCWORD/LCWORD before sorting inputs */
   int numCons;                 /* number of context defs */
   EditCmd contexts[MAXCONS];   /* array of context defs */
   ScriptItem *script;          /* edit script for this input dictionary */
   int headSkip;                /* num header lines to skip on input */
   Source src;                  /* input file source */
   Boolean isPipe;              /* dictionary is input thru pipe */
   char *name;                  /* full path of dictionary file */
   LabId source;                /* name of source dict */
   WordBuf wbuf;                /* current input word */
   LabId nextWord;              /* next input word - for lookahead */
   LabId nextOutSym;            /* next input output sym - for lookahead */
   Pronunciation pbuf;          /* and its pronunciation - for lookahead */
   int totalWords;              /* total words in this source */
   int totalProns;              /* total prons in this source */
   int wordsUsed;               /* num words actually used */
   int pronsUsed;               /* num prons actually used */
}DBuffer;

/* Global storage */
static int nInputs = 0;              /* number of input dictionaries */
static DBuffer inbuf[MAXDICTS];      /* the input buffers and associated scripts */
static DBuffer outbuf;               /* the output buffer with its global script */
static FILE *outfile = NULL;         /* The output file */
static int nWords = 0;               /* number of words in word list */
static LabId *wList = NULL;          /* filter word list */
static FILE *newPhones = NULL;       /* file of newly created phones */
static char *scriptDir = NULL;       /* directory to look for scripts */
static char *gScriptFN = NULL;       /* name of global edit script */
static char *wListFN = NULL;         /* name of word list file */
static char *pListFN = NULL;         /* name of phone list file */
static int numOut = 0;               /* num words processed */
static int numMissing = 0;           /* num words not found */
static int numActive;                /* num active input dictionaries */
static int widx = 0;                 /* next word to take from wordList */
static LabId required;               /* current required word */
static MemHeap memStak;              /* all storage allocated in this */

/* Flags etc */
static Boolean mergeProns = FALSE;   /* merge prons from all sources */
static Boolean nullOutput = FALSE;   /* suppress generation of output dict */
static Boolean incOutSyms = FALSE;   /* write out extra field */
static Boolean incProbs = FALSE;     /* write out extra field */
static Boolean tagSources = FALSE;   /* tag output words with name of source dict */
static char commentChars[10] = "#";  /* default dictionary comment char */
static char wdBndSym[10] = "#";      /* word boundary symbol */

/* Global names */
static LabId asterix;                /* LabId of a "*" */
static LabId wdBnd;                  /* LabId of word boundary symbol  */   
static LabId cmuId;                  /* "cmu" */

/* Log Information */
static Boolean isLogging = FALSE;
static FILE *logF = NULL;            /* log file if any */
static int nNewPhones = 0;               /* num new phones encountered */
static int nDefPhones = 0;               /* num predefined phones */
static LabId newList[MAXPVOC];       /* list of new phones encountered */
static LabId defList[MAXPVOC];       /* list of predefined phones */

/* ------------------ Process Command Line ------------------------- */

void Summary(void)
{
   printf("\nHDMan Command Summary\n\n"); 
   printf("AS A B ...   - append silence models A, B, etc to each pronunciation\n");
   printf("CR X A Y B   - replace phone Y in the context of A_B by X.  Contexts\n");
   printf("               may include '*' [any] or defined context set (see DC)\n");
   printf("DC X A B ... - define set A B .... as context X\n");
   printf("DD X A B ... - delete definition for word X starting with phones A B ...\n");
   printf("DP A B C ... - delete any occurrences of phones A or B or C ...\n");
   printf("DS A         - delete pron from source A unless it is only one\n");
   printf("DW X Y Z ... - delete words (& definitions) X,Y,Z\n");
   printf("FW X Y Z ... - define X Y Z as function words and change\n");
   printf("               each phone in the definition to a function word\n");
   printf("               specific phone. In word W phone A becomes W.A etc.\n");
   printf("IR           - select raw input mode.  Each input word is single white\n");
   printf("               space delimited string (',\" and \\ not treated specially).\n");
   printf("LC [X]       - convert phones to Left-context dependent. If X given\n");
   printf("               then 1st phone in word -> X-a otherwise it is unchanged\n");
   printf("LP           - convert all phones to lowercase\n");
   printf("LW           - convert all words to lowercase\n");
   printf("MP X A B ... - merge any sequence of phones A B .. by X\n");
   printf("RC [X]       - convert phones to riGht-context dependent. If X given\n");
   printf("               then last phone in word -> z+X otherwise it is unchanged\n");
   printf("RP X A B ... - replace all occurrences of phones A or B .. by X\n");
   printf("RS system    - remove stress marking: system = cmu\n");
   printf("RW X A B ... - replace all occurrences of word A or B .. by X\n");
   printf("SP X A B ... - split phone X into sequence A B C ...\n");
   printf("TC [X [Y]]   - convert phones to Triphones. If X is given then 1st\n");
   printf("               phone -> X-a+b otherwise it is unchanged. If Y is given\n");
   printf("               last phone -> y-z+Y otherwise if X is given\n");
   printf("               then it -> y-z+X otherwise it is unchanged.\n");
   printf("UP           - convert all phones to uppercase\n");
   printf("UW           - convert all words to uppercase\n\n");
   Exit(0);
}

void ReportUsage(void)
{
   printf("\nUSAGE: HDMan [options] newDict srcDict1 srcDict2 ... \n\n");
   printf(" Option                                       Default\n\n");
   printf(" -a s    chars in s start comment lines       #\n");
   printf(" -b s    define word boundary symbol          #\n");
   printf(" -e dir  look for edit scripts in dir\n");
   printf(" -g f    global dictionary is in file f       global.ded\n");
   printf(" -h i j  skip 1st i lines of j'th dic file    0\n");
   printf(" -i      include output symbols               off\n");
   printf(" -j      include pronunciation probabilities  off\n");
   printf(" -l s    write log file in file s             no logging\n");
   printf(" -m      merge prons from all sources         first_only\n");
   printf(" -n f    output union of all phones to f      off\n");
   printf(" -o      disable dictionary output            enabled\n");
   printf(" -p f    load phone list stored in f\n");
   printf(" -t      tag output words with source         off\n");
   printf(" -w f    load word list stored in f\n");
   PrintStdOpts("Q");
   printf("\n\n");
}

int main(int argc, char *argv[])
{
   char *s,*fn;
   int i,skip;
   void Initialise(void);
   void EditFile(char *labfn);
   void CreateBuffer(char *dName, Boolean isInput);
   void EditAndMerge(void);
   void LoadWordList(void);
   void LoadPhoneList(void);
   void PrintLog(void);

   if(InitShell(argc,argv,hdman_version,hdman_vc_id)<SUCCESS)
      HError(1400,"HDMan: InitShell failed");
   InitMem();   InitMath();
   InitWave();  InitLabel();

   if (!InfoPrinted() && NumArgs() == 0)
      ReportUsage();
   if (NumArgs() == 0) Exit(0);

   for (i=0; i<MAXDICTS; i++)
      inbuf[i].headSkip = 0;
   Initialise();

   while (NextArg() == SWITCHARG) {
      s = GetSwtArg();
      if (strlen(s)!=1) 
         HError(1419,"HDMan: Bad switch %s; must be single letter",s);
      switch(s[0]){
      case 'a':
         if (NextArg() != STRINGARG)
            HError(1419,"HDMan: String of comment chars expected");
         strcpy(commentChars,GetStrArg());
         break;
      case 'b':
         if (NextArg() != STRINGARG)
            HError(1419,"HDMan: Word boundary symbol expected");
         strcpy(wdBndSym,GetStrArg());
         break;
      case 'e':
         if (NextArg() != STRINGARG)
            HError(1419,"HDMan: edit script directory expected");
         scriptDir = GetStrArg(); 
         break;  
      case 'g':
         if (NextArg() != STRINGARG)
            HError(1419,"HDMan: name of global edit script expected");
         gScriptFN = GetStrArg(); 
         break;  
      case 'h':
         skip = GetChkedInt(0,1000,s); 
         i = GetChkedInt(1,MAXDICTS,s); 
         inbuf[i-1].headSkip = skip;
         break;
      case 'i':
         incOutSyms = TRUE; break;
      case 'j':
         incProbs = TRUE; break;
      case 'l':
         if (NextArg() != STRINGARG)
            HError(1419,"HDMan: Log file name expected");
         fn = GetStrArg();
         if ((logF = fopen(fn,"w")) == NULL)
            HError(1411,"HDMan: Cannot create log file %s",fn);
         isLogging = TRUE;
         break;
      case 'm': 
         mergeProns = TRUE; break;
      case 'n':
         if (NextArg() != STRINGARG)
            HError(1419,"HDMan: New phone list file name expected");
         fn = GetStrArg();
         if ((newPhones = fopen(fn,"w")) == NULL)
            HError(1411,"HDMan: Cannot create new phone file %s",fn);
         break;
      case 'o': 
         nullOutput = TRUE; break;
      case 'p':
         if (NextArg() != STRINGARG)
            HError(1419,"HDMan: name of phone list expected");
         pListFN = GetStrArg(); 
         break;  
      case 't': 
         tagSources = TRUE; break;
      case 'w':
         if (NextArg() != STRINGARG)
            HError(1419,"HDMan: name of word list expected");
         wListFN = GetStrArg(); 
         break;  
      case 'Q':
         Summary(); break;
      case 'T':
         trace = GetChkedInt(0,01777,s); break;
      default:
         HError(1419,"HDMan: Unknown switch %s",s);
      }
   }
   wdBnd = GetLabId(wdBndSym,TRUE);
   if (NumArgs() < 2)
      ReportUsage();
   if (NextArg() != STRINGARG)
      HError(1419,"HDMan: Output dictionary file name expected");
   CreateBuffer(GetStrArg(),FALSE);
   i = 0;
   while (NumArgs()>0){
      if (NextArg() != STRINGARG)
         HError(1419,"HDMan: Input dictionary file name expected");
      if( ++i > MAXDICTS )
         HError(1430,"HDMan: Number of srcDicts exceeded %d",MAXDICTS);
      CreateBuffer(GetStrArg(),TRUE);
   }
   if (wListFN != NULL) LoadWordList();
   if (pListFN != NULL) LoadPhoneList();
   EditAndMerge();
   if (isLogging)
      PrintLog();
   Exit(0);
   return (0);          /* never reached -- make compiler happy */
}

/* --------------------- Initialisation ----------------------- */

/* SetConfParms: set conf parms relevant to HLEd */
void SetConfParms(void)
{
   int i;
   
   nParm = GetConfig("HDMAN", TRUE, cParm, MAXGLOBS);
   if (nParm>0) {
      if (GetConfInt(cParm,nParm,"TRACE",&i)) trace = i;
   }
}

/* Initialise: confparms and globals */
void Initialise(void)
{
   SetConfParms();
   asterix = GetLabId("*",TRUE);
   cmuId = GetLabId("cmu",TRUE);
   CreateHeap(&memStak, "memStak", MSTAK, 1, 1.4, 10000, 100000);
}

/* ------------------- New Phone Recording -------------------- */

/* LoadPhoneList: load list of phones in pListFN */
void LoadPhoneList(void)
{
   Source src;
   char buf[MAXSTRLEN];
   LabId id;
   
   if(InitSource(pListFN,&src,NoFilter)<SUCCESS)
      HError(1410,"LoadPhoneList: Can't open file %s", pListFN);
   if (trace&T_NPHN)
      printf("Loading predefined phones from file %s\n",pListFN);
   while(ReadString(&src,buf)) {
      if (nDefPhones == MAXPVOC)
         HError(1430,"LoadPhoneList: MAXPVOC exceeded");
      id = defList[nDefPhones++] = GetLabId(buf,TRUE);
      SkipLine(&src);
      id->aux = (Ptr)-1;
   }
   CloseSource(&src);
}

/* PutPhone: if given output phone new then output it to newPhones */
/*           aux = 0, if undef phone, aux = -1 if defined  */
/*           aux = -2, if undef and printed, aux = -3 if defd and printed */
void PutPhone(LabId id)
{
   char buf[80];
   LabId baseId;

   if (((int)id->aux == 0 || (int)id->aux == -1) && newPhones != NULL) {
      fprintf(newPhones,"%s\n",ReWriteString(id->name,NULL,ESCAPE_CHAR));
      /* avoid printing it again */
      id->aux = (Ptr)((int)id->aux - 2);
   }
   strcpy(buf,id->name);
   TriStrip(buf);
   baseId=GetLabId(buf,TRUE);
   if ((int)baseId->aux <= 0 ) {  /* not seen this label before */
      if ((int)baseId->aux == 0 || (int)baseId->aux == -2){
         if (nNewPhones == MAXPVOC)
            HError(1430,"PutPhone: MAXPVOC exceeded");
         newList[nNewPhones++] = baseId;
      }
      baseId->aux = (Ptr)0;            
   }
   baseId->aux = (Ptr)((int)baseId->aux + 1);
}

/* ListNewPhones: list new phones to log file along with counts */
void ListNewPhones(void)
{
   int i,c;

   if (nDefPhones>0){
      fprintf(logF,"Def Phone Usage Counts\n");
      fprintf(logF,"---------------------\n");
      for (i=0; i<nDefPhones; i++) {
         c = (int)defList[i]->aux;
         if (c<0) c=0;
         fprintf(logF," %2d. %-5s : %5d\n",i+1,defList[i]->name,c);
      }
   }
   if (nNewPhones>0){
      fprintf(logF,"New Phone Usage Counts\n");
      fprintf(logF,"---------------------\n");
      for (i=0; i<nNewPhones; i++){
         c = (int)newList[i]->aux;
         if (c<0) c=0;
         fprintf(logF," %2d. %-5s : %5d\n",i+1, newList[i]->name,c);
      }
   }
}

/* ------------------- Load and Print Script ------------------ */

/* PrintIdList: print list of ids */
void PrintIdList(LabId *i)
{
   while (*i != NULL) {
      printf(" %s",(*i)->name);
      ++i;
   }
}

/* PrintScript: prints the given script - for tracing only */
void PrintScript(char *name, DBuffer *db)
{
   ScriptItem *i;
   int j=0;
   LabId src;
   
   i = db->script;
   
   printf("Script %s\n",name);
   if (db->rawMode)
      printf("%2d. Raw input mode\n",++j);
   while (i != NULL) {
      ++j;
      printf("%2d. ",j);
      src = *(i->cmd.args);
      switch (i->cmd.op) {
      case UCWORD:
         printf("Change Words to Upper Case\n"); 
         break;
      case UCPHONE:
         printf("Change Phones to Upper Case\n"); 
         break;
      case LCWORD:
         printf("Change Words to Lower Case\n"); 
         break;
      case LCPHONE:
         printf("Change Phones to Lower Case\n"); 
         break;
      case DELETEW:
         printf("Delete Defs For Words ["); 
         PrintIdList(i->cmd.args); printf("]\n");
         break;
      case DELSOURCE:
         printf("Delete Mult Prons from Source [%s]\n",src->name); 
         break;
      case DELDEF:
         printf("Delete Def For %s = ",src->name); 
         PrintIdList(i->cmd.args+1); printf("\n");
         break;
      case FUNCW:
         printf("Make Function Word Specific Defs ["); 
         PrintIdList(i->cmd.args); printf("]\n");
         break;
      case REPLACEP:
         printf("Replace Phones ["); PrintIdList(i->cmd.args+1); 
         printf("] by %s\n",src->name);
         break;
      case REPLACEW:
         printf("Replace Words ["); PrintIdList(i->cmd.args+1); 
         printf("] by %s\n",src->name);
         break;
      case CONREPLACE:
         printf("Replace Phone %s in context %s_%s", (*(i->cmd.args+2))->name,
                (*(i->cmd.args+1))->name,(*(i->cmd.args+3))->name); 
         printf(" by %s\n",src->name);
         break;
      case MERGEP:
         printf("Merge   ["); PrintIdList(i->cmd.args+1); 
         printf("] to %s\n",src->name);
         break;
      case SPLITP:
         printf("Split %s to [",src->name);
         PrintIdList(i->cmd.args+1); printf("]\n");
         break;
      case APPSIL:
         printf("Append Silences ["); 
         PrintIdList(i->cmd.args); printf("]\n");
         break;
      case REMSTRESS:
         printf("Remove stress marks (mode %s)\n",src->name); 
         break;
      case DELETEP:
         printf("Delete Phones ["); 
         PrintIdList(i->cmd.args); printf("]\n");
         break;
      case LCTXT:
         printf("Left Context ["); PrintIdList(i->cmd.args);
         printf("]\n");
         break;
      case RCTXT:
         printf("Right Context ["); PrintIdList(i->cmd.args);
         printf("]\n");
         break;
      case TCTXT:
         printf("Triphonise ["); PrintIdList(i->cmd.args);
         printf("]\n");
         break;
      default: HError(1450,"PrintScript: Unknown Item number %d",i->cmd.op);
         break;
      }
      i = i->next;
   }
   if (db->numCons>0){
      printf("Contexts:\n");
      for (j=0; j<db->numCons; j++){
         printf("%2d. %s = ",j+1,db->contexts[j].args[0]->name);
         PrintIdList(db->contexts[j].args+1);
         printf("\n");
      }
   }
}

/* ReadCmd: returns next command on input f or NOCMD if none */
EdOp ReadCmd(Source *src)
{
   char cmd[MAXSTRLEN];
   int i;
   EdOp cmdop;
   
   SkipComment(src);
   if (!ReadString(src,cmd))
      return(NOCMD);
   SkipWhiteSpace(src);
   for (i=0,cmdop=UCWORD; i<nCmds; i++,cmdop = (EdOp) (cmdop+1))
      if (strcmp(cmdMap[i],cmd) == 0) return cmdop;
   HError(1450,"ReadCmd: Invalid Command <%s> in file %s",cmd,src->name);
   return NOCMD;
}

/* ReadIdList: Read a list of args and store in argList, return num args */
int ReadIdList(Source *src, LabId *argList)
{
   char buf[MAXSTRLEN];
   int count = 0;
   
   do {
      SkipWhiteSpace(src);
      if (src->wasNewline) break;
      if (!ReadString(src,buf)) break;
      if (count == MAXARGS)
         HError(1430,"ReadIdList: MAXARGS exceeded");
      *argList++ = GetLabId(buf,TRUE); ++count;
   }
   while (TRUE);
   *argList = NULL;
   return count;   
}

/* ReadScript: read a file of cmds and store script in db. If
   it is an input script UW/LW must be applied during reading
   to prevent sort order breaking. */
void ReadScript(char *scriptFn, DBuffer *db, Boolean isInput)
{
   ScriptItem *i, *head, *tail;
   EdOp op;
   Source source;
   FILE *file;

   db->rawMode=FALSE;
   if ((file=fopen(scriptFn,"r"))==NULL){
      if (isLogging)
         fprintf(logF,"WARNING: no script file %s\n",scriptFn);
      db->script = NULL;
      return;
   }
   fclose(file);

   if(InitSource(scriptFn,&source,NoFilter)<SUCCESS)
      HError(1410,"ReadScript: Can't open file %s", scriptFn);
   head = tail = NULL;
   while (TRUE) {
      if ((op = ReadCmd(&source)) == NOCMD) break;
      if (op==DEFCON) {
         if (db->numCons == MAXCONS)
            HError(1430,"ReadScript: MAXCONS exceeded");
         db->contexts[db->numCons].nArgs = 
            ReadIdList(&source,db->contexts[db->numCons].args);
         db->contexts[db->numCons].op = DEFCON;
         ++db->numCons;
      }
      else if (op==RAWMODE) {
         db->rawMode=TRUE;
      }
      else {
         i = (ScriptItem *)New(&memStak,sizeof(ScriptItem));
         i->cmd.op = op;
         i->cmd.nArgs = ReadIdList(&source,i->cmd.args);
         switch (op) {
         case DELETEW:
         case FUNCW:
         case REPLACEP:
         case REPLACEW:
         case MERGEP:
         case SPLITP:
         case DELETEP:
         case DELDEF:
         case APPSIL:
            if (i->cmd.nArgs==0)
               HError(1450,"ReadScript: Command %s with too few args",
                      cmdMap[op]);
            break;
         case CONREPLACE:
            if (i->cmd.nArgs != 4)
               HError(1450,"ReadScript: CR command must have 4 args");
            break;
         case DELSOURCE:
         case REMSTRESS:
         case LCTXT:
         case RCTXT:
            if (i->cmd.nArgs > 1)
               HError(1450,"ReadScript: Command %s has too many args",
                      cmdMap[op]);
            break;
         case TCTXT:
            if (i->cmd.nArgs > 2)
               HError(1450,"ReadScript: TC command has too many args");
            break;
         case UCWORD:
         case LCWORD:
            if (i->cmd.nArgs > 0)
               HError(1450,"ReadScript: %s command has too many args",
                      cmdMap[op]);
            if (isInput)  /* set input preprocessor op */
               db->wop = op;
            break;
         case UCPHONE: 
         case LCPHONE:
            if (i->cmd.nArgs > 0)
               HError(1450,"ReadScript: %s command has too many args",
                      cmdMap[op]);
            break;
         default: break;
         }
         i->next = NULL;
         if (head==NULL) 
            head = tail = i;
         else {
            tail->next = i;
            tail = i;
         }
      }
   }
   CloseSource(&source);
   db->script = head;
   if (trace&T_SCPT)
      PrintScript(scriptFn,db);
}

/* --------------------- Buffer Creation -------------------------- */

/* SkipHeader: skip the dictionary header lines */
void SkipHeader(Source *src, int skipHeaderLines)
{
   int i;
   
   for (i=1; i <= skipHeaderLines; i++)
      SkipLine(src);
}

/* CreateBuffer: initialise an input or output buffer */
void CreateBuffer(char *dName, Boolean isInput)
{
   DBuffer *db;
   char buf[256],scriptFN[256],*src;
   Boolean ReadNextWord(DBuffer *db);

   if (isInput) {
      db = inbuf+nInputs;
      ++nInputs;
      strcpy(buf,dName); strcat(buf,".ded");
      MakeFN(buf,scriptDir,NULL,scriptFN);

      if(InitSource(dName,&db->src,DictFilter)<SUCCESS)
         HError(1410,"CreateBuffer: Can't open file %s", dName);
      SkipHeader(&db->src,db->headSkip);
   }else{
      db = &outbuf;
      db->isPipe = FALSE;
      if (gScriptFN==NULL)
         MakeFN("global.ded",scriptDir,NULL,scriptFN);
      else
         MakeFN(gScriptFN,scriptDir,NULL,scriptFN);
      if (!nullOutput){
         if ((outfile = fopen(dName,"w")) == NULL)
            HError(1411,"CreateBuffer: cannot create dictionary file %s",dName);
      }
   }
   db->numCons = 0; db->wop = NOCMD;
   db->totalWords = db->totalProns = 0;
   db->wordsUsed  = db->pronsUsed = 0;
   ReadScript(scriptFN,db,isInput);
   db->name = NewString(&memStak,strlen(dName));
   strcpy(db->name,dName);
   src = strrchr(db->name,PATHCHAR);
   if (src == NULL) 
      src = db->name;
   else
      ++src;
   db->source = GetLabId(src,TRUE);
   if (isInput)
      if (!ReadNextWord(db))
         HError(1413,"CreateBuffer: cannot read first word in dict %s",dName);
   if (trace&T_TOP)
      printf("%s dictionary %s opened %s\n",isInput?"Source":"Output",
             dName,db->isPipe?"via pipe":"");
}

/* -------------------- Load Word List ---------------------------- */

/* CmpWord: qsort compare routine for word list sorting */
static int CmpWord(const void *p1, const void *p2)
{
   LabId w1, w2;

   w1 = * ((LabId *) p1);
   w2 = * ((LabId *) p2);
   return strcmp(w1->name, w2->name);
}

/* LoadWordList: load a word list from file wListFN */
void LoadWordList(void)
{
   Source src;
   int i;
   char buf[MAXSTRLEN];
   Boolean mustSort = FALSE;
   

   if(InitSource(wListFN,&src,NoFilter)<SUCCESS)
      HError(1410,"LoadWordList: Can't open file %s", wListFN);
   for (nWords=0;ReadString(&src,buf);nWords++) SkipLine(&src);
   CloseSource(&src);
   wList = (LabId *)New(&memStak,sizeof(LabId)*nWords);

   if(InitSource(wListFN,&src,NoFilter)<SUCCESS)
      HError(1410,"LoadWordList: Can't open file %s", wListFN);
   for (i=0;i<nWords;i++) {
      ReadString(&src,buf);
      wList[i] = GetLabId(buf,TRUE);
      if (!mustSort && i>0)  /* check in sort order */
         mustSort = strcmp(wList[i-1]->name,buf) > 0;
      SkipLine(&src);
   }
   CloseSource(&src);
   if (mustSort)  /* word list not in sort order */
      qsort(wList, nWords, sizeof(LabId), CmpWord);
}

/* ------------------- Read/Write Dictionary Entries ------------------ */


/* UCase: convert id to upper case and return new id */
LabId UCase(LabId id)
{
   static char s[255];
   int len,i;

   strcpy(s,id->name);
   len = strlen(s);
   for (i=0;i<len;i++)
      s[i] = toupper(s[i]);
   return(GetLabId(s,TRUE));
}

/* LCase: convert id to lower case and return new id */
LabId LCase(LabId id)
{
   static char s[255];
   int len,i;

   strcpy(s,id->name);
   len = strlen(s);
   for (i=0;i<len;i++)
      s[i] = tolower(s[i]);
   return(GetLabId(s,TRUE));
}

/* IsCommentChar: Test whether c is in commentChars */
Boolean IsCommentChar(int c)
{
   char *s;
 
   for (s = commentChars; *s != '\0'; s++)
      if (c == *s) return TRUE;
   return FALSE;
}

/* SetCase: of given string in s */
static void SetCase(EdOp cmd, char *s)
{
   int len,i;

   len = strlen(s);
   for (i=0;i<len;i++)
      switch(cmd) {
      case UCWORD:
         s[i] = toupper(s[i]);
         break;
      case LCWORD:
         s[i] = tolower(s[i]);
         break;
      default:
         HError(1450, "SetCase: invalid case op %d",cmd);
      }
}

/* ReadNextWord: read next word, return FALSE if EOF */
Boolean ReadNextWord(DBuffer *db)
{
   static LabId labels[MAXPHONES+3];
   char buf[MAXSTRLEN];
   char *ptr;
   float prob=1.0,p=-1.0,v;
   int ch,i,n,len;

   db->nextWord=NULL;

   while ((ch = GetCh(&db->src)) != EOF) {
      if (!IsCommentChar(ch)) break;
      SkipLine(&db->src);
   }
   if (ch == EOF) return FALSE;
   UnGetCh(ch,&db->src);

   if (db->rawMode) {
      if(!ReadRawString(&db->src,buf)) 
         return FALSE;
      if (db->wop != NOCMD)   /* make any case changes before merging */
         SetCase(db->wop,buf);
      labels[0]=GetLabId(buf,TRUE);
      labels[1]=NULL;n=0;
      SkipWhiteSpace(&db->src);
      while (!db->src.wasNewline) {
         if (!ReadRawString(&db->src,buf))
            HError(1451,"ReadNextWord: Phone or outsym expected in word %s",
                   labels[0]->name);
         len = strlen(buf);
         if (buf[0] == '[' && buf[len-1] == ']') {    /* outsym */
            if (labels[1]!=NULL) 
               HError(1451,"ReadNextWord: Only single outsym allowed for word %s",
                      labels[0]->name);
            buf[len-1] = '\0';
            labels[1] = GetLabId(buf+1,TRUE);
         } else {
            if (n==0 && p<0) v=strtod(buf,&ptr);
            else v=0.0,ptr=buf;
            if (ptr!=buf) {
               if (v<=0.0 || v>1.0 || *ptr!=0) 
                  HError(8050,"ReadDict: Probability malformed %s",buf);
               prob=p=v;
            }
            else {
               if (n==MAXPHONES)
                  HError(1451,"ReadNextWord: Too many phones in word %s",
                         labels[0]->name);
               labels[2+n++] = GetLabId(buf,TRUE);
            }
         }
         SkipWhiteSpace(&db->src);
      }
      labels[n+2] = NULL;
   }
   else {
      if(ReadDictWord(&db->src,labels,&prob,&n)<SUCCESS)
         HError(1413, "ReadNextWord: ReadDictWord failed");
      if (n<0) return FALSE;
      if (db->wop != NOCMD) {
         switch(db->wop) {
         case UCWORD:
            labels[0] = UCase(labels[0]);
            break;
         case LCWORD:
            labels[0] = LCase(labels[0]);
            break;
         default:
            HError(1450, "ReadNextWord: invalid case op %d",db->wop);
         }
      }
   }

   db->nextWord=labels[0];
   db->pbuf.source=db->source;
   if (labels[1]!=NULL) 
      db->nextOutSym=labels[1];
   else 
      db->nextOutSym=labels[0];
   db->pbuf.phone[0]=wdBnd;
   for (i=0;i<n;i++)
      db->pbuf.phone[i+1]=labels[i+2];
   db->pbuf.phone[n+1]=wdBnd;
   db->pbuf.nPhone=n+2;
   db->pbuf.prob=prob;

   return TRUE;
}

/* WriteEntry: write out a dictionary entry and update newPhones */
void WriteEntry(FILE *f, LabId word, LabId outsym, Pronunciation *p, int margin, Boolean findNew)
{
   int i,st,en;
   char buf[256],m[20];
   
   if (p->nPhone == 0) return;
   if (!nullOutput) {
      if (word != NULL)
         strcpy(buf,ReWriteString(word->name,NULL,ESCAPE_CHAR));
      else
         strcpy(buf,"");
      for (i=0; i<margin; i++) m[i] = ' '; m[i]='\0';
      fprintf(f,"%s%-15s",m,buf);
      if (incOutSyms) {
         strcpy(buf,"["); 
         strcat(buf,ReWriteString(outsym->name,NULL,ESCAPE_CHAR)); 
         strcat(buf,"]");
         fprintf(f," %-15s",buf);
      }
      if (incProbs) {
         if (p->prob<=1.0)
            fprintf(f," %8.6f",p->prob);
         else
            fprintf(f,"         ");
      }
   }
   st = 1;
   if  (p->phone[0] != wdBnd){
      st = 0;
      if (isLogging && margin==0)
         fprintf(logF,"WARNING: no left word bnd in word %s\n",word->name);
   }
   en = p->nPhone-1;
   if (p->phone[en] != wdBnd){
      ++en;
      if (isLogging && margin==0)
         fprintf(logF,"WARNING: no right word bnd in word %s\n",word->name);
   }      
   for (i=st; i<en; i++) { 
      if (!nullOutput)
         fprintf(f," %s",ReWriteString((p->phone[i])->name,NULL,ESCAPE_CHAR));
      if (findNew)
         PutPhone(p->phone[i]); 
   }
   if (tagSources && !nullOutput)
      fprintf(f,"   [%s]",ReWriteString(p->source->name,NULL,ESCAPE_CHAR));
   if (!nullOutput) fprintf(f,"\n");
}

/* ----------------- Read/Write Dictionary Words --------------- */

/* ReadDictProns: read entries for next word, return FALSE if none found */
Boolean ReadDictProns(DBuffer *db)
{
   LabId thisWord;
   Boolean ok;
   int i = 0;

   if (db->nextWord == NULL) return FALSE;
   if (db->wbuf.word != NULL && strcmp(db->wbuf.word->name,db->nextWord->name) > 0 )
      HError(1452,"ReadDictProns: word %s out of order in dict %s",
             db->nextWord->name,db->name);
   db->wbuf.word = thisWord = db->nextWord;
   db->wbuf.outsym = db->nextOutSym;
   db->wbuf.pron[i++]=db->pbuf;
   ok = ReadNextWord(db);
   while (ok && db->nextWord == thisWord){
      if (i == MAXPRONS)
         HError(1430,"ReadDictProns: max prons exceeded in word %s from dict %s",
                thisWord->name,db->name);
      db->wbuf.pron[i++]=db->pbuf;
      ok = ReadNextWord(db);
   }
   db->wbuf.nPron = i;
   ++db->totalWords;
   db->totalProns += i;
   return TRUE;
}

/* WriteDictWord: write current word to output file */
void WriteDictWord(DBuffer *db, FILE *f, int margin, Boolean findNew)
{
   int i;

   if (f==NULL) return;
   for (i=0; i<db->wbuf.nPron; i++)
      WriteEntry(f,db->wbuf.word,db->wbuf.outsym,db->wbuf.pron+i,margin, findNew);
}

void ShowDB(DBuffer *db, char * title)
{
   printf("%s - DB %s: word %s, next %s\n",
          title,
          db->name,
          db->wbuf.word==NULL?"NULL":db->wbuf.word->name,
          db->nextWord==NULL ?"NULL":db->nextWord->name);
   fflush(stdout);
}

/* --------------------------- Editing ------------------------- */

/* IsInIdList: return true if id is in idlist */
Boolean IsInIdList(LabId id, LabId *idlist)
{
   while (*idlist != NULL) {
      if (id == *idlist) return TRUE;
      ++idlist;
   }
   return FALSE;
}

/* UCPhoneOp: Upper Case Phone command */
void UCPhoneOp(WordBuf *wb)
{
   int i,j;
   Pronunciation *p;
   
   for (i=0;i<wb->nPron; i++){
      p = wb->pron+i;
      for (j=0; j<p->nPhone; j++)
         p->phone[j] = UCase(p->phone[j]);
   }
}

/* LCPhoneOp: Lower Case Phone command */
void LCPhoneOp(WordBuf *wb)
{
   int i,j;
   Pronunciation *p;
   
   for (i=0; i<wb->nPron; i++){
      p = wb->pron+i;
      for (j=0; j<p->nPhone; j++)
         p->phone[j] = LCase(p->phone[j]);
   }
}

/* DeleteWordOp: Delete Word Command - marks by setting nPron to zero */
void DeleteWordOp(WordBuf *wb, LabId *args)
{
   if (IsInIdList(wb->word,args))
      wb->nPron = 0;
}

/* DeleteSourceOp: del source if more than one */
int DeleteSourceOp(WordBuf *wb, LabId *args)
{
   int i,j,k,del=0;
   Pronunciation *p, *s, *t;
   
   if (wb->nPron < 2) return 0;
   for (i=0; i<wb->nPron; i++){
      p = wb->pron+i;
      if (p->source == *args){
         if (trace&T_DSOP)
            printf("Removing pron %s from word %s\n",
                   (*args)->name,wb->word->name);
         --wb->nPron;
         ++del;
         for (j=i; j<wb->nPron; j++){
            t = wb->pron+j; s = t+1;
            t->nPhone = s->nPhone;
            t->source = s->source;
            t->prob = s->prob;
            for (k=0; k<t->nPhone; k++)
               t->phone[k] = s->phone[k];
         }
      }
   }
   return del;
}

/* DelDefOp: Delete Word Command - marks by setting nPron to zero */
void DelDefOp(WordBuf *wb, LabId *args)
{
   int i,j,idx=0;
   Boolean found = FALSE;
   Pronunciation *p;
   LabId *ph;
   char buf[256];

   if (wb->word == *args){
      for (i=0; !found && i<wb->nPron; i++){
         found = TRUE; idx = i; p = wb->pron+i; ph = args+1;
         for (j=1; *ph != NULL && j<p->nPhone; j++,ph++){
            if (p->phone[j] != *ph){
               found = FALSE; break;
            }
         }
      }
      if (found){
         if (trace&T_DWOP){
            buf[0] = '\0';
            for (ph=args; *ph!=NULL; ph++){
               strcat(buf," "); strcat(buf,(*ph)->name);
            }
            printf("      deleting definition %s\n",buf);
         }
         for (j=idx+1; j<wb->nPron; j++)
            wb->pron[j-1] = wb->pron[j];
         --wb->nPron;
      }else{
         buf[0] = '\0';
         for (ph=args; *ph!=NULL; ph++){
            strcat(buf," "); strcat(buf,(*ph)->name);
         }
         HError(-1431,"DelDefOp: pron %s not found",buf);
      }
   }
}

/* FunctionWordOp: Function Word Command - makes fn word phones of form W.A */
void FunctionWordOp(WordBuf *wb, LabId *args)
{
   int i,j;
   static char s[255];
   Pronunciation *p;
   
   if (IsInIdList(wb->word,args)) 
      for (i=0; i<wb->nPron; i++){
         p = wb->pron+i;
         for (j=1; j<p->nPhone-1; j++){
            sprintf(s,"%s.%s",(wb->word)->name,(p->phone[j])->name);
            p->phone[j] = GetLabId(s,TRUE);
         }
      }
}  

/* ReplacePhoneOp: Replace Phone Command */
void ReplacePhoneOp(WordBuf *wb, LabId *args)
{
   int i,j;
   Pronunciation *p;
   
   for (i=0; i<wb->nPron; i++){
      p = wb->pron+i;
      for (j=0; j<p->nPhone; j++)
         if (IsInIdList(p->phone[j],args+1)) 
            p->phone[j] = *args;
   }
} 

/* ReplaceWordOp: Replace Word Command */
void ReplaceWordOp(WordBuf *wb, LabId *args)
{
   if (IsInIdList(wb->word,args+1)) {
      if (wb->word == wb->outsym)
         wb->word = wb->outsym = *args;
      else
         wb->word = *args;
   }
} 

/* GetContextList: extract context list for id if any */   
LabId *GetContextList(LabId id, DBuffer *db)
{
   int i;
   LabId *list;

   for (i=0; i<db->numCons; i++){
      list = db->contexts[i].args;
      if (id == *list) return list+1;
   }
   return NULL;
}

/* ContextRep: do context replace on given pronunciation - note 
   * matches any context */
void ContextRep(Pronunciation *p, LabId *args, DBuffer *db)
{
   LabId lc,rc,cc;
   Boolean ltrue, replace;
   LabId *lcList,*rcList;
   int i;
   
   lc = *(args+1); cc = *(args+2); rc = *(args+3);
   lcList = GetContextList(lc,db);
   rcList = GetContextList(rc,db);
   for (i=1; i<p->nPhone-1; i++) {
      replace = FALSE;
      if (p->phone[i] == cc) {
         if (lcList != NULL)
            ltrue = IsInIdList(p->phone[i-1],lcList);
         else
            ltrue = (lc == asterix || lc == p->phone[i-1] );
         if (ltrue){
            if (rcList != NULL)
               replace = IsInIdList(p->phone[i+1],rcList);
            else
               replace = (rc == asterix || rc == p->phone[i+1] );
         }
      }
      if (replace) p->phone[i] = *args;
   }  
}

/* ContextReplaceOp: ContextReplace Command */
void ContextReplaceOp(WordBuf *wb, LabId *args, DBuffer *db)
{
   int i;
   Pronunciation *p;
   
   for (i=0; i<wb->nPron; i++){
      p = wb->pron+i;
      ContextRep(p,args,db);
   }
}        

/* SeqMatch: Returns true if the nMerge sequence of LabIds 
             are the same in list1 and list2 */
Boolean SeqMatch(int nMerge, LabId *list1, LabId *list2)
{
   int i;

   for (i=0; i<nMerge; i++,list1++,list2++)
      if (*list1 != *list2)
         return FALSE;
   return TRUE;
}
   
/* MergePhon: merge all occs of phone sequence in args */
void MergePhon(Pronunciation *p, int nArgs, LabId *args)
{
   int i,j,nMerge;
   
   nMerge = nArgs-1;
   for (i=0;i<=p->nPhone-nMerge;i++)
      if (SeqMatch(nMerge,p->phone+i,args+1)) {
         p->phone[i] = *args;
         for (j=i+1;j<p->nPhone;j++)
            p->phone[j] = p->phone[j+nMerge-1];
         p->nPhone -= nMerge-1;    
      }
}


/* MergePhoneOp: MergePhone Command */
void MergePhoneOp(WordBuf *wb, int nArgs, LabId *args)
{
   int i;
   Pronunciation *p;
   
   for (i=0; i<wb->nPron; i++){
      p = wb->pron+i;
      MergePhon(p,nArgs,args);
   }
}        

/* SplitPhon: split a phone sequence into multiple phones */
void SplitPhon(Pronunciation *p, int nArgs, LabId *args)
{
   int i,j,nSplit,nExtra;
   
   nSplit = nArgs-1;
   nExtra = nSplit-1;
   for (i=0;i<p->nPhone;)
      if (p->phone[i] == args[0]) {
         if (p->nPhone+nExtra > MAXPHONES)
            HError(1430,"SplitPhon: Too many phones to split");
         for (j=p->nPhone-1+nExtra; j>=i+1; j--)
            p->phone[j] = p->phone[j-nExtra];
         for (j=0; j<nSplit; j++)
            p->phone[i+j] = args[j+1];
         i += nExtra+1;
         p->nPhone += nExtra;
      }
      else i++;
}            

/* SplitPhoneOp: SplitPhone Command */
void SplitPhoneOp(WordBuf *wb, int nArgs, LabId *args)
{
   int i;
   Pronunciation *p;
   
   for (i=0; i<wb->nPron; i++){
      p = wb->pron+i;
      SplitPhon(p,nArgs,args);
   }
}        

/* DeletePhoneOp: DeletePhone Command */
void DeletePhoneOp(WordBuf *wb, LabId *args)
{
   int i,j,k;
   Pronunciation *p;
   
   for (k=0; k<wb->nPron; k++){
      p = wb->pron+k;
      for (i=0;i<p->nPhone;)
         if (IsInIdList(p->phone[i],args)) {
            for (j=i+1;j<p->nPhone;j++)
               p->phone[j-1] = p->phone[j]; 
            p->nPhone--;
         }
         else
            i++;
   }
}

/* AppendPhone: append id to given pronunciation */
void AppendPhone(Pronunciation *p, LabId id)
{
   if (p->nPhone == MAXPHONES)
      HError(1430,"AppendPhone: MAXPHONES exceeded");
   p->phone[p->nPhone-1] = id;
   p->phone[p->nPhone++] = wdBnd;
}

/* DuplicatePron: duplicate selected pron and insert at end of wbuf */
void DuplicatePron(WordBuf *wb, int i)
{
   int j;
   Pronunciation *s, *t;

   if (wb->nPron == MAXPRONS)
      HError(1430,"DuplicatePron: MAXPRONS exceeded");
   s = wb->pron+i;
   t = wb->pron+wb->nPron;
   t->source = s->source;
   t->nPhone = s->nPhone;
   t->prob = s->prob;
   for (j=0; j<s->nPhone; j++) t->phone[j] = s->phone[j];
   ++wb->nPron;   
}

/* AppendSilenceOp: AppendSilences Command */
void AppendSilenceOp(WordBuf *wb, LabId *args)
{
   LabId *i;
   int j,n,k;

   n = wb->nPron-1;
   for (j=n; j>=0; j--){
      for (i=args+1; *i != NULL; i++){
         DuplicatePron(wb,j);
         k = wb->nPron-1;
         AppendPhone(wb->pron+k,*i);
      }
      AppendPhone(wb->pron+j, *args);
   }
}

/* MakeTriId:  concatenate args separated by - and +'s and return its id */
LabId MakeTriId(LabId l, LabId c, LabId r)
{
   char buf[100];
   LabId item;
   
   if (l!=NULL && l!=wdBnd && c!=wdBnd){
      strcpy(buf,l->name); strcat(buf,"-"); 
      strcat(buf,c->name);
   } else 
      strcpy(buf,c->name);
   if (r!=NULL && r!=wdBnd && c!=wdBnd){
      strcat(buf,"+"); strcat(buf,r->name);
   }
   item = GetLabId(buf,TRUE);
   return item;
}

/* TriPhon: convert phone labels in pronunciation to either
      left, right or triphone (ie both) contexts */
void TriPhon(Pronunciation *p, Boolean left, Boolean right, 
             LabId stId, LabId enId)
{
   int i;
   LabId leftId, centreId, rightId;
   
   if (p->nPhone <= 2)
      HError(1430,"TriPhon: cant add context to null phone");
   leftId =left?stId:NULL;    /* First Phone */
   centreId = p->phone[1];
   rightId = right?enId:NULL;
   if (p->nPhone > 3 && right)
      rightId = p->phone[2];
   p->phone[1] = MakeTriId(leftId,centreId,rightId);
   for (i=2;i<p->nPhone-2;i++) {         /* Middle Phones */
      leftId =left?centreId:NULL;
      centreId = p->phone[i];
      rightId = right?p->phone[i+1]:NULL;
      p->phone[i] = MakeTriId(leftId,centreId,rightId);
   }
   if (p->nPhone > 3) {  
      leftId = left?centreId:NULL;     /* Last Phone */
      centreId = p->phone[p->nPhone-2];
      rightId = right?enId:NULL;
      p->phone[p->nPhone-2] = MakeTriId(leftId,centreId,rightId);
   }
}

/* Triphonise: given word buffer */
void Triphonise(WordBuf *wb,  Boolean left, Boolean right, 
                LabId stId, LabId enId)
{
   int i;
   Pronunciation *p;
   
   for (i=0; i<wb->nPron; i++){
      p = wb->pron+i;
      TriPhon(p,left,right,stId,enId);
   }
}        

/* LCtxtOp: Left context command */
void LCtxtOp(WordBuf *wb, int nArgs,LabId *args)
{
   LabId x;
   
   x = nArgs==1?*args:NULL;
   Triphonise(wb,TRUE,FALSE,x,x);
}

/* RCtxtOp: Right context command */
void RCtxtOp(WordBuf *wb, int nArgs,LabId *args)
{
   LabId x;
   
   x = nArgs==1?*args:NULL;
   Triphonise(wb,FALSE,TRUE,x,x);
}

/* TCtxtOp: Left and Right context command */
void TCtxtOp(WordBuf *wb, int nArgs,LabId *args)
{
   LabId l,r;
   
   if (nArgs == 1)
      l = r = *args;
   else if (nArgs == 2){
      l = *args; r = *(args+1);
   } else
      l = r = NULL;
   Triphonise(wb,TRUE,TRUE,l,r);
}

/* RemStress: remove stress marks from all phones */
void RemStress(WordBuf *wb, LabId *args)
{
   LabId mode = *args;
   int i,j,k;
   Pronunciation *p;
   char buf[50];
   
   if (mode != cmuId)
      HError(1450,"RemStress: Unknown stress marking %s",mode->name);
   for (k=0; k<wb->nPron; k++){
      p = wb->pron+k;
      for (i=0; i<p->nPhone; i++)
         if (mode == cmuId) {
            strcpy(buf,p->phone[i]->name);
            j = strlen(buf) - 1;
            if (isdigit((int) buf[j])){
               buf[j] = '\0';
               p->phone[i] = GetLabId(buf,TRUE);
            }
         }
   }
}

/* EditWordBuf: apply each script command to given DBuffer */ 
void EditWordBuf(DBuffer *db)
{
   ScriptItem *i;
   WordBuf *wb;

   if (db->script == NULL) return;
   wb = &(db->wbuf);
   if (trace&T_EDW0) {
      printf(" editing word %s in dict %s\n",(wb->word)->name,db->name);
   }
   if (trace&T_EDW1) {
      printf("   before:\n"); 
      if (db->wbuf.nPron>0 ) 
         WriteDictWord(db,stdout,4, FALSE);
      else
         printf("    ** deleted **\n");
   }
   i = db->script;
   while (i != NULL && wb->nPron > 0) {
      switch (i->cmd.op) {
      case UCWORD:  /* done on input */
         if (db->wop == NOCMD) /* must be output edit */
            wb->word = UCase(wb->word);
         break;
      case LCWORD:
         if (db->wop == NOCMD) /* must be output edit */
            wb->word = LCase(wb->word);
         break;
      case UCPHONE:
         UCPhoneOp(wb); break;
      case LCPHONE:
         LCPhoneOp(wb); break;
      case DELETEW:
         DeleteWordOp(wb,i->cmd.args); break;
      case DELSOURCE:
         db->pronsUsed -= DeleteSourceOp(wb,i->cmd.args); 
         break;
      case DELDEF:
         DelDefOp(wb,i->cmd.args); break;
      case FUNCW:
         FunctionWordOp(wb,i->cmd.args); break;
      case REPLACEP:
         ReplacePhoneOp(wb,i->cmd.args); break;
      case REPLACEW:
         ReplaceWordOp(wb,i->cmd.args); break;
      case CONREPLACE:
         ContextReplaceOp(wb,i->cmd.args,db); break;
      case MERGEP:
         MergePhoneOp(wb,i->cmd.nArgs,i->cmd.args); break;
      case SPLITP:
         SplitPhoneOp(wb,i->cmd.nArgs,i->cmd.args); break;
      case DELETEP:
         DeletePhoneOp(wb,i->cmd.args); break;
      case APPSIL:
         AppendSilenceOp(wb,i->cmd.args); break;
      case LCTXT:
         LCtxtOp(wb,i->cmd.nArgs,i->cmd.args); break;
      case RCTXT:
         RCtxtOp(wb,i->cmd.nArgs,i->cmd.args); break;
      case TCTXT:
         TCtxtOp(wb,i->cmd.nArgs,i->cmd.args); break;
      case REMSTRESS:
         RemStress(wb,i->cmd.args); break;
      default: break;
      }
      if (trace&T_EDW1) {
         printf("      after %s: ",cmdMap[i->cmd.op]);
         WriteDictWord(db,stdout,6, FALSE);
      }
      i = i->next;
   }
   if (trace&T_EDW0) {
      printf("   after:\n"); 
      if (db->wbuf.nPron>0 ) 
         WriteDictWord(db,stdout,4, FALSE);
      else
         printf("    ** deleted **\n");
   }
}

/* ----------------------- Log Information  --------------------------- */

void PrintUsage(DBuffer *db)
{
   char buf[50];

   NameOf(db->name,buf); buf[12] = '\0';
   fprintf(logF,"%12s %9d %10d %10d %10d\n", buf, db->totalWords, db->wordsUsed,
           db->totalProns, db->pronsUsed);
}

/* PrintLog: output log information to log file */
void PrintLog(void)
{
   int i;

   fprintf(logF,"\nDictionary Usage Statistics\n");
   fprintf(logF,"---------------------------\n");
   fprintf(logF,"  Dictionary    TotalWords WordsUsed  TotalProns PronsUsed\n");
   for (i=0; i<nInputs; i++)
      PrintUsage(inbuf+i);
   PrintUsage(&outbuf);
   fprintf(logF,"\n");
   fprintf(logF,"%d words required, %d missing\n\n", numOut,numMissing);
   ListNewPhones();
   fprintf(logF,"\n");
   if (!nullOutput)
      fprintf(logF,"Dictionary %s created\n",outbuf.name);
}

/* ----------------------- Edit and Merge  --------------------------- */

/* SetActiveCount: scan inputs and count num still active */
void SetActiveCount(void)
{
   int i,sum;

   for (i=0,sum=0; i<nInputs; i++)
      if (inbuf[i].nextWord != NULL) ++sum;
   numActive = sum;
}

/* HighestInput: return alphabetically highest next word */
LabId HighestInput(void)
{
   int i, hi;
   LabId best,next;

   i = 0; 
   while ((next=inbuf[i].nextWord) == NULL) i++;
   hi = i++; best = next;
   while (i<nInputs){
      if ((next=inbuf[i].nextWord) != NULL)
         if (strcmp(next->name,best->name) < 0){
            best = next; hi = i;
         }
      i++;
   }
   return best;
}

/* ScanDict: scan thru given dictionary until required word is found or
   dictionary is exhausted.  Dicts are sorted so stop as soon as next
   word is alphabetically after required word. If found then return true
   and load required word in wordbuf. */
Boolean ScanDict(DBuffer *db, LabId reqd)
{
   int scmp;

   if (db->nextWord == NULL) return FALSE;
   scmp = strcmp(db->nextWord->name,reqd->name);
   while (scmp<0){
      ReadDictProns(db);
      if (db->nextWord == NULL) return FALSE;
      scmp = strcmp(db->nextWord->name,reqd->name);
   }
   if (scmp==0)
      ReadDictProns(db);
   return scmp==0;
}

/* FillInputs: scan inputs until current word in each is >= required word.
   If no wordlist is given then the required word is the alphabetically
   highest ranked word across all the inputs.  If a wordlist is given
   then it is just the next word in the list. Returns false when all
   inputs or wordlist is exhausted */
Boolean FillInputs(Boolean *valid)
{
   int i;

   if (wList != NULL){
      if (widx == nWords) return FALSE;
      required = wList[widx++];
   } else 
      required = HighestInput();
   for (i=0; i<nInputs; i++)
      valid[i] = ScanDict(inbuf+i,required);
   SetActiveCount();
   return TRUE;
}

/* CopyWordBuf: copy word buf in s to target t */
void CopyWordBuf(DBuffer *s, DBuffer *t)
{
   WordBuf *ws,*wt;
   Pronunciation *ps,*pt;
   int i,j;
      
   ++t->totalWords;
   if (s->wbuf.nPron == 0) return;
   ++t->wordsUsed;
   ++s->wordsUsed;
   s->pronsUsed += s->wbuf.nPron;
   t->totalProns  += s->wbuf.nPron;
   t->pronsUsed   += s->wbuf.nPron;
   ws = &(s->wbuf); wt = &(t->wbuf);
   if (trace&T_WBUF){
      printf(" copy word %-15s in %s to %s\n",
             ws->word->name,s->name,t->name); fflush(stdout);
   }
   wt->nPron = ws->nPron;
   wt->word = ws->word;
   wt->outsym = ws->outsym;
   for (i=0; i<ws->nPron; i++){
      ps = ws->pron+i; pt = wt->pron+i;
      pt->nPhone = ps->nPhone;
      pt->source = ps->source;
      pt->prob = ps->prob;
      for (j=0; j<ps->nPhone; j++)
         pt->phone[j] = ps->phone[j];
   }
}

/* AppendWordBuf: append word buf in s to target t */
void AppendWordBuf(DBuffer *s, DBuffer *t)
{
   WordBuf *ws,*wt;
   Pronunciation *ps,*pt;
   int i,j,k;

   if (s->wbuf.nPron == 0) return;
   ++s->wordsUsed; ++t->totalWords;
   s->pronsUsed += s->wbuf.nPron;
   t->totalProns   += s->wbuf.nPron;
   ws = &(s->wbuf); wt = &(t->wbuf);
   if (trace&T_WBUF)
      printf(" appd word %-15s in %s to %s\n",
             ws->word->name,s->name,t->name);
   j = wt->nPron;
   wt->nPron += ws->nPron;
   if (wt->nPron > MAXPRONS)
      HError(1430,"AppendWordBuf: MAXPRONS exceeded");
   if (wt->word != ws->word)
      HError(1490,"AppendWordBuf: words differ %s vs %s",
             ws->word->name,wt->word->name);
   for (i=0; i<ws->nPron; i++,j++){
      ps = ws->pron+i; pt = wt->pron+j;
      pt->nPhone = ps->nPhone;
      pt->source = ps->source;
      pt->prob = ps->prob;
      for (k=0; k<ps->nPhone; k++)
         pt->phone[k] = ps->phone[k];
   }
}

/* HasDuplicate: return true if given pron has preceding duplicate */
Boolean HasDuplicate(DBuffer *db, int pronNum)
{
   Pronunciation *s, *t;
   int i,k;
   Boolean found = FALSE;

   s = db->wbuf.pron+pronNum;
   for (i=0; !found && i<pronNum; i++) {
      t = db->wbuf.pron+i;
      if (t->nPhone != s->nPhone) continue;
      found = TRUE;
      for (k=1; found && k<t->nPhone-1; k++)
         if (t->phone[k] != s->phone[k]) found = FALSE;
   }
   return found;
}

/* RemDuplicates: remove duplicate pronunciations from buffer */
void RemDuplicates(DBuffer *db)
{
   int i,j,k;
   Pronunciation *s, *t;

   for (i=db->wbuf.nPron-1; i>0; i--) {
      if (HasDuplicate(db,i)){
         if (trace&T_WBUF)
            printf(" removing dup %d from word %s in %s\n",i,
                   db->wbuf.word->name,db->name);
         --db->wbuf.nPron;
         --db->pronsUsed;
         for (j=i; j<db->wbuf.nPron; j++){
            t = db->wbuf.pron+j; s = t+1;
            t->nPhone = s->nPhone;
            t->source = s->source;
            t->prob = s->prob;
            for (k=0; k<t->nPhone; k++)
               t->phone[k] = s->phone[k];
         }
      }
   }
}

/* EditAndMerge: main processing loop */
void EditAndMerge(void)
{
   Boolean valid[MAXDICTS];
   int i;

   SetActiveCount();
   while(numActive > 0  && FillInputs(valid)){
      ++numOut;
      if (trace&T_WBUF) printf("\n%d:\n",numOut);
      if (trace&T_VALI){
         printf(" valid inputs [");
         for (i=0; i<nInputs; i++)
            if (valid[i]) printf(" %d",i);
         printf("]\n");
      }
      i=0;
      while (!valid[i] && i<nInputs) i++;
      if (i<nInputs){
         EditWordBuf(inbuf+i);
         CopyWordBuf(inbuf+i,&outbuf); 
         i++;
         if (mergeProns){
            while (i<nInputs){
               if (valid[i]){
                  EditWordBuf(inbuf+i);
                  AppendWordBuf(inbuf+i,&outbuf);
               }
               ++i;  
            }
         }
         RemDuplicates(&outbuf);
         EditWordBuf(&outbuf);
         RemDuplicates(&outbuf);
         WriteDictWord(&outbuf, outfile, 0, TRUE);
      } else {
         if (isLogging){
            if (numMissing==0){
               fprintf(logF,"Missing Words\n");
               fprintf(logF,"-------------\n");
            }
            fprintf(logF,"%s\n",required->name);
         }
         ++numMissing;
         if (trace&T_WBUF)
            printf("%s missing\n",required->name);
      }
   }
   if (trace&T_TOP) {
      if (!nullOutput)
         printf("Dictionary %s created - ",outbuf.name);
      printf("%d words processed, %d missing\n", numOut,numMissing);
   }
}

/* ---------------------------------------------------------------- */
/*                         END:  HDMan.c                            */
/* ---------------------------------------------------------------- */
