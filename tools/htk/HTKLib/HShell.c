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
/*         File: HShell.c:   Interface to the Shell            */
/* ----------------------------------------------------------- */

char *hshell_version = "!HVER!HShell:   3.4.1 [CUED 12/03/09]";
char *hshell_vc_id = "$Id: HShell.c,v 1.1.1.1 2006/10/11 09:54:58 jal58 Exp $";

#include "HShell.h"

#ifdef WIN32
#include <windows.h>
#include <mmsystem.h>
#include <fcntl.h>
#endif

#ifdef UNIX
#include <sys/ioctl.h>
#endif

/* ------------------------ Trace Flags --------------------- */

static int trace = 0;
#define T_IOP   0002       /* i/o input via FOpen */
#define T_EXF   0004       /* extended file name processing */

/* --------------------- Global Variables ------------------- */

static Boolean infoPrinted = FALSE;      /* set when -A -B or -V is used */
static Boolean abortOnError = FALSE;     /* causes HError to abort */
static Boolean printVersionInfo = FALSE; /* request version info */
static Boolean showConfig = FALSE;       /* show configuration params */
static Boolean noNumEscapes = FALSE;     /* Prevent writing in \012 format */
static Boolean natReadOrder = FALSE;     /* Preserve natural mach read order*/
static Boolean natWriteOrder = FALSE;    /* Preserve natural mach write order*/
static Boolean extendedFileNames = TRUE; /* allow extended file names */

/* Global variable indicating VAX-order architecture for storing numbers */
Boolean vaxOrder = FALSE;

#define MAXEFS 5                        /* max num ext files to remember */

typedef struct {                        /* extended file name */
   char logfile[1024];                  /* logical name */
   char actfile[1024];                  /* actual file name */
   long stindex;                        /* start sample to extract */
   long enindex;                        /* end sample to extract */
}ExtFile;

static ExtFile extFiles[MAXEFS];        /* circ buf of ext file names */
static int extFileNext = 0;             /* next slot to save into */
static int extFileUsed = 0;             /* total ext files in buffer */

/* ------------- Extended File Name Handling ---------------- */

/* EXPORT->RegisterExtFileName: record details of fn exts if any in circ buffer */
char * RegisterExtFileName(char *s)
{
   char *eq,*rb,*lb,*co;
   char buf[1024];
   ExtFile *p;

   if (!extendedFileNames)
      return s;

   strcpy(buf,s);
   eq = strchr(buf,'=');
   lb = strchr(buf,'[');
   if (eq == NULL && lb == NULL) 
      return s;

   if (trace&T_EXF)
      printf("Ext File Name: %s\n",buf);

   p = extFiles+extFileNext;
   ++extFileNext; 
   if (extFileNext==MAXEFS) 
      extFileNext=0;
   if (extFileUsed < MAXEFS) 
      ++extFileUsed;
   p->stindex = p->enindex = -1;

   if (lb!=NULL) {
      if ((co = strchr(buf,',')) == NULL)
         HError(5024,"RegisterExtFileName: comma missing in index spec");
      if ((rb = strchr(buf,']')) == NULL)
         HError(5024,"RegisterExtFileName: ] missing in index spec");
      *rb = '\0'; p->enindex = atol(co+1);
      *co = '\0'; p->stindex = atol(lb+1);
      *lb = '\0';
   }

   if (eq!=NULL) {
      strcpy(p->actfile,eq+1); *eq = '\0';
      strcpy(p->logfile,buf);
   } else {
      strcpy(p->logfile,buf);
      strcpy(p->actfile,buf);
   }

   if (trace&T_EXF) {
      printf("%s=%s", p->logfile, p->actfile);
      if(p->stindex >=0) 
         printf("[%ld,%ld]", p->stindex, p->enindex);
      printf("\n");
   }

   return p->logfile;
}

/* GetFileNameExt: return true if given file has extensions and return
   the extend info.  The problem with this routine is that the logical
   name can be repeated in the buffer.  This is normally handled by 
   comparing the pointer rather than the string itself.  However, if
   the application copies the logical file name, this would break.
   Hence, if the pointer is not there, the name is searched for
   going back in time.  If the name is found and it occurs more
   than once, a warning is printed.
*/
Boolean GetFileNameExt(char *logfn, char *actfn, long *st, long *en)
{
   int i, noccs;
   ExtFile *p;
   Boolean found = FALSE;
   Boolean ambiguous = FALSE;

   /* First count number of times logfn occurs in buffer */
   noccs = 0;
   for (i=0,p=extFiles; i<extFileUsed; i++,p++){
      if (strcmp(logfn,p->logfile) == 0 ) 
         ++noccs;
   }
   if (noccs==0) 
      return FALSE;

   /* Try to find the logfn, by pointer first */
   for (i=0,p=extFiles; i<extFileUsed && !found; i++){
      if (logfn == p->logfile) 
         found = TRUE; 
      else 
         p++;
   }

   if (!found) {   /* look for actual name */
      if (noccs>1) 
         ambiguous = TRUE;
      p = extFiles + extFileNext;
      for (i=0; i<extFileUsed && !found; i++){
         if (p==extFiles) 
            p += MAXEFS;
         else
            p--;
         if (strcmp(logfn,p->logfile) == 0 ) 
            found = TRUE;
      }
   }

   if (!found) 
      return FALSE;

   /* Copy back info and warn if ambiguous */
   strcpy(actfn,p->actfile);
   *st = p->stindex; *en = p->enindex;
   if (trace&T_EXF)
      printf("%sFile Ext found: %s=%s[%ld,%ld]\n",
             (ambiguous) ? "Ambiguous " : "",
             logfn, actfn, *st, *en);
   if (ambiguous)
      HError(-1,"GetFileNameExt: ambiguous extended file name %s=%s[%ld,%ld]",
             logfn, actfn, *st, *en);

   return TRUE;
}


/* --------------------- Version Display -------------------- */

typedef struct _VersionEntry{
   char *ver;
   char *sccs;
   struct _VersionEntry *next;
}VersionEntry;

static VersionEntry *vInfoHd = NULL;  /* head of version info list */
static VersionEntry *vInfoTl = NULL;  /* tail of version info list */

/* EXPORT->Register:module name with HTK version info ver and sccs info */
void Register(char *ver, char *sccs)
{
   VersionEntry *v;

   v = (VersionEntry *)malloc(sizeof(VersionEntry));
   v->ver  = (char *)malloc(strlen(ver)+1);
   strcpy(v->ver,ver);
   v->sccs = (char *)malloc(strlen(sccs)+1);
   strcpy(v->sccs,sccs);
   v->next = NULL;
   if (vInfoTl==NULL) vInfoHd = v; else vInfoTl->next = v; 
   vInfoTl = v;
}

/* PrVInfo: print version info */
static void PrVInfo(char *s,char *sccs)
{
   char buf[MAXSTRLEN];
   char *name,*ver,*who,*date,*p;
   
   strcpy(buf,s);
   if ((p=strrchr(buf,']')) == NULL)
      HError(5070,"PrVInfo: no ']' in %s",s);
   *p = '\0';
   if ((p=strrchr(buf,'[')) == NULL)
      HError(5070,"PrVInfo: no '[' in %s",s);
   who = p+1; *p = '\0';
   if (strlen(who) <8)
      HError(5070,"PrVInfo: who/date field too short in %s",s);
   if ((p=strrchr(who,' ')) == NULL)
      HError(5070,"PrVInfo: no space in who/date field in %s",s);
   date = p+1; *p = '\0';
   if ((p=strrchr(buf,'!')) == NULL)
      HError(5070,"PrVInfo: no '!' in %s",s);
   name = p+1;
   if ((p=strchr(name,':')) == NULL)
      HError(5070,"PrVInfo: no ':' in %s",s);
   ver = p+1; *p = '\0';
   while (*ver == ' ') ++ver;
   if ((p=strchr(ver,' ')) != NULL) *p = '\0';
   printf("%-10s %-10s %-6s %-9s : %s\n",name, ver, who, date, sccs);
}

/* EXPORT->InfoPrinted: true if info printed by Shell */
Boolean InfoPrinted(void)
{
   VersionEntry *v;

   if (printVersionInfo) {
      printf("\nHTK Version Information\n");
      PrVInfo("!HVER!Module:  Version [Who Date]","CVS Info");
      for (v = vInfoHd; v != NULL; v=v->next)
         PrVInfo(v->ver,v->sccs); 
      printf("\n");
   }
   return infoPrinted;
}

/* ------------- Configuration Parameter File Handling --------------- */

/* 
   A configuration file consists of a sequence of parameter declarations
   of the form
   
   [USER:]PARAM_NAME = VALUE
      
   If included, USER indicates that the parameter is only visible to
   the module or tool of the same name.  VALUE is an integer, float or
   string.  A string is any sequence of nonblank characters, or any
   sequence inside double quotes.Otherwise, the parameter is global.
   Unless it appears inside a string, a hash (#) indicates that the
   rest of the line is a comment.
*/

typedef struct _ConfigEntry{
   ConfParam param;
   struct _ConfigEntry *next;
}ConfigEntry;

static int numConfigParms = 0;
static ConfigEntry *confList = NULL;
static char *cfkmap[] = { 
   "StrCKind","IntCKind","FltCKind","BoolCKind","AnyCKind"
};

/* ReadConfName: read module or paramname field and cvt to ucase */
static Boolean ReadConfName(Source *src, char *s)
{
   int i,c;

   while (isspace(c=GetCh(src)));
   if (c == EOF) return FALSE;
   for (i=0; i<MAXSTRLEN ; i++){
      if (c == EOF || isspace(c) || !isalnum(c)){
         if (c==':' || c=='=') UnGetCh(c,src);
         s[i] = '\0';
         return TRUE;
      }
      s[i] = toupper(c); c = GetCh(src);
   }     
   return FALSE;
}

/* FindConfEntry: return entry with given name and user */
static ConfigEntry *FindConfEntry(char *user, char *name)
{
   ConfigEntry *e;
   char *s;
   
   for (e=confList; e!= NULL; e=e->next)
      if (strcmp(e->param.name,name)==0){
         s = e->param.user;
         if (s==NULL?user==NULL:(user!=NULL && strcmp(s,user)==0) )
            return e;
      }
   return NULL;
}

/* NumHead: returns TRUE if the first two chars of a string 
            are ('+'|'-') digit | digit */
static Boolean NumHead(char *s)
{
   if (*s!='\0')
      if (isdigit((int) *s))
         return TRUE;
   if (((*s=='-') || (*s=='+')) && (isdigit((int) *(s+1))))
      return TRUE;
   return FALSE;
}

/* ParseInclude: skip comments or return #include argument */
static char *ParseComment(Source *src,char *name)
{
   const char comch = '#';
   int c;
   char buf[MAXSTRLEN];

   c = GetCh(src);
   while (c!=EOF && (isspace(c) || c==comch)) {
      if (c==comch) {
         src->wasNewline=FALSE;
         SkipWhiteSpace(src);
         if(src->wasNewline){
            c = GetCh(src);
            continue;
         }
         if (ReadString(src,buf) && (strcmp(buf,"include")==0)) {
            if (ReadString(src,name)) {
               SkipLine(src);
               return name;
            }
         }
         SkipLine(src);
      }
      c = GetCh(src);
   }
   UnGetCh(c,src);

   return NULL;
}

/* ReadConfigFile: read the given configuration file */
static ReturnStatus ReadConfigFile(char *fname)
{
   double x;
   Source src;
   ConfigEntry *e;
   Boolean gotParam,hasUser;
   char c,*s,buf[32],sbuf[MAXSTRLEN];
   char user[MAXSTRLEN],name[MAXSTRLEN],value[MAXSTRLEN];
   static int recurse = 0;

   if (recurse++ > 15){ 
      HRError(5050,"ReadConfigFile: max #include depth reached (%s)",fname);
      recurse--;
      return(FAIL);
   }
   
   if(InitSource(fname,&src,NoFilter)<SUCCESS){
      HRError(5010,"ReadConfigFile: Can't open file %s", fname);
      return(FAIL);
   }
   
   /* skip comments and parse #include */
   while (ParseComment(&src,name)!=NULL) {
      PathOf(name,sbuf);
      if (*sbuf=='\0') PathOf(fname,sbuf);
      strcat(sbuf,name);
      if(ReadConfigFile(sbuf)<SUCCESS){
         recurse--;
         return(FAIL);
      }
   }
   hasUser=FALSE;
   gotParam = ReadConfName(&src,name);
   while (gotParam) {
      while (isspace((int) (c=GetCh(&src))));  
      if (c==':') {  /* user field given */
         hasUser = TRUE; strcpy(user,name);
         if (!ReadConfName(&src,name)){
            HRError(5050,"ReadConfigFile: param name expected %s",
                    SrcPosition(src,buf));
            recurse--;
            return(FAIL);
         }
         while (isspace((int) (c=GetCh(&src))));  
      }
      if (c != '='){
         HRError(5050,"ReadConfigFile: = expected %s",
                 SrcPosition(src,buf));
         recurse--;
         return(FAIL);
      }
      if (!ReadString(&src,value)){
         HRError(5050,"ReadConfig: parameter value expected %s",
                 SrcPosition(src,buf));
         recurse--;
         return(FAIL);
      }
      e = FindConfEntry(hasUser?user:NULL,name);
      if (e==NULL){ /* new param */
         e = (ConfigEntry *) malloc(sizeof(ConfigEntry));
         e->next = confList; confList = e;
         e->param.seen = FALSE;
         ++numConfigParms;
      }
      if (hasUser){
         e->param.user = (char *) malloc(strlen(user)+1);
         strcpy(e->param.user,user);
      }else
         e->param.user = NULL;   
      e->param.name = (char *) malloc(strlen(name)+1);
      strcpy(e->param.name,name);
      if (strcmp(value,"T")==0 || strcmp(value,"TRUE")==0) {
         e->param.kind = BoolCKind; e->param.val.b = TRUE;
      } else 
         if (strcmp(value,"F")==0 || strcmp(value,"FALSE")==0) {
            e->param.kind = BoolCKind; e->param.val.b = FALSE;
         } else 
            if (NumHead(value)){
               x = strtod(value,&s);
               if (s==NULL || *s == '\0'){
                  if (strchr(value,'.') == NULL){
                     e->param.kind = IntCKind; 
                     e->param.val.i = strtol(value,NULL,0);
                  }else{
                     e->param.kind = FltCKind; 
                     e->param.val.f = x;
                  }
               }
            } else {
               e->param.kind = StrCKind; 
               e->param.val.s = (char *) malloc(strlen(value)+1);
               strcpy(e->param.val.s,value);
            }
      /* skip comments and parse #include */
      while (ParseComment(&src,name)!=NULL) {
         PathOf(name,sbuf);
         if (*sbuf=='\0') PathOf(fname,sbuf);
         strcat(sbuf,name);
         if(ReadConfigFile(sbuf)<SUCCESS){
            recurse--;
            return(FAIL);
         }
      }
      hasUser=FALSE;
      gotParam = ReadConfName(&src,name);
   }
   recurse--;
   return(SUCCESS);
}

/* EXPORT PrintConfig: print the current config params */
void PrintConfig(void)
{
   ConfigEntry *e;
   
   printf("\n");
   if (numConfigParms==0)
      printf("No HTK Configuration Parameters Set\n");
   else {
      printf("HTK Configuration Parameters[%d]\n",numConfigParms);
      printf("  %-14s  %-14s  %16s\n","Module/Tool","Parameter","Value");
      for (e=confList; e!= NULL; e=e->next){
         printf("%c %-14s  %-14s  ",(e->param.seen?' ':'#'),
                e->param.user==NULL?"":e->param.user,e->param.name);
         switch(e->param.kind){
         case StrCKind:  printf("%16s",e->param.val.s); break;
         case BoolCKind: printf("%16s",e->param.val.b?"TRUE":"FALSE"); break;
         case IntCKind:  printf("%16d",e->param.val.i); break;
         case FltCKind:  printf("%16f",e->param.val.f); break;
         }
         printf("\n");
      }
   }
   printf("\n");
}

/* EXPORT->GetConfig: return a list of selected config values */
int GetConfig(char *user, Boolean incGlob, ConfParam **list, int max)
{
   ConfigEntry *e;
   int found = 0;

   /* Specific ones first */
   if (user!=NULL)
      for (e=confList; e!= NULL; e=e->next){
         if (e->param.user!=NULL && strcmp(e->param.user,user)==0) {
            ++found;
            if (list != NULL) {
               if (found>max) 
                  HError(5071,"GetConfig: more than %d for user %s%s",
                         max,user==NULL?"<null>":user,incGlob?"with globals":"");
               *list++ = &e->param;
            }
         }
      }
   /* Then append globals */
   if (incGlob || user==NULL)
      for (e=confList; e!= NULL; e=e->next){
         if (e->param.user==NULL) {
            ++found;
            if (list != NULL) {
               if (found>max) 
                  HError(5071,"GetConfig: more than %d for user %s%s",
                         max,user==NULL?"<null>":user,incGlob?"with globals":"");
               *list++ = &e->param;
            }
         }
      }
   return found;
}

/* FindConfParm: return index of conf parameter with given name and kind*/
static int FindConfParm(ConfParam **list,int size,char *name,ConfKind kind)
{
   int i;
   
   for (i=0; i<size; i++)
      if (strcmp(list[i]->name,name)==0){
         if (kind != AnyCKind && list[i]->kind != kind && 
             !(list[i]->kind==IntCKind && kind==FltCKind))
            HError(5072,"FindConfParm: %s is %s but should be type %s",
                   name,cfkmap[list[i]->kind],cfkmap[kind]);
         list[i]->seen=TRUE;
         return i;
      }
   return -1;
}

/* EXPORT->HasConfParm: true if parameter exists with given name */
Boolean HasConfParm(ConfParam **list, int size, char *name)
{
   return (FindConfParm(list,size,name,AnyCKind) != -1);
}

/* EXPORT->GetConfStr: return string parameter with given name */
Boolean GetConfStr(ConfParam **list,int size,char *name,char *str)
{
   int i;
   
   if ((i = FindConfParm(list,size,name,StrCKind)) != -1){
      strcpy(str,list[i]->val.s);
      return TRUE;
   }
   return FALSE;
}

/* EXPORT->GetConfBool: return Boolean parameter with given name */
Boolean GetConfBool(ConfParam **list,int size,char *name, Boolean *b)
{
   int i;
   
   if ((i = FindConfParm(list,size,name,BoolCKind)) != -1){
      *b = list[i]->val.b;
      return TRUE;
   }
   return FALSE;
}

/* EXPORT->GetConfInt: return integer parameter with given name */
Boolean GetConfInt(ConfParam **list,int size,char *name, int *ival)
{
   int i;
   
   if ((i = FindConfParm(list,size,name,IntCKind)) != -1){
      *ival = list[i]->val.i;
      return TRUE;
   }
   return FALSE;
}

/* EXPORT->GetConfFlt: return float parameter with given name */
Boolean GetConfFlt(ConfParam **list,int size,char *name, double *fval)
{
   int i;
   
   if ((i = FindConfParm(list,size,name,FltCKind)) != -1){
      *fval = (list[i]->kind==FltCKind)?list[i]->val.f:list[i]->val.i;
      return TRUE;
   }
   return FALSE;
}


/* ------------------ Argument Processing -------------------- */

/*
  Arguments are processed in sequence by calls to Getxxx().
  The enum type ArgKind is used to indicate the kind of the next
  argument 
  
  SWITCHARG - any str starting with '-' char
  FLOATARG  - any str which is a valid number and contains a '.'
  INTARG    - any str which is a valid number and is not a FLOATARG
  STRINGARG - any str in double quotes or not one of the above 
  NOARG     - returned when all arguments processed
     
  Each of the Getxxx() functions will return value of next arg in 
  form specified by xxx, an error is reported if next arg is not of 
  the required type except that an int will be converted to a float 
  if requested.  The list of args is actually a copy from which the
  standard HShell options -C and -S have been removed.
   
  If a script file is set, then the contents of the script file
  are effectively appended to the command line.  However, the
  actual strings returned for args in the script are volatile and
  should be copied by the host program where necessary.

*/

static int argcount;          /* total args = argc */
static int nextarg=1;         /* next arg to return in GetxxxArg */
static char *defargs[2]={ "<Uninitialised>", "" };
static char **arglist=defargs;/* actual arg list */
static FILE *script = NULL;   /* script file if any */
static int scriptcount = 0;   /* num words in script */
static char scriptBuf[256];   /* buffer for current script arg */
static Boolean scriptBufLoaded = FALSE;
static Boolean wasQuoted;     /* true if next arg was quoted */
static ConfParam *cParm[MAXGLOBS];      /* config parameters */
static int nParm = 0;

/* ScriptWord: return next word from script */
static char * ScriptWord(void)
{
   int ch,qch,i;
   
   i=0; ch=' ';
   while (isspace(ch)) ch = fgetc(script);
   if (ch==EOF) return NULL;
   if (ch=='\'' || ch=='"'){
      qch = ch;
      ch = fgetc(script);
      while (ch != qch && ch != EOF) {
         scriptBuf[i++] = ch;           /* #### ge: should check for overflow of scriptBuf */
         ch = fgetc(script);
      }
      if (ch==EOF)
         HError(5051,"ScriptWord: Closing quote missing in script file");
      wasQuoted = TRUE;
   } else {
      do {
         scriptBuf[i++] = ch; 
         ch = fgetc(script);
      }while (!isspace(ch) && ch != EOF);
      wasQuoted = FALSE;
   }
   scriptBuf[i] = '\0';
   scriptBufLoaded = TRUE;
   return scriptBuf;
}
         
/* GetNextArg: from either command line or script file */
static char * GetNextArg(Boolean step)
{
   char *s;
   
   if (argcount>nextarg){
      s = arglist[nextarg];
      wasQuoted = FALSE;
   } else if (scriptBufLoaded)
      s = scriptBuf;
   else
      s = ScriptWord();
   if (step) {
      ++nextarg;
      scriptBufLoaded = FALSE;
   }
   return s;
}

/* EXPORT->NumArgs: Number of actual cmd line arguments left unprocessed */
int  NumArgs(void)
{
   return argcount+scriptcount-nextarg;
}

/* EXPORT->NextArg: Kind of next argument */
ArgKind NextArg(void)
{
   char *s,*p;
   
   if (NumArgs() == 0) return NOARG;
   s = GetNextArg(FALSE);
   if (wasQuoted)
      return STRINGARG;
   if (NumHead(s)){
      strtod(s,&p);
      if (p==NULL || *p == '\0'){
         if (strchr(s,'.') == NULL)
            return INTARG;
         else
            return FLOATARG;
      }
   }
   if (*s=='-') 
      return SWITCHARG;
   return STRINGARG;
}

/* ArgError: called when next arg is different to requested arg 
   or no more args are left */
static void ArgError(char *s)
{
   if (NextArg()==NOARG)
      HError(5020,"ArgError: %s arg requested but no args left",s);
   else
      HError(5021,"ArgError: %s arg requested but next is %s",s,GetNextArg(FALSE));
}

/* EXPORT->GetStrArg: get string arg */
char * GetStrArg(void)
{
   char *s;

   if (NextArg() != STRINGARG) 
      ArgError("String");

   s = GetNextArg (TRUE);
   if (extendedFileNames)
      s = RegisterExtFileName (s);

   return s;
}
   
/* EXPORT->GetSwtArg: get switch arg */
char * GetSwtArg(void)
{
   if (NextArg() != SWITCHARG) 
      ArgError("Switch");
   return GetNextArg(TRUE)+1;
}

/* EXPORT->GetIntArg: get int arg */
int GetIntArg(void)
{
   int i;
   char *s;
   
   if (NextArg() != INTARG) 
      ArgError("Int");
   s = GetNextArg(TRUE);
   if (sscanf(s,"%i",&i) !=1)
      HError(5090,"GetIntArg: Integer Argument %s",s);
   return i;
}

/* EXPORT->GetLongArg: get long arg */
long GetLongArg(void)
{
   long i;
   char *s;
   
   if (NextArg() != INTARG) 
      ArgError("int");
   s = GetNextArg(TRUE);
   if (sscanf(s,"%li",&i) !=1)
      HError(5090,"GetLongArg: Integer Argument %s",s);
   return i;
}

/* EXPORT->GetFltArg: get float arg */
float GetFltArg(void)
{
   int k = NextArg();
   
   if (k != INTARG && k != FLOATARG) 
      ArgError("Float");
   return atof(GetNextArg(TRUE));

}

/* EXPORT->GetChkedInt: range checked version of GetIntArg */
int GetChkedInt(int min, int max, char * swtname)
{
   int val;
   
   if (NextArg() != INTARG) 
      HError(5021,"GetChkedInt: Integer Arg Required for %s option",swtname);
   val=GetIntArg();
   if (val<min || val>max) 
      HError(5022,"GetChkedInt: Integer arg out of range in %s option",swtname);
   return val;
}

/* EXPORT->GetChkedLong: range checked version of GetIntArg */
long GetChkedLong(long min, long max, char * swtname)
{
   long val;
   
   if (NextArg() != INTARG) 
      HError(5021,"GetChkedLong: Integer Arg Required for %s option",swtname);
   val=GetLongArg();
   if (val<min || val>max) 
      HError(5022,"GetChkedLong: Long arg out of range in %s option",swtname);
   return val;
}

/* EXPORT->GetChkedFlt: range checked version of GetFltArg */
float GetChkedFlt(float min, float max, char * swtname)
{
   float val;
   
   if (NextArg() != INTARG && NextArg() != FLOATARG) 
      HError(5021,"GetChkedFlt: Float Arg Required for %s option",swtname);
   val=GetFltArg();
   if (val<min || val>max) 
      HError(5022,"GetChkedFlt: Float arg out of range in %s option",swtname);
   return val;
}

/* EXPORT->GetIntEnvVar: get integer environment variable */
Boolean GetIntEnvVar(char *envVar, int *value)
{
   char *env;
   
   if ((env = getenv(envVar)) == NULL) return FALSE;
   *value = atoi(env);
   return TRUE;
}

static char *CheckFn(char *fn);

/* SetScriptFile: open script file and count words in it */

ReturnStatus SetScriptFile(char *fn)
{
   CheckFn(fn);
   if ((script = fopen(fn,"r")) == NULL){  /* Don't care if text/binary */
      HRError(5010,"SetScriptFile: Cannot open script file %s",fn);
      return(FAIL);
   }
   while (ScriptWord() != NULL) ++scriptcount;
   rewind(script);
   scriptBufLoaded = FALSE;
   return(SUCCESS);
}

/* -------------------- Input Files/Pipes -------------------- */

static char *filtermap[] = {
   "HWAVEFILTER", "HPARMFILTER", "HLANGMODFILTER", "HMMLISTFILTER", 
   "HMMDEFFILTER", "HLABELFILTER", "HNETFILTER", "HDICTFILTER", 
   "LGRAMFILTER",  "LWMAPFILTER",  "LCMAPFILTER", "LMTEXTFILTER", 
   "HNOFILTER",
   "HWAVEOFILTER", "HPARMOFILTER", "HLANGMODOFILTER", "HMMLISTOFILTER", 
   "HMMDEFOFILTER", "HLABELOFILTER", "HNETOFILTER", "HDICTOFILTER",
   "LGRAMOFILTER",  "LWMAPOFILTER",  "LCMAPOFILTER", 
   "HNOOFILTER"
};

/* FilterSet: returns true and puts filter cmd in s if configuration
              parameter or environment variable set */
static Boolean FilterSet(IOFilter filter, char *s)
{
   char *env;
 
   if ((env = getenv(filtermap[filter])) != NULL){
      strcpy(s,env);
      return TRUE;
   } else if (GetConfStr(cParm,nParm,filtermap[filter],s))
      return TRUE;
   else
      return FALSE;
}

/* EXPORT->SubstFName: subst fname for $, if any, in s */
void SubstFName(char *fname, char *s)
{
   char *p;
   char buf[1028];

   while ((p=strchr(s,'$')) != NULL){
      *p = '\0'; ++p;
      strcpy(buf,s); strcat(buf,fname); strcat(buf,p);
      strcpy(s,buf);
   }
}

static int maxTry = 1;

#ifdef WIN32
#define popen _popen
#define pclose _pclose
#endif

/* EXPORT->FOpen: return either a file or a pipe */
FILE *FOpen(char *fname, IOFilter filter, Boolean *isPipe)
{
   FILE *f;
   int i;
   Boolean isInput;
   char mode[8],cmd[1028];

   if (filter <= NoFilter){ /* then input */
      isInput = TRUE;
      strcpy(mode,"r"); /* May be binary */
   } else {
      isInput = FALSE;
      strcpy(mode,"w"); /* May be binary */
   }
   
#ifndef NOPIPES
   if (FilterSet(filter,cmd)){
      SubstFName(fname,cmd);
      f = (FILE *)popen(cmd,mode);
      *isPipe = TRUE;
      if (trace&T_IOP)
         printf("HShell: FOpen - file %s %s pipe %s\n",
                fname,isInput?"<-":"->",cmd);
      return f;
   }
#endif
   *isPipe = FALSE; strcat(mode,"b");
   for (i=1; i<=maxTry; i++){
      f = fopen(fname,mode);
      if (f!=NULL) return f;
#ifdef UNIX
      if (i<maxTry) sleep(5);
#endif
      if (trace&T_IOP)
         printf("HShell: FOpen - try %d failed on %s in mode %s\n",
                i,fname,mode);
   }
   return NULL;
}

/* EXPORT->FClose: close the given file or pipe */
void FClose(FILE *f, Boolean isPipe)
{
#ifndef NOPIPES
   if (isPipe){
      pclose(f);
      return;
   }
#endif
   if (fclose(f) != 0)
      HError (5010, "FClose: closing file failed");
}



/* EXPORT->InitSource: initialise a source */
ReturnStatus InitSource(char *fname, Source *src,  IOFilter filter)
{
   CheckFn(fname);
   strcpy(src->name,fname);
   if ((src->f = FOpen(fname, filter, &(src->isPipe))) == NULL){
      HRError(5010,"InitSource: Cannot open source file %s",fname);
      return(FAIL);
   }
   src->pbValid = FALSE;
   src->chcount = 0;
   return(SUCCESS);
}

/* EXPORT->AttachSource: attach a source to a file */
void AttachSource(FILE *file, Source *src)
{
   src->f=file;
   strcpy(src->name,"attachment");
   src->isPipe=TRUE;
   src->pbValid = FALSE;
   src->chcount = 0;
}

/* EXPORT->CloseSource: close a source */
void CloseSource(Source *src)
{
   FClose(src->f,src->isPipe);
}

/* EXPORT->SrcPosition: return string giving position in src */
char *SrcPosition(Source src, char *s)
{
   int i,line,col,c;
   long pos;

   if (src.isPipe || src.chcount>100000)
      sprintf(s,"char %d in %s",src.chcount,src.name);
   else{
      pos = ftell(src.f); rewind(src.f);
      for (line=1,col=0,i=0; i<=pos; i++){
         c = fgetc(src.f);
         if (c == '\n'){
            ++line; col = 0;
         } else
            ++col;
      }
      sprintf(s,"line %d/col %d/char %ld in %s",
              line, col, pos, src.name);
   }
   return s;
}

/* EXPORT->GetCh: get next character from given source */
int GetCh(Source *src)
{
   int c;
   
   if (!src->pbValid){
      c = fgetc(src->f);  ++src->chcount;
   } else{
      c = src->putback; src->pbValid = FALSE;
   }
   return c;
}

/* EXPORT->UnGetCh: return given character to given source */
void UnGetCh(int c, Source *src)
{
   if (src->pbValid == TRUE) {
      ungetc(src->putback,src->f);
      src->chcount--;
   }
   src->putback = c;  src->pbValid = TRUE;
}

/* EXPORT->SkipLine: skip to next line in source */
Boolean SkipLine(Source *src)
{
   int c;
   
   c = GetCh(src);
   while (c != EOF && c != '\n') c = GetCh(src);
   return(c!=EOF);
}

/* EXPORT->ReadLine: read to next newline in source */
Boolean ReadLine(Source *src,char *s)
{
   int c;
   
   c = GetCh(src);
   while (c != EOF && c != '\n') *s++=c,c=GetCh(src);
   *s=0;
   return(c!=EOF);
}

/* EXPORT->ReadUntilLine: read to next occurrence of string */
void ReadUntilLine (Source *src, char *s)
{
   char buf[20*MAXSTRLEN];
   
   do {
      if (!ReadLine (src, buf))
         HError (5013, "ReadUntilLine: reached EOF while scanning for '%s'", s);
   } while (strcmp (buf, s) != 0);
}

/* EXPORT->SkipComment: skip comment if any */
void SkipComment(Source *src)
{
   const char comch = '#';
   int c;
   
   c = GetCh(src);
   while (c != EOF && (isspace(c) || c == comch)) {
      if (c == comch)
         while (c != EOF && c != '\n') c = GetCh(src);
      c = GetCh(src);
   }
   UnGetCh(c,src);
}

/* EXPORT->SkipWhiteSpace: skip white space if any */
void SkipWhiteSpace(Source *src)
{
   int c;
   
   c=GetCh(src);
   UnGetCh(c,src);
   if (!isspace(c))
      return; /* Does not alter wasNewline! */
   src->wasNewline=FALSE;
   do {
      c=GetCh(src);
      if (c=='\n') src->wasNewline=TRUE;
   } while(c != EOF && isspace(c));
   if (c==EOF) src->wasNewline=TRUE;
   UnGetCh(c,src);
}

/* EXPORT->ParseString: get next string from src and store it in s */
char *ParseString(char *src, char *s)
{
   Boolean wasQuoted;
   int c,q;

   wasQuoted=FALSE; *s=0; q=0;
   while (isspace((int) *src)) src++;
   if (*src == DBL_QUOTE || *src == SING_QUOTE){
      wasQuoted = TRUE; q = *src;
      src++;
   }
   else if (*src==0) return(NULL);
   while(!wasQuoted || *src!=0) {
      if (wasQuoted) {
         if (*src == q) return (src+1);
      } else {
         if (*src==0 || isspace((int) *src)) return (src);
      }
      if (*src==ESCAPE_CHAR) {
         src++;
         if (src[0]>='0' && src[1]>='0' && src[2]>='0' &&
             src[0]<='7' && src[1]<='7' && src[2]<='7') {
            c = 64*(src[0] - '0') + 8*(src[1] - '0') + (src[2] - '0');
            src+=2;
         } else
            c = *src;
      }
      else c = *src;
      *s++ = c;
      if (c==0) break;
      *s=0;
      src++;
   }
   return NULL;
}

/* EXPORT->ReadString: get next string from src and store it in s */
Boolean ReadString(Source *src, char *s){  
  /* could be just: return ReadStringWithLen(src,s,MAXSTRLEN); but this is called often so do it like this.. */
   int i,c,n,q;

   src->wasQuoted=FALSE;
   q=0;
   while (isspace(c=GetCh(src)));
   if (c == EOF) return FALSE;
   if (c == DBL_QUOTE || c == SING_QUOTE){
      src->wasQuoted = TRUE; q = c;
      c = GetCh(src);
   }
   for (i=0; i<MAXSTRLEN; i++){
      if (src->wasQuoted){
         if (c == EOF)
            HError(5013,"ReadString: File end within quoted string");
         if (c == q) {
            s[i] = '\0';
            return TRUE;
         }
      } else {
         if (c == EOF || isspace(c)){
            UnGetCh(c,src);
            s[i] = '\0';
            return TRUE;
         }
      }
      if (c==ESCAPE_CHAR) {
         c = GetCh(src); if (c == EOF) return(FALSE);
         if (c>='0' && c<='7') {
            n = c - '0'; 
            c = GetCh(src); if (c == EOF || c<'0' || c>'7') return(FALSE);
            n = n*8 + c - '0'; 
            c = GetCh(src); if (c == EOF || c<'0' || c>'7') return(FALSE);
            c += n*8 - '0';
         }
      }
      s[i] = c; c = GetCh(src);
   }     
   HError(5013,"ReadString: String too long");
   return FALSE;
}  


/* EXPORT->ReadStringWithLen: get next string from src and store it in s */
Boolean ReadStringWithLen(Source *src, char *s, int buflen)
{
   int i,c,n,q;

   src->wasQuoted=FALSE;
   q=0;
   while (isspace(c=GetCh(src)));
   if (c == EOF) return FALSE;
   if (c == DBL_QUOTE || c == SING_QUOTE){
      src->wasQuoted = TRUE; q = c;
      c = GetCh(src);
   }
   for (i=0; i<buflen ; i++){
      if (src->wasQuoted){
         if (c == EOF)
            HError(5013,"ReadString: File end within quoted string");
         if (c == q) {
            s[i] = '\0';
            return TRUE;
         }
      } else {
         if (c == EOF || isspace(c)){
            UnGetCh(c,src);
            s[i] = '\0';
            return TRUE;
         }
      }
      if (c==ESCAPE_CHAR) {
         c = GetCh(src); if (c == EOF) return(FALSE);
         if (c>='0' && c<='7') {
            n = c - '0'; 
            c = GetCh(src); if (c == EOF || c<'0' || c>'7') return(FALSE);
            n = n*8 + c - '0'; 
            c = GetCh(src); if (c == EOF || c<'0' || c>'7') return(FALSE);
            c += n*8 - '0';
         }
      }
      s[i] = c; c = GetCh(src);
   }     
   HError(5013,"ReadStringWithLen: String too long");
   return FALSE;
}

/* EXPORT->ReadRawString: get next raw string (i.e. word) from src and store it in s */
/* ReadRawString: ie ignore normal HTK escaping */
Boolean ReadRawString(Source *src, char *s)
{
   int i,c;

   while (isspace(c=GetCh(src)));
   if (c == EOF) return FALSE;
   for (i=0; i<MAXSTRLEN ; i++){
      if (c == EOF || isspace(c)){
         UnGetCh(c,src);
         s[i] = '\0';
         return TRUE;
      }
      s[i] = c; c = GetCh(src);
   }     
   HError (5013, "ReadRawString: String too long");
   return FALSE;
}

/* EXPORT->WriteString: Write string s in readable format */
void WriteString(FILE *f,char *s,char q)
{
   Boolean noSing,noDbl;
   int n;
   unsigned char *p;

   if (s[0]!=SING_QUOTE) noSing=TRUE;
   else noSing=FALSE;
   if (s[0]!=DBL_QUOTE) noDbl=TRUE;
   else noDbl=FALSE;
   
   if (q!=ESCAPE_CHAR && q!=SING_QUOTE && q!=DBL_QUOTE) {
      q=0;
      if (!noDbl || !noSing) {
         if (noSing && !noDbl) q=SING_QUOTE;
         else q=DBL_QUOTE;
      }
   }
   if (q>0 && q!=ESCAPE_CHAR) fputc(q,f);
   for (p=(unsigned char*)s;*p;p++) {
      if (*p==ESCAPE_CHAR || *p==q ||
          (q==ESCAPE_CHAR && p==(unsigned char*)s && 
           (*p==SING_QUOTE || *p==DBL_QUOTE)))
         fputc(ESCAPE_CHAR,f),fputc(*p,f);
      else if (isprint(*p) || noNumEscapes) fputc(*p,f);
      else {
         n=*p;
         fputc(ESCAPE_CHAR,f);
         fputc(((n/64)%8)+'0',f);fputc(((n/8)%8)+'0',f);fputc((n%8)+'0',f);
      }
   }
   if (q>0 && q!=ESCAPE_CHAR) fputc(q,f);
}

/* EXPORT->ReWriteString: Convert string s to writeable format */
char *ReWriteString(char *s,char *dst, char q)
{
   static char stat[MAXSTRLEN*4];
   Boolean noSing,noDbl;
   int n;
   unsigned char *p,*d;

   if (dst==NULL) d=(unsigned char*)(dst=stat);
   else d=(unsigned char*)dst;

   if (s[0]!=SING_QUOTE) noSing=TRUE;
   else noSing=FALSE;
   if (s[0]!=DBL_QUOTE) noDbl=TRUE;
   else noDbl=FALSE;
   
   if (q!=ESCAPE_CHAR && q!=SING_QUOTE && q!=DBL_QUOTE) {
      q=0;
      if (!noDbl || !noSing) {
         if (noSing && !noDbl) q=SING_QUOTE;
         else q=DBL_QUOTE;
      }
   }
   if (q>0 && q!=ESCAPE_CHAR) *d++=q;
   for (p=(unsigned char*)s;*p;p++) {
      if (*p==ESCAPE_CHAR || *p==q ||
          (q==ESCAPE_CHAR && p==(unsigned char*)s && 
           (*p==SING_QUOTE || *p==DBL_QUOTE)))
         *d++=ESCAPE_CHAR,*d++=*p;
      else if (isprint(*p) || noNumEscapes) *d++=*p;
      else {
         n=*p;
         *d++=ESCAPE_CHAR;
         *d++=((n/64)%8)+'0';*d++=((n/8)%8)+'0';*d++=(n%8)+'0';
      }
   }
   if (q>0 && q!=ESCAPE_CHAR) *d++=q;
   *d=0;
   return(dst);
}

/* IsVAXOrder: returns true if machine has VAX ordered bytes */
static Boolean IsVAXOrder(void)
{
   short x, *px;
   unsigned char *pc;
   
   px = &x;
   pc = (unsigned char *) px;
   *pc = 1; *(pc+1) = 0;         /* store bytes 1 0 */
   return x==1;          /* does it read back as 1? */
}

/* SwapInt32: swap byte order of int32 data value *p */
void SwapInt32(int32 *p)
{
   char temp,*q;
   
   q = (char*) p;
   temp = *q; *q = *(q+3); *(q+3) = temp;
   temp = *(q+1); *(q+1) = *(q+2); *(q+2) = temp;
}

/* SwapShort: swap byte order of short data value *p */
void SwapShort(short *p)
{
   char temp,*q;
   
   q = (char*) p;
   temp = *q; *q = *(q+1); *(q+1) = temp;
}

/* EXPORT->ReadShort: read n short's from src in ascii or binary */
Boolean RawReadShort(Source *src, short *s, int n, Boolean bin, Boolean swap)
{
   int j,k,count=0,x;
   short *p;
   
   if (bin){
      if (fread(s,sizeof(short),n,src->f) != n)
         return FALSE;
      if (swap) 
         for(p=s,j=0;j<n;p++,j++)
            SwapShort(p);  /* Need to swap to machine order */

      count = n*sizeof(short);      
   } else {
      if (src->pbValid) {
         ungetc(src->putback, src->f); src->pbValid = FALSE;
      }
      for (j=1; j<=n; j++){
         if (fscanf(src->f,"%d%n",&x,&k) != 1)
            return FALSE;
         *s++ = x; count += k;
      }
   }
   src->chcount += count;
   return TRUE;
}

/* EXPORT->ReadInt: read n ints from src in ascii or binary */
Boolean RawReadInt(Source *src, int *i, int n, Boolean bin, Boolean swap)
{
   int j,k,count=0;
   int *p;
   
   if (bin){
      if (fread(i,sizeof(int),n,src->f) != n)
         return FALSE;
      if (swap)
         for(p=i,j=0;j<n;p++,j++)
            SwapInt32((int32*)p);  /* Read in SUNSO unless natReadOrder=T */

      count = n*sizeof(int);     
   } else {
      if (src->pbValid) {
         ungetc(src->putback, src->f); src->pbValid = FALSE;
      }
      for (j=1; j<=n; j++){
         if (fscanf(src->f,"%d%n",i,&k) != 1)
            return FALSE;
         i++; count += k;
      }
   }
   src->chcount += count;
   return TRUE;
}

/* EXPORT->ReadFloat: read n floats from src in ascii or binary */
Boolean RawReadFloat(Source *src, float *x, int n, Boolean bin, Boolean swap)
{
   int k,count=0,j;
   float *p;
   
   if (bin){
      if (fread(x,sizeof(float),n,src->f) != n)
         return FALSE;
      if (swap)
         for(p=x,j=0;j<n;p++,j++)
            SwapInt32((int32*)p);  /* Read in SUNSO unless natReadOrder=T */

      count += n*sizeof(float);     
   } else {
      if (src->pbValid) {
         ungetc(src->putback, src->f); src->pbValid = FALSE;
      }
      for (j=1; j<=n; j++){
         if (fscanf(src->f,"%e%n",x,&k) != 1)
            return FALSE;
         x++; count += k;
      }
   }
   src->chcount += count;
   return TRUE;
}

/* EXPORT->ReadShort: read n short's from src in ascii or binary */
Boolean ReadShort(Source *src, short *s, int n, Boolean binary)
{
   return(RawReadShort(src,s,n,binary,(vaxOrder && !natReadOrder)));
}

/* EXPORT->ReadInt: read n ints from src in ascii or binary */
Boolean ReadInt(Source *src, int *i, int n, Boolean binary)
{
   return(RawReadInt(src,i,n,binary,(vaxOrder && !natReadOrder)));
}

/* EXPORT->ReadFloat: read n floats from src in ascii or binary */
Boolean ReadFloat(Source *src, float *x, int n, Boolean binary)
{
   return(RawReadFloat(src,x,n,binary,(vaxOrder && !natReadOrder)));
}

/* EXPORT->KeyPressed: returns TRUE if input is pending on stdin */
Boolean KeyPressed(int tWait)
{
   Boolean rtn=FALSE; 
   char c[1];
     
#ifdef UNIX
   {
      long numchars=0;

      fflush(stdout);
      ioctl(0,FIONREAD,&numchars); 
      if ( numchars > 0){
         read(0, c, 1);
         rtn=TRUE;
      }
   }
#endif
#ifdef WIN32
   INPUT_RECORD inRec;
   DWORD charToRead = 1;
   DWORD charsRead, charsPeeked;
   HANDLE stdinHdle;

   stdinHdle = GetStdHandle( STD_INPUT_HANDLE );
   PeekConsoleInput( stdinHdle, &inRec, charToRead, &charsPeeked );
   if( charsPeeked > 0 ){
      ReadConsoleInput( stdinHdle, &inRec, charToRead, &charsRead );
      FlushConsoleInputBuffer( stdinHdle );
      if(inRec.Event.KeyEvent.bKeyDown &&
         inRec.Event.KeyEvent.uChar.AsciiChar == '\r') {
         rtn = TRUE;
      }
   } 
#endif  
   return rtn;
}
                  
/* -------------------- Error Reporting -------------------- */

/*
  If HAPI is controlling things want it to be able to control
  exit behaviour.
*/
void Exit(int exitcode)
{
   if (exitcode==0 && showConfig)
      PrintConfig();
   exit(exitcode);
}

/*
  Prints toolname and then message to stderr.  Form of message and any
  subsequent arguments is similar to 'printf' with just %d %f %e %s being
  recognised.  If status<>0 then tool is aborted and status returned to OS,
  return status is also printed in error message in this case.
*/

/* EXPORT->HError: print error message on stderr and abort if status<>0 */
void HError(int errcode, char *message, ...)
{
   va_list ap;             /* Pointer to unnamed args */
   FILE *f;

   fflush(stdout);        /* Flush any pending output */
   va_start(ap,message);
   if (errcode<=0) {
      fprintf(stdout," WARNING [%+d]  ",errcode);
      f = stdout;
      vfprintf(f, message, ap);
      va_end(ap);
      fprintf(f," in %s\n", arglist[0]);
   }else{
      fprintf(stderr,"  ERROR [%+d]  ",errcode);
      f = stderr;
      vfprintf(f, message, ap);
      va_end(ap);
      fprintf(f,"\n FATAL ERROR - Terminating program %s\n", arglist[0]);
   }
   fflush(f);
   if (errcode>0) {
      if (abortOnError) abort();
      else Exit(errcode);
   }
}

/* EXPORT->HRError: New function - print error message on stderr and don't abort*/
void HRError(int errcode, char *message, ...)
{
   va_list ap;             /* Pointer to unnamed args */
   FILE *f;

   fflush(stdout);        /* Flush any pending output */
   va_start(ap,message);
   if (errcode<=0) {
      fprintf(stdout," WARNING [%+d]  ",errcode);
      f = stdout;
      vfprintf(f, message, ap);
      va_end(ap);
      fprintf(f," in %s\n", arglist[0]);
   }else{
      fprintf(stderr,"  ERROR [%+d]  ",errcode);
      f = stderr;
      vfprintf(f, message, ap);
      va_end(ap);
      fprintf(f,"\n");
   }
   fflush(f);
}

/* ------------------- Output Routines ----------------------- */

/* EXPORT->WriteShort: write n shorts to f */
void WriteShort (FILE *f, short *s, int n, Boolean binary)
{
   int j,x;
   short *p;
   
   if (binary){
      if (vaxOrder && !natWriteOrder){
         for(p=s,j=0;j<n;p++,j++)
            SwapShort(p);  /* Write in SUNSO unless natWriteOrder=T */
      }
      if (fwrite(s,sizeof(short),n,f) != n)
         HError(5014,"WriteShort: cant write to file");
      if (vaxOrder && !natWriteOrder){
         for(p=s,j=0;j<n;p++,j++)
            SwapShort(p);  /* Swap back */
      }
   } else {
      for (j=1; j<=n; j++){
         x = *s++;
         fprintf(f," %d",x);
      }
   }
}

/* EXPORT->WriteInt: write n ints to f */
void WriteInt(FILE *f, int *i, int n, Boolean binary)
{
   int j;
   int *p;
   
   if (binary){
      if (vaxOrder && !natWriteOrder){
         for(p=i,j=0;j<n;p++,j++)
            SwapInt32((int32*)p);  /* Write in SUNSO unless natWriteOrder=T */
      }
      if (fwrite(i,sizeof(int),n,f) != n)
         HError(5014,"WriteInt: cant write to file");
      if (vaxOrder && !natWriteOrder){
         for(p=i,j=0;j<n;p++,j++)
            SwapInt32((int32*)p);  /* Swap Back */
      }
   } else {
      for (j=1; j<=n; j++){
         fprintf(f," %d",*i++);
      }
   }
}

/* EXPORT->WriteFloat: write n floats to f */
void WriteFloat (FILE *f, float *x, int n, Boolean binary)
{
   int j;
   float *p;
   
   if (binary){
      if (vaxOrder && !natWriteOrder){
         for(p=x,j=0;j<n;p++,j++)
            SwapInt32((int32*)p);  /* Write in SUNSO unless natWriteOrder=T */
      }
      if (fwrite(x,sizeof(float),n,f) != n)
         HError(5014,"WriteFloat: cant write to file");
      if (vaxOrder && !natWriteOrder){
         for(p=x,j=0;j<n;p++,j++)
            SwapInt32((int32*)p);  /* Swap Back */
      }
   } else {
      for (j=1; j<=n; j++){
         fprintf(f," %e",*x++);
      }
   }
}

/* -------------------- File Name Handling ------------------- */

/*
  Given a filename of the general form "path/n.x" in fn, the following
  functions return "n.x", "n", "x" and "path", respectively. In each case,
  the string is returned in s which must be large enough and s is returned
  as the function result.
*/

/* EXPORT->CheckFn: Check (and modify) fn matches machine path conventions */
static char *CheckFn(char *fn)
{
   if (fn==NULL) return(fn);
#ifdef ALTPATHCHAR
   {
      char *s;
      for(s=fn; *s!=0; s++) 
         if (*s==ALTPATHCHAR) *s=PATHCHAR;
   }
#endif
   return(fn);
}

/* EXPORT->NameOf: name of fn */
char * NameOf(char *fn, char *s)
{
   char *t;

   CheckFn(fn);
   t = strrchr(fn,PATHCHAR);
   if (t == NULL) 
      t = fn;
   else 
      t++;
   return strcpy(s,t);
}

/* EXPORT->BaseOf: base name part of fn */
char * BaseOf(char *fn, char *s)
{
   char *t;
   
   NameOf(fn,s);
   t = strrchr(s,'.');
   if (t == NULL) return s;
   if (t>=s) *t = '\0';
   return s;
}

/* EXPORT->PathOf: path part of fn */
char * PathOf(char *fn, char *s)
{
   char *t;
   
   CheckFn(fn);
   strcpy(s,fn);
   t = strrchr(s,PATHCHAR);
   if (t == NULL) 
      *s='\0';
   else
      *++t = '\0';
   return s;
}

/* EXPORT->ExtnOf: extension part of fn */
char * ExtnOf(char *fn, char *s)
{
   char *t,buf[100];
   
   NameOf(fn,buf);
   t = strrchr(buf,'.');
   if (t == NULL){
      *s ='\0'; return s;
   }
   return strcpy(s,t+1);
}

/* EXPORT->MakeFN: construct a filename from fn */
char * MakeFN(char *fn, char *path, char *ext, char *s)
{
   char newPath[MAXFNAMELEN], base[MAXSTRLEN], newExt[MAXSTRLEN];  /* components of new fn */
   int i;
   
   CheckFn(path);
   BaseOf(fn,base);
   if (path==NULL) 
      PathOf(fn,newPath);
   else
      strcpy(newPath,path);
   if (ext==NULL)
      ExtnOf(fn,newExt);
   else 
      strcpy(newExt,ext);
   i = strlen(newPath);
   if (i>0 && newPath[i-1] != PATHCHAR) {
      newPath[i] = PATHCHAR;
      newPath[i+1] = '\0';
   }
   strcpy(s,newPath);
   strcat(s,base);
   if (strlen(newExt) > 0) {
      strcat(s,".");
      strcat(s,newExt);
   }
   return s;
}

/* EXPORT->CounterFN: generate a file name with embedded count */
char * CounterFN(char *prefix, char* suffix, int count, int width, char *s)
{
   char num[20];

   sprintf(num,"%0*d",width,count);
   s[0] = '\0';
   if (prefix!=NULL) strcat(s,prefix);
   strcat(s,num);
   if (suffix!=NULL) strcat(s,suffix);
   return s;
}


/* ------------------- Pattern Matching ------------------- */

/* RMatch: recursively match s against pattern p, minplen
   is the min length string that can match p and
   numstars is the number of *'s in p */
Boolean RMatch(char *s,char *p,int slen,int minplen,int numstars)
{
   if (slen==0 && minplen==0)
      return TRUE;
   if (numstars==0 && minplen!=slen)
      return FALSE;
   if (minplen>slen)
      return FALSE;
   if (*p == '*')
      return RMatch(s+1,p+1,slen-1,minplen,numstars-1) ||
         RMatch(s,p+1,slen,minplen,numstars-1) ||
         RMatch(s+1,p,slen-1,minplen,numstars);
   if (*p == *s || *p == '?')
      return RMatch(s+1,p+1,slen-1,minplen-1,numstars);
   else
      return FALSE;  
}

/* EXPORT->DoMatch: return TRUE if s matches pattern p */
Boolean DoMatch(char *s, char *p)
{
   int slen, minplen, numstars;
   char *q,c;
   
   slen = strlen(s);
   minplen = 0; numstars = 0; q = p;
   while ((c=*q++))
      if (c == '*') ++numstars; else ++minplen;
   return RMatch(s,p,slen,minplen,numstars);
}


/* SpRMatch: recursively match s against pattern p, minplen
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

/* EXPORT->MaskMatch: return spkr if s matches pattern p */
Boolean MaskMatch(char *mask, char *spkr, char *str)
{
   int spkrlen, slen, minplen, numstars;
   char *q,c;
  
   if (mask == NULL || str==NULL) return(FALSE);
   slen = strlen(str);
   spkrlen = minplen = numstars = 0;
   q = mask;
   while ((c=*q++)) {
      if (c == '*') ++numstars; else ++minplen;
      if (c == '%') ++spkrlen;
   }
   if (spkrlen>=MAXSTRLEN)
      HError(3390,"MaskMatch: Speaker name too long %d vs %d",spkrlen,MAXSTRLEN);
   spkr[0]=0;
   if (SpRMatch(str,mask,spkr,slen,minplen,numstars))
      return(TRUE);
   else {
      spkr[0]=0;
      return(FALSE);
   }
}

static char *savedCommandLine;
/* SaveCommandLine: Stores all command line arguments in 1 string */
static void SaveCommandLine(int argc, char **argv)
{
   int i, len=0;

   for (i=0;i<argc;i++)
      len+=strlen(argv[i])+1;
   savedCommandLine = (char *) malloc(len);
   savedCommandLine[0]='\000';
   strcat(savedCommandLine, argv[0]);
   for (i=1;i<argc;i++)
      sprintf(savedCommandLine,"%s %s", savedCommandLine, argv[i]);
}
/* EXPORT->RetrieveCommandLine: return a pointer to the saved command
line string */
char *RetrieveCommandLine(void)
{
   return savedCommandLine;
}

/* ------------------- Initialisation ------------------- */

/* EXPORT->InitShell:   Called by main to initialise the module.
   Main passes over the command line arguments which are then
   copied after processing -A/-B/-C/-S/-V.  */
ReturnStatus InitShell(int argc, char *argv[], char *ver, char *sccs)
{
   char *fn;
   int i,j;
   Boolean b;

   argcount = 1; arglist = (char **) malloc(argc*sizeof(char *));
   arglist[0] = argv[0];
   Register(ver,sccs);
   Register(hshell_version,hshell_vc_id);
   /* read default configuration parameter file, if any */
   if ((fn=getenv("HCONFIG")) != NULL){
      if(ReadConfigFile(fn)<SUCCESS){
         HRError(5020,"InitShell: ReadConfigFile failed on file %s", fn);
         return(FAIL);
      }
   }
   SaveCommandLine(argc, argv);
   
#ifdef WIN32
   _fmode = _O_BINARY;
#endif   
   vaxOrder = IsVAXOrder();

   /* copy arg list and process -C and/or -S options */
   for (i=1; i<argc; i++){
      if (strcmp(argv[i],"-A") == 0){
         for (j=0; j<argc; j++)
            printf("%s ",argv[j]);
         printf("\n"); fflush(stdout);
         infoPrinted = TRUE;
      } else
         if (strcmp(argv[i],"-C") == 0){
            if (++i >= argc){ 
               HRError(5020,"InitShell: Config file name expected");
               return(FAIL);
            }
            if(ReadConfigFile(argv[i])<SUCCESS){
               HRError(5020,"InitShell: ReadConfigFile failed on file %s", argv[i]);
               return(FAIL);
            }
         } else
            if (strcmp(argv[i],"-S") == 0){
               if (++i >= argc){ 
                  HRError(5020,"InitShell: Script file name expected");
                  return(FAIL);
               }
               if(SetScriptFile(argv[i])<SUCCESS){
                  HRError(5020,"InitShell: SetScriptFile failed on file %s", argv[i]);
                  return(FAIL);
               }
            } else
               if (strcmp(argv[i],"-V") == 0){
                  printVersionInfo = TRUE;
                  infoPrinted = TRUE;
               } else
                  if (strcmp(argv[i],"-D") == 0){
                     showConfig = TRUE;
                     infoPrinted = TRUE;
                  } else
                     arglist[argcount++] = argv[i];
   }
   if (showConfig)
      PrintConfig();
   /* process this module's config params */
   nParm = GetConfig("HSHELL", TRUE, cParm, MAXGLOBS);
   if (nParm>0) {
      if (GetConfInt(cParm,nParm,"TRACE",&i)) trace = i;
      if (GetConfBool(cParm,nParm,"NONUMESCAPES",&b)) noNumEscapes = b;
      if (GetConfBool(cParm,nParm,"ABORTONERR",&b)) abortOnError = b;
      if (GetConfBool(cParm,nParm,"NATURALREADORDER",&b)) natReadOrder = b;
      if (GetConfBool(cParm,nParm,"NATURALWRITEORDER",&b)) natWriteOrder = b;
      if (GetConfBool(cParm,nParm,"EXTENDFILENAMES",&b)) extendedFileNames = b;
      if (GetConfInt(cParm,nParm,"MAXTRYOPEN",&i)) {
         maxTry = i;
         if (maxTry<1 || maxTry>3){
            HRError(5073,"InitShell: MAXTRYOPEN out of range (%d)",maxTry);
            maxTry=1;
         }
      }
   }

   
   return(SUCCESS);
}


/* EXPORT->PrintStdOpts: print standard options */
void PrintStdOpts(char *opt)
{
   printf(" -A      Print command line arguments         off\n");
   if (strchr(opt,'B'))
      printf(" -B      Save HMMs/transforms as binary       off\n");
   printf(" -C cf   Set config file to cf                default\n");
   printf(" -D      Display configuration variables      off\n");
   if (strchr(opt,'E')) {
      printf(" -E s [s] set dir for parent xform to s       off\n");
      printf("         and optional extension                 \n");
   }
   if (strchr(opt,'F'))
      printf(" -F fmt  Set source data format to fmt        as config\n");
   if (strchr(opt,'G'))
      printf(" -G fmt  Set source label format to fmt       as config\n");
   if (strchr(opt,'H'))
      printf(" -H mmf  Load HMM macro file mmf\n");
   if (strchr(opt,'I'))
      printf(" -I mlf  Load master label file mlf\n");
   if (strchr(opt,'J')) {
      printf(" -J s [s] set dir for input xform to s        none\n");
      printf("         and optional extension                 \n");
   }
   if (strchr(opt,'K')) {
      printf(" -K s [s] set dir for output xform to s       none\n");
      printf("         and optional extension                 \n");
   }
   if (strchr(opt,'L'))
      printf(" -L dir  Set input label (or net) dir         current\n");
   if (strchr(opt,'M'))
      printf(" -M dir  Dir to write HMM macro files         current\n");
   if (strchr(opt,'O'))
      printf(" -O      Set target data format to fmt        as config\n");
   if (strchr(opt,'P'))
      printf(" -P      Set target label format to fmt       as config\n");
   if (strchr(opt,'Q'))
      printf(" -Q      Print command summary\n");
   printf(" -S f    Set script file to f                 none\n");
   printf(" -T N    Set trace flags to N                 0\n");
   printf(" -V      Print version information            off\n");
   if (strchr(opt,'X'))
      printf(" -X ext  Set input label (or net) file ext    lab\n");
}

/* -------------------------- End of HShell.c ----------------------------- */
