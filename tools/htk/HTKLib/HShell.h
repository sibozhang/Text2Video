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
/*         File: HShell.h:   Interface to the Shell            */
/* ----------------------------------------------------------- */

/* !HVER!HShell:   3.4.1 [CUED 12/03/09] */

#ifndef _HSHELL_H_
#define _HSHELL_H_

#include <stdio.h>         /* Standard Libraries */
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <ctype.h>
#include <math.h>
#include <limits.h>
#ifdef WIN32 /* WIN32 modification */
#include <time.h>
#include <Winsock2.h>
#else
#include <sys/time.h>
#endif
#include <errno.h>
#include <signal.h>
#include <assert.h>
#ifdef CYGWIN
#include <asm/socket.h>
#endif


#ifdef __cplusplus
extern "C" {
#endif

#if !defined UNIX && !defined WIN32 && !defined MPW &&!defined VMS
#define UNIX     /* Choices are UNIX, WIN32, MPW, VMS */
#endif

#ifdef UNIX
#include <unistd.h>
#include <time.h>
#endif


#define MAXSTRLEN 256    /* max length of a string */
#define MAXFNAMELEN 1034 /* max length of a file name */
#define SMAX      5      /* max num data streams + 1 */
#define MAXGLOBS  256    /* max num global config parms */

#define SING_QUOTE '\''  /* character used as quote */
#define DBL_QUOTE '"'    /* character used as quote */
#define ESCAPE_CHAR '\\' /* character used as escape */

#ifdef UNIX
#include <sys/types.h>
#include <sys/ioctl.h>
#endif

#undef FALSE
#undef TRUE

typedef int int32;

typedef enum {FAIL=-1, SUCCESS=0} ReturnStatus;

/* Boolean type definition */
typedef enum {FALSE=0, TRUE=1} Boolean;

typedef double HTime;      /* time in 100ns units */

typedef enum{       /* Input filters for various types of file */
   WaveFilter,      /* waveforms input via HWave */
   ParmFilter,      /* parameter files input via HParm */
   LangModFilter,   /* language model files input via HLM */
   HMMListFilter,   /* HMM lists input via HModel */
   HMMDefFilter,    /* HMM definition files input via HModel */
   LabelsFilter,    /* Label files input via HLabel */
   NetFilter,       /* Network file input via HNet */
   DictFilter,      /* Dictionary file input via HDict */
   LGramFilter,     /* NGram Input via LGBase */
   LWMapFilter,     /* LM Word Map Input via LWMap */
   LCMapFilter,     /* LM Class Map Input */
   LMTextFilter,    /* LM source text input via LGPrep */
   NoFilter,        /* Direct input - no pipe */

   WaveOFilter,     /* waveforms output via HWave */
   ParmOFilter,     /* parameter files output via HParm */
   LangModOFilter,  /* language model files output via HLM */
   HMMListOFilter,  /* HMM lists output via HModel */
   HMMDefOFilter,   /* HMM definition files output via HModel */
   LabelsOFilter,   /* Label files output via HLabel */
   NetOFilter,      /* Network file output via HNet */
   DictOFilter,     /* Dictionary file output via HDict */
   LGramOFilter,    /* NGram Output via LGBase */
   LWMapOFilter,    /* LM Word Map Output via LWMap */
   LCMapOFilter,    /* LM Class Map Output */
   NoOFilter        /* Direct output - no pipe */
}IOFilter;


typedef struct {     /* Defines a source file with position tracking */
   char name[256];      /* file name for error messages */
   FILE *f;             /* input stream */
   Boolean isPipe;      /* input is a pipe */
   Boolean pbValid;     /* true if putback holds char */
   Boolean wasQuoted;   /* true if ReadString returned quoted string */
   Boolean wasNewline;  /* true if SkipWhiteSpace went over newline */
   int putback;         /* put back character */
   int chcount;         /* num chars from start */
} Source;

typedef enum{        /* Type of configuration parameter */
   StrCKind,            /* string, optionally in dble quotes */
   IntCKind,            /* integer value - coercable to float */
   FltCKind,            /* float value */
   BoolCKind,           /* Boolean: T,F,True,False */
   AnyCKind             /* dont care */
} ConfKind;

typedef union {      /* union of possible config param kinds */
   char *s;
   int i;
   double f;
   Boolean b;
}ConfVal;

typedef struct {     /* Configuration Parameter */
   char *user;          /* name of module/tool to use this param */
   char *name;          /* name of param - upper case always */
   ConfKind kind;       /* kind of config param value */
   ConfVal val;         /* value */
   Boolean seen;        /* set true when read by any module */
} ConfParam;


/* ---------------- Termination and error handling ------------------- */

void Exit(int exitcode);
/*
   Exit from tool (and print termination diagnostics if required).
*/

void HError(int errcode, char *message, ...);
/*
   Print the given message in 'printf' style, then if err > 0 terminate
   returning errcode as status.
*/

void HRError(int errcode, char *message, ...);
/* 
New function - print error message on stderr and don't abort.
*/


/* ------------------------ Initialisation --------------------------- */

ReturnStatus SetScriptFile(char *fn); 
/* 
    Allows more than one script file to be used by an HTK Tool
    For example, with the -S and -f options in HDistance
*/


extern Boolean vaxOrder;  
/* 
   Global variable indicating VAX-order architecture for storing numbers 
*/

ReturnStatus InitShell(int argc, char *argv[], char *ver, char *sccs);
/* 
   Called from main to initialise module and store command line parameters 
   for subsequent access by following xxxArg routines.  The following
   standard command line options are processed immediately by InitShell
   and then deleted from the command line:
      -A                   print command line arguments
      -C configfile        load given configuration file
      -D                   display config parameters
      -S scriptfile        append the given script file
      -V                   print version and sccs info and abort
   ver and sccs are the HTK version info ver and sccs info for the
   current tool.
*/

void Register(char *ver, char *sccs);
/*
   Register module version and sccs info.
   This information is used by PrintVersion.  It is called by
   each module's Init routine.
*/

char * RegisterExtFileName(char *s);
/* Record details of fn extended attributed if any in circular buffer */


Boolean InfoPrinted(void);
/*
   Returns true if Shell has printed out information via one
   of the special commands -A, -B or -V.  This MUST be called
   immediately after Initialising all modules since it executes
   a pending -V request, if any.
*/

void PrintStdOpts(char *opts);
/*
   Print standard command line options - as above.  Additional
   standard options (ie capital letter options not implemented
   directly by the shell) must be listed in opts from the 
   following set:
       -B      Save HMM macro files as binary
       -F fmt  Set source format to fmt 
       -G fmt  Set source label format to fmt
       -H mmf  Load HMM macro file mmf
       -I mlf  Load master label file mlf
       -L dir  Dir holding label files
       -M      Dir to write HMM macro files
       -O      Set target data format to fmt
       -P      Set target label format to fmt
       -Q      Print command summary
       -X ext  Set label file extension to ext
   Only exception to this is -T which is always printed.
*/

/* ------------- Configuration Parameter File Handling --------------- */

void PrintConfig(void);
/*
   Print the current configuration and usage
*/

int GetConfig(char *user, Boolean incGlob, ConfParam **list, int max);
/*
    Store a list of upto max configuration parameters whose user name
    matches the user and return the number of parameters stored.  If
    list is NULL then just return the number of parameters that would
    have been stored.  If incGlob then global parameters are
    included.  If user is NULL then all parameters are returned.
    If max is exceeded then an error occurs.
*/

Boolean HasConfParm(ConfParam **list, int size, char *name);
Boolean GetConfStr(ConfParam **list,int size,char *name, char *str);
Boolean GetConfBool(ConfParam **list,int size,char *name, Boolean *b);
Boolean GetConfInt(ConfParam **list,int size,char *name, int *ival);
Boolean GetConfFlt(ConfParam **list,int size,char *name, double *fval);
/*
   Access routines for the array of size ConfParam elements returned
   by GetConfig.  Returns true if name is found.  Generates an error
   if kind is different
*/

/* ----------------- Command Line Argument Handling ------------------ */

typedef enum {SWITCHARG, STRINGARG, INTARG, FLOATARG, NOARG} ArgKind;

int  NumArgs(void);
/*
   Returns number of command line args which have not yet been processed.
   Initially this is equal to argc-1 since actual command name is ignored
*/

ArgKind NextArg(void);
/*
   Returns the kind of the next command line argument.  If NumArgs()==0 then
   NOARG will be returned. 
*/

char * GetStrArg(void);
char * GetSwtArg(void);
int    GetIntArg(void);
long   GetLongArg(void);
float  GetFltArg(void);
/*
   Return value of next arg and 'consume' that arg.  An error is raised if
   the kind of the next arg is different to that requested except that 
   GetFltArg will accept either an integer or a float argument
*/

int   GetChkedInt(int min, int max, char * swtname);
long   GetChkedLong(long min, long max, char * swtname);
float GetChkedFlt(float min, float max, char * swtname);
/* 
   Range checked versions of GetIntArg and GetFltArg.  Swtname
   is the name of the preceding switch and is only used in
   error message.
*/

Boolean GetIntEnvVar(char *envVar, int *value);
/*
   Get the integer value of env variable envVar.
   Returns false if envVar not set
*/

Boolean GetFileNameExt(char *logfn, char *actfn, long *st, long *en);
/* 
   Return true if given file has extensions, i.e. an alias and/or st/end
   indices.  If true actual file name is copied into actfn and indices
   are copied into st and en.   Extended file names have the form
             file[1,999]
   or        logfile=actfile
   or        logfile=actfile[1,999]
     
   File extensions can be disabled by seting EXTENDFILENAMES to F
*/       

/* ---------------------- Input Handling ----------------------------- */

FILE *FOpen(char *fname, IOFilter filter, Boolean *isPipe);
/*
   Open the file fname for reading or writing (depending on
   whether IOFilter is a Filter or OFilter) and return a file pointer.  
   If the environment variable HxxxxFILTER is set to a 
   command of the form "foo $ a b ..." then the given fname
   replaces the $ and popen is used to connect to the output
   of foo. The Boolean isPipe returns true if input is a pipe.
   In addition, if the environment variable HMAXTRY is set to
   an integer n, then an fopen call which fails will be retried
   n-1 more times before failing completely.  This is useful 
   in combatting occassional NFS errors.
*/

void FClose(FILE *f, Boolean isPipe);
/*
   Close the given file or pipe
*/

ReturnStatus InitSource(char *fname, Source *src, IOFilter filter);
/*
   Initialise a text source using file fname and filter - returns
*/

void CloseSource(Source *src);
/*
   Close source
*/

void AttachSource(FILE *file, Source *src);
/*
   Attach a source to already open file
*/

char *SrcPosition(Source src, char *s);
/* 
   return string describing the current position in src
*/

int  GetCh(Source *src);
void UnGetCh(int c, Source *src);
/*
   Get/Unget a character from the given source 
*/

Boolean ReadString(Source *src, char *s);
Boolean ReadStringWithLen(Source *src, char *s, int buflen); /* With specified length of buffer*/
char *ParseString(char *src, char *s);
/*
   Read a string from the given source where a string is any
   sequence of non-white space characters, or any sequence of
   characters enclosed by single or double quotes.
   Also supports use of escape character to allow quotes to appear
   within the string and also to allow three digit octal specification
   of any character.  
   Return TRUE (for ReadString) or pointer to next unread character
   (for ParseString) if no error.
   '"QUOTE' "\"QUOTE" \"QUOTE \042QUOTE all return "QUOTE in s
*/

Boolean ReadRawString(Source *src, char *s);
/* 
   Read a raw string (i.e. word upto next white-space) from src and store it in s 
*/

void WriteString(FILE *f,char *s,char q);
char *ReWriteString(char *s,char *dst, char q);
/*
   Write string s in ReadString format (with quotes and escapes as
   needed).  Either writes direct to file (WriteString), to supplied
   buffer (dst!=NULL) or to static buffer (dst==NULL).
   q can either be SING_QUOTE, DBL_QUOTE to force quoting of string
   or 0 to quote strings that contain quotes, or ESCAPE_CHAR to never
   quote the string. 
*/

Boolean SkipLine(Source *src);
/* 
   skip to next line in source, return FALSE when EOF reached
*/

Boolean ReadLine(Source *src,char *s);
/* 
   read to next newline in source, return FALSE when EOF reached
*/

void ReadUntilLine (Source *src, char *s);
/* 
   read to next occurrence of string 
*/

void SkipWhiteSpace(Source *src);
/* 
   skip white space (if any)
   if any space is skipped sets wasNewline to indicate if the space
   contained a newline character
*/

void SkipComment(Source *src);
/* 
   skip to next non-blank, if it is a # then skip to next line
*/

Boolean ReadShort(Source *src, short *s, int n, Boolean binary);
Boolean ReadInt  (Source *src, int *i,   int n, Boolean binary);
Boolean ReadFloat(Source *src, float *x, int n, Boolean binary);
/*
   Read n short/int/float(s) from the given source, return 
   TRUE if no error.  If binary then binary read is performed - 
   byte swapping is controlled by HShell config variables.
*/
Boolean RawReadShort(Source *src, short *s, int n, Boolean bin, Boolean swap);
Boolean RawReadInt(Source *src, int *i, int n, Boolean bin, Boolean swap);
Boolean RawReadFloat(Source *src, float *x, int n, Boolean bin, Boolean swap);
/*
   Read n short/int/float(s) from the given source, return 
   TRUE if no error.  
   If binary then binary read is performed.
   If swap then values are byte swapped after reading.
*/

void SwapShort(short *p);
void SwapInt32(int32 *p);
/* 
   Byte swap various types
*/

Boolean KeyPressed(int tWait);
/*
   Returns TRUE if input is pending on stdin within tWait seconds
*/   

/* -------------------------- Output Handling ------------------------ */

void WriteShort(FILE *f, short *s, int n, Boolean binary);
void WriteInt  (FILE *f, int *i,   int n, Boolean binary);
void WriteFloat(FILE *f, float *x, int n, Boolean binary);
/*
   Write n short/int/float(s) to the given file.  
   If binary then binary Write is performed.
*/

/* ---------------------- File Name Manipulation --------------------- */

#if defined MPW
#define PATHCHAR ':'
#endif
#if defined UNIX
#define PATHCHAR '/'
#endif
#if defined WIN32
#define PATHCHAR '/'
#define ALTPATHCHAR '\\'
#endif
#if defined VMS
#define PATHCHAR ']'
#endif

char * NameOf(char *fn, char *s);
char * BaseOf(char *fn, char *s);
char * PathOf(char *fn, char *s);
char * ExtnOf(char *fn, char *s);
/*
   Given a filename of the general form "path/n.x" in fn, the above
   functions return "n.x", "n", "x" and "path/", respectively. In each case,
   the string is returned in s which must be large enough and s is returned
   as the function result.
*/

char * MakeFN(char *fn, char *path, char *ext, char *s);
/*
   The supplied file name fn is used as a template for constructing a 
   new file name returned in s as function result.  The construction
   rules are as follows:
      a) if path is not NULL then any path is stripped from fn and
         replaced by the given path
      b) if ext is not NULL then any extension is stripped from fn 
         and replaced by the given extension
*/

char * CounterFN(char *prefix, char* suffix, int count, int width, char *s);
/*
   creates a file name in s of form "PREFNNNSUF" where NNNN is count 
   expressed in width digits with leading zeroes if needed.
*/


void SubstFName(char *fname, char *s);
/* 
   Subst fname for any occurrences of $ in s which must be large
   enough to accommodate the expanded string
*/

/* ------------------------ Pattern Matching ------------------------- */

Boolean DoMatch(char *s, char *p);
/* 
   Returns true if the string s matches the pattern p.
   The pattern p may contain the metacharacters '?'
   which will match exactly 1 character and '*'
   which will match zero or more characters.
*/

Boolean MaskMatch(char *mask, char *spkr, char *str);
/* Returns true if the string str matches the pattern.
   The string matched to the '%' is returned in spkr.
*/

char *RetrieveCommandLine(void);
/*
   Retrieves the savedCommandLine, that contains the
   actual command line used to run the program at hand.
*/

#ifdef __cplusplus
}
#endif

#endif  /* _HSHELL_H_ */

/* ----------------------- End of HShell.h --------------------------- */
