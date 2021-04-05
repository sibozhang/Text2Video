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
/*     File: HBuild.c:  Word-Lattice Building                  */
/* ----------------------------------------------------------- */

char *hbuild_version = "!HVER!HBuild:   3.4.1 [CUED 12/03/09]";
char *hbuild_vc_id = "$Id: HBuild.c,v 1.1.1.1 2006/10/11 09:54:59 jal58 Exp $";

/* The HBuild program takes input files in a number of different
   formats and constructs suitable HTK word lattice files.

   The formats currently supported by HBuild include:

   a) Bigrams in either ARPA/Lincol-Labs format or HTK matrix format
   b) HTK Multi-Level lattices
   c) Word Lists for simple loops
   d) ARPA word-pair grammars (Resource Management style)
*/

/* Trace Flags */
#define T_TOP        0001    /* Top Level tracing */

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
#include "HLM.h"

typedef enum {unknown, wordLoop, boBiGram, matBiGram, multiLat, wordPair} BuildType;

static int trace     = 0;           /* Trace flags */

static LabId enterId;               /* id of !ENTRY label in ngram */
static LabId exitId;                /* id of !EXIT label in ngram */
static LabId bStartId=NULL;         /* id of start bracket */
static LabId bEndId=NULL;           /* id of end bracket */

static LabId unknownId;             /* id of unknown label in ngram */
static Boolean zapUnknown = FALSE;  /* zap unknown symbols from bigram */

MemHeap buildStack;

/* ---------------- Configuration Parameters --------------------- */

static ConfParam *cParm[MAXGLOBS];
static int nParm = 0;            /* total num params */

/* ---------------- Process Command Line ------------------------- */

/* SetConfParms: set conf parms relevant to this tool */
void SetConfParms(void)
{
   int i;

   nParm = GetConfig("HBUILD", TRUE, cParm, MAXGLOBS);
   if (nParm>0){
      if (GetConfInt(cParm,nParm,"TRACE",&i)) trace = i;
   }
}

void ReportUsage(void)
{
   printf("\nUSAGE: HBuild [options] wordList latFile\n\n");
   printf(" Option                                       Default\n\n");
   printf(" -b      binary lattice output                ASCII\n");
   printf(" -m s    load matrix bigram from s            off\n");
   printf(" -n s    load back-off bigram from s          off\n");
   printf(" -s s1 s2 s1/s2 are bigram start/end labels   !ENTER !EXIT\n");
   printf(" -t s1 s2 bracket word-loop/pair with s1 s2   off\n");
   printf(" -u s    set unknown symbol to s              !NULL\n");
   printf(" -w s    load word-pair grammar from s        off\n");
   printf(" -x s    load multi-level lattice from s      off\n");
   printf(" -z      ignore ngrams with unknown symbol    off\n");
   PrintStdOpts(""); 
   printf("\n\n");
}

int main(int argc, char *argv[])
{
   char *wordListFn,*latFn,*ipFn=NULL;
   LModel *bigramLm;
   BuildType bType = unknown;
   Boolean saveLatBin = FALSE;
   LatFormat format = HLAT_LMLIKE;
   Lattice *lat,*ipLat;
   Vocab voc;
   char  *s;

   Lattice *ProcessWordLoop(MemHeap *latHeap, Vocab *voc);
   Lattice *ProcessBiGram(MemHeap *latHeap, Vocab *voc, LModel *biLM);
   void SaveLattice(Lattice *lat, char *latFn, LatFormat format);
   Lattice *LoadLattice(MemHeap *latHeap, char *latFn, Vocab *voc,
                        Boolean shortArc);
   Lattice *ProcessWordPair(MemHeap *latHeap, Vocab *voc, char *fn);

   if(InitShell(argc,argv,hbuild_version,hbuild_vc_id)<SUCCESS)
      HError(3000,"HBuild: InitShell failed");
   InitMem();   InitLabel();
   InitMath();  
   InitDict();  InitNet();  
   InitLM();

   CreateHeap(&buildStack, "HBuild Stack",  MSTAK, 1, 0.0, 100000, LONG_MAX );

   if (!InfoPrinted() && NumArgs() == 0)
      ReportUsage();
   if (NumArgs() == 0) Exit(0);
   SetConfParms();

   enterId=GetLabId("!ENTER",TRUE);   /* All sentences should or are coerced */
   exitId=GetLabId("!EXIT",TRUE);     /*  to start enterId and end exitId */
   unknownId=GetLabId("!NULL",TRUE);  /* Name for words not in list */

   while (NextArg() == SWITCHARG) {
      s = GetSwtArg();
      if (strlen(s)!=1) 
         HError(3019,"HBuild: Bad switch %s; must be single letter",s);
      switch(s[0]){
      case 'b':
         saveLatBin = TRUE; break;    
      case 'm':
         if (bType != unknown)
            HError(3019,"HBuild: Can only specifiy one of -m, -n, -w, -x");
         bType = matBiGram;
         if (NextArg()!=STRINGARG)
            HError(3019,"HBuild: Matrix Bigram file name expected");
         ipFn = GetStrArg(); 
         break;
      case 'n':
         if (bType != unknown)
            HError(3019,"HBuild: Can only specifiy one of -m, -n, -w, -x");
         bType = boBiGram;
         if (NextArg()!=STRINGARG)
            HError(3019,"HBuild: Back-off Bigram file name expected");
         ipFn = GetStrArg(); 
         break;
      case 's':
         if (NextArg() != STRINGARG)
            HError(3019,"HBuild: Bigram ENTER label name expected");
         enterId=GetLabId(GetStrArg(),TRUE);
         if (NextArg() != STRINGARG)
            HError(3019,"HBuild: Bigram EXIT label name expected");
         exitId=GetLabId(GetStrArg(),TRUE);
         break;
      case 't':
         if (NextArg() != STRINGARG)
            HError(3019,"HBuild: Bracket start label name expected");
         bStartId=GetLabId(GetStrArg(),TRUE);
         if (NextArg() != STRINGARG)
            HError(3019,"HBuild: Bracket end label name expected");
         bEndId=GetLabId(GetStrArg(),TRUE);
         break;
      case 'u':
         if (NextArg() != STRINGARG)
            HError(3019,"HBuild: Unknown label name expected");
         unknownId=GetLabId(GetStrArg(),TRUE);
         break;
      case 'w':
         if (bType != unknown)
            HError(3019,"HBuild: Can only specifiy one of -m, -n, -w, -x");
         bType = wordPair;
         if (NextArg()!=STRINGARG)
            HError(3019,"HBuild: Word pair grammar file name expected");
         ipFn = GetStrArg(); 
         break;
      case 'x':
         if (bType != unknown)
            HError(3019,"HBuild: Can only specifiy one of -m, -n, -w, -x");
         bType = multiLat;
         if (NextArg()!=STRINGARG)
            HError(3019,"HBuild: Multi-level lattice file name expected");
         ipFn = GetStrArg(); 
         break;
      case 'z':
         zapUnknown = TRUE; break;    
      case 'T':
         trace = GetChkedInt(0,511,s); break;
      default:
         HError(3019,"HBuild: Unknown switch %s",s);
      }
   } 
   if (NextArg()!=STRINGARG)
      HError(3019,"HBuild: Word List file name expected");
   wordListFn = GetStrArg();
   if (NextArg()!=STRINGARG)
      HError(3019,"HBuild: output lattice file name expected");
   latFn = GetStrArg();
   if (bType == unknown) bType = wordLoop;
   if (saveLatBin) format |= HLAT_LBIN;   
   /* Read the word-list into a Vocab data structure */
   InitVocab(&voc);
   if(ReadDict(wordListFn, &voc)<SUCCESS)
      HError(3013,"HBuild: ReadDict failed");
   switch (bType) {
   case matBiGram:
      if (trace & T_TOP)
         printf("Reading bigram from file %s\n",ipFn);
      bigramLm = ReadLModel(&gstack, ipFn);
      if (bigramLm->type != matBigram)
         HError(3030,"HBuild: File specified is not a matrix bigram");
      lat = ProcessBiGram(&gstack,&voc,bigramLm);
      SaveLattice(lat,latFn,format);
      break;
   case boBiGram:
      if (trace & T_TOP)
         printf("Reading bigram from file %s\n",ipFn);
      bigramLm = ReadLModel(&gstack, ipFn);
      if (bigramLm->type != boNGram)
         HError(3030,"HBuild: File specified is not a back-off bigram");
      lat = ProcessBiGram(&gstack,&voc,bigramLm);
      SaveLattice(lat,latFn,format);
      break;
   case multiLat:
      if (trace & T_TOP)
         printf("Reading input lattice from file %s\n",ipFn);
      ipLat = LoadLattice(&buildStack,ipFn,&voc,FALSE);
      if (ipLat->subList!=NULL) {
         if (trace & T_TOP)
            printf("Expanding multi-level lattice\n");
         lat = ExpandMultiLevelLattice(&buildStack,ipLat,&voc);
      }
      else
         lat = ipLat;
      SaveLattice(lat,latFn,format);
      break;
   case wordLoop:
      if (trace & T_TOP)
         printf("Building word loop\n");
      lat = ProcessWordLoop(&gstack,&voc);
      SaveLattice(lat,latFn,format);
      break;
   case wordPair:
      lat = ProcessWordPair(&gstack,&voc,ipFn);
      SaveLattice(lat,latFn,format);
      break;
   default:
      HError(3001,"Only Bigram LMs / multiLats currently implemented");
   }
   Exit(0);
   return (0);          /* never reached -- make compiler happy */
}

/* Save a lattice to a file latFn  */
void SaveLattice(Lattice *lat, char *latFn, LatFormat format)
{
   FILE *latf;
   Boolean isPipe;

   if (trace & T_TOP)
      printf("Saving lattice to file %s\n",latFn);
   if ( (latf = FOpen(latFn,NetOFilter,&isPipe)) == NULL)
      HError(3011,"SaveLattice : Cannot create new lattice file  %s",latFn);
   if(WriteLattice(lat,latf,format)<SUCCESS)
      HError(3011,"SaveLattice : Cannot create new lattice file  %s",latFn);
   FClose(latf,isPipe);
}

/* Load a lattice from file latFn  */
Lattice *LoadLattice(MemHeap *latHeap, char *latFn, Vocab *voc,
                     Boolean shortArc)
{
   FILE *latf;
   Boolean isPipe;
   Lattice *lat;

   if ( (latf = FOpen(latFn,NetFilter,&isPipe)) == NULL)
      HError(3010,"LoadLattice : Cannot open lattice file %s",latFn);
   if((lat = ReadLattice(latf,latHeap,voc,shortArc,FALSE))==NULL)
      HError(3010,"LoadLattice : ReadLattice failed");
   FClose(latf,isPipe);
   return lat;
}

Lattice *ProcessWordLoop(MemHeap *latHeap, Vocab *voc)
{
   int nNode,nArc;
   LNode *ln;
   LArc *la;
   Word wd;
   Lattice *lat;
   int i;

   nNode = voc->nwords+4; 
   nArc =  voc->nwords*2 + 3;
   
   lat = NewLattice(latHeap,nNode,nArc);
   lat->voc = voc;
   lat->lmscale = 1.0; lat->wdpenalty = 0.0; 

   /* fill in start/end/loop word entries with !NULL */
   wd = voc->nullWord;
   ln = lat->lnodes; ln->word = wd; ln->n=0; ln->v=0;
   ln = lat->lnodes+1; ln->word = wd; ln->n=0; ln->v=0;
   ln = lat->lnodes+nNode-1; ln->word = wd; ln->n=0; ln->v=0;
   ln = lat->lnodes+nNode-2; ln->word = wd; ln->n=0; ln->v=0;

   ln = lat->lnodes+2;
   for (i = 0; i< VHASHSIZE; i++)
      for ( wd = voc->wtab[i]; wd != NULL; wd = wd->next ) 
         if ((wd != voc->nullWord) && (wd != voc->subLatWord)) {
            ln->word = wd;
            ln++;
         }

   la =lat->larcs;
   la->start = lat->lnodes;  
   la->end = lat->lnodes+1; 
   la->lmlike = 0.0;
   la = lat->larcs+1;
   la->start = lat->lnodes+nNode-2;  
   la->end = lat->lnodes+nNode-1;
   la->lmlike = 0.0;
   la = lat->larcs+2;
   la->start = lat->lnodes+nNode-2;   
   la->end = lat->lnodes+1;
   la->lmlike = 0.0;

   la = lat->larcs+3;
   for (i = 0; i < voc->nwords; i++) {
      la->start = lat->lnodes+1;
      la->end = lat->lnodes+2+i;
      la->lmlike = log(1.0/(float) (voc->nwords));
      la++;
   }
   for (i = 0; i < voc->nwords; i++) {
      la->start = lat->lnodes+2+i;
      la->end = lat->lnodes+nNode-2;
      la->lmlike = 0.0;
      la++;
   }
   /* finally overwrite start/end !NULL words if sil at start/end */
   if (bStartId != NULL) {
      wd = GetWord(voc,bStartId,TRUE);
      ln = lat->lnodes; ln->word = wd;
      wd = GetWord(voc,bEndId,TRUE);
      ln = lat->lnodes+nNode-1; ln->word = wd;
   }
   return lat;
}


/*ProcessBoBiGram: Convert back-off bigram in nLM into lattice */ 
Lattice *ProcessBoBiGram(MemHeap *latHeap, Vocab *voc, NGramLM *nLM)
{
   int nNode,nArc;
   NEntry *ne;
   SEntry *se;
   Word wd,fromWd,toWd;
   LNode *ln,*fromNode,*toNode;
   LArc *la;

   lmId ndx[NSIZE+1];  
   int i,j,k;
   Lattice *lat;
   Boolean enterFound=FALSE;
   Boolean exitFound=FALSE;

   if (nLM->nsize > 2)
      HError(3030,"ProcessBoBiGram: Not BiGram LM: Order = %d",nLM->nsize);
   for (i=1; i <= nLM->counts[1]; i++) {
      if (nLM->wdlist[i] == enterId) enterFound = TRUE;
      if (nLM->wdlist[i] == exitId) exitFound = TRUE;
      if (enterFound && exitFound)
         break;
   }
   if (!enterFound) 
      HError(3030,"ProcessBoBiGram: Bigram does not contain ENTER symbol %s",
             enterId->name);
   if (!exitFound) 
      HError(3030,"ProcessBoBiGram: Bigram does not contain EXIT symbol %s",
             exitId->name);

   nNode = nLM->counts[1] + 1;   /* this is a maximum size */
   nArc =  nLM->counts[2] + 2*nLM->counts[1];
   lat = NewLattice(latHeap,nNode,nArc);
   lat->voc = voc;
   lat->lmscale = 1.0; lat->wdpenalty = 0.0; 
   /* go through the LM - get wordId from voc and add LM probs */
   wd = voc->nullWord;
   ln = lat->lnodes;
   ln->word = wd; ln->n=0; ln->v=0;
   for (i = 0 ; i <= NSIZE; i++) ndx[i] = 0; 

   for (i=1,j=1,k=0; i <= nLM->counts[1]; i++) {
      wd = GetWord(voc,nLM->wdlist[i],FALSE);
      if ((nLM->wdlist[i] == unknownId) && zapUnknown)
         continue;
      if (wd == NULL)
         HError(3031,"ProcessBoBiGram: Word %s in LM not in WordList",
                nLM->wdlist[i]->name);
      ln = lat->lnodes+j;
      ln->word = wd; ln->n=0; ln->v=0;
      wd->aux = (Ptr) j;
      if (nLM->wdlist[i] != enterId) {
         la = lat->larcs+k;
         la->start = lat->lnodes;
         la->end = lat->lnodes+j;
         la->lmlike = nLM->unigrams[i];
         k++;
      }
      j++;
   }
   lat->nn = j;
   lat->na = k;
   la = lat->larcs+k;
   for (i=1; i <= nLM->counts[1]; i++) {
      if ((nLM->wdlist[i] == unknownId) && zapUnknown)
         continue;
      if (nLM->wdlist[i] == exitId)
         continue;
      ndx[0] = i;
      ne = GetNEntry(nLM,ndx,FALSE);
      fromWd =  GetWord(voc,nLM->wdlist[i],FALSE);
      fromNode =  lat->lnodes+((int) fromWd->aux);
      la->start = fromNode;    /* backoff weight */
      la->end = lat->lnodes;
      if (ne==NULL) la->lmlike = 0.0;
      else la->lmlike = ne->bowt;
      la++; lat->na++;
      if (ne!=NULL)
         for (k = 0, se = ne->se; k < ne->nse; k++, se++) {
            if ((nLM->wdlist[se->word] == unknownId) && zapUnknown)
               continue;
            toWd = GetWord(voc,nLM->wdlist[se->word],FALSE);
            toNode = lat->lnodes+((int) toWd->aux);
            if (nLM->wdlist[se->word] != enterId) {
               la->start = fromNode;
               la->end = toNode;
               la->lmlike = se->prob;
               la++; lat->na++;
            }
         }
   }
   return lat;
}

/*ProcessMatBiGram: Convert matrix bigram in bg into lattice */ 
Lattice *ProcessMatBiGram(MemHeap *latHeap, Vocab *voc, MatBiLM *bg)
{
   int nNode,nArc;
   LNode *ln,*fromNode,*toNode;
   LArc *la;
   Word wd,fromWd,toWd;
   int i,j;
   int skipWord=0;
   Lattice *lat;
   Vector row;

   if (bg->wdlist[1] != enterId)
      HError(3030,"ProcessMatBiGram: Bigram does not contain ENTER symbol %s",
             enterId->name);
   if (bg->wdlist[bg->numWords] != exitId)
      HError(3030,"ProcessMatBiGram: Bigram does not contain EXIT symbol %s",
             exitId->name);
   nNode = bg->numWords;           /* this is a maximum size */
   nArc =  (bg->numWords-2)*bg->numWords;
   lat = NewLattice(latHeap,nNode,nArc);
   lat->voc = voc;
   lat->lmscale = 1.0; lat->wdpenalty = 0.0; 
   for (i=1,j=0; i <= bg->numWords; i++) {
      wd = GetWord(voc,bg->wdlist[i],FALSE);
      if ((bg->wdlist[i] == unknownId) && zapUnknown) {
         skipWord = i; 
         continue;
      }
      if (wd == NULL)
         HError(3031,"ProcessMatBiGram: Word %s in LM not in WordList",
                bg->wdlist[i]->name);
      ln = lat->lnodes+j;
      ln->word = wd; ln->n=0; ln->v=0;
      wd->aux = (Ptr) j;
      j++;
   }
   lat->nn = j;
   lat->na = (j-2)*j;
   la = lat->larcs;
   for (i=1,j=0; i < bg->numWords; i++) {
      row = bg->bigMat[i];
      fromWd =  GetWord(voc,bg->wdlist[i],FALSE);
      fromNode =  lat->lnodes+((int) fromWd->aux);
      if (i == skipWord) continue;
      for (j=2; j <= (i==1?bg->numWords-1:bg->numWords); j++) {
         if (j == skipWord) continue;
         toWd = GetWord(voc,bg->wdlist[j],FALSE);
         toNode = lat->lnodes+((int) toWd->aux);
         la->start = fromNode;
         la->end = toNode;
         la->lmlike = row[j];
         la++;
      }
   }
   return lat;
}

/* ProcessBiGram: Convert bigram in biLM into lattice */
Lattice *ProcessBiGram(MemHeap *latHeap, Vocab *voc, LModel *biLM)
{
   Lattice *lat = NULL;

   switch (biLM->type) {
   case boNGram:
      if (trace & T_TOP)
         printf("Converting back-off bigram -> lattice\n");
      lat = ProcessBoBiGram(latHeap,voc,biLM->data.ngram);
      break;
   case matBigram:
      if (trace & T_TOP)
         printf("Converting matrix bigram -> lattice\n");
      lat = ProcessMatBiGram(latHeap,voc,biLM->data.matbi);
      break;
   default:
      HError(3030,"ProcessBiGram: Unknown bigram type");
   }
   return lat;
}         

/* --------------- Word-Pair Grammar types and routines ------------- */

typedef struct _WordFllr{   /* storage for word followers */
   Word wd;
   struct _WordFllr *next;
}WordFllr;

typedef struct _GramEntry{
   int wordNum;
   Word wd;
   int numFllrs;
   WordFllr *entry;
   struct _GramEntry *next;
}GramEntry;

typedef struct {
   int nwords;
   int nfllrs;
   GramEntry *glist;   
   MemHeap entryHeap;
   MemHeap fllrHeap;
}WPGrammar;



/* --------------- Read the WP Grammar ----------------------- */

/* SkipHeader: skip comments at top of file */
/*             and return true if not eof   */
Boolean SkipHeader(FILE *f)
{
   int ch;
   Boolean inComment;
   
   ch = getc(f);        /* skip leading space */
   while (ch != EOF && isspace(ch))
      ch = getc(f);
   if (ch == '/') {
      ch = getc(f);      
      inComment = (ch == '*');
      if (!inComment)
         HError(3040,"SkipHeader: / char illegal if not in comment or delimiter");  
      else
         while (ch != EOF && inComment) {
            ch = getc(f);
            if (ch == '*') {
               ch = getc(f);
               inComment = (ch != '/');
            }
         }
   }     
   ch = getc(f);
   while (ch != EOF && isspace(ch))
      ch = getc(f);
   if (ch == EOF) return FALSE;
   ungetc(ch,f);
   return TRUE;
}

/* SkipSpacesEoln: skip white to eoln return true if not eof */
Boolean SkipSpacesEoln(FILE *f)
{
   int ch;

   ch = getc(f);
   while (ch != EOF && isspace(ch) && ch != '\n')
      ch = getc(f);
   if (ch == EOF) return FALSE;
   return TRUE;
}

/* NumberEntries: number all entries in the wpg */
void NumberEntries(WPGrammar *wpg, Word sentEnd)
{
   GramEntry *gid;
   int count = 0;

   gid = wpg->glist;
   while (gid != NULL) {
      if (gid->wd !=  sentEnd) {
         count++; 
         gid->wordNum = count;
      }
      else
         gid->wordNum = 0;
      gid = gid->next;
   }
}      


void ReadWPGrammar(WPGrammar *wpg, Vocab * voc, char *gramFn)
{
   FILE *gf;
   char buf[255];
   int ch;
   Word newWord;
   GramEntry *newGram = NULL;
   Boolean newEntry;
   WordFllr *wdfllr; 
   Word sentEnd;   

   
   sentEnd = GetWord(voc,GetLabId("SENTENCE-END",TRUE),TRUE); 
   if ( (gf = fopen(gramFn,"r")) == NULL)
      HError(3010,"ReadWPGrammar: Cannot open word-pair grammar file %s",gramFn);
   if (trace && T_TOP)
      printf("Loading word-pair grammar %s\n",gramFn);
   if (!SkipHeader(gf))
      HError(3040,"ReadWPGrammar: Unexpected eof while reading %s", gramFn);
   do {
      ch = getc(gf);
      newEntry = (ch == '>');
      if (wpg->nwords == 0 && !newEntry)
         HError(3040,"ReadWPGrammar: > expected while reading %s", gramFn);
      if (!ReadLabel(gf,buf)) {
         if (newEntry)
            HError(3040,"ReadWPGrammar: Word entry expected in %s",gramFn);
         else
            break;
      }
      if (newEntry) {
         newWord = GetWord(voc,GetLabId(buf, TRUE),FALSE);
         if (newWord == NULL)
            HError(3040,"ReadWPGrammar: Word %s not in wordlist but in grammar file",buf);
         newGram = (GramEntry *) New(&wpg->entryHeap,sizeof(GramEntry));
         newWord->aux = (Ptr) newGram;
         newGram->wd = newWord;
         newGram->next = wpg->glist;
         newGram->entry = NULL;
         wpg->glist = newGram;
         wpg->nwords++;
      } 
      else {
         wdfllr = (WordFllr *) New(&wpg->fllrHeap,sizeof(WordFllr));
         wdfllr->next = newGram->entry;
         wdfllr->wd = GetWord(voc,GetLabId(buf, TRUE),FALSE);
         if (wdfllr->wd == NULL)
            HError(3040,"ReadWPGrammar: Word %s not in wordlist but in grammar file",buf);
         newGram->entry = wdfllr;
         newGram->numFllrs++;
         wpg->nfllrs++;
      }
   } while (SkipSpacesEoln(gf));
   fclose(gf);
   NumberEntries(wpg,sentEnd);
   if (trace & T_TOP)
      printf("Word-pair grammar %s loaded\n",gramFn);
}

Lattice *ProcessWordPair(MemHeap *latHeap, Vocab *voc, char *gramFn)
{
   int nNode,nArc;
   LNode *ln,*toNode;
   LArc *la;
   Word wd;
   Lattice *lat;
   int j;
   WPGrammar wpg;
   GramEntry *gid;
   WordFllr *fid;

   wpg.nwords = 0;
   wpg.nfllrs = 0;
   wpg.glist = NULL;
   CreateHeap(&wpg.entryHeap,"GramEntry Heap",MHEAP,sizeof(GramEntry),
              1.2,100,1000);
   CreateHeap(&wpg.fllrHeap,"WordFllr Heap",MHEAP,sizeof(WordFllr),
              1.2,1000,10000);
   ReadWPGrammar(&wpg,voc,gramFn);

   nNode = wpg.nwords+1;
   nArc = wpg.nfllrs;
   lat = NewLattice(latHeap,nNode,nArc);
   lat->voc = voc;
   lat->lmscale = 1.0; lat->wdpenalty = 0.0; 
   ln = lat->lnodes; ln->n=0; ln->v=0;
   ln = lat->lnodes+nNode-1; ln->n=0; ln->v=0;
   gid = wpg.glist;
   j = 0;
   while (gid != NULL) {
      ln = lat->lnodes+gid->wordNum;
      ln->word = gid->wd; ln->n=0; ln->v=0;
      fid = gid->entry;
      while (fid != NULL) {
         la = lat->larcs+j;
         la->start = ln;
         toNode = lat->lnodes+((GramEntry *) (fid->wd->aux))->wordNum;
         if (toNode == lat->lnodes) toNode = lat->lnodes+nNode-1;
         la->end = toNode;
         la->lmlike = log(1.0/((float) gid->numFllrs));
         j++;
         fid = fid->next;
      }
      gid = gid->next;
   }
   if (bStartId != NULL) {
      wd = GetWord(voc,bStartId,TRUE);
      ln = lat->lnodes; ln->word = wd;
      wd = GetWord(voc,bEndId,TRUE);
      ln = lat->lnodes+nNode-1; ln->word = wd;
   }
   else {
      ln =  lat->lnodes; ln->word = voc->nullWord;
      ln = lat->lnodes+nNode-1; ln->word = voc->nullWord;
   }
   return lat;
}


/* ------------------- End of HBuild.c --------------------------------- */



