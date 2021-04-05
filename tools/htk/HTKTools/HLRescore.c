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
/* author: Gunnar Evermann <ge204@eng.cam.ac.uk>               */
/* ----------------------------------------------------------- */
/*         Copyright:                                          */
/*         2001-2002  Cambridge University                     */
/*                    Engineering Department                   */
/*                                                             */
/*   Use of this software is governed by a License Agreement   */
/*    ** See the file License for the Conditions of Use  **    */
/*    **     This banner notice must not be removed      **    */
/*                                                             */
/* ----------------------------------------------------------- */
/*       File: HLRescore.c: Lattice rescoring/pruning          */
/* ----------------------------------------------------------- */

/*#### todo:

     - implement lattice oracle WER calculation
     - allow batch processing?
*/

char *hlrescore_version = "!HVER!HLRescore:   3.4.1 [CUED 12/03/09]";
char *hlrescore_vc_id = "$Id: HLRescore.c,v 1.1.1.1 2006/10/11 09:55:01 jal58 Exp $";

#include "HShell.h"
#include "HMem.h"
#include "HMath.h"
#include "HWave.h"
#include "HLabel.h"
#include "HAudio.h"
#include "HParm.h"
#include "HModel.h"
#include "HUtil.h"
#include "HDict.h"
#include "HNet.h"
#include "HRec.h"
#include "HLM.h"
#include "HLat.h"

/* -------------------------- Trace Flags & Vars ------------------------ */

#define T_TOP  00001      /* Basic progress reporting */
#define T_TRAN 00002      /* Output transcriptions */
#define T_LAT  00004      /* Lattice I/O */
#define T_MEM  00010      /* Memory usage, start and finish */

static int trace = 0;

/* ---------------- Configuration Parameters --------------------- */

static ConfParam *cParm[MAXGLOBS];
static int nParm = 0;            /* total num params */

/* -------------------------- Global Variables etc ---------------------- */

static char *dictfn;		/* dict filename from commandline */
static char *latInDir = NULL;   /* Lattice input dir, set by -L  */
static char *latInExt = "lat";  /* Lattice Extension, set by -X */
static char *labInDir = NULL;   /* Label input dir, set by -L  */
static char *labInExt = "lab";  /* Label Extension, set by -X */

static FileFormat ifmt=UNDEFF;  /* Label input file format */
static FileFormat ofmt=UNDEFF;  /* Label output file format */
static char *labOutDir = NULL;  /* output label file directory */
static char *labOutExt = "rec"; /* output label file extension */
static char *labOutForm = NULL; /* output label format */
static char *latOutForm = NULL; /* output lattice format */

static double lmScale = 1.0;    /* LM scale factor */
static double acScale = 1.0;    /* acoustic scale factor */
static LogDouble wordPen = 0.0; /* inter word log penalty */
static double prScale = 1.0;    /* pronunciation scale factor */

static Boolean fixPronprobs = FALSE; /* get pron probs from dict */

static char *lmFile = NULL;     /* LM filename */
static LModel *lm;              /* LM for expandin lattices */
static char *wpNetFile = NULL;  /* word pair LM network filename */
static Lattice *wpNet;          /* the word level recognition network */
static Vocab vocab;		/* wordlist or dictionary */

static char *startWord;         /* word at start of Lattice (!SENT_START) */
static LabId startLab;          /* corresponding LabId */
static char *endWord;           /* word at end of Lattice (!SENT_END) */
static LabId endLab;            /* corresponding LabId */
static LabId nullLab;           /* !NULL LabId */
static char *startLMWord;       /* word at start in LM (<s>) */
static char *endLMWord;         /* word at end in LM (</s>) */
static char *mergeDir;          /* lattice merging direction */

static Boolean fixBadLats = FALSE;         /* fix final word in lattices */
static Boolean sortLattice = TRUE;         /* sort lattice nodes by time & posterior */

static LogDouble pruneInThresh = - LZERO;  /* beam for pruning (-t) */
static LogDouble pruneOutThresh = - LZERO; /* beam for pruning (-u) */
static LogDouble pruneInArcsPerSec = 0.0;  /* arcs per second threshold (-t) */
static LogDouble pruneOutArcsPerSec = 0.0; /* arcs per second threshold (-u) */

/* operations to perform: */
static Boolean pruneInLat = FALSE;  /* -t */
static Boolean writeLat = FALSE;    /* -w */
static Boolean expandLat = FALSE;   /* -n */
static Boolean pruneOutLat = FALSE; /* -u */
static Boolean findBest = FALSE;    /* -f */
static Boolean calcStats = FALSE;   /* -c */
static Boolean lab2Lat = FALSE;     /* -I */
static Boolean mergeLat = FALSE;    /* -m */

/* -------------------------- Heaps ------------------------------------- */

static MemHeap latHeap;
static MemHeap labHeap;
static MemHeap lmHeap;
static MemHeap transHeap;

/* -------------------------- Prototypes -------------------------------- */

void SetConfParms (void);
void ReportUsage (void);
void ProcessLattice (char *latfn);
void ProcessLabels (char *labfn);


/* ---------------- Process Command Line ------------------------- */

/* SetConfParms: set conf parms relevant to this tool */
void SetConfParms(void)
{
   int i;
   char buf[MAXSTRLEN];
   Boolean b;

   nParm = GetConfig ("HLRESCORE", TRUE, cParm, MAXGLOBS);
   if (nParm>0){
      if (GetConfInt (cParm, nParm, "TRACE", &i)) trace = i;
      if (GetConfStr (cParm, nParm, "STARTWORD", buf))
         startWord = CopyString (&gstack, buf);
      if (GetConfStr (cParm, nParm, "ENDWORD", buf))
         endWord = CopyString (&gstack, buf);
      if (GetConfStr (cParm, nParm, "STARTLMWORD", buf))
         startLMWord = CopyString (&gstack, buf);
      if (GetConfStr (cParm, nParm, "ENDLMWORD", buf))
         endLMWord = CopyString (&gstack, buf);
      if (GetConfBool (cParm, nParm, "FIXBADLATS", &b)) fixBadLats = b;
      if (GetConfBool (cParm, nParm, "SORTLATTICE", &b)) sortLattice = b;
   }
}

void ReportUsage(void)
{
   printf("\nUSAGE: HLRescore [options] vocabFile Files...\n\n");
   printf(" Option                                   Default\n\n");
   printf(" -i s    Output transcriptions to MLF s       off\n"); 
   printf(" -l s    dir to store label/lattice files     current\n");
   printf(" -m s    merge nodes and arcs of lattice      off\n");
   printf(" -n s    load n-gram LM and expand lattice    off\n");
   printf(" -wn s   load word pair LM network            off\n");
   printf(" -o s    output label formating NCSTWMX       none\n");
   printf(" -t f [f] input lattice pruning threshold     off\n");
   printf(" -u f [f] output lattice pruning threshold    off\n");
   printf(" -p f    inter model trans penalty (log)      0.0\n");
   printf(" -s f    grammar scale factor                 1.0\n");
   printf(" -a f    acoustic scale factor                1.0\n");
   printf(" -r f    pronunciation scale factor           1.0\n");
   printf(" -d      get pronprobs from dict              off\n");
   printf(" -c      calculate statistics                 off\n");
   printf(" -f      find 1-best transcription            off\n");
   printf(" -w      write output lattices                off\n");
   printf(" -q s    output lattice format                tvaldmnr\n"); 
   printf(" -y s    output label file extension          rec\n");
   PrintStdOpts("ILSXTGP");
   printf("\n\n");
}

int main(int argc, char *argv[])
{
   char *s, *latfn, *labfn;
   FILE *nf;
   Boolean isPipe;

   /*#### new error code range */
   if(InitShell (argc, argv, hlrescore_version, hlrescore_vc_id) < SUCCESS)
      HError (4000, "HLRescore: InitShell failed");
  
   InitMem();
   InitMath();
   InitWave();
   InitLabel();
   InitAudio();
   if (InitParm()<SUCCESS) 
      HError(4000,"HLRescore: InitParm failed");
   InitModel();
   InitUtil();
   InitDict();
   InitNet(); 
   InitRec();
   InitLM();
   InitLat();

   if (!InfoPrinted() && NumArgs() == 0)
      ReportUsage();
   if (NumArgs() == 0) Exit(0);
  
   SetConfParms();

   while (NextArg() == SWITCHARG) {
      s = GetSwtArg();
      switch (s[0]){
      case 'L':
         if (lab2Lat) {
            if (NextArg() != STRINGARG)
               HError (4019,"HLRescore: Label file directory expected");
            labInDir = GetStrArg(); 
         }
         else {
         if (NextArg() != STRINGARG)
            HError (4019,"HLRescore: Lattice file directory expected");
         latInDir = GetStrArg(); 
         }
         break;
      case 'X':
         if (lab2Lat) {
            if (NextArg() != STRINGARG)
               HError (4019, "HLRescore: Label filename extension expected");
            labInExt = GetStrArg(); 
                    }
         else {
         if (NextArg() != STRINGARG)
            HError (4019, "HLRescore: Lattice filename extension expected");
         latInExt = GetStrArg(); 
         }
         break;
      case 'i':
         if (NextArg() != STRINGARG)
            HError (4019, "HLRescore: Output MLF file name expected");
         if (SaveToMasterfile (GetStrArg()) < SUCCESS)
            HError (4014, "HLRescore: Cannot write to MLF");
         break;

      case 'I':
         if (NextArg() != STRINGARG)
            HError (4019, "HLRescore: Input MLF file name expected");
         LoadMasterFile (GetStrArg ());
         if (latInDir) {
            HError (4019,"HLRescore: Cannot load lattices while converting MLF to lattices");
         }
         else {
            lab2Lat = TRUE;
         }
         break;

      case 'P':
         if (NextArg() != STRINGARG)
            HError (4019, "HLRescore: Output Label File format expected");
         if((ofmt = Str2Format (GetStrArg ())) == ALIEN)
            HError (-4089, "HLRescore: Warning ALIEN Label output file format set");
         break;
      case 'q':
	 if (NextArg () != STRINGARG)
	    HError (4019, "HLRescore: Output lattice format expected");
	 latOutForm = GetStrArg ();
	 break;
      
      case 'l':
         if (NextArg() != STRINGARG)
            HError (4019, "HLRescore: Output Label file directory expected");
         labOutDir = GetStrArg(); 
         break;
      case 'o':
         if (NextArg() != STRINGARG)
            HError (4019, "HLRescore: Output label format expected");
         labOutForm = GetStrArg(); 
         break;
      case 'y':
         if (NextArg() != STRINGARG)
            HError (4019, "HLRescore: Output label file extension expected");
         labOutExt = GetStrArg(); break;
      
      case 'p':
         if (NextArg() != FLOATARG)
            HError (4019, "HLRescore: word insertion penalty expected");
         wordPen = GetChkedFlt (-1000.0, 1000.0, s); 
         break;
      case 's':
         if (NextArg() != FLOATARG)
            HError (4019, "HLRescore:  grammar scale factor expected");
         lmScale = GetChkedFlt (0.0, 1000.0, s); 
         break;
      case 'a':
         if (NextArg() != FLOATARG)
            HError (4019, "HLRescore:  acoustic scale factor expected");
         acScale = GetChkedFlt (0.0, 1000.0, s); 
         break;
      case 'r':
         if (NextArg() != FLOATARG)
            HError (4019, "HLRescore:  pronunciation scale factor expected");
         prScale = GetChkedFlt (0.0, 1000.0, s); 
         break;

      case 'd':
         fixPronprobs = TRUE;
         break;

      case 't':
         if (NextArg() != FLOATARG)
            HError (4019, "HLRescore:  lattice pruning threshold expected");
         pruneInThresh = GetChkedFlt (0.0, 10000.0, s); 
         pruneInLat = TRUE;
         if (NextArg() == FLOATARG)
            pruneInArcsPerSec = GetChkedFlt (0.0, 150000.0, s);
         break;
      case 'u':
         if (NextArg() != FLOATARG)
            HError (4019, "HLRescore:  lattice pruning threshold expected");
         pruneOutThresh = GetChkedFlt (0.0, 10000.0, s); 
         pruneOutLat = TRUE;
         if (NextArg() == FLOATARG)
            pruneOutArcsPerSec = GetChkedFlt (0.0, 150000.0, s);
         break;

      case 'n':
         if (NextArg() != STRINGARG)
            HError (4019, "HLRescore: language model file name expected");
         lmFile = GetStrArg(); 
         expandLat = TRUE;
         break;

      case 'f':
         findBest = TRUE;
         break;
         
      case 'w':
         if (!strcmp(s, "w"))
         writeLat = TRUE;
         else if (!strcmp(s, "wn")){
            wpNetFile = GetStrArg(); 
            expandLat = TRUE;            
         }
         else HError(4019, "Unknown switch %s",s);
         break;

      case 'c':
         calcStats = TRUE;
         break;
         
      case 'T':
         trace = GetChkedInt(0, 100, s); 
         break;

      case 'm':
         if (NextArg() != STRINGARG)
            HError(4019, "Lattice merge direction expected");
         mergeDir = GetStrArg();
         if (*mergeDir != 'f' && *mergeDir != 'b')
            HError(4019, "Incorrect lattice merge direction (f/b) : %s!", mergeDir);      
         mergeLat = TRUE;
         break;         

      default:
         HError (4019, "HLRescore: Unknown switch %s",s);
      }
   }
   
   if (!writeLat && !findBest && !calcStats)
      HError (4019, "HLRescore: No operation specified. What do you want me to do?");

   /* init Heaps */
   CreateHeap (&latHeap, "Lattice heap", MSTAK, 1, 0, 8000, 80000);
   CreateHeap (&labHeap, "Labels heap", MSTAK, 1, 0, 8000, 80000);
   CreateHeap (&lmHeap, "LM heap", MSTAK, 1, 1.0, 10000, 100000);
   CreateHeap (&transHeap, "Transcription heap",MSTAK, 1, 0, 8000, 80000);

   if (NextArg() != STRINGARG)
      HError(4019, "Vocab file name expected");
   dictfn = GetStrArg();
  
   /* Read dictionary */
   if (trace & T_TOP) 
      printf ("Reading dictionary from %s\n", dictfn);
   InitVocab (&vocab);
   if (ReadDict (dictfn, &vocab)<SUCCESS)
      HError(4013, "HLRescore: ReadDict failed");
     
   if (lmFile && wpNetFile) {
      HError (4019, "HLRescore: Cant load both N-gram and word pair network, what are you doing?!");
   }
   
   if (!lab2Lat && wpNetFile) {
      HError (4019, "HLRescore: Word network LMs not applicable to lattices");
   }

   /* language model */
   if (lmFile) {
      if (trace & T_TOP) 
         printf ("Reading LM from %s\n", lmFile);
      lm = ReadLModel (&lmHeap, lmFile);
   }
   
   /* word network */
   if (wpNetFile) {
      if (trace & T_TOP) 
         printf ("Reading word pair LM from %s\n", wpNetFile);
      if ( (nf = FOpen(wpNetFile, NetFilter, &isPipe)) == NULL)
         HError(4019, "HLRescore:: Cannot open word pair LM network file %s", wpNetFile);
      if((wpNet = ReadLattice(nf, &lmHeap, &vocab, FALSE, FALSE)) == NULL)
         HError(4019,"HLRescore: Read lattice failed");
      FClose(nf, isPipe);
   }

   /* merge lattice nodes and arcs */
   if (mergeLat) {
      if (trace & T_TOP) {
         fprintf(stdout, "\n Converting lattices to equivalent word networks ...\n\n");
         fflush(stdout);
      }
      if (pruneOutLat) {
         pruneOutLat = FALSE;
         HError(-9999, "\n Ignoring output lattice pruning ... \n");
      }
      if (expandLat) {
         expandLat = FALSE;
         HError(-9999, "\n Ignoring lattice expansion ... \n");
      }
      if (findBest) {
         findBest = FALSE;
         HError(-9999, "\n Ignoring 1-best transcription output ... \n");
      }
      sortLattice = FALSE; 
   }
   
   LatSetBoundaryWords (startWord, endWord, startLMWord, endLMWord);
   
   /* process dictionary */
   startLab = GetLabId (startWord, FALSE);
   if (!startLab)
      HError (9999, "HLRescore: cannot find STARTWORD '%s'\n", startWord);
   endLab = GetLabId (endWord, FALSE);
   if (!endLab)
      HError (9999, "HLRescore: cannot find ENDWORD '%s'\n", endWord);
   nullLab = vocab.nullWord->wordName;
   
   while (NumArgs() > 0) {
      if (NextArg() != STRINGARG)
         HError (4019, "HLRescore: Transcription file name expected");
      if (!lab2Lat) {
      latfn = GetStrArg();

      if (trace & T_TOP)
         printf ("File: %s\n", latfn);  fflush(stdout);

      ProcessLattice (latfn);
   }
      else {
         labfn = GetStrArg();

         if (trace & T_TOP)
            printf ("File: %s\n", labfn);  fflush(stdout);

         ProcessLabels (labfn);
      }
   }

   if (trace & T_MEM) {
      printf("Memory State on Completion\n");
      PrintAllHeapStats();
   }
  
   Exit (0);
   return(0); 
}


/* ProcessLattice

     apply all the requested operations on lattice
*/
void ProcessLattice (char *latfn)
{
   Lattice *lat;
   char lfn[MAXSTRLEN];
   FILE *lf;
   Boolean isPipe;

   MakeFN (latfn, latInDir, latInExt, lfn);
  
   if ((lf = FOpen(lfn,NetFilter,&isPipe)) == NULL)
      HError(4010,"HLRescore: Cannot open Lattice file %s", lfn);
  
   lat = ReadLattice (lf, &latHeap, &vocab, FALSE, FALSE);
   FClose(lf, isPipe);

   if (!lat)
      HError (4013, "HLRescore: can't read lattice");
   
   if (fixBadLats)
      FixBadLat (lat);

   if (fixPronprobs)
      FixPronProbs (lat, &vocab);

   if (trace & T_LAT)
      printf ("lattice size: %d nodes/ %d arcs\n", lat->nn, lat->na);

   lat->lmscale = lmScale;
   lat->wdpenalty = wordPen;
   lat->acscale = acScale;
   lat->prscale = prScale;

   /* prune original lattice */
   if (pruneInLat) {
      lat = LatPrune (&latHeap, lat, pruneInThresh, pruneInArcsPerSec);
   }

   /* expand lattice with new LM */
   if (expandLat) {
#ifndef NO_LAT_LM
      lat = LatExpand (&latHeap, lat, lm);
#else 
      HError (4090, "LatExpand not supported. Recompile without NO_LAT_LM");
#endif
   }

   /* merge lattice nodes and arcs */
   if (mergeLat) {
      if (*mergeDir == 'f') 
         lat = MergeLatNodesArcs(lat, &latHeap, TRUE);      
      else 
         lat = MergeLatNodesArcs(lat, &latHeap, FALSE);
   }

   /* find 1-best Transcription */
   if (findBest) {
      Transcription *trans;

      trans = LatFindBest (&transHeap, lat, 1);
      if (trace & T_TRAN)
         PrintTranscription (trans, "1-best path");

      /* write transcription */
      if (labOutForm)
         FormatTranscription (trans, 
                              1.0e7, FALSE, FALSE,
                              strchr(labOutForm,'X')!=NULL,
                              strchr(labOutForm,'N')!=NULL,strchr(labOutForm,'S')!=NULL,
                              strchr(labOutForm,'C')!=NULL,strchr(labOutForm,'T')!=NULL,
                              strchr(labOutForm,'W')!=NULL,strchr(labOutForm,'M')!=NULL);
      
      MakeFN (latfn, labOutDir, labOutExt, lfn);
      if (LSave (lfn, trans, ofmt) < SUCCESS)
         HError (4014, "ProcessLattice: Cannot save file %s", lfn);
      ResetHeap (&transHeap);
   }

   /* prune generated lattice */
   if (pruneOutLat) {
      lat = LatPrune (&latHeap, lat, pruneOutThresh, pruneOutArcsPerSec);
   }

   /* calc lattice stats */
   if (calcStats) {
      CalcStats (lat);
   }

   /* write lattice */
   if (writeLat) {
      LatFormat form;
      int i;
      LNode *ln;

      if (sortLattice)
         LatSetScores (lat);
      else
         for(i=0, ln=lat->lnodes; i<lat->nn; i++, ln++)
            ln->score=0.0;

      MakeFN (latfn, labOutDir, latInExt, lfn);
      lf = FOpen (lfn, NetOFilter, &isPipe);
      if (!lf)
         HError (4014, "ProcessLattice: Could not open file '%s' for lattice output", lfn);
      if (!latOutForm)
         form = HLAT_DEFAULT|HLAT_PRLIKE;
      else {
         char *p;
         for (p = latOutForm, form=0; *p != 0; ++p) {
            switch (*p) {
            case 'A': form|=HLAT_ALABS; break;
            case 'B': form|=HLAT_LBIN; break;
            case 't': form|=HLAT_TIMES; break;
            case 'v': form|=HLAT_PRON; break;
            case 'a': form|=HLAT_ACLIKE; break;
            case 'l': form|=HLAT_LMLIKE; break;
            case 'd': form|=HLAT_ALIGN; break;
            case 'm': form|=HLAT_ALDUR; break;
            case 'n': form|=HLAT_ALLIKE; break;
            case 'r': form|=HLAT_PRLIKE; break;
            }
         }
      }
      if (WriteLattice (lat, lf, form) < SUCCESS)
         HError(4014, "ProcessLattice: WriteLattice failed");
      
      FClose (lf, isPipe);
   }

   if (trace & T_MEM) {
      printf("Memory State after processing lattice\n");
      PrintAllHeapStats();
   }
   ResetHeap (&latHeap);
}


/* ProcessLabels

     apply all the requested operations on labels
*/
void ProcessLabels (char *labfn)
{
   Lattice *lat;
   char lfn[MAXSTRLEN];
   FILE *lf;
   Boolean isPipe;
   LabList *ll = NULL, *expll = NULL;
   LLink l;
   Transcription *reftrans;
   int i, N;

   MakeFN (labfn, labInDir, labInExt, lfn);
  
   reftrans = LOpen(&labHeap, lfn, ifmt);
   if (reftrans->numLists >= 1)
      ll = GetLabelList(reftrans,1);
   if (!ll && !startLab)
      HError (4013, "HLRescore: can't convert empty labels to lattice");

   N = CountLabs(ll);  /* total number of symbols */
   expll = CreateLabelList(&labHeap, 0);
   
   /* add start and end word symbols if needed */
   if (GetLabN(ll, 1)->labid != startLab) {
      l = AddLabel(&labHeap, expll, startLab, 0.0, 0.0, 0.0);
   }
   for (i=1; i<=N; i++) {
      l = AddLabel(&labHeap, expll, GetLabN(ll, i)->labid, 0.0, 0.0, 0.0);
   }
   if (GetLabN(ll, 1)->labid != endLab) {
      l = AddLabel(&labHeap, expll, endLab, 0.0, 0.0, 0.0);                  
   }
     
   lat = LatticeFromLabels(expll, nullLab, &vocab, &labHeap);

   if (trace&T_TOP) {
      printf("Created lattice with %d nodes / %d arcs from label file\n",
             lat->nn, lat->na);
      fflush(stdout);
   }

   if (!lat)
      HError (4013, "HLRescore: can't convert labels to lattice");
   
   if (fixBadLats)
      FixBadLat (lat);

   if (trace & T_LAT)
      printf ("lattice size: %d nodes/ %d arcs\n", lat->nn, lat->na);

   lat->utterance = labfn;
   lat->vocab = dictfn;
   lat->voc = &vocab;
   lat->acscale = acScale;
   lat->prscale = prScale;

   /* prune original lattice */
   if (pruneInLat) {
      lat = LatPrune (&latHeap, lat, pruneInThresh, pruneInArcsPerSec);
   }

   /* apply LM to label lattice */
   if (expandLat) {
      if (lm) {
         lat->net = lmFile;
         lat->lmscale = lmScale;
         lat->wdpenalty = wordPen;
         ApplyNGram2LabLat(lat, lm);
      }
      if (wpNet) {
         lat->net = wpNetFile;
         ApplyWPNet2LabLat(lat, wpNet);
      }
   }

   /* find 1-best Transcription */
   if (findBest) {
      Transcription *trans;

      trans = LatFindBest (&transHeap, lat, 1);
      if (trace & T_TRAN)
         PrintTranscription (trans, "1-best path");

      /* write transcription */
      if (labOutForm)
         FormatTranscription (trans,
                              1.0e7, FALSE, FALSE,
                              strchr(labOutForm,'X')!=NULL,
                              strchr(labOutForm,'N')!=NULL,strchr(labOutForm,'S')!=NULL,
                              strchr(labOutForm,'C')!=NULL,strchr(labOutForm,'T')!=NULL,
                              strchr(labOutForm,'W')!=NULL,strchr(labOutForm,'M')!=NULL);
      
      MakeFN (labfn, labOutDir, labOutExt, lfn);
      if (LSave (lfn, trans, ofmt) < SUCCESS)
         HError (4014, "ProcessLattice: Cannot save file %s", lfn);
      ResetHeap (&transHeap);
   }

   /* prune generated lattice */
   if (pruneOutLat) {
      lat = LatPrune (&latHeap, lat, pruneOutThresh, pruneOutArcsPerSec);
   }

   /* calc lattice stats */
   if (calcStats) {
      CalcStats (lat);
   }

   /* write lattice */
   if (writeLat) {
      LatFormat form;
      int i;
      LNode *ln;

      if (sortLattice)
         LatSetScores (lat);
      else
         for(i=0, ln=lat->lnodes; i<lat->nn; i++, ln++)
            ln->score=0.0;

      MakeFN (labfn, labOutDir, latInExt, lfn);
      lf = FOpen (lfn, NetOFilter, &isPipe);
      if (!lf)
         HError (4014, "ProcessLattice: Could not open file '%s' for lattice output", lfn);
      if (!latOutForm)
         form = HLAT_DEFAULT|HLAT_PRLIKE;
      else {
         char *p;
         for (p = latOutForm, form=0; *p != 0; ++p) {
            switch (*p) {
            case 'A': form|=HLAT_ALABS; break;
            case 'B': form|=HLAT_LBIN; break;
            case 't': form|=HLAT_TIMES; break;
            case 'v': form|=HLAT_PRON; break;
            case 'a': form|=HLAT_ACLIKE; break;
            case 'l': form|=HLAT_LMLIKE; break;
            case 'd': form|=HLAT_ALIGN; break;
            case 'm': form|=HLAT_ALDUR; break;
            case 'n': form|=HLAT_ALLIKE; break;
            case 'r': form|=HLAT_PRLIKE; break;
            }
         }
      }
      if (WriteLattice (lat, lf, form) < SUCCESS)
         HError(4014, "ProcessLattice: WriteLattice failed");
      
      FClose (lf, isPipe);
   }

   if (trace & T_MEM) {
      printf("Memory State after processing lattice\n");
      PrintAllHeapStats();
   }
   ResetHeap (&latHeap);
   ResetHeap (&labHeap);
}


/*  CC-mode style info for emacs
    Local Variables:
    c-file-style: "htk"
    End:
*/
