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
/*      File: LFoF.c - compute frequency of frequency files    */
/* ----------------------------------------------------------- */

char *lfof_version = "!HVER!LFoF:   3.4.1 [CUED 12/03/09]";
char *lfof_vc_id = "$Id: LFoF.c,v 1.1.1.1 2006/10/11 09:54:44 jal58 Exp $";

#include "HShell.h"
#include "HMem.h"
#include "HMath.h"
#include "HWave.h"
#include "HLabel.h"
#include "LWMap.h"
#include "LGBase.h"
#include "LUtil.h"
#include "LModel.h"
#include "LPCalc.h"

/* 
   This tool reads one or more gram base files and counts 
   the frequency of 1-gram, 2-gram, ... N-gram events.
   The output is a table of frequency of frequency stats
   fof[i][j] which is the number of times a j-gram occurred
   exactly i times.
*/

/* -------------------------- Trace Flags ------------------------ */

static int trace = 0;
#define T_TOP  0001     /* Top Level tracing */

/* ---------------- Configuration Parameters --------------------- */

static ConfParam *cParm[MAXGLOBS];
static int nParm = 0;            /* total num params */


/* ---------------------- Global Variables ----------------------- */

static int        nSize = 0;        /* ngram size N */
static char       *fofFN = NULL;    /* output FoF file name */
static char       *mapFN = NULL;    /* word map file name */
static WordMap    wmap;             /* word map for this corpus */
static NGInputSet inSet;            /* input file set */
static int        fofSize = 128;    /* size of fof table */
static FoFTab     *fofTab;          /* FoF Table */

/* ---------------- Process Command Line ------------------------- */

/* SetConfParms: set conf parms relevant to this tool */
void SetConfParms(void)
{
   int i;

   nParm = GetConfig("LFOF", TRUE, cParm, MAXGLOBS);
   if (nParm>0){
      if (GetConfInt(cParm,nParm,"TRACE",&i)) trace = i;
   }
}

void ReportUsage(void)
{
   printf("\nUSAGE: LFoF [options] wordMap fofFile <[mult] file ....>\n\n");
   printf(" Option                                       Default\n\n");
   printf(" -f n    set FoF table size to n              128\n");
   printf(" -n n    set N-gram                           3\n");
   PrintStdOpts("");
   printf("\n\n");
}

int main(int argc, char *argv[])
{
   char *s;

   void Initialise(void);

   InitShell(argc,argv,lfof_version,lfof_vc_id);
   InitMem();
   InitMath();
   InitWave();
   InitLabel();
   InitWMap();
   InitGBase();
   InitPCalc();

   if (!InfoPrinted() && NumArgs() == 0)
      ReportUsage();
   if (NumArgs() == 0) Exit(EXIT_SUCCESS);

   SetConfParms();

   while (NextArg() == SWITCHARG) {
      s = GetSwtArg();
      if (strlen(s)!=1) 
         HError(16719,"Bad switch %s; must be single letter",s);
      switch(s[0]){
         case 'f':
            fofSize = GetChkedInt(1, 1000, s); break;
         case 'n':
            nSize = GetChkedInt(1, MAXNG, s); break;
         case 'T':
            trace = GetChkedInt(0,077,s); break;
         default:
            HError(16719,"LFoF: Unknown switch %s",s);
      }
   }
   if (NextArg() != STRINGARG)
      HError(16719,"LFoF: map file name expected");
   mapFN = GetStrArg();
   if (NextArg() != STRINGARG)
      HError(16719,"LFoF: FoF file name expected");
   fofFN = GetStrArg();
   Initialise();
   if (trace&T_TOP) {
      printf("Calculating FoF table\n"); fflush(stdout);
   }
   fofTab = CreateFoFTab(&gstack,fofSize,nSize);
   ComputeFoFTab(fofTab,nSize,&inSet);
   WriteFoFTab(fofFN,fofTab,NULL);

   Exit(EXIT_SUCCESS);
   return EXIT_SUCCESS; /* never reached -- make compiler happy */
}


/* ------------------------ Initialisation ----------------------- */

/* Initialise: initialise global data structures */
void Initialise(void)
{
   float weight;
   char *fn;

   CreateWordMap(mapFN,&wmap,1);
   SortWordMap(&wmap);

   weight = 1.0;
   CreateInputSet(&gstack,&wmap,&inSet);
   while (NextArg() == STRINGARG || NextArg() == FLOATARG) {
      if (NextArg() == FLOATARG)  
	 weight = GetFltArg();
      if (weight==0.0 || weight<-10000.0 || weight>10000.0)
         HError(-16719,"Unlikely ngram weight[%.4f]",weight);
      if (NextArg()!=STRINGARG)
         HError(16719,"Gram base file name expected");
      fn = GetStrArg();
      AddInputGFile(&inSet,fn,weight);
      if (trace&T_TOP)
         printf("Input file %s added, weight=%.4f\n",fn,weight);
   }
   if (nSize==0) 
      nSize = inSet.N;
}

/* ---------------------- End of LFoF.c ----------------------- */

