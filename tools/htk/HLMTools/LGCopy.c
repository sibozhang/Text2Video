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
/*   File: LGCopy - copy ngram files and optionally map OOVs   */
/* ----------------------------------------------------------- */

char *lgcopy_version = "!HVER!LGCopy:   3.4.1 [CUED 12/03/09]";
char *lgcopy_vc_id = "$Id: LGCopy.c,v 1.1.1.1 2006/10/11 09:54:44 jal58 Exp $";

#include "HShell.h"
#include "HMem.h"
#include "HMath.h"
#include "HWave.h"
#include "HLabel.h"
#include "LWMap.h"
#include "LCMap.h"
#include "LGBase.h"

/* 
   This tool reads one or more gram base files and stores them as a
   sorted sequence of gram files.  If a word list is specified, each
   word in each copied n-gram which is not in the list is mapped to
   DefaultUnknown (see LWMap.h).  The default is to copy all n-grams
   but an option is provided to only copy n-grams which have been
   mapped.  Input files can be weighted by prepending with an
   integer count.
*/

/* -------------------------- Trace Flags ------------------------ */

static int trace = 0;
#define T_TOP  0001     /* Top Level tracing */
#define T_SAV  0002     /* Monitor Buffer Saving */

/* ---------------- Configuration Parameters --------------------- */

static ConfParam *cParm[MAXGLOBS];
static int nParm = 0;            /* total num params */


/* ---------------------- Global Variables ----------------------- */

static int nSize     = 0;           /* ngram size */
static int ngbSize   = 2000000;     /* ngram buffer size */
static int freeSlots = 100;         /* free class slots */
static char *rootFN  = "data";      /* gbase root file name */
static int dumpOfs   = 0;           /* initial numeric ext of gbase files */
static char *dbsDir  = NULL;        /* directory to store gbase files */
static char *mapFN   = NULL;        /* word map file name */
static char *cmapFN  = NULL;        /* word list file name */
static char *omapFN  = NULL;        /* output map filename */

static Boolean outMapped = FALSE;   /* output mapped IDs only */
static Boolean mapWords = FALSE;    /* map words to classes */

static WordMap    *omap = NULL;     /* output buffer map */
static WordMap    wmap;             /* word map for this corpus */
static ClassMap   cmap;             /* word list for OOV mapping */
static NGInputSet inset;            /* input file set */
static NGBuffer   *ngb;             /* output ngram buffer */
static MemHeap    ngbHeap;          /* memory for NGBuffers */

/* Function prototypes */
static void Initialise(void);
static void CopyFiles(void);


/* ---------------- Process Command Line ------------------------- */

/* SetConfParms: set conf parms relevant to this tool */
void SetConfParms(void)
{
   int i;

   nParm = GetConfig("LGCOPY", TRUE, cParm, MAXGLOBS);
   if (nParm>0){
      if (GetConfInt(cParm,nParm,"TRACE",&i)) trace = i;
   }
}

void ReportUsage(void)
{
   printf("\nUSAGE: LGCopy [options] map <[mult] file ....>\n\n");
   printf(" Option                                       Default\n\n");
   printf(" -a n    allow up to 'n' new classes          100\n");
   printf(" -b n    set ngram buffer size to 'n'         2000000\n");
   printf(" -d s    database directory 's'               current\n");
   printf(" -i n    set output gram file start index     0\n");
   printf(" -m fn   save new word map to 'fn'            off\n");
   printf(" -n n    create n-grams of size 'n'           max\n");
   printf(" -o      output class mappings only           off\n");
   printf(" -r s    set root data base file name 's'     data\n");
   printf(" -w fn   load class map from 'fn'             none\n");
   PrintStdOpts("");
   printf("\n\n");
}

int main(int argc, char *argv[])
{
   char *s;

   InitShell(argc,argv,lgcopy_version,lgcopy_vc_id);
   InitMem();
   InitMath();
   InitWave();
   InitLabel();
   InitWMap();
   InitCMap();
   InitGBase();
   SetConfParms();

   if (!InfoPrinted() && NumArgs() == 0)
      ReportUsage();
   if (NumArgs() == 0) Exit(EXIT_SUCCESS);

   while (NextArg() == SWITCHARG) {
      s = GetSwtArg();
      if (strlen(s)!=1) 
         HError(16219,"Bad switch %s; must be single letter",s);
      switch(s[0]){
         case 'a':
            freeSlots = GetChkedInt(10,10000000,s); break;
         case 'b':
	    ngbSize = GetChkedInt(1,10000000,s); break;
         case 'd':
            if (NextArg() != STRINGARG)
               HError(16219,"Database file directory expected");
            dbsDir = GetStrArg(); break;
         case 'i':
            dumpOfs = GetChkedInt(0, 100000, s); break;
         case 'm':
            if (NextArg()!=STRINGARG)
               HError(16219,"Output map filename expected");
            omapFN = GetStrArg(); break;
         case 'n':
            nSize = GetChkedInt(1, MAXNG, s); break;
         case 'o':
            outMapped = TRUE; break;
         case 'r':
            if (NextArg()!=STRINGARG)
               HError(16219,"Gram base root file name expected");
            rootFN = GetStrArg(); break;
         case 'w':
            if (NextArg() != STRINGARG)
               HError(16219,"Class map filename expected");
	    cmapFN = GetStrArg();
	    mapWords = TRUE;
	    break;
         case 'T':
            trace = GetChkedInt(0,077,s); break;
         default:
            HError(16219,"LGCopy: Unknown switch %s",s);
      }
   }
   /* Look for wordmap */
   if (NextArg() != STRINGARG)
      HError(16219,"LGCopy: map file name expected");
   mapFN = GetStrArg();
   Initialise();
   /* Warn about any left-over command line parameters */
   if (NumArgs() != 0)
      HError(-16219,"LGCopy: unused args left on cmd line");
   /* Do the actual copy */
   CopyFiles();
   /* And exit */
   Exit(EXIT_SUCCESS);
   return EXIT_SUCCESS;
}


/* ------------------------ Initialisation ----------------------- */

/* BuildMap: create new map with both word and class IDs */
static WordMap *BuildOutputMap(MemHeap *heap,ClassMap *cmap)
{
   char s[256];
   int i,nme;
   MapEntry *me,*tme;
   ClassEntry *ce;
   WordMap *omap,*wmap;

   wmap = cmap->wmap;
   for (nme=0,me=wmap->me,i=0; i<wmap->used; i++,me++)
      if (me->class==NULL) nme++;
   nme += cmap->entries;

   omap = (WordMap *) New(heap,sizeof(WordMap));
   CreateWordMap(NULL,omap,nme);
   omap->hasCnts = FALSE;
   omap->isSorted = FALSE;
   omap->seqno = wmap->seqno+1;
   omap->htkEsc = wmap->htkEsc;
   sprintf(s,"%s%%%%%s",wmap->name,cmap->name); 
   omap->name = CopyString(heap,s);
   sprintf(s,"derived from %s using class map %s",wmap->name,cmap->name); 
   omap->source = CopyString(heap,s);

   /* copy class IDs to the new map */
   tme = omap->me; nme = 0;
   for (i=0; i<CLMHASHSIZE; i++) {
      for (ce=cmap->htab[i]; ce!=NULL; ce=ce->next) {
	 tme->ndx = ce->ndx;
	 tme->sort = 0;
	 tme->count = 0;
	 tme->class = NULL;

	 omap->id[nme] = ce->id;
	 ce->id->aux = tme;        /* !!! aux field no longer point to class entry !!! */
	 nme++; tme++;
      }
   }

   /* copy remaining word IDs to new map */
   for (me = wmap->me,i=0; i<wmap->used; i++,me++) {
      if ((ce=me->class)!=NULL) {
	 tme = (MapEntry *) ce->id->aux;
	 tme->count += me->count;
         continue;
      }
      omap->me[nme] = wmap->me[i];
      omap->id[nme] = wmap->id[i];
      omap->id[nme]->aux = (Ptr) (omap->me+nme);
      nme++;
   }
   omap->used = nme;
   BuildLookupTables(omap);


   SortWordMap(omap);
   return omap;
}


/* Initialise: initialise global data structures */
static void Initialise(void)
{
   float weight;
   char *fn,path[256];

   CreateWordMap(mapFN, &wmap, freeSlots);
   SortWordMap(&wmap);
   if (cmapFN != NULL) {
      CreateClassMap(cmapFN, &cmap, &wmap);
      omap = BuildOutputMap(&gstack, &cmap);
      if (omapFN!=NULL)
	 SaveWordMap(omapFN,omap,FALSE);
   }
   weight = 1.0;
   CreateInputSet(&gstack, &wmap, &inset);
   while (NextArg() == STRINGARG || NextArg() == FLOATARG) {
      if (NextArg() == FLOATARG)  
	 weight = GetFltArg();
      if (weight==0.0 || weight<-10000.0 || weight>10000.0)
         HError(-16219,"Unlikely ngram weight[%.4f]",weight);
      if (NextArg()!=STRINGARG)
         HError(16219,"Gram base file name expected");
      fn = GetStrArg();
      AddInputGFile(&inset,fn,weight);
      if (trace&T_TOP)
         printf("Input file %s added, weight=%.4f\n",fn,weight);
   }
   if (nSize==0) nSize = inset.N;

   CreateHeap(&ngbHeap,"NGB mem",MSTAK,1,0.0,1000,1000);
   MakeFN(rootFN,dbsDir,NULL,path);
   ngb = CreateNGBuffer(&ngbHeap,nSize,ngbSize,path,(omap)?omap:&wmap);
   ngb->fndx += dumpOfs;
}

/* ----------------- File Processing -------------------- */

/* SaveOutBuffer: save the output buffer */
static void SaveOutBuffer(void)
{
   if (trace&T_TOP) {
      printf(" saving %d ngrams to file %s.%d\n",
              ngb->used, ngb->fn, ngb->fndx);
   }
   WriteNGBuffer(ngb,"LGCopy");
}

/* CompressBuffer: and save if necessary or mustSave is TRUE */
static void CompressBuffer(Boolean mustSave)
{
   float compx;

   if (ngb->used == 0) return;
   SortNGBuffer(ngb);  
   compx = 100.0 * (float)ngb->used / (float)ngb->poolsize;
   if (trace&T_SAV)
      printf(" buffer %s.%d compressed%s to %.3f%%\n",
              ngb->fn, ngb->fndx, mustSave?"[must save]":"",compx);
   if (compx > 60.0 || mustSave) SaveOutBuffer();
}

#define MAP_ENTRY(map,i) map.me + ((i<BASEWORDNDX) ? i : i-BASEWORDNDX+map.nClass)

/* CopyFiles: read input N-grams and write to output */
static void CopyFiles(void)
{
   int i;
   float count;
   MapEntry *me;
   ClassEntry *ce;
   Boolean outThis, success;
   UInt ng[MAXNG], nin, nout;

   nin = nout = 0;
   OpenInputSet(&inset);
   if (trace&T_TOP) {
      printf("Copying %d input files to output files with %d entries\n",
              inset.nFiles, ngbSize);
      if (cmapFN != NULL)
         printf("Class map = %s%s\n", cmapFN, outMapped ? " [Class mappings only]":"");
   }

   while (GetNextNGram(&inset, ng, &count, nSize)) {
      ++nin;
      if (trace&T_TOP && (nin%500000==0))
         printf("  %d ngrams read\n",nin);

      if (mapWords)
      {
	 /* Only output this if we're either outputting all entries or it's in a class */
	 outThis = !outMapped;

         for (i=0; i<nSize; i++) {
            me = wmap.me + GetMEIndex(&wmap, ng[i]);
            ce = me->class;
            if (ce) {
	       ng[i] = ce->ndx; /* Map to class */
	       outThis = TRUE;
            }
         }
      }
      else
      {
	outThis = TRUE;
      }

#ifdef FLOAT_COUNT
      fp = (float *) (ng+ngram); *fp = count; 
#else
      ng[nSize] = (UInt) count;
#endif

      if (outThis) {
         ++nout;
	 /* StoreNGram returns TRUE if the NGram buffer is full */
	 success = StoreNGram(ngb, ng);
         if (success) {
            if (mapWords)      /* filtering (eg. using classmap) so need to compress */
	       CompressBuffer(FALSE);  /* also calls SaveOutBuffer() */
            else               /* otherwise assume inputs already sorted */
               SaveOutBuffer();
         }
      }
   } 
   if (mapWords)      /* filtering so need to compress */
      CompressBuffer(TRUE);
   else               /* otherwise assume inputs already sorted */
      SaveOutBuffer();
   printf("%d out of %d ngrams stored in %d files\n",nout,nin,ngb->fndx);
   CloseInputSet(&inset);
}

/* ---------------------- End of LGCopy.c ----------------------- */

