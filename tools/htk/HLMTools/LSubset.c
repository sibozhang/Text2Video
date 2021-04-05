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
/*      File: LSubset.c: produce a subset map                  */
/* ----------------------------------------------------------- */

char *lsubset_version = "!HVER!LSubset:   3.4.1 [CUED 12/03/09]";
char *lsubset_vc_id = "$Id: LSubset.c,v 1.1.1.1 2006/10/11 09:54:44 jal58 Exp $";

#include "HShell.h"
#include "HMem.h"
#include "HMath.h"
#include "HWave.h"
#include "HLabel.h"
#include "LUtil.h"
#include "LWMap.h"
#include "LCMap.h"

/* -------------------------- Trace Flags ------------------------ */

static int trace = 0;
#define T_TOP  0001              /* Top Level tracing */

/* ---------------------- Global Variables ----------------------- */

static WordMap  wmap;            /* the word map */
static ClassMap cmap;            /* class map */
static WordMap  *omap;           /* the ouput word/class map */
static char     *omapFN;         /* output map filename */
static int      newWords = 1000; /* new words from class map */
  
/* ---------------- Configuration Parameters --------------------- */

static ConfParam *cParm[MAXGLOBS];
static int nParm = 0;            /* total num params */

/* ---------------- Process Command Line ------------------------- */

/* SetConfParms: set conf parms relevant to this tool */
void SetConfParms(void)
{
   int i;

   nParm = GetConfig("LSubset", TRUE, cParm, MAXGLOBS);
   if (nParm>0){
      if (GetConfInt(cParm,nParm,"TRACE",&i)) trace = i;
   }
}

void ReportUsage(void)
{
   printf("\nUSAGE: LSubset [options] inMapFile classMap outMapFile\n\n");
   printf(" Option                                       Default\n\n");
   printf(" -a n    allow up to n new classes in map     1000\n");
   PrintStdOpts("");
   printf("\n\n");
}
   
int main(int argc, char *argv[])
{
   char *s;
   WordMap *BuildOutputMap(MemHeap *heap,ClassMap *cmap);

   InitShell(argc,argv,lsubset_version,lsubset_vc_id);
   InitMem();
   InitMath();
   InitWave();
   InitLabel();
   InitLUtil();
   InitWMap();
   InitCMap();

   if (!InfoPrinted() && NumArgs() == 0)
      ReportUsage();
   if (NumArgs() == 0) Exit(EXIT_SUCCESS);

   SetConfParms();

   while (NextArg() == SWITCHARG) {
      s = GetSwtArg();
      if (strlen(s) !=1 )
         HError(17019,"LSubset: Bad switch %s; must be single letter",s);
      switch(s[0]){
         case 'a':
            newWords = GetChkedInt(10, 10000000, s); break;
         case 'T':
            trace = GetChkedInt(0,077,s); break;
         default:
            HError(17019,"LSubset: Unknown switch %s",s);
      }
   }
   if (NextArg()!=STRINGARG)
      HError(17019,"LSubset: word map filename expected");
   CreateWordMap(GetStrArg(),&wmap,newWords);
   if (NextArg()!=STRINGARG)
      HError(17019,"LSubset: word map filename expected");
   CreateClassMap(GetStrArg(),&cmap,&wmap);
   if (NextArg()!=STRINGARG)
      HError(17019,"LSubset: output word map filename expected");
   omapFN = GetStrArg();
   omap = BuildOutputMap(&gstack,&cmap);
   SaveWordMap(omapFN,omap,FALSE);

   Exit(EXIT_SUCCESS);
   return EXIT_SUCCESS; /* never reached -- make compiler happy */
} 

/* BuildMap: create new map with both word and class IDs */
WordMap *BuildOutputMap(MemHeap *heap,ClassMap *cmap)
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

/* ----------------------------------------------------------- */
/*                      END:  LSubset.c                         */
/* ----------------------------------------------------------- */
