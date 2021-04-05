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
/*      File: LGList.c: Display contents of a gram file        */
/* ----------------------------------------------------------- */

char *lglist_version = "!HVER!LGList:   3.4.1 [CUED 12/03/09]";
char *lglist_vc_id = "$Id: LGList.c,v 1.1.1.1 2006/10/11 09:54:44 jal58 Exp $";

#include "HShell.h"
#include "HMem.h"
#include "HMath.h"
#include "HWave.h"
#include "HLabel.h"
#include "LUtil.h"
#include "LWMap.h"
#include "LGBase.h"

/* -------------------------- Trace Flags ------------------------ */

static int trace = 0;
#define T_TOP  0001              /* Top Level tracing */

/* ---------------------- Global Variables ----------------------- */

static  WordMap wmap;            /* and the word map */

/* ---------------- Configuration Parameters --------------------- */

static ConfParam *cParm[MAXGLOBS];
static int nParm = 0;            /* total num params */

/* ---------------- Process Command Line ------------------------- */

/* SetConfParms: set conf parms relevant to this tool */
void SetConfParms(void)
{
   int i;

   nParm = GetConfig("LGLIST", TRUE, cParm, MAXGLOBS);
   if (nParm>0){
      if (GetConfInt(cParm,nParm,"TRACE",&i)) trace = i;
   }
}

void ReportUsage(void)
{
   printf("\nUSAGE: LGList [options] mapfile gramfile ...\n\n");
   printf(" Option                                       Default\n\n");
   printf(" -i n    add id n to filter list              none\n");
   printf(" -f w    add word w to filter list            none\n");
   PrintStdOpts("");
   printf("\n\n");
}
   
int main(int argc, char *argv[])
{
   char *s;
   void DisplayNGramFile(char *fn);
   void AddWrdtoFilter(char *s, int ndx);
   void ResolveFilterList(void);

   InitShell(argc,argv,lglist_version,lglist_vc_id);
   InitMem();
   InitMath();
   InitWave();
   InitLabel();
   InitLUtil();
   InitWMap();
   InitGBase();

   if (!InfoPrinted() && NumArgs() == 0)
      ReportUsage();
   if (NumArgs() == 0) Exit(EXIT_SUCCESS);

   SetConfParms();

   while (NextArg() == SWITCHARG) {
      s = GetSwtArg();
      if (strlen(s) !=1 )
         HError(16119,"LGList: Bad switch %s; must be single letter",s);
      switch(s[0]){
         case 'f':
            if (NextArg()!=STRINGARG)
               HError(16119,"LGList: word expected");
            AddWrdtoFilter(GetStrArg(),-1);
            break;
         case 'i':
            if (NextArg()!=INTARG)
               HError(16119,"LGList: word index expected");
            AddWrdtoFilter(NULL,GetIntArg());
            break;
         case 'T':
            trace = GetChkedInt(0,077,s); break;
         default:
            HError(16119,"LGList: Unknown switch %s",s);
      }
   }
   if (NextArg()!=STRINGARG)
      HError(16119,"LGList: word map file name expected");
   CreateWordMap(GetStrArg(),&wmap,0); 
   ResolveFilterList();
   while (NextArg() == STRINGARG)
      DisplayNGramFile(GetStrArg());

   Exit(EXIT_SUCCESS);
   return EXIT_SUCCESS; /* never reached -- make compiler happy */
} 


/* ------------------------ Gram Filtering ------------------- */

typedef struct fitem {
   LabId wid;
   int  ndx;
   struct fitem * next;
} FItem;

static MemHeap fmem;
static FItem *filtList = NULL;

/* AddWrdtoFilter: add word or index to filter list */
void AddWrdtoFilter(char *s, int ndx)
{
   FItem *p;
   
   if (filtList == NULL)
         CreateHeap(&fmem,"filtlist",MSTAK,1,0.0,1000,1000);
   p = (FItem *)New(&fmem,sizeof(FItem));
   p->next = filtList; filtList = p;
   p->wid = (s==NULL)?NULL:GetLabId(s,TRUE);
   p->ndx = ndx;
}

/* ResolveFilterList:  add in missing names/ndxes */
void ResolveFilterList(void)
{
   FItem *p;
   
   for (p=filtList; p != NULL; p=p->next) {
      if (p->wid == NULL) p->wid = WordLMName(p->ndx,&wmap);
      if (p->ndx == -1)   p->ndx = WordLMIndex(p->wid);
   }
}

/* ShowNgram: return true if any item is in the filter list */
Boolean ShowNgram(int N, NGram ng)
{
   FItem *p;
   int i;
   
   for (p=filtList; p != NULL; p=p->next) 
      for (i=0; i<N; i++)
         if (p->ndx == ng[i]) return TRUE;
   return FALSE;
}

/* --------------------- Display N Grams ----------------- */

/* DisplayNGramFile: display contents of given NGram file */
void DisplayNGramFile(char *fn)
{
   UInt ng[MAXNG],nPrinted,N;
   NGSource ngs;
   
   OpenNGramFile(&ngs,fn,&wmap); N = ngs.info.N;
   printf("\n%d-Gram File %s[%d entries]:\n", N,fn, ngs.nItems);
   if (strlen(ngs.txtsrc) > 0)
      printf(" Text Source: %s\n",ngs.txtsrc);
   nPrinted = 0;
   do {
      ReadNGram(&ngs, ng);
      if (filtList == NULL || ShowNgram(N,ng)) {
         PrintNGram(N,ng,&wmap); ++nPrinted;
      }
   }while (ngs.nItems > 0);
   printf("%d ngram entries printed\n",nPrinted);
   CloseNGramFile(&ngs);
}

/* ----------------------------------------------------------- */
/*                      END:  LGList.c                         */
/* ----------------------------------------------------------- */
