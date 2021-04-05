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
/*    File: HSGen: Generate Sentences from a Lattice           */
/* ----------------------------------------------------------- */

char *hsgen_version = "!HVER!HSGen:   3.4.1 [CUED 12/03/09]";
char *hsgen_vc_id = "$Id: HSGen.c,v 1.1.1.1 2006/10/11 09:55:01 jal58 Exp $";

#include "HShell.h"
#include "HMem.h"
#include "HMath.h"
#include "HSigP.h"
#include "HWave.h"
#include "HVQ.h"
#include "HAudio.h"
#include "HParm.h"
#include "HLabel.h"
#include "HModel.h"
#include "HDict.h"
#include "HNet.h"


/* -------------------------- Trace Flags ------------------------ */

static int trace = 0;
#define T_TOP  0001     /* Top Level tracing */
#define T_DET  0002     /* Detailed trace of lattice traversal */

/* ---------------------- Global Variables ----------------------- */


static int ngen = 100;              /* num sents to gen */
static Boolean lnum = FALSE;        /* enable line numbers */
static Boolean stats = FALSE;       /* enable grammar stats estimation */
static Boolean quiet = FALSE;       /* suppress sentence output */

/* ---------------- Configuration Parameters --------------------- */

static ConfParam *cParm[MAXGLOBS];
static int nParm = 0;            /* total num params */

/* ------------------ Process Command Line ------------------------- */


/* SetConfParms: set conf parms relevant to this tool */
void SetConfParms(void)
{
   int i;

   nParm = GetConfig("HSGEN", TRUE, cParm, MAXGLOBS);
   if (nParm>0){
      if (GetConfInt(cParm,nParm,"TRACE",&i)) trace = i;
   }
}

void ReportUsage(void)
{
   printf("\nUSAGE: HSGen [options] latticeFile dictFile\n\n");
   printf(" Option                                       Default\n\n");
   printf(" -l      Include line numbers                 off\n");
   printf(" -n N    Number of sentences to generate      100\n");
   printf(" -q      Suppress sentence output             off\n");
   printf(" -s      Compute grammar stats                off\n");
   PrintStdOpts("");
   printf("\n\n");
}

int main(int argc, char *argv[])
{
   char *s,*lfn,*dfn;
   void  GenSentences(char *latfn, char *dicfn);
  
   if(InitShell(argc,argv,hsgen_version,hsgen_vc_id)<SUCCESS)
      HError(3400,"HSGen: InitShell failed");

   InitMem();   InitLabel();
   InitMath();  InitSigP();
   InitWave();  InitAudio();
   InitVQ();    InitModel();
   if(InitParm()<SUCCESS)
      HError(3200,"HSGen: InitParm failed");
   InitDict();
   InitNet();

   if (!InfoPrinted() && NumArgs() == 0)
      ReportUsage();
   if (NumArgs() == 0) Exit(0);

   SetConfParms();
   while (NextArg() == SWITCHARG) {
      s = GetSwtArg();
      if (strlen(s)!=1) 
         HError(3419, "Bad switch %s; must be single letter", s);
      switch(s[0]){
      case 's':
         stats = TRUE; break;
      case 'l':
         lnum = TRUE; break;
      case 'n':
         ngen = GetChkedInt(1,1000000,s); break;
      case 'q':
         quiet = TRUE; break;
      case 'T':
         trace = GetChkedInt(0,07,s); break;
      default:
         HError(3419, "Unknown switch %s", s);
      }
   }
   if (NextArg()!=STRINGARG)
      HError(3419, "lattice file name expected");
   lfn = GetStrArg(); 
   if (NextArg()!=STRINGARG)
      HError(3419, "dictionary file name expected");
   dfn = GetStrArg(); 
   GenSentences(lfn,dfn);
   Exit(0);
   return (0);          /* never reached -- make compiler happy */
}

/* ------------------------ Sentence Generation --------------------- */

static Lattice *lat;    /* The defining syntax Lattice */
static Vocab   voc;     /* associated vocab */
static double psSum;    /* total log prob of all sentences */
static long lenSum;     /* total length of all sentences */

/* ComputeVSize: compute vocabulary size of grammar */
void ComputeVSize(void)
{
   int i,wordsUsed = 0,nullNodes=0;
   NodeId ln;

   for(i=0,ln=lat->lnodes;i<lat->nn;i++,ln++)
      ln->word->aux = (void *)0;
   for(i=0,ln=lat->lnodes;i<lat->nn;i++,ln++)
      if (ln->word == voc.nullWord) 
         ++nullNodes;
      else if (ln->word->aux == (void *)0){
         ++wordsUsed; ln->word->aux = (void *)1;
      }
   for(i=0,ln=lat->lnodes;i<lat->nn;i++,ln++)
      ln->word->aux = (void *)0;
   printf("Number of Nodes = %d [%d null], Vocab Size = %d\n",
          lat->nn,nullNodes,wordsUsed);      
}

/* Select: randomly select one from the nfoll arcs attached to n using */
int Select(NodeId n, int nfoll, LogFloat *prob)
{
   Vector wtp; /* word transition probs */
   float x,csum,sum = 0.0;
   int i,sel;
   ArcId a;

   for (i=1,a = n->foll; i<=nfoll; i++, a = a->farc) 
      sum += a->lmlike;
   if (sum==0.0) { /* no lm probs are set */
      x = RandomValue() * nfoll;
      sel = (int) (x + 1);   if (sel>nfoll) sel = nfoll;
      *prob = -log(1.0/(float)nfoll);
   }else{
      wtp = CreateVector(&gstack,nfoll);
      sum = 0.0;
      for (i=1,a = n->foll; i<=nfoll; i++, a = a->farc){
         wtp[i] = exp(a->lmlike); sum += wtp[i];
      }
      csum = 0.0;
      for (i=1; i<=nfoll; i++){
         x = wtp[i]/sum;
         wtp[i] = csum + x;  csum = wtp[i];
      }
      csum = 0.0;
      x = RandomValue();
      for (i=1,a = n->foll; i<=nfoll; i++, a = a->farc) {
         *prob = -a->lmlike;
         if (x<=wtp[i] || i==nfoll) break;
      }
      sel = i;
      FreeVector(&gstack,wtp);
   }
   return sel;
}

/* RandSucc: select a successor randomly */
NodeId RandSucc(NodeId n)
{
   int sel,j,nfoll;
   ArcId a;
   LogFloat prob = 0.0;
   
   nfoll = NumNodeFoll(n);
   if (nfoll==0) return NNODE;
   sel = (nfoll>1)?Select(n,nfoll,&prob):1;
   if (stats) psSum += prob;
   if (trace&T_DET){
      printf("  preds:");
      for (a = n->foll; a != NARC; a=a->farc)
         printf(" %s",a->end->word->wordName->name);
   }
   a = n->foll;
   for (j=1; j<sel; j++){
      a=a->farc;
      if (a==NARC) 
         HError(3420, "RandSucc: null arc j=%d,sel=%d,nfoll=%d",j,sel,nfoll);
   }
   if (trace&T_DET) printf(": choose %s\n",a->end->word->wordName->name);
   return a->end;
}

/* PrintWord: print outsym of 1st pron associated with given node if there is
              one, otherwise print node name */
void PrintWord(NodeId n)
{
   Word w;
   Pron p;

   if ((w = n->word)==NULL)
      HError(3420, "PrintWord: node %d has no word",n->n);
   if ((p = w->pron)==NULL)
      printf("%s ",w->wordName->name);
   else if (p->outSym!=NULL)
      printf("%s ", p->outSym->name);
}

/* GenSent: Generate a single sentence using lat, returns length of sent */
int GenSent(int snum)
{
   NodeId n;
   int len = 0;
   
   if (trace&T_TOP)
      printf("Sentence %d:\n",snum);
   if (lnum && !quiet) printf("%d. ",snum);
   n = FindLatStart(lat);
   if (n->word != voc.nullWord)  {
      len++;
      if (!quiet) PrintWord(n);
   }
   while ((n = RandSucc(n)) != NULL) {
      if (n->word != voc.nullWord)  {
         len++;
         if (!quiet) PrintWord(n);
      }
   }
   if (!quiet) {
      printf("\n"); fflush(stdout);
   }
   return len;
}

/* GenSentences: top level control of the sentence generator */
void  GenSentences(char * latfn, char * dicfn)
{
   int i,min,max,len;
   double e,p;
   MemHeap lheap;
   FILE *f;
   Boolean isPipe;

   InitVocab(&voc);
   if(ReadDict(dicfn,&voc)<SUCCESS)
      HError(3413,"GenSententces:ReadDict failed" );
   CreateHeap(&lheap,"Lattice Heap",MSTAK,1,0.4,1000,5000);
   if ((f=FOpen(latfn,NetFilter,&isPipe)) == NULL)
      HError(3410,"GenSentences: Can't open lattice file %s",latfn);
   if((lat = ReadLattice(f, &lheap, &voc, TRUE, FALSE))==NULL)
      HError(3410,"GenSentences: ReadLattice failed");
   FClose(f,isPipe);

   if (trace&T_TOP)
      printf("HSGen %d sents from lattice %s/dictionary %s\n",
             ngen,latfn,dicfn);
   psSum = 0.0; lenSum = 0; min = 100000; max = 0;
   if (trace&T_DET) quiet = TRUE;  /* kill output if detailed trace */
   for (i=1; i<=ngen; i++){
      len = GenSent(i);
      lenSum += len;
      if (len>max) max = len;
      if (len<min) min = len;
   }
   if (stats)  {
      ComputeVSize();
      e = psSum / lenSum;
      p = exp(e);
      e = e / log(2.0);
      printf("Entropy = %f,  Perplexity = %f\n",e,p);
      printf("%d Sentences: average len = %.1f, min=%d, max=%d\n",
             ngen,(float)lenSum/ngen,min,max);
   }
}

/* ------------------------------------------------------------ */
/*                      END:  HSGen.c                           */
/* ------------------------------------------------------------ */

