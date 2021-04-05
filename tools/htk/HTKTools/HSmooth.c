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
/* File: HSmooth.c: Perform Parameter Smoothing on a HMM Set   */
/* ----------------------------------------------------------- */

char *hsmooth_version = "!HVER!HSmooth:   3.4.1 [CUED 12/03/09]";
char *hsmooth_vc_id = "$Id: HSmooth.c,v 1.1.1.1 2006/10/11 09:55:01 jal58 Exp $";


#include "HShell.h"     /* HMM ToolKit Modules */
#include "HMem.h"
#include "HMath.h"
#include "HSigP.h"
#include "HAudio.h"
#include "HWave.h"
#include "HVQ.h"
#include "HParm.h"
#include "HLabel.h"
#include "HModel.h"
#include "HTrain.h"
#include "HUtil.h"

#define MAXMONOPHONES  500        /* max number of monophones in HMMSet */

/* Trace Flags */
#define T_TOP   0001    /* Top level tracing */
#define T_INT   0002    /* Interpolation weights */
#define T_OPT   0004    /* Optimisation algorithm details */

/* -------------- Global Settings ------------------ */
/* Command line controlled */
static char * hmmDir = NULL;     /* directory to look for hmm def files */
static char * hmmExt = NULL;     /* hmm def file extension */
static char * newDir = NULL;     /* directory to store new hmm def files */
static char * newExt = NULL;     /* extension of new reestimated hmm files */
static char * statFN;            /* stats file, if any */
static float minVar  = 0.0;      /* minimum variance (diagonal only) */
static float mixWeightFloor=0.0; /* Floor for mixture weights */
static int minEgs    = 3;        /* min examples to train a model */
static int maxStep   = 16;       /* max number of binary chops */
static float epsilon = 0.0001;   /* binary chop convergence criterion */
static UPDSet uFlags = (UPDSet) (UPMEANS|UPVARS|UPTRANS|UPMIXES);   /* update flags */
static Boolean stats = FALSE;    /* enable statistics reports */
static Boolean saveBinary = FALSE;  /* save output in binary  */
static Boolean ldBinary = TRUE;     /* load in binary */
static int trace     = 0;        /* Trace level */
static int nBlk = 0;       /* number of data blocks */

/* HMMSet related */
static HMMSet hset;        /* Set of HMMs to be re-estimated */
static int maxStates = 0;        /* max states in any model */
static int maxMixes = 0;         /* max mixtures in any model */
static int vSize;                /* input vector size */
static int nStreams;             /* number of data streams */
static HSetKind hsKind;          /* kind of loaded hmm set */
static int nLogHmms;       /* number of logical HMM's */
static int nPhyHmms;       /* number of physical HMM's */

/* Dynamic storage */
static int aSize;            /* size of current aSet */
static HLink *aSet;          /* array[1..aSize]of allophone */
static StreamElem **sSet;    /* array[1..aSize]of stream */
static Vector *wbar;         /* array[0..nBlk]of weight vector */
static Vector *wcd;          /* array[0..nBlk]of weight vector */
static int nPhones;          /* number of monophones */
static LabId *monophones;    /* array[1..nPhones]of LabId */
static int totalT=0;         /* total number of frames in training data */
static LogDouble totalPr;    /* total log prob of all training utterances */
static Vector vFloor[SMAX];  /* variance floor - default is all zero */

/* Configuration parameters */
static ConfParam *cParm[MAXGLOBS];
static int nParm = 0;        /* total num params */

/* Memory heaps */
static MemHeap hmmStack;     /*For Storage of dynamic data structures */
static MemHeap aSetStack;
static MemHeap sSetStack;
static MemHeap wsStack;
static MemHeap labIdStack;
static MemHeap wtAccStack;


/* -------------------------- Config Params ----------------------- */

/* SetConfParms: set conf parms relevant to HSmooth  */
void SetConfParms(void)
{
   int i;
   Boolean b;
   
   nParm = GetConfig("HSMOOTH", TRUE, cParm, MAXGLOBS);
   if (nParm>0) {
      if (GetConfInt(cParm,nParm,"TRACE",&i)) trace = i;
      if (GetConfBool(cParm,nParm,"SAVEBINARY",&b)) saveBinary = b;
      if (GetConfBool(cParm,nParm,"BINARYACCFORMAT",&b)) ldBinary = b;
   }
}

/* ------------------ Process Command Line -------------------------- */

void ReportUsage(void)
{
   printf("\nUSAGE: HSmooth [options] hmmList AccFiles...\n\n");
   printf(" Option                                       Default\n\n");
   printf(" -b f    set convergence epsilon              0.0001\n");
   printf(" -c N    max num steps in binary chop         16\n");
   printf(" -d s    dir to find hmm definitions          current\n");
   printf(" -m N    set min examples needed per model    3\n");
   printf(" -o s    extension for new hmm files          as src\n");
   printf(" -s s    print statistics to file s           off\n"); 
   printf(" -u tmvw Update t)rans m)eans v)ars w)ghts    tmvw\n");
   printf(" -v f    Set minimum variance to f            0.0\n");
   printf(" -w f    Set mix weight floor to f*MINMIX     0.0\n");
   printf(" -x s    extension for hmm files              none\n");
   PrintStdOpts("BHMST");
   printf("\n\n");
}

void SetuFlags(void)
{
   char *s;
   
   s=GetStrArg();
   uFlags=(UPDSet) 0;
   while (*s != '\0')
      switch (*s++) {
      case 't': uFlags = (UPDSet) (uFlags+UPTRANS); break;
      case 'm': uFlags = (UPDSet) (uFlags+UPMEANS); break;
      case 'v': uFlags = (UPDSet) (uFlags+UPVARS); break;
      case 'w': uFlags = (UPDSet) (uFlags+UPMIXES); break;
      default: HError(2420,"SetuFlags: Unknown update flag %c",*s);
         break;
      }
}

int main(int argc, char *argv[])
{
   Source src;
   int tmpInt;
   float tmpFlt;
   char *accfn, *s;

   void Initialise(char *hmmListFn);
   void Interpolate(void);
   void UpdateModels(void);
   void MakeWtAccLists(void);
   void AttachWtAccLists(void);
   void StatReport(void);
   
   if(InitShell(argc,argv,hsmooth_version,hsmooth_vc_id)<SUCCESS)
      HError(2400,"HSmooth: InitShell failed");

   InitMem();   InitLabel();
   InitMath();  InitSigP();
   InitWave();  InitAudio();
   InitVQ();    InitModel();
   if(InitParm()<SUCCESS)  
      HError(2400,"HSmooth: InitParm failed");

   InitTrain(); InitUtil();

   if (!InfoPrinted() && NumArgs() == 0)
      ReportUsage();
   if (NumArgs() == 0) Exit(0);

   SetConfParms();
   CreateHeap(&hmmStack,"HmmStore", MSTAK, 1, 1.0, 50000, 500000);
   CreateHMMSet(&hset,&hmmStack,TRUE);
   while (NextArg() == SWITCHARG) {
      s = GetSwtArg();
      if (strlen(s)!=1) 
         HError(2419,"HSmooth: Bad switch %s; must be single letter",s);
      switch(s[0]){
      case 'b':
         epsilon = GetChkedFlt(0.0,1.0,s); break;           
      case 'c':
         maxStep = GetChkedInt(1,1000,s); break;            
      case 'd':
         if (NextArg()!=STRINGARG)
            HError(2419,"HSmooth: HMM definition directory expected");
         hmmDir = GetStrArg(); break;  
      case 'e':
         if (NextArg()!=STRINGARG)
            HError(2419,"HSmooth: HMM definition directory expected");
         newDir = GetStrArg(); break;  
      case 'm':
         minEgs = GetChkedInt(1,1000,s); break;
      case 'o':
         if (NextArg()!=STRINGARG)
            HError(2419,"HSmooth: HMM file extension expected");
         newExt = GetStrArg(); break;
      case 's':
         stats = TRUE;
         if (NextArg()!=STRINGARG)
            HError(2419,"HSmooth: Stats file name expected");
         statFN = GetStrArg(); break;
      case 'u':
         SetuFlags(); break;
      case 'v':
         minVar = GetChkedFlt(0.0,10.0,s); break;
      case 'w':
         mixWeightFloor = MINMIX * GetChkedFlt(0.0,10000.0,s); 
         break;
      case 'x':
         if (NextArg()!=STRINGARG)
            HError(2419,"HSmooth: HMM file extension expected");
         hmmExt = GetStrArg(); break;
      case 'B':
         saveBinary=TRUE;
         break;
      case 'H':
         if (NextArg() != STRINGARG)
            HError(2419,"HSmooth: HMM macro file name expected");
         AddMMF(&hset,GetStrArg());
         break;
      case 'M':
         if (NextArg()!=STRINGARG)
            HError(2419,"HSmooth: Output macro file directory expected");
         newDir = GetStrArg();
         break;    
      case 'T':
         trace = GetChkedInt(0,0100000,s); break;
      default:
         HError(2419,"HSmooth: Unknown switch %s",s);
      }
   } 
   if (NextArg() != STRINGARG)
      HError(2419,"HSmooth: file name of HMM list expected");
   Initialise(GetStrArg());
   do {
      if (NextArg()!=STRINGARG)
         HError(2419,"HSmooth: accumulator file name expected");
      accfn = GetStrArg();
      src=LoadAccs(&hset,accfn,uFlags);
      ReadFloat(&src,&tmpFlt,1,ldBinary);
      totalPr += (LogDouble)tmpFlt;
      ReadInt(&src,&tmpInt,1,ldBinary);
      totalT += tmpInt;
      CloseSource(&src);      
      nBlk++;
      MakeWtAccLists();
   } while (NumArgs()>0);
   AttachWtAccLists();
   Interpolate();
   if (stats) StatReport();
   UpdateModels();
   Exit(0);
   return (0);          /* never reached -- make compiler happy */
}


/* ------------------------- Mix Weight Accs ---------------------- */

/* ChWtAcc - Chainable weight accumulator */
typedef struct _ChWtAcc *WALink;

typedef struct _ChWtAcc{     /* attached to StreamElem */
   Vector c;         /* array[1..M] of mixture weight */
   float occ;        /* occ for states sharing this pdf */
   long nInc;        /* num times this acc incremented */
   WALink next;      /* chain for wt accs */
} ChWtAcc;

static WALink ***wtStore;  /* array [1..nPhyHmms][2..nStates-1][1..nStreams]
                              of WALink */

/* CreateWtStore: Create store for linked lists of WALink, dimensions
   [1..nPhyHmms][2..nStates-1][1..nStreams] */
void CreateWtStore(MemHeap *x)
{
   int m,n,s;

   wtStore = (WALink ***) New(x,nPhyHmms*sizeof(WALink **));
   wtStore--;
   for (m=1;m<=nPhyHmms;m++){
      wtStore[m]=(WALink **) New(x,(maxStates-2)*sizeof(WALink *));
      wtStore[m]-=2;
      for (n=2;n<maxStates;n++){
         wtStore[m][n]=(WALink *)New(x,nStreams*sizeof(WALink));
         wtStore[m][n]--;
         for (s=1;s<=nStreams;s++)
            wtStore[m][n][s] = NULL;
      }
   }
}

/* CreateChWtAcc: create an accumulator for mixture weights */
WALink CreateChWtAcc(MemHeap *x, int M)
{
   WALink wa;
   
   wa = (WALink) New(x,sizeof(ChWtAcc));
   wa->c = CreateVector(x,M);
   ZeroVector(wa->c);
   wa->occ = 0.0; wa->nInc = 0;
   wa->next = NULL;
   return wa;
}

/* MakeWtAccLists: Copy info from WtAcc to WALink and add WALink to wtStore,
                   Zero WtAcc afterwards */
void MakeWtAccLists()
{
   int ix,n,s,i,nMix;
   HMMScanState hss;
   HLink hmm;
   WALink *w;
   StateElem *se;
   StreamElem *ste;
   WtAcc *wa;
   
   NewHMMScan(&hset,&hss);
   ix=1;
   do {
      hmm = hss.hmm;
      for (i=2,se = hmm->svec+2; i<hmm->numStates;i++,se++)
         for (s=1,ste = se->info->pdf+1; s<=nStreams; s++,ste++){
            w = &(wtStore[ix][i][s]); n = 0;
            while (*w != NULL){
               ++n; w = &((*w)->next);
            }
            nMix = (hset.hsKind==TIEDHS) ? hset.tmRecs[s].nMix : ste->nMix;
            (*w) = CreateChWtAcc(&wtAccStack, nMix);
            wa = (WtAcc *)ste->hook;
            CopyVector(wa->c,(*w)->c);
            (*w)->occ = wa->occ;
            wa->occ = 0;
            ZeroVector(wa->c);
         }
      ix++;
   } while (GoNextHMM(&hss));
   EndHMMScan(&hss);
}

/* AttachWtAccLists: Replace WtAccs in HMMSet with lists of WALink */
void AttachWtAccLists()
{
   int ix,s,i;
   HMMScanState hss;
   HLink hmm;
   StateElem *se;
   StreamElem *ste;

   NewHMMScan(&hset,&hss);
   ix=1;
   do {
      hmm = hss.hmm;
      for (i=2,se = hmm->svec+2; i<hmm->numStates;i++,se++)
         for (s=1,ste = se->info->pdf+1; s<=nStreams; s++,ste++){
            ste->hook = wtStore[ix][i][s];
            /* Note that this is known and tolerable memory leak */
         }
      ix++;
   } while (GoNextHMM(&hss));
   EndHMMScan(&hss);
}

/* -------------------------- Trace Support ----------------------- */

/* PrintLog: print a log value */
void PrintLog(LogDouble x)
{
   if (x<LSMALL)
      printf("       LZERO");
   else
      printf("%12.5f",x);
}

/* -------------------------- Initialisation ----------------------- */

void Initialise(char *hmmListFn)
{   
   /* Stacks for global structures requiring memory allocation */
   CreateHeap(&aSetStack,"ASetStore", MSTAK, 1, 0.0, 1000, 1000);
   CreateHeap(&sSetStack,"SSetStore", MSTAK, 1, 0.0, 1000, 1000);
   CreateHeap(&wsStack,"WsStore", MSTAK, 1, 0.0, 1000, 1000);
   CreateHeap(&labIdStack,"LabIdStore", MSTAK, 1, 0.0, 1000, 1000);
   CreateHeap(&wtAccStack,"WtAccStore", MSTAK, 1, 0.0, 1000, 1000);
   
   /* Load HMMs and init HMMSet related global variables */
   if(MakeHMMSet( &hset, hmmListFn )<SUCCESS)
      HError(2429,"Initialise: MakeHMMSet failed");
   if(LoadHMMSet( &hset,hmmDir,newExt)<SUCCESS)
      HError(2429,"Initialise: LoadHMMSet failed");
   AttachAccs(&hset, &gstack,uFlags);
   ZeroAccs(&hset,uFlags);   
   nPhyHmms = hset.numPhyHMM;
   nLogHmms = hset.numLogHMM;
   vSize = hset.vecSize;
   nStreams = hset.swidth[0];
   maxMixes = MaxMixInSet(&hset);
   maxStates = MaxStatesInSet(&hset);
   hsKind = hset.hsKind;

   /* Store for mix weights accs from each input acc file */
   CreateWtStore(&wtAccStack);

   if ((hsKind != TIEDHS) && (hsKind != DISCRETEHS))
      HError(2421,"Initialise: HMM's must be full Tied Mixture or Discrete");

   if (trace&T_TOP) {
      printf("HSmooth  Updating: ");
      if (uFlags&UPTRANS) printf("Transitions "); 
      if (uFlags&UPMEANS) printf("Means "); 
      if (uFlags&UPVARS)  printf("Variances "); 
      if (uFlags&UPMIXES && maxMixes>1)  printf("MixWeights "); 
      printf("\n ");
      printf("   Max Steps: %d, Epsilon : %f, Max Mixes: %d\n",
             maxStep,epsilon,maxMixes);
      printf("%d Logical/%d Physical Models Loaded, VecSize=%d\n",nLogHmms,nPhyHmms,vSize);
      fflush(stdout);
   }

   SetVFloor( &hset, vFloor, minVar);

   aSet=(HLink *)New(&aSetStack, nPhyHmms*sizeof(HLink));
   --aSet;

   sSet=(StreamElem **)New(&sSetStack, nPhyHmms*sizeof(StreamElem *));
   --sSet;   
}

/* ------------------- Statistics Reporting  -------------------- */

/* PrintStats: for given hmm */
void PrintStats(FILE *f, int n, HLink hmm, int numEgs)
{
   WtAcc *wa;
   char buf[MAXSTRLEN];
   StateInfo *si;
   int i,N;
    
   N = hmm->numStates;
   ReWriteString(HMMPhysName(&hset,hmm),buf,DBL_QUOTE);
   fprintf(f,"%4d %14s %4d ",n,buf,numEgs);
   for (i=2;i<N;i++) {
      si = hmm->svec[i].info;
      wa = (WtAcc *)((si->pdf+1)->hook);
      fprintf(f," %10f",wa->occ);
   }
   fprintf(f,"\n");
}

/* StatReport: print statistics report */
void StatReport(void)
{
   HMMScanState hss;
   HLink hmm;
   FILE *f;
   int px;

   if ((f = fopen(statFN,"w")) == NULL){
      HError(2311,"StatReport: Unable to open stats file %s",statFN);
      return;
   }
   NewHMMScan(&hset,&hss);
   px=1;
   do {
      hmm = hss.hmm;
      PrintStats(f,px,hmm,(int)hmm->hook);
      px++;
   } while (GoNextHMM(&hss));
   EndHMMScan(&hss);
   fclose(f);
}

/* ----------------- Deleted Interpolation -------------------- */

/* CreateWStore - create the global weight vectors */
void CreateWStore(void)
{
   long size;
   int i;
   
   size = (nBlk+1)*sizeof(Vector);
   wbar = (Vector *)New(&wsStack, size);
   wcd = (Vector *)New(&wsStack, size);
   for (i=0; i<=nBlk; i++){
      wbar[i] = CreateVector(&wsStack, maxMixes);
      wcd[i] = CreateVector(&wsStack, maxMixes);
   }
}

/* CreateMonoList: set nPhones and create list of monophones */
void CreateMonoList(void)
{
   int i,j;
   Boolean found;
   LabId list[MAXMONOPHONES], id;
   char buf[255];
   MLink q;

   nPhones = 0;
   for (i=0; i<MACHASHSIZE; i++)
      for (q=hset.mtab[i]; q!=NULL; q=q->next)
         if (q->type=='l'){      
            strcpy(buf,q->id->name);
            TriStrip(buf);
            id = GetLabId(buf,TRUE);
            found = FALSE;
            for (j=0; j<nPhones; j++)
               if (list[j] == id) {
                  found = TRUE; break;
               }
            if (!found){
               if (nPhones>=MAXMONOPHONES)
                  HError(2422,"CreateMonoList: Too many monophones");
               list[nPhones++] = id;
            }
         }
   
   monophones = (LabId *)New(&labIdStack, nPhones*sizeof(LabId));
   for (i=0; i<nPhones; i++)
      monophones[i] = list[i];
   --monophones;
}

/* LoadASet: set aSize, load the allophone set of x and return the
             number of states in each model (all allophones must have
             the same number of states) */
int LoadASet(LabId x)
{
   int i,N=0;
   HLink hmm;
   MLink q;
   LabId id;
   char *aid,buf[255];
   
   aSize = 0;
   for (i=0; i<MACHASHSIZE; i++)
      for (q=hset.mtab[i]; q!=NULL; q=q->next)
         if (q->type=='l'){       
            aid = q->id->name;
            strcpy(buf,aid);
            TriStrip(buf);
            id = GetLabId(buf,FALSE);
            if (id==x){
               if (trace&T_OPT)
                  printf("    loading allophone %s\n",aid);
               hmm = (HLink)q->structure;
               if (hmm->numStates>0) {
                  if (N==0) 
                     N = hmm->numStates;
                  else
                     if (N != hmm->numStates)
                        HError(2423,"LoadASet: allophones must have same num states %d vs %d",
                               N, hmm->numStates);
                  hmm->numStates = -N;
                  aSet[++aSize] = hmm;
               }
            }
         }
   for (i=1; i<=aSize; i++)
      aSet[i]->numStates = N;
   return N;
}

/* LoadSSet: load sSet with pointers to state i, stream s of each 
             allophone in the aSet array */
void LoadSSet(int i, int s)
{
   int k;
   
   for (k=1; k<=aSize; k++)
      sSet[k] = aSet[k]->svec[i].info->pdf+s;
}

/* SumWtChain: sum chain of accs whose head is wa excluding dBlk, put
               result in v and return occ sum */
float SumWtChain(WALink wa, Vector v, int dBlk, int M)
{
   float occ = 0.0;
   Vector x;
   int blk=0,i;
   
   while (wa != NULL) {
      ++blk;
      if (blk != dBlk) {
         occ += wa->occ;
         x = wa->c;
         for (i=1; i<=M; i++)
            v[i] += x[i];
      }
      wa = wa->next;
   }
   return occ;
}

/* CalcWBar: store context independent weights in wb from all blocks
             across all models excluding deleted block dblk */
void CalcWBar(Vector wb, int dBlk, int M)
{
   int i;
   WALink wa;
   float occ = 0.0;
   
   ZeroVector(wb);
   for (i=1; i<=aSize; i++){
      wa = (WALink)sSet[i]->hook;
      occ += SumWtChain(wa,wb,dBlk,M);
   }
   if (occ==0.0)
      ZeroVector(wb);
   else
      for (i=1; i<=M; i++) wb[i] /= occ;
}

/* CalcWCd: store context dependent weights in wc using all blocks
            of stream ste except the deleted block dBlk */
void CalcWCd(Vector wc, int dBlk, StreamElem *ste, int M)
{
   WALink wa;
   float occ;
   int i;
   
   ZeroVector(wc);
   wa = (WALink)ste->hook;
   occ = SumWtChain(wa,wc,dBlk,M);
   if (occ==0.0)
      ZeroVector(wc);
   else
      for (i=1; i<=M; i++) wc[i] /= occ;
}

/* SmoothWtAcc: change 1st WtAcc to l*wcd[0] + (1.0-l)*wbar[0] */
void SmoothWtAcc(StreamElem *ste, float l, int M)
{
   int i;
   WALink wa;
   
   wa = (WALink)ste->hook;
   for (i=1; i<=M; i++)
      wa->c[i] = l*wcd[0][i] + (1.0-l)*wbar[0][i];
   wa->occ = 1.0;
}

/* D: return derivative of log likelihood */
float D(float l, WALink wa, int M)
{
   int n,i;
   double sum = 0.0;
   double wni, wbni,y;
   Vector wn, wbn, x;
   
   for (n=1; n<=nBlk; n++){
      x = wa->c;
      wn = wcd[n];
      wbn = wbar[n];
      for (i=1; i<=M; i++){
         wni = wn[i];
         wbni = wbn[i];
         y = l*wni + (1.0-l)*wbni;
         if (y>0.0)
            sum += x[i] * ((wni - wbni) / y);
      }
      wa = wa->next;
   }
   return sum;
}

/* LambdaOpt: perform binary chop optimisation */
float LambdaOpt(StreamElem *ste, int M)
{
   float l=0.0, r=1.0, m=0.5, Dm;
   WALink wa;
   int n;

   wa = (WALink)ste->hook;
   if (D(0.0,wa,M) <= 0.0) return 0.0;
   if (D(1.0,wa,M) >= 0.0) return 1.0;
   for (n = 1; n<=maxStep; n++) {
      m = (l+r)/2.0;
      Dm = D(m,wa,M);
      if (trace&T_OPT)
         printf("   step %d: l=%.3f r=%.3f m=%.3f D(m)=%e\n",n,l,r,m,Dm);
      if (fabs(Dm) < epsilon) return m;
      if (Dm > 0 ) l = m; else r = m;
   }
   return m;
}

/* Interpolate: top level of deleted interpolation */
void Interpolate(void)
{
   LabId x;
   int i,N,p,s,b,j,M=0;
   float l;
   StreamElem *ste;
   
   CreateWStore();
   CreateMonoList();
   for (p=1; p<=nPhones; p++){
      x = monophones[p];
      if (trace&T_INT)
         printf("Smoothing phone %s [%d]\n",x->name,p);
      N = LoadASet(x);
      for (i=2; i<N; i++) {
         if (trace&T_INT)
            printf(" State %d\n",i);
         for (s=1; s<=nStreams; s++){
            if (trace&T_INT)
               printf("  Stream %d\n",s);
            LoadSSet(i,s);
            switch(hsKind){
            case TIEDHS:
               M = hset.tmRecs[s].nMix;
               break;
            case DISCRETEHS:
               M = sSet[1]->nMix;
               break;
            }
            CalcWBar(wbar[0],0,M);
            for (b=1; b<=nBlk; b++)
               CalcWBar(wbar[b],b,M);
            for (j=1; j<=aSize; j++){
               ste = sSet[j];
               CalcWCd(wcd[0],0,ste,M);
               for (b=1; b<=nBlk; b++)
                  CalcWCd(wcd[b],b,ste,M);
               l = LambdaOpt(ste,M);
               SmoothWtAcc(ste,l,M);
               if (trace&T_INT)
                  printf("   Model %s lambda = %f\n",HMMPhysName(&hset,aSet[j]),l);
            }
         }
      }
   }
}

               
      
/* --------------------------- Model Update --------------------- */

/* UpdateTrans: use acc values to calc new estimate for transP */
void UpdateTrans(HLink hmm)
{
   int i,j,N;
   float x,occi;
   TrAcc *ta;
   MLink q;

   q=FindMacroStruct(&hset,'h',hmm);
   ta = (TrAcc *) GetHook(hmm->transP);
   if (ta==NULL) return;   /* already done */
   N = hmm->numStates;
   for (i=1;i<N;i++) {
      occi = ta->occ[i];
      if (occi > 0.0) 
         for (j=2;j<=N;j++) {
            x = ta->tran[i][j]/occi;
            hmm->transP[i][j] = (x>MINLARG)?log(x):LZERO;
         }
      else
         HError(-2424,"UpdateTrans: Model [%s]: no transitions out of state %d",q->id->name,i);
   }
   SetHook(hmm->transP,NULL);
}

/* FloorMixes: apply floor to given mix set */
void FloorTMMixes(Vector mixes, int M,float floor)
{
   float sum,fsum,scale,fltWt;
   int m;
   
   sum = fsum = 0.0;
   for (m=1; m<=M; m++) {
      fltWt = mixes[m];
      if (fltWt>floor)
         sum += fltWt;
      else {
         fsum += floor;
         mixes[m] = floor;
      }
   }
   if (fsum>1.0)
      HError(2425,"FloorMixes: Floor sum too large");
   if (fsum == 0.0) return;
   if (sum == 0.0) HError(2428,"FloorTMMixes: No mixture weights above floor");
   scale = (1.0-fsum)/sum;
   for (m=1; m<=M; m++){
      fltWt = mixes[m];
      if (fltWt>floor)
         mixes[m] = fltWt*scale;
   }
}

/* FloorDProbs: apply floor to given discrete prob set */
void FloorDProbs(ShortVec mixes, int M, float floor)
{
   float sum,fsum,scale,fltWt;
   int m;
   
   sum = fsum = 0.0;
   for (m=1; m<=M; m++) {
      fltWt = Short2DProb(mixes[m]);
      if (fltWt>floor)
         sum += fltWt;
      else {
         fsum += floor;
         mixes[m] = DProb2Short(floor);
      }
   }
   if (fsum>1.0) HError(2327,"FloorDProbs: Floor sum too large");
   if (fsum == 0.0) return;
   if (sum == 0.0) HError(2328,"FloorDProbs: No probabilities above floor");
   scale = (1.0-fsum)/sum;
   for (m=1; m<=M; m++){
      fltWt = Short2DProb(mixes[m]);
      if (fltWt>floor)
         mixes[m] = DProb2Short(fltWt*scale);
   }
}

/* UpdateWeights: use acc values to calc new estimate of mix weights */
void UpdateWeights(HLink hmm)
{
   int i,s,m,M=0,N;
   float x,occi;
   WALink wa;
   StateElem *se;
   StreamElem *ste;
   MLink q;

   q=FindMacroStruct(&hset,'h',hmm);
   N = hmm->numStates;
   se = hmm->svec+2;
   for (i=2; i<N; i++,se++){
      ste = se->info->pdf+1;
      for (s=1;s<=nStreams; s++,ste++){
         wa = (WALink)ste->hook;
         if (wa != NULL) {
            switch(hsKind){
            case TIEDHS:
               M = hset.tmRecs[s].nMix;
               break;
            case DISCRETEHS:
               M = ste->nMix;
               break;
            }
            occi = wa->occ;
            if (occi>0) {
               for (m=1; m<=M; m++){
                  x = wa->c[m]/occi;
                  if (x>1.0){
                     if (x>1.001)
                        HError(-2490,"UpdateWeights: Model [%s]: mix too big in %d.%d.%d",q->id->name,i,s,m);
                     x = 1.0;
                  }
                  switch (hsKind){
                  case TIEDHS:
                     ste->spdf.tpdf[m] = (x>MINMIX) ? x : 0;
                     break;
                  case DISCRETEHS:
                     ste->spdf.dpdf[m] = (x>MINMIX) ? DProb2Short(x) :DLOGZERO;
                     break;
                  }
               }
               if (mixWeightFloor>0.0){
                  switch (hsKind){
                  case DISCRETEHS:
                     FloorDProbs(ste->spdf.dpdf,M,mixWeightFloor);
                     break;
                  case TIEDHS:
                     FloorTMMixes(ste->spdf.tpdf,M,mixWeightFloor);
                     break;
                  }
               }
            }else
               HError(-2427,"UpdateWeights: Model [%s]: no use of mixtures in %d.%d",q->id->name,i,s);
            ste->hook = NULL;
         }
      }
   }
}
      
/* UpdateTMMeans: use acc values to calc new estimate of means */
void UpdateTMMeans(void)
{
   int s,m,k,M,vSize;
   float occim;
   MuAcc *ma;
   MixPDF *mpdf;
   Vector mean;

   for (s=1;s<=nStreams;s++){
      vSize = hset.swidth[s];
      M = hset.tmRecs[s].nMix;
      for (m=1;m<=M;m++){
         mpdf = hset.tmRecs[s].mixes[m];
         mean = mpdf->mean;
         ma = (MuAcc *) GetHook(mean);
         if (ma != NULL){
            occim = ma->occ;
            if (occim > 0.0)
               for (k=1; k<=vSize; k++)
                  mean[k] += ma->mu[k]/occim;
            else
               HError(-2427,"UpdateTMMeans: No use of mean %d in stream %d",m,s);
            SetHook(mean,NULL);
         }
      }
   }
}

/* UpdateTMVars: use acc values to calc new estimate of variances */
void UpdateTMVars(void)
{
   int s,m,k,l,M,vSize;
   float occim,x,muDiffk,muDiffl;
   Vector minV;
   VaAcc *va;
   MuAcc *ma;
   MixPDF *mpdf;
   Vector mean;
   Covariance cov;
   Boolean mixFloored,shared;

   for (s=1;s<=nStreams;s++){
      vSize = hset.swidth[s];
      minV = vFloor[s];
      M = hset.tmRecs[s].nMix;
      for (m=1;m<=M;m++){
         mpdf = hset.tmRecs[s].mixes[m];
         cov = mpdf->cov;
         va = (VaAcc *) GetHook(cov.var);
         mean = mpdf->mean;
         ma = (MuAcc *) GetHook(mean);     
         if (va != NULL){
            occim = va->occ;
            mixFloored = FALSE;
            if (occim > 0.0){
               shared=(GetUse(cov.var)>1 || ma==NULL || ma->occ<=0.0);
               if ((mpdf->ckind==DIAGC)||(mpdf->ckind==INVDIAGC))
                  for (k=1; k<=vSize; k++){
                     muDiffk=(shared)?0.0:ma->mu[k]/ma->occ;
                     x = va->cov.var[k]/occim - muDiffk*muDiffk;
                     if (x<minV[k]) {
                        x = minV[k];
                        mixFloored = TRUE;
                     }
                     cov.var[k] = x;
                  }
               else { /* FULLC */
                  for (k=1; k<=vSize; k++){
                     muDiffk=(shared)?0.0:ma->mu[k]/ma->occ;
                     for (l=1; l<=k; l++){
                        muDiffl=(shared)?0.0:ma->mu[l]/ma->occ;
                        x = va->cov.inv[k][l]/occim - muDiffk*muDiffl;
                        if (k==l && x<minV[k]){
                           x = minV[k];
                           mixFloored = TRUE;
                        }              
                        cov.inv[k][l] = x;
                     }
                  }
                  CovInvert(cov.inv,cov.inv);
               }
            }
            else
               HError(-2427,"UpdateTMVars: No use of var %d in stream %d",m,s);
            SetHook(cov.var,NULL);
         }
      }
   }
}

/* ResetHeaps: Reset all heaps used for dynamic memory allocation */
void ResetHeaps(void)
{
   ResetHeap(&hmmStack);
   ResetHeap(&aSetStack);
   ResetHeap(&sSetStack);
   ResetHeap(&wsStack);
   ResetHeap(&labIdStack);
   ResetHeap(&wtAccStack);
   ResetHeap(&gstack);
}

/* UpdateModels: update all models and save them in newDir if set,
   new files have newExt if set */
void UpdateModels(void)
{
   int n;
   HLink hmm;
   HMMScanState hss;
   
   if (trace&T_INT){
      printf("Starting Model Update\n"); fflush(stdout);
   }

   if (hsKind==TIEDHS){
      if (uFlags & UPVARS)  /* TIEDHS therefore only done once per HMMSet */
         UpdateTMVars();
      if (uFlags & UPMEANS) 
         UpdateTMMeans();      
      if (uFlags & (UPMEANS|UPVARS))
         FixAllGConsts(&hset);
   }

   NewHMMScan(&hset,&hss);
   do {
      hmm = hss.hmm;   
      n = (int)hmm->hook;
      if (n<minEgs && !(trace&T_OPT))
         HError(-2428,"%s copied: only %d egs\n",HMMPhysName(&hset,hmm),n);
      if (n>=minEgs) {
         if (uFlags & UPTRANS)
            UpdateTrans(hmm);
         if (maxMixes>1 && uFlags & UPMIXES)
            UpdateWeights(hmm);
      }
      if (trace&T_OPT) {
         if (n<minEgs)
            printf("Model %s copied: only %d examples\n",
                   HMMPhysName(&hset,hmm),n);
         else
            printf("Model %s updated with %d examples\n",
                   HMMPhysName(&hset,hmm),n);
         fflush(stdout);
      }
   } while (GoNextHMM(&hss));
   EndHMMScan(&hss);
   if (trace&T_TOP){
      printf("Saving hmm's to dir %s\n",(newDir==NULL)?"Current":newDir); 
      fflush(stdout);
   }

   if(SaveHMMSet(&hset,newDir,newExt,NULL,saveBinary)<SUCCESS)
      HError(2411,"UpdateModels: SaveHMMSet failed");
   ResetHeaps();                               /* Clean Up */
   if (trace&T_TOP)
      printf("Reestimation complete - average log prob per frame = %e\n",
             totalPr/(double)totalT);
}

/* ----------------------------------------------------------- */
/*                     END:  HSmooth.c                         */
/* ----------------------------------------------------------- */
