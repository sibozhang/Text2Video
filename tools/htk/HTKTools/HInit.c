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
/*              2002  Cambridge University                     */
/*                    Engineering Department                   */
/*                                                             */
/*   Use of this software is governed by a License Agreement   */
/*    ** See the file License for the Conditions of Use  **    */
/*    **     This banner notice must not be removed      **    */
/*                                                             */
/* ----------------------------------------------------------- */
/*         File: HInit.c: HMM initialisation program           */
/* ----------------------------------------------------------- */

char *hinit_version = "!HVER!HInit:   3.4.1 [CUED 12/03/09]";
char *hinit_vc_id = "$Id: HInit.c,v 1.1.1.1 2006/10/11 09:55:01 jal58 Exp $";

/*
   This program is used to initialise (or tune) a single hidden
   Markov model using a segmental K-means algorithm.  Only
   means, variances, mixture weights and transition probs can be
   estimated, stream weights are left unchanged.
*/

/* Trace Flags */
   
#define T_TOP 0001      /* Top level tracing */
#define T_LD0 0002      /* File Loading */
#define T_LD1 0004      /* + segments within each file */
#define T_UNI 0010      /* Uniform segmentation */
#define T_VIT 0020      /* Detailed Viterbi Alignment */
#define T_ALN 0040      /* State Alignment */
#define T_MIX 0100      /* Mixture Component Alignment */
#define T_CNT 0200      /* Count Updating */
#define T_OBP 0400      /* Trace output probs */

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
#include "HUtil.h"
#include "HTrain.h"

/* Global Settings */
static char * segLab = NULL;        /* segment label if any */
static LabId  segId  = NULL;        /* and its id */
static char * labDir = NULL;        /* label file directory */
static char * labExt = "lab";       /* label file extension */
static int  maxIter  = 20;          /* max iterations in param estimation */
static float epsilon = 1.0E-4;      /* convergence criterion */
static float minVar  = 1.0E-2;      /* minimum variance */
static float mixWeightFloor=0.0;    /*Floor for mixture/discrete prob weights*/
static int minSeg    = 3;           /* min segments to train a model */
static Boolean  newModel = TRUE;    /* enable initial uniform segmentation */
static Boolean saveBinary = FALSE;  /* save output in binary  */
static Boolean firstTime = TRUE;    /* Flag used to enable InitSegStore */
static FileFormat dff=UNDEFF;       /* data file format */
static FileFormat lff=UNDEFF;       /* label file format */
static char *hmmfn;                 /* HMM definition file name (& part dir)*/
static char *outfn=NULL;            /* output HMM file name (name only) */
static char *outDir=NULL;           /* HMM output directory */
static UPDSet uFlags = (UPDSet) (UPMEANS|UPVARS|UPMIXES|UPTRANS); /* update flags */
static int trace = 0;               /* Trace level */
static ConfParam *cParm[MAXGLOBS];   /* configuration parameters */
static int nParm = 0;               /* total num params */
static Vector vFloor[SMAX];         /* variance floor - default is all zero */

/* Major Data Structures plus related global vars*/
static HMMSet hset;              /* The current unitary hmm set */
static MLink macroLink;          /* Access to macro in HMMSet */
static HLink hmmLink;            /* link to the hmm itself */
static int maxMixInS[SMAX];      /* array[1..swidth[0]] of max mixes */
static int nStates;              /* number of states in hmm */
static int nStreams;             /* number of streams in hmm */
static SegStore segStore;        /* Storage for data segments */
static MemHeap segmentStack;     /* Used by segStore */
static MemHeap sequenceStack;    /* For storage of sequences */
static MemHeap clustSetStack;    /* For storage of cluster sets */
static MemHeap transStack;       /* For storage of transcription */
static MemHeap traceBackStack;   /* For storage of traceBack info */
static MemHeap bufferStack;      /* For storage of buffer */
static ParmBuf pbuf;             /* Currently input parm buffer */

/* Storage for Viterbi Decoding */
static Vector   thisP,lastP;     /* Columns of log probabilities */
static short   **traceBack;      /* array[1..segLen][2..numStates-1] */
   

/* ---------------- Process Conf File & Command Line ----------------- */

/* SetConfParms: set conf parms relevant to HInit  */
void SetConfParms(void)
{
   int i;

   nParm = GetConfig("HINIT", TRUE, cParm, MAXGLOBS);
   if (nParm>0) {
      if (GetConfInt(cParm,nParm,"TRACE",&i)) trace = i;
   }
}

void ReportUsage(void)
{
   printf("\nUSAGE: HInit [options] hmmFile trainFiles...\n\n");
   printf(" Option                                       Default\n\n");
   printf(" -e f    Set convergence factor epsilon       1.0E-4\n");
   printf(" -i N    Set max iterations to N              20\n");
   printf(" -l s    Set segment label to s               none\n");
   printf(" -m N    Set min segments needed              3\n");
   printf(" -n      Update hmm (suppress uniform seg)    off\n");
   printf(" -o fn   Store new hmm def in fn (name only)  outDir/srcfn\n");
   printf(" -u mvwt Update m)eans v)ars w)ghts t)rans    mvwt\n");
   printf(" -v f    Set minimum variance to f            1.0E-2\n");
   printf(" -w f    set mix wt/disc prob floor to f      0.0\n");
   PrintStdOpts("BFGHILMX");
   printf("\n\n");
}


/* ------------------------- Set Update Flags ------------------------- */

void SetuFlags(void)
{
   char *s;
   
   s=GetStrArg();
   uFlags=(UPDSet) 0;      
   while (*s != '\0')
      switch (*s++) {
      case 'm': uFlags = (UPDSet) (uFlags+UPMEANS); break;
      case 'v': uFlags = (UPDSet) (uFlags+UPVARS); break;
      case 'w': uFlags = (UPDSet) (uFlags+UPMIXES); break;
      case 't': uFlags = (UPDSet) (uFlags+UPTRANS); break;
      default: HError(2120,"SetuFlags: Unknown update flag %c",*s);
         break;
      }
}

int main(int argc, char *argv[])
{
   char *datafn, *s;
   int nSeg;
   void Initialise(void);
   void LoadFile(char *fn);
   void EstimateModel(void);
   void SaveModel(char *outfn);
   
   if(InitShell(argc,argv,hinit_version,hinit_vc_id)<SUCCESS)
      HError(2100,"HInit: InitShell failed");
   InitMem();   InitLabel();
   InitMath();  InitSigP();
   InitWave();  InitAudio();
   InitVQ();    InitModel();
   if(InitParm()<SUCCESS)  
      HError(2100,"HInit: InitParm failed");
   InitTrain(); InitUtil();

   if (!InfoPrinted() && NumArgs() == 0)
      ReportUsage();
   if (NumArgs() == 0) Exit(0);
   SetConfParms();

   CreateHMMSet(&hset,&gstack,FALSE);
   while (NextArg() == SWITCHARG) {
      s = GetSwtArg();
      if (strlen(s)!=1) 
         HError(2119,"HInit: Bad switch %s; must be single letter",s);
      switch(s[0]){
      case 'e':
         epsilon = GetChkedFlt(0.0,1.0,s); break;
      case 'i':
         maxIter = GetChkedInt(0,100,s); break;
      case 'l':
         if (NextArg() != STRINGARG)
            HError(2119,"HInit: Segment label expected");
         segLab = GetStrArg();
         break;
      case 'm':
         minSeg = GetChkedInt(1,1000,s); break;
      case 'n':
         newModel = FALSE; break;
      case 'o':
         outfn = GetStrArg();
         break;
      case 'u':
         SetuFlags(); break;
      case 'v':
         minVar = GetChkedFlt(0.0,10.0,s); break;
      case 'w':
         mixWeightFloor = MINMIX * GetChkedFlt(0.0,10000.0,s); 
         break;
      case 'B':
         saveBinary = TRUE;
         break;
      case 'F':
         if (NextArg() != STRINGARG)
            HError(2119,"HInit: Data File format expected");
         if((dff = Str2Format(GetStrArg())) == ALIEN)
            HError(-2189,"HInit: Warning ALIEN Data file format set");
         break;
      case 'G':
         if (NextArg() != STRINGARG)
            HError(2119,"HInit: Label File format expected");
         if((lff = Str2Format(GetStrArg())) == ALIEN)
            HError(-2189,"HInit: Warning ALIEN Label file format set");
         break;
      case 'H':
         if (NextArg() != STRINGARG)
            HError(2119,"HInit: HMM macro file name expected");
         AddMMF(&hset,GetStrArg());
         break;
      case 'I':
         if (NextArg() != STRINGARG)
            HError(2119,"HInit: MLF file name expected");
         LoadMasterFile(GetStrArg());
         break;
      case 'L':
         if (NextArg()!=STRINGARG)
            HError(2119,"HInit: Label file directory expected");
         labDir = GetStrArg(); break;
      case 'M':
         if (NextArg()!=STRINGARG)
            HError(2119,"HInit: Output macro file directory expected");
         outDir = GetStrArg();
         break;
      case 'T':
         if (NextArg() != INTARG)
            HError(2119,"HInit: Trace value expected");
         trace = GetChkedInt(0,01777,s);
         break;
      case 'X':
         if (NextArg()!=STRINGARG)
            HError(2119,"HInit: Label file extension expected");
         labExt = GetStrArg(); break;
      default:
         HError(2119,"HInit: Unknown switch %s",s);
      }
   }
   if (NextArg()!=STRINGARG)
      HError(2119,"HInit: source HMM file name expected");
   hmmfn = GetStrArg();
   Initialise();
   do {
      if (NextArg()!=STRINGARG)
         HError(2119,"HInit: training data file name expected");
      datafn = GetStrArg();
      LoadFile(datafn);
   } while (NumArgs()>0);
   nSeg = NumSegs(segStore);
   if (nSeg < minSeg)
      HError(2121,"HInit: Too Few Observation Sequences [%d]",nSeg);
   if (trace&T_TOP) {
      printf("%d Observation Sequences Loaded\n",nSeg);
      fflush(stdout);
   }
   EstimateModel();
   SaveModel(outfn);
   if (trace&T_TOP)
      printf("Output written to directory %s\n",
             outDir==NULL?"current":outDir);
   Exit(0);
   return (0);          /* never reached -- make compiler happy */
}

/* ------------------------ Initialisation ----------------------- */

/* PrintInitialInfo: print a header of program settings */
void PrintInitialInfo(void)
{     
   if (newModel) printf("Initialising  "); else printf("Updating  ");
   printf("HMM %s . . . \n",hmmfn);
   PrintHMMProfile(stdout, hmmLink);
   printf(" SegLab   :  %s\n",(segLab==NULL)?"NONE":segLab);
   printf(" maxIter  :  %d\n",maxIter);
   printf(" epsilon  :  %f\n",epsilon);
   printf(" minSeg   :  %d\n",minSeg);
   printf(" Updating :  ");
   if (uFlags&UPMEANS) printf("Means ");
   if (uFlags&UPVARS)  printf("Variances "); 
   if (uFlags&UPMIXES) printf("MixWeights/DProbs ");
   if (uFlags&UPTRANS) printf("TransProbs");
   printf("\n\n");
   printf(" - system is ");
   switch (hset.hsKind){
   case PLAINHS:  printf("PLAIN\n");  break;
   case SHAREDHS: printf("SHARED\n"); break;
   case TIEDHS:   printf("TIED\n");   break;
   case DISCRETEHS: printf("DISCRETE\n"); break;
   }
   fflush(stdout);
}

/* Initialise: load hmm and initialise global data structures */
void Initialise(void)
{
   LabId  hmmId;
   char base[MAXSTRLEN];
   char path[MAXSTRLEN];
   char ext[MAXSTRLEN]; 
   int s;  

   /* Stacks for global structures requiring memory allocation */
   CreateHeap(&segmentStack,"SegStore", MSTAK, 1, 0.0, 100000, LONG_MAX);
   CreateHeap(&sequenceStack,"SeqStore", MSTAK, 1, 0.0, 1000, 1000);
   CreateHeap(&clustSetStack,"ClustSetStore", MSTAK, 1, 0.0, 1000, 1000);
   CreateHeap(&transStack,"TransStore", MSTAK, 1, 0.0, 1000, 1000);
   CreateHeap(&traceBackStack,"TraceBackStore", MSTAK, 1, 0.0, 1000, 1000);
   CreateHeap(&bufferStack,"BufferStore", MSTAK, 1, 0.0, 1000, 1000);

   /* Load HMM def */
   if(MakeOneHMM( &hset, BaseOf(hmmfn,base))<SUCCESS)
      HError(2128,"Initialise: MakeOneHMM failed");
   if(LoadHMMSet( &hset,PathOf(hmmfn,path),ExtnOf(hmmfn,ext))<SUCCESS)
      HError(2128,"Initialise: LoadHMMSet failed");
   SetParmHMMSet(&hset);
   if ((hset.hsKind==DISCRETEHS)||(hset.hsKind==TIEDHS))
      uFlags = (UPDSet) (uFlags & (~(UPMEANS|UPVARS)));
   AttachAccs(&hset, &gstack, uFlags);

   /* Get a pointer to the physical HMM and set related globals */
   hmmId = GetLabId(base,FALSE);   
   macroLink = FindMacroName(&hset,'h',hmmId);
   hmmLink = (HLink)macroLink->structure; 
   nStates = hmmLink->numStates;
   nStreams = hset.swidth[0];
   for(s=1; s<=nStreams; s++)
      maxMixInS[s] = MaxMixInS(hmmLink, s);

   SetVFloor( &hset, vFloor, minVar);

   if(segLab != NULL)
      segId = GetLabId(segLab,TRUE);
   if(trace>0)
      PrintInitialInfo();

   thisP = CreateVector(&gstack,nStates);
   lastP = CreateVector(&gstack,nStates);
}

/* InitSegStore : Initialise segStore for particular observation */
void InitSegStore(BufferInfo *info)
{
   Observation obs;
   Boolean eSep;

   SetStreamWidths(info->tgtPK,info->tgtVecSize,hset.swidth,&eSep);
   obs = MakeObservation(&gstack,hset.swidth,info->tgtPK,
                         hset.hsKind==DISCRETEHS,eSep);
   segStore = CreateSegStore(&segmentStack,obs,10);
   firstTime = FALSE;
}


/* ------------------------ Data Loading ----------------------- */

/* CheckData: check data file consistent with HMM definition */
void CheckData(char *fn, BufferInfo info) 
{

   char tpk[80];
   char mpk[80];
   
   if (info.tgtPK != hset.pkind)
      HError(2150,"CheckData: Parameterisation in %s[%s] is incompatible with hmm %s[%s]",
             fn,ParmKind2Str(info.tgtPK,tpk),hmmfn,ParmKind2Str(hset.pkind,mpk));
   if (info.tgtVecSize!=hset.vecSize)
      HError(2150,"CheckData: Vector size in %s[%d] is incompatible with hmm %s[%d]",
             fn,info.tgtVecSize,hmmfn,hset.vecSize);
}

/* LoadFile: load whole file or segments into segStore */
void LoadFile(char *fn)
{
   BufferInfo info;
   char labfn[80];
   Transcription *trans;
   long segStIdx,segEnIdx;
   static int segIdx=1;  /* Between call handle on latest seg in segStore */  
   static int prevSegIdx=1;
   HTime tStart, tEnd;
   int i,k,s,ncas,nObs=0,segLen;
   LLink p;
   Observation obs;

   if((pbuf=OpenBuffer(&bufferStack, fn, 10, dff, FALSE_dup, FALSE_dup))==NULL)
      HError(2150,"LoadFile: Config parameters invalid");
   GetBufferInfo(pbuf,&info);
   CheckData(fn,info);
   if (firstTime) InitSegStore(&info);

   if (segId == NULL)  {   /* load whole parameter file */
      nObs = ObsInBuffer(pbuf);
      tStart = 0.0;
      tEnd = (info.tgtSampRate * nObs);
      LoadSegment(segStore, tStart, tEnd, pbuf);
      segIdx++;
   }
   else {                  /* load segment of parameter file */
      MakeFN(fn,labDir,labExt,labfn);
      trans = LOpen(&transStack,labfn,lff);
      ncas = NumCases(trans->head,segId);
      if ( ncas > 0) {
         for (i=1,nObs=0; i<=ncas; i++) {
            p = GetCase(trans->head,segId,i);
            segStIdx = (long)(p->start/info.tgtSampRate);
            segEnIdx = (long)(p->end/info.tgtSampRate);
            if (segEnIdx >= ObsInBuffer(pbuf))
               segEnIdx = ObsInBuffer(pbuf)-1;
            if (segEnIdx - segStIdx + 1 >= nStates-2) {
               LoadSegment(segStore, p->start, p->end, pbuf);
               if (trace&T_LD1)
                  printf("  loading seg %s %f[%ld]->%f[%ld]\n",segId->name,
                         p->start,segStIdx,p->end,segEnIdx);
               nObs += SegLength(segStore, segIdx);
               segIdx++;
            }else if (trace&T_LD1)
               printf("   seg %s %f->%f ignored\n",segId->name,
                      p->start,p->end);
         }        
      }  
   }
   if (hset.hsKind == DISCRETEHS){
      for (k=prevSegIdx; k<segIdx; k++){
         segLen = SegLength(segStore, k);
         for (i=1; i<=segLen; i++){
            obs = GetSegObs(segStore, k, i);
            for (s=1; s<=nStreams; s++){
               if( (obs.vq[s] < 1) || (obs.vq[s] > maxMixInS[s]))
                  HError(2150,"LoadFile: Discrete data value [ %d ] out of range in stream [ %d ] in file %s",obs.vq[s],s,fn);
            }
         }
      }
      prevSegIdx=segIdx;
   }

   if (trace&T_LD0)
      printf(" %d observations loaded from %s\n",nObs,fn);
   CloseBuffer(pbuf);
   ResetHeap(&transStack);
}

/* ---------- Initialise with Uniform Segmentation ------------------- */


/* CreateSeqMat: Create a matrix of sequences */
Sequence ** CreateSeqMat(void)
{
   int i,j;
   Sequence **seqMat;

   seqMat = (Sequence**)New(&sequenceStack,(nStates-2)*sizeof(Sequence*));
   seqMat -= 2;   /* index is 2, ...,nStates-1 */
   for (i=2; i<nStates; i++){
      seqMat[i] = (Sequence*)New(&gstack, nStreams*sizeof(Sequence));
      --seqMat[i];
      for (j=1; j<=nStreams; j++ ){
         seqMat[i][j] = CreateSequence(&sequenceStack, 100);
      }
   }
   return seqMat;
}

/* ShowSeqMat: show number of obs for each state/stream */
void ShowSeqMat(Sequence **seqMat)
{
   int j,s;
   
   printf("Sequence Matrix\n");
   for (j=2; j<nStates; j++) {
      printf(" state %2d: ",j);
      for (s=1; s<=nStreams; s++)
         printf("%5d",seqMat[j][s]->nItems);
      printf("\n");
   }
}

/* UCollectData: Collect data from segStore for each stream s of each 
   state n and store in seqMat[n][s]*/
void UCollectData(Sequence **seqMat)
{
   int i,j,n,s,numSegs,segLen;
   float obsPerState;
   Observation obs;
   Ptr p;
   
   numSegs = NumSegs(segStore);
   for (i=1;i<=numSegs;i++) {
      segLen=SegLength(segStore,i);
      obsPerState=((float) segLen)/((float) (nStates-2));
      if (obsPerState < 1.0)
         HError(2122,"UCollectData: segment too short[%d]",segLen);
      for (j=1;j<=segLen;j++) {
         obs = GetSegObs(segStore,i,j);
         n = (int)(((float)(j-1)/obsPerState)+2);
         for (s=1; s<=nStreams; s++){
            if (hset.hsKind==DISCRETEHS){
               p = (Ptr)((int)obs.vq[s]);
               StoreItem(seqMat[n][s],p);
            }else
               StoreItem(seqMat[n][s],obs.fv[s]);
         }
      }
   }
}

/* UniformSegment: and cluster within each state/segment */
void UniformSegment(void)
{
   Sequence **seqMat;   /* Matrix [2..numStates-1][1..numStreams]*/
   Sequence seq;
   int count,size,i,vqidx,s,n,m,M,j,k;
   ClusterSet *cset;
   Cluster *c;
   StreamElem *ste;
   Covariance cov;
   CovKind ck;
   MixPDF *mp;
   TMixRec *tmRec = NULL;
   ShortVec dw;
   float x,z;
   Vector floor;

   if (trace & T_UNI)
      printf(" Uniform Segmentation\n");
   seqMat = CreateSeqMat();
   cov.var = NULL;      /* dummy */
   UCollectData(seqMat);
   if (trace&T_UNI) ShowSeqMat(seqMat);

   /* Cluster Each State/Stream and Init HMM Parms */
   for (n=2; n<nStates; n++) {
      if (trace&T_UNI) printf(" state %d ",n);
      for (s=1; s<=nStreams; s++){
         size = hset.swidth[s];
         floor = vFloor[s];
         ste = hmmLink->svec[n].info->pdf+s;
         if (hset.hsKind == TIEDHS){
            tmRec = &(hset.tmRecs[s]);
            M = tmRec->nMix;
            tmRec->topM = tmRec->nMix;
         }
         else
            M = ste->nMix; 
         if (trace&T_UNI) printf(" stream %d\n",s);
         seq = seqMat[n][s];
         switch (hset.hsKind){
         case PLAINHS:
         case SHAREDHS:
            ck = ste->spdf.cpdf[1].mpdf->ckind;
            cset = FlatCluster(&clustSetStack,seq,M,NULLC,ck,cov);
            if (trace&T_UNI) ShowClusterSet(cset);
            for (m=1; m<=M; m++){
               mp = ste->spdf.cpdf[m].mpdf;
               if (mp->ckind != ck)
                  HError(2123,"UniformSegment: different covkind within a mix\n");
               c = cset->cl+m;
               if (uFlags&UPMIXES)
                  ste->spdf.cpdf[m].weight = (float)c->csize/(float)seq->nItems;
               if (uFlags&UPMEANS)
                  CopyVector(c->vCtr,mp->mean);
               if (uFlags&UPVARS)
                  switch(ck){
                  case DIAGC:
                     for (j=1; j<=size; j++){
                        z= c->cov.var[j];
                        mp->cov.var[j] = (z<floor[j])?floor[j]:z;
                     }
                     break;
                  case FULLC:
                     for (j=1; j<=size; j++){
                        for (k=1; k<j; k++) {
                           mp->cov.inv[j][k] = c->cov.inv[j][k];
                        }
                        z = c->cov.inv[j][j];
                        mp->cov.inv[j][j] = (z<floor[j])?floor[j]:z;
                     }
                     break;
                  default:
                     HError(2124,"UniformSegment: bad cov kind %d\n",ck);
                  }
            }
            break;
         case DISCRETEHS:
            count = 0; dw = ste->spdf.dpdf;
            ZeroShortVec(dw);             
            for (i=1; i<=seq->nItems; i++){
               vqidx = (int)GetItem(seq,i);
               if (vqidx<1 || vqidx>M)
                  HError(2170,"UniformSegment: vqidx out of range[%d]",vqidx);
               ++dw[vqidx]; ++count;
            }
            for (m=1; m<=M; m++){
               x = (float)dw[m]/(float)count;
               if (x<mixWeightFloor) x = mixWeightFloor;
               dw[m] = DProb2Short(x);
            }
            break;
         case TIEDHS:
            ck = tmRec->mixes[1]->ckind;
            cset = FlatCluster(&clustSetStack,seq,M,NULLC,ck,cov);
            if (trace&T_UNI) ShowClusterSet(cset);
            for (m=1; m<=M; m++){
               mp = tmRec->mixes[m];
               if (mp->ckind != ck)
                  HError(2123,"UniformSegment: different covkind within a mix\n");
               c = cset->cl+m;
               if (uFlags&UPMIXES)
                  ste->spdf.tpdf[m] = (float)c->csize/(float)seq->nItems;
               if (uFlags&UPMEANS)
                  CopyVector(c->vCtr,mp->mean);
               if (uFlags&UPVARS)
                  switch(ck){
                  case DIAGC:
                     for (j=1; j<=size; j++){
                        z= c->cov.var[j];
                        mp->cov.var[j] = (z<floor[j])?floor[j]:z;
                     }
                     break;
                  case FULLC:
                     for (j=1; j<=size; j++){
                        for (k=1; k<j; k++) {
                           mp->cov.inv[j][k] = c->cov.inv[j][k];
                        }
                        z = c->cov.inv[j][j];
                        mp->cov.inv[j][j] = (z<floor[j])?floor[j]:z;
                     }
                     break;                     
                  default:
                     HError(2124,"UniformSegment: bad cov kind %d\n",ck);
                  }                 
            }
            break;       
         }
         ResetHeap(&clustSetStack);
      }
      if ((hset.hsKind == PLAINHS) || (hset.hsKind == SHAREDHS))
         FixGConsts(hmmLink);
   }
   ResetHeap(&sequenceStack);
}

/* ----------------- Viterbi Segmentation Routines ------------------- */

/* ShowTraceBack: print the Viterbi traceBack matrix */
void ShowTraceBack(int len, short **tB)
{
   int i,state;
   
   printf(" traceback matrix\n");
   for (state=2; state<nStates; state++) {
      printf("     %d     ",state);
      for (i=1;i<=len;i++)
         printf("%3d",tB[i][state]);
      printf("\n");
   }
   fflush(stdout);
}

/* ShowP: print current column of log probabilites */
void ShowP(int col, Vector colVec)
{
   int state;
      
   printf("     %d     ",col);
   for (state=2; state<nStates; state++) {
      if (colVec[state]<=LSMALL)
         printf("  ----  ");
      else
         printf("%8.2f",colVec[state]);
   }
   printf("\n"); fflush(stdout);
}

/* ShowAlignment: print states and mixes arrays */
void ShowAlignment(int segNum, int segLen, IntVec states, IntVec *mixes)
{
   int s;

   printf("Alignment for segment %d [%d obs]\n",segNum,segLen);
   ShowIntVec(" states ",states,40);
   if (mixes != NULL)
      for (s=1; s<=nStreams; s++){
         printf("%2d ",s); ShowIntVec("mix",mixes[s],40);
      }
}

/* MakeTraceBack: create the traceBack matrix */
void MakeTraceBack(int segLen)
{
   int segIdx;
   short *tmpPtr;

   traceBack = (short **)New(&traceBackStack, segLen*sizeof(short *));
   --traceBack;
   for (segIdx=1; segIdx<=segLen; segIdx++){
      tmpPtr = (short *)New(&traceBackStack, (nStates-2)*sizeof(short));
      traceBack[segIdx] = tmpPtr-2;
   }
}

/* DoTraceBack:  traceBack and set states array */
void DoTraceBack(int segLen, IntVec states, int thisState)
{
   int segIdx;

   for (segIdx=segLen; segIdx>0; segIdx--) {
      states[segIdx] = thisState;
      thisState=traceBack[segIdx][thisState];
   }
}

/* FindBestMixes: for each state/obs pair find most likely mix component */
void FindBestMixes(int segNum, int segLen, IntVec states, IntVec *mixes)
{
   int i,s,m,bestm,M=0;
   StreamElem *ste;
   IntVec smix;
   Observation obs;
   Vector v;
   LogFloat bestP,p;
   MixtureElem *me;
   MixPDF *mp;

   if (trace&T_MIX)
      printf(" Mixture component alignment\n");
   for (i=1; i<=segLen; i++){
      ste = hmmLink->svec[states[i]].info->pdf+1;
      obs = GetSegObs(segStore, segNum, i);
      if (hset.hsKind == TIEDHS)
         PrecomputeTMix(&hset, &obs, 0.0, 1);
      for (s=1; s<=nStreams; s++,ste++){
         if (hset.hsKind != TIEDHS)
            M = ste->nMix;
         smix = mixes[s];
         if (hset.hsKind==TIEDHS) /* PrecomputeTMix has already sorted probs */
            bestm = hset.tmRecs[s].probs[1].index;
         else if (M==1)
            bestm = 1;   
         else{
            v = obs.fv[s];
            bestP = LZERO; bestm=0;
            if (trace&T_MIX)
               printf("  seg %d, stream %d: ",i,s);
            for (m=1; m<=M; m++){
               me =  ste->spdf.cpdf+m;
               mp = me->mpdf;
               p = MOutP(v,mp);
               if (p>bestP){
                  bestP=p; bestm=m;
               }
               if (trace&T_MIX)
                  printf(" P(mix[%d])=%.1f",m,p);
            }
            if (bestm==0)
               HError(2125,"FindBestMixes: no best mix");
            if (trace&T_MIX)
               printf(" [best=%d]\n",bestm);
         }
         smix[i] = bestm;
      }
   }
}

/* ViterbiAlign: align the segNum'th segment.  For each frame k, store aligned
   state in states and mostly likely mix comp in mixes.  Return logP. */
LogFloat ViterbiAlign(int segNum,int segLen, IntVec states, IntVec *mixes)
{
   int currState,prevState,bestPrevState;
   int segIdx;
   LogFloat  bestP,currP,tranP,prevP;
   Observation obs;

   if (trace & T_VIT)
      printf(" Aligning Segment Number %d\n",segNum);
   MakeTraceBack(segLen);
   
   /* From entry state 1: Column 1 */
   obs = GetSegObs(segStore, segNum, 1);
   if (hset.hsKind == TIEDHS)
      PrecomputeTMix(&hset, &obs, 50.0, 0);
   for (currState=2;currState<nStates;currState++) {
      tranP = hmmLink->transP[1][currState];
      if (tranP<LSMALL) 
         lastP[currState] = LZERO;
      else
         lastP[currState] = tranP + OutP(&obs,hmmLink,currState);
      traceBack[1][currState] = 1;
   }
   if (trace & T_VIT) ShowP(1,lastP);  
   
   /* Columns[2] -> Columns[segLen] -- this is the general case */
   for (segIdx=2; segIdx<=segLen; segIdx++) {
      obs = GetSegObs(segStore, segNum, segIdx);
      if (hset.hsKind == TIEDHS)
         PrecomputeTMix(&hset, &obs, 50.0, 0);      
      for (currState=2;currState<nStates;currState++) {
         bestPrevState=2;
         tranP = hmmLink->transP[2][currState]; prevP = lastP[2];
         bestP = (tranP<LSMALL) ? LZERO : tranP+prevP;
         for (prevState=3;prevState<nStates;prevState++) {
            tranP = hmmLink->transP[prevState][currState];
            prevP = lastP[prevState];
            currP = (tranP<LSMALL) ? LZERO : tranP+prevP;
            if (currP > bestP) {
               bestPrevState=prevState; bestP=currP;
            }
         }
         if (bestP<LSMALL)
            currP = thisP[currState] = LZERO;
         else {
            currP = OutP(&obs,hmmLink,currState);
            thisP[currState] = bestP+currP;
         }
         if (trace&T_OBP)
            printf("OutP[s=%d,t=%d] = %f\n",currState,segIdx,currP);
         traceBack[segIdx][currState]=bestPrevState;
      }
      CopyVector(thisP,lastP);
      if (trace & T_VIT) ShowP(segIdx,lastP);   
   }
   
   /* column[segLen]--> exit state(numStates) */
   bestPrevState=2;
   tranP = hmmLink->transP[2][nStates]; prevP = lastP[2];
   bestP=(tranP<LSMALL) ? LZERO : tranP+prevP;
   for (prevState=3;prevState<nStates;prevState++) {
      tranP = hmmLink->transP[prevState][nStates]; prevP = lastP[prevState];
      currP = (tranP<LSMALL) ? LZERO : tranP+prevP;
      if (currP > bestP) {
         bestPrevState=prevState; bestP=currP;
      }
   }  

   /* bestPrevState now gives last internal state along best state sequence */
   if (bestP<LSMALL)
      HError(2126,"ViterbiAlign: No path found in %d'th segment",segNum);
   if (trace & T_VIT) {
      ShowTraceBack(segLen,traceBack);
      printf(" bestP = %12.5f via state %d\n",bestP,bestPrevState);
      fflush(stdout);
   }
   DoTraceBack(segLen,states,bestPrevState);
   if (mixes!=NULL)  /* ie not DISCRETE */
      FindBestMixes(segNum,segLen,states,mixes);
   ResetHeap( &traceBackStack );
   return bestP;  
}

/* ----------------- Update Count Routines --------------------------- */

/* UpdateCounts: using frames in seg i and alignment in states/mixes */
void UpdateCounts(int segNum, int segLen, IntVec states,IntVec *mixes)
{
   int M=0,i,j,k,s,m,state,last;
   StreamElem *ste;
   MixPDF *mp = NULL;
   WtAcc *wa;
   MuAcc *ma;
   VaAcc *va;
   TrAcc *ta;
   Vector v;
   Observation obs;
   TMixRec *tmRec = NULL;
   float x,y;

   last = 1;  /* last before 1st emitting state must be 1 */
   ta = (TrAcc *)GetHook(hmmLink->transP); 
   for (i=1; i<=segLen; i++){
      state = states[i];
      if (trace&T_CNT)
         printf("  Seg %d -> state %d\n",i,state);
      if (uFlags&(UPMEANS|UPVARS|UPMIXES)){
         obs = GetSegObs(segStore, segNum, i);
         if (hset.hsKind == TIEDHS)
            PrecomputeTMix(&hset, &obs, 50.0, 0);         
         ste = hmmLink->svec[state].info->pdf+1;
         for (s=1; s<=nStreams; s++,ste++){
            if (hset.hsKind==DISCRETEHS){
               m = obs.vq[s]; v = NULL;
            } else {
               v = obs.fv[s]; m = mixes[s][i];
            }
            switch(hset.hsKind){
            case TIEDHS:
               tmRec = &(hset.tmRecs[s]);
               M = tmRec->nMix;
               break;
            case PLAINHS:
            case SHAREDHS:
            case DISCRETEHS:
               M = ste->nMix;
               break;
            }
            if (m<1 || m > M)
               HError(2170,"UpdateCounts: mix/vq idx out of range[%d]",m);
            if (trace&T_CNT)
               printf("   stream %d -> mix %d[%d]\n",s,m,M); 
            /* update mixture weight */
            if (M>1 && (uFlags&UPMIXES)) {
               wa = (WtAcc *)ste->hook;
               wa->occ += 1.0; wa->c[m] += 1.0;
               if (trace&T_CNT)
                  printf("   mix wt -> %.1f\n",wa->c[m]);
            }
            if (hset.hsKind==DISCRETEHS) continue;
            
            /* update state/mixture component */
            switch(hset.hsKind){
            case PLAINHS:
            case SHAREDHS:
               mp = ste->spdf.cpdf[m].mpdf;
               break;
            case TIEDHS:
               mp = tmRec->mixes[m];
               break;
            }
            ma = (MuAcc *)GetHook(mp->mean);
            va = (VaAcc *)GetHook(mp->cov.var);
            ma->occ += 1.0; va->occ += 1.0;
            for (j=1; j<=hset.swidth[s]; j++) {
               x = v[j] - mp->mean[j];
               ma->mu[j] += x;
               if (uFlags&UPVARS)
                  switch(mp->ckind){
                  case DIAGC: 
                     va->cov.var[j] += x*x;
                     break;
                  case FULLC:
                     for (k=1; k<=j; k++){
                        y = v[k]-mp->mean[k];
                        va->cov.inv[j][k] += x*y;
                     }
                     break;
                  default:
                     HError(2124,"UpdateCounts: bad cov kind %d\n",
                            mp->ckind);
                  }                 
            }
            if (trace&T_CNT) {
               ShowVector("   mean ->",ma->mu,6);
               if (uFlags&UPVARS) {
                  if (mp->ckind==DIAGC)
                     ShowVector("   var ->",va->cov.var,6);
                  else
                     ShowTriMat("   cov ->",va->cov.inv,6,6);
               }
               fflush(stdout);
            }
         }
      }
      /* update transition probs */
      if (uFlags&UPTRANS){
         ta->occ[last] += 1.0;
         ta->tran[last][state] += 1.0;
         last = state;
         if (i==segLen){  /* remember final state */
            ta->occ[state] += 1.0;
            ta->tran[state][nStates] += 1.0;
         }
         if (trace&T_CNT) {
            ShowMatrix("   tran ->",ta->tran,6,6);
            fflush(stdout);
         }
      }
   }
}

/* ----------------- Update Parameters --------------------------- */

/* UpWeights: update given mixture weights */
void UpWeights(int i, int s, int M, WtAcc *wa, StreamElem *ste)
{
   int m;
   float sum=0.0;
   
   if (wa->occ == 0.0)
      HError(2127,"UpWeights: zero occ i=%d/s=%d",i,s);
   for (m=1; m<=M; m++){
      sum += wa->c[m];
      switch(hset.hsKind){
      case PLAINHS:
      case SHAREDHS:
         ste->spdf.cpdf[m].weight = wa->c[m] / wa->occ;
         break;
      case TIEDHS:
         ste->spdf.tpdf[m] = wa->c[m] / wa->occ;
         break;
      }
   }
   if (fabs(sum-wa->occ)/sum > 0.001)
      HError(2190,"UpWeights: mix weight sum error");
}

/* UpMeans: update mean, leave old mean in acc */
void UpMeans(int i, int s, int m, int size, MuAcc *ma, Vector mean)
{
   int k;
   float x;
   
   if (ma->occ == 0.0)
      HError(2127,"UpMeans: zero occ i=%d/s=%d/m=%d",i,s,m);
   for (k=1; k<=size; k++){
      x = mean[k] + ma->mu[k]/ma->occ;
      ma->mu[k] = mean[k];  /* remember old mean */
      if (uFlags&UPMEANS) mean[k] = x;
   }
}

/* UpVars: update variances, apply correction if covariance is 
           not shared */
void UpVars(int i, int s, int m, int size, VaAcc *va, Vector oldMean,
            Vector newMean, Boolean shared, MixPDF *mp)
{
   int j,k;
   float x,y,z;
   Vector floor;
                        
   if (va->occ == 0.0)
      HError(2127,"UpVars: zero occ i=%d/s=%d/m=%d",i,s,m);
   floor=vFloor[s];
   switch(mp->ckind){
   case DIAGC:
      for (j=1; j<=size; j++){
         x = (shared)?0.0:newMean[j]-oldMean[j];
         z = va->cov.var[j]/va->occ - x*x;
         mp->cov.var[j] = (z<floor[j])?floor[j]:z;
      }
      FixDiagGConst(mp);
      break;
   case FULLC:
      for (j=1; j<=size; j++){
         x = (shared)?0.0:newMean[j]-oldMean[j];
         for (k=1; k<j; k++) {
            y = (shared)?0.0:newMean[k]-oldMean[k];
            mp->cov.inv[j][k] = va->cov.inv[j][k]/va->occ - x*y;
         }
         z = va->cov.inv[j][j]/va->occ - x*x;
         mp->cov.inv[j][j] = (z<floor[j])?floor[j]:z;
      }
      FixFullGConst(mp,CovInvert(mp->cov.inv,mp->cov.inv));
      break;
   default:
      HError(2124,"UpVars: bad cov kind %d",mp->ckind);
   }
}

/* UpTrans: update transition parameters */
void UpTrans(TrAcc *ta, Matrix tr)
{
   int i,j;
   float occi,x,sum;

   for (i=1; i<nStates; i++){
      occi = ta->occ[i];
      if (occi == 0.0)
         HError(2127,"UpTrans: zero occ in state %d",i);
      sum = 0.0;
      tr[i][1] = LZERO;
      for (j=2;j<=nStates;j++) {
         x = ta->tran[i][j]/occi;
         tr[i][j] = x; sum += x;
      }
      if (fabs(sum-1.0) > 0.001)
         HError(2190,"UpTrans: row %d, sum=%f",i,sum,occi);
      for (j=2;j<=nStates;j++) {
         x = tr[i][j]/sum;
         tr[i][j] = (x<MINLARG) ? LZERO : log(x);
      }
   }
}

/* UpDProbs: update given mixture weights */
void UpDProbs(int i, int s, int M, WtAcc *wa, ShortVec dw)
{
   int m;
   float x,sum=0.0;
   
   if (wa->occ == 0.0)
      HError(2127,"UpDProbs: zero occ i=%d/s=%d",i,s);
   for (m=1; m<=M; m++){
      sum += wa->c[m];
      x = wa->c[m] / wa->occ;
      if (x<mixWeightFloor) x = mixWeightFloor;
      dw[m] = DProb2Short(x);
   }
   if (fabs(sum-wa->occ)/sum > 0.001)
      HError(2190,"UpDProbs: dprob weight sum error");
}

/* UpdateParameters: in hmm using counts in accumulators */
void UpdateParameters(void)
{
   HMMScanState hss;
   int size;
   StreamElem *ste;
   WtAcc *wa;
   MuAcc *ma = NULL;
   VaAcc *va;
   TrAcc *ta;
   Boolean hFound = FALSE,shared;

   NewHMMScan(&hset,&hss);
   do if (hmmLink == hss.hmm){
      hFound = TRUE;
      while (GoNextState(&hss,TRUE)) {
         while (GoNextStream(&hss,TRUE)) {
            ste = hss.ste;
            if (hss.M>1 && (uFlags&UPMIXES)){
               wa = (WtAcc *)ste->hook;
               if (hset.hsKind == DISCRETEHS)
                  UpDProbs(hss.i,hss.s,hss.M,wa,ste->spdf.dpdf);
               else
                  UpWeights(hss.i,hss.s,hss.M,wa,ste);
            }
            size = hset.swidth[hss.s];
            if (hss.isCont && (uFlags&(UPMEANS|UPVARS)))/*PLAINHS or SHAREDHS*/
               while (GoNextMix(&hss,TRUE)) {
                  if (!IsSeenV(hss.mp->mean)) {
                     ma = (MuAcc *)GetHook(hss.mp->mean);
                     UpMeans(hss.i,hss.s,hss.m,size,ma,hss.mp->mean);
                     /* NB old mean left in ma->mu */
                     TouchV(hss.mp->mean);
                  }
                  if (!IsSeenV(hss.mp->cov.var)) {
                     if (uFlags&UPVARS) {
                        va = (VaAcc *)GetHook(hss.mp->cov.var);
                        shared = GetUse(hss.mp->cov.var) > 1;
                        UpVars(hss.i,hss.s,hss.m,size,va,ma->mu,hss.mp->mean,
                               shared,hss.mp);
                     }
                     TouchV(hss.mp->cov.var);
                  }
               }
         }
      }
      if (!IsSeenV(hmmLink->transP)) {
         if (uFlags&UPTRANS){
            ta = (TrAcc *)GetHook(hmmLink->transP);
            UpTrans(ta,hmmLink->transP);
         }
         TouchV(hmmLink->transP);       
      }
   } while (!hFound && GoNextHMM(&hss));
   EndHMMScan(&hss);
   if (!hFound)
      HError(2129,"UpdateParameters: hmm not found");
}

/* ----------------- Top Level of Estimation Procedure --------------- */


/* CreateMixes: create array[1..S][1..segLen] of mix component index */
IntVec *CreateMixes(MemHeap *x,int segLen)
{
   IntVec *mixes;
   int s;

   mixes = (IntVec*)New(x,sizeof(IntVec)*nStreams);
   --mixes;
   for (s=1; s<=nStreams; s++)
      mixes[s] = CreateIntVec(x,segLen);
   return mixes;
}

/* EstimateModel: top level of iterative estimation process */
void EstimateModel(void)
{
   LogFloat totalP,newP,delta;
   Boolean converged = FALSE;
   int i,iter,numSegs,segLen;    
   IntVec states;  /* array[1..numSegs] of State */
   IntVec *mixes;  /* array[1..S][1..numSegs] of MixComp */

   if (trace&T_TOP) printf("Starting Estimation Process\n");
   if (newModel){
      UniformSegment();
   }
   totalP=LZERO;
   for (iter=1; !converged && iter<=maxIter; iter++){
      ZeroAccs(&hset, uFlags);              /* Clear all accumulators */
      numSegs = NumSegs(segStore);
      /* Align on each training segment and accumulate stats */
      for (newP=0.0,i=1;i<=numSegs;i++) {
         segLen = SegLength(segStore,i);
         states = CreateIntVec(&gstack,segLen);
         mixes  = (hset.hsKind==DISCRETEHS)?NULL:
            CreateMixes(&gstack,segLen);
         newP += ViterbiAlign(i,segLen,states,mixes);
         if (trace&T_ALN) ShowAlignment(i,segLen,states,mixes);
         UpdateCounts(i,segLen,states,mixes);
         FreeIntVec(&gstack,states); /* disposes mixes too */
      }
      /* Update parameters or quit */
      newP /= (float)numSegs;
      delta = newP - totalP;
      converged = (iter>1) && (fabs(delta) < epsilon);
      if (!converged)
         UpdateParameters();
      totalP = newP;
      if (trace & T_TOP){
         printf("Iteration %d: Average LogP =%12.5f",iter,totalP);
         if (iter > 1)
            printf("  Change =%12.5f\n",delta);
         else
            printf("\n");
         fflush(stdout);
      }
   }
   if (trace&T_TOP) {
      if (converged) 
         printf("Estimation converged at iteration %d\n",iter);
      else
         printf("Estimation aborted at iteration %d\n",iter);
      fflush(stdout);
   }
}

/* ------------------------- Save Model ----------------------- */

/* SaveModel: save HMMSet containing one model */
void SaveModel(char *outfn)
{
   if (outfn != NULL)
      macroLink->id = GetLabId(outfn,TRUE);
   if(SaveHMMSet(&hset,outDir,NULL,NULL,saveBinary)<SUCCESS)
      HError(2111,"SaveModel: SaveHMMSet failed");
}

/* ----------------------------------------------------------- */
/*                      END:  HInit.c                         */
/* ----------------------------------------------------------- */





