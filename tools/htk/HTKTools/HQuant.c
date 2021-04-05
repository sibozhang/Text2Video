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
/*       File: HQuant.c: VQ table generation                   */
/* ----------------------------------------------------------- */

char *hquant_version = "!HVER!HQuant:   3.4.1 [CUED 12/03/09]";
char *hquant_vc_id = "$Id: HQuant.c,v 1.1.1.1 2006/10/11 09:55:01 jal58 Exp $";

/* 
   This program calculates a vector quantisation table from a
   sequence of training files.
*/ 

#include "HShell.h"
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

/* ------------------- Trace Flags & Vars ------------------------ */

#define T_TOP     0001              /* basic progress reporting */
#define T_MEAN    0002              /* dump global mean and cov */
#define T_LOAD    0004              /* trace data loading */
#define T_SEGS    0010              /* list label segments */
#define T_CLUST   0020              /* dump clusters */
#define T_TAB     0040              /* dump vq table */

static int trace = 0;               /* trace level */
static ConfParam *cParm[MAXGLOBS];   /* configuration parameters */
static int nParm = 0;               /* total num params */

/* ---------------------- Global Variables ----------------------- */

#define DEF_NCLUST 256              /* default codebook size */

static BufferInfo info;             /* global observation format, etc. */
static int cbSizes[SMAX];           /* codebook sizes, per stream */
static short swidth[SMAX];          /* stream widths */
static TreeType tType = linTree;    /* codebook structure */
static char *vqfn = NULL;           /* filename for output VQ table */
static char *segLab = NULL;         /* segment label, if any */
static LabId segId  = NULL;         /* and its id */
static char *labDir = NULL;         /* label file directory */
static char *labExt = "lab";        /* label file extension */
static FileFormat dff=UNDEFF;       /* data file format */
static FileFormat lff=UNDEFF;       /* label file format */

static MemHeap iStack;             /* input buffer  */
static MemHeap dStack;             /* sequence stack */
static MemHeap cStack;             /* cluster stack */
Sequence dSeq[SMAX];               /* main data pools; one per stream */
ClusterSet *cs[SMAX];              /* stores vector clusters */
CovKind ck = NULLC;                /* determines distance metric */
Vector mean[SMAX];                 /* Global stream mean  */
Covariance cov[SMAX];              /* Global stream covariance  */
Boolean widthSet = FALSE;          /* true if width of any stream is set */

static Observation obs;             /* storage for observations  */
static Boolean globClustVar = FALSE;/*Output global variance of data to
                                      codebook in place of individual vars*/

/* ------------- Process Command Line and Check Data ------------ */

/* SetConfParms: set conf parms relevant to HQuant  */
void SetConfParms(void)
{
   int i;
   
   nParm = GetConfig("HQUANT", TRUE, cParm, MAXGLOBS);
   if (nParm>0) {
      if (GetConfInt(cParm,nParm,"TRACE",&i)) trace = i;
   }
}

/* InitStreamVars: set initial values of stream-indexed variables */
void InitStreamVars(void)
{
   int s;

   swidth[0] = cbSizes[0] = 0;
   for(s=1;s<SMAX;s++) {
      swidth[s] = 0;
      cbSizes[s] = DEF_NCLUST;
      mean[s] = NULL;
   }
}

void ReportUsage(void)
{
   printf("\nUSAGE: HQuant [options] vqFile trainFiles...\n\n" );
   printf(" Option                                       Default\n\n");
   printf(" -d      Use diagonal cov Mahalanobis         Euclidean\n");
   printf(" -f      Use full covariance Mahalanobis      Euclidean\n");
   printf(" -g      Output global covar to codebook      off\n");
   printf(" -l s    Set segment label to s               none\n");
   printf(" -n S N  Set codebook size for stream S to N  N=%d\n",DEF_NCLUST);
   printf(" -s N    Set number of streams to N           1\n");
   printf(" -t      Create tree-stuctured codebooks      linear\n");
   printf(" -w S N  Set width of stream S to N           default\n");
   PrintStdOpts("FGILX");
   printf("\n\n");
}

int main(int argc, char *argv[])
{
   char *datafn, *s;
   int stream = 0;
   void Initialise(char *datafn);
   void LoadFile(char *fn);
   void CalcMeanCov(Sequence seq[], int s);
   void ClusterVecs(Sequence seq[], int s);
   void WriteVQTable(ClusterSet *cs[], char *fn);  

   if(InitShell(argc,argv,hquant_version,hquant_vc_id)<SUCCESS)
      HError(2500,"HQuant: InitShell failed");

   InitMem();   InitLabel();
   InitMath();  InitSigP();
   InitWave();  InitAudio();
   InitVQ();    InitModel();
   if(InitParm()<SUCCESS)  
      HError(2500,"HQuant: InitParm failed");
   InitTrain();

   if (!InfoPrinted() && NumArgs() == 0)
      ReportUsage();
   if (NumArgs() == 0) Exit(0);
   SetConfParms();
   InitStreamVars();

   while (NextArg() == SWITCHARG) {
      s = GetSwtArg();
      if (strlen(s)!=1) 
         HError(2519,"HQuant: Bad switch %s; must be single letter",s);
      switch(s[0]){
      case 'd':
         if ( ck != NULLC) HError(2519,"HQuant: Specify one of -d or -f, not both");
         ck = INVDIAGC;
         break;
      case 'f':
         if ( ck != NULLC) HError(2519,"HQuant: Specify one of -d or -f, not both");
         ck = FULLC;
         break;
      case 'g':
         globClustVar = TRUE;
         break;
      case 'l':
         if (NextArg() != STRINGARG)
            HError(2519,"HQuant: Segment label expected");
         segLab = GetStrArg();
         break;
      case 'n':
         if (NextArg() != INTARG)
            HError(2519,"HQuant: Stream number expected");
         stream = GetChkedInt(1,SMAX,s);
         if (NextArg() != INTARG)
            HError(2519,"HQuant: Codebook size expected");
         cbSizes[stream]= GetChkedInt(1,32768,s);
         break;
      case 's':
         if (NextArg() != INTARG)
            HError(2519,"HQuant: Number of streams expected");
         swidth[0] = GetChkedInt(1,SMAX,s);
         break;
      case 't':
         tType = binTree;
         break;
      case 'w':
         if (NextArg() != INTARG)
            HError(2519,"HQuant: Stream number expected");
         stream = GetChkedInt(1,SMAX,s);
         if(swidth[0] < stream) swidth[0] = stream;
         widthSet = TRUE;
         if (NextArg() != INTARG)
            HError(2519,"HQuant: Stream width expected");
         swidth[stream]= GetChkedInt(1,256,s);
         break;
      case 'F':
         if (NextArg() != STRINGARG)
            HError(2519,"HQuant: Data File format expected");
         if((dff = Str2Format(GetStrArg())) == ALIEN)
            HError(-2589,"HQuant: Warning ALIEN Data file format set");
         break;
      case 'G':
         if (NextArg() != STRINGARG)
            HError(2519,"HQuant: Label File format expected");
         if((lff = Str2Format(GetStrArg())) == ALIEN)
            HError(-2589,"HQuant: Warning ALIEN Label file format set");
         break;
      case 'I':
         if (NextArg() != STRINGARG)
            HError(2519,"HQuant: MLF file name expected");
         LoadMasterFile(GetStrArg());
         break;
      case 'L':
         if (NextArg()!=STRINGARG)
            HError(2519,"HQuant: Label file directory expected");
         labDir = GetStrArg();
         break;
      case 'T':
         if (NextArg() != INTARG)
            HError(2519,"HQuant: Trace value expected");
         trace = GetChkedInt(0,077,s); 
         break;
      case 'X':
         if (NextArg()!=STRINGARG)
            HError(2519,"HQuant: Label file extension expected");
         labExt = GetStrArg();
         break;
      default:
         HError(2519,"HQuant: Unknown switch %s",s);
      }
   }
   if (NextArg()!=STRINGARG)
      HError(2519,"HQuant: Output VQ table file name expected");
   vqfn = GetStrArg();

   if (NextArg()!=STRINGARG)
      HError(2519,"HQuant: Training data file name expected");
   datafn = GetStrArg();
   Initialise(datafn);
   LoadFile(datafn);
   while (NumArgs()>0) {
      if (NextArg()!=STRINGARG) 
         HError(2519,"HQuant: Training data file name expected");
      datafn = GetStrArg();
      LoadFile(datafn);
   }
   
   for (stream=1;stream<=swidth[0];stream++){
      if (trace&T_TOP)
         printf("%s-clustering data for stream %d (width %d)\n",
                (tType==linTree)?"Flat":"Tree",stream,swidth[stream]);
      CalcMeanCov(dSeq,stream);
      ClusterVecs(dSeq,stream);
   }
   WriteVQTable(cs,vqfn);
   Exit(0);
   return (0);          /* never reached -- make compiler happy */
}


/* Determine global mean and covariance of data in sequence s */
void CalcMeanCov(Sequence seq[], int s)
{
   mean[s] = CreateVector(&dStack,swidth[s]);
   SequenceMean(seq[s],mean[s]);
   if (trace&T_MEAN) {
      printf("Stream %d",s);
      ShowVector(" global mean:\n",mean[s],20);
   }
   switch(ck){
   case INVDIAGC: 
   case DIAGC: 
      cov[s].var = CreateVector(&dStack,swidth[s]);
      SequenceCov(seq[s],ck,cov[s],mean[s]);
      if(trace&T_MEAN) ShowVector("Global variance",cov[s].var,20);
      break;
   case FULLC: 
      cov[s].inv = CreateTriMat(&dStack,swidth[s]);
      SequenceCov(seq[s],ck,cov[s],mean[s]);
      if(trace&T_MEAN) ShowTriMat("Global covariance",cov[s].inv,20,20);
      break;
   case NULLC: 
   default:          
      cov[s].var = NULL;
      if(trace&T_MEAN) printf("Using Euclidean distance\n");      
      break;
   }
}

/* ClusterVecs: apply required clustering to vectors in stream s */
void ClusterVecs(Sequence *seq, int s)
{
   switch(tType) {
   case binTree:
      cs[s] = TreeCluster(&cStack,seq[s],cbSizes[s],ck,ck,cov[s]);
      break;
   
   case linTree:
      cs[s] = FlatCluster(&cStack,seq[s],cbSizes[s],ck,ck,cov[s]);
      break;
   
   default:
      HError(2590,"ClusterVecs: Unsupported tree type %d",tType);
   }
   if(trace&T_CLUST) ShowClusterSet(cs[s]);
}

/* ---------------- Procedures to construct a VQ table ------------ */

/* AddLinEntries: Construct linear VQ table from cluster set */
VQNode  AddLinEntries(ClusterSet *cs, int s)
{
   int i,rid;
   VQNode n = NULL, lastN = NULL;
   Cluster *c;
   
   for (i=1,c=cs->cl+1; i<=cs->numClust; i++,c++) {
      rid = (i==cs->numClust)?0:i+1;
      if( globClustVar )
         n = CreateVQNode(i,i,0,rid,c->vCtr,ck,cov[s]);
      else
         n = CreateVQNode(i,i,0,rid,c->vCtr,ck,c->cov);
      n->right = lastN;
      lastN = n;
   }  
   return(n);
}

/* AddBinEntries: Construct binary tree VQ table from cluster set */ 
VQNode AddBinEntries(ClusterSet *cs, short nid, int s)
{
   Cluster *c;
   VQNode n;
   Covariance* cptr;

   /* parent is leaf node so end recursion */
   if(nid > cs->numClust) return(NULL);

   /* Create this node */
   c = cs->cl + nid;    /* cluster corresponding to this node */

   if( globClustVar )   /* Output global covariance */
      cptr = &(cov[s]);
   else
      cptr = &(c->cov);

   if (nid > cs->numClust/2)    /* if a leaf node */
      n = CreateVQNode(nid-(cs->numClust/2),nid,0,0,c->vCtr,ck,*cptr);
   else
      n = CreateVQNode(0,nid,2*nid,2*nid + 1,c->vCtr,ck,*cptr);

   /* tree-structured clusters stored as follows:
      1
      2   3
      4   5 6  7
      . . . . . . . .
      so lid = 2*nid, rid = 2*nid+1, leaves have nid > numClust/2 */

   /* recursively create  children */
   n->left  = AddBinEntries(cs, 2*nid, s);
   n->right = AddBinEntries(cs, 2*nid+1, s);
   return(n);
}

/* WriteVQTable: Create VQ table in memory then write to file */
void WriteVQTable(ClusterSet *cs[], char *fn)
{
   int s;
   VQTable vq;

   vq = CreateVQTab(fn,(short)info.tgtPK,tType,ck,swidth);
   for (s=1;s<=swidth[0];s++){
      if (cs[s]->isTree)
         vq->tree[s] = AddBinEntries(cs[s],1,s);
      else
         vq->tree[s] = AddLinEntries(cs[s],s);
      if (trace&T_TOP) printf("[%d] ",cs[s]->numClust);
   }
   StoreVQTab(vq,fn);
   if (trace&T_TOP) printf("entries -> %s\n",fn);
   if(trace & T_TAB) PrintVQTab(vq);
}


/* ------------------------ Initialisation ----------------------- */

/* CheckStreamWidths: check that user-specified stream widths make sense */
void CheckStreamWidths(BufferInfo info)
{
   int s, sum = 0;

   for(s=1; s<=swidth[0];s++){
      if(swidth[s] > info.tgtVecSize)
         HError(2530,"CheckStreamWidths: Specified stream %d width [%d] is wider than data [%d]",
                s,swidth[s], info.tgtVecSize);
      if(widthSet && swidth[s] == 0 )
         HError(2530,"CheckStreamWidths: Width for stream %d must be specified",s);
      sum += swidth[s];
   }

   if(sum > info.tgtVecSize)
      HError(2530,"CheckStreamWidths: Specified stream widths are wider than data [%d]",
             info.tgtVecSize);

   for(; s<SMAX; s++) {
      if(cbSizes[s] != DEF_NCLUST)
         HError(-2530,"CheckStreamWidths: Codebook size set for non-existent stream %d",s);
   }
}

/* Initialise: set up global data storage */
void Initialise(char *datafn)
{
   ParmBuf pbuf;
   int s;
   Boolean eSep;

   CreateHeap(&iStack,"inBuf",     MSTAK, 1, 0.5, 100000, LONG_MAX);
   CreateHeap(&dStack,"seqStack",  MSTAK, 1, 0.5, 100000, LONG_MAX);
   CreateHeap(&cStack,"clustStack",MSTAK, 1, 0.5, 100000, LONG_MAX);

   /* Peek at first data file to get observation format */
   if((pbuf = OpenBuffer(&iStack, datafn, 0, UNDEFF, FALSE_dup, FALSE_dup))==NULL)
      HError(2550,"Initialise: Config parameters invalid");
   GetBufferInfo(pbuf, &info);
   CloseBuffer(pbuf);
   ResetHeap(&iStack);

   /* set/validate stream widths */
   if(swidth[0] > 0)
      CheckStreamWidths(info);
   else
      ZeroStreamWidths(1,swidth);

   /* Create an observation to hold the input parameters */
   SetStreamWidths(info.tgtPK,info.tgtVecSize,swidth,&eSep);
   obs = MakeObservation(&gstack,swidth,info.tgtPK,FALSE,eSep);

   if (segLab != NULL)
      segId = GetLabId(segLab,TRUE);

   /* Create sequences to hold all data*/
   for (s=1;s<=swidth[0];s++)
      dSeq[s] = CreateSequence(&dStack,4096);
}

/* ------------------------- Load Data  ----------------------------- */

/* CheckData: check data file consistent with already loaded data */
void CheckData(char *fn, BufferInfo newInfo) 
{
   if (newInfo.tgtVecSize!=info.tgtVecSize)
      HError(2531,"CheckData: Wrong vector size in %s [%d], should be [%d]",
             fn,newInfo.tgtVecSize,info.tgtVecSize);
   if (newInfo.tgtPK != info.tgtPK)
      HError(2531,"CheckData: Parm kind of %s differs from data already read",fn);
}

/* LoadFile: load whole file or segments and accumulate variance */
void LoadFile(char *fn)
{
   ParmBuf pbuf;
   BufferInfo info;
   char labfn[80];
   Transcription *trans;
   long segStIdx,segEnIdx;  
   int i,s,j,ncas,nObs=0;
   LLink p;

   if (segId == NULL)  {   /* load whole parameter file */
      if((pbuf=OpenBuffer(&iStack, fn, 0, dff, FALSE_dup, FALSE_dup))==NULL)
         HError(2550,"LoadFile: Config parameters invalid");
      GetBufferInfo(pbuf,&info);
      CheckData(fn,info);
      nObs = ObsInBuffer(pbuf);
      
      for (i=0; i<nObs; i++) {
         for(s=1;s<=swidth[0];s++)
            obs.fv[s] = CreateVector(&dStack,swidth[s]);
         ReadAsTable(pbuf,i,&obs);
         for(s=1;s<=swidth[0];s++)
            StoreItem(dSeq[s],(Ptr)obs.fv[s]);
      }
      CloseBuffer(pbuf);
   }
   else { /* load segment of parameter file */
      MakeFN(fn,labDir,labExt,labfn);
      trans = LOpen(&iStack,labfn,lff);
      ncas = NumCases(trans->head,segId);
      if ( ncas > 0) {
         if((pbuf=OpenBuffer(&iStack, fn, 0, dff, FALSE_dup, FALSE_dup))==NULL)
            HError(2550,"LoadFile: Config parameters invalid");
         GetBufferInfo(pbuf,&info);
         CheckData(fn,info);
         for (i=1,nObs=0; i<=ncas; i++) {
            p = GetCase(trans->head,segId,i);
            segStIdx= (long) (p->start/info.tgtSampRate);
            segEnIdx  = (long) (p->end/info.tgtSampRate);
            if (trace&T_SEGS)
               printf(" loading seg %s [%ld->%ld]\n",
                      segId->name,segStIdx,segEnIdx);
            if (segEnIdx >= ObsInBuffer(pbuf))
               segEnIdx = ObsInBuffer(pbuf)-1;
            if (segEnIdx >= segStIdx) {
               for (j=segStIdx;j<=segEnIdx;j++) {
                  /* SJY: The HInit code I copied this from had no */
                  /* SJY: CreateVector call here -- a bug? */
                  for(s=1;s<=swidth[0];s++)
                     obs.fv[s] = CreateVector(&dStack,swidth[s]);
                  ReadAsTable(pbuf,j,&obs);
                  for(s=1;s<=swidth[0];s++)
                     StoreItem(dSeq[s],(Ptr)obs.fv[s]);
                  ++nObs;
               }
            }
         }        
         CloseBuffer(pbuf);
      }  
   }
   ResetHeap(&iStack);
   if (trace&T_LOAD) {
      printf(" %5d obs loaded from %s, streams: ",nObs,fn);
      for(s=1;s<=swidth[0];s++) printf("[%d]" ,swidth[s]);
      printf("\n"); fflush(stdout);
   }        
}

/* ----------------------------------------------------------- */
/*                      END:  HQuant.c                         */
/* ----------------------------------------------------------- */
