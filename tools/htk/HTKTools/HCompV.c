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
/*              2001  Cambridge University                     */
/*                    Engineering Department                   */
/*                                                             */
/*   Use of this software is governed by a License Agreement   */
/*    ** See the file License for the Conditions of Use  **    */
/*    **     This banner notice must not be removed      **    */
/*                                                             */
/* ----------------------------------------------------------- */
/*  File: HCompV.c: HMM global mean/variance initialisation    */
/* ----------------------------------------------------------- */

char *hcompv_version = "!HVER!HCompV:   3.4.1 [CUED 12/03/09]";
char *hcompv_vc_id = "$Id: HCompV.c,v 1.1.1.1 2006/10/11 09:54:59 jal58 Exp $";


/* 
   This program calculates a single overall variance vector from a
   sequence of training files.  This vector is then copied into
   all of the components of all the states of the hidden Markov
   model specified on the command line.  Optionally, HCompV
   will also update the mean vector. HCompV.c can be used as
   the initial step in Fixed Variance and Grand Variance training schemes,
   and to initialise HMMs for "flat-start" training.  
   The structure of the HMM to be initialised must be
   defined beforehand (eg using a prototype HMM def).  

   It can also be used to generate variance floor vectors.
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
#include "HUtil.h"

/* -------------------------- Trace Flags & Vars ------------------------ */

#define T_TOP     0001           /* basic progress reporting */
#define T_COVS    0002           /* show covariance matrices */
#define T_LOAD    0004           /* trace data loading */
#define T_SEGS    0010           /* list label segments */
#define T_CMV     0012           /* doing CMV */

static int  trace    = 0;           /* trace level */
static ConfParam *cParm[MAXGLOBS];   /* configuration parameters */
static int nParm = 0;               /* total num params */

/* ---------------------- Global Variables ----------------------- */

static char *segLab = NULL;         /* segment label if any */
static LabId  segId  = NULL;        /* and its id */
static LabId  hmmId  = NULL;        /* id of model */
static char *labDir = NULL;         /* label file directory */
static char *labExt = "lab";        /* label file extension */
static float minVar  = 0.0;         /* minimum variance */
static FileFormat dff=UNDEFF;       /* data file format */
static FileFormat lff=UNDEFF;       /* label file format */
static char *hmmfn=NULL;            /* HMM definition file name */
static char *outfn=NULL;            /* output HMM file name (name only) */
static char *outDir=NULL;           /* HMM output directory */
static long totalCount=0;           /* total number of vector samples*/
static Boolean meanUpdate = FALSE;  /* update means  */
static Boolean saveBinary = FALSE;  /* save output in binary  */
static float vFloorScale = 0.0;     /* if >0.0 then vFloor scaling */

/* Major Data Structures */
static MLink macroLink;             /* Link to specific HMM macro */
static HLink hmmLink;               /* Link to the physical HMM */
static HMMSet hset;                 /* HMM to be initialised with */

static MemHeap iStack;

/* Storage for mean and covariance accumulators */
typedef struct {
   Vector       meanSum;            /* acc for mean vector value */
   Covariance   squareSum;          /* acc for sum of squares */
   Covariance   fixed;              /* fixed (co)variance values */
} CovAcc;
static CovAcc accs[SMAX];           /* one CovAcc for each stream */
static Boolean fullcNeeded[SMAX];   /* true for each stream that needs full
                                       covariance calculated */
static Observation obs;             /* storage for observations  */


/* ------------ structures for cmn ------------ */

typedef struct{
   char SpkrName[MAXSTRLEN];             /* speaker name */
   int NumFrame;                         /* number of frames for speaker */
   Vector meanSum;                       /* mean accumulate structure for speaker */
   Vector squareSum;                     /* variance structure for speaker */
}SpkrAcc;                  

typedef struct SpkrAccListItem{
   SpkrAcc *sa;                          /* speaker accumulate */
   struct SpkrAccListItem *nextSpkr;     /* next pointer */
}SpkrAccListItem;

static SpkrAccListItem *salist = NULL;   /* global speaker accumulate list */
static int vSize = 0;                    /* target observation vector size */
static char spPattern[MAXSTRLEN];        /* speaker mask */
static char pathPattern[MAXSTRLEN];      /* path mask */
static char oflags[MAXSTRLEN] = "m";     /* export flags for CMV */  
static char cmDir[MAXSTRLEN];            /* directory to export CMV */
static char TargetPKStr[MAXSTRLEN];      /* target parm kind string */
static Boolean DoCMV = FALSE;            /* switch from old HCompV to CMV */


/* ------------- Process Command Line and Check Data ------------ */

/* SetConfParms: set conf parms relevant to HCompV  */
void SetConfParms(void)
{
   Boolean b,c;
   int i;
   double d;
   
   nParm = GetConfig("HCOMPV", TRUE, cParm, MAXGLOBS);
   if (nParm>0) {
      if (GetConfInt(cParm,nParm,"TRACE",&i)) trace = i;
      if (GetConfBool(cParm,nParm,"UPDATEMEANS",&b)) meanUpdate = b;
      if (GetConfBool(cParm,nParm,"SAVEBINARY",&c)) saveBinary = c;
      if (GetConfFlt(cParm,nParm,"MINVARFLOOR",&d)) minVar = d;
   }
}

void ReportUsage(void)
{
   printf("\nUSAGE: HCompV [options] [hmmFile] trainFiles...\n" );
   printf(" Option                                       Default\n\n");
   printf(" -c dir  Set output directiry for CMV         none\n");
   printf(" -f f    Output vFloor as f * global var      none\n");
   printf(" -k s    spkr pattern for CMV                 none\n");
   printf(" -l s    Set segment label to s               none\n");
   printf(" -m      Update means                         off\n");
   printf(" -o fn   Store new hmm def in fn (name only)  outDir/srcfn\n");
   printf(" -p s    path pattern for CMV                 none\n");
   printf(" -q nmv  output type flags for CMV            m\n");
   printf(" -v f    Set minimum variance to f            0.0\n");
   PrintStdOpts("BCFGHILMX");
   printf("\n\n");
}


/* ------------------------ Initialisation ----------------------- */

/* CheckVarianceKind: set fullcNeeded[s] for each non-diag stream s */
void CheckVarianceKind(void)
{
   int i,s,m;
   StateElem *se;
   StreamElem *ste;
   MixtureElem *me;
   
   for (s=1;s<=hset.swidth[0];s++)
      fullcNeeded[s]=FALSE;
   for (i=2,se=hmmLink->svec+2; i < hmmLink->numStates; i++,se++)
      for (s=1,ste=se->info->pdf+1; s <= hset.swidth[0]; s++,ste++)
         for (m=1,me = ste->spdf.cpdf+1; m<=ste->nMix; m++, me++)
            if (me->mpdf->ckind == FULLC) 
               fullcNeeded[s] = TRUE;
}

/* Initialise: load HMMs and create accumulators */
void Initialise(void)
{
   int s,V;
   Boolean eSep;
   char base[MAXSTRLEN];
   char path[MAXSTRLEN];
   char ext[MAXSTRLEN];

   /* Load HMM defs */     
   if(MakeOneHMM(&hset,BaseOf(hmmfn,base))<SUCCESS)
      HError(2028,"Initialise: MakeOneHMM failed");
   if(LoadHMMSet(&hset,PathOf(hmmfn,path),ExtnOf(hmmfn,ext))<SUCCESS)
      HError(2028,"Initialise: LoadHMMSet failed");
   SetParmHMMSet(&hset);
   if (hset.hsKind==DISCRETEHS || hset.hsKind==TIEDHS)
      HError(2030,"Initialise: HCompV only uses continuous models");

   /* Create a heap to store the input data */
   CreateHeap(&iStack,"InBuf", MSTAK, 1, 0.5, 100000, LONG_MAX);
   
   /* Get a pointer to the physical HMM */
   hmmId = GetLabId(base,FALSE);
   macroLink = FindMacroName(&hset,'h',hmmId);
   if (macroLink==NULL)
      HError(2020,"Initialise: cannot find hmm %s in hset",hmmfn);
   hmmLink = (HLink)macroLink->structure;

   /* Find out for which streams full covariance is needed */
   CheckVarianceKind( );

   /* Create accumulators for the mean and variance */
   for (s=1;s<=hset.swidth[0]; s++){
      V = hset.swidth[s];
      accs[s].meanSum=CreateVector(&gstack,V);
      ZeroVector(accs[s].meanSum);
      if (fullcNeeded[s]) {
         accs[s].squareSum.inv=CreateSTriMat(&gstack,V);
         accs[s].fixed.inv=CreateSTriMat(&gstack,V);
         ZeroTriMat(accs[s].squareSum.inv);
      }
      else {
         accs[s].squareSum.var=CreateSVector(&gstack,V);
         accs[s].fixed.var=CreateSVector(&gstack,V);
         ZeroVector(accs[s].squareSum.var);
      }
   }

   /* Create an object to hold the input parameters */
   SetStreamWidths(hset.pkind,hset.vecSize,hset.swidth,&eSep);
   obs=MakeObservation(&gstack,hset.swidth,hset.pkind,FALSE,eSep);
   if(segLab != NULL) {
      segId = GetLabId(segLab,TRUE);
   }

   if (trace&T_TOP) {
      printf("Calculating Fixed Variance\n");
      printf("  HMM Prototype: %s\n",hmmfn);
      printf("  Segment Label: %s\n",(segLab==NULL)?"None":segLab);
      printf("  Num Streams  : %d\n",hset.swidth[0]);
      printf("  UpdatingMeans: %s\n",(meanUpdate)?"Yes":"No");
      printf("  Target Direct: %s\n",(outDir==NULL)?"Current":outDir);     
   }
}

/* ----------------------[Co]Variance Estimation ---------------------- */

/* CalcCovs: calculate covariance of speech data */
void CalcCovs(void)
{
   int x,y,s,V;
   float meanx,meany,varxy,n;
   Matrix fullMat;
   
   if (totalCount<2)
      HError(2021,"CalcCovs: Only %d speech frames accumulated",totalCount);
   if (trace&T_TOP)
      printf("%ld speech frames accumulated\n", totalCount);
   n = (float)totalCount;     /* to prevent rounding to integer below */
   for (s=1; s<=hset.swidth[0]; s++){  /* For each stream   */
      V = hset.swidth[s];
      for (x=1; x<=V; x++)            /* For each coefficient ... */
         accs[s].meanSum[x] /= n;         /* ... calculate mean */
      for (x=1;x<=V;x++) {
         meanx = accs[s].meanSum[x];      /* ... and [co]variance */
         if (fullcNeeded[s]) {
            for (y=1; y<=x; y++) {
               meany = accs[s].meanSum[y];
               varxy = accs[s].squareSum.inv[x][y]/n - meanx*meany;
               accs[s].squareSum.inv[x][y] =
                  (x != y || varxy > minVar) ? varxy : minVar;    
            }
         }
         else {
            varxy = accs[s].squareSum.var[x]/n - meanx*meanx;
            accs[s].fixed.var[x] = (varxy > minVar) ? varxy :minVar;
         }
      }
      if (fullcNeeded[s]) { /* invert covariance matrix */
         fullMat=CreateMatrix(&gstack,V,V);
         ZeroMatrix(fullMat); 
         CovInvert(accs[s].squareSum.inv,fullMat);
         Mat2Tri(fullMat,accs[s].fixed.inv);
         FreeMatrix(&gstack,fullMat);
      }
      if (trace&T_COVS) {
         printf("Stream %d\n",s);
         if (meanUpdate)
            ShowVector(" Mean Vector ", accs[s].meanSum,12);
         if (fullcNeeded[s]) {
            ShowTriMat(" Covariance Matrix ",accs[s].squareSum.inv,12,12);
         } else
            ShowVector(" Variance Vector ", accs[s].fixed.var,12);
      }
   }
}

/* TriDiag2Vector: Copy diagonal from m into v */
void TriDiag2Vector(TriMat m, Vector v)
{
   int i,size;

   if (TriMatSize(m) != (size=VectorSize(v)))
      HError(2090,"TriDiag2Vector: Covariance sizes differ %d vs %d",
             TriMatSize(m),VectorSize(v));
   for (i=1; i<=size; i++)
      v[i] = m[i][i];
}

/* SetCovs: set covariance values in hmm */
void SetCovs(void)
{
   int i,s,m;
   StateElem *se;
   StreamElem *ste;
   MixtureElem *me;
   MixPDF *mp;

   CalcCovs();
   if (trace&T_TOP) {
      printf("Updating HMM ");
      if (meanUpdate) printf("Means and ");
      printf("Covariances\n");
   }
   for (i=2,se=hmmLink->svec+2; i < hmmLink->numStates; i++,se++)
      for (s=1,ste=se->info->pdf+1; s <= hset.swidth[0]; s++,ste++)
         for (m=1,me = ste->spdf.cpdf+1; m<=ste->nMix; m++, me++) {
            mp = me->mpdf;
            if (meanUpdate && !IsSeenV(mp->mean)){      /* meanSum now holds mean */
               CopyVector(accs[s].meanSum,mp->mean); 
               TouchV(mp->mean);
            }
            if (!IsSeenV(mp->cov.var)){
               if (mp->ckind==FULLC)
                  CopyMatrix(accs[s].fixed.inv,mp->cov.inv);
               else if (fullcNeeded[s])  /* dont need full cov, but its all we have */                
                  TriDiag2Vector(accs[s].fixed.inv,mp->cov.var);
               else
                  CopyVector(accs[s].fixed.var,mp->cov.var);
               TouchV(mp->cov.var);
            }
         }
   ClearSeenFlags(&hset,CLR_ALL);
}

/* PutVFloor: output variance floor vectors */
void PutVFloor(void)
{
   int i,s;
   char outfn[MAXSTRLEN],vName[32],num[10];
   FILE *f;
   Vector v;
   
   MakeFN("vFloors",outDir,NULL,outfn);
   if ((f = fopen(outfn,"w")) == NULL)
      HError(2011,"PutVFloor: cannot create %s",outfn);
   for (s=1; s <= hset.swidth[0]; s++) {
      v = CreateVector(&gstack,hset.swidth[s]);
      sprintf(num,"%d",s); 
      strcpy(vName,"varFloor"); strcat(vName,num);
      fprintf(f,"~v %s\n",vName);
      if (fullcNeeded[s])              
         TriDiag2Vector(accs[s].squareSum.inv,v);
      else
         CopyVector(accs[s].fixed.var,v);
      for (i=1; i<=hset.swidth[s]; i++)
         v[i] *= vFloorScale;
      fprintf(f,"<Variance> %d\n",hset.swidth[s]);
      WriteVector(f,v,FALSE);
      FreeVector(&gstack,v);
   }
   fclose(f);
   if (trace&T_TOP)
      printf("Var floor macros output to file %s\n",outfn);
}

/* ---------------- Load Data and Accumulate Stats --------------- */

/* AccVar:  update global accumulators with given observation */
void AccVar(Observation obs)
{
   int x,y,s,V;
   float val;
   Vector v;

   totalCount++;
   for (s=1; s<=hset.swidth[0]; s++){
      v = obs.fv[s]; V = hset.swidth[s];
      for (x=1;x<=V;x++) { 
         val=v[x];            
         accs[s].meanSum[x] += val;     /* accumulate mean */                             
         if (fullcNeeded[s]) {          /* accumulate covar */ 
            accs[s].squareSum.inv[x][x] += val*val;
            for (y=1;y<x;y++) 
               accs[s].squareSum.inv[x][y] += val*v[y];
         } else                         /* accumulate var */
            accs[s].squareSum.var[x] += val*val;
      }
   }
}

/* CheckData: check data file consistent with HMM definition */
void CheckData(char *fn, BufferInfo info) 
{
   if (info.tgtVecSize!=hset.vecSize)
      HError(2050,"CheckData: Vector size in %s[%d] is incompatible with hmm %s[%d]",
             fn,info.tgtVecSize,hmmfn,hset.vecSize);
   if (info.tgtPK != hset.pkind)
      HError(2050,"CheckData: Parameterisation in %s is incompatible with hmm %s",
             fn,hmmfn);
}

/* LoadFile: load whole file or segments and accumulate variance */
void LoadFile(char *fn)
{
   ParmBuf pbuf;
   BufferInfo info;
   char labfn[80];
   Transcription *trans;
   long segStIdx,segEnIdx;  
   int i,j,ncas,nObs;
   LLink p;
   
   if (segId == NULL)  {   /* load whole parameter file */
      if((pbuf=OpenBuffer(&iStack, fn, 0, dff, FALSE_dup, FALSE_dup))==NULL)
         HError(2050,"LoadFile: Config parameters invalid");
      GetBufferInfo(pbuf,&info);
      CheckData(fn,info);
      nObs = ObsInBuffer(pbuf);
      for (i=0; i<nObs; i++){
         ReadAsTable(pbuf,i,&obs);
         AccVar(obs);  
      }
      if (trace&T_LOAD) {
         printf(" %d observations loaded from %s\n",nObs,fn);
         fflush(stdout);
      }        
      CloseBuffer(pbuf);
   }
   else {                  /* load segment of parameter file */
      MakeFN(fn,labDir,labExt,labfn);
      trans = LOpen(&iStack,labfn,lff);
      ncas = NumCases(trans->head,segId);
      if ( ncas > 0) {
         if((pbuf=OpenBuffer(&iStack, fn, 0, dff, FALSE_dup, FALSE_dup))==NULL)
            HError(2050,"LoadFile: Config parameters invalid");
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
                  ReadAsTable(pbuf,j,&obs);
                  AccVar(obs); ++nObs;
               }
            }
         }        
         CloseBuffer(pbuf);
         if (trace&T_LOAD)
            printf(" %d observations loaded from %s\n",nObs,fn);
      }  
   }
   ResetHeap(&iStack);
}

/* ------------------------- Save Model ----------------------- */

/* SaveModel: save HMMSet containing one model */
void SaveModel(char *outfn)
{
   if (outfn != NULL)
      macroLink->id = GetLabId(outfn,TRUE);
   if(SaveHMMSet(&hset,outDir,NULL,NULL,saveBinary)<SUCCESS)
      HError(2011,"SaveModel: SaveHMMSet failed");
}



/* ------------- Cepstral Mean Substraction & Variance Normalisation ------------ */

/* initialise and return an instance of SpkrAcc */  
SpkrAcc *InitSpkrAcc(void)
{
   SpkrAcc *sa;

   sa = New(&gstack,sizeof(SpkrAcc));
   sa->meanSum = CreateVector(&gstack,vSize);
   ZeroVector(sa->meanSum);
   sa->squareSum = CreateVector(&gstack,vSize);
   ZeroVector(sa->squareSum);
   sa->NumFrame = 0;
   return sa;
}

/* reset an instance of SpkrAcc type */ 
void ClrSpkrAcc(SpkrAcc *sa)
{
   ZeroVector(sa->meanSum);
   ZeroVector(sa->squareSum);
   sa->NumFrame = 0;
}
  
/* Accumulate stats from an utterance file */
SpkrAcc *AccGenUtt(char *SpkrPattern, char *UttFileName, SpkrAcc *sa)
{
   char SpkrName[MAXSTRLEN];
   ParmBuf pbuf;
   BufferInfo info;
   short swidth[SMAX];
   Boolean eSep;
   Vector tempV;
   int i;

   if (MaskMatch(SpkrPattern,SpkrName,UttFileName)==TRUE){
      /* open buffer and construct observation */
      pbuf = OpenBuffer(&iStack,UttFileName,0,dff,FALSE_dup,FALSE_dup);
      GetBufferInfo(pbuf,&info);
      if ((info.tgtPK & HASZEROM) && strchr(oflags,'m')) {
         HError(-2021,"HCompV: AccGenUtt: qualifier _Z not appropriate when calculating means!\n");
      }
      /* treat as single stream system though a bit weird */
      ZeroStreamWidths(1,swidth);
      SetStreamWidths(info.tgtPK,info.tgtVecSize,swidth,&eSep);
      obs = MakeObservation(&gstack,swidth,info.tgtPK,FALSE,eSep);
      if (info.tgtVecSize != vSize){
         vSize = info.tgtVecSize;
         /* if needed init a SpkrAcc */
         sa = InitSpkrAcc();
         fprintf(stdout,"Target observation vector size set to %d ......\n",info.tgtVecSize);
         fflush(stdout);
      }
      ParmKind2Str(info.tgtPK,TargetPKStr);
      /* accumulate stats for current utterance file */
      StartBuffer(pbuf);
      while (BufferStatus(pbuf) != PB_CLEARED) 
         {
            /* copy current observation and set vector ptr to first stream */
            ReadAsBuffer(pbuf,&obs);
            tempV = obs.fv[1];
            for (i=1;i<=vSize;i++){
               sa->meanSum[i] += tempV[i];
               sa->squareSum[i] += tempV[i]*tempV[i];
            }
            sa->NumFrame += 1;
         }
      CloseBuffer(pbuf);
      strcpy(sa->SpkrName,SpkrName);
      if (trace&T_CMV){
         fprintf(stdout,"Utterance %s accumulate generated for speaker %s\n",UttFileName,sa->SpkrName);
         fflush(stdout);
      }
      ResetHeap(&iStack);
      return sa;
   }
   else {
      HError(2039,"HCompV: AccGenUtt: speaker pattern matching failure on file: %s\n",UttFileName);
      return NULL;
   }
}


/* Append speaker accumulate structure list */
SpkrAccListItem *AppendSpkrAccList(SpkrAccListItem *sal, SpkrAcc *sa)
{
   SpkrAccListItem *temp;
   int i;

   temp = New(&gstack,sizeof(SpkrAccListItem));
   temp->sa = InitSpkrAcc();
   for (i=1;i<=vSize;i++){
      temp->sa->meanSum[i] = sa->meanSum[i];
      temp->sa->squareSum[i] = sa->squareSum[i];
   }
   temp->sa->NumFrame = sa->NumFrame;
   strcpy(temp->sa->SpkrName,sa->SpkrName);
   temp->nextSpkr = sal;
   sal = temp;
  
   if (trace&T_CMV){
      fprintf(stdout,"Creating entry for speaker %s ......\n",sa->SpkrName);
      fflush(stdout);
   }

   return sal;
}

/* 
   search the speaker accumulate list and if there is an entry for the
   current speaker given in sa, then update the entry in the list; otherwise
   insert sa into the list as a new speaker entry
*/
SpkrAccListItem *UpdateSpkrAccList(SpkrAccListItem *sal, SpkrAcc *sa)
{ 
   SpkrAccListItem *p;
   int i;
   Boolean tag = FALSE;

   p = sal;

   while (p != NULL){
      if (strcmp(p->sa->SpkrName,sa->SpkrName)==0){
         for (i=1;i<=vSize;i++){
            p->sa->meanSum[i] += sa->meanSum[i];
            p->sa->squareSum[i] += sa->squareSum[i];
         }
         p->sa->NumFrame += sa->NumFrame;
         tag = TRUE;
         break;
      }
      p = p->nextSpkr;
   }
   if (tag == FALSE){
      sal = AppendSpkrAccList(sal,sa);
   }

   return sal;
}


/* Compute Mean and Var */
void UpdateMeanVar(SpkrAccListItem *sal)
{
   int i;
   SpkrAccListItem *p;
  
   p = sal;
   while (p != NULL){
      for (i=1;i<=vSize;i++){
         p->sa->meanSum[i] /= ((float)(p->sa->NumFrame));
      }
      for (i=1;i<=vSize;i++){
         p->sa->squareSum[i] /= ((float)(p->sa->NumFrame));
         p->sa->squareSum[i] -= (p->sa->meanSum[i])*(p->sa->meanSum[i]);
      }
      p = p->nextSpkr;
   }  
}

/* Report output type NumFrame/cms/cvn */
void ReportOutput()
{
   if (strcmp(oflags,"m")==0){
      fprintf(stdout,"\n  HCompV: Comptuting side based cepstral mean ......\n\n");
   }
   else if (strcmp(oflags,"v")==0){
      fprintf(stdout,"\n  HCompV: Computing side based cepstral variance normaliser ......\n\n");
   }
   else if (strcmp(oflags,"mv")==0){
      fprintf(stdout,"\n  HCompV: Comptuting side based cepstral mean & variance normaliser ......\n\n");
   }
   else if (strcmp(oflags,"nv")==0){
      fprintf(stdout,"\n  HCompV: Comptuting side based frame number & variance normaliser ......\n\n");
   }
   else if (strcmp(oflags,"nmv")==0){
      fprintf(stdout,"\n  HCompV: Comptuting side based frame number & cepstral mean & variance normaliser ......\n\n");
   }
   else {
      fprintf(stdout,"\n  HCompV: ReportOutput: Unrecognisable output flag setting: %s\n",oflags);
      fflush(stdout);
      HError(2019,"HCompV: ReportOutput: Unrecognisable output flag setting: %s\n",oflags);
   }
   fflush(stdout);
}


/* Export number of frames, mean, var to a given directory */
void ExportNMV(SpkrAccListItem *sal, char *OutDirName, char *tgtPKStr) 
{
   FILE *oFile;
   Boolean isPipe;
   char oFileName[MAXSTRLEN];
   char pathBuffer1[MAXSTRLEN];
   char pathBuffer2[MAXSTRLEN];
   SpkrAccListItem *p;
   int i;

   p = sal;
   while(p != NULL){
      /* create output file name for current spkr index */    
      if ( pathPattern[0] != '\0'){
         if ( MaskMatch(pathPattern,pathBuffer1,p->sa->SpkrName) != TRUE ){
            HError(2039,"HCompV: ExportNMV: path pattern matching failure on speaker: %s\n",p->sa->SpkrName);
         }
         MakeFN(pathBuffer1,OutDirName,NULL,pathBuffer2); 
         MakeFN(p->sa->SpkrName,pathBuffer2,NULL,oFileName);
      }
      else
         MakeFN(p->sa->SpkrName,OutDirName,NULL,oFileName);

      /* open and write */
      oFile = FOpen(oFileName,NoOFilter,&isPipe);
      if (oFile == NULL){
         HError(2011,"HCompV: ExportNMV: output file creation error %s",oFileName);
      }
    
      /* write header */
      fprintf(oFile,"<CEPSNORM> <%s>",tgtPKStr);
    
      /* write number frames */
      if (strchr(oflags,'n')){
         fprintf(oFile,"\n<NFRAMES> %d",p->sa->NumFrame);
      }
      /* write mean */
      if (strchr(oflags,'m')){
         fprintf(oFile,"\n<MEAN> %d\n",vSize);
         for (i=1;i<=vSize;i++){
            fprintf(oFile," %e",(p->sa->meanSum[i]));
         }
      }
      /* write variance */
      if (strchr(oflags,'v')){   
         fprintf(oFile,"\n<VARIANCE> %d\n",vSize);
         for (i=1;i<=vSize;i++){
            fprintf(oFile," %e",(p->sa->squareSum[i]));
         }
      }
      fprintf(oFile,"\n");
      FClose(oFile,isPipe);
      p = p->nextSpkr;
   }
}

/* main func */
int main(int argc, char *argv[])
{
   char *datafn, *s;
   void Initialise(void);
   void LoadFile(char *fn);
   void SetCovs(void);
   void PutVFloor(void);
   void SaveModel(char *outfn);
   SpkrAcc *sa = NULL;

   if(InitShell(argc,argv,hcompv_version,hcompv_vc_id)<SUCCESS)
      HError(2000,"HCompV: InitShell failed");
   InitMem();   InitLabel();
   InitMath();  InitSigP();
   InitWave();  InitAudio();
   InitVQ();    InitModel();
   if(InitParm()<SUCCESS)  
      HError(2000,"HCompV: InitParm failed");

   if (!InfoPrinted() && NumArgs() == 0)
      ReportUsage();
   if (NumArgs() == 0) Exit(0);
   SetConfParms();

   CreateHMMSet(&hset,&gstack,FALSE);
   pathPattern[0]='\0';
   while (NextArg() == SWITCHARG) {
      s = GetSwtArg();
      if (strlen(s)!=1) 
         HError(2019,"HCompV: Bad switch %s; must be single letter",s);
      switch(s[0]){
      case 'f':
         if (NextArg() != FLOATARG)
            HError(2019,"HCompV: Variance floor scale expected");
         vFloorScale = GetChkedFlt(0.0,100.0,s);
         break;
      case 'l':
         if (NextArg() != STRINGARG)
            HError(2019,"HCompV: Segment label expected");
         segLab = GetStrArg();
         break;
      case 'm':
         meanUpdate = TRUE;
         break;
      case 'o':
         outfn = GetStrArg();
         break;     
      case 'v':
         if (NextArg() != FLOATARG)
            HError(2019,"HCompV: Minimum variance level expected");
         minVar = GetChkedFlt(0.0,100.0,s);
         break;
      case 'k':
         if (NextArg() != STRINGARG)
            HError(2019,"HCompV: speaker pattern expected");
         strcpy(spPattern,GetStrArg());
         if (strchr(spPattern,'%')==NULL)
            HError(2019,"HCompV: Speaker mask invalid");
         break;
      case 'c':
         if (NextArg() != STRINGARG)
            HError(2019,"HCompV: CMV output dir expected");
         strcpy(cmDir,GetStrArg());
         DoCMV = TRUE;
         break;
      case 'p':
         if (NextArg() != STRINGARG)
            HError(2019,"HCompV: path pattern expected");
         strcpy(pathPattern,GetStrArg());
         if (strchr(pathPattern,'%')==NULL)
            HError(2019,"HCompV: Path mask invalid");
         break;
      case 'q':
         if (NextArg() != STRINGARG)
            HError(2019,"HCompV: output flags (nmv)");
         strcpy(oflags,GetStrArg());
         break;
      case 'B':
         saveBinary = TRUE;
         break;
      case 'F':
         if (NextArg() != STRINGARG)
            HError(2019,"HCompV: Data File format expected");
         if((dff = Str2Format(GetStrArg())) == ALIEN)
            HError(-2089,"HCompV: Warning ALIEN Data file format set");
         break;
      case 'G':
         if (NextArg() != STRINGARG)
            HError(2019,"HCompV: Label File format expected");
         if((lff = Str2Format(GetStrArg())) == ALIEN)
            HError(-2089,"HCompV: Warning ALIEN Label file format set");
         break;
      case 'H':
         if (NextArg() != STRINGARG)
            HError(2019,"HCompV: HMM macro file name expected");
         AddMMF(&hset,GetStrArg());
         break;
      case 'I':
         if (NextArg() != STRINGARG)
            HError(2019,"HCompV: MLF file name expected");
         LoadMasterFile(GetStrArg());
         break;
      case 'L':
         if (NextArg()!=STRINGARG)
            HError(2019,"HCompV: Label file directory expected");
         labDir = GetStrArg();
         break;
      case 'M':
         if (NextArg()!=STRINGARG)
            HError(2019,"HCompV: Output macro file directory expected");
         outDir = GetStrArg();
         break;
      case 'T':
         if (NextArg() != INTARG)
            HError(2019,"HCompV: Trace value expected");
         trace = GetChkedInt(0,077,s); 
         break;
      case 'X':
         if (NextArg()!=STRINGARG)
            HError(2019,"HCompV: Label file extension expected");
         labExt = GetStrArg();
         break;
      default:
         HError(2019,"HCompV: Unknown switch %s",s);
      }
   }

   /* if not doing CMV, do standard HCompV */
   if (DoCMV == FALSE){
      if (NextArg()!=STRINGARG)
         HError(2019,"HCompV: Source HMM file name expected");
      hmmfn = GetStrArg();
      Initialise();
      do {
         if (NextArg()!=STRINGARG)
            HError(2019,"HCompV: Training data file name expected");
         datafn = GetStrArg();
         LoadFile(datafn);
      } while (NumArgs()>0);
      SetCovs();
      FixGConsts(hmmLink);
      SaveModel(outfn);   
      if (trace&T_TOP)
         printf("Output written to directory %s\n",(outDir==NULL)?"./":outDir);
      if (vFloorScale>0.0)
         PutVFloor();
   }
   else {
      /* report export data type */
      ReportOutput();
      /* init input buffer mem heap */
      CreateHeap(&iStack,"BufferIn",MSTAK,1,0.5,100000,5000000);
      do {
         if (NextArg()!=STRINGARG){
            HError(2019,"HCompV: Training data file name expected");
         }
         datafn = GetStrArg();       
         /* accumulate stats for current utterance file and update speaker list */
         sa = AccGenUtt(spPattern,datafn,sa);
         salist = UpdateSpkrAccList(salist,sa);
         /* reset for next utterance */
         ClrSpkrAcc(sa);
      } while (NumArgs()>0);
      /* compute the means and variances for each speaker */
      UpdateMeanVar(salist);
      /* export NMV for each speaker */
      ExportNMV(salist,cmDir,TargetPKStr);
   }

   Exit(0);
   return (0);          /* never reached -- make compiler happy */
}

/* ----------------------------------------------------------- */
/*                      END:  HCompV.c                         */
/* ----------------------------------------------------------- */
