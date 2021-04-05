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
/*         File: HTrain.c   HMM Training Support Routines      */
/* ----------------------------------------------------------- */

char *htrain_version = "!HVER!HTrain:   3.4.1 [CUED 12/03/09]";
char *htrain_vc_id = "$Id: HTrain.c,v 1.1.1.1 2006/10/11 09:54:58 jal58 Exp $";

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
#include "HTrain.h"

/* --------------------------- Trace Flags ------------------------- */

#define T_TOP  00001       /* Top Level tracing */
#define T_SEQ  00002       /* Trace sequence operations */
#define T_CGE  00004       /* General cluster tracing */
#define T_CLC  00010       /* Trace cluster convergence */
#define T_DCM  00020       /* Dump cluster map after each cycle */
#define T_CAL  00040       /* Trace item to cluster allocation */
#define T_CDI  00100       /* Trace cluster distance calc */
#define T_NAC  00200       /* Count accs created */
#define T_ALD  00400       /* Trace acc load/dump */

static int trace = 0;

/* --------------------------- Initialisation ---------------------- */

static ConfParam *cParm[MAXGLOBS];      /* config parameters */
static int nParm = 0;
static int maxIter = 10;               /* max num cluster iterations */
static int minClustSize = 3;           /* min num vectors in cluster */
static Boolean ldBinary = TRUE;        /* load/dump in binary */

Boolean strmProj = FALSE; 

#define DoPreComps(hsKind) (hsKind==SHAREDHS||hsKind==PLAINHS)

/* EXPORT->InitTrain: initialise configuration parameters */
void InitTrain(void)
{
   int i;
   Boolean b;

   Register(htrain_version,htrain_vc_id);
   nParm = GetConfig("HTRAIN", TRUE, cParm, MAXGLOBS);
   if (nParm>0){
      if (GetConfInt(cParm,nParm,"TRACE",&i)) trace = i;
      if (GetConfInt(cParm,nParm,"MAXCLUSTITER",&i)) maxIter = i;
      if (GetConfInt(cParm,nParm,"MINCLUSTSIZE",&i)) minClustSize = i;
      if (GetConfBool(cParm,nParm,"BINARYACCFORMAT",&b)) ldBinary = b;
      if (GetConfBool(cParm,nParm,"STREAMPROJECTION",&b)) strmProj = b;
   }
}

/* -------------------- Generic Sequence Type ------------------- */

/*
  Items are stored within chained lists of blocks.  Items are 
  normally added to the end of the last block.  If the last
  block is full, then the item is inserted into the first block
  with a free item slot.  If all blocks are full, then a new
  block is created and chained to the end of the list.
*/
   
/* NewItemBlock: create and return an emptry ItemBlock */
static IBLink NewItemBlock(MemHeap *x, int blkSize)
{
   IBLink b;
   
   if (trace&T_SEQ)
      printf("HTrain: creating item block, %d items on stack %s\n",
             blkSize,x->name);
   b = (IBLink)New(x,sizeof(ItemBlock));
   b->used = 0; b->next = NULL;
   b->items = (Ptr *)New(x,sizeof(Ptr)*blkSize);
   return b;
}

/* EXPORT->CreateSequence: create and return an empty sequence */
Sequence CreateSequence(MemHeap *x, int blkSize)
{
   Sequence s;
   
   s = (Sequence)New(x,sizeof(SequenceInfo));
   s->mem = x;    s->nItems = 0;
   s->nFree = s->blkSize = blkSize;
   s->hd = s->tl = NewItemBlock(x,blkSize);
   return s;
}

/* EXPORT->StoreItem: store item in seq, extend if needed */
void StoreItem(Sequence seq, Ptr item)
{
   IBLink b;

   if (seq->nFree==0){     /* add a block */
      b = NewItemBlock(seq->mem,seq->blkSize);
      seq->tl->next = b; seq->tl = b;
      seq->nFree = seq->blkSize;
   }
   if (seq->tl->used < seq->blkSize) { /* try last block first */
      b = seq->tl;
      b->items[b->used++] = item;
   }else{   /* otherwise search blocks for empty slot */
      for (b = seq->hd; b->used == seq->blkSize; b = b->next)
         if (b==NULL)
            HError(7190,"StoreItem: free slot missing");
      b->items[b->used++] = item;
   }
   --seq->nFree; ++seq->nItems;
}

/* FindItemBlock: find block containing i'th item, j is index in block */
static IBLink FindItemBlock(Sequence seq, int i, int *j)
{
   IBLink b;
   
   if (i > seq->nItems)
      HError(7171,"FindItemBlock: %d'th item from %d requested",
             i,seq->nItems);
   b = seq->hd;
   while (i>b->used){
      i -= b->used; b = b->next;
   }
   *j = i-1;
   return b;
}

/* EXPORT->DeleteItem: delete i'th item */
void DeleteItem(Sequence seq, int i)
{
   IBLink b;
   int j,k;
   
   b = FindItemBlock(seq,i,&j);
   --b->used;
   for (k=j; k<b->used; k++)
      b->items[k] = b->items[k+1];
   ++seq->nFree; --seq->nItems;
}

/* EXPORT->GetItem: return ptr to the i'th item in sequence */
Ptr GetItem(Sequence seq, int i)
{
   IBLink b;
   int j;
   
   b = FindItemBlock(seq,i,&j);
   return b->items[j];  
}

/* ------------ Segment (Observation Sequence) Storage ------------ */

/* EXPORT->CreateSegStore: Create and return an empty segment store
   suitable for observations of form obs. */
SegStore CreateSegStore(MemHeap *x, Observation obs, int segLen)
{
   SegStore ss;
   
   ss = (SegStore)New(x,sizeof(SegStoreRec));
   ss->mem = x;
   ss->o = obs; ss->segLen = segLen;
   ss->hasfv = (obs.pk&BASEMASK) != DISCRETE;
   ss->hasvq = (obs.pk&HASVQ)  || (obs.pk&BASEMASK) == DISCRETE;
   if (ss->hasfv) ss->fvSegs = CreateSequence(x,100);
   if (ss->hasvq) ss->vqSegs = CreateSequence(x,100);
   return ss;
}

/* EXPORT->LoadSegment: of obs in pbuf from start to end */
void LoadSegment(SegStore ss, HTime start, HTime end, ParmBuf pbuf)
{
   Sequence vq = NULL;
   Sequence fv = NULL;
   BufferInfo info;
   long i,st,en,len;
   int s,S = ss->o.swidth[0];
   short *vqItem;
   Vector *fvItem;
   
   GetBufferInfo(pbuf,&info);
   st = (long)(start/info.tgtSampRate);
   en = (long)(end/info.tgtSampRate);
   len = ObsInBuffer(pbuf);
   if (en >= len) en = len-1;
   if (st > en) {
      HError(-7173,"LoadSegment: empty segment");
      return;
   }
   if (ss->hasvq) vq = CreateSequence(ss->mem,ss->segLen);
   if (ss->hasfv) fv = CreateSequence(ss->mem,ss->segLen);
   for (i=st; i<=en; i++) {
      if (ss->hasfv) { /* put new vectors in ss->o */
         for (s=1; s<=S; s++) 
            ss->o.fv[s] = CreateVector(ss->mem,ss->o.swidth[s]);        
      }
      ReadAsTable(pbuf,i,&(ss->o));
      if (ss->hasvq) {
         vqItem = (short *)New(ss->mem,sizeof(short)*(S+1));
         for (s=1; s<=S; s++) 
            vqItem[s] = ss->o.vq[s];
         StoreItem(vq, (Ptr) vqItem);
      }
      if (ss->hasfv) {
         fvItem = (Vector *)New(ss->mem,sizeof(Vector)*(S+1));
         for (s=1; s<=S; s++) 
            fvItem[s] = ss->o.fv[s];
         StoreItem(fv, (Ptr) fvItem);
      }
   }
   if (ss->hasvq) StoreItem(ss->vqSegs, (Ptr)vq);
   if (ss->hasfv) StoreItem(ss->fvSegs, (Ptr)fv);
}

/* EXPORT->SegLength: Return num obs in i'th segment */
int SegLength(SegStore ss, int i)
{
   Sequence s = NULL;
   
   if (ss->hasfv) 
      s = (Sequence) GetItem(ss->fvSegs,i);
   else if (ss->hasvq)
      s = (Sequence) GetItem(ss->vqSegs,i);
   else
      HError(7191,"SegLength: no fv or vq seg stored");
   return s->nItems;
}

/* EXPORT->NumSegs: Return num segments in ss */
int NumSegs(SegStore ss)
{
   if (ss->hasfv) 
      return ss->fvSegs->nItems;
   else if (ss->hasvq)
      return ss->vqSegs->nItems;
   else
      HError(7191,"NumSegs: no fv or vq segs stored");
   return 0;  /* to keep compiler happy */
}

/* EXPORT->GetSegObs: Return j'th observation from i'th segment */
Observation GetSegObs(SegStore ss, int i, int j)
{
   Sequence vq;
   Sequence fv;
   int s,S = ss->o.swidth[0];
   short *vqItem;
   Vector *fvItem;
   
   if (ss->hasvq) {
      vq = (Sequence) GetItem(ss->vqSegs,i);
      vqItem = (short *) GetItem(vq,j);
      for (s=1; s<=S; s++) 
         ss->o.vq[s] = vqItem[s];
   }     
   if (ss->hasfv) {
      fv = (Sequence) GetItem(ss->fvSegs,i);
      fvItem = (Vector *) GetItem(fv,j);
      for (s=1; s<=S; s++) 
         ss->o.fv[s] = fvItem[s];
   }     
   return ss->o;
}


/* EXPORT->SequenceMean: compute mean of vectors in ss */
void SequenceMean(Sequence ss, Vector mean)
{
   Vector v;
   int i,j,size;
   double n;
   DVector sum;

   size = VectorSize(mean);
   sum = CreateDVector(&gstack,size);
   ZeroDVector(sum);
   n = ss->nItems;
   for (i=1; i<=ss->nItems; i++) {
      v = (Vector) GetItem(ss,i);
      for (j=1; j<=size; j++)
         sum[j] += v[j];
   }
   for (j=1; j<=size; j++)
      mean[j] = sum[j]/n;
   FreeDVector(&gstack,sum);
}

/* EXPORT->SequenceCov: compute covariance of vectors in ss */
void SequenceCov(Sequence ss, CovKind ck, Covariance cov, Vector mean)
{
   Vector v;
   int i,j,k,size;
   double n,x,y;
   DVector sqsum;
   DMatrix xsum;
   
   switch(ck){
   case DIAGC:       /* diagonal covariance matrix */
   case INVDIAGC:    /* inverse diag covariance matrix */
      size = VectorSize(cov.var);
      sqsum = CreateDVector(&gstack,size);
      ZeroDVector(sqsum);
      n = ss->nItems;
      for (i=1; i<=ss->nItems; i++) {
         v = (Vector) GetItem(ss,i);
         for (j=1; j<=size; j++){
            x = v[j]-mean[j];
            sqsum[j] += x*x;
         }
      }
      for (j=1; j<=size; j++)
         cov.var[j] = (ck==DIAGC)?sqsum[j]/n:n/sqsum[j];
      FreeDVector(&gstack,sqsum);
      break;
   case FULLC:    /* inverse full covariance matrix */
      size = TriMatSize(cov.inv);
      xsum = CreateDMatrix(&gstack,size,size);
      ZeroDMatrix(xsum);
      n = ss->nItems;
      for (i=1; i<=ss->nItems; i++) {
         v = (Vector) GetItem(ss,i);
         for (j=1; j<=size; j++)
            for (k=1; k<=j; k++){
               x = v[j]-mean[j];
               y = v[k]-mean[k];
               xsum[j][k] += x*y;
            }
      }
      for (j=1; j<=size; j++)
         for (k=1; k<=j; k++)
            cov.inv[j][k] = xsum[j][k]/n;
      CovInvert(cov.inv,cov.inv);
      FreeDMatrix(&gstack,xsum);
      break;
   default:
      HError(7170,"SequenceCov: unsupported cov kind [%d]",ck);
   }
}

/* --------------------- Vector Clustering -------------------- */

/* 
   The routines in this section implement a top-down clustering
   algorithm.  There are two clustering modes: linear and tree
   based.  Both are top-down and start from a single cluster.
   
   In the default linear mode, the largest cluster is
   successively split until the required number of clusters are
   formed.  Each cluster centre is the average of all vectors in the
   cluster.  Splitting is performed by perturbing the cluster centre to
   form two cluster centres.  The largest cluster is defined as that
   which has the largest average construction cost.
   
   In tree clustering mode, the current leaves are split regardless
   of their size.
   
   Linear mode is best for clustering small data sets (eg as in HInit).
   Either mode can be used for large data sets (eg as in HQuant).
*/

static int curNumCl;    /* num clust currently created */
static int nItems;      /* num items in pool */
static ClusterSet *ccs; /* current cluster set */
static Sequence cvp;    /* current vector pool being clustered */
static ShortVec cmap;   /* array[1..cvp->nItems]of cluster index */
static CovKind dck;     /* defines type of distance calc */
static Covariance dcov; /* covariance to use in distance calc */
static int vSize;       /* size of vectors */
static Vector vTmp;     /* temp vector */

/* DumpClusterMap: dump the vector pool -> cluster map */
static void DumpClusterMap(void)
{
   int i,lc=0;
   
   for (i=1; i<=nItems; i++) {
      printf("%3d",cmap[i]);
      if (++lc == 20) { printf("\n"); lc = 0; }
   }
   if (lc != 0) printf("\n");
   fflush(stdout);
}

/* Distance: compute distance between v1 and v2 */
static float Distance(Vector v1, Vector v2)
{
   Vector iv,crow;
   TriMat ic;
   double sum=0.0,x;
   int i,j;

   switch(dck){
   case NULLC:
      sum = 0.0;
      for (i=1; i<=vSize; i++){
         x = v1[i]-v2[i]; sum += x*x;
      }
      break;
   case DIAGC:
      iv = dcov.var;  /* covkind == DIAGC */
      sum = 0.0;
      for (i=1; i<=vSize; i++){
         x = v1[i]-v2[i]; sum += x*x/iv[i];
      }
      break;
   case INVDIAGC:
      iv = dcov.var;  /* covkind == INVDIAGC */
      sum = 0.0;
      for (i=1; i<=vSize; i++){
         x = v1[i]-v2[i]; sum += x*x*iv[i];
      }
      break;
   case FULLC:
      ic = dcov.inv; /* covkind == FULLC */
      for (i=1;i<=vSize;i++)
         vTmp[i] = v1[i] - v2[i];
      sum = 0.0;
      for (i=2;i<=vSize;i++) {
         crow = ic[i];
         for (j=1; j<i; j++)
            sum += vTmp[i]*vTmp[j]*crow[j];
      }
      sum *= 2;
      for (i=1;i<=vSize;i++)
         sum += vTmp[i] * vTmp[i] * ic[i][i];
      break;
   default:
      HError(7170,"Distance: bad cov kind %d",dck);
   }
   if (trace&T_CDI) {
      ShowVector("   dvec 1",v1,10);
      ShowVector("   dvec 2",v2,10);
      printf("   distance = %f\n",sqrt(sum));
   }
   return sqrt(sum);
}

/* AllocateVectors: distribute all pool vectors amongst clusters
   1..curNumCl.  The centres of these clusters have already set.  Each
   vector is placed in cluster with nearest centre.  Also, sets
   aveCost of each cluster and returns totalCost.  Index of any cluster
   with < minClustSize elements is returned, 0 otherwise */
static int AllocateVectors(float *totalCost)
{
   int n, i, bestn, cs;
   float d,min;
   Vector v;

   if (trace&T_CAL)
      printf("  allocating pool amongst %d clusters\n",curNumCl);
   /* zero all clusters */
   *totalCost=0.0;
   for (n=1; n<=curNumCl; n++){
      ccs->cl[n].aveCost = 0.0;
      ccs->cl[n].csize = 0;
   }
   /* scan pool of vectors */
   for (i=1; i<=nItems; i++) {
      v = (Vector)GetItem(cvp,i);
      /* find centre nearest to i'th vector */ 
      min=Distance(v,ccs->cl[1].vCtr); bestn = 1;
      for (n=2; n<=curNumCl; n++) {
         d = Distance(v,ccs->cl[n].vCtr);
         if (d < min) {
            min = d; bestn = n;
         }
      }
      if (trace&T_CAL)
         printf("   item %d -> cluster %d, cost = %f\n",i,bestn,min);
      /* increment costs and allocate vector to bestn */
      *totalCost += min;   
      ccs->cl[bestn].aveCost += min;
      cmap[i] = bestn; ++ccs->cl[bestn].csize;
   }
   /* Check for any empty clusters and average costs */
   for (n=1; n<=curNumCl; n++) {
      cs = ccs->cl[n].csize;
      if (cs < minClustSize) {
         if (trace&T_CAL)
            printf("   cluster %d empty\n",n);
         return n;
      }
      ccs->cl[n].aveCost /= cs;
   }
   return 0;
}

/* SplitVectors: distribute all pool vectors in cluster n between
   clusters n1 and n2.  The centres of these clusters have already
   set.  Each vector is placed in cluster with nearest centre.   */
static void SplitVectors(int n, int n1, int n2)
{
   int i, bestn;
   float d1,d2;
   Vector v;
   int c,c1,c2;

   if (trace&T_CAL)
      printf("  SplitVectors: ");
   for (i=1; i<=nItems; i++)
      if (cmap[i]==n) {
         v = (Vector)GetItem(cvp,i);
         /* find centre nearest to i'th vector */ 
         d1=Distance(v,ccs->cl[n1].vCtr); 
         d2=Distance(v,ccs->cl[n2].vCtr); 
         bestn = (d1<d2)?n1:n2;
         /* allocate vector to bestn */
         cmap[i] = bestn; ++ccs->cl[bestn].csize;
      }
   /* Check for any empty clusters and average costs */
   c = ccs->cl[n].csize;
   c1 = ccs->cl[n1].csize;
   c2 = ccs->cl[n2].csize;
   if (trace&T_CAL)
      printf("  clusters split %d[%d] ->%d[%d] + %d[%d]\n",
             n,c,n1,c1,n2,c2);   
   if (c1 == 0 || c2 == 0)
      HError(7120,"SplitVectors: empty cluster %d[%d] ->%d[%d] + %d[%d]",
             n,c,n1,c1,n2,c2);
}

/* FindCentres: of clusters a..b */
static void FindCentres(int a, int b)
{
   int n,i,j,cidx;
   Vector v,ctr;
   float cs;
   
   for (n=a; n<=b; n++)
      ZeroVector(ccs->cl[n].vCtr);
   for (i=1; i<=nItems; i++) {
      cidx = cmap[i];
      if (cidx>=a && cidx<=b){
         v = (Vector)GetItem(cvp,i);
         ctr = ccs->cl[cidx].vCtr;
         for (j=1; j<=vSize; j++)
            ctr[j] += v[j];
      }
   }
   for (n=a; n<=b; n++){
      ctr = ccs->cl[n].vCtr;
      cs = ccs->cl[n].csize;
      for (j=1; j<=vSize; j++) ctr[j] /= cs;
   }
}

/* FindCovariance: of cluster n */
static void FindCovariance(int n)
{
   Vector v,mean;
   TriMat t;
   int i,j,k;
   double nx,x,y;
   DVector sqsum;
   DMatrix xsum;
   Matrix c;
   
   nx = ccs->cl[n].csize;
   mean = ccs->cl[n].vCtr;
   switch(ccs->ck){
   case NULLC:
      ccs->cl[n].cov.var = NULL;
      break;
   case DIAGC:       /* diagonal covariance matrix */
   case INVDIAGC:    /* inverse diag covariance matrix */
      sqsum = CreateDVector(&gstack,vSize);
      ZeroDVector(sqsum);
      for (i=1; i<=nItems; i++) {
         if (cmap[i] == n) {
            v = (Vector) GetItem(cvp,i);
            for (j=1; j<=vSize; j++){
               x = v[j]-mean[j];
               sqsum[j] += x*x;
            }
         }
      }
      v = ccs->cl[n].cov.var;
      for (j=1; j<=vSize; j++)
         v[j] = (ccs->ck==DIAGC)?sqsum[j]/nx:nx/sqsum[j];
      FreeDVector(&gstack,sqsum);
      break;
   case FULLC:    /* inverse full covariance matrix */
      xsum = CreateDMatrix(&gstack,vSize,vSize);
      c = CreateMatrix(&gstack,vSize,vSize);
      ZeroDMatrix(xsum);
      for (i=1; i<=nItems; i++) {
         if (cmap[i] == n) {
            v = (Vector) GetItem(cvp,i);
            for (j=1; j<=vSize; j++)
               for (k=1; k<=j; k++){
                  x = v[j]-mean[j];
                  y = v[k]-mean[k];
                  xsum[j][k] += x*y;
               }
         }
      }
      t = ccs->cl[n].cov.inv;
      for (j=1; j<=vSize; j++)
         for (k=1; k<=j; k++)
            t[j][k] = xsum[j][k]/nx;
      CovInvert(t,c); Mat2Tri(c,t);
      FreeDMatrix(&gstack,xsum);
      break;
   default:
      HError(7170,"FindCovariance: unsupported cov kind [%d]",ccs->ck);
   }
}

/* BiggestCluster: return index of cluster with highest average cost */
static int BiggestCluster(void)
{
   float maxCost;
   int n,biggest;
   
   maxCost=ccs->cl[1].aveCost; biggest = 1;
   for (n=2; n<=curNumCl; n++)
      if (ccs->cl[n].aveCost > maxCost) {
         maxCost=ccs->cl[n].aveCost;
         if( ccs->cl[n].csize >= minClustSize )
            biggest=n;
      }
   return biggest;
}

/* FullestCluster: return index of cluster with most elements */
static int FullestCluster(void)
{
   int max,n,fullest;
   
   max=ccs->cl[1].csize; fullest = 1;
   for (n=1; n<=curNumCl; n++)
      if (ccs->cl[n].csize > max) {
         max=ccs->cl[n].csize; fullest=n;
      }
   return fullest;
}

/* Perturb: copy perturbed versions of cluster n into n1 and n2. */
static void Perturb(int n, int n1, int n2)
{
   int i;
   Vector v,v1,v2;
   float x;
   
   if (trace&T_CGE)
      printf("  Perturb: cluster %d -> %d + %d\n",n,n1,n2);
   v  = ccs->cl[n].vCtr;
   v1 = ccs->cl[n1].vCtr;
   v2 = ccs->cl[n2].vCtr;
   if (n!=n1) CopyVector(v,v1);
   if (n!=n2) CopyVector(v,v2);  
   for (i=1; i<=vSize; i++) {
      x = fabs(v[i]*0.01);
      if (x<0.0001) x=0.0001;
      v1[i] += x; v2[i] -= x;
   }
}

/* NumTreeNodes: check that nc is power of 2, return 2nc-1 */
static int NumTreeNodes(int nc)
{
   int n = nc;
   
   while (n>2) {
      if ((n % 2) == 1)
         HError(7172,"NumTreeNodes: nc %d not a power of 2",nc);
      n /= 2;
   }
   return 2*nc-1;
}

/* InitClustering: create the cluster set data structure */
static void InitClustering(MemHeap *x, Sequence vpool, int nc,
                           Boolean treeCluster, CovKind distck, CovKind clusck, Covariance distcov)
{
   int i,numClust;
   Vector v;
   Covariance cov;

   nItems = vpool->nItems;
   if (nItems < nc)
      HError(7120,"InitClustering: only %d items for %d clusters",nItems,nc);
   cvp = vpool;   
   numClust = (treeCluster)?NumTreeNodes(nc):nc;
   curNumCl = 1;
   ccs = (ClusterSet *)New(x,sizeof(ClusterSet));
   ccs->isTree = treeCluster;
   ccs->numClust = numClust;
   ccs->ck = clusck;
   ccs->cl = (Cluster *)New(x,sizeof(Cluster)*numClust);
   --ccs->cl;  /* index is 1..numClust */
   cmap = CreateShortVec(&gstack,nItems);
   for (i=1; i<=nItems; i++)  /* put all vecs in cluster 1 */
      cmap[i] = 1;
   v = (Vector)GetItem(cvp,1); vSize = VectorSize(v);
   vTmp = CreateVector(&gstack,vSize);
   dck = distck; dcov = distcov;
   for (i=1; i<=numClust; i++){
      ccs->cl[i].csize = 0;
      ccs->cl[i].vCtr = CreateVector(x,vSize);
      ccs->cl[i].aveCost = 0.0;
      switch (clusck){
      case NULLC:
         cov.var = NULL;
         break;
      case DIAGC:
      case INVDIAGC:
         cov.var = CreateVector(x,vSize);
         break;
      case FULLC:
         cov.inv = CreateTriMat(x,vSize);
         break;
      default:
         HError(7170,"InitClustering: bad cluster ckind");
      }
      ccs->cl[i].cov = cov;
   }
   ccs->cl[1].csize = nItems;
   ccs->cl[1].aveCost = 1.0;
   FindCentres(1,1);
}

/* EXPORT->FlatCluster: apply linear clustering to vpool */
ClusterSet *FlatCluster(MemHeap *x, Sequence vpool, int nc, 
                        CovKind dck, CovKind cck, Covariance dcov)
{
   int i,c,cc,ce,iter,repairCount;
   float oldCost,newCost;
   Boolean converged;
   
   InitClustering(x,vpool,nc,FALSE,dck,cck,dcov);
   if (trace&T_CGE)
      printf("FlatCluster: %d items -> %d clusters\n",
             ccs->cl[1].csize,ccs->numClust);
   for (c=2; c<=ccs->numClust; c++) {
      if (trace&T_CGE)
         printf(" increasing from %d -> %d clusters\n",c-1,c);
      /* increase num clusters by splitting biggest */
      cc = BiggestCluster();
      Perturb(cc,cc,c); curNumCl = c;
      oldCost = 1e10;
      /* reallocate vectors until cost stabilises */
      for (iter=0,converged=FALSE; !converged; iter++){
         repairCount = 0; /* try to fill empty clusters by splitting fullest */
         while ((ce=AllocateVectors(&newCost)) != 0  && ++repairCount <=c){
            cc = FullestCluster();
            Perturb(cc,cc,ce);      /* ce = empty cluster */
         }
         if (ce != 0)
            HError(7120,"FlatCluster: Failed to make %d clusters at iter %d\n",c,iter);
         if (trace & T_CLC){
            printf("   c=%d, iter=%d, cost = %e\n",c,iter,newCost);
            fflush(stdout);
         }
         converged = (iter>=maxIter) || ((oldCost-newCost) / oldCost < 0.001);
         FindCentres(1,curNumCl); oldCost = newCost;
      }
      if (trace & T_DCM) DumpClusterMap();
   }
   for (i=1; i<=ccs->numClust; i++)
      FindCovariance(i);
   FreeShortVec(&gstack,cmap);
   return ccs;
}  

/* EXPORT->TreeCluster: apply binary tree clustering to vpool */
ClusterSet *TreeCluster(MemHeap *x, Sequence vpool, int nc, 
                        CovKind dck, CovKind cck, Covariance dcov)
{
   int rowsize,c,i,j;
   
   InitClustering(x,vpool,nc,TRUE,dck,cck,dcov);
   if (trace&T_CGE)
      printf("TreeCluster: %d items -> %d clusters[%d nodes]\n",
             ccs->cl[1].csize,nc,ccs->numClust);
   FindCovariance(1);
   c = 1; rowsize = 1;
   while (rowsize < nc){
      for (i=c,j=c+rowsize; i<c+rowsize; i++, j+=2){
         Perturb(i,j,j+1);
         SplitVectors(i,j,j+1);
         FindCentres(j,j+1);
         FindCovariance(j);
         FindCovariance(j+1);
      }
      c += rowsize;
      rowsize *= 2;
   }
   FreeShortVec(&gstack,cmap);
   return ccs;
}

/* EXPORT->FreeClusterSet: free storage used by cs */
void FreeClusterSet(ClusterSet *cs)
{
   Dispose(cs->x,cs);
}

/* EXPORT->ShowClusterSet: print the given clusterset */
void ShowClusterSet(ClusterSet *cs)
{
   int i;
   Cluster *c;

   printf("%s Cluster Set: %d nodes\n",
          cs->isTree?"Tree":"Flat", cs->numClust);
   for (i=1,c=cs->cl+1; i<=cs->numClust; i++,c++) {
      printf("%d. size=%d",i,c->csize);
      if (!cs->isTree) printf(" cost=%.3f",c->aveCost);
      printf("\n");
      ShowVector("  mean",c->vCtr,20);
      switch(cs->ck){
      case NULLC: 
         break;
      case INVDIAGC: 
         ShowVector("  invdiagC",c->cov.var,20);
         break;
      case DIAGC: 
         ShowVector("  diagC",c->cov.var,20);
         break;
      case FULLC:
         ShowTriMat("  fullC",c->cov.inv,20,20);
         break;
      }     
   }
}

/* ------------------------- Accumulators ---------------------- */

static int muC,vaC,trC,wtC,prC;

/* CreateMuAcc:  create an accumulator for means */
static MuAcc *CreateMuAcc(MemHeap *x, int vSize, int nPara)
{
   MuAcc *ma;
   int count;
   ma = (MuAcc *)New(x,sizeof(MuAcc)*nPara);
   for(count=0;count<nPara;count++){
     ma[count].mu = CreateVector(x,vSize);
     ZeroVector(ma[count].mu);
     ma[count].occ = 0.0;
     ++muC;
  }
   return ma;
}

/* CreateVaAcc: create an accumulator for square sums */
static VaAcc *CreateVaAcc(MemHeap *x, int vSize, CovKind ck, int nPara)
{
   VaAcc *va;
   int count;
   va = (VaAcc *) New(x,sizeof(VaAcc)*nPara);
   for(count=0;count<nPara;count++){
     switch(ck){
     case DIAGC:
     case INVDIAGC:
       va[count].cov.var = CreateVector(x,vSize);
       ZeroVector(va[count].cov.var);
       break;
     case FULLC:
       va[count].cov.inv = CreateTriMat(x,vSize);
       ZeroTriMat(va[count].cov.inv);
       break;
     default:
       HError(7170,"CreateVaAcc: bad cov kind %d",ck);
     }
     va[count].occ = 0.0;
     ++vaC;
   }
   return va;
}

/* CreateTrAcc: create an accumulator for transition counts */
static TrAcc *CreateTrAcc(MemHeap *x, int numStates, int nPara)
{
   TrAcc *ta;
   int count;
   ta = (TrAcc *) New(x,sizeof(TrAcc)*nPara);
   for(count=0;count<nPara;count++){
     ta[count].tran = CreateMatrix(x,numStates,numStates);
     ZeroMatrix(ta[count].tran);
     ta[count].occ = CreateVector(x,numStates);
     ZeroVector(ta[count].occ);
     ++trC;
   }
   return ta;
}

/* CreateWtAcc: create an accumulator for mixture weights */
static WtAcc *CreateWtAcc(MemHeap *x, int nMix, int nPara)
{
   WtAcc *wa;
   int count;
   wa = (WtAcc *) New(x,sizeof(WtAcc)*nPara);
   for(count=0;count<nPara;count++){
     wa[count].c = CreateVector(x,nMix);
     ZeroVector(wa[count].c);
     wa[count].occ = 0.0;
     wa[count].time = -1; wa[count].prob = NULL;
     ++wtC;
   }
   return wa;
}

/* CreatePreComp: create a struct for precomputed probs */
static PreComp *CreatePreComp(MemHeap *x)
{
   PreComp *p;
   
   p = (PreComp *) New(x,sizeof(PreComp));
   p->time = -1; p->prob = LZERO;
   ++prC;
   return p;
}

/* TMAttachAccs: attach accumulators to tied mixes in hset */
void TMAttachAccs(HMMSet *hset, MemHeap *x, int nPara)
{
   int size,s,m,nStreams;
   TMixRec tmRec;
   MixPDF *mp;
   
   nStreams = hset->swidth[0];
   for (s=1;s<=nStreams;s++){
      size = hset->swidth[s];
      tmRec = hset->tmRecs[s];
      for (m=1;m<=tmRec.nMix;m++){
         mp = tmRec.mixes[m];
         SetHook(mp->mean,CreateMuAcc(x,size,nPara));
         SetHook(mp->cov.var,CreateVaAcc(x,size,mp->ckind,nPara));
      }
   }
}



/* EXPORT->AttachAccs: attach accumulators to hset */
void AttachAccs(HMMSet *hset, MemHeap *x, UPDSet uFlags){ AttachAccsParallel(hset,x,uFlags,1); }
void AttachAccsParallel(HMMSet *hset, MemHeap *x, UPDSet uFlags, int nPara)
{
   HMMScanState hss;
   StreamElem *ste;
   HLink hmm;
   int size;

   muC=vaC=trC=wtC=prC=0;
   NewHMMScan(hset,&hss);
   do {
      hmm = hss.hmm;
      hmm->hook = (void *)0;  /* used as numEg counter */
      while (GoNextState(&hss,TRUE)) {
         while (GoNextStream(&hss,TRUE)) {
            ste = hss.ste;
            ste->hook = CreateWtAcc(x,hss.M, nPara);
            if ((uFlags&UPSEMIT) && (strmProj)) size = hset->vecSize; /* handles multiple streams */
            else size = hset->swidth[hss.s];
            if (hss.isCont)                     /* PLAINHS or SHAREDHS */
               while (GoNextMix(&hss,TRUE)) {
                  if (DoPreComps(hset->hsKind))
                     hss.mp->hook = CreatePreComp(x);
		  if (!IsSeenV(hss.mp->mean)) {
                     if (uFlags&UPMEANS) 
                        SetHook(hss.mp->mean,CreateMuAcc(x,size,nPara));
                     else 
                        SetHook(hss.mp->mean,NULL);
                     TouchV(hss.mp->mean);
                  }
		  if (!IsSeenV(hss.mp->cov.var)) {
                     if (uFlags&UPSEMIT)
                        SetHook(hss.mp->cov.var,
                                CreateVaAcc(x,size,FULLC,nPara));
                     else if (uFlags&UPVARS) 
                        SetHook(hss.mp->cov.var,
                                CreateVaAcc(x,size,hss.mp->ckind,nPara));
                     else 
                        SetHook(hss.mp->cov.var,NULL);
                     TouchV(hss.mp->cov.var);
                  }
               }
         }
      }
      if (!IsSeenV(hmm->transP)) {
         SetHook(hmm->transP,CreateTrAcc(x,hmm->numStates,nPara));
         TouchV(hmm->transP);       
      }
   } while (GoNextHMM(&hss));
   EndHMMScan(&hss);
   
   if (hset->hsKind==TIEDHS)    
      TMAttachAccs(hset, x, nPara);
   if (trace&T_NAC)
      printf("AttachAccs: %d mu, %d va, %d tr, %d wt, %d pr\n",
             muC,vaC,trC,wtC,prC);
}

/* TMZeroAccs: zero all accs attached to tied mixes in given HMMSet */
void TMZeroAccs(HMMSet *hset, int start, int end)
{
   TMixRec tmRec;
   int i,m,s,nStreams;
   MixPDF* mp;
   MuAcc *ma;
   VaAcc *va;
      
   nStreams = hset->swidth[0];
   for (s=1;s<=nStreams;s++){
      tmRec = hset->tmRecs[s];
      for (m=1;m<=tmRec.nMix;m++){
         mp = tmRec.mixes[m];
         ma = (MuAcc *)GetHook(mp->mean);
         va = (VaAcc *)GetHook(mp->cov.var);
         for(i=start;i<=end;i++){ 
	   ZeroVector(ma[i].mu); ma[i].occ = 0.0; 
	   switch(mp->ckind){
	   case DIAGC:
	   case INVDIAGC:
	     ZeroVector(va[i].cov.var);
	     break;
	   case FULLC:
	     ZeroTriMat(va[i].cov.inv);
	     break;
	   default:
	     HError(7170,"TMZeroAccs: bad cov kind %d",
		    mp->ckind);
	   }
	   va[i].occ = 0.0;
	 }
      }
   }
}

/*  EXPORT->ZeroAccs: zero all accumulators in given HMM set */
void ZeroAccs(HMMSet *hset, UPDSet uFlags){ ZeroAccsParallel(hset,uFlags,1); }
void ZeroAccsParallel(HMMSet *hset, UPDSet uFlags, int nPara)
{
   HMMScanState hss;
   StreamElem *ste;
   HLink hmm;
   TrAcc *ta;
   WtAcc *wa;
   MuAcc *ma;
   VaAcc *va;
   PreComp *p;
   int i,start,end;
   if(nPara>0){start=0;end=nPara-1;}else{start=end=-nPara;}
   NewHMMScan(hset,&hss);
   do {
      hmm = hss.hmm;
      hmm->hook = (void *)0;  /* used as numEg counter */
      while (GoNextState(&hss,TRUE)) {
         while (GoNextStream(&hss,TRUE)) {
            ste = hss.ste;
            wa = (WtAcc *)ste->hook;
            for(i=start;i<=end;i++){
	      ZeroVector(wa[i].c); wa[i].occ = 0.0;
	      wa[i].time = -1; wa[i].prob = NULL;
	    }
            if (hss.isCont)
               while (GoNextMix(&hss,TRUE)) {
		  if (DoPreComps(hset->hsKind)){
		     p = (PreComp *)hss.mp->hook;  
		     p->time = -1; p->prob = LZERO;
                  }
                  if ((uFlags&UPMEANS) && (!IsSeenV(hss.mp->mean))) {
                     ma = (MuAcc *)GetHook(hss.mp->mean);
		     for(i=start;i<=end;i++){
                        ZeroVector(ma[i].mu); ma[i].occ = 0.0;
                     }
                     TouchV(hss.mp->mean);
                  }
                  if ((uFlags&UPSEMIT) && (!IsSeenV(hss.mp->cov.var))) {
		     va = (VaAcc *)GetHook(hss.mp->cov.var);
		     for(i=start;i<=end;i++){
                        ZeroTriMat(va[i].cov.inv);
                        va[i].occ = 0.0;
		     }
                     TouchV(hss.mp->cov.var);
                  } else if ((uFlags&UPVARS) && (!IsSeenV(hss.mp->cov.var))) {
		     va = (VaAcc *)GetHook(hss.mp->cov.var);
		     for(i=start;i<=end;i++){
                        switch(hss.mp->ckind){
                        case DIAGC:
                        case INVDIAGC:
                           ZeroVector(va[i].cov.var);
                           break;
                        case FULLC:
                           ZeroTriMat(va[i].cov.inv);
                           break;
                        default:
                           HError(7170,"ShowAccs: bad cov kind %d",
                                  hss.mp->ckind);
                        }
                        va[i].occ = 0.0;
		     }
                     TouchV(hss.mp->cov.var);
                  }
               }
         }
      }
      if (!IsSeenV(hmm->transP)) {
         ta = (TrAcc *)GetHook(hmm->transP);
	 for(i=start;i<=end;i++){
	   ZeroMatrix(ta[i].tran);
	   ZeroVector(ta[i].occ);
	 }
         TouchV(hmm->transP);       
      }
   } while (GoNextHMM(&hss));
   EndHMMScan(&hss);
   if (hset->hsKind==TIEDHS)   
      TMZeroAccs(hset,start,end);
}

/* TMShowAccs: show accs attached to tied mixes in hset */
void TMShowAccs(HMMSet *hset, int index)
{
   int m,s,nStreams;
   MixPDF* mp;
   const int mw=12;
   TMixRec tmRec;
   MuAcc *ma;
   VaAcc *va;
   
   nStreams = hset->swidth[0];
   for (s=1;s<=nStreams;s++){
      tmRec = hset->tmRecs[s];
      printf("Tied Mixtures for Stream %d\n",s);
      for (m=1;m<=tmRec.nMix;m++){
         mp = tmRec.mixes[m];
         printf("   mix %d\n",m);
         ma = (MuAcc *)GetHook(mp->mean);
         printf("    mean occ=%f\n",ma[index].occ);
         ShowVector("    means=",ma[index].mu,mw);

         va = (VaAcc *)GetHook(mp->cov.var);
         printf("    var occ=%f\n",va[index].occ);
         switch(mp->ckind){
         case DIAGC:
         case INVDIAGC:
            ShowVector("    vars=",va[index].cov.var,mw);
            break;
         case FULLC:
            ShowTriMat("    covs=",va[index].cov.inv,mw,mw);
            break;
         default:
            HError(7170,"TMShowAccs: bad cov kind %d",
                   mp->ckind);
         }
      }
   }
}

/* EXPORT->ShowAccs: show accs attached to hset */
void ShowAccs(HMMSet *hset, UPDSet uFlags){ ShowAccsParallel(hset, uFlags, 0); }

void ShowAccsParallel(HMMSet *hset, UPDSet uFlags, int index)
{
   const int mw=12;
   HMMScanState hss;
   StreamElem *ste;
   HLink hmm;
   TrAcc *ta;
   WtAcc *wa;
   MuAcc *ma;
   VaAcc *va;

   NewHMMScan(hset,&hss);
   do {
      hmm = hss.hmm;
      printf("%s\n",hss.mac->id->name);
      while (GoNextState(&hss,TRUE)) {
         printf(" state %d\n",hss.i);
         while (GoNextStream(&hss,TRUE)) {
            ste = hss.ste;
            printf("  stream %d\n",hss.s);
            if (ste->hook != NULL) {
               wa = (WtAcc *)ste->hook;
               printf("   wt occ=%f\n",wa[index].occ);
               ShowVector("   wts=",wa[index].c,mw);
            }
            if (hss.isCont)
               while (GoNextMix(&hss,TRUE)) {
                  printf("   mix %d\n",hss.m);
                  if ((uFlags&UPMEANS) && (!IsSeenV(hss.mp->mean))) {
                     ma = (MuAcc *)GetHook(hss.mp->mean);
                     printf("    mean occ=%f\n",ma[index].occ);
                     ShowVector("    means=",ma[index].mu,mw);
                     TouchV(hss.mp->mean);
                  }
                  if ((uFlags&UPVARS) && (!IsSeenV(hss.mp->cov.var))) {
                     va = (VaAcc *)GetHook(hss.mp->cov.var);
                     printf("    var occ=%f\n",va[index].occ);
                     switch(hss.mp->ckind){
                     case DIAGC:
                     case INVDIAGC:
                        ShowVector("    vars=",va[index].cov.var,mw);
                        break;
                     case FULLC:
                        ShowTriMat("    covs=",va[index].cov.inv,mw,mw);
                        break;
                     default:
                        HError(7170,"ShowAccs: bad cov kind %d",
                               hss.mp->ckind);
                     }
                     TouchV(hss.mp->cov.var);
                  }
               }
         }
      }
      if (!IsSeenV(hmm->transP)) {
         ta = (TrAcc *)GetHook(hmm->transP);
         ShowVector("  tr oc",ta[index].occ,mw);
         ShowMatrix("  trs",ta[index].tran,mw,mw);
         TouchV(hmm->transP);       
      }
   } while (GoNextHMM(&hss));
   EndHMMScan(&hss);
   
   if (hset->hsKind==TIEDHS)    
      TMShowAccs(hset, index);
}

/* EXPORT->AttachPreComps: attach PreComps to hset */
void AttachPreComps(HMMSet *hset, MemHeap *x)
{
   HMMScanState hss;
   StreamElem *ste;
   HLink hmm;

   prC=0; wtC=0;
   NewHMMScan(hset,&hss);
   do {
      hmm = hss.hmm;
      hmm->hook = (void *)0;  /* used as numEg counter */
      while (GoNextState(&hss,TRUE)) {
         while (GoNextStream(&hss,TRUE)) {
            ste = hss.ste;
            ste->hook = CreateWtAcc(x,hss.M, 1);
            if (hss.isCont)                     /* PLAINHS or SHAREDHS */
               while (GoNextMix(&hss,TRUE)) {
                  if (DoPreComps(hset->hsKind))
                     hss.mp->hook = CreatePreComp(x);
               }
         }
      }
   } while (GoNextHMM(&hss));
   EndHMMScan(&hss);
   if (trace&T_NAC)
      printf("AttachPreComps:  %d wt, %d pr\n",wtC,prC);
}

/* EXPORT->ResetPreComps: reset the precomputed prob fields in hset */
void ResetPreComps(HMMSet *hset)
{
   StreamElem *ste;
   HMMScanState hss;
   WtAcc *wa;
   PreComp *p;
   
   NewHMMScan(hset,&hss);
   do {
      while (GoNextState(&hss,TRUE)) {
         while (GoNextStream(&hss,TRUE)) {
            ste = hss.ste;
            wa = (WtAcc *)ste->hook;
            wa->time = -1; wa->prob = NULL;
            if (hss.isCont)
               while (GoNextMix(&hss,TRUE)) {
                  p = (PreComp *)hss.mp->hook;
                  p->time = -1; p->prob = LZERO;
               }
         }
      }
   } while (GoNextHMM(&hss));
   EndHMMScan(&hss);
}

/* EXPORT->ResetHMMPreComps: reset the precomputed prob fields in hmm */
void ResetHMMPreComps(HLink hmm, int nStreams)
{
   StateElem *se;
   StreamElem *ste;
   MixtureElem *me;
   WtAcc *wa;
   PreComp *p;
   int i,s,m,nStates,nMixes;

   nStates = hmm->numStates;
   se = hmm->svec+2;
   for (i=2; i<nStates; i++,se++){
      ste = se->info->pdf+1;
      for (s=1;s<=nStreams; s++,ste++){
         wa = (WtAcc *)ste->hook; nMixes = ste->nMix;
         if (wa != NULL) {
            wa->time = -1; wa->prob = NULL;
            me = ste->spdf.cpdf+1;
            for (m=1; m<=nMixes; m++,me++){
               p = (PreComp *)me->mpdf->hook;
               p->time = -1; p->prob = LZERO;
            }
         }
      }
   }
}

/* EXPORT->ResetHMMWtAccs: reset the wt accs for the specified HMM */
void ResetHMMWtAccs(HLink hmm, int nStreams)
{
   StateElem *se;
   StreamElem *ste;
   WtAcc *wa;
   int i,s,nStates;

   nStates = hmm->numStates;
   se = hmm->svec+2;
   for (i=2; i<nStates; i++,se++){
      ste = se->info->pdf+1;
      for (s=1;s<=nStreams; s++,ste++){
         wa = (WtAcc *)ste->hook;
         if (wa != NULL) {
            wa->time = -1; wa->prob = NULL;
         }
      }
   }
}

/* DumpPName: dump physical HMM name */
static void DumpPName(FILE *f, char *pname)
{
   WriteString(f,pname,DBL_QUOTE);
   fprintf(f,"\n");
}

/* DumpWtAcc: dump wt acc to file f */
static void DumpWtAcc(FILE *f, WtAcc *wa)
{
   WriteVector(f,wa->c,ldBinary);
   WriteFloat(f,&(wa->occ),1,ldBinary);
   if (!ldBinary) fprintf(f,"\n");
}

/* DumpMuAcc: dump mean acc to file f */
static void DumpMuAcc(FILE *f, MuAcc *ma)
{
   WriteVector(f,ma->mu,ldBinary);
   WriteFloat(f,&(ma->occ),1,ldBinary);
   if (!ldBinary) fprintf(f,"\n");
}

/* DumpVaAcc: dump variance acc to file f */
static void DumpVaAcc(FILE *f, VaAcc *va, CovKind ck)
{
   switch(ck){
   case DIAGC:
   case INVDIAGC:
      WriteVector(f,va->cov.var,ldBinary);
      break;
   case FULLC:
   case LLTC:
      WriteTriMat(f,va->cov.inv,ldBinary);
      break;
   default:
      HError(7170,"DumpVaAcc: bad cov kind");
   }
   WriteFloat(f,&(va->occ),1,ldBinary);
   if (!ldBinary) fprintf(f,"\n");
}

/* DumpTrAcc: dump transition acc to file f */
static void DumpTrAcc(FILE *f, TrAcc *ta)
{
   WriteMatrix(f,ta->tran,ldBinary);
   WriteVector(f,ta->occ,ldBinary);
   if (!ldBinary) fprintf(f,"\n");
}

/* DumpMarker: dump a marker into file f */
static void DumpMarker(FILE *f)
{
   int mark = 123456;
   
   WriteInt(f,&mark,1,ldBinary);
   if (!ldBinary) fprintf(f,"\n");
}

/* GetDumpFile: Process dump file name and open it */
static FILE * GetDumpFile(char *name, int n)
{
   char buf[MAXSTRLEN],num[20];
   int i,j,k,len,nlen;
   FILE *f;
   
   sprintf(num,"%d",n);
   len = strlen(name); nlen = strlen(num);
   for (i=0,j=0; i<len; i++) {
      if (name[i] == '$')
         for (k=0; k<nlen; k++)
            buf[j++] = num[k];
      else
         buf[j++] = name[i];
   }
   buf[j] = '\0';
   f = fopen(buf,"wb"); /* Binary file */
   if (f==NULL)
      HError(7111,"GetDumpFile: cannot open acc dump file %s",buf);
   if (trace & T_ALD)
      printf("Dumping accumulators to file %s\n",buf);
   return f;
}

/* EXPORT->DumpAccs: Dump a copy of the accs in hset to fname.
       Any occurrence of the $ symbol in fname is replaced by n.
       The file is left open and returned */
FILE * DumpAccs(HMMSet *hset, char *fname, UPDSet uFlags, int n){ return DumpAccsParallel(hset,fname,n,uFlags,0); }
FILE * DumpAccsParallel(HMMSet *hset, char *fname, int n, UPDSet uFlags, int index)
{
   FILE *f;
   HLink hmm;
   HMMScanState hss;
   int m,s;
   MixPDF* mp;
   
   f = GetDumpFile(fname,n);
   NewHMMScan(hset, &hss);
   do {
      hmm = hss.hmm;
      DumpPName(f,hss.mac->id->name);     
      WriteInt(f,(int *)&hmm->hook,1,ldBinary); 
      while (GoNextState(&hss,TRUE)) {
         while (GoNextStream(&hss,TRUE)) {
            DumpWtAcc(f,((WtAcc *)hss.ste->hook)+index);
            if (hss.isCont){
               while (GoNextMix(&hss,TRUE)) {
                  if ((uFlags&UPMEANS) && (!IsSeenV(hss.mp->mean))) {
                     DumpMuAcc(f,((MuAcc *)GetHook(hss.mp->mean))+index);
                     TouchV(hss.mp->mean);
                  }
                  if ((uFlags&UPSEMIT) && (!IsSeenV(hss.mp->cov.var))) {
                     DumpVaAcc(f,((VaAcc *)GetHook(hss.mp->cov.var))+index,FULLC);
                     TouchV(hss.mp->cov.var);
                  }else if ((uFlags&UPVARS) && (!IsSeenV(hss.mp->cov.var))) {
                     DumpVaAcc(f,((VaAcc *)GetHook(hss.mp->cov.var))+index,hss.mp->ckind);
                     TouchV(hss.mp->cov.var);
                  }
               }
            }
         }
      }     
      if (!IsSeenV(hmm->transP)){
	 DumpTrAcc(f, ((TrAcc *) GetHook(hmm->transP))+index);
         TouchV(hmm->transP);
      }
      DumpMarker(f);
   } while (GoNextHMM(&hss));   
   EndHMMScan(&hss);
   if (hset->hsKind == TIEDHS){
      for (s=1; s<=hset->swidth[0]; s++){
         for (m=1; m<=hset->tmRecs[s].nMix; m++){
            mp = hset->tmRecs[s].mixes[m];
            DumpMuAcc(f,((MuAcc *)GetHook(mp->mean))+index);
            DumpVaAcc(f,((VaAcc *)GetHook(mp->cov.var))+index,mp->ckind);
         }
      }
   }    
   return f;
}

/* LoadWtAcc: new inc of wt acc from file f */
static void LoadWtAcc(Source *src, WtAcc *wa, int numMixtures)
{
   int m;
   float f;
   Vector cTemp;
   
   cTemp = CreateVector(&gstack,numMixtures);
   ReadVector(src,cTemp,ldBinary);
   for (m=1;m<=numMixtures;m++){
     if(!finite(cTemp[m]))
       HError(7191, "Infinite WtAcc!");
     wa->c[m] += cTemp[m];
   }
   ReadFloat(src,&f,1,ldBinary);
   wa->occ += f;
   FreeVector(&gstack,cTemp);
}

/* LoadMuAcc: new inc of mean acc from file f */
static void LoadMuAcc(Source *src, MuAcc *ma, int vSize)
{
   int k;
   Vector vTemp;
   float f;
   
   vTemp = CreateVector(&gstack,vSize);
   ReadVector(src,vTemp,ldBinary);
   for (k=1;k<=vSize;k++){
     if(!finite(vTemp[k]))
       HError(7191, "Infinite MuAcc!");
      ma->mu[k] += vTemp[k];
   }
   ReadFloat(src,&f,1,ldBinary);
   ma->occ += f;
   FreeVector(&gstack,vTemp);
}

/* LoadVaAcc: p'th inc of variance acc from file f */
static void LoadVaAcc(Source *src, VaAcc *va, int vSize, CovKind ck)
{
   int k,kk;
   Vector vTemp;
   TriMat mTemp;
   float f;
   
   switch(ck){
   case DIAGC:
   case INVDIAGC:
      vTemp = CreateVector(&gstack, vSize);
      ReadVector(src,vTemp,ldBinary);
      for (k=1;k<=vSize;k++){
        if(!finite(vTemp[k]))
           HError(7191, "Infinite VaAcc!");
         va->cov.var[k] += vTemp[k];
      }
      FreeVector(&gstack, vTemp);
      break;
   case FULLC:
   case LLTC:
      mTemp = CreateTriMat(&gstack,vSize);
      ReadTriMat(src,mTemp,ldBinary);
      for (k=1;k<=vSize;k++)
         for (kk=1; kk<=k; kk++) {
            va->cov.inv[k][kk] += mTemp[k][kk];
         }
      FreeTriMat(&gstack,mTemp);
      break;
   }
   ReadFloat(src,&f,1,ldBinary);
   va->occ += f;
}

/* LoadTrAcc: p'th inc of transition acc from file f */
static void LoadTrAcc(Source *src, TrAcc *ta, int numStates)
{
   int i,j;
   Matrix tTemp;
   Vector nTemp;
   
   tTemp = CreateMatrix(&gstack,numStates,numStates);
   nTemp = CreateVector(&gstack,numStates);
   ReadMatrix(src,tTemp,ldBinary);
   for (i=1;i<=numStates;i++)
      for (j=1;j<=numStates;j++)
         ta->tran[i][j] += tTemp[i][j];
   ReadVector(src,nTemp,ldBinary);
   for (i=1;i<=numStates;i++)
      ta->occ[i] += nTemp[i];
   FreeMatrix(&gstack,tTemp);
}

/* CheckPName: check dumped name matches hmm phys name */
static void CheckPName(Source *src, char *pname)
{
   int c;
   char buf[MAXSTRLEN];

   ReadString(src,buf);
   c = GetCh(src);
   if (c != '\n')
      HError(7150,"CheckPName: Cant find EOL");
   if (strcmp(pname,buf) != 0)
      HError(7150,"CheckPName: expected %s got %s",pname,buf);
}

/* CheckMarker: check file f has a marker next */
static void CheckMarker(Source *src)
{
   int mark = 123456, temp;
   
   ReadInt(src,&temp,1,ldBinary);
   if (temp != mark)
      HError(7150,"CheckMarker: Marker Expected in Dump File");
}

/* EXPORT->LoadAccs: inc accumulators in hset by vals in fname */

Source LoadAccs(HMMSet *hset, char *fname, UPDSet uFlags){ return LoadAccsParallel(hset,fname,uFlags,0); }
Source LoadAccsParallel(HMMSet *hset, char *fname, UPDSet uFlags, int index)
{
   Source src;
   HLink hmm;
   HMMScanState hss;
   int size,negs,m,s;
   MixPDF* mp;
   
   if (trace & T_ALD)
      printf("Loading accumulators from file %s\n",fname);

   if(InitSource(fname,&src,NoFilter)<SUCCESS)
      HError(7110,"LoadAccs: Can't open file %s", fname);
   NewHMMScan(hset, &hss);
   do {
      hmm = hss.hmm;
      CheckPName(&src,hss.mac->id->name); 
      ReadInt(&src,&negs,1,ldBinary);
      negs += (int)hmm->hook; hmm->hook = (void *)negs;
      while (GoNextState(&hss,TRUE)) {
         while (GoNextStream(&hss,TRUE)) {
            if ((uFlags&UPSEMIT) && (strmProj)) size = hset->vecSize;
            else size = hset->swidth[hss.s];
            LoadWtAcc(&src,((WtAcc *)hss.ste->hook)+index,hss.M);
            if (hss.isCont){
               while (GoNextMix(&hss,TRUE)) {
                  if ((uFlags&UPMEANS) && (!IsSeenV(hss.mp->mean))) {
		     LoadMuAcc(&src,((MuAcc *)GetHook(hss.mp->mean))+index,size);
                     TouchV(hss.mp->mean);
                  }
                  if ((uFlags&UPSEMIT) && (!IsSeenV(hss.mp->cov.var))) {
                     LoadVaAcc(&src,((VaAcc *)GetHook(hss.mp->cov.var))+index,
                               size,FULLC);
                     TouchV(hss.mp->cov.var);
                  } else if ((uFlags&UPVARS) && (!IsSeenV(hss.mp->cov.var))) {
                     LoadVaAcc(&src,((VaAcc *)GetHook(hss.mp->cov.var))+index,
                               size,hss.mp->ckind);
                     TouchV(hss.mp->cov.var);
                  }
               }
            }
         }
      }     
      if (!IsSeenV(hmm->transP)){
         LoadTrAcc(&src, ((TrAcc *) GetHook(hmm->transP))+index,hss.N);
         TouchV(hmm->transP);
      }
      CheckMarker(&src);
   } while (GoNextHMM(&hss));
   EndHMMScan(&hss);
   if (hset->hsKind == TIEDHS){
      for (s=1; s<=hset->swidth[0]; s++){
         size = hset->swidth[s];
         for (m=1;m<=hset->tmRecs[s].nMix; m++){
            mp = hset->tmRecs[s].mixes[m];
            LoadMuAcc(&src,((MuAcc *)GetHook(mp->mean))+index,size);
            LoadVaAcc(&src,((VaAcc *)GetHook(mp->cov.var))+index,size,mp->ckind);
         }
      }
   }    
   return src;
}

void RestorePDF(MixPDF *mp, int index){
   int i,j;
   MuAcc *ma = ((MuAcc *)GetHook(mp->mean))+index;
   VaAcc *va = ((VaAcc*)GetHook(mp->cov.var))+index;
   int size = VectorSize(mp->mean);

   for(i=1;i<=size;i++){ ma->mu[i] += ma->occ * mp->mean[i]; }
   switch(mp->ckind){
   case DIAGC: case INVDIAGC:
      for(i=1;i<=size;i++){ va->cov.var[i] += 2*ma->mu[i]*mp->mean[i] - va->occ*mp->mean[i]*mp->mean[i]; }
      break;
   case FULLC: case LLTC:
      for(i=1;i<=size;i++){ 
         for(j=1;j<=i;j++){
            va->cov.inv[i][j] += ma->mu[i]*mp->mean[j] + ma->mu[j]*mp->mean[i] - va->occ*mp->mean[i]*mp->mean[j]; 
         }
      }
      break;
   default: HError(7191, "Unknown ckind [RestoreAccsParallel]");
   }
}


void RestoreAccs(HMMSet *hset){ RestoreAccsParallel(hset,0); }
void RestoreAccsParallel(HMMSet *hset, int index)
{
   HMMScanState hss;
   int s,m,size;

   if(hset->hsKind==TIEDHS){
      for (s=1; s<=hset->swidth[0]; s++){
         size = hset->swidth[s];
         for (m=1;m<=hset->tmRecs[s].nMix; m++)
            RestorePDF(hset->tmRecs[s].mixes[m], index);
      }
   } else {
      NewHMMScan(hset, &hss);
      while (GoNextMix(&hss,FALSE)) {
         RestorePDF(hss.mp, index);
      }
      EndHMMScan(&hss);
   }
}


double ScalePDF(MixPDF *mpdf, int vSize, int index, float wt)
{
   float ans;
   MuAcc *ma = ((MuAcc*)GetHook(mpdf->mean))+index;
   VaAcc *va = ((VaAcc*)GetHook(mpdf->cov.var))+index; /*diagonal case, of course.*/
   {/*Scale the mu.*/
      int x;
      ma->occ *= wt;
      for(x=1;x<=vSize;x++)
         ma->mu[x] *= wt;
   }
   {/*Scale the var.*/
      int x;
      ans = va->occ;
      va->occ *= wt;
      for(x=1;x<=vSize;x++)
         va->cov.var[x] *= wt;
   }
   return ans;
}


double ScaleAccs(HMMSet *hset, float wt)
{
   return ScaleAccsParallel(hset,wt,0);
}

double ScaleAccsParallel(HMMSet *hset, float wt, int index)
{ 
   HMMScanState hss;
   int s,m,size;
   float ans=0;
  
   if(hset->ckind != DIAGC || !(hset->hsKind==PLAINHS || hset->hsKind==SHAREDHS || 
                                hset->hsKind==TIEDHS))
      HError(-1, "ScaleAccsParallel: wrong kind of hset.");

   /* Do gaussians. */
   if(hset->hsKind==TIEDHS){
      for (s=1; s<=hset->swidth[0]; s++){
         size = hset->swidth[s];
         for (m=1;m<=hset->tmRecs[s].nMix; m++)
            ans += ScalePDF(hset->tmRecs[s].mixes[m], size, index, wt);
      }
   } else {
      NewHMMScan(hset, &hss);
      while (GoNextMix(&hss,FALSE)) {
         size = hset->swidth[hss.s];
         ans += ScalePDF(hss.mp, size, index, wt);
      }
      EndHMMScan(&hss);
   }


   /* Do weights. */
   NewHMMScan(hset,&hss);
   while(GoNextState(&hss,FALSE)){ /*skip over hmm boundaries.*/
      while(GoNextStream(&hss,TRUE)){ /*Don't skip over state boundaries.*/
         StreamElem *ste = hss.ste; int m, nMix;
         WtAcc *wa = ((WtAcc*) hss.ste->hook)+index;
         switch(hset->hsKind){
         case PLAINHS: case SHAREDHS:
            nMix = (ste->nMix>0?ste->nMix:-ste->nMix);
            wa->occ*=wt; /*take the value wa->occ to the desired value.*/
            for(m=1;m<=nMix;m++){
               wa->c[m] *= wt; /*scale the WtAcc->c[]*/
            }
            break;
         case TIEDHS:
            nMix = hset->tmRecs[hss.s].nMix;
            for(m=1;m<=nMix;m++){
               wa->c[m] *= wt; /*scale the WtAcc->c[]*/
            }
            break;
         default: HError(1, "ScaleAccs- unknown hsKind.");
         }
      }
   }
   EndHMMScan(&hss);

   /* Do transitions. */
   NewHMMScan(hset,&hss);
   do{
      HLink hmm = hss.hmm; 
      int i,j,N; /*This code taken from UpdateTransP*/
      TrAcc *ta;
    
      if (!IsSeenV(hmm->transP)){
         TouchV(hmm->transP);
         ta = ((TrAcc*)GetHook(hmm->transP))+index;
         if (ta==NULL) HError(1, "HTrain.c: ScaleAccs: null TransP.");
         N = hmm->numStates;
         for (i=1;i<N;i++) {
            ta->occ[i]*=wt;
            for (j=2;j<=N;j++) {
               ta->tran[i][j]*=wt;
            }
         }
      }
   } while(GoNextHMM(&hss));
   EndHMMScan(&hss);
   return ans;
}


/* ------------------------ End of HTrain.c ----------------------- */
