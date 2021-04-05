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
/*         File: HVQ.c:   Vector Quantisation                  */
/* ----------------------------------------------------------- */

char *hvq_version = "!HVER!HVQ:   3.4.1 [CUED 12/03/09]";
char *hvq_vc_id = "$Id: HVQ.c,v 1.1.1.1 2006/10/11 09:54:59 jal58 Exp $";

#include "HShell.h"
#include "HMem.h"
#include "HMath.h"
#include "HVQ.h"

/* ------------------------ Trace Flags ------------------------- */

static int trace = 0;

/* -------------------------------------------------------------- */

#define MAXVQNODES 5000    /* just for error checking */

static MemHeap vqHeap;     /* MSTAK for allocating VQTables */
static VQTable vqList = NULL;
static ConfParam *cParm[MAXGLOBS];       /* config parameters */
static int numParm = 0;


/* EXPORT->InitVQ: initialise the VQ module */
void InitVQ(void)
{
   int i;
   
   Register(hvq_version,hvq_vc_id);
   numParm = GetConfig("HVQ", TRUE, cParm, MAXGLOBS);
   if (numParm>0){
      if (GetConfInt(cParm,numParm,"TRACE",&i)) trace = i;
   }
   CreateHeap(&vqHeap,"vqHeap",MSTAK,1,0.2,2000,10000);
}

/* CKCheck: check that ck is one of NULLC,INVDIAGC or FULLC */
static CovKind CKCheck(CovKind ck)
{
   if (ck != NULLC && ck != INVDIAGC && ck != FULLC) 
      HError(6172,"CKCheck: bad covkind %d",ck);
   return ck;
}

/* FindVQTable: find VQ table with given name and unless magic is 0, check 
                that it is same, return NULL if not found */
static VQTable FindVQTable(char * tabFN, short magic)
{
   Boolean found=FALSE;
   VQTable p;
      
   for (p=vqList; p!=NULL && !found; p=p->next)
      if (strcmp(tabFN,p->tabFN)==0){
         if (magic != 0 && magic != p->magic)
            HError(6170,"FindVQTable: %s has magic=%d but new magic=%d",
                   tabFN,p->magic,magic);
         return p;
      }
   return NULL;
}

/* EXPORT->CreateVQTab: Create an empty VQTable with given attributes */
VQTable CreateVQTab(char *tabFN, short magic, TreeType type,
                    CovKind ck, short *swidth)
{
   VQTable vq;
   int s;
   
   if (FindVQTable(tabFN,0) != NULL)
      HError(6171,"CreateVQTab: VQ table %s already exists",tabFN);
   vq = (VQTable)New(&vqHeap,sizeof(VQTabRec));
   vq->next = vqList; vqList = vq;
   vq->tabFN = CopyString(&vqHeap,tabFN);
   vq->magic = magic; vq->type = type; 
   vq->ckind = CKCheck(ck); 
   vq->numNodes = 0;
   for (s=0; s<SMAX; s++) {
      vq->swidth[s] = swidth[s];
      vq->tree[s] = NULL;
   }
   return vq;
}

/* InvDiagGConst: compute gConst for given inv variance */
static float InvDiagGConst(Vector iv)
{
   float sum;
   int i,n;

   n=VectorSize(iv); sum = n*log(TPI);
   for (i=1; i<=n; i++)
      sum -= log(iv[i]);
   return sum;
}

/* FullInvGConst: compute gConst for inv covariance */
static float FullInvGConst(TriMat ic)
{
   return TriMatSize(ic)*log(TPI) - CovDet(ic);
}

/* EXPORT->CreateVQNode: Create a VQ node with given values */
VQNode CreateVQNode(short vqidx, short nid, short lid, short rid, 
                    Vector mean, CovKind ck, Covariance cov)
{
   VQNode n;
   
   n = (VQNode)New(&vqHeap,sizeof(VQNodeRec));
   n->vqidx = vqidx; n->mean = mean; n->cov = cov;
   n->nid = nid; n->lid = lid; n->rid = rid;
   n->left = n->right = NULL; 
   switch(ck){
   case NULLC:
      n->gconst = 0.0;
      break;
   case INVDIAGC:
      n->gconst = InvDiagGConst(cov.var);
      break;
   case FULLC:
      n->gconst = FullInvGConst(cov.inv);
      break;
   }
   return n;
}

/* GetVal: get a short from given source and check range */
static short GetVal(Source *src, short lo, short hi, char *item)
{
   short x;
   char buf[MAXSTRLEN];
   
   if (!ReadShort(src,&x,1,FALSE))
      HError(6150,"GetVal: cant read %s at %s",item,
             SrcPosition(*src, buf));
   if (lo==0 && hi==0) return x;
   if (x<lo || x>hi)
      HError(6151,"GetVal: %s out of range at %s",item,
             SrcPosition(*src, buf));   
   return x;
}

/* GetNode: read a node definition and create node */
static VQNode GetNode(Source *src, CovKind ck, short width)
{
   char buf[MAXSTRLEN];
   VQNode n;
   short vqidx,nid,lid,rid;
   Vector mean;
   Covariance cov;
   
   vqidx = GetVal(src,0,0,"VQ Index");
   nid = GetVal(src,0,0,"Node Id");
   lid = GetVal(src,0,0,"Left Id");
   rid = GetVal(src,0,0,"Right Id");
   mean = CreateVector(&vqHeap,width);
   if (!ReadVector(src, mean, FALSE))
      HError(6150,"GetNode: cannot read mean vector at %s",
             SrcPosition(*src, buf));   
   switch(ck){
   case NULLC:
      cov.var = NULL;
      n = CreateVQNode(vqidx,nid,lid,rid,mean,ck,cov);
      break;
   case INVDIAGC:
      cov.var = CreateVector(&vqHeap,width);
      if (!ReadVector(src, cov.var, FALSE))
         HError(6150,"GetNode: cannot read variance vector at %s",
                SrcPosition(*src, buf));   
      n = CreateVQNode(vqidx,nid,lid,rid,mean,ck,cov);
      break;
   case FULLC:
      cov.inv = CreateTriMat(&vqHeap,width);
      if (!ReadTriMat(src, cov.inv, FALSE))
         HError(6150,"GetNode: cannot read covariance matrix at %s",
                SrcPosition(*src, buf));
      n = CreateVQNode(vqidx,nid,lid,rid,mean,ck,cov);
      break;
   default:
      n = CreateVQNode(vqidx,nid,lid,rid,mean,ck,cov);
      break;   
   }
   return n;
}

/* FindVQNode: find and unlink node from list with given id */
static VQNode FindVQNode(VQNode *list, short nid)
{
   VQNode p,q;
   
   p = *list;
   if (p==NULL) return NULL;
   if (p->nid == nid) {
      *list = p->right; return p;
   }
   q = p; 
   for (p=p->right; p != NULL; q=p, p=p->right)
      if (p->nid == nid){
         q->right = p->right; return p;
      }
   return NULL;
}

/* SortEntries: use the id fields of the supplied list of nodes
                to build the required linTree or binTree */
static VQNode SortEntries(VQNode *list, short rootId)
{
   VQNode newNode;
   
   newNode = FindVQNode(list,rootId);
   if (newNode == NULL)
      HError(6173,"SortEntries: cannot find node %d",rootId);
   if (newNode->lid != 0)
      newNode->left = SortEntries(list,newNode->lid);
   else
      newNode->left = NULL;
   if (newNode->rid != 0)
      newNode->right = SortEntries(list,newNode->rid);
   else
      newNode->right = NULL;
   return newNode;
}

/* EXPORT->LoadVQTab: create a VQTable using defs in tabFN */
VQTable LoadVQTab(char *tabFN, short magic)
{
   VQTable vq;
   Source src;
   short fmagic, numNodes, swidth[SMAX];
   TreeType type;
   CovKind ck;
   VQNode n;
   int s,i;
   
   /* See if this VQ table already loaded */
   if ((vq=FindVQTable(tabFN,magic)) != NULL)
      return vq;
   /* Load Definition Header Info and create table */
   if(InitSource(tabFN,&src,NoFilter)<SUCCESS)
      HError(6110,"LoadVQTab: Can't open file %s", tabFN);

   fmagic = GetVal(&src,0,0,"magic number");
   if (magic != 0 && magic != fmagic)
      HError(6170,"LoadVQTab: %s has magic=%d but reqd magic=%d",
             tabFN,fmagic,magic);
   type = (TreeType) GetVal(&src,linTree,binTree,"tree type");
   ck = CKCheck((CovKind) GetVal(&src,DIAGC,NULLC,"cov kind"));
   numNodes = GetVal(&src,1,MAXVQNODES,"number of nodes");
   swidth[0] = GetVal(&src,1,SMAX,"number of streams");
   for (s=1; s<=swidth[0]; s++)
      swidth[s] = GetVal(&src,1,10000,"stream width");
   vq = CreateVQTab(tabFN, fmagic, type, ck, swidth);
   /* Load Entries as unordered list */
   for (i=1; i<=numNodes; i++){
      s = GetVal(&src,1,SMAX,"stream index");
      n = GetNode(&src,ck,swidth[s]);
      n->right = vq->tree[s];
      vq->tree[s] = n;
   }
   vq->numNodes = numNodes;
   /* Sort Entries according to id numbers */
   for (s=1; s<=swidth[0]; s++){
      n = vq->tree[s];
      vq->tree[s] = SortEntries(&n,1);
   }
   /* Close definition file and leave */
   FClose(src.f,src.isPipe);
   return vq;
}

/* MarkTree: scan tree and give each node a unique id */
static void MarkTree(VQNode n, short *nid)
{
   if (n != NULL){
      n->nid = (*nid)++;
      n->lid = n->rid = 0;
      if (n->left != NULL){
         MarkTree(n->left,nid);  
         n->lid = n->left->nid;
      }
      if (n->right != NULL){
         MarkTree(n->right,nid);
         n->rid = n->right->nid;
      }
   }
}

/* StoreTree: store each node as an entry */
static void StoreTree(FILE *f, VQNode n, CovKind ck, short s)
{
   if (n != NULL){
      fprintf(f,"%d %d %d %d %d\n",s,n->vqidx,n->nid,n->lid,n->rid);
      WriteVector(f,n->mean,FALSE);
      switch(ck){
      case NULLC:   
         break;
      case INVDIAGC:
         WriteVector(f,(Vector)n->cov.var,FALSE);
         break;
      case FULLC:
         WriteTriMat(f,(TriMat)n->cov.inv,FALSE);
         break;
      }
      fprintf(f,"\n");
      StoreTree(f,n->left,ck,s);
      StoreTree(f,n->right,ck,s);
   }
}

/* EXPORT->StoreVQTab: store VQTable in tabFN */
void StoreVQTab(VQTable vqTab, char *tabFN)
{
   FILE *f;
   char *fn;
   short nid,s,sum;
   VQTable v=vqTab;
   VQNode n;
   
   fn = (tabFN==NULL)?vqTab->tabFN:tabFN;
   if ((f = fopen(fn,"w")) == NULL)
      HError(6111,"StoreVQTab: cannot create file %s",fn);
   /* Stamp each node with a unique id */
   sum = 0;
   for (s=1; s<=v->swidth[0]; s++){
      nid = 1; n = v->tree[s];
      MarkTree(n,&nid);
      sum += (nid-1);
   }
   v->numNodes = sum;
   /* Write the Header */
   fprintf(f,"%d %d %d %d %d ",
           v->magic,v->type,v->ckind,v->numNodes,v->swidth[0]);
   for (s=1; s<=v->swidth[0]; s++) fprintf(f,"%d ",v->swidth[s]);
   fprintf(f,"\n");   
   for (s=1; s<=v->swidth[0]; s++){
      n = v->tree[s];
      StoreTree(f,n,v->ckind,s);
   }
   fclose(f);
}

/* PrintTree: Print each node as an entry */
static void PrintTree(VQNode n, CovKind ck)
{
   if (n != NULL) {
      printf("vqidx=%d ids=[%d %d %d]\n",
             n->vqidx,n->nid,n->lid,n->rid);
      ShowVector("Mean",n->mean,10);
      switch(ck){
      case NULLC:   
         break;
      case INVDIAGC:
         ShowVector("IVar",(Vector)n->cov.var,10);
         break;
      case FULLC:
         ShowTriMat("ICov",(TriMat)n->cov.inv,10,10);
         break;
      }
      printf("\n");
      PrintTree(n->left,ck);
      PrintTree(n->right,ck);
   }
}

/* EXPORT->PrintVQTab: Print the given VQTable */
void PrintVQTab(VQTable vqTab)
{
   VQTable v=vqTab;
   VQNode n;
   short nid,s;

   printf("VQTable %s: ",v->tabFN);
   switch (v->type){
   case linTree: printf("LinTree "); break;
   case binTree: printf("BinTree "); break;
   }
   switch (v->ckind){
   case NULLC:    printf("Euclidean "); break;
   case INVDIAGC: printf("Inv Diag "); break;
   case FULLC:    printf("Inv Full "); break;
   }
   printf(" magic=%d nodes=%d\n",v->magic,v->numNodes);
   for (s=1; s<=v->swidth[0]; s++){
      nid = 1; n = v->tree[s];
      MarkTree(n,&nid);
   }
   for (s=1; s<=v->swidth[0]; s++){
      printf("Stream %d\n",s);
      n = v->tree[s];
      PrintTree(n,v->ckind);
   }
}

/* VQNodeScore: compute VQNodeScore between v and n, smallest score is best. */
float VQNodeScore(VQNode n, Vector v, int size, CovKind ck)
{
   Vector m,iv,crow,vx;
   TriMat ic;
   float x,sum;
   int i,j;
   
   m = n->mean;
   switch(ck){
   case NULLC:
      sum = 0.0;
      for (i=1; i<=size; i++){
         x = v[i]-m[i]; sum += x*x;
      }
      return n->gconst+sum;
   case INVDIAGC:
      iv = (Vector)n->cov.var;
      sum = 0.0;
      for (i=1; i<=size; i++){
         x = v[i]-m[i]; sum += x*x*iv[i];
      }
      return n->gconst+sum;
   case FULLC:
      ic = (TriMat)n->cov.inv;
      vx = CreateVector(&gstack,size);
      for (i=1;i<=size;i++)
         vx[i] = v[i] - m[i];
      sum = 0.0;
      for (i=2;i<=size;i++) {
         crow = ic[i];
         for (j=1; j<i; j++)
            sum += vx[i]*vx[j]*crow[j];
      }
      sum *= 2;
      for (i=1;i<=size;i++)
         sum += vx[i] * vx[i] * ic[i][i];
      FreeVector(&gstack,vx);
      return n->gconst+sum;
   default:
      HError(6172,"VQNodeScore: bad kind %d",ck);
   }
   return 0; /* to keep compiler happy */
}

/* EXPORT->GetVQ: get vq indices for vectors in fv */
void GetVQ(VQTable vqTab, int numS, Vector *fv, short *vq)
{
   short s,idx=0,size;
   float bestx, x,xl,xr;
   VQNode n,bestn;
   Vector v;
   
   for (s=1; s<=numS; s++) {
      n = vqTab->tree[s]; v = fv[s];
      size = VectorSize(v);
      if (n==NULL)
         HError(6174,"GetVQ: null tree in stream %d",s);
      if (size != vqTab->swidth[s])
         HError(6174,"GetVQ: stream %d width incompatible",s);
      switch(vqTab->type){
      case linTree:
         bestn = n; bestx = VQNodeScore(n,v,size,vqTab->ckind);
         for(n=bestn->right; n != NULL; n=n->right){
            x = VQNodeScore(n,v,size,vqTab->ckind);
            if (x<bestx) {
               bestx = x; bestn = n;
            }
         }
         idx = bestn->vqidx;
         break;
      case binTree:
         while (n->right != NULL){
            xr = VQNodeScore(n->right,v,size,vqTab->ckind);
            xl = VQNodeScore(n->left,v,size,vqTab->ckind);
            n =  (xr<xl)?n->right:n->left;
         }
         idx = n->vqidx;
         break;
      }
      vq[s] = idx;
   }
}

/* ------------------------------  HVQ.c ----------------------------- */
