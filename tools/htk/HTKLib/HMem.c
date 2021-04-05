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
/*         File: HMem.c:   Memory Management Module            */
/* ----------------------------------------------------------- */

char *hmem_version = "!HVER!HMem:   3.4.1 [CUED 12/03/09]";
char *hmem_vc_id = "$Id: HMem.c,v 1.1.1.1 2006/10/11 09:54:58 jal58 Exp $";

#include "HShell.h"
#include "HMem.h"

int debug_level = 0;               /* For esps linking */

/* --------------------------- Trace Flags ------------------------ */

static int trace = 0;

#define T_TOP  0001       /* Top Level Trace */
#define T_MHP  0002       /* M HEAP alloc/free */
#define T_CHP  0004       /* C HEAP alloc/free */
#define T_STK  0010       /* M STAK alloc/free */

/* ---------------------- Alignment Issues -------------------------- */

#define FWORD 8   /* size of a full word = basic alignment quanta */
/* 
   HMem only guarantees to allocate MSTAK objects on aligned boundaries.
   All MHEAP objects may be any size and they are allocated sequentially.
   However, a heap of math objects is guaranteed to work provided that the
   supplied size routines are used since these round up appropriately.
*/

/* EXPORT->MRound: round up a mem size request to be a multiple of FWORD */
size_t MRound(size_t size)
{
   return ((size % FWORD) == 0)?size : (size/FWORD + 1) * FWORD;
}

/* -------------- General Purpose Memory Management ----------------- */

static ConfParam *cParm[MAXGLOBS];       /* config parameters */
static int numParm = 0;
static Boolean protectStaks = FALSE;    /* enable stack protection */

MemHeap gstack;   /* global MSTAK for general purpose use */
MemHeap gcheap;   /* global CHEAP for general purpose use */

typedef struct _MemHeapRec {
   MemHeap *heap;
   struct _MemHeapRec *next;
} MemHeapRec;

static MemHeapRec *heapList = NULL;

/* RecordHeap: add given heap to list */
static void RecordHeap(MemHeap *x)
{
   MemHeapRec *p;
   
   if ((p=(MemHeapRec *)malloc(sizeof(MemHeapRec))) == NULL)
      HError(5105,"RecordHeap: Cannot allocate memory for MemHeapRec");
   p->heap = x; p->next = heapList;
   heapList = p;
}

/* UnRecordHeap: remove given heap from list */
static void UnRecordHeap(MemHeap *x)
{
   MemHeapRec *p, *q;
   
   p = heapList; q = NULL;
   while (p != NULL && p->heap != x){
      q = p;
      p = p->next;
   }
   if (p == NULL)
      HError(5171,"UnRecordHeap: heap %s not found",x->name);
   if (p==heapList) 
      heapList = p->next;
   else
      q->next = p->next;
   free(p);
}

/* AllocBlock: allocate and initialise a block for num items each of size */
static BlockP AllocBlock(size_t size, size_t num, HeapType type)
{
   BlockP p;
   ByteP c;
   int i;
   
   if (trace&T_TOP)
      printf("HMem: AllocBlock of %u bytes\n",num*size);
   if ((p = (BlockP) malloc(sizeof(Block))) == NULL)
      HError(5105,"AllocBlock: Cannot allocate Block");
   if ((p->data = (void *)malloc(size*num)) == NULL)
      HError(5105,"AllocBlock: Cannot allocate block data of %u bytes",size*num);
   switch (type){
   case MHEAP:
      if ((p->used = (ByteP)malloc((num+7)/8)) == NULL)
         HError(5105,"AllocBlock: Cannot allocate block used array");
      for (i=0,c=p->used; i < (num+7)/8; i++,c++) *c = 0;
      break;
   case MSTAK:
      p->used = NULL;
      break;
   default:
      HError(5190,"AllocBlock: bad type %d",type);
   }
   p->numElem = p->numFree = num; 
   p->firstFree=0; p->next=NULL;
   return p;
}    

/* BlockReorder: reorder blks so that one with n free elems/bytes is 1st */
static void BlockReorder(BlockP *p, int n)
{
   BlockP head,cur,prev;
   
   if (p==NULL) return;
   head = cur = *p; prev=NULL;
   while (cur != NULL) {
      if (cur->numFree >= n) {
         if (prev != NULL) {
            prev->next = cur->next;
            cur->next = head;
         }
         *p = cur;
         return;
      }
      prev = cur;
      cur = cur->next;
   }
}               

/* GetElem: return a pointer to the next free item in the block p */
static void *GetElem(BlockP p, size_t elemSize, HeapType type)
{
   int i,index;
   
   if (p == NULL) return NULL;
   switch (type){
   case MHEAP:
      /* firstFree indexes next free elem to get */
      if (p->numFree == 0) return NULL;
      index=p->firstFree;
      p->used[p->firstFree/8] |= 1<<(p->firstFree&7);
      p->numFree--;
      /* Look thru 'used' bitmap for next free elem */
      if (p->numFree > 0) {
         for (i=p->firstFree+1; i<p->numElem;i++)
            if ((p->used[i/8] & (1 <<(i&7))) == 0) {
               p->firstFree = i;
               break;
            }
      } else
         p->firstFree = p->numElem; /* one over the end */             
      return (void *)((ByteP)p->data+index*elemSize);
   case MSTAK:
      /* take elemSize bytes from top of stack */
      if (p->numFree < elemSize)  return NULL;
      index=p->firstFree;
      p->firstFree += elemSize;
      p->numFree -= elemSize;
      return (void *)((ByteP)p->data + index);
   default:
      HError(5190,"GetElem: bad type %d",type);
   }
   return NULL;  /* just to keep compiler happy */
}

/* EXPORT->InitMem: Initialise the module.  */
void InitMem(void)
{
   int i;
   Boolean b;
   
   Register(hmem_version, hmem_vc_id);
   CreateHeap(&gstack, "Global Stack",  MSTAK, 1, 0.0, 100000, ULONG_MAX ); /* #### should be max size_t */
   CreateHeap(&gcheap, "Global C Heap", CHEAP, 1, 0.0, 0,      0 );
   numParm = GetConfig("HMEM", TRUE, cParm, MAXGLOBS);
   if (numParm>0){
      if (GetConfInt(cParm,numParm,"TRACE",&i)) trace = i;
      if (GetConfBool(cParm,numParm,"PROTECTSTAKS",&b)) protectStaks = b;
   }
}

/* EXPORT->CreateHeap: create a memory heap with given characteristics */
void CreateHeap(MemHeap *x, char *name, HeapType type, size_t elemSize, 
                float growf, size_t numElem, size_t maxElem)
{
   char c=0;

   if (growf<0.0)
      HError(5170,"CreateHeap: -ve grow factor in heap %s",name);
   if (numElem>maxElem)
      HError(5170,"CreateHeap: init num elem > max elem in heap %s",name);
   if (elemSize <= 0)
      HError(5170,"CreateHeap: elem size = %u in heap %s",elemSize,name);
   if (type == MSTAK && elemSize !=1)
      HError(5170,"CreateHeap: elem size = %u in MSTAK heap %s",elemSize,name);
   x->name = (char *)malloc(strlen(name)+1);
   strcpy(x->name,name); /* cant use a MemHeap for this!! */
   x->type  = type; x->growf = growf;
   x->elemSize = elemSize;
   x->maxElem = maxElem;
   x->curElem = x->minElem = numElem;
   x->totUsed = x->totAlloc = 0;
   x->heap = NULL; 
   x->protectStk = (x==&gstack)?FALSE:protectStaks; 
   RecordHeap(x);
   if (trace&T_TOP){
      switch (type){
      case MHEAP: c='M'; break;
      case MSTAK: c='S'; break;
      case CHEAP: c='C'; break;
      }
      printf("HMem: Create Heap %s[%c] %u %.1f %u %u\n",name,c,
             elemSize, growf, numElem, maxElem);
   }
}

/* EXPORT->ResetHeap: Free all items from heap x */
void ResetHeap(MemHeap *x)
{
   BlockP cur,next;

   switch(x->type){
   case MHEAP:
      if (trace&T_TOP)
         printf("HMem: ResetHeap %s[M]\n",x->name);
      cur=x->heap;
      /* delete all blocks */
      while (cur != NULL) {
         next = cur->next;
         free(cur->data); free(cur->used);
         free(cur); cur = next;
      }
      x->curElem = x->minElem;
      x->totAlloc = 0; x->heap = NULL;
      break;
   case MSTAK:    
      if (trace&T_TOP)
         printf("HMem: ResetHeap %s[S]\n",x->name);
      cur=x->heap;       
      if (cur != NULL) {
         /* delete all blocks but first */
         while (cur->next != NULL) { 
            next = cur->next;
            x->totAlloc -= cur->numElem;
            free(cur->data); free(cur);
            cur = next;
         }
         x->heap = cur;
      }
      x->curElem = x->minElem;
      if (cur != NULL){
         cur->numFree = cur->numElem;
         cur->firstFree = 0;
      }
      break;
   case CHEAP:
      HError(5172,"ResetHeap: cannot reset C heap");
   }
   x->totUsed = 0;
}

/* EXPORT->DeleteHeap: delete given heap */
void DeleteHeap(MemHeap *x)
{
   if (x->type == CHEAP) 
      HError(5172,"DeleteHeap: cant delete C Heap %s",x->name);
   if (trace&T_TOP)
      printf("HMem: DeleteHeap %s\n",x->name);
   /* free all data blocks */
   ResetHeap(x);
   if (x->heap != NULL){
      free(x->heap->data);
      free(x->heap);
   }
   /* expunge all trace of it */
   UnRecordHeap(x);
   /* free name */
   free(x->name);
}

/* EXPORT->New: create a new element from heap x  */
void *New(MemHeap *x,size_t size)
{
   void *q;
   BlockP newp;
   size_t num,bytes,*ip,chdr;
   Boolean noSpace;
   Ptr *pp;
  
   if (x->elemSize <= 0)
      HError(5174,"New: heap %s not initialised",
             (x->name==NULL)? "Unnamed":x->name);
   switch(x->type){
   case MHEAP:
      /* Element is taken from first available slot in block list.  
         If none found a new block is allocated with num elems
         determined by the curElem, the grow factor growf and the
         upper limit maxElem. */
      if (size != 0 && size != x->elemSize)
         HError(5173,"New: MHEAP req for %u size elem from heap %s size %u",
                size,x->name,x->elemSize);

      noSpace = x->totUsed == x->totAlloc;
      if (noSpace || (q=GetElem(x->heap,x->elemSize,x->type)) == NULL) {
         if (!noSpace) BlockReorder(&(x->heap),1);
         if (noSpace || (q=GetElem(x->heap,x->elemSize,x->type)) == NULL) {
            num = (size_t) ((double)x->curElem * (x->growf + 1.0) + 0.5);
            if (num>x->maxElem) num = x->maxElem;
            newp = AllocBlock(x->elemSize, num, x->type);
            x->totAlloc += num; x->curElem = num;
            newp->next = x->heap;
            x->heap = newp;
            if ((q=GetElem(x->heap,x->elemSize,x->type)) == NULL)
               HError(5191,"New: null elem but just made block in heap %s",
                      x->name);
         }
      }
      x->totUsed++;
      if (trace&T_MHP)
         printf("HMem: %s[M] %u bytes at %p allocated\n",x->name,size,q);
      return q;
   case CHEAP:
      chdr = MRound(sizeof(size_t));
      q = malloc(size+chdr);
      if (q==NULL)
         HError(5105,"New: memory exhausted");
      x->totUsed += size; 
      x->totAlloc += size+chdr;
      ip = (size_t *)q; *ip = size;
      if (trace&T_CHP)
         printf("HMem: %s[C] %u+%u bytes at %p allocated\n",x->name,chdr,size,q);
      return (Ptr)((ByteP)q+chdr);
   case MSTAK:
      /* set required size - must alloc on double boundaries */
      if (x->protectStk) size += sizeof(Ptr);
      size = MRound(size);
      /* get elem from current block if possible */
      if ((q=GetElem(x->heap,size,x->type)) == NULL) {
         /* no space - so add a new (maybe bigger) block */
         bytes = (size_t)((double)x->curElem * (x->growf + 1.0) + 0.5);
         if (bytes > x->maxElem) bytes = x->maxElem;
         x->curElem = bytes;
         if (bytes < size) bytes = size;
         bytes = MRound(bytes);
         newp = AllocBlock(1, bytes, x->type);
         x->totAlloc += bytes; 
         newp->next = x->heap;
         x->heap = newp;
         if ((q=GetElem(x->heap,size,x->type)) == NULL)
            HError(5191,"New: null elem but just made block in heap %s",
                   x->name);
      }
      x->totUsed += size;
      if (trace&T_STK)
         printf("HMem: %s[S] %u bytes at %p allocated\n",x->name,size,q);
      if (x->protectStk) {
         pp = (Ptr *)((long)q + size - sizeof(Ptr)); /* #### fix this! */
         *pp = q;
      }
      return q;
   }
   return NULL;  /* just to keep compiler happy */
}


/* EXPORT->CNew: create a new element from heap x and initialise to zero */
Ptr CNew (MemHeap *x, size_t size)
{
   void *ptr;

   ptr = New (x, size);
   if (x->type == MHEAP && size ==0)
      size = x->elemSize;
   memset (ptr, 0, size);

   return ptr;
}

/* EXPORT->Dispose: Free item p from memory heap x */
void Dispose(MemHeap *x, void *p)
{
   BlockP head,cur,prev;
   Boolean found=FALSE;
   ByteP bp;
   size_t size,chdr;
   size_t num,index, *ip;
   Ptr *pp;
   
   if (x->totUsed == 0)
      HError(5105,"Dispose: heap %s is empty",x->name);
   switch(x->type){
   case MHEAP:
      head = x->heap; cur=head; prev=NULL;
      size = x->elemSize;
      while (cur != NULL && !found) {
         num = cur->numElem;
         found = cur->data <= p && 
            (((void*)((ByteP)cur->data+(num-1)*size)) >= p);
         if (!found) {
            prev=cur; cur=cur->next;
         }   
      }
      if (cur == NULL)
         HError(5175,"Dispose: Item to free in MHEAP %s not found",x->name);
      index = ((size_t)p-(size_t)cur->data)/size;
      cur->used[index/8] &= ~(1 <<(index&7));
      if (index < cur->firstFree) cur->firstFree = index;
      cur->numFree++; x->totUsed--;
      if (cur->numFree == cur->numElem) { 
         if (cur != head)                /* free the whole block */
            prev->next = cur->next;
         else
            head = cur->next;
         x->heap = head; x->totAlloc -= cur->numElem;
         free(cur->data); free(cur->used); free(cur);
      }
      if (trace&T_MHP)
         printf("HMem: %s[M] %u bytes at %p de-allocated\n",x->name,size,p);
      return;
   case MSTAK:
      /* search for item to dispose */
      cur = x->heap;
      if (x->protectStk){
         if (cur->firstFree > 0 ) /* s-top in current block */
            pp = (Ptr *)((size_t)cur->data+cur->firstFree-sizeof(Ptr));
         else{                      /* s-top in previous block */
            if (cur->next == NULL)
               HError(5175,"Dispose: empty stack");
            pp = (Ptr *)((size_t)cur->next->data+cur->next->firstFree-sizeof(Ptr));
         }
         if (*pp != p)
            HError(-5175,"Dispose: violation of stack discipline in %s [%p != %p]",
                   x->name, *pp, p);
      }
      while (cur != NULL && !found){
         /* check current block */
         num = cur->numElem;
         found = cur->data <= p && 
            (((void*)((ByteP)cur->data+num)) > p);
         if (!found) {     /* item not in cur block so delete it */
            x->heap = cur->next;
            x->totAlloc -= cur->numElem;
            x->totUsed -= cur->firstFree;
            free(cur->data);
            free(cur);
            cur = x->heap;
            if (trace&T_STK)
               printf("HMem: deleleting block in %s[S]\n",x->name);
         }
      }
      if (!found) 
         HError(5175,"Dispose: Item to free in MSTAK %s not found",x->name);
      /* finally cut back the stack in the current block */
      size = ((ByteP)cur->data + cur->firstFree) - (ByteP)p;
      if (((ByteP)cur->data + cur->firstFree) < (ByteP)p)
         HError(5175,"Dispose: item to free in MSTAK %s is above stack top",
                x->name);
      cur->firstFree -= size;
      cur->numFree += size; x->totUsed -= size;
      if (trace&T_STK)
         printf("HMem: %s[S] %u bytes at %p de-allocated\n",x->name,size,p);
      return;
   case CHEAP:
      chdr = MRound(sizeof(size_t));
      bp = (ByteP)p-chdr;
      ip = (size_t *)bp;
      x->totAlloc -= (*ip + chdr); x->totUsed -= *ip;
      if (trace&T_CHP)
         printf("HMem: %s[C] %u+%u bytes at %p de-allocated\n",
                x->name,chdr,*ip,bp);
      free(bp);
      return;
   }
}

/* EXPORT->PrintHeapStats: print summary stats for given memory heap */
void PrintHeapStats(MemHeap *x)
{
   char tc=0;
   BlockP p;
   int nBlocks = 0;
   
   switch (x->type){
   case MHEAP: tc = 'M'; break;
   case MSTAK: tc = 'S'; break;
   case CHEAP: tc = 'C'; break;
   }
   for (p=x->heap; p != NULL; p = p->next) ++nBlocks;
   printf("nblk=%3d, siz=%6u*%-3u, used=%9u, alloc=%9u : %s[%c]\n",
          nBlocks, x->curElem, x->elemSize, x->totUsed, 
          x->totAlloc*x->elemSize,x->name,tc) ;
   fflush(stdout);
}

/* EXPORT->PrintAllHeapStats: print summary stats for all memory heaps */
void PrintAllHeapStats(void)
{
   MemHeapRec *p;
   
   printf("\n---------------------- Heap Statistics ------------------------\n");
   for (p = heapList; p != NULL; p = p->next)
      PrintHeapStats(p->heap);
   printf(  "---------------------------------------------------------------\n");
}

/* ------------- Vector/Matrix Memory Management -------------- */

/*
  Vectors are pointers to arrays of float (ie float*); matrices are pointers
  to an array of vectors (ie float**).  All indexing is v[1..n] and 
  m[1..r][1..c].  The actual size of each vector is stored in v[0].  For 
  matrices, the row lengths (number of columns) are stored in every 
  row since they are genuine vectors, the number of rows is stored in m[0].
  Triangular matrices are the same except that the rows increase in length
  with the first row being a vector of one element.
  Short and IntVecs are arranged in a similar way to Vectors.
   
  Shared vectors and matrices have an extra 2 * sizeof(Ptr) bytes 
  prepended to hold a usage count and a hook.
*/

/* EXPORT->vectorElemSize: size of vectors for creating heaps */
size_t ShortVecElemSize(int size) { return (size+1)*sizeof(short); }
size_t IntVecElemSize(int size) { return (size+1)*sizeof(int); }
size_t VectorElemSize(int size) { return (size+1)*sizeof(float); }
size_t DVectorElemSize(int size){ return (size+1)*sizeof(double);}
size_t SVectorElemSize(int size){ return (size+1)*sizeof(float)+2*sizeof(Ptr); }

/* EXPORT->CreateShortVec:  Allocate space for short array v[1..size] */
ShortVec CreateShortVec(MemHeap *x,int size)
{
   short *v;
   
   v = (short *)New(x,ShortVecElemSize(size));
   *v = size;
   return (ShortVec)v;
}


/* EXPORT->CreateIntVec:  Allocate space for int array v[1..size] */
IntVec CreateIntVec(MemHeap *x,int size)
{
   int *v;
   
   v = (int *)New(x,IntVecElemSize(size));
   *v = size;
   return (IntVec)v;
}


/* EXPORT->CreateVector:  Allocate space for vector v[1..size] */
Vector CreateVector(MemHeap *x, int size)
{
   Vector v;
   int *i;
   
   v = (Vector)New(x,VectorElemSize(size));
   i = (int *) v; *i = size;
   return v;
}

/* EXPORT->CreateDVector:  Allocate space for double vector v[1..size] */
DVector CreateDVector(MemHeap *x, int size)
{
   DVector v;
   int *i;
   
   v = (DVector)New(x,DVectorElemSize(size));
   i = (int *) v; *i = size;
   return v;
}

/* EXPORT->CreateSVector:  Shared version */
Vector CreateSVector(MemHeap *x, int size)
{
   SVector v;
   Ptr *p;
   int *i;
   
   p = (Ptr *)New(x,SVectorElemSize(size));
   v = (SVector) (p+2);
   i = (int *) v; *i = size;
   SetHook(v,NULL);
   SetUse(v,0);
   return v;
}

/* EXPORT->ShortVecSize: returns number of components in v */
int ShortVecSize(ShortVec v)
{
   return (int)(*v);
}

/* EXPORT->IntVecSize: returns number of components in v */
int IntVecSize(IntVec v)
{
   return *v;
}

/* EXPORT->VectorSize: returns number of components in v */
int VectorSize(Vector v)
{
   int *i;
   
   i = (int *) v;
   return *i;
}

/* EXPORT->DVectorSize: returns number of components in v */
int DVectorSize(DVector v)
{
   int *i;
   
   i = (int *) v;
   return *i;
}

/* EXPORT->FreeShortVec: Free space allocated for short vector v */
void FreeShortVec(MemHeap *x,ShortVec v)
{
   Dispose(x,v);
}

/* EXPORT->FreeIntVec: Free space allocated for int vector v */
void FreeIntVec(MemHeap *x,IntVec v)
{
   Dispose(x,v);
}

/* EXPORT->FreeVector: Free space allocated for vector v[1..size] */
void FreeVector(MemHeap *x, Vector v)
{
   Dispose(x,v);
}

/* EXPORT->FreeDVector: Free space allocated for vector v[1..size] */
void FreeDVector(MemHeap *x, DVector v)
{
   Dispose(x,v);
}

/* EXPORT->FreeSVector: Free space allocated for vector v[1..size] */
void FreeSVector(MemHeap *x, Vector v)
{
   DecUse(v);
   if (GetUse(v) <= 0)
      Dispose(x,(Ptr *)(v)-2);
}

/* EXPORT->MatrixElemSize: size of matrices for creating heaps */
size_t MatrixElemSize(int nrows,int ncols)
{
   return VectorElemSize(ncols) * nrows + (nrows+1)*sizeof(Vector);
}
size_t DMatrixElemSize(int nrows,int ncols)
{
   return MRound(DVectorElemSize(ncols) * nrows + (nrows+1)*sizeof(DVector));
}
size_t SMatrixElemSize(int nrows,int ncols)
{
   return VectorElemSize(ncols) * nrows + (nrows+3)*sizeof(Vector);
}
size_t TriMatElemSize(int size)
{
   return size*(VectorElemSize(0)*2 + (size+1)*sizeof(float))/2
      + (size+1)*sizeof(Vector);
}
size_t STriMatElemSize(int size)
{
   return size*(VectorElemSize(0)*2 + (size+1)*sizeof(float))/2
      + (size+1)*sizeof(Vector) + 2*sizeof(Ptr);
}

/* EXPORT->CreateMatrix:  Allocate space for matrix m[1..nrows][1..ncols] */
Matrix CreateMatrix(MemHeap *x, int nrows, int ncols)
{
   size_t vsize;
   int *i,j;
   Vector *m;   
   char *p;
   
   p =(char *)  New(x,MatrixElemSize(nrows,ncols)); 
   i = (int *)p; *i = nrows;
   vsize = VectorElemSize(ncols);
   m = (Vector *)p;
   p += (nrows+1)*sizeof(Vector);
   for (j=1;j<=nrows; j++, p += vsize) {
      i = (int *) p; *i = ncols;
      m[j] = (Vector) p;
   }
   return m;
}

/* EXPORT->CreateTriMat:  Allocate space for matrix m[1..size][1..i] */
TriMat CreateTriMat(MemHeap *x,int size)
{
   int *i,j;
   Vector *m;   
   char *p;
      
   p = (char *) New(x,TriMatElemSize(size)); 
   i = (int *)p; *i = size;
   m = (Vector *)p;
   p += (size+1)*sizeof(Vector);
   for (j=1;j<=size; j++) {
      i = (int *) p; *i = j;
      m[j] = (Vector) p; p += VectorElemSize(j);
   }
   return m;
}

/* EXPORT->CreateDMatrix:  Allocate space for double matrix m[1..nrows][1..ncols] */
DMatrix CreateDMatrix(MemHeap *x, int nrows,int ncols)
{
   size_t vsize;
   int *i,j;
   DVector *m;   
   char *p;
   
   p = (char *) New(x,DMatrixElemSize(nrows,ncols));
   i = (int *) p; *i = nrows;
   vsize = DVectorElemSize(ncols);
   m = (DVector *) p;
   p += MRound((nrows+1)*sizeof(DVector));
   for (j=1; j<=nrows; j++, p += vsize) {
      i = (int *) p; *i = ncols;
      m[j] = (DVector) p;
   }
   return m;
}

/* EXPORT->CreateSMatrix:  Allocate space for matrix m[1..nrows][1..ncols] */
Matrix CreateSMatrix(MemHeap *x, int nrows,int ncols)
{
   size_t vsize;
   int *i,j;
   Vector *m;   
   char *p;
   
   p = (char *)New(x,SMatrixElemSize(nrows,ncols)) + 2*sizeof(Ptr *); 
   i = (int *)p; *i = nrows;
   vsize = VectorElemSize(ncols);
   m = (Vector *)p;
   p += (nrows+1)*sizeof(Vector);
   for (j=1;j<=nrows; j++, p += vsize) {
      i = (int *) p; *i = ncols;
      m[j] = (Vector) p;
   }
   SetHook(m,NULL);
   SetUse(m,0);
   return m;
}

/* EXPORT->CreateSTriMat:  Allocate space for matrix m[1..size][1..i] */
STriMat CreateSTriMat(MemHeap *x,int size)
{
   int *i,j;
   Vector *m;   
   char *p;
      
   p = (char *)New(x,STriMatElemSize(size)) + 2*sizeof(Ptr *); 
   i = (int *)p; *i = size;
   m = (Vector *)p;
   p += (size+1)*sizeof(Vector);
   for (j=1;j<=size; j++) {
      i = (int *) p; *i = j;
      m[j] = (Vector) p; p += VectorElemSize(j);
   }
   SetHook(m,NULL);
   SetUse(m,0);
   return m;
}

/* EXPORT->IsTriMat: True if matrix is lower triangular */
Boolean IsTriMat(Matrix m)
{
   int i,n;

   n=NumRows(m);
   for(i=1;i<=n;i++)
      if (VectorSize(m[i])!=i) return(FALSE);
   return(TRUE);
}

/* EXPORT->NumRows: number of rows in matrix m */
int NumRows(Matrix m)
{
   int *nrows;
   
   nrows = (int *) m;
   return *nrows;
}

/* EXPORT->NumCols: number of columns in matrix m */
int NumCols(Matrix m)
{
   int *ncols;
   
   ncols = (int *) m[1];
   return *ncols;
}

/* EXPORT->NumDRows: number of rows in double matrix m */
int NumDRows(DMatrix m)
{
   int *nrows;
   
   nrows = (int *) m;
   return *nrows;
}

/* EXPORT->NumDCols: number of columns in double matrix m */
int NumDCols(DMatrix m)
{
   int *ncols;
   
   ncols = (int *) m[1];
   return *ncols;
}

/* EXPORT->TriMatSize: number of rows/cols in triangular matrix m */
int TriMatSize(TriMat m)
{
   int *nrows;
   
   nrows = (int *) m;
   return *nrows;
}

/* EXPORT->FreeMatrix: Free space allocated for matrix m */
void FreeMatrix(MemHeap *x, Matrix m)
{
   Dispose(x,m);
}

/* EXPORT->FreeDMatrix: Free space allocated for matrix m */
void FreeDMatrix(MemHeap *x, DMatrix m)
{
   Dispose(x,m);
}

/* EXPORT->FreeSMatrix: Free space allocated for matrix m */
void FreeSMatrix(MemHeap *x, Matrix m)
{
   DecUse(m);
   if (GetUse(m) <= 0)
      Dispose(x,(Ptr *)(m) - 2);
}

/* EXPORT->FreeTriMat: Free space allocated for tri matrix m */
void FreeTriMat(MemHeap *x,TriMat m)
{
   Dispose(x,m);
}

/* EXPORT->FreeSTriMat: Free space allocated for shared tri matrix m */
void FreeSTriMat(MemHeap *x,STriMat m)
{
   DecUse(m);
   if (GetUse(m) <= 0) 
      Dispose(x,(Ptr *)(m) - 2);
}

/* EXPORT->SetUse: set usage count of m to n */
void SetUse(Ptr m,int n)
{
   Ptr *p;
   
   p = (Ptr *) m; --p; *((int *)p) = n;
}

/* EXPORT->IncUse: Increment usage count of m by one */
void IncUse(Ptr m)
{
   Ptr *p;
   
   p = (Ptr *) m; --p; ++(*((int *)p));
}

/* EXPORT->DecUse: Decrement usage count of m by one */
void DecUse(Ptr m)
{
   Ptr *p;
   
   p = (Ptr *) m; --p; --(*((int *)p));
}

/* EXPORT->GetUse: return usage count of m */
int  GetUse(Ptr m)
{
   Ptr *p;
   
   p = (Ptr *) m; --p; return *((int *)p);
}

/* EXPORT->SetHook: set hook of m to p */
void SetHook(Ptr m, Ptr ptr)
{
   Ptr *p;
   
   p = (Ptr *) m; p -= 2; *p = ptr;
}

/* EXPORT->GetHook: return hook of m */
Ptr GetHook(Ptr m)
{
   Ptr *p;
   
   p = (Ptr *) m; p -=2; return *p;
}

/* EXPORT->IsSeenV: return true if seen */
Boolean IsSeenV(Ptr m)
{
   Ptr *p;
   int i;
   
   p = (Ptr *) m; --p; i = *((int *)p);
   return i<0;
}

/* EXPORT->TouchV: mark use flag as seen */
void TouchV(Ptr m)
{
   Ptr *p;
   int i;
   
   p = (Ptr *) m; --p; i = *((int *)p);
   if (i==0)
      *((int *)p) = INT_MIN;
   else if (i>0)
      *((int *)p) = -i;
}

/* EXPORT->UntouchV: mark use flag as unseen */
void UntouchV(Ptr m)
{
   Ptr *p;
   int i;
   
   p = (Ptr *) m; --p; i = *((int *)p);
   if (i==INT_MIN)
      *((int *)p) = 0;
   else if (i<0)
      *((int *)p) = -i;
}

/* ------------------ String Memory Management ----------------- */

/* EXPORT->NewString: return a string of given size */   
char *NewString(MemHeap *x, int size)
{
   char *s;
   
   s = (char *) New(x,size+1);
   *s='\0'; /* make it empty */
   return s;
}

/* EXPORT->CopyString: return a copy of string s */   
char *CopyString(MemHeap *x, char *s)
{
   char *t;
   
   t = (char *) New(x,strlen(s)+1);
   strcpy(t,s);
   return t;
}

/* -------------------------- End of HMem.c ---------------------------- */
