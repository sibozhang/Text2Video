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
/*         File: HMem.h:   Memory Management Module            */
/* ----------------------------------------------------------- */

/* !HVER!HMem:   3.4.1 [CUED 12/03/09] */

/*
   This module provides a type MemHeap which once initialised
   acts as a memory heap.  Every heap has a name which is used 
   in error messages and a type:
   
      MHEAP    = fixed size objects, with random order 
                 new/free operations and global reset
             
      MSTAK    = variable size objects with LIFO order 
                 new/free operations and global reset
             
      CHEAP    = variable size objects with random order 
                 new/free operations but no reset (this uses 
                 malloc and free directly)
             
   Storage for each heap (except CHEAP) is allocated in blocks.
   Blocks grow according to the growf(actor up to a specified limit.
   When items are freed from a MHEAP heap and a block becomes empty 
   then the block is free'd.  Every item in a heap can be freed via the 
   ResetHeap function.  For MSTAK heaps this is a very low cost
   operation.
   
   On top of the above basic memory types, this module defines
   vector, matrix and string memory manipulation routines.
*/

#ifndef _HMEM_H_
#define _HMEM_H_

#ifdef __cplusplus
extern "C" {
#endif

/* ----------------- Define Memory Management Functions ---------------- */

typedef enum{MHEAP, MSTAK, CHEAP} HeapType;
typedef unsigned char * ByteP;
typedef void * Ptr;

typedef struct _Block *BlockP;

typedef struct _Block{  /*      MHEAP                     MSTAK           */
   size_t numFree;      /* #free elements            #free bytes          */
   size_t firstFree;    /* idx of 1st free elem      idx of stack top     */
   size_t numElem;      /* #elems in blk             #bytes in blk        */
   ByteP used;          /* alloc map, 1 bit/elem         not used         */
   Ptr   data;          /*        actual data for this block              */
   BlockP next;         /*           next block in chain                  */
} Block;

typedef struct {
   char *name;          /*            name of this memory heap            */
   HeapType type;       /*              type of this heap                 */
   float growf;         /*           succ blocks grow as 1+growf          */
   size_t elemSize;     /*  size of each elem              1 always       */
   size_t minElem;      /*  init #elems per blk      init #bytes per blk  */
   size_t maxElem;      /*  max #elems per block     max #bytes per blk   */
   size_t curElem;      /*  current #elems per blk   curr #bytes per blk  */
   size_t totUsed;      /*  total #elems used        total #bytes used    */
   size_t totAlloc;     /*  total #elems alloc'ed    total #bytes alloc'd */
   BlockP heap;         /*               linked list of blocks            */
   Boolean protectStk;  /*  MSTAK only, prevents disposal below Stack Top */
}MemHeap;

/* ---------------------- Alignment Issues -------------------------- */

size_t MRound(size_t size);
/*
   Round size to align elements of array on full word boundary.
*/

/* ---------------- General Purpose Memory Management ---------------- */

extern MemHeap gstack;  /* global MSTAK for general purpose use */
extern MemHeap gcheap;  /* global CHEAP for general purpose use */

void InitMem(void);
/*
   Initialise the module.  This routine must be called before any other
   routine in this module
*/

void CreateHeap(MemHeap *x, char *name, HeapType type, size_t elemSize, 
                float growf, size_t numElem,  size_t maxElem);
/*
   Create a memory heap x for elements of size elemSize and numElem in
   first block.  If type is MSTAK or CHEAP then elemSize should be 1.
*/

void ResetHeap(MemHeap *x);
/*
   Frees all items currently allocated from the given heap. Fails
   if type is CHEAP.
*/

void DeleteHeap(MemHeap *x);
/*
   Delete given heap and all associated data structures.
*/

Ptr New(MemHeap *x,size_t size);
/*
   Allocate and return a new element of given size from memory heap x.
   If heap is MHEAP then the size can be 0 in which case it is
   ignored, otherwise it must be the correct size for that heap.  Note
   that New aborts if requests cannot be satisfied.  It never
   returns NULL.
*/

Ptr CNew(MemHeap *x,size_t size);
/*
  Create a new element from heap x and initialise to zero. See
  comment for New() above.
*/

void Dispose(MemHeap *x, Ptr p);
/*
   Free the element pointed to by p from memory heap x
*/

void PrintHeapStats(MemHeap *x);
/* 
   Print summary stats for given memory heap 
*/

void PrintAllHeapStats(void);
/* 
   Print summary stats for all allocated heaps
*/

/* ------------- Vector/Matrix Memory Management -------------- */

/* Basic Numeric Types */
typedef short *ShortVec;   /* short vector[1..size] */
typedef int   *IntVec;     /* int vector[1..size] */
typedef float *Vector;     /* vector[1..size]   */
typedef float **Matrix;    /* matrix[1..nrows][1..ncols] */
typedef Matrix TriMat;     /* matrix[1..nrows][1..i] (lower triangular) */
typedef double *DVector;   /* double vector[1..size]   */
typedef double **DMatrix;  /* double matrix[1..nrows][1..ncols] */

/* Shared versions */
typedef Vector SVector;    /* shared vector[1..size]   */
typedef Matrix SMatrix;    /* shared matrix[1..nrows][1..ncols] */
typedef Matrix STriMat;    /* shared matrix[1..nrows][1..i] (lower tri) */

size_t ShortVecElemSize(int size);
size_t IntVecElemSize(int size);
size_t VectorElemSize(int size);
size_t DVectorElemSize(int size);
size_t SVectorElemSize(int size);
/* 
   Return elemSize of a vector with size components.  These
   functions should be used for creating MHEAP heaps
*/

ShortVec CreateShortVec(MemHeap *x,int size);
IntVec   CreateIntVec(MemHeap *x,int size);
Vector   CreateVector(MemHeap *x,int size);
DVector  CreateDVector(MemHeap *x,int size);
SVector  CreateSVector(MemHeap *x,int size);
/*
   Create and return a vector of size components.
   The SVector version prepends 8 bytes for use by HModel's shared
   data structure mechanism.
*/

int ShortVecSize(ShortVec v);
int IntVecSize(IntVec v);
int VectorSize(Vector v);
int DVectorSize(DVector v);
/*
   Return the number of components in vector v
*/

void FreeShortVec(MemHeap *x,ShortVec v);
void FreeIntVec(MemHeap *x,IntVec v);
void FreeVector(MemHeap *x,Vector v);
void FreeDVector(MemHeap *x,DVector v);
void FreeSVector(MemHeap *x,SVector v);
/*
   Free the memory allocated for vector v
*/

size_t MatrixElemSize(int nrows,int ncols);
size_t DMatrixElemSize(int nrows,int ncols);
size_t SMatrixElemSize(int nrows,int ncols);
size_t TriMatElemSize(int size);
size_t STriMatElemSize(int size);
/* 
   Return elemSize of a Matrix with given number of rows
   and columns, or Triangular Matrix with size rows and
   columns.  These functions should be used for creating 
   MHEAP heaps.
*/
Matrix  CreateMatrix(MemHeap *x,int nrows,int ncols);
DMatrix CreateDMatrix(MemHeap *x,int nrows,int ncols);
SMatrix CreateSMatrix(MemHeap *x,int nrows,int ncols);
TriMat  CreateTriMat(MemHeap *x,int size);
STriMat CreateSTriMat(MemHeap *x,int size);
/*
   Create and return a matrix with nrows rows and ncols columns.
   The S version prepends 8 bytes for use by HModel's shared
   datastructure mechanism. The TriMat versions allocate only the 
   lower triangle of a square matrix.
*/

Boolean IsTriMat(Matrix m);
/*
   Return true if m is actually TriMat
*/

int NumRows(Matrix m);
int NumDRows(DMatrix m);
int NumCols(Matrix m);
int NumDCols(DMatrix m);
int TriMatSize(TriMat m);
/*
   Return the number of rows/cols in matrix m.  These can be
   applied to shared variants also.
*/

void FreeMatrix(MemHeap *x,Matrix m);
void FreeDMatrix(MemHeap *x,DMatrix m);
void FreeSMatrix(MemHeap *x,SMatrix m);
void FreeTriMat(MemHeap *x,TriMat m);
void FreeSTriMat(MemHeap *x,STriMat m);
/*
   Free the space occupied by matrix m
*/

void SetUse(Ptr m,int n);
void IncUse(Ptr m);
void DecUse(Ptr m);
int  GetUse(Ptr m);
/*
   Access to Usage count attached to Shared Vector/Matrix m 
*/

Boolean IsSeenV(Ptr m);
void TouchV(Ptr m);
void UntouchV(Ptr m);
/*
   Set/clear/check nUse as "seen" flag
*/

void SetHook(Ptr m, Ptr ptr);
Ptr GetHook(Ptr m);
/*
   Access to Hook attached to Shared Vector/Matrix m 
*/

/* ------------------ String Memory Management ----------------- */


char *NewString(MemHeap *x, int size);
/*
   Returns a pointer to a string of given size
*/

char *CopyString(MemHeap *x, char *s);
/*
   Returns a pointer to a copy of string s
*/

#ifdef __cplusplus
}
#endif

#endif  /* _HMEM_H_ */

/* -------------------------- End of HMem.h ---------------------------- */
