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
/*         File: HMath.c   Math Support Module                 */
/* ----------------------------------------------------------- */

char *hmath_version = "!HVER!HMath:   3.4.1 [CUED 12/03/09]";
char *hmath_vc_id = "$Id: HMath.c,v 1.1.1.1 2006/10/11 09:54:58 jal58 Exp $";

/*
   This library provides math support in the following three areas
   
      a) Vector/Matrix Operators
      b) Log Arithmetic
      c) Random Deviates
      
   It relies on the basic vector and matrix types defined by HMem.
   The separation of functionality betwee HMem and HMath is such
   that no routine in this module requires any knowledge of the
   hidden fields in these types.  Thus, if a change of representation
   is required, it should only be necessary to change HMem.
*/

#include "HShell.h"        /* HTK Libraries */
#include "HMem.h"
#include "HMath.h"

/* ----------------------------- Trace Flags ------------------------- */

static int trace = 0;

/* -------------------- Configuration Parameters --------------------- */

static ConfParam *cParm[MAXGLOBS];       /* config parameters */
static int numParm = 0;

/* ------------------ Vector Oriented Routines ----------------------- */

/*
   ShortVecs and IntVecs are pointers to arrays of short/int.  Vectors
   are pointers to arrays of float (ie float*).  All indexing is
   v[1..n].  The size is stored in v[0].
*/

/* EXPORT->ZeroShortVec: Zero the elements of v */
void ZeroShortVec(ShortVec v)
{
   int i,n;
   
   n=ShortVecSize(v);
   for (i=1;i<=n;i++) v[i]=0;
}

/* EXPORT->ZeroIntVec: Zero the elements of v */
void ZeroIntVec(IntVec v)
{
   int i,n;
   
   n=IntVecSize(v);
   for (i=1;i<=n;i++) v[i]=0;
}

/* EXPORT->ZeroVector: Zero the elements of v */
void ZeroVector(Vector v)
{
   int i,n;
   
   n=VectorSize(v);
   for (i=1;i<=n;i++) v[i]=0.0;
}

/* EXPORT->ZeroDVector: Zero the elements of v */
void ZeroDVector(DVector v)
{
   int i,n;
   
   n=DVectorSize(v);
   for (i=1;i<=n;i++) v[i]=0.0;
}

/* EXPORT->CopyShortVec: copy v1 into v2 */
void CopyShortVec(ShortVec v1, ShortVec v2)
{
   int i,size; 
   
   size = ShortVecSize(v1);
   if (size != ShortVecSize(v2))
      HError(5270,"CopyShortVec: sizes differ %d vs %d",
             size,ShortVecSize(v2));
   for (i=1; i<=size; i++) 
      v2[i] = v1[i];
}

/* EXPORT->CopyIntVec: copy v1 into v2 */
void CopyIntVec(IntVec v1, IntVec v2)
{
   int i,size; 
   
   size = IntVecSize(v1);
   if (size != IntVecSize(v2))
      HError(5270,"CopyIntVec: sizes differ %d vs %d",
             size,IntVecSize(v2));
   for (i=1; i<=size; i++) 
      v2[i] = v1[i];
}

/* EXPORT->CopyVector: copy v1 into v2 */
void CopyVector(Vector v1, Vector v2)
{
   int i,size; 
   
   size = VectorSize(v1);
   if (size != VectorSize(v2))
      HError(5270,"CopyVector: sizes differ %d vs %d",
             size,VectorSize(v2));
   for (i=1; i<=size; i++) 
      v2[i] = v1[i];
}

/* EXPORT->CopyDVector: copy v1 into v2 */
void CopyDVector(DVector v1, DVector v2)
{
   int i,size; 
   
   size = DVectorSize(v1);
   if (size != DVectorSize(v2))
      HError(5270,"CopyDVector: sizes differ %d vs %d",
             size,DVectorSize(v2));
   for (i=1; i<=size; i++) 
      v2[i] = v1[i];
}

/* EXPORT->ReadShortVec: read vector from src in ascii or binary */ 
Boolean ReadShortVec(Source *src, ShortVec v, Boolean binary)
{
   return ReadShort(src,v+1,ShortVecSize(v),binary);
}

/* EXPORT->ReadIntVec: read vector from src in ascii or binary */ 
Boolean ReadIntVec(Source *src, IntVec v, Boolean binary)
{
   return ReadInt(src,v+1,IntVecSize(v),binary);
}

/* EXPORT->ReadVector: read vector from src in ascii or binary */ 
Boolean ReadVector(Source *src, Vector v, Boolean binary)
{
   return ReadFloat(src,v+1,VectorSize(v),binary);
}

/* EXPORT->WriteShortVec: write vector v to stream f */
void WriteShortVec(FILE *f, ShortVec v, Boolean binary)
{
   WriteShort(f,v+1,ShortVecSize(v),binary);
   if (!binary) fputc('\n',f);
}

/* EXPORT->WriteIntVec: write vector v to stream f */
void WriteIntVec(FILE *f, IntVec v, Boolean binary)
{
   WriteInt(f,v+1,IntVecSize(v),binary);
   if (!binary) fputc('\n',f);
}

/* EXPORT->WriteVector: write vector v to stream f */
void WriteVector(FILE *f, Vector v, Boolean binary)
{
   WriteFloat(f,v+1,VectorSize(v),binary);
   if (!binary) fputc('\n',f);
}

/* Export->ShowShortVec: show the short vector v preceded by title */
void ShowShortVec(char * title, ShortVec v,int maxTerms)
{
   int i, size, maxi;
   
   size = maxi = ShortVecSize(v);
   if (maxi>maxTerms) maxi=maxTerms;
   printf("%s\n   ",title);   
   for (i=1;i<=maxi;i++)  printf("%3d ",v[i]);
   if (maxi<size) printf("...");
   printf("\n");
}

/* Export->ShowIntVec: show the int vector v preceded by title */
void ShowIntVec(char * title, IntVec v,int maxTerms)
{
   int i, size, maxi;
   
   size = maxi = IntVecSize(v);
   if (maxi>maxTerms) maxi=maxTerms;
   printf("%s\n   ",title);   
   for (i=1;i<=maxi;i++)  printf("%5d ",v[i]);
   if (maxi<size) printf("...");
   printf("\n");
}

/* Export->ShowVector: show the vector v preceded by title */
void ShowVector(char * title,Vector v,int maxTerms)
{
   int i, size, maxi;
   
   size = maxi = VectorSize(v);
   if (maxi>maxTerms) maxi=maxTerms;
   printf("%s\n   ",title);   
   for (i=1;i<=maxi;i++)  printf("%8.2f ",v[i]);
   if (maxi<size) printf("...");
   printf("\n");
}

/* Export->ShowDVector: show the vector v preceded by title */
void ShowDVector(char * title, DVector v,int maxTerms)
{
   int i, size, maxi;
   
   size = maxi = DVectorSize(v);
   if (maxi>maxTerms) maxi=maxTerms;
   printf("%s\n   ",title);   
   for (i=1;i<=maxi;i++)  printf("%10.4f ",v[i]);
   if (maxi<size) printf("...");
   printf("\n");
}

/* ------------------ Matrix Oriented Routines ----------------------- */

/*
  Matrices are pointers to an array of vectors (ie float**).  Matrices
  are indexed m[1..r][1..c].  The rows of matrix are genuine vectors
  and can be treated as such except that matrices must be disposed of
  in their entirety.  If the rows of a matrix are substituted by
  other vectors then it should not be disposed.  TriMats are indexed
  m[1..r][1..i] where i is the row number ie only the lower triangle
  is stored.
*/

/* EXPORT->ZeroMatrix: Zero the elements of m */
void ZeroMatrix(Matrix m)
{
   int i,j,nr,nc;
   
   nr=NumRows(m); nc=NumCols(m);
   for (i=1;i<=nr;i++)
      for (j=1;j<=nc;j++) m[i][j]=0.0;
}

/* EXPORT->ZeroDMatrix: Zero the elements of m */
void ZeroDMatrix(DMatrix m)
{
   int i,j,nr,nc;
   
   nr=NumDRows(m); nc=DVectorSize(m[1]);
   for (i=1;i<=nr;i++)
      for (j=1;j<=nc;j++) m[i][j]=0.0;
}

/* EXPORT->ZeroTriMat: Zero the elements of m */
void ZeroTriMat(TriMat m)
{
   int i,j,size;
   
   size = TriMatSize(m);
   for (i=1;i<=size;i++)
      for (j=1;j<=i;j++) m[i][j]=0.0;
}

/* EXPORT->CopyMatrix: copy matrix m1 to m2 */
void CopyMatrix(Matrix m1, Matrix m2)
{
   int i,nrows;
   
   nrows = NumRows(m1);
   if (nrows != NumRows(m2))
      HError(5270,"CopyMatrix: row sizes differ %d vs %d",
             nrows,NumRows(m2));
   for (i=1; i<=nrows; i++)
      CopyVector(m1[i],m2[i]);
}

/* EXPORT->CopyDMatrix: copy matrix m1 to m2 */
void CopyDMatrix(DMatrix m1, DMatrix m2)
{
   int i,nrows;
   
   nrows = NumDRows(m1);
   if (nrows != NumDRows(m2))
      HError(5270,"CopyDMatrix: row sizes differ %d vs %d",
             nrows,NumDRows(m2));
   for (i=1; i<=nrows; i++)
      CopyDVector(m1[i],m2[i]);
}

/* EXPORT->CopyTriMat: copy triangular matrix m1 to m2 */
void CopyTriMat(TriMat m1, TriMat m2)
{
   int i,size;
   
   size = TriMatSize(m1);
   if (size != TriMatSize(m2))
      HError(5270,"CopyTriMat: sizes differ %d vs %d",
             size,TriMatSize(m2));
   for (i=1; i<=size; i++)
      CopyVector(m1[i],m2[i]);
}

/* EXPORT->Mat2DMat: convert matrix m1 to double matrix m2 */
void Mat2DMat(Matrix m1,  DMatrix m2)
{
   int i,j,nrows,ncols;

   nrows = NumRows(m1);
   if (nrows != NumDRows(m2))
      HError(5270,"Mat2DMat: row sizes differ %d vs %d",
             nrows,NumDRows(m2));
   ncols = NumCols(m1);
   if (ncols != NumDCols(m2))
      HError(5270,"Mat2DMat: col sizes differ %d vs %d",
             ncols,NumDCols(m2));
   for (i=1; i<=nrows; i++)
      for (j=1; j<=ncols; j++) 
         m2[i][j] = m1[i][j];
}

/* EXPORT->DMat2Mat: convert double matrix m1 to matrix m2 */
void DMat2Mat(DMatrix m1, Matrix m2)
{
   int i,j,nrows,ncols;

   nrows = NumDRows(m1);
   if (nrows != NumRows(m2))
      HError(5270,"DMat2Mat: row sizes differ %d vs %d",
             nrows,NumRows(m2));
   ncols = NumDCols(m1);
   if (ncols != NumCols(m2))
      HError(5270,"DMat2Mat: col sizes differ %d vs %d",
             ncols,NumCols(m2));
   for (i=1; i<=nrows; i++)
      for (j=1; j<=ncols; j++) 
         m2[i][j] = m1[i][j];
}

/* EXPORT->Mat2Tri: convert matrix m1 to tri matrix m2 */
void Mat2Tri (Matrix m1,  TriMat m2)
{
   int i,j,nrows,ncols;

   nrows = NumRows(m1); ncols = NumCols(m1);
   if (nrows != ncols)
      HError(5270,"Mat2Tri: source matrix not square %d vs %d",
             nrows,ncols);   
   if (ncols != TriMatSize(m2))
      HError(5270,"Mat2Tri: sizes differ %d vs %d",
             ncols,TriMatSize(m2));
   for (i=1; i<=nrows; i++)
      for (j=1; j<=i; j++) 
         m2[i][j] = m1[i][j];
}

/* EXPORT->Tri2Mat: convert tri matrix m1 to matrix m2 */
void Tri2Mat (TriMat m1, Matrix m2)
{
   int i,j,nrows,ncols;

   nrows = NumRows(m2); ncols = NumCols(m2);
   if (nrows != ncols)
      HError(5270,"Tri2Mat: target matrix not square %d vs %d",
             nrows,ncols);   
   if (ncols != TriMatSize(m1))
      HError(5270,"Tri2Mat: sizes differ %d vs %d",
             TriMatSize(m1),ncols);
   for (i=1; i<=nrows; i++)
      for (j=1; j<=i; j++) {
         m2[i][j] = m1[i][j];
         if (i!=j) m2[j][i] = m1[i][j];
      }
}

/* EXPORT->ReadMatrix: read matrix from source into m */
Boolean ReadMatrix(Source *src, Matrix m, Boolean binary)
{
   int i,nrows;
   
   nrows = NumRows(m);
   for (i=1; i<=nrows; i++)
      if (!ReadVector(src,m[i],binary)) 
         return FALSE;
   return TRUE;
}

/* EXPORT->ReadTriMat: read symmetric matrix in lower triangular
                       form from source into m */
Boolean ReadTriMat(Source *src, TriMat m, Boolean binary)
{
   int i,j,size;
   
   size = TriMatSize(m);
   for (j=1; j<=size; j++) {
      for (i=j; i<=size; i++)
         if (!ReadFloat(src,&(m[i][j]),1,binary))
            return FALSE;
   }
   return TRUE;
}

/* EXPORT->WriteMatrix: write matrix to f */
void WriteMatrix(FILE *f, Matrix m, Boolean binary)
{
   int i,nrows;
   
   nrows = NumRows(m);
   for (i=1; i<=nrows; i++)
      WriteVector(f,m[i],binary);
}

/* EXPORT->WriteTriMat: write symmetric matrix to stream f in
                        upper triangular form */
void WriteTriMat(FILE *f, TriMat m, Boolean binary)
{
   int i,j,size;
   
   size = TriMatSize(m);
   for (j=1; j<=size; j++) {
      for (i=j; i<=size; i++)
         WriteFloat(f,&(m[i][j]),1,binary);
      if (!binary) fputc('\n',f);
   }
}

/* Export->ShowMatrix: show the matrix m preceded by title */
void ShowMatrix(char * title,Matrix m,int maxCols,int maxRows)
{
   int i,j;
   int maxi,maxj,nrows,ncols;
   
   maxi = nrows = NumRows(m);
   if (maxi>maxRows) maxi = maxRows;
   maxj = ncols = NumCols(m);
   if (maxj>maxCols) maxj = maxCols;
   printf("%s\n",title);
   for (i=1;i<=maxi;i++) {
      printf("   ");
      for (j=1;j<=maxj;j++)
         printf("%8.2f ",m[i][j]);
      if (maxj<ncols) printf("...");
      printf("\n");
   }
   if (maxi<nrows)
      printf("   ...\n");
}

/* Export->ShowDMatrix: show the matrix m preceded by title */
void ShowDMatrix(char * title,DMatrix m,int maxCols,int maxRows)
{
   int i,j;
   int maxi,maxj,nrows,ncols;
   
   maxi = nrows = NumDRows(m);
   if (maxi>maxRows) maxi = maxRows;
   maxj = ncols = DVectorSize(m[1]);
   if (maxj>maxCols) maxj = maxCols;
   printf("%s\n",title);
   for (i=1;i<=maxi;i++) {
      printf("   ");
      for (j=1;j<=maxj;j++)
         printf("%10.4f ",m[i][j]);
      if (maxj<ncols) printf("...");
      printf("\n");
   }
   if (maxi<nrows)
      printf("   ...\n");
}

/* Export->ShowTriMat: show the matrix m preceded by title */
void ShowTriMat(char * title,TriMat m,int maxCols,int maxRows)
{
   int i,j;
   int maxi,maxj,size;
   
   size = TriMatSize(m);
   maxi = size;
   if (maxi>maxRows) maxi = maxRows;
   printf("%s\n",title);
   for (i=1;i<=maxi;i++) {
      printf("   ");
      maxj = i;
      if (maxj>maxCols) maxj = maxCols;
      for (j=1;j<=maxj;j++)
         printf("%8.2f ",m[i][j]);
      if (maxj<i) printf("...");
      printf("\n");
   }
   if (maxi<size)
      printf("   ...\n");
}

/* -------------------- Matrix Operations ---------------------- */

/* Choleski: Place lower triangular choleski factor of A in L.*/
/*           Return FALSE if matrix singular or not +definite */
static Boolean Choleski(TriMat A, DMatrix L)
{
   int size,i,j,k;
   double sum;

   size = TriMatSize(A);
   for (i=1; i<=size; i++)
      for (j=1; j<=i; j++) {
         sum=A[i][j];
         for (k=1; k<j; k++)
            sum -= (L[i][k]*L[j][k]);
         if ((i==j)&&(sum<=0.0)) 
            return FALSE;
         else if (i==j)
            sum = sqrt(sum);
         else if (L[j][j]==0.0)
            return FALSE;
         else
            sum /= L[j][j];
         L[i][j] = sum;
      }
   for (i=1; i<=size; i++) 
      for (j=i+1; j<=size; j++) 
         L[i][j] = 0.0;
   return TRUE;
}

/* MSolve: solve Ly=e^i and L^t x = y, where e^i is a unit vector */
static void MSolve(DMatrix L, int i, DVector x, DVector y)
{
   int nr,j,k;
   double sum;
   
   nr=NumDRows(L);
   for (j=1; j<i; j++) y[j] = 0.0;  /* forward sub */
   y[i] = 1.0/L[i][i];
   for (j=i+1; j<=nr; j++){
      sum = 0.0;
      for (k=i; k<j; k++)
         sum -= L[j][k]*y[k];
      y[j] = sum /L[j][j];
   }
   x[nr] = y[nr]/L[nr][nr];         /* backward sub */
   for (j=nr-1; j>=1; j--){
      sum = y[j];
      for (k=j+1; k<=nr; k++)
         sum -= L[k][j]*x[k];
      x[j] = sum / L[j][j];
   }
}

/* EXPORT->CovInvert: puts inverse of c in invc, returns log(Det(c)) */
/*          Note that c must be positive definite */
LogFloat CovInvert(TriMat c, Matrix invc)
{
   DMatrix l;     /* Lower Tri Choleski Matrix */
   DVector x,y;   /* for f/b substitution */
   LogFloat ldet = 0.0;
   int i,j,n;
   Boolean isTri;
   
   n = TriMatSize(c); isTri = IsTriMat(invc);
   l = CreateDMatrix(&gstack,n,n);
   x = CreateDVector(&gstack,n);
   y = CreateDVector(&gstack,n);
   if (Choleski(c,l)){
      for (j=1; j<=n; j++){
         MSolve(l,j,x,y);
         for (i=isTri?j:1; i<=n; i++)
            invc[i][j] = x[i];
         ldet += log(l[j][j]);
      }
   } else
      HError(5220,"CovInvert: [%f ...] not invertible",c[1][1]);
   FreeDMatrix(&gstack,l);    /* cut back stack to entry state */
   return 2.0*ldet;
}

/* EXPORT->CovDet: Returns log(Det(c)), c must be positive definite */
LogFloat CovDet(TriMat c)
{
   DMatrix l;  /* Lower Tri Choleski Matrix */
   LogFloat ldet = 0.0;
   int j,n;
   
   n = TriMatSize(c);
   l = CreateDMatrix(&gstack,n,n);
   if (Choleski(c,l)){
      for (j=1; j<=n; j++)
         ldet += log(l[j][j]);
   } else
      HError(5220,"CovDet: [%f ...] not invertible",c[1][1]);
   FreeDMatrix(&gstack,l);
   return 2.0*ldet;
}

/* Quadratic prod of a full square matrix C and an arbitry full matrix transform A */
void LinTranQuaProd(Matrix Prod, Matrix A, Matrix C)
{
   int i,j,k;
   float tempElem;
   Matrix tempMatrix_A_mult_C;
   
   if (NumRows(C) != NumCols(C)){
      HError(999,"HMath: LinTranQuaProd: Matrix C is not square!\n");
   }
   else {
      tempMatrix_A_mult_C = CreateMatrix(&gstack,NumRows(A),NumCols(C));
      ZeroMatrix(tempMatrix_A_mult_C);
      
      for (i=1;i<=NumRows(tempMatrix_A_mult_C);i++){
         for (j=1;j<=NumCols(tempMatrix_A_mult_C);j++){
            tempElem = 0.0;
            for (k=1;k<=NumCols(A);k++){
               tempElem += A[i][k]*C[j][k];
            }
            tempMatrix_A_mult_C[i][j] = tempElem;
         }
      }
      
      for (i=1;i<=NumRows(Prod);i++){
         for (j=1;j<=i;j++){
            tempElem = 0.0;
            for (k=1;k<=NumCols(tempMatrix_A_mult_C);k++){
               tempElem += tempMatrix_A_mult_C[i][k]*A[j][k];
            }
            Prod[i][j] = tempElem;
         }
      }
      
      for (i=1;i<=NumRows(Prod);i++){
         for (j=1;j<i;j++){
            Prod[j][i] = Prod[i][j];
         }
      }    
      
      FreeMatrix(&gstack,tempMatrix_A_mult_C);
   }
}


/* ------------- Singular Value Decomposition --------------- */
/**************************************************************************
 **
 ** Copyright (C) 1993 David E. Steward & Zbigniew Leyk, all rights reserved.
 **
 **                          Meschach Library
 ** 
 ** This Meschach Library is provided "as is" without any express 
 ** or implied warranty of any kind with respect to this software. 
 ** In particular the authors shall not be liable for any direct, 
 ** indirect, special, incidental or consequential damages arising 
 ** in any way from use of the software.
** 
** Everyone is granted permission to copy, modify and redistribute this
** Meschach Library, provided:
**  1.  All copies contain this copyright notice.
**  2.  All modified copies shall carry a notice stating who
**      made the last modification and the date of such modification.
**  3.  No charge is made for this software or works derived from it.  
**      This clause shall not be construed as constraining other software
**      distributed on the same medium as this software, nor is a
**      distribution fee considered a charge.
**
**  Modifications made to conform with HTK formats by 
**  Daniel Kershaw, Entropic Ltd, Cambridge, England.
**
***************************************************************************/
#define MACHEPS 2.22045e-16
#define FZERO 1.0e-6
#define sgn(x)  ((x) >= 0 ? 1 : -1)
#define minab(a,b) ((a) > (b) ? (b) : (a))
#define MAX_STACK       100

/* Givens -- returns c,s parameters for Givens rotation to
   eliminate y in the vector [ x y ]' */
static void Givens(double x, double y, double *c, double *s)
{
   double norm;
  
   norm = sqrt(x*x+y*y);
   if ( norm == 0.0 ) {
      *c = 1.0;
      *s = 0.0;       
   }       /* identity */
   else {
      *c = x/norm;
      *s = y/norm;
   }
}


/* RotRows -- premultiply mat by givens rotation described by c,s */
static void RotRows(DMatrix M, int i, int k, 
                    double c, double s)
{
   int   j, n;
   double temp;
  
   n = NumDRows(M);
  
   if (i > n || k > n)
      HError(1, "RotRows: Index tooo big i=%d k=%d\n", i, k);
  
  
   for ( j=1; j<=n; j++ ) {
      temp = c*M[i][j] + s*M[k][j];
      M[k][j] = -s*M[i][j] + c*M[k][j];
      M[i][j] = temp;
   }
  
}

/* FixSVD -- fix minor details about SVD make singular values non-negative
   -- sort singular values in decreasing order */
static void FixSVD(DVector d, DMatrix U, DMatrix V)
{
  
   int  i, j, n;
  
   n = DVectorSize(d);


   /* make singular values non-negative */
   for (i = 1; i <= n; i++) {
      if ( d[i] < 0.0 ) {
         d[i] = - d[i];
         for ( j = 1; j <= NumDRows(U); j++ )
            U[i][j] = - U[i][j];
      }
   }

   return;

#if 0 /* #### ge: what is this code after return supposed to do here? */
   {
      int  k, l, r, stack[MAX_STACK], sp;
      double tmp, v;

   /* sort singular values */
   sp = -1;
   l = 1;       
   r = n;
   for ( ; ; ) {
      while ( r >= l ) {
         /* i = partition(d->ve,l,r) */
         v = d[r];
         i = l-1;           
         j = r;
         for ( ; ; ) {
            /* inequalities are "backwards" for **decreasing** order */
            while ( d[++i] > v );
            while ( d[--j] < v );
            if ( i >= j )
               break;
            /* swap entries in d->ve */
            tmp = d[i];   
            d[i] = d[j];
            d[j] = tmp;
            /* swap rows of U & V as well */
            for ( k = 1; k <= DVectorSize(U[1]); k++ ) {
               tmp = U[i][k];
               U[i][k] = U[j][k];
               U[j][k] = tmp;
            }
            for ( k = 1; k <= DVectorSize(V[1]); k++ ) {
               tmp = V[i][k];
               V[i][k] = V[j][k];
               V[j][k] = tmp;
            }
         }
         tmp = d[i];
         d[i] = d[r];
         d[r] = tmp;
         for ( k = 1; k <= DVectorSize(U[1]); k++ ) {
            tmp = U[i][k];
            U[i][k] = U[r][k];
            U[r][k] = tmp;
         }
         for ( k = 1; k <= DVectorSize(V[1]); k++ ) {
            tmp = V[i][k];
            V[i][k] = V[r][k];
            V[r][k] = tmp;
         }
         /* end i = partition(...) */
         if ( i - l > r - i ) {
            stack[++sp] = l;    
            stack[++sp] = i-1;
            l = i+1;    
         }
         else {
            stack[++sp] = i+1;
            stack[++sp] = r;
            r = i-1;    
         }
      }
      if ( sp < 0 )
         break;
      r = stack[sp--];
      l = stack[sp--];
   }
   }
#endif
}

/* BiSvd -- svd of a bidiagonal m x n matrix represented by d (diagonal) and
   f (super-diagonals) */
static void BiSVD(DVector d, DVector f, DMatrix U, DMatrix V)
{
   int i, j, n;
   int i_min, i_max, split;
   double c, s, shift, size, z;
   double d_tmp, diff, t11, t12, t22;

   if ( ! d || ! f )
      HError(1,"BiSVD: Vectors are null!");
   if ( DVectorSize(d) != DVectorSize(f) + 1 )
      HError(1, "BiSVD: Error with the vector sizes!");
 
   n = DVectorSize(d);
  
   if ( ( U && DVectorSize(U[1]) < n ) || ( V && NumDRows(V) < n ) )
      HError(1, "BiSVD: Error Matrix sizes!");
   if ( ( U && NumDRows(U) != DVectorSize(U[1])) || 
        ( V && NumDRows(V) != DVectorSize(V[1])) )
      HError(1, "BiSVD: One of the matrices must be square");
  
  
   if ( n == 1 )
      return;
    
   s = 0.0;
   for ( i = 1; i <= n; i++)
      s += d[i]*d[i];
   size = sqrt(s);
   s = 0.0;
   for ( i = 1; i < n; i++)
      s += f[i]*f[i];
   size += sqrt(s);
   s = 0.0;
  
   i_min = 1;
   while ( i_min <= n ) {   /* outer while loop */
      /* find i_max to suit;
         submatrix i_min..i_max should be irreducible */
      i_max = n;
      for ( i = i_min; i < n; i++ )
         if ( d[i] == 0.0 || f[i] == 0.0 ) {
            i_max = i;
            if ( f[i] != 0.0 ) {
               /* have to ``chase'' f[i] element out of matrix */
               z = f[i];
               f[i] = 0.0;
               for ( j = i; j < n && z != 0.0; j++ ) {
                  Givens(d[j+1],z, &c, &s);
                  s = -s;
                  d[j+1] =  c*d[j+1] - s*z;
                  if ( j+1 < n ) {
                     z      = s*f[j+1];
                     f[j+1] = c*f[j+1];
                  }
                  RotRows(U,i,j+1,c,s);
               }
            }
            break;
         }    
  

      if ( i_max <= i_min ) {
         i_min = i_max + 1;
         continue;
      }

      split = FALSE;
      while ( ! split ) {
         /* compute shift */
         t11 = d[i_max-1]*d[i_max-1] +
            (i_max > i_min+1 ? f[i_max-2]*f[i_max-2] : 0.0);
         t12 = d[i_max-1]*f[i_max-1];
         t22 = d[i_max]*d[i_max] + f[i_max-1]*f[i_max-1];
         /* use e-val of [[t11,t12],[t12,t22]] matrix
            closest to t22 */
         diff = (t11-t22)/2;
         shift = t22 - t12*t12/(diff +
                                sgn(diff)*sqrt(diff*diff+t12*t12));
      
         /* initial Givens' rotation */
         Givens(d[i_min]*d[i_min]-shift,
                d[i_min]*f[i_min], &c, &s);
      
         /* do initial Givens' rotations */
         d_tmp      = c*d[i_min] + s*f[i_min];
         f[i_min]   = c*f[i_min] - s*d[i_min];
         d[i_min]   = d_tmp;
         z          = s*d[i_min+1];
         d[i_min+1] = c*d[i_min+1];
         RotRows(V,i_min,i_min+1,c,s);

         /* 2nd Givens' rotation */
         Givens(d[i_min],z, &c, &s);
         d[i_min]   = c*d[i_min] + s*z;
         d_tmp      = c*d[i_min+1] - s*f[i_min];
         f[i_min]   = s*d[i_min+1] + c*f[i_min];
         d[i_min+1] = d_tmp;
         if ( i_min+1 < i_max ) {
            z          = s*f[i_min+1];
            f[i_min+1] = c*f[i_min+1];
         }
         RotRows(U,i_min,i_min+1,c,s);
      
         for ( i = i_min+1; i < i_max; i++ ) {
            /* get Givens' rotation for zeroing z */
            Givens(f[i-1],z, &c, &s);
            f[i-1] = c*f[i-1] + s*z;
            d_tmp  = c*d[i] + s*f[i];
            f[i]   = c*f[i] - s*d[i];
            d[i]   = d_tmp;
            z      = s*d[i+1];
            d[i+1] = c*d[i+1];
            RotRows(V,i,i+1,c,s);

            /* get 2nd Givens' rotation */
            Givens(d[i],z, &c, &s);
            d[i]   = c*d[i] + s*z;
            d_tmp  = c*d[i+1] - s*f[i];
            f[i]   = c*f[i] + s*d[i+1];
            d[i+1] = d_tmp;
            if ( i+1 < i_max ) {
               z      = s*f[i+1];
               f[i+1] = c*f[i+1];
            }
            RotRows(U,i,i+1,c,s);
         }
         /* should matrix be split? */
         for ( i = i_min; i < i_max; i++ )
            if ( fabs(f[i]) <
                 MACHEPS*(fabs(d[i])+fabs(d[i+1])) )
               {
                  split = TRUE;
                  f[i] = 0.0;
               }
            else if ( fabs(d[i]) < MACHEPS*size )
               {
                  split = TRUE;
                  d[i] = 0.0;
               }
      }
   }
}

/* HholdVec -- calulates Householder vector to eliminate all entries after the
   i0 entry of the vector vec. It is returned as out. May be in-situ */
static void HholdVec(DVector tmp, int i0, int size,
                     double *beta, double *newval)
{
   int i;
   double norm = 0.0;

   for (i = i0; i <= size; i++) {
      norm += tmp[i]*tmp[i];
   }
   norm = sqrt(norm);

   if ( norm <= 0.0 ) {
      *beta = 0.0;
   }
   else {
      *beta = 1.0/(norm * (norm+fabs(tmp[i0])));
      if ( tmp[i0] > 0.0 )
         *newval = -norm;
      else
         *newval = norm;
      tmp[i0] -= *newval;
   }

}


/* HholdTrRows -- transform a matrix by a Householder vector by rows
   starting at row i0 from column j0 -- in-situ */
static void HholdTrRows(DMatrix M, int i0, int j0, DVector hh, double beta)
{
   double ip, scale;
   int i, j;
   int m,n;

   m = NumDRows(M);
   n = DVectorSize(M[1]);

   if ( M==NULL || hh==NULL )
      HError(1,"HholdTrRows: matrix or vector is NULL!");
   if ( DVectorSize(hh) != n )
      HError(1,"HholdTrRows: hh vector size must = number of M columns");
   if ( i0 > m+1 || j0 > n+1 )
      HError(1,"HholdTrRows: Bounds matrix/vec size error i=%d j=%d m=%d n=%d",
             i0, j0, m, n);
  
   if ( beta != 0.0 ) {
      /* for each row ... */
      for ( i = i0; i <= m; i++ )
         {  
            /* compute inner product */
            /* ip = __ip__(&(M->me[i][j0]),&(hh->ve[j0]),(int)(M->n-j0));*/
            ip = 0.0;
            for ( j = j0; j <= n; j++ )
               ip += M[i][j]*hh[j];
            scale = beta*ip;
            if ( scale == 0.0 )
               continue;
            /* __mltadd__(&(M->me[i][j0]),&(hh->ve[j0]),-scale,
               (int)(M->n-j0)); */
            for ( j = j0; j <= n; j++ )
               M[i][j] -= scale*hh[j];
         }
   }
}

/* HholdTrCols -- transform a matrix by a Householder vector by columns
   starting at row i0 from column j0 -- in-situ */
static void HholdTrCols(DMatrix M, int i0, int j0, 
                        DVector hh, double beta, DVector w)
{
   int i, j;
   int n;

   n = NumDRows(M);

   if ( M==NULL || hh==NULL )
      HError(1,"HholdTrCols: matrix or vector is NULL!");
   if ( DVectorSize(hh) != n )
      HError(1,"HholdTrCols: hh vector size must = number of M columns");
   if ( i0 > n+1 || j0 > n+1 )
      HError(1,"HholdTrCols: Bounds matrix/vec size error i=%d j=%d n=%d",
             i0, j0, n);

   ZeroDVector(w);

   if ( beta != 0.0 ) {

      for ( i = i0; i <= n; i++ )
         if ( hh[i] != 0.0 )
            for ( j = j0; j <= n; j++ )
               w[j] += M[i][j]*hh[i];

      for ( i = i0; i <= n; i++ )
         if ( hh[i] != 0.0 )
            for ( j = j0; j <= n; j++ )
               M[i][j] -= w[j]*beta*hh[i];

   }
}


/* copy a row from a matrix into  a vector */
static void CopyDRow(DMatrix M, int k, DVector v) 
{
   int i, size;
   DVector w;

   if (v == NULL)
      HError(1, "CopyDRow: Vector is NULL");

   size = DVectorSize(v);
   w = M[k];

   for (i = 1; i <= size; i++)
      v[i] = w[i];
}

/* copy a column from a matrix into  a vector */
static void CopyDColumn(DMatrix M, int k, DVector v) 
{
   int i, size;

   if (v == NULL)
      HError(1, "CopyDColumn: Vector is NULL");

   size = DVectorSize(v);
   for (i = 1; i <= size; i++)
      v[i] = M[i][k];
}

/* BiFactor -- perform preliminary factorisation for bisvd
   -- updates U and/or V, which ever is not NULL */
static void BiFactor(DMatrix A, DMatrix U, DMatrix V)
{
   int n, k;
   DVector tmp1, tmp2, tmp3;
   double beta;

   n = NumDRows(A);

   tmp1 = CreateDVector(&gstack, n);
   tmp2 = CreateDVector(&gstack, n);
   tmp3 = CreateDVector(&gstack, n);

   for ( k = 1; k <= n; k++ ) {
      CopyDColumn(A,k,tmp1);
      HholdVec(tmp1,k,n,&beta,&(A[k][k]));
      HholdTrCols(A,k,k+1,tmp1,beta,tmp3);
      if ( U )
         HholdTrCols(U,k,1,tmp1,beta,tmp3);
      if ( k+1 > n )
         continue;
      CopyDRow(A,k,tmp2);
      HholdVec(tmp2,k+1,n,&beta,&(A[k][k+1]));
      HholdTrRows(A,k+1,k+1,tmp2,beta);
      if ( V )
         HholdTrCols(V,k+1,1,tmp2,beta,tmp3);
   }

   FreeDVector(&gstack, tmp1);
}

/* mat_id -- set A to being closest to identity matrix as possible
        -- i.e. A[i][j] == 1 if i == j and 0 otherwise */
static void InitIdentity(DMatrix A) 
{
   int     i, size;
  
   ZeroDMatrix(A);
   size = minab(NumDRows(A), DVectorSize(A[1]));
   for ( i = 1; i <= size; i++ )
      A[i][i] = 1.0;
}

/* EXPORT->SVD: Calculate the decompostion of matrix A.
   NOTE: on return that U and V hold U' and V' respectively! */
void SVD(DMatrix A, DMatrix U, DMatrix V, DVector d)
{
   DVector f=NULL;
   int i, n;
   DMatrix A_tmp;

   /* do initial size checks! */
   if ( A == NULL )
      HError(1,"svd: Matrix A is null");

   n = NumDRows(A);

   if (U == NULL || V == NULL || d == NULL)
      HError(1, "SVD: The svd matrices and vector must be initialised b4 call");
 
   A_tmp = CreateDMatrix(&gstack, n, n);
   CopyDMatrix(A, A_tmp);
   InitIdentity(U);
   InitIdentity(V);
   f = CreateDVector(&gstack,n-1);

   BiFactor(A_tmp,U,V);
   for ( i = 1; i <= n; i++ ) {
      d[i] = A_tmp[i][i];
      if ( i+1 <= n )
         f[i] = A_tmp[i][i+1];
   }

   BiSVD(d,f,U,V);
   FixSVD(d,U,V);
   FreeDMatrix(&gstack, A_tmp);
}

/* EXPORT->InvSVD: Inverted Singular Value Decomposition (calls SVD)
   and inverse of A is returned in Result */
void InvSVD(DMatrix A, DMatrix U, DVector W, DMatrix V, DMatrix Result)
{
   int m, n, i, j, k;
   double wmax, wmin;
   Boolean isSmall = FALSE;
   DMatrix tmp1;

   m = NumDRows(U);
   n = DVectorSize(U[1]);

   if (m != n)
      HError(1, "InvSVD: Matrix inversion only for symmetric matrices!\n");

   SVD(A, U, V, W);
   /* NOTE U and V actually now hold U' and V' ! */

   tmp1 = CreateDMatrix(&gstack,m, n);

   wmax = 0.0;
   for (k = 1; k <= n; k ++)
      if (W[k] > wmax)
         wmax = W[k];
   wmin = wmax * 1.0e-8;
   for (k = 1; k <= n; k ++)
      if (W[k] < wmin) {
         /* A component of the diag matrix 'w' of the SVD of 'a'
            was smaller than 1.0e-6 and consequently set to zero. */
         if (trace>0) {
            printf("%d (%e) ", k, W[k]); 
            fflush(stdout);
         }
         W[k] = 0.0;
         isSmall = TRUE;
      }
   if (trace>0 && isSmall) {
      printf("\n"); 
      fflush(stdout);
   }
   /* tmp1 will be the product of matrix v and the diagonal 
      matrix of singular values stored in vector w. tmp1 is then
      multiplied by the transpose of matrix u to produce the 
      inverse which is returned */
   for (j = 1; j <= m; j++)
      for (k = 1; k <= n; k ++)
         if (W[k] > 0.0)
            /* Only non-zero elements of diag matrix w are 
               used to compute the inverse. */
            tmp1[j][k] = V[k][j] / W[k];
         else
            tmp1[j][k] = 0.0;

   ZeroDMatrix(Result);
   for (i=1;i<=m;i++)
      for (j=1;j<=m;j++)
         for (k=1;k<=n;k++)
            Result[i][j] += tmp1[i][k] * U[k][j];
   FreeDMatrix(&gstack,tmp1);
}

/* LUDecompose: perform LU decomposition on Matrix a, the permutation
       of the rows is returned in perm and sign is returned as +/-1
       depending on whether there was an even/odd number of row 
       interchanges */
static Boolean LUDecompose(Matrix a, int *perm, int *sign)
{
   int i,imax,j,k,n;
   double scale,sum,xx,yy;
   Vector vv,tmp;
   
   n = NumRows(a); imax = 0;
   vv = CreateVector(&gstack,n);
   *sign = 1;
   for (i=1; i<=n; i++) {
      scale = 0.0;
      for (j=1; j<=n; j++)
         if ((xx = fabs(a[i][j])) > scale )
            scale = xx;
      if (scale == 0.0) {
         HError(-1,"LUDecompose: Matrix is Singular");
	 return(FALSE);
      }
      vv[i] = 1.0/scale;
   }
   for (j=1; j<=n; j++) {
      for (i=1; i<j; i++) {
         sum = a[i][j];
         for (k=1; k<i; k++) sum -= a[i][k]*a[k][j];
         a[i][j]=sum;
      }
      scale=0.0;
      for (i=j; i<=n; i++) {
         sum = a[i][j];
         for (k=1; k<j; k++) sum -= a[i][k]*a[k][j];
         a[i][j]=sum;
         if ( (yy=vv[i]*fabs(sum)) >=scale) {
            scale = yy; imax = i;
         }
      }
      if (j != imax) {
         tmp = a[imax]; a[imax] = a[j]; a[j] = tmp;
         *sign = -(*sign);
         vv[imax]=vv[j];
      }
      perm[j]=imax;
      if (a[j][j] == 0.0) {
         HError(-1,"LUDecompose: Matrix is Singular");
	 return(FALSE);
      }
      if (j != n) {
         yy = 1.0/a[j][j];
         for (i=j+1; i<=n;i++) a[i][j] *= yy;
      }
   }
   FreeVector(&gstack,vv);
   return(TRUE);
}


/* EXPORT->MatDet: determinant of a matrix */
float MatDet(Matrix c)
{
   Matrix a;
   float det;
   int n,perm[1600],i,sign;
   
   n=NumRows(c);
   a=CreateMatrix(&gstack,n,n);
   CopyMatrix(c,a);                /* Make a copy of c */
   LUDecompose(a,perm,&sign);      /* Do LU Decomposition */
   det = sign;                     /* Calc Det(c) */
   for (i=1; i<=n; i++) {
      det *= a[i][i];
   }
   FreeMatrix(&gstack,a);
   return det;
}


/* DLUDecompose: perform LU decomposition on Matrix a, the permutation
       of the rows is returned in perm and sign is returned as +/-1
       depending on whether there was an even/odd number of row 
       interchanges */
static Boolean DLUDecompose(DMatrix a, int *perm, int *sign)
{
   int i,imax,j,k,n;
   double scale,sum,xx,yy;
   DVector vv,tmp;
   
   n = NumDRows(a); imax = 0;
   vv = CreateDVector(&gstack,n);
   *sign = 1;
   for (i=1; i<=n; i++) {
      scale = 0.0;
      for (j=1; j<=n; j++)
         if ((xx = fabs(a[i][j])) > scale )
            scale = xx;
      if (scale == 0.0) {
         HError(-1,"LUDecompose: Matrix is Singular");
         return(FALSE);
      }
      vv[i] = 1.0/scale;
   }
   for (j=1; j<=n; j++) {
      for (i=1; i<j; i++) {
         sum = a[i][j];
         for (k=1; k<i; k++) sum -= a[i][k]*a[k][j];
         a[i][j]=sum;
      }
      scale=0.0;
      for (i=j; i<=n; i++) {
         sum = a[i][j];
         for (k=1; k<j; k++) sum -= a[i][k]*a[k][j];
         a[i][j]=sum;
         if ( (yy=vv[i]*fabs(sum)) >=scale) {
            scale = yy; imax = i;
         }
      }
      if (j != imax) {
         tmp = a[imax]; a[imax] = a[j]; a[j] = tmp;
         *sign = -(*sign);
         vv[imax]=vv[j];
      }
      perm[j]=imax;
      if (a[j][j] == 0.0) {
         HError(-1,"LUDecompose: Matrix is Singular");
         return(FALSE);
      }
      if (j != n) {
         yy = 1.0/a[j][j];
         for (i=j+1; i<=n;i++) a[i][j] *= yy;
      }
   }
   FreeDVector(&gstack,vv);
   return(TRUE);
}


/* EXPORT->DMatDet: determinant of a double matrix */
double DMatDet(DMatrix c)
{
   DMatrix a;
   double det;
   int n,perm[1600],i,sign;
   
   n=NumDRows(c);
   a=CreateDMatrix(&gstack,n,n);
   CopyDMatrix(c,a);                /* Make a copy of c */
   DLUDecompose(a,perm,&sign);      /* Do LU Decomposition */
   det = sign;                     /* Calc Det(c) */
   for (i=1; i<=n; i++) {
      det *= a[i][i];
   }
   FreeDMatrix(&gstack,a);
   return det;
}



/* LinSolve: solve the set of linear equations Ax = b, returning
        the result x in  b */
static void LinSolve(Matrix a, int *perm, float *b)
{
   int i,ii=0,ip,j,n;
   double sum;
   
   n=NumRows(a);
   for (i=1;i<=n;i++) {
      ip=perm[i]; sum=b[ip]; b[ip]=b[i];
      if (ii)
         for (j=ii;j<=i-1;j++) sum -=a[i][j]*b[j];
      else
         if (sum) ii=i;
      b[i]=sum;
   }
   for (i=n; i>=1; i--) {
      sum=b[i];
      for (j=i+1; j<=n; j++)
         sum -=a[i][j]*b[j];
      b[i]=sum/a[i][i];
   }
}        


/* EXPORT-> MatInvert: puts inverse of c in invc, returns Det(c) */
float MatInvert(Matrix c, Matrix invc)
{
   Matrix a;
   float col[100];
   float det;
   int sign;
   int n,i,j,perm[100];
   
   n=NumRows(c);
   a=CreateMatrix(&gstack,n,n);
   CopyMatrix(c,a);           /* Make a copy of c */
   LUDecompose(a,perm,&sign);      /* Do LU Decomposition */
   for (j=1; j<=n; j++) {     /* Invert matrix */
      for (i=1; i<=n; i++)
         col[i]=0.0;
      col[j]=1.0;
      LinSolve(a,perm,col);
      for (i=1; i<=n; i++)
         invc[i][j] = col[i];
   }  
   det = sign;                /* Calc log(det(c)) */
   for (i=1; i<=n; i++) {
      det *= a[i][i];
   }
   FreeMatrix(&gstack,a);
   return det;
}                
 
/* DLinSolve: solve the set of linear equations Ax = b, returning
        the result x in  b */
static void DLinSolve(DMatrix a, int *perm, double *b)
{
   int i,ii=0,ip,j,n;
   double sum;
   
   n=NumDRows(a);
   for (i=1;i<=n;i++) {
      ip=perm[i]; sum=b[ip]; b[ip]=b[i];
      if (ii)
         for (j=ii;j<=i-1;j++) sum -=a[i][j]*b[j];
      else
         if (sum) ii=i;
      b[i]=sum;
   }
   for (i=n; i>=1; i--) {
      sum=b[i];
      for (j=i+1; j<=n; j++)
         sum -=a[i][j]*b[j];
      b[i]=sum/a[i][i];
   }
}       

/* Inverting a double matrix */
double DMatInvert(DMatrix c, DMatrix invc)
{
   DMatrix a;
   double col[100];
   double det;
   int sign;
   int n,i,j,perm[100];
   
   n=NumDRows(c);
   a=CreateDMatrix(&gstack,n,n);
   CopyDMatrix(c,a);           /* Make a copy of c */
   DLUDecompose(a,perm,&sign);      /* Do LU Decomposition */
   for (j=1; j<=n; j++) {     /* Invert matrix */
      for (i=1; i<=n; i++)
         col[i]=0.0;
      col[j]=1.0;
      DLinSolve(a,perm,col);
      for (i=1; i<=n; i++)
         invc[i][j] = col[i];
   }  
   det = sign;                /* Calc log(det(c)) */
   for (i=1; i<=n; i++) {
      det *= a[i][i];
   }
   FreeDMatrix(&gstack,a);
   return det;
}

/* EXPORT-> DMatCofact: generates the cofactors of row r of matrix c */
double DMatCofact(DMatrix c, int r, DVector cofact)
{
   DMatrix a;
   double col[100];
   double det;
   int sign;
   int n,i,perm[100];
   
   n=NumDRows(c);
   a=CreateDMatrix(&gstack,n,n);
   CopyDMatrix(c,a);                      /* Make a copy of c */
   if (! DLUDecompose(a,perm,&sign))      /* Do LU Decomposition */
     return 0;
   det = sign;                         /* Calc det(c) */
   for (i=1; i<=n; i++) {
      det *= a[i][i];
   }
   for (i=1; i<=n; i++)
     col[i]=0.0;
   col[r]=1.0;
   DLinSolve(a,perm,col);
   for (i=1; i<=n; i++)
     cofact[i] = col[i]*det;
   FreeDMatrix(&gstack,a);
   return det;
}

/* EXPORT-> MatCofact: generates the cofactors of row r of matrix c */
double MatCofact(Matrix c, int r, Vector cofact)
{
   DMatrix a;
   DMatrix b;
   double col[100];
   float det;
   int sign;
   int n,i,perm[100];
 
   n=NumRows(c);
   a=CreateDMatrix(&gstack,n,n);
   b=CreateDMatrix(&gstack,n,n);
   Mat2DMat(c,b);
   CopyDMatrix(b,a);                      /* Make a copy of c */
   if (! DLUDecompose(a,perm,&sign))      /* Do LU Decomposition */
     return 0;
   det = sign;                         /* Calc det(c) */
   for (i=1; i<=n; i++) {
      det *= a[i][i];
   }
   for (i=1; i<=n; i++)
     col[i]=0.0;
   col[r]=1.0;
   DLinSolve(a,perm,col);
   for (i=1; i<=n; i++)
     cofact[i] = col[i]*det;
   
   FreeDMatrix(&gstack,b);
   FreeDMatrix(&gstack,a);
   return det;
}

/* -------------------- Log Arithmetic ---------------------- */

/*
  The types LogFloat and LogDouble are used for representing
  real numbers on a log scale.  LZERO is used for log(0) 
  in log arithmetic, any log real value <= LSMALL is 
  considered to be zero.
*/

static LogDouble minLogExp;

/* EXPORT->LAdd: Return sum x + y on log scale, 
                sum < LSMALL is floored to LZERO */
LogDouble LAdd(LogDouble x, LogDouble y)
{
   LogDouble temp,diff,z;
   
   if (x<y) {
      temp = x; x = y; y = temp;
   }
   diff = y-x;
   if (diff<minLogExp) 
      return  (x<LSMALL)?LZERO:x;
   else {
      z = exp(diff);
      return x+log(1.0+z);
   }
}

/* EXPORT->LSub: Return diff x - y on log scale, 
                 diff < LSMALL is floored to LZERO */
LogDouble LSub(LogDouble x, LogDouble y)
{
   LogDouble diff,z;
   
   if (x<y)    
      HError(5271,"LSub: result -ve");
   diff = y-x;
   if (diff<minLogExp) 
      return  (x<LSMALL)?LZERO:x;
   else {
      z = 1.0 - exp(diff);
      return (z<MINLARG) ? LZERO : x+log(z);
   }
}

/* EXPORT->L2F: Convert log(x) to double, result is
                floored to 0.0 if x < LSMALL */
double   L2F(LogDouble x)
{
   return (x<LSMALL) ? 0.0 : exp(x);
}

/* -------------------- Random Numbers ---------------------- */


#ifdef UNIX
/* Prototype for C Library functions drand48 and srand48 */
double drand48(void);
void srand48(long);
#define RANDF() drand48()
#define SRAND(x) srand48(x)
#else
/* if not unix use ANSI C defaults */
#define RANDF() ((float)rand()/RAND_MAX)
#define SRAND(x) srand(x)
#endif

/* EXPORT->RandInit: Initialise random number generators 
           if seed is -ve, then system clock is used */
void RandInit(int seed)
{
   if (seed<0) seed = (int)time(NULL)%257;
   SRAND(seed);
}

/* EXPORT->RandomValue:  */
float RandomValue(void)
{
   return RANDF();
}

/* EXPORT->GaussDeviate: random number with a N(mu,sigma) distribution */
float GaussDeviate(float mu, float sigma)
{
   double fac,r,v1,v2,x;
   static int gaussSaved = 0; /* GaussDeviate generates numbers in pairs */
   static float gaussSave;    /* 2nd of pair is remembered here */


   if (gaussSaved) {
      x = gaussSave; gaussSaved = 0;
   }
   else {
      do {
         v1 = 2.0*(float)RANDF() - 1.0;
         v2 = 2.0*(float)RANDF() - 1.0;
         r = v1*v1 + v2*v2;
      }
      while (r>=1.0);
      fac = sqrt(-2.0*log(r)/r);
      gaussSaved = 1;
      gaussSave = v1*fac;
      x = v2*fac;
   }
   return x*sigma+mu;
}

/* --------------------- Initialisation ---------------------- */

/* EXPORT->InitMath: initialise this module */
void InitMath(void)
{
   int i;

   Register(hmath_version,hmath_vc_id);
   RandInit(-1);
   minLogExp = -log(-LZERO);
   numParm = GetConfig("HMATH", TRUE, cParm, MAXGLOBS);
   if (numParm>0){
      if (GetConfInt(cParm,numParm,"TRACE",&i)) trace = i;
   }
}

/* ------------------------- End of HMath.c ------------------------- */
