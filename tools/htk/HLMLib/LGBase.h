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
/* main authors: Valtcho Valtchev, Steve Young,                */
/*               Julian Odell, Gareth Moore                    */
/* ----------------------------------------------------------- */
/*         Copyright:                                          */
/*                                                             */
/*          1994-2002 Cambridge University                     */
/*                    Engineering Department                   */
/*                                                             */
/*   Use of this software is governed by a License Agreement   */
/*    ** See the file License for the Conditions of Use  **    */
/*    **     This banner notice must not be removed      **    */
/*                                                             */
/* ----------------------------------------------------------- */
/*         File: LGBase: Gram File Database Routines           */
/* ----------------------------------------------------------- */

/* !HVER!lgbase:   3.4.1 [CUED 12/03/09] */

#ifndef _LGBASE_H_
#define _LGBASE_H_

#ifdef __cplusplus
extern "C" {
#endif

#define SQUASH       3     /* #bytes per word id */
#define GSIZE        32    /* max bytes per N-gram */
#define MAXNG        7     /* max value of N */
#define MAXINF       128   /* max number of open source files */

#ifndef LM_TYPES_DEFINED
typedef unsigned short UShort;
typedef unsigned int   UInt;
typedef unsigned char  Byte;
#define LM_TYPES_DEFINED
#endif

typedef UInt *NGram;          /* N-gram: {w1,w2,...,wN count} */

typedef struct {        /* NGram size/mapping info information */
   int N;                  /* N-gram size N (2..MAXNG)*/
   int ng_size;            /* byte size of squashed N-gram records */
   int ng_full;            /* byte size of expanded N-gram ie N+1 ints */
}NGInfo;

typedef struct {        /* open file containing packed ngrams */
   Source src;              /* the input stream */
   char txtsrc[MAXSTRLEN];  /* text source description */
   int nItems;              /* number of n-grams in file */
   LabId firstGram[MAXNG];  /* first entry in file */
   LabId lastGram[MAXNG];   /* last entry in file */
   Byte buf[GSIZE];         /* next compressed N-gram to read */
   UInt nxt[MAXNG];         /* next expanded N-gram (no count) */
   WordMap *wm;             /* word map to be used with this source */
   NGInfo info;             /* ngram size information */
}NGSource;

typedef struct gramfile *GFLink;
typedef struct gramfile{ /* info record for a packed gram file */
   char fn[MAXSTRLEN];     /* file name */
   LabId firstGram[MAXNG];  /* first entry in file */
   LabId lastGram[MAXNG];   /* last entry in file */
   float weight;           /* weight for subsequent mixing */
   GFLink next;            /* next file to open + ... */
   GFLink alt;             /* ... alt files to open */
   GFLink chain;           /* linked list of all files */
}GramFile;

typedef struct {        /* input N-gram file set */
   int N;                  /* N-gram size */
   WordMap *wm;            /* covering word map */
   MemHeap *mem;           /* memory stack for this input set */
   int nFiles;             /* num files in input set */
   int nOpen;              /* num open files */
   int maxNOpen;           /* max number of open files */
   GramFile head;          /* dummy head of tree */
   NGSource ngs[MAXINF];   /* currently open sources */
   GFLink gf[MAXINF];      /* list of ptrs to gram files */
   int gfsort[MAXINF];     /* idx's of sorted gram files */
   UInt nextGram[MAXNG];   /* next gram to read from inset */  
   float nextWt;           /* weight of next gram */
   Boolean nextValid;      /* true if nextGram is valid */
}NGInputSet;

typedef struct {        /* N-gram buffer */
   NGInfo info;            /* N-gram size and related byte sizes */
   int poolsize;           /* number of N-gram slots in pool */
   int used;               /* number used slots in pool */
   char *fn;               /* basename of output file name */
   int fndx;               /* index of next output file */
   UInt *pool;             /* array[0..used-1] of ngrams */
   UInt *next;             /* next free slot in pool */
   WordMap *wm;            /* word map for ngrams */
} NGBuffer;

typedef struct {        /* N-gram frequency of frequency table */
   int size;               /* size of fof table */
   int N;                  /* N-gram */
   UInt **fof;             /* array[1..N][1..size] of count */
} FoFTab;

void InitGBase(void);
/* 
   Initialise the module
*/

/* --------------- Basic NGram Operations --------------- */

Boolean SameGrams(int N, NGram ng1, NGram ng2);
/*
   Returns true if grams (ignoring counts) are equal
*/

void NGramSquash(int N, NGram ng, Byte *comp);
void NGramExpand(int N, Byte *comp, NGram ng);
/*
   Convert between expanded and squashed form of N-gram array.
   expd is an array of N N-gram indices.  comp is the same array
   with each element squashed to SQUASH (=3) bytes.  Note that
   the count is ignored in both cases.
*/

void PrintNGram(int N, NGram ng, WordMap *wm);
/*
   Print given N-gram.
*/

/* ------------------- N-Gram File I/O ------------------ */

void OpenNGramFile(NGSource *ngs, char *fn, WordMap *wm);
/*
   Open an N-gram file called fn, check that the gram file is 
   consistent with wm and initialise NGSource.  .
   The first N-gram is input and left in the buffer.  
*/

void CloseNGramFile(NGSource *ngs);
/*
   Close given N-gram source file
*/

void ReadNGram(NGSource *ngs, NGram ng);
/* 
   Read the next N-gram from given source into ng. 
   ngs->eof is set true when end of file was reached
   after reading this ngram.
*/  

int WriteNGram(FILE *f, int N, NGram ng);
/*
   Write compressed nGram to file f.  Returns the number
   of copies written (depends on count)
*/

/* --------------- N-Gram Buffer Operations ---------------- */

NGBuffer *CreateNGBuffer(MemHeap *mem, int N, int size, char *fn, WordMap *wm);
/*
   Create an N-gram buffer with size slots, output file fn and 
   word map wm.
*/

void ResetNGBuffer(NGBuffer *ngb);
/*
   Reset buffer to empty and increment file index
*/

Boolean StoreNGram(NGBuffer *ngb, NGram ng);
/*
   Store expanded (N+1 length) N-gram in buf into ngb.
   Return TRUE if ngb is full
*/   

void SortNGBuffer(NGBuffer *ngb);
/*
   Sort the N-grams in ngb and merge duplicates. 
*/

void WriteNGBuffer(NGBuffer *ngb,char *source);
/*
   Write the contents of ngb to its output file, reset
   the buffer to empty and increment file index. The buffer 
   is sorted before output.  If not null, source is stored
   verbatim in the header field Source.
*/

void PrintNGBuffer(NGBuffer *ngb);
/*
   Print contents of ngb.
*/

/* ------------- Multiple N-Gram Input File Handling --------- */

void CreateInputSet(MemHeap *mem, WordMap *wm, NGInputSet *inset);
/*
   Create a data structure to hold a set of parallel
   partially ordered N-gram files.
*/

void AddInputGFile(NGInputSet *inset, char *fn, float weight);
/* 
   Add file fn to input set with given weight.
*/

void OpenInputSet(NGInputSet *inset);
/*
   Examine all input files and sort such that files which can be
   opened and closed sequentially will be ordered as such.  Then
   open all files which can possibly supply the first N-gram.
*/

Boolean GetNextNGram(NGInputSet *inset,NGram ng,float *cnt,int N);
/*
   This is the primary interface to an input set of gram files.
   The open N-gram files in the input set are repeatedly scanned
   and Ngrams returned in sequence.   If any file read from is exhausted, 
   then it is closed and any immediate successor files are opened.  
   Ngrams are buffered so that identical n-grams from different files
   are merged and their counts accumulated.  Furthermore, N can be less
   than the stored N-gram size, in this case, the counts are accumulated
   across all grams which are equivalent in their first N ndx'es.
   The accumulated count is returned in cnt (not in ng since it can be
   scaled by an arbitrary floating point weight).  Function returns false
   when no more ngrams left.
*/

void CloseInputSet(NGInputSet *inset);
/*
   Close any remaining open files in the input set and
   free memory used.
*/

/* ------------------- FoF Table Handling -------------- */

FoFTab *CreateFoFTab(MemHeap *mem, int size, int N);
/* 
   Create a FoF table with size rows
*/

void WriteFoFTab(char *fn, FoFTab *tab, char *source);
/*
   Write given table to fn.  If fn==NULL, write to stdout.
   Source is optional header entry.
*/

FoFTab *ReadFoFTab(MemHeap *mem, char *fn);
/*
   Create a FoF table holding contents of file fn
*/

/* -------------------- End of LGBase.h ---------------------- */

#ifdef __cplusplus
}
#endif

#endif
