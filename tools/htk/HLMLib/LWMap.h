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
/*      File: LWMap:  Word List and Word Map Routines          */
/* ----------------------------------------------------------- */

/* !HVER!lwmap:   3.4.1 [CUED 12/03/09] */


#ifndef _LWMAP_H
#define _LWMAP_H

#ifdef __cplusplus
extern "C" {
#endif

#define BASEWORDNDX 65536

typedef struct MAPENTRY {  /* word map entry */
   int ndx;                /* index of word in map */
   int sort;               /* for sorting (gives lexical ordering) */
   int count;              /* word frequency count */
   void *class;		   /* pointer to class if any */
   union {                 /* for general use */
      Ptr auxptr;
      float auxflt;
      int  auxint;
   }aux;
} MapEntry;

/* A word map is an array of word/class ids.  Each id points to
   a mapentry via its aux field.  Each class is stored at 
   id[ndx], each word is stored at id[ndx-BASEWORDNDX+nClass].  
   Free space is only ever available for new words, not classes
*/



typedef struct {
   int size;             /* size of lookup array */
   int minId;            /* min index (mapped to 0) */
   int maxId;            /* max index (mapped to size-1) */
   int *tlb;             /* the actual lookup array */
} LookupTable;

typedef struct {         /* array of words (map or list) */
   MemHeap mem;          /* memory heap for this word map */
   char    *name;        /* name of map */
   char    *lang;        /* language */
   char    *source;      /* description of sources */
   int     seqno;        /* sequence number */
   Boolean htkEsc;       /* has HTK escaping */
   Boolean isMap;        /* true if map, false if word list */
   Boolean hasCnts;      /* has word freq cnts, map option only */
   Boolean isSorted;     /* used to avoid redundant sorts */
   int     size;         /* size of array */
   int     used;         /* total words and classes in map */
   int     nClass;       /* number of class ids in map */
   int     firstNdx;     /* ndx of entry which is first in sort order */
   int     lastUsed;     /* last word index */
   /* the actual entries */
   LabId    *id;         /* array[0..used-1] of labid: aux-> MapEntry */
   MapEntry *me;         /* array[0..used-1] of mapentry */
   LookupTable *wlt;     /* word lookup table */
   LookupTable *clt;     /* class lookup table */
} WordMap;


void InitWMap(void);
/* 
   Initialise the module.
*/

void CreateWordMap(char *fn, WordMap *w, int freeSlots);
/*
   Create a word map structure with given number of free slots
   If fn is not NULL entries are loaded from fn.
*/

void CreateWordList(char *fn, WordMap *w, int freeSlots);
/*
   Create a word list (ie a word map with no map entries) with given 
   number of free slots. If fn is not NULL entries are first 
   loaded from fn.
*/

void SaveWordMap(char *fn, WordMap *w, Boolean noHeader);
/*
   Write given word map or list to file fn.  If noHeader
   then the standard HLM header is suppressed.
*/

void ShowWordMap(WordMap *w);
/*
   Print the contents of word map/or list w
*/

void AddWordToMap(WordMap *wm, LabId word);
/*
   Add word to wmap.  If word is already in map, the count 
   is incremented. Otherwise, a new entry is created.
*/

void MarkWordList(WordMap *wl);
/*
   mark words in wlist by setting me->aux non-NULL 
*/

void SortWordMap(WordMap *wm);
/*
   Sort the word map into lexical order leaving the ordering
   in the sort field of each mapentry.
*/

void BuildLookupTables(WordMap *wm);
/*
  construct class/word ndx -> ME lookup tables
*/

/* --------------  General Utility Routines --------------- */

Boolean GetSrcString(Source *src, char *s, Boolean htkEsc);
/* 
   Get next string from src in appropriate in either raw mode 
   (htkEsc=FALSE) or HTK escaped mode (htkEsc=TRUE)
*/

/* -------------------  Access Routines ------------------- */


int GetMEIndex(WordMap *wm, int ndx);
/* 
   obtain index of MapEntry corresponding to word ndx in wm 
*/

int WordLMIndex(LabId id);
int WordLMCount(LabId id);
LabId WordLMName(int ndx, WordMap *wm);
/*
   Word/Class access routines.  Returns word map index/name.
*/

int WordLMCmp(int ndx1, int ndx2, WordMap *wm);
/* 
   Return -1 if word with ndx1 is ordered before ndx2, 0 if identical
   +1 if ndx1 ordered after ndx2 (ie same as strcmp)
*/

/* -------------------- End of LWMap.h ---------------------- */

#ifdef __cplusplus
}
#endif

#endif
