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
/*      File: LUtil:    General Utility Routines               */
/* ----------------------------------------------------------- */

/* !HVER!LUtil:   3.4.1 [CUED 12/03/09] */

#ifndef _LUTIL_H
#define _LUTIL_H

#ifdef __cplusplus
extern "C" {
#endif

void InitLUtil(void);
/* 
   Initialise the module.
*/

/* ----------- HLM Header Type and Operations -------------- */
     
typedef enum { 
   NO_HDR, WMAP_HDR, CMAP_HDR, GRAM_HDR, LFOF_HDR
} LMHdrKind;

typedef struct lmFileHdrRec * LMFileHdr;     /* Abstract type */       

LMHdrKind ReadLMHeader(MemHeap *mem, Source *src, IOFilter filter,  
                       LMFileHdr *hdr, int *n);
/*
   Read an LM header from the given source and create an LMFileHdr
   in mem.  The type of header is returned. On successful completion,
   src will be left positioned at the first byte of the data segment
   and the number of entries will be stored in n.  If the src is
   headerless, then n is determined by reading whole file, closing
   it and reopening it.  The filter parameter is only needed if file
   is headerless.
*/

int NumLMHdrFields(LMFileHdr hdr);
/*
   Return number of fields in hdr.
*/

char * GetLMHdrStr(char *name, LMFileHdr hdr, Boolean ucase);
/*
   Return the str value of header field name.  If ucase then
   upper case it first. Returns NULL if name not defined in hdr.
*/

Boolean GetLMHdrInt(char *name, int *value, LMFileHdr hdr);
/*
   Store the int value of header field name in value.  Returns 
   false if name not defined in hdr or value not interpretable as
   an integer.
*/

/* ----------------- Miscellaneous Operations --------------- */
     
void UpperCase(char *s);
/*
    Convert s to upper case
*/

Boolean BlankString(char *s);
/*
   Return true if s is blank 
*/

/* ------------------ Hash table Operations ----------------- */

typedef struct _NameHolder{    /* Hash Table Linked List Item */
   char *name;                 /* Label Name */
   int aux;                    /* User flags etc*/
   void *ptr;                  /* General-purpose ptr */
   struct _NameHolder *wptr;   /* equivalence class */
   struct _NameHolder *next;   /* Chain */
}NameHolder;

typedef NameHolder *NameId;    /* Internal representation of names */

typedef struct {               /* hash tale structure */
  int hashSize;                /* hash size - prime number */
  char *description;           /* hash table description */
  long numAccesses;            /* access statistics */
  long numTests;               /* more access statistics */
  NameHolder **nameTab;        /* the actual table */
} HashTab;

HashTab *CreateHashTable(int hashSize, char *description);
/*
   Create and initialise a hash table 
*/

NameId GetNameId(HashTab *htab, char *name, Boolean insert);
/*
   Query/insert name into hash table
*/

void PrintHashTabStats(HashTab *htab);
/*
   Print out statistics on hash table usage
*/

/* -------------------- Sorting Routines  -------------------- */

void usort(void *base, int n, size_t size,
	   int (*compar)(const void *, const void *));

/* --------------------- End of LUtil.h ---------------------- */

#ifdef __cplusplus
}
#endif

#endif
