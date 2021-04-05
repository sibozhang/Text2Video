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
/*       File: LCMap:  Class Map and Associated Routines       */
/* ----------------------------------------------------------- */

/* !HVER!lcmap:   3.4.1 [CUED 12/03/09] */

#ifndef _LCMAP_H
#define _LCMAP_H

#ifdef __cplusplus
extern "C" {
#endif

#define CLMHASHSIZE 503

#define DEF_UNKNOWNNAME    "!!UNK"
#define DEF_UNKNOWNID      1


typedef struct clEntry { /* class map entry */
   LabId id;		   /* name of class id->aux points here */
   int ndx;                /* index of class */
   int size;               /* size of class */
   Boolean inClass;	   /* "in class" class */
   struct clEntry *next;   /* next class in list */
} ClassEntry;

/* 
  A class map is a linked list of class entries.  A word belongs
  to a class if its mapentry->class points to the class entry.
*/

typedef struct {   	       /* Class Map - hash table of ClassEntrys */
   MemHeap mem;    	     	/* memory heap for this class map */
   Boolean hdrless;	     	/* if true, nentries is size of unk class */
   Boolean htkEsc;              /* has HTK escaping */
   int entries;    	     	/* total classes (or unk entries) in map */
   char *name;     	     	/* name of class map */
   char *lang;     	     	/* language */
   int maxClndx;   	     	/* highest class index in map */
   WordMap *wmap;	     	/* associated word map */
   ClassEntry *htab[CLMHASHSIZE];  /* hash table of ClassEntrys */
   int nfree;		     	/* num records in free list */
   ClassEntry *flist;	        /* free list of ClassEntry records */
} ClassMap;

void InitCMap(void);
/* 
   Initialise the module.
*/

void CreateClassMap(char *fn, ClassMap *c, WordMap *w);
/*
   If fn is not NULL, load the class map stored in it.
   Otherwise create an empty class map.
*/

void SaveClassMap(char *fn, ClassMap *c);
/*
   Write given class map to file fn.  
*/

void ShowClassMap(ClassMap *c);
/*
   Print the contents of class map c
*/

ClassEntry *MakeNewClass(ClassMap *c, LabId id, int clndx, Boolean inClass);
/*
   Create a new class called name, add it to the class list.
   If clndx is -ve a new index is allocated otherwise clndx is used.
   A pointer to the class entry is returned.
*/

void DeleteClass(ClassMap *c, int clndx);
/*
   Delete the given class
*/

void AddWordToClass  (ClassMap *c, int clndx, int wdndx);
void RemWordFromClass(ClassMap *c, int clndx, int wdndx);
/*
   Add/delete given word from specified class
*/

int WordClass(ClassMap *c, int wdndx);
/*
   Returns class of given word, -1 if no class found
*/

int ClassSize(ClassMap *c, int clndx);
/*
   Returns size of given class, -1 if class not found
*/

void GetClassMembers(ClassMap *c, int clndx, int *words);
/*
   Copies ndx'es of all words in given class to the array
   words [0..Classsize(c,clndx)-1], storage for words must be
   provided by the caller.
*/

Boolean IsAnInClass(ClassMap *c, int clndx);
/*
   Returns true if given class is a regular 'inClass' set
   rather than a complement 'notinClass' set.
*/

Boolean IsClassMember(ClassMap *c, int clndx, int wdndx);
/*
   Returns true if given word is a member of the given class.
*/

/* ------------------- Low Level Access ---------------- */

ClassEntry *GetClassEntry(ClassMap *c, int clndx);
/* 
   Return pointer to ClassEntry for given class, NULL if 
   not found.  Note that aux field of class name id also
   points to this entry.
*/

ClassEntry *GetWordClassEntry(ClassMap *c, int wdndx);
/* 
   Return pointer to ClassEntry for given word, NULL if 
   not found.  
*/

/* -------------------- End of LCMap.h ---------------------- */

#ifdef __cplusplus
}
#endif

#endif
