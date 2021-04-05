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
/*         File: HLabel.h:   Speech Label File Input           */
/* ----------------------------------------------------------- */

/* !HVER!HLabel:   3.4.1 [CUED 12/03/09] */


/* 
   This module allows an internal data structure representing a
   transcription to be constructed by reading labels from an external
   file.  Each transcription consists of a list of label lists where
   each label list represents one distinct interpretation of the
   input.  Each time segment in a label list has a primary label.  It
   can also have one or more auxiliary labels attached to it.  In 
   this case, each additional label corresponds to a different
   level of representation.
   
   A number of single alternative/single level input formats are
   supported but multiple alternatives and multiple levels are limited
   to the HTK format.
   
   The module also provides the internal data structures needed to
   support master label files and the hash table for all string to
   LabId mapping.
*/

/*  Configuration Parameters:
   SOURCELABEL    - format of input label files
   TARGETLABEL    - format of output label files
   STRIPTRIPHONES - strip contexts from triphones
   TRANSALT       - filter alt lab list on read in
   TRANSLEV       - filter level on read in
   TRACE          - trace level
*/

#ifndef _HLABEL_H_
#define _HLABEL_H_

#ifdef __cplusplus
extern "C" {
#endif

typedef struct _NameCell{  /* Hash Table Linked List Item */
   char * name;             /* Label Name */
   Ptr aux;                 /* User pointer */
   struct _NameCell *next;  /* Chain */
}NameCell;

typedef NameCell *LabId;   /* Internal representation of names */
  
typedef struct _Label *LLink;
typedef struct _Label{     /* Information for each label */
   LabId labid;             /* primary label id */
   float score;             /* primary score eg. logP */
   LabId *auxLab;           /* array [1..maxAuxLab] OF LabId */
   float *auxScore;         /* array [1..maxAuxLab] OF float */
   HTime start,end;         /* Start and end times in 100ns units */
   LLink succ;              /* Successor label */
   LLink pred;              /* Preceding label */
}Label; /* NB: head and tail of every list are dummy sentinels */

typedef struct _LabList{
   LLink head;              /* Pointer to head of List */
   LLink tail;              /* Pointer to tail of List */
   struct _LabList *next;   /* Next label list */
   int maxAuxLab;           /* max aux labels (default=0) */
}LabList;

typedef struct {
   LabList *head;          /* Pointer to head of Label List */
   LabList *tail;          /* Pointer to tail of Label List */
   int numLists;           /* num label lists (default=1) */
}Transcription;

enum _MLFDefType {
   MLF_IMMEDIATE, MLF_SIMPLE, MLF_FULL
};
typedef enum _MLFDefType MLFDefType;

enum _MLFPatType {
   PAT_FIXED,     /* whole "pattern" is hashed */
   PAT_ANYPATH,   /* pat is "* / name" and name is hashed */
   PAT_GENERAL    /* general pattern - no hashing */
};
typedef enum _MLFPatType MLFPatType;

typedef struct{
   int fidx;      /* MLF file index in mlfile */
   long offset;   /* offset into MLF file */
}ImmDef;

typedef union{
   char *subdir;     /* Sub-directory to search for MLF_SIMPLE & MLF_FULL */
   ImmDef immed;     /* Immediate Definition for MLF_IMMEDIATE */
}MLFDef;

typedef struct _MLFEntry{
   char *pattern;       /* pattern to match for this definition */
   MLFPatType patType;  /* type of pattern */
   unsigned patHash;    /* hash of pattern if not general */
   MLFDefType type;     /* type of this definition */
   MLFDef def;          /* the actual def */
   struct _MLFEntry *next;    /* next in chain */
}MLFEntry;

/* ------------------- Label/Name Handling ------------------- */

void InitLabel(void);
/*
   Initialises hash table - must be called before using any of
   the routines in this module.
*/

LabId GetLabId(char *name, Boolean insert);
/*
   Lookup given name in hash table and return its id.  If it
   is not there and insert is true then insert the new name
   otherwise return NULL.
*/

void PrintNameTabStats(void);
/* 
   Print out statistics on hash table usage: string heap,
   name cell heap and average search length
*/

Boolean ReadLabel(FILE *f, char *buf);
/*
   Read the next label from f into buf.  A label is any
   sequence of printing chars.  Returns false if no label
   found. Skips white space and puts terminator back into
   f.
*/

/* ---------- Transcription and Label List Handling --------------- */

/* 
   MemHeap must be a STAK or a CHEAP.  For stack case, deallocation
   can be by Disposing back to pointer to transcription (normally the 
   first object allocated) or by reseting the heap
*/

Transcription *CreateTranscription(MemHeap *x);
/*
   Create a transcription with no label lists.
*/

Transcription *CopyTranscription(MemHeap *x, Transcription *t);
/*
   Return a copy of transcription t, allocated in x.
*/

void PrintTranscription(Transcription *t, char *title);
/*
   Print transcription t (for debugging/diagnostics)
*/

LabList* CreateLabelList(MemHeap *x, int maxAuxLab);
/* 
   Create and return a new label list with upto maxAuxLab 
   alternative labels.  This will have sentinel labels
   with NULL labid fields at the head and tail.
*/

void AddLabelList(LabList *ll, Transcription *t);
/* 
   Add given label list to transcription t. 
*/

LabList* GetLabelList(Transcription *t, int n);
/*
   Return pointer to n'th label list from transcription t indexed
   1 .. numLists
*/

LabList* CopyLabelList(MemHeap *x, LabList* ll);
/*
   Return a copy of given label list, allocated in x.
*/

LLink CreateLabel(MemHeap *x, int maxAux);
/*
   create a label with maxAux auxiliary slots 
*/

LLink AddLabel(MemHeap *x, LabList *ll, LabId id,
               HTime st, HTime en, float score);
/*
   Append a new item to end of given list and store the given info.
   Return a pointer to the newly created label item
*/

void AddAuxLab(LLink lab, int n, LabId *auxLab, float *auxScore);
/*
   Store n auxiliary label/score in lab
*/

void DeleteLabel(LLink item);
/*
   Unlink given item from a label list 
*/

int NumCases(LabList *ll, LabId id);
int NumAuxCases(LabList *ll, LabId id, int i);
/* 
   find number of cases of primary label/ i'th auxiliary
   label in given label list 
*/

LLink GetCase(LabList *ll, LabId id, int n);
LLink GetAuxCase(LabList *ll, LabId id, int n, int i);
/*
   return the nth occurrence of given primary label/ i'th auxiliary
   label in given label list 
*/

LLink GetLabN(LabList *ll, int n);
LLink GetAuxLabN(LabList *ll, int n, int i);
/* 
   return n'th primary label / i'th auxiliary label in given label list
*/

int CountLabs(LabList *ll);
int CountAuxLabs(LabList *ll, int i);
/*
   return number of primary labels / i'th auxiliary labels in
   given label list
*/

HTime AuxLabEndTime(LLink p, int i);
/* 
   return the end time for the i'th aux lab in label p.  This will be 
   the end time of the label before the next one containing an aux lab i
   or the end of the last label, whichever comes first
*/

/* ------------------ Label File Opening/Closing -------------------- */

Transcription *LOpen(MemHeap *x, char *fname, FileFormat fmt);
/*
   Reads the labels stored in file fname and build the internal
   representation in a transcription allocated in x.  If a Master
   Label File has been loaded, then that file is searched before
   looking for fname directly.  If fmt is UNDEFF then source format
   will be SOURCEFORMAT if set else source format will be HTK.
   If TRANSALT is set to N then all but the N'th alternative is
   discarded on read in.  If TRANSLEV is set to L then all but the 
   L'th level is discarded on read in.
*/

ReturnStatus SaveToMasterfile(char *fname);
/*
   Once called, all subsequent LSave's go to the MLF file fname
   rather than to individual files.  Format must be HTK.
*/

void CloseMLFSaveFile(void);
/*
   Close the MLF output file.  Any subsequent LSave's return to
   normal behaviour.
*/

ReturnStatus LSave(char *fname, Transcription *t, FileFormat fmt);
/* 
   Save the given transcription in file fname.  If fmt is UNDEFF then
   target format will be TARGETFORMAT if set else target format will
   be HTK.
*/

/* -------------------- TriPhone Stripping -------------------- */

void TriStrip(char *s);
/* 
   Remove contexts of form A- and +B from s 
*/

void LTriStrip(Boolean enab);
/*
   When enab is set, all triphone labels with the form A-B+C
   are converted to B on read in by LOpen.
*/

/* ------------------ Master Label File Handling -------------------- */

void LoadMasterFile(char *fname);
/*
   Load the Master Label File stored in fname
*/

int NumMLFFiles(void);
int NumMLFEntries(void);
/*
   Return the number of loaded MLF files and the total number of
   entries in the MLF table
*/

FILE *GetMLFFile(int fidx);
/*
   Return the fidx'th loaded MLF file.  Index base is 0.
*/

Boolean IsMLFFile(char *fn);
/* 
   Return true if fn is an MLF file
*/

MLFEntry *GetMLFTable(void);
/*
   Return the first entry in the MLF table
*/

#ifdef __cplusplus
}
#endif

#endif  /* _HLABEL_H_ */

/* ------------------------ End of HLabel.h ------------------------- */



